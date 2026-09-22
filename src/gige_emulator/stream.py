#
# The GVSP stream channel: one thread, one UDP socket on an ephemeral port.
#
# The socket is bound explicitly to the device's own IP because the client's
# receive path filters on source address -- letting the kernel pick a source
# by route works until the machine has a second interface.
#
# A frame goes out as one uninterrupted burst. The client starts asking for
# resends after 20 ms of silence within a frame and abandons the frame after
# 100 ms, so pacing between packets is not an option unless the client asked
# for it via the packet delay register.
#
# There is deliberately no frame rate timer here. The physical camera sets
# the rate, and next_frame() blocking until the sensor has something is what
# paces the loop. A timer on this side would either fight the camera's own
# timing or, worse, add to it -- a camera that blocks for a frame period and
# then waits another one runs at exactly half the rate it was asked for,
# which reads like a protocol fault rather than a scheduling one. A camera
# with no physical timing of its own is responsible for pacing itself; see
# the noise example.
#

import logging
import socket
import threading
import time

from . import constants as c
from . import gvsp
from . import netif
from .camera import Frame

log = logging.getLogger(__name__)

# Bytes the kernel may hold for the stream socket.
#
# Deliberately small. This buffer is a queue in front of the wire, and
# everything in it is video a client is already waiting for. It was 8 MB
# here, which at gigabit is 65 ms of frame sitting in the sender before it
# is sent -- and a resend queues behind all of it, arriving long after the
# client gave up on the frame it was meant to repair. Worse, a queue that
# deep makes the device its own source of loss: it overruns whatever the
# machine uses to schedule its transmissions.
#
# Measured on two benches (a Pi 5 and an x86 NUC, both streaming full frames
# at wire rate to VimbaX's GigE producer), 8 MB against 256 KB:
#
#   queue actually held    5.5 MB median, 8.2 MB peak   ->  0.36 MB, 0.51 MB
#   frames arriving whole  Pi 35/40, NUC 9/40           ->  Pi 40/40, NUC 397/400
#   packets resent         up to 10,700 per 15 s        ->  0 to 400
#
# The cost is about 3% of the frame rate, from the wire going briefly idle
# while Python refills a shallower buffer. Aravis hides the old behaviour
# almost entirely -- it repairs everything and reports no failures, at the
# price of that resend traffic -- so a client that completes every frame is
# not on its own evidence that the device is behaving.
#
# The kernel doubles this for its own bookkeeping and caps it at
# net.core.wmem_max, so what a host grants is not what is asked for here.
SEND_BUFFER_SIZE = 256 * 1024

# How long stop() waits for the stream thread. Not sized to cover an exposure
# on purpose -- see stop().
SHUTDOWN_JOIN_TIMEOUT = 3.0

# Linux socket options for the don't-fragment bit, from <linux/in.h>. Python's
# socket module does not export these on every build, so they are spelled out
# and applied defensively rather than imported.
IP_MTU_DISCOVER = 10
IP_PMTUDISC_DONT = 0
IP_PMTUDISC_DO = 2


class StreamChannel(object):

    #: How many recent frames stay answerable for a resend.
    #:
    #: There is no "frame complete" message in GigE Vision -- the whole
    #: command set is discovery, bye, resend, and register and memory access.
    #: A client speaks only when something is *missing*, so silence means
    #: either that it has everything or that its request is still in flight,
    #: and the device cannot tell which.
    #:
    #: Two, so that sending frame N+1 does not end N's availability. Holding
    #: only the current frame forces the sender to sit idle after each one
    #: instead, waiting to see whether a request arrives, and that wait is
    #: not small: measured at full resolution, a 5 ms wait left 510 requests
    #: arriving too late while 250 ms left none -- and 250 ms of dead time
    #: per frame halves the frame rate. That was measured when SEND_BUFFER_SIZE
    #: was 8 MB, and the lateness blamed on the client's backlog was mostly
    #: this device's own send queue, so the numbers overstate it. The overlap
    #: is kept because it costs one buffer and no time at all: N stays
    #: answerable for exactly as long as N+1 takes to send.
    RETAIN_FRAMES = 2

    #: Extra dwell after a frame before fetching the next, on top of the
    #: overlap above. Zero by default -- the overlap is what provides the
    #: window now. Raise it only for a client that asks later than a whole
    #: frame period, which is a thing to measure before assuming.
    RESEND_GUARD = 0.0

    #: Share of the link the stream is allowed to occupy.
    #:
    #: Not throttling for its own sake -- the remainder is the quiet the
    #: repairs go out in, and what it really buys is *time*: a client closes
    #: a frame shortly after the next one starts arriving, so a resent
    #: packet is only useful if it lands before then.
    #:
    #: Measured 2026-09-22 against VimbaX, full frames from a 12 MPix camera
    #: over a link that loses a little, 400 frames per setting: 0.85 left
    #: 397 whole, 0.95 left 386 (Fisher p=0.012). Both lost and repaired the
    #: same amount -- the difference is only whether the repair arrived in
    #: time, 18 ms of quiet after a 105 ms frame against 5 ms. 0.95 is 6 to
    #: 12% faster, which is not worth four times the lost frames. On a link
    #: that loses nothing the reservation buys nothing either, so this costs
    #: throughput there and protects nothing; it is the lossy link it is for.
    #:
    #: Applied against the time the frame itself took, so it needs no idea
    #: of the link's speed and follows it if it changes: a frame that took
    #: 0.2 s to put on the wire is followed by 0.2 * (1/0.85 - 1) = 35 ms in
    #: which the sender is quiet and the control thread can answer.
    LINK_UTILISATION = 0.85

    def __init__(self, camera, memory, lock, device_ip,
                 control=None, idle_poll=0.02, interface=None,
                 resend_guard=None, retain_frames=None,
                 link_utilisation=None, allow_any_destination=False):
        self.camera = camera
        self.memory = memory
        self.lock = lock
        self.device_ip = device_ip
        self.control = control
        self.idle_poll = idle_poll
        self.interface = interface

        self.socket = None
        self.thread = None
        self.running = False

        self.resend_guard = (self.RESEND_GUARD if resend_guard is None
                             else resend_guard)
        self.retain_frames = max(1, self.RETAIN_FRAMES if retain_frames is None
                                 else retain_frames)
        self.link_utilisation = min(1.0, max(0.05,
                                    self.LINK_UTILISATION
                                    if link_utilisation is None
                                    else link_utilisation))

        #: Send the stream anywhere a client asks, rather than only back to
        #: the client that asked. Off by default: see _destination_allowed().
        self.allow_any_destination = allow_any_destination

        #: (control ip, stream ip) the last message fired for, so a client is
        #: told once rather than once per frame.
        self._warned_destination = None

        self.frame_id = 0
        self.n_frames = 0
        self.n_packets = 0
        self.n_send_errors = 0
        self.n_test_packets = 0
        self.n_resend_requests = 0
        self.n_resent_packets = 0
        self.n_resend_unavailable = 0
        self.n_resend_refused = 0

        # Recent frames a resend can still be served from, oldest first, each
        # (frame_id, data, geometry, packet_size, timestamp_ns).
        #
        # Bounded and short. The cost is real -- two full resolution frames
        # is 49 MB -- so it is not a queue that grows, and it is emptied
        # whenever the stream goes quiet rather than lingering while a camera
        # sits idle.
        self._retained = []
        self._last_resend = 0.0

    # --- lifecycle -------------------------------------------------------

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF,
                               SEND_BUFFER_SIZE)

        # Binding to device_ip already fixes the source address, but routing
        # still picks the egress interface. Pinning the device too means a
        # client reachable only some other way fails loudly with
        # ENETUNREACH -- which the send loop logs -- rather than quietly
        # emitting frames from the wrong NIC with a source the client's
        # filter may not match.
        if self.interface is not None:
            if not netif.bind_to_device(self.socket, self.interface):
                log.warning("cannot restrict the stream socket to %s",
                            self.interface)

        self.socket.bind((self.device_ip, 0))
        with self.lock:
            self.memory.poke_register(c.BS_SC0_SOURCE_PORT,
                                      self.socket.getsockname()[1])
        self.running = True
        self.thread = threading.Thread(target=self._run, name="gvsp-stream",
                                       daemon=True)
        self.thread.start()

    def stop(self):
        """
        Stop streaming. Returns once the thread is gone, or after
        SHUTDOWN_JOIN_TIMEOUT if it is still inside next_frame().

        The join is deliberately not long enough to cover any exposure. A
        camera can be asked for a ten second one, and blocking a caller that
        long to shut down is worse than letting the thread finish on its own
        -- so the thread is built to come back to a closed socket and do
        nothing, rather than the socket being kept alive to wait for it.
        """
        self.running = False
        thread = self.thread
        if thread is not None:
            thread.join(timeout=SHUTDOWN_JOIN_TIMEOUT)
            if thread.is_alive():
                # Keep the handle. Clearing it here would lose the only
                # reference to a live thread, and a second stop() would then
                # believe it had nothing to wait for.
                log.warning("stream thread did not stop within %.1f s and is "
                            "probably still inside next_frame(); it will exit "
                            "when that returns",
                            SHUTDOWN_JOIN_TIMEOUT)
            else:
                self.thread = None
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        # After the join, so a thread still inside next_frame() cannot retain
        # a frame on its way out and leave it held for the process's life.
        self._release_retained()

    # --- the frame loop --------------------------------------------------

    def _stream_target(self):
        """
        Where to send, from the registers the client wrote.

        Both of these live in the LOW 16 bits despite the transport layer XML
        describing them as bits 31..16 -- GenICam counts mask bits from the
        MSB for a big endian register. Shifting right by 16 here yields port
        zero and a camera that holds control but never streams.
        """
        address = self.memory.peek_register(c.BS_SC0_IP_ADDRESS)
        port = self.memory.peek_register(c.BS_SC0_PORT) & c.SC_PORT_MASK
        if address == 0 or port == 0:
            return None
        ip = "%d.%d.%d.%d" % ((address >> 24) & 0xFF, (address >> 16) & 0xFF,
                              (address >> 8) & 0xFF, address & 0xFF)
        return (ip, port)

    def set_dont_fragment(self, on):
        """
        Ask the kernel to set (or clear) DF on this socket.

        Not fatal if it fails: without it the emulator still streams, it just
        cannot fail a client's oversized probe, so the client may settle on a
        packet size the path has to fragment.
        """
        sock = self.socket
        if sock is None:
            return False
        mode = IP_PMTUDISC_DO if on else IP_PMTUDISC_DONT
        try:
            sock.setsockopt(socket.IPPROTO_IP, IP_MTU_DISCOVER, mode)
            return True
        except OSError as e:
            log.debug("cannot set the don't fragment bit: %s", e)
            return False

    def send_test_packet(self, packet_size, do_not_fragment):
        """
        Answer a client's packet size probe with one packet of that exact size.

        Called from the control thread, because the write that asks for it
        arrives on the control channel. That is safe here only because a
        client sizes its packets before it starts acquisition, so the stream
        loop is parked in its idle poll and not touching the socket.

        Returning False is a legitimate outcome, not just an error path: when
        the client asks for more than the path carries and DF is set, the send
        fails with EMSGSIZE and *that silence is the answer*. The client times
        out, steps down a size, and tries again.
        """
        # Snapshot rather than re-read: stop() can null this between the
        # check and the send.
        sock = self.socket
        if sock is None:
            return False
        with self.lock:
            target = self._stream_target()
        if target is None:
            log.warning("packet size probe arrived before the stream "
                        "destination was set; nothing to answer it with")
            return False

        datagram = gvsp.test_packet(packet_size)
        if datagram is None:
            log.warning("packet size probe of %d bytes is too small to hold "
                        "a GVSP header", packet_size)
            return False

        self.set_dont_fragment(do_not_fragment)
        try:
            sock.sendto(datagram, target)
        except OSError as e:
            log.debug("packet size probe of %d bytes not sent: %s",
                      packet_size, e)
            return False
        self.n_test_packets += 1
        return True

    # --- packet resend ---------------------------------------------------

    #: Largest share of a frame worth resending, before refusing the request
    #: outright.
    #:
    #: A client this far behind is not one packet short, it has lost a run of
    #: hundreds -- and repairing that costs bandwidth the next frame needs.
    #: At full link utilisation there is none spare, so the repair starves
    #: the following frame, which then needs repairing too. That is a
    #: collapse, not a hiccup: once it starts the stream never recovers.
    #:
    #: Refusing lets the client drop the frame and start clean on the next
    #: one. Aravis asks for up to 25% of a frame before giving up on its own
    #: (ARV_GV_STREAM_PACKET_REQUEST_RATIO_DEFAULT), so this threshold is
    #: what decides the outcome first.
    MAX_RESEND_FRACTION = 0.10

    #: Floor, so a small frame is not refused for asking about two packets.
    MIN_RESEND_PACKETS = 64

    def resend(self, frame_id, first_id, last_id):
        """
        Re-send a run of packets, or say they are gone.

        Runs on the control thread, because the request arrives on the
        control channel and the client is waiting on the answer now -- it
        gives up on the frame 100 ms after the first packet. Handing this to
        the stream thread would put it behind a whole frame's send.
        """
        self.n_resend_requests += 1
        sock = self.socket
        if sock is None:
            return False

        with self.lock:
            retained = next((f for f in self._retained if f[0] == frame_id),
                            None)
            target = self._stream_target()
        if target is None:
            return False

        if retained is None:
            # Answer rather than ignore: silence costs the client its whole
            # retention timeout before it gives up on a frame we cannot
            # complete anyway.
            self.n_resend_unavailable += 1
            try:
                sock.sendto(gvsp.unavailable_packet(frame_id, first_id), target)
            except OSError:
                pass
            return False

        _, data, geometry, packet_size, timestamp_ns = retained

        if last_id < first_id:
            return False

        # Serve all of it or none of it. Truncating was worse than refusing:
        # the frame then cannot complete whatever else arrives, so every
        # packet sent for it is wasted -- and wasted at exactly the moment
        # the link is most oversubscribed. The client waits out its retention
        # timeout for packets that were never coming.
        count = last_id - first_id + 1
        limit = max(self.MIN_RESEND_PACKETS,
                    int(gvsp.packet_count(len(data), packet_size)
                        * self.MAX_RESEND_FRACTION))
        if count > limit:
            self.n_resend_refused += 1
            log.warning("refusing a resend of %d packets for frame %d (over "
                        "%d); the client is too far behind for repair to be "
                        "cheaper than the next frame", count, frame_id, limit)
            try:
                sock.sendto(gvsp.unavailable_packet(frame_id, first_id), target)
            except OSError:
                pass
            return False

        sent = 0
        for packet_id in range(first_id, last_id + 1):
            datagram = gvsp.packet_by_id(data, packet_size, frame_id,
                                         geometry, timestamp_ns, packet_id)
            if datagram is None:
                continue
            try:
                sock.sendto(datagram, target)
            except OSError as e:
                self.n_send_errors += 1
                log.warning("resend failed: %s", e)
                break
            sent += 1

        self.n_resent_packets += sent
        # Restart the guard, so a frame that keeps losing packets keeps being
        # repaired instead of being released on a fixed deadline.
        self._last_resend = time.monotonic()
        return sent > 0

    def _pace(self, send_seconds):
        """
        Stay quiet for long enough to leave the link its reserved share.

        The frame's own send time is the measurement: with a blocking socket
        and a send buffer small enough not to hide the wire behind it -- see
        SEND_BUFFER_SIZE -- it is how long the wire took, so the pause needs
        no configured link speed and tracks one that changes. Enlarge that
        buffer and this measures how fast the kernel accepted the frame
        instead, and the pause is spent draining what is still queued. This is the window resends are
        answered in -- the control thread runs throughout, and the stream is
        not competing with it for the wire.
        """
        if send_seconds <= 0 or self.link_utilisation >= 1.0:
            return
        idle = send_seconds * (1.0 / self.link_utilisation - 1.0)
        deadline = time.perf_counter() + idle
        while self.running:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            time.sleep(min(0.002, remaining))

    def _release_retained(self):
        """
        Drop the frame kept for resends.

        Today this is belt and braces: _await_resends() already releases
        after every frame, so an idle stream holds nothing. It is here
        because that is a property of the current shape rather than a
        guarantee -- anything that overlaps frames, keeping N answerable
        while N+1 goes out, releases on a rule ("when N+2 starts") that a
        stopped acquisition never satisfies. The buffers would then sit
        pinned for as long as the camera idled, which at full resolution is
        49 MB doing nothing.
        """
        with self.lock:
            self._retained = []
            self._last_resend = 0.0

    def _await_resends(self):
        """
        Optional extra dwell after a frame, for a client that asks later than
        the overlap covers.

        Off by default, and it does not release anything -- retention is
        bounded by frame count now, not by this clock. Note what it costs
        when switched on: the sender is idle throughout, so the frame rate
        becomes 1 / (send + guard). At full resolution a 0.25 s guard halves
        it, which is why the overlap replaced it rather than joining it.
        """
        if self.resend_guard <= 0:
            return
        deadline = time.monotonic() + self.resend_guard
        while self.running:
            now = time.monotonic()
            if self._last_resend > 0.0:
                deadline = max(deadline, self._last_resend + self.resend_guard)
                self._last_resend = 0.0
            if now >= deadline:
                break
            time.sleep(min(0.001, deadline - now))

    def _snapshot(self):
        """
        Take everything the burst needs under the lock, then let go of it.
        The lock must not be held during the send loop or next_frame().
        """
        with self.lock:
            if not self.camera.acquiring:
                return None
            if self.control is not None and not self.control.has_control():
                return None
            target = self._stream_target()
            if target is None:
                return None
            if not self._destination_allowed(target):
                return None
            packet_size = (self.memory.peek_register(c.BS_SC0_PACKET_SIZE)
                           & c.SC_PACKET_SIZE_MASK)
            packet_delay = self.memory.peek_register(c.BS_SC0_PACKET_DELAY)
            geometry = self.camera.geometry
            if geometry is None:
                return None
            mode = self.camera.settings.get("AcquisitionMode", "Continuous")
            return target, packet_size, packet_delay, geometry, mode

    def _destination_allowed(self, target):
        """
        Whether to send the images where the client asked.

        Only back to the client that asked, unless the caller opted out with
        allow_any_destination. Handing a stream to a third machine is a real
        use, but the default has to be the safe one: GVCP carries no
        authentication, the client's address is a UDP source that anybody can
        forge, and a few small packets naming someone else would otherwise
        turn this device into an amplifier pointed at them -- with no reply
        ever going back to whoever sent them.

        Perfectly legal, and sometimes deliberate -- a client can hand the
        stream to another machine. But on a host with two interfaces on one
        subnet it is usually an accident, and an expensive one: the commands
        go over one interface and the images over the other, so a camera on a
        wire ends up delivering its frames over WiFi. That presents as
        latency, or as frames that arrive damaged, and nothing else in the
        system says why. Both addresses are printed because which is which is
        the whole question.
        """
        if self.control is None:
            return True
        controller = self.control.controller
        if controller is None or controller[0] == target[0]:
            return True

        pair = (controller[0], target[0])
        said = self._warned_destination == pair
        self._warned_destination = pair
        if self.allow_any_destination:
            if not said:
                log.warning("the client controlling this camera is at %s but "
                            "asked for the stream at %s; sending it anyway "
                            "because allow_any_destination is set. If those "
                            "are two interfaces on one machine, the images "
                            "are taking a different path from the commands",
                            *pair)
            return True
        if not said:
            log.warning("refusing to stream to %s: the client controlling "
                        "this camera is at %s, and a device that sends where "
                        "it is told is an amplifier for anyone who can forge "
                        "a source address. Pass allow_any_destination=True if "
                        "this is deliberate", target[0], controller[0])
        return False

    def _run(self):
        while self.running:
            state = self._snapshot()
            if state is None:
                # Covers every way the stream goes quiet -- acquisition
                # stopped, control lost, no destination yet -- rather than
                # AcquisitionStop alone, because all of them leave a frame
                # nobody will ever ask about again.
                self._release_retained()
                time.sleep(self.idle_poll)
                continue

            target, packet_size, packet_delay, geometry, mode = state

            if packet_size < 64 or packet_size > 65536:
                log.warning("client asked for an unusable packet size %d",
                            packet_size)
                time.sleep(self.idle_poll)
                continue

            try:
                frame = self.camera.next_frame()
            except Exception:
                log.exception("next_frame() raised")
                time.sleep(self.idle_poll)
                continue

            if frame is None:
                time.sleep(0.001)
                continue

            # next_frame() blocks for as long as an exposure takes, so the
            # client may have stopped acquisition while we were inside it.
            # Sending anyway would deliver a frame after AcquisitionStop,
            # which a client is entitled to treat as a protocol error.
            with self.lock:
                still_wanted = self.camera.acquiring
            if not still_wanted:
                continue

            sent_at = time.perf_counter()
            self._send_frame(frame, target, packet_size, packet_delay, geometry)
            self._pace(time.perf_counter() - sent_at)
            self._await_resends()

            # One frame per AcquisitionStart, and the send is over either
            # way. Stopping only on a successful send would turn a camera
            # that keeps returning the wrong number of bytes into a
            # continuous stream of dropped frames, which is the one outcome
            # the client cannot distinguish from the device ignoring the
            # mode entirely. It asked for one frame; it gets one attempt,
            # and a timeout if that attempt failed.
            if mode == "SingleFrame":
                with self.lock:
                    self.camera.acquiring = False
                log.info("single frame delivered, acquisition stopped")

    def _send_frame(self, frame, target, packet_size, packet_delay, geometry):
        if isinstance(frame, Frame):
            data = frame.data
            timestamp_ns = frame.timestamp_ns
            explicit_id = frame.frame_id
        else:
            data = frame
            timestamp_ns = None
            explicit_id = None

        expected = geometry["payload"]
        if len(data) != expected:
            # Sending a short frame produces a trailer the client did not
            # expect and a buffer it reports as incomplete, so refusing is
            # both safer and far easier to diagnose.
            log.error("next_frame() returned %d bytes, expected %d for "
                      "%dx%d -- frame dropped",
                      len(data), expected, geometry["width"], geometry["height"])
            return

        if timestamp_ns is None:
            timestamp_ns = time.time_ns()

        if explicit_id is not None:
            frame_id = explicit_id % 65535 + 1
        else:
            frame_id = gvsp.next_frame_id(self.frame_id)
        self.frame_id = frame_id

        # Snapshot the socket. stop() closes it and drops the reference
        # without waiting for an exposure to finish, so by the time a slow
        # next_frame() returns there may be nothing left to send on -- the
        # same "stopped while we were inside next_frame()" case the
        # acquiring check above covers, for shutdown rather than
        # AcquisitionStop.
        sock = self.socket
        if sock is None or not self.running:
            return

        # Retained before the first packet goes out, not after the last: the
        # client arms a missing packet after 1 ms and asks while the burst is
        # still in flight, so a frame that only became resendable once it had
        # finished sending would miss most of the requests for it.
        #
        # Trimming here rather than on a timer is what makes the window
        # self-scaling: a frame stays answerable for as long as the frames
        # after it take to send, which is the same clock the client's own
        # backlog runs on. Nothing to tune, and no constant that is right at
        # one resolution and wrong at another.
        with self.lock:
            self._retained.append((frame_id, data, geometry, packet_size,
                                   timestamp_ns))
            del self._retained[:-self.retain_frames]
            self._last_resend = 0.0

        ceiling = gvsp.max_datagram_size(packet_size)
        send = sock.sendto
        delay_s = packet_delay / 1e9 if packet_delay else 0.0

        count = 0
        for datagram in gvsp.packetize(data, packet_size, frame_id, geometry,
                                       timestamp_ns):
            if len(datagram) > ceiling:
                log.error("datagram of %d bytes exceeds the client's %d byte "
                          "receive buffer; frame abandoned",
                          len(datagram), ceiling)
                return
            try:
                send(datagram, target)
            except OSError as e:
                self.n_send_errors += 1
                log.warning("GVSP send failed: %s", e)
                return
            count += 1
            if delay_s:
                self._spin(delay_s)

        self.n_packets += count
        self.n_frames += 1

    @staticmethod
    def _spin(seconds):
        """
        time.sleep() cannot resolve the sub-microsecond delays the packet
        delay register asks for, so busy wait instead.
        """
        deadline = time.perf_counter() + seconds
        while time.perf_counter() < deadline:
            pass
