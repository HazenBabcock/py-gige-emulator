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

SEND_BUFFER_SIZE = 8 * 1024 * 1024

# Linux socket options for the don't-fragment bit, from <linux/in.h>. Python's
# socket module does not export these on every build, so they are spelled out
# and applied defensively rather than imported.
IP_MTU_DISCOVER = 10
IP_PMTUDISC_DONT = 0
IP_PMTUDISC_DO = 2


class StreamChannel(object):

    def __init__(self, camera, memory, lock, device_ip,
                 control=None, idle_poll=0.02, interface=None):
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

        self.frame_id = 0
        self.n_frames = 0
        self.n_packets = 0
        self.n_send_errors = 0
        self.n_test_packets = 0

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
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            self.thread = None
        if self.socket is not None:
            self.socket.close()
            self.socket = None

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
        if self.socket is None:
            return False
        mode = IP_PMTUDISC_DO if on else IP_PMTUDISC_DONT
        try:
            self.socket.setsockopt(socket.IPPROTO_IP, IP_MTU_DISCOVER, mode)
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
        if self.socket is None:
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
            self.socket.sendto(datagram, target)
        except OSError as e:
            log.debug("packet size probe of %d bytes not sent: %s",
                      packet_size, e)
            return False
        self.n_test_packets += 1
        return True

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
            packet_size = (self.memory.peek_register(c.BS_SC0_PACKET_SIZE)
                           & c.SC_PACKET_SIZE_MASK)
            packet_delay = self.memory.peek_register(c.BS_SC0_PACKET_DELAY)
            geometry = self.camera.geometry
            if geometry is None:
                return None
            return target, packet_size, packet_delay, geometry

    def _run(self):
        while self.running:
            state = self._snapshot()
            if state is None:
                time.sleep(self.idle_poll)
                continue

            target, packet_size, packet_delay, geometry = state

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

            self._send_frame(frame, target, packet_size, packet_delay, geometry)

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

        ceiling = gvsp.max_datagram_size(packet_size)
        send = self.socket.sendto
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
