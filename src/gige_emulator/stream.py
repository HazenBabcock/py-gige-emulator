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

import logging
import socket
import threading
import time

from . import constants as c
from . import gvsp
from .camera import Frame

log = logging.getLogger(__name__)

SEND_BUFFER_SIZE = 8 * 1024 * 1024


class StreamChannel(object):

    def __init__(self, camera, memory, lock, device_ip,
                 control=None, idle_poll=0.02):
        self.camera = camera
        self.memory = memory
        self.lock = lock
        self.device_ip = device_ip
        self.control = control
        self.idle_poll = idle_poll

        self.socket = None
        self.thread = None
        self.running = False

        self.frame_id = 0
        self.n_frames = 0
        self.n_packets = 0
        self.n_send_errors = 0

    # --- lifecycle -------------------------------------------------------

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF,
                               SEND_BUFFER_SIZE)
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

            self._send_frame(frame, target, packet_size, packet_delay, geometry)

            self._pace(geometry)

    def _pace(self, geometry):
        rate = self.camera.settings.get("AcquisitionFrameRate", 0.0)
        if rate and rate > 0:
            time.sleep(max(0.0, 1.0 / rate - 0.001))

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
