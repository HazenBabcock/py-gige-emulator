#
# A minimal GigE Vision client, enough to drive the emulator end to end over
# loopback.
#
# It exists so the whole handshake can be exercised in CI with no Aravis, no
# second machine and no network. Where it reassembles a frame it deliberately
# uses the rule the real client uses -- offset derived from the packet id --
# rather than trusting any offset the device might send.
#

import socket
import struct
import time

from gige_emulator import constants as c
from gige_emulator import gvcp


class FakeClientError(Exception):
    pass


class FakeClient(object):

    def __init__(self, device_address, timeout=2.0):
        self.device_address = device_address
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(timeout)
        self.packet_id = 65300          # start near the wrap, as Aravis does
        self.stream_socket = None

    def close(self):
        self.socket.close()
        if self.stream_socket is not None:
            self.stream_socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # --- command plumbing ------------------------------------------------

    def _next_packet_id(self):
        self.packet_id = 1 if self.packet_id == 0xFFFF else self.packet_id + 1
        return self.packet_id

    def _command(self, command, payload, expect_ack, packet_id=None,
                 retries=3):
        if packet_id is None:
            packet_id = self._next_packet_id()
        data = gvcp.HEADER.pack(c.PACKET_TYPE_CMD, c.CMD_FLAGS_ACK_REQUIRED,
                                command, len(payload), packet_id) + payload
        for _ in range(retries):
            self.socket.sendto(data, self.device_address)
            try:
                reply, _ = self.socket.recvfrom(4096)
            except socket.timeout:
                continue
            packet_type, flags, ack_command, size, ack_id = \
                gvcp.HEADER.unpack_from(reply)
            if packet_type == c.PACKET_TYPE_ERROR:
                raise FakeClientError("device returned error 0x%02x for "
                                      "command 0x%04x" % (flags, command))
            if ack_command == expect_ack and ack_id == packet_id:
                return reply[8:]
        raise FakeClientError("no answer to command 0x%04x" % command)

    # --- the commands ----------------------------------------------------

    def discover(self):
        payload = self._command(c.CMD_DISCOVERY, b"", c.ACK_DISCOVERY,
                                packet_id=0xFFFF)
        if len(payload) != c.DISCOVERY_DATA_SIZE:
            raise FakeClientError("discovery payload is %d bytes"
                                  % len(payload))
        return {
            "ip": socket.inet_ntoa(payload[0x24:0x28]),
            "mac": payload[0x0A:0x10],
            "manufacturer": _string(payload, 0x48, 32),
            "model": _string(payload, 0x68, 32),
            "serial": _string(payload, 0xD8, 16),
            # Not part of the device id, which is why a client lists this
            # camera as vendor-model-serial no matter what it is set to.
            "user_defined_name": _string(payload, 0xE8, 16),
        }

    def read_register(self, address):
        payload = self._command(c.CMD_READ_REGISTER,
                                struct.pack(">I", address),
                                c.ACK_READ_REGISTER)
        return struct.unpack_from(">I", payload, 0)[0]

    def write_register(self, address, value):
        self._command(c.CMD_WRITE_REGISTER, struct.pack(">II", address, value),
                      c.ACK_WRITE_REGISTER)

    def read_memory(self, address, size):
        out = bytearray()
        while len(out) < size:
            # Chunked the way the real client does, and rounded up to a
            # multiple of four, which is what makes XML padding necessary.
            want = min(c.GVCP_DATA_SIZE_MAX, size - len(out))
            rounded = (want + 3) // 4 * 4
            payload = self._command(
                c.CMD_READ_MEMORY,
                struct.pack(">II", address + len(out), rounded),
                c.ACK_READ_MEMORY)
            echoed = struct.unpack_from(">I", payload, 0)[0]
            if echoed != address + len(out):
                raise FakeClientError("read memory ack echoed 0x%x, wanted 0x%x"
                                      % (echoed, address + len(out)))
            out += payload[4:4 + want]
        return bytes(out[:size])

    def write_memory(self, address, data):
        self._command(c.CMD_WRITE_MEMORY, struct.pack(">I", address) + data,
                      c.ACK_WRITE_MEMORY)

    # --- higher level ----------------------------------------------------

    def fetch_genicam_xml(self):
        url = self.read_memory(c.BS_XML_URL_0, c.BS_XML_URL_SIZE)
        url = url.split(b"\x00", 1)[0].decode()
        if not url.lower().startswith("local:"):
            raise FakeClientError("unsupported XML url %r" % url)
        path, address, size = url.rsplit(";", 2)
        return url, self.read_memory(int(address, 16), int(size, 16))

    def take_control(self):
        self.write_register(c.BS_CONTROL_CHANNEL_PRIVILEGE, c.CCP_CONTROL)

    def release_control(self):
        self.write_register(c.BS_CONTROL_CHANNEL_PRIVILEGE, 0)

    def heartbeat(self):
        return self.read_register(c.BS_CONTROL_CHANNEL_PRIVILEGE)

    def open_stream(self, packet_size=1400):
        self.stream_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stream_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                                      8 * 1024 * 1024)
        self.stream_socket.bind(("127.0.0.1", 0))
        self.stream_socket.settimeout(2.0)
        host_ip, host_port = self.stream_socket.getsockname()

        self.write_register(c.BS_SC0_IP_ADDRESS,
                            struct.unpack("!I", socket.inet_aton(host_ip))[0])
        # Low 16 bits. Writing this to the high half is the classic mistake
        # and produces a device that holds control but never streams.
        self.write_register(c.BS_SC0_PORT, host_port)
        self.write_register(c.BS_SC0_PACKET_SIZE, packet_size)
        return packet_size

    def receive_frame(self, packet_size, timeout=5.0):
        """
        Reassemble one complete frame, using the client's own offset rule.
        """
        per_packet = packet_size - c.GVSP_PROTOCOL_OVERHEAD
        deadline = time.monotonic() + timeout
        frames = {}

        while time.monotonic() < deadline:
            try:
                data, _ = self.stream_socket.recvfrom(packet_size)
            except socket.timeout:
                continue
            status, block_id, infos = struct.unpack_from(">HHI", data, 0)
            if status != 0:
                continue
            content = (infos >> 24) & 0x7F
            packet_id = infos & c.GVSP_PACKET_ID_MASK
            frame = frames.setdefault(block_id, {"leader": None,
                                                 "trailer": None,
                                                 "chunks": {}})
            if content == c.GVSP_CONTENT_LEADER:
                if packet_id != 0:
                    raise FakeClientError("leader has packet id %d, must be 0"
                                          % packet_id)
                fields = struct.unpack_from(">HHIIIIIIIHH", data, 8)
                frame["leader"] = {
                    "payload_type": fields[1],
                    "timestamp_ns": (fields[2] << 32) | fields[3],
                    "pixel_format": fields[4],
                    "width": fields[5],
                    "height": fields[6],
                }
            elif content == c.GVSP_CONTENT_TRAILER:
                frame["trailer"] = {"packet_id": packet_id,
                                    "height": struct.unpack_from(">I", data, 12)[0]}
            else:
                frame["chunks"][packet_id] = data[8:]

            if frame["leader"] is None or frame["trailer"] is None:
                continue
            expected = frame["trailer"]["packet_id"] - 1
            if len(frame["chunks"]) != expected:
                continue

            out = bytearray()
            for pid in range(1, expected + 1):
                if (pid - 1) * per_packet != len(out):
                    raise FakeClientError(
                        "packet %d would land at %d, expected %d"
                        % (pid, (pid - 1) * per_packet, len(out)))
                out += frame["chunks"][pid]
            return block_id, frame["leader"], bytes(out)

        raise FakeClientError("no complete frame within %.1f s" % timeout)


def _string(payload, offset, size):
    return payload[offset:offset + size].split(b"\x00", 1)[0].decode(
        "utf-8", "replace")
