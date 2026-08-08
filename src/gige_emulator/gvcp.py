#
# GVCP packet codec. Pure functions over bytes -- no sockets here.
#
# Every header is 8 bytes big endian:
#
#     u8 packet_type, u8 packet_flags, u16 command, u16 size, u16 id
#
# `size` counts the payload only, so a datagram is always 8 + size bytes.
#

import struct
from collections import namedtuple

from . import constants as c

HEADER = struct.Struct(">BBHHH")
HEADER_SIZE = HEADER.size

Command = namedtuple("Command", "packet_type flags command packet_id payload")


class MalformedPacket(Exception):
    pass


def parse_command(data):
    """
    Parse a datagram from the client. Returns None if it is not a command
    packet, which is the correct response to stray traffic on port 3956.
    """
    if len(data) < HEADER_SIZE:
        return None
    packet_type, flags, command, size, packet_id = HEADER.unpack_from(data, 0)
    if packet_type != c.PACKET_TYPE_CMD:
        return None
    payload = data[HEADER_SIZE:HEADER_SIZE + size]
    if len(payload) < size:
        raise MalformedPacket("truncated payload: want %d, got %d"
                              % (size, len(payload)))
    return Command(packet_type, flags, command, packet_id, payload)


def _ack(command, packet_id, payload=b""):
    return HEADER.pack(c.PACKET_TYPE_ACK, 0, command, len(payload),
                       packet_id) + payload


def encode_discovery_ack(packet_id, bootstrap_page):
    """
    bootstrap_page is memory [0, 0xf8), copied verbatim. The client reads the
    device's real IP out of offset 0x24 of this payload and connects there --
    it does not use the UDP source address of this reply.
    """
    if len(bootstrap_page) != c.DISCOVERY_DATA_SIZE:
        raise ValueError("discovery payload must be %d bytes"
                         % c.DISCOVERY_DATA_SIZE)
    return _ack(c.ACK_DISCOVERY, packet_id, bootstrap_page)


def encode_read_register_ack(packet_id, values):
    payload = b"".join(struct.pack(">I", v & 0xFFFFFFFF) for v in values)
    return _ack(c.ACK_READ_REGISTER, packet_id, payload)


def encode_write_register_ack(packet_id, n_written=1):
    return _ack(c.ACK_WRITE_REGISTER, packet_id, struct.pack(">I", n_written))


def encode_read_memory_ack(packet_id, address, data):
    """
    The address is echoed ahead of the data. Omitting it shifts every
    subsequent byte by four, which corrupts the XML download in a way that
    looks like a parse error rather than a protocol error.
    """
    return _ack(c.ACK_READ_MEMORY, packet_id,
                struct.pack(">I", address) + bytes(data))


def encode_write_memory_ack(packet_id, address):
    return _ack(c.ACK_WRITE_MEMORY, packet_id, struct.pack(">I", address))


def encode_pending_ack(packet_id, timeout_ms):
    return _ack(c.ACK_PENDING, packet_id, struct.pack(">I", timeout_ms))


def encode_error(command, packet_id, error_code):
    """
    An error ack still has to carry the ACK command id of the command that
    failed, and the packet id it is answering, or the client ignores it and
    burns its full retry budget instead.
    """
    return HEADER.pack(c.PACKET_TYPE_ERROR, error_code, command, 0, packet_id)


# --- request payload decoding -------------------------------------------

def decode_read_register(payload):
    """
    The spec allows a batch of addresses. Aravis only ever sends one, but
    handling N costs nothing and keeps other clients working.
    """
    if len(payload) < 4 or len(payload) % 4:
        raise MalformedPacket("read register payload must be a multiple of 4")
    return [struct.unpack_from(">I", payload, i)[0]
            for i in range(0, len(payload), 4)]


def decode_write_register(payload):
    if len(payload) < 8 or len(payload) % 8:
        raise MalformedPacket("write register payload must be a multiple of 8")
    out = []
    for i in range(0, len(payload), 8):
        address, value = struct.unpack_from(">II", payload, i)
        out.append((address, value))
    return out


def decode_read_memory(payload):
    if len(payload) < 8:
        raise MalformedPacket("read memory payload must be 8 bytes")
    address, count = struct.unpack_from(">II", payload, 0)
    # Only the low 16 bits are the count; the high half is reserved.
    return address, count & 0xFFFF


def decode_write_memory(payload):
    if len(payload) < 4:
        raise MalformedPacket("write memory payload must be at least 4 bytes")
    address = struct.unpack_from(">I", payload, 0)[0]
    return address, payload[4:]
