#
# GVSP packet codec. Pure functions over bytes -- no sockets here.
#
# Every packet starts with the same 8 byte big endian header:
#
#     u16 status (0), u16 block_id (frame id),
#     u32 packet_infos = (content_type << 24) | (packet_id & 0x00ffffff)
#
# A frame is one LEADER (packet id 0), N PAYLOAD packets (ids 1..N), and one
# TRAILER (id N+1).
#

import struct

from . import constants as c

_HEADER = struct.Struct(">HHI")
_LEADER_BODY = struct.Struct(">HHIIIIIIIHH")
_TRAILER_BODY = struct.Struct(">II")

LEADER_SIZE = _HEADER.size + _LEADER_BODY.size        # 44
TRAILER_SIZE = _HEADER.size + _TRAILER_BODY.size      # 16


def _header(frame_id, content_type, packet_id):
    return _HEADER.pack(c.GVSP_PACKET_TYPE_OK, frame_id,
                        (content_type << 24) | (packet_id & c.GVSP_PACKET_ID_MASK))


def test_packet(packet_size):
    """
    A packet of exactly `packet_size` bytes on the wire, for a client sizing
    its receive path.

    Block id zero, which `next_frame_id` never emits, so a client that hands
    this to its reassembler cannot mistake it for part of a real frame. The
    body is the header plus filler because only the length is under test --
    the client measures whether a datagram this large arrives at all.
    """
    body = max_datagram_size(packet_size) - _HEADER.size
    if body < 0:
        return None
    return _header(0, c.GVSP_CONTENT_PAYLOAD, 0) + b"\x00" * body


def next_frame_id(frame_id):
    """
    Frame ids are 16 bit, must strictly increase, and zero is not valid --
    the client treats a frame id that does not advance as a late frame and
    discards it.
    """
    frame_id = (frame_id + 1) & 0xFFFF
    return 1 if frame_id == 0 else frame_id


def leader(frame_id, timestamp_ns, pixel_format, width, height,
           x_offset=0, y_offset=0, x_padding=0, y_padding=0):
    """
    The leader must carry packet id 0; the client rejects the frame outright
    otherwise.
    """
    return _header(frame_id, c.GVSP_CONTENT_LEADER, 0) + _LEADER_BODY.pack(
        0,                          # flags
        c.PAYLOAD_TYPE_IMAGE,
        (timestamp_ns >> 32) & 0xFFFFFFFF,
        timestamp_ns & 0xFFFFFFFF,
        pixel_format, width, height,
        x_offset, y_offset,
        x_padding, y_padding)


def payload(frame_id, packet_id, data):
    return _header(frame_id, c.GVSP_CONTENT_PAYLOAD, packet_id) + bytes(data)


def trailer(frame_id, packet_id, height):
    """
    The trailer's packet id is what tells the client how long the frame
    actually was, so it must be exactly one past the last payload packet.
    """
    return _header(frame_id, c.GVSP_CONTENT_TRAILER, packet_id) + \
        _TRAILER_BODY.pack(c.PAYLOAD_TYPE_IMAGE, height)


def data_bytes_per_packet(packet_size):
    return packet_size - c.GVSP_PROTOCOL_OVERHEAD


def packet_count(payload_length, packet_size):
    """Number of PAYLOAD packets, excluding leader and trailer."""
    per_packet = data_bytes_per_packet(packet_size)
    if per_packet <= 0:
        raise ValueError("packet size %d is too small" % packet_size)
    return (payload_length + per_packet - 1) // per_packet


def packetize(frame_data, packet_size, frame_id, geometry, timestamp_ns):
    """
    Yields every datagram for one frame, in order.

    The chunking is not a free choice. The client never reads a per packet
    offset -- it computes one as (packet_id - 1) * (packet_size - 36) -- so
    every payload packet except the last must carry exactly that many bytes.
    Any other split silently scrambles the image.
    """
    per_packet = data_bytes_per_packet(packet_size)
    if per_packet <= 0:
        raise ValueError("packet size %d leaves no room for data" % packet_size)

    view = memoryview(frame_data)
    total = len(view)

    yield leader(frame_id, timestamp_ns, geometry["pixel_format"],
                 geometry["width"], geometry["height"])

    packet_id = 1
    offset = 0
    while offset < total:
        chunk = view[offset:offset + per_packet]
        yield payload(frame_id, packet_id, chunk)
        offset += len(chunk)
        packet_id += 1

    yield trailer(frame_id, packet_id, geometry["height"])


def max_datagram_size(packet_size):
    """
    The client's receive buffer is exactly packet_size - 28 bytes and it
    truncates anything larger without reporting an error, so this is a hard
    ceiling rather than a guideline.
    """
    return packet_size - c.GVSP_UDP_OVERHEAD
