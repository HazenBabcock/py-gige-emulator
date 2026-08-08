import os
import struct

import pytest

from gige_emulator import constants as c
from gige_emulator import gvsp


def geometry(width=64, height=48, pixel_format=c.PIXEL_FORMAT_MONO8):
    return {"width": width, "height": height, "pixel_format": pixel_format,
            "payload": c.payload_size(width, height, pixel_format)}


def split(datagram):
    status, block_id, infos = struct.unpack_from(">HHI", datagram, 0)
    return status, block_id, (infos >> 24) & 0x7F, infos & c.GVSP_PACKET_ID_MASK


def reassemble(datagrams, packet_size):
    """
    Rebuild the frame the way the client does.

    This deliberately mirrors arvgvstream.c rather than the packetizer: the
    client never reads a per packet offset, it computes one from the packet
    id, so a packetizer that chunks differently produces a scrambled image
    that no amount of testing against our own logic would catch.
    """
    per_packet = packet_size - c.GVSP_PROTOCOL_OVERHEAD
    chunks = {}
    leader = trailer = None
    for datagram in datagrams:
        status, block_id, content, packet_id = split(datagram)
        assert status == 0
        if content == c.GVSP_CONTENT_LEADER:
            leader = (block_id, packet_id, datagram)
        elif content == c.GVSP_CONTENT_TRAILER:
            trailer = (block_id, packet_id, datagram)
        else:
            chunks[packet_id] = datagram[8:]

    assert leader is not None and trailer is not None
    out = bytearray()
    for packet_id in sorted(chunks):
        offset = (packet_id - 1) * per_packet
        assert offset == len(out), "packet %d lands at %d, expected %d" % (
            packet_id, offset, len(out))
        out += chunks[packet_id]
    return leader, trailer, bytes(out)


@pytest.mark.parametrize("packet_size", [576, 1400, 1500, 9000])
@pytest.mark.parametrize("width,height", [(64, 48), (640, 480), (13, 7)])
def test_a_frame_survives_the_round_trip(packet_size, width, height):
    geo = geometry(width, height)
    frame = os.urandom(geo["payload"])
    datagrams = list(gvsp.packetize(frame, packet_size, 1, geo, 12345))
    leader, trailer, rebuilt = reassemble(datagrams, packet_size)
    assert rebuilt == frame


def test_leader_is_44_bytes_at_packet_id_zero():
    geo = geometry()
    datagrams = list(gvsp.packetize(b"\x00" * geo["payload"], 1400, 5, geo, 99))
    assert len(datagrams[0]) == gvsp.LEADER_SIZE == 44
    status, block_id, content, packet_id = split(datagrams[0])
    assert content == c.GVSP_CONTENT_LEADER
    assert packet_id == 0, "the client rejects a leader that is not packet 0"
    assert block_id == 5


def test_trailer_is_16_bytes_one_past_the_last_payload():
    geo = geometry(640, 480)
    datagrams = list(gvsp.packetize(b"\x00" * geo["payload"], 1400, 5, geo, 99))
    assert len(datagrams[-1]) == gvsp.TRAILER_SIZE == 16
    status, block_id, content, packet_id = split(datagrams[-1])
    assert content == c.GVSP_CONTENT_TRAILER
    n_payload = gvsp.packet_count(geo["payload"], 1400)
    assert packet_id == n_payload + 1
    assert len(datagrams) == n_payload + 2


def test_every_payload_but_the_last_is_exactly_full():
    geo = geometry(640, 480)
    packet_size = 1400
    per_packet = packet_size - c.GVSP_PROTOCOL_OVERHEAD
    datagrams = list(gvsp.packetize(b"\x00" * geo["payload"], packet_size, 1,
                                    geo, 0))
    payloads = datagrams[1:-1]
    for datagram in payloads[:-1]:
        assert len(datagram) - 8 == per_packet
    assert 0 < len(payloads[-1]) - 8 <= per_packet


@pytest.mark.parametrize("packet_size", [576, 1400, 9000])
def test_no_datagram_exceeds_the_clients_receive_buffer(packet_size):
    geo = geometry(640, 480)
    ceiling = gvsp.max_datagram_size(packet_size)
    for datagram in gvsp.packetize(b"\x00" * geo["payload"], packet_size, 1,
                                   geo, 0):
        assert len(datagram) <= ceiling


def test_leader_carries_the_geometry():
    geo = geometry(640, 480, c.PIXEL_FORMAT_MONO16)
    datagram = gvsp.leader(7, 0x1122334455667788, geo["pixel_format"],
                           geo["width"], geo["height"])
    fields = struct.unpack_from(">HHIIIIIIIHH", datagram, 8)
    flags, payload_type, ts_hi, ts_lo, fmt, width, height = fields[:7]
    assert payload_type == c.PAYLOAD_TYPE_IMAGE
    assert (ts_hi << 32 | ts_lo) == 0x1122334455667788
    assert fmt == c.PIXEL_FORMAT_MONO16
    assert (width, height) == (640, 480)


def test_frame_ids_wrap_past_zero():
    assert gvsp.next_frame_id(0) == 1
    assert gvsp.next_frame_id(1) == 2
    assert gvsp.next_frame_id(65534) == 65535
    # Zero is not a valid frame id; the client treats it as a late frame.
    assert gvsp.next_frame_id(65535) == 1


def test_frame_id_never_returns_zero_over_a_full_wrap():
    frame_id = 0
    for _ in range(70000):
        frame_id = gvsp.next_frame_id(frame_id)
        assert frame_id != 0


def test_packet_size_too_small_is_refused():
    geo = geometry()
    with pytest.raises(ValueError):
        list(gvsp.packetize(b"\x00" * geo["payload"], 36, 1, geo, 0))
