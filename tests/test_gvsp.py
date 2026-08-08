import math
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


@pytest.mark.parametrize("packet_size", [576, 1024, 1500, 8228, 9000])
def test_a_test_packet_is_exactly_the_size_asked_for(packet_size):
    """
    The client measures the arriving datagram, so being one byte out makes it
    settle on a packet size the device never agreed to.
    """
    datagram = gvsp.test_packet(packet_size)
    assert len(datagram) == gvsp.max_datagram_size(packet_size)
    # The IP and UDP headers the kernel adds bring it to the requested size.
    assert len(datagram) + 28 == packet_size


def test_a_test_packet_can_never_be_mistaken_for_a_frame():
    """
    Block id zero is what makes this safe: next_frame_id never emits it, so a
    client that feeds this to its reassembler cannot attach it to a real
    frame.
    """
    block_id = struct.unpack_from(">H", gvsp.test_packet(1500), 2)[0]
    assert block_id == 0


def test_a_test_packet_smaller_than_the_header_is_refused():
    assert gvsp.test_packet(c.GVSP_PROTOCOL_OVERHEAD - 1) is None


# --- gain in dB ----------------------------------------------------------
#
# The Pi example cannot be imported here (it needs picamera2), so its two
# conversion helpers are re-derived from the same definition. What is being
# pinned is the factor of 20: sensor gain is an amplitude ratio, and using 10
# halves every number while still looking entirely reasonable.

def _gain_to_db(linear):
    return 20.0 * math.log10(max(linear, 1e-6))


def _db_to_gain(db):
    return 10.0 ** (db / 20.0)


@pytest.mark.parametrize("linear,db", [
    (1.0, 0.0),
    (2.0, 6.0206),
    (10.0, 20.0),
    (22.2609, 26.950854),
])
def test_gain_converts_as_an_amplitude_ratio_not_a_power_one(linear, db):
    assert _gain_to_db(linear) == pytest.approx(db, abs=1e-3)


def test_gain_survives_the_round_trip_a_client_puts_it_through():
    """
    A client writes dB and reads dB back; the hardware only ever sees the
    linear factor in between.
    """
    for db in (0.0, 3.0, 6.0206, 12.0, 26.950854):
        assert _gain_to_db(_db_to_gain(db)) == pytest.approx(db, abs=1e-9)


def test_the_imx477_ceiling_is_where_analogue_gain_actually_saturates():
    linear = 1024.0 / (1024.0 - 978.0)
    assert linear == pytest.approx(22.2609, abs=1e-4)
    assert _gain_to_db(linear) == pytest.approx(26.95, abs=0.01)
