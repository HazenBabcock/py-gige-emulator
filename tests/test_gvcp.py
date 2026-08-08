import struct

import pytest

from gige_emulator import constants as c
from gige_emulator import gvcp


def test_header_size_excludes_the_header():
    packet = gvcp.encode_read_register_ack(0x1234, [0xDEADBEEF])
    packet_type, flags, command, size, packet_id = gvcp.HEADER.unpack_from(packet)
    assert packet_type == c.PACKET_TYPE_ACK
    assert command == c.ACK_READ_REGISTER
    assert packet_id == 0x1234
    assert size == 4
    assert len(packet) == 8 + size


def test_discovery_ack_is_256_bytes():
    page = bytes(range(256))[:c.DISCOVERY_DATA_SIZE]
    packet = gvcp.encode_discovery_ack(0xFFFF, page)
    assert len(packet) == 8 + 248
    assert packet[8:] == page
    # The client only accepts a discovery ack whose id is 0xffff.
    assert gvcp.HEADER.unpack_from(packet)[4] == 0xFFFF


def test_discovery_ack_rejects_a_wrong_sized_page():
    with pytest.raises(ValueError):
        gvcp.encode_discovery_ack(1, b"\x00" * 10)


def test_read_memory_ack_echoes_the_address_before_the_data():
    data = b"payload!"
    packet = gvcp.encode_read_memory_ack(7, 0x10000, data)
    assert len(packet) == 8 + 4 + len(data)
    assert struct.unpack_from(">I", packet, 8)[0] == 0x10000
    assert packet[12:] == data


def test_parse_command_rejects_non_command_packets():
    ack = gvcp.encode_write_register_ack(1)
    assert gvcp.parse_command(ack) is None
    assert gvcp.parse_command(b"") is None
    assert gvcp.parse_command(b"\x42\x01\x00") is None


def test_parse_command_round_trip():
    payload = struct.pack(">II", 0x0A00, 2)
    data = gvcp.HEADER.pack(c.PACKET_TYPE_CMD, c.CMD_FLAGS_ACK_REQUIRED,
                            c.CMD_WRITE_REGISTER, len(payload), 42) + payload
    command = gvcp.parse_command(data)
    assert command.command == c.CMD_WRITE_REGISTER
    assert command.packet_id == 42
    assert gvcp.decode_write_register(command.payload) == [(0x0A00, 2)]


def test_read_memory_count_uses_only_the_low_16_bits():
    # The client puts a reserved field in the high half of the count word.
    payload = struct.pack(">II", 0x200, 0xDEAD0200)
    address, count = gvcp.decode_read_memory(payload)
    assert address == 0x200
    assert count == 0x0200


def test_read_register_decodes_a_batch():
    payload = struct.pack(">III", 1, 2, 3)
    assert gvcp.decode_read_register(payload) == [1, 2, 3]


def test_error_carries_the_ack_command_and_packet_id():
    packet = gvcp.encode_error(c.ACK_WRITE_REGISTER, 99, c.ERROR_ACCESS_DENIED)
    packet_type, flags, command, size, packet_id = gvcp.HEADER.unpack_from(packet)
    assert packet_type == c.PACKET_TYPE_ERROR
    assert flags == c.ERROR_ACCESS_DENIED
    assert command == c.ACK_WRITE_REGISTER
    assert packet_id == 99
    assert size == 0
    # Bytes 0 and 1 together are the GEV status word.
    assert struct.unpack_from(">H", packet, 0)[0] == 0x8006


def test_malformed_payloads_raise():
    with pytest.raises(gvcp.MalformedPacket):
        gvcp.decode_read_register(b"\x00\x00")
    with pytest.raises(gvcp.MalformedPacket):
        gvcp.decode_write_register(b"\x00" * 5)
    with pytest.raises(gvcp.MalformedPacket):
        gvcp.decode_read_memory(b"\x00" * 4)
