#
# The parts of netif that do not touch a socket.
#

import pytest

from gige_emulator import netif


def test_a_mac_round_trips_through_both_forms():
    mac = netif.parse_mac("00:30:53:12:34:56")
    assert mac == b"\x00\x30\x53\x12\x34\x56"
    assert netif.format_mac(mac) == "00:30:53:12:34:56"


def test_dashes_are_accepted_because_the_ieee_registry_uses_them():
    # oui.txt writes "00-30-53   (hex)   Basler AG", and copying an OUI
    # straight out of it is the whole reason anyone types one of these.
    assert netif.parse_mac("00-30-53-12-34-56") == \
        netif.parse_mac("00:30:53:12:34:56")


def test_upper_and_lower_case_are_the_same_address():
    assert netif.parse_mac("00:0A:47:AB:CD:EF") == \
        netif.parse_mac("00:0a:47:ab:cd:ef")


@pytest.mark.parametrize("text", ["00:30:53:12:34",          # five octets
                                  "00:30:53:12:34:56:78",    # seven
                                  "",
                                  "003053123456"])           # no separators
def test_the_wrong_number_of_octets_is_refused(text):
    with pytest.raises(ValueError):
        netif.parse_mac(text)


def test_a_non_hexadecimal_octet_is_refused():
    with pytest.raises(ValueError):
        netif.parse_mac("00:30:53:12:34:gg")


def test_an_octet_that_does_not_fit_in_a_byte_is_refused():
    # int(part, 16) is happy to return 0x100, and the six bytes that reach
    # the bootstrap page would then not be the address that was asked for.
    with pytest.raises(ValueError):
        netif.parse_mac("00:30:53:12:34:100")
