#
# The Pi example's sensor mode resolution, without a Pi.
#
# Only the pure logic is exercised. picamera2 is stubbed so the module
# imports on a machine with no camera, which is worth the small amount of
# machinery because picking the wrong sensor readout is silent -- the stream
# runs, it is just capped at a lower frame rate than the mode you asked for.
#

import importlib
import os
import sys
import types

import pytest

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "examples")


def _load():
    for name, attrs in (("picamera2", {"Picamera2": object}),
                        ("picamera2.encoders", {"Encoder": object}),
                        ("picamera2.outputs", {"Output": object})):
        module = types.ModuleType(name)
        for attr, value in attrs.items():
            setattr(module, attr, value)
        sys.modules.setdefault(name, module)
    sys.path.insert(0, os.path.abspath(EXAMPLES))
    return importlib.import_module("pi_camera")


pi_camera = _load()


# The real IMX477 list, as reported by picamera2 on a Pi 5. Every size exists
# at three depths, and only the 8 bit one is spelled without _CSI2P.
MODES = [
    {"size": (1332, 990), "format": "SRGGB10_CSI2P", "bit_depth": 10, "fps": 120.5},
    {"size": (2028, 1520), "format": "SRGGB10_CSI2P", "bit_depth": 10, "fps": 53.77},
    {"size": (1332, 990), "format": "SRGGB12_CSI2P", "bit_depth": 12, "fps": 101.68},
    {"size": (2028, 1520), "format": "SRGGB12_CSI2P", "bit_depth": 12, "fps": 45.19},
    {"size": (1332, 990), "format": "SRGGB8", "bit_depth": 8, "fps": 147.91},
    {"size": (2028, 1520), "format": "SRGGB8", "bit_depth": 8, "fps": 66.38},
]


def test_a_bare_size_picks_the_deepest_readout():
    """
    A client can always throw precision away; it cannot recover a shallower
    readout, so depth is the right default when the size alone is given.
    """
    mode = pi_camera.parse_mode("1332x990", MODES)
    assert mode["bit_depth"] == 12


def test_the_format_selects_the_readout_and_therefore_the_rate():
    """
    This is the whole point of honouring it: same size, 8 bit against 12,
    and the sensor's own numbers differ by 45%.
    """
    fast = pi_camera.parse_mode("1332x990/SRGGB8", MODES)
    deep = pi_camera.parse_mode("1332x990/SRGGB12_CSI2P", MODES)
    assert fast["fps"] > deep["fps"] * 1.4
    assert fast["bit_depth"] == 8 and deep["bit_depth"] == 12


def test_the_packed_spelling_is_requested_unpacked():
    """
    Asking libcamera for _CSI2P yields a PiSP-compressed buffer with no
    GenICam equivalent, which silently costs the raw Bayer format. The
    unpacked name selects the same readout and stays labelable.
    """
    assert pi_camera.unpacked_format("SRGGB12_CSI2P") == "SRGGB12"
    assert pi_camera.unpacked_format("SRGGB10_CSI2P") == "SRGGB10"
    # Already unpacked, and not truncated by accident.
    assert pi_camera.unpacked_format("SRGGB8") == "SRGGB8"


def test_an_unknown_size_lists_what_the_sensor_has():
    with pytest.raises(ValueError) as excinfo:
        pi_camera.parse_mode("2028x1521", MODES)
    assert "2028x1520/SRGGB8" in str(excinfo.value)


def test_an_unknown_format_lists_only_that_size():
    with pytest.raises(ValueError) as excinfo:
        pi_camera.parse_mode("2028x1520/SRGGB14", MODES)
    message = str(excinfo.value)
    assert "2028x1520/SRGGB12_CSI2P" in message
    assert "1332x990" not in message      # the other size is just noise here


def test_a_malformed_mode_says_what_was_wanted():
    with pytest.raises(ValueError) as excinfo:
        pi_camera.parse_mode("nonsense", MODES)
    assert "WIDTHxHEIGHT" in str(excinfo.value)


def test_the_label_round_trips_through_the_parser():
    """
    --list-modes prints these and tells the user to pass one back, so every
    label it emits has to be accepted.
    """
    for mode in MODES:
        assert pi_camera.parse_mode(pi_camera.mode_label(mode), MODES) is mode
