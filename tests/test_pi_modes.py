#
# The Pi example's sensor mode resolution, without a Pi.
#
# Only the pure logic is exercised. picamera2 is stubbed so the module
# imports on a machine with no camera, which is worth the small amount of
# machinery because picking the wrong sensor readout is silent -- the stream
# runs, it is just capped at a lower frame rate than the mode you asked for.
#

import time
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


# --- read-back of a value the sensor has not applied yet -----------------
#
# Constructed without __init__, because that opens a camera. Only the
# pending-value bookkeeping is under test and it touches nothing else.

def _settings_stub():
    obj = pi_camera.PiCamera.__new__(pi_camera.PiCamera)
    obj._pending = {}
    return obj


def test_a_just_written_value_reads_back_as_written():
    """
    libcamera metadata lags a frame or more, and get_camera_settings()
    reports from it. Without this, writing 200 ms and reading back returns
    the *previous* exposure -- which in a GUI is a control that snaps back to
    its old value and then changes its mind a second later.
    """
    cam = _settings_stub()
    cam._note_requested("ExposureTime", 200000.0)
    assert cam._reported("ExposureTime", 90000.0) == 200000.0


def test_the_measurement_takes_over_once_the_sensor_agrees():
    """
    Exposure is quantised to the sensor's line time, so an exact match never
    arrives -- 200000 us is granted as 199787. Requiring equality would pin
    the reported value to the request forever and hide what the sensor did.
    """
    cam = _settings_stub()
    cam._note_requested("ExposureTime", 200000.0)
    assert cam._reported("ExposureTime", 199787.0) == 199787.0
    # And the request is finished with, so later readings pass straight
    # through rather than being held at the old value.
    assert cam._reported("ExposureTime", 150000.0) == 150000.0


def test_a_value_the_sensor_refuses_surfaces_in_the_end():
    """
    Reporting the request indefinitely would hide coercion. The deadline is
    what lets a value the hardware would not grant become visible.
    """
    cam = _settings_stub()
    cam._note_requested("ExposureTime", 200000.0)
    assert cam._reported("ExposureTime", 5000.0) == 200000.0
    cam._pending["ExposureTime"] = (200000.0, time.monotonic() - 1.0)
    assert cam._reported("ExposureTime", 5000.0) == 5000.0


def test_an_unrequested_feature_is_reported_as_measured():
    cam = _settings_stub()
    assert cam._reported("Gain", 6.0) == 6.0
