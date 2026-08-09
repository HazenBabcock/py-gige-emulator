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
import threading
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

def _sink():
    # 2x2, one byte a pixel, so a frame is four bytes and unpadded.
    return pi_camera._FrameSink(2, 2, 1)


def _settings_stub(rate=10.0):
    obj = pi_camera.PiCamera.__new__(pi_camera.PiCamera)
    obj._pending = {}
    obj._settling = {}
    obj._settle_lock = threading.Lock()
    obj._requested_rate = rate
    # Consulted for the frame duration the sensor reports, which is what the
    # settle window is counted in.
    obj.sink = _sink()
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


# --- the exposure / frame rate constraint --------------------------------
#
# Both directions, and both sides of the conflict. The no-conflict cases are
# here because they are the ones that broke: the cross-clamp binds its
# variable inside the conflict, so a write that needed no clamping raised
# UnboundLocalError before set_controls() ran and applied *nothing*. That is
# also the ordinary case -- the bench only ever exercised writes large enough
# to conflict, which took the working path.


class _FakePicam2(object):

    def __init__(self):
        self.controls = None

    def set_controls(self, controls):
        self.controls = dict(controls)


def _constraint_stub(exposure_us, rate):
    cam = _settings_stub(rate=rate)
    cam.picam2 = _FakePicam2()
    cam._requested_exposure_us = float(exposure_us)
    cam._requested_rate = float(rate)
    return cam


def test_an_exposure_the_rate_allows_is_applied_alone():
    # 10 fps needs 100 ms per frame and the exposure asks for 20, so there is
    # nothing to resolve and the rate must be left alone.
    cam = _constraint_stub(exposure_us=5000, rate=10.0)
    cam.set_camera_settings({"ExposureTime": 20000})
    assert cam.picam2.controls == {"ExposureTime": 20000}
    assert cam._requested_rate == 10.0
    # And the rate is not claimed as pending, so it keeps reading back as
    # whatever the sensor is actually achieving.
    assert "AcquisitionFrameRate" not in cam._pending


def test_an_exposure_longer_than_the_frame_drags_the_rate_down():
    cam = _constraint_stub(exposure_us=5000, rate=10.0)
    cam.set_camera_settings({"ExposureTime": 500000})
    assert cam.picam2.controls == {"ExposureTime": 500000, "FrameRate": 2.0}
    assert cam._requested_rate == 2.0
    assert cam._pending["AcquisitionFrameRate"][0] == 2.0


def test_a_rate_the_exposure_allows_is_applied_alone():
    cam = _constraint_stub(exposure_us=20000, rate=2.0)
    cam.set_camera_settings({"AcquisitionFrameRate": 10.0})
    assert cam.picam2.controls == {"FrameRate": 10.0}
    assert cam._requested_exposure_us == 20000.0
    assert "ExposureTime" not in cam._pending


def test_a_rate_faster_than_the_exposure_shortens_it():
    cam = _constraint_stub(exposure_us=500000, rate=2.0)
    cam.set_camera_settings({"AcquisitionFrameRate": 10.0})
    assert cam.picam2.controls == {"FrameRate": 10.0, "ExposureTime": 100000}
    assert cam._requested_exposure_us == 100000.0
    assert cam._pending["ExposureTime"][0] == 100000.0


# --- frames exposed before the control reached the sensor ----------------
#
# libcamera applies a control to a frame that has not started exposing, so
# for several frames after a write the sensor is still delivering the old
# setting. Handing those out is what made a snap in micro-manager show the
# previous exposure until you had clicked through the whole queue.


def test_a_frame_from_before_the_write_is_held_back():
    cam = _settings_stub()
    cam._note_settling({"ExposureTime": 40000})
    assert cam._settled({"ExposureTime": 5000}) is False
    # Quantisation means the sensor never reports the exact request, so the
    # match is the same tolerance the read-back uses.
    assert cam._settled({"ExposureTime": 39900}) is True


def test_the_hold_ends_once_the_sensor_has_agreed_once():
    """
    The check is for the change arriving, not for the value staying. Leaving
    the entry in place would re-judge every later frame against a request the
    client may since have moved on from.
    """
    cam = _settings_stub()
    cam._note_settling({"ExposureTime": 40000})
    assert cam._settled({"ExposureTime": 40000}) is True
    assert cam._settled({"ExposureTime": 5000}) is True


def test_gain_is_judged_in_libcamera_units():
    # Gain is dB across the wire and a linear multiplier here. Recording the
    # controls actually pushed, rather than the feature values, is what keeps
    # those from being compared against each other.
    cam = _settings_stub()
    cam._note_settling({"AnalogueGain": 4.0})
    assert cam._settled({"AnalogueGain": 1.0}) is False
    assert cam._settled({"AnalogueGain": 4.0}) is True


def test_the_frame_rate_alone_does_not_hold_frames():
    # A frame taken at the old rate is the same picture, sooner.
    cam = _settings_stub()
    cam._note_settling({"FrameRate": 2.0})
    assert cam._settled({"FrameRate": 10.0}) is True


def test_a_value_the_sensor_never_applies_stops_holding_frames():
    """
    A snap that never returns is worse than one showing the value the
    hardware actually chose, so the hold is bounded.
    """
    cam = _settings_stub()
    cam._note_settling({"ExposureTime": 40000})
    assert cam._settled({"ExposureTime": 5000}) is False
    cam._settling["ExposureTime"] = (40000.0, time.monotonic() - 1.0)
    assert cam._settled({"ExposureTime": 5000}) is True
    assert cam._settling == {}


def test_a_frame_with_no_metadata_is_not_held():
    # Nothing to judge by is not a reason to withhold frames.
    cam = _settings_stub()
    cam._note_settling({"ExposureTime": 40000})
    assert cam._settled({}) is True


def test_the_settle_window_follows_the_frame_rate():
    # It is a frame count, so the time it comes to has to track the rate. A
    # flat two seconds -- what this was -- expires before the sensor has
    # applied anything at the ~4.75 fps of a full resolution raw stream.
    assert _settings_stub(rate=10.0)._settle_seconds() == pytest.approx(1.2)
    assert _settings_stub(rate=2.0)._settle_seconds() == pytest.approx(6.0)


def test_the_settle_window_believes_the_sensor_over_the_request():
    cam = _settings_stub(rate=100.0)        # more than the readout can serve
    cam.sink.metadata = {"FrameDuration": 25000}            # 40 fps, really
    assert cam._settle_seconds() == pytest.approx(0.3)


def test_the_settle_window_is_capped():
    cam = _settings_stub(rate=0.1)
    assert cam._settle_seconds() == pi_camera.PiCamera.SETTLE_SECONDS_MAX


# --- the sink side of the same thing --------------------------------------


def test_the_sink_hands_over_a_frame_that_passes():
    sink = _sink()
    sink.note_metadata({"ExposureTime": 5000})
    sink.outputframe(bytes([1, 2, 3, 4]), timestamp=7)
    taken = sink.take(timeout=0.5, accept=lambda md: md["ExposureTime"] == 5000)
    assert taken is not None
    assert taken[0] == bytes([1, 2, 3, 4])
    assert taken[2] == 7000            # microseconds in, nanoseconds out


def test_the_sink_drops_a_rejected_frame_rather_than_spinning_on_it():
    sink = _sink()
    sink.note_metadata({"ExposureTime": 5000})
    sink.outputframe(bytes([1, 2, 3, 4]), timestamp=1)
    started = time.monotonic()
    assert sink.take(timeout=0.2,
                     accept=lambda md: md["ExposureTime"] == 40000) is None
    # Left in place it would satisfy the wait immediately and be rejected
    # again, burning the timeout at whatever rate the loop can run.
    assert sink.frame is None
    assert time.monotonic() - started >= 0.2


def test_the_sink_waits_for_the_frame_that_has_the_change():
    sink = _sink()
    sink.note_metadata({"ExposureTime": 5000})
    sink.outputframe(bytes([1, 2, 3, 4]), timestamp=1)

    def deliver_later():
        time.sleep(0.05)
        sink.note_metadata({"ExposureTime": 40000})
        sink.outputframe(bytes([9, 9, 9, 9]), timestamp=2)

    threading.Thread(target=deliver_later, daemon=True).start()
    taken = sink.take(timeout=2.0,
                      accept=lambda md: md["ExposureTime"] == 40000)
    assert taken is not None
    assert taken[0] == bytes([9, 9, 9, 9])
