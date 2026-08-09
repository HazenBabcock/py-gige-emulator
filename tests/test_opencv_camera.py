#
# The OpenCV example's frame rate handling, without a camera.
#
# cv2 is stubbed so this runs anywhere. Only the parts that do not touch
# OpenCV itself are exercised: whether the rate is offered as writable, and
# what it reports.
#

import importlib
import os
import sys
import types

import pytest

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "examples")

# The property ids are arbitrary here -- nothing under test depends on their
# real values, only on the stub and the example agreeing.
_PROPS = ["CAP_PROP_FPS", "CAP_PROP_FRAME_WIDTH", "CAP_PROP_FRAME_HEIGHT",
          "CAP_PROP_EXPOSURE", "CAP_PROP_GAIN", "CAP_PROP_AUTO_EXPOSURE",
          "COLOR_BGR2GRAY", "COLOR_BGR2RGB"]


class _FakeCapture(object):
    """
    A UVC webcam's actual behaviour: one frame interval per mode, so
    CAP_PROP_FPS is refused and keeps reading back as the camera's nominal
    figure. Measured on an Integrated_Webcam_FHD, which reports 30 while
    delivering 15.9.
    """

    def __init__(self, device=0):
        self.device = device
        self.values = {}
        self.rejected = []

    def isOpened(self):
        return True

    def get(self, prop):
        if prop == cv2.CAP_PROP_FPS:
            return 30.0
        return self.values.get(prop, 0.0)

    def set(self, prop, value):
        if prop == cv2.CAP_PROP_FPS:
            self.rejected.append(value)
            return False
        self.values[prop] = value
        return True

    def release(self):
        pass


def _load():
    module = types.ModuleType("cv2")
    for index, name in enumerate(_PROPS):
        setattr(module, name, index)
    module.VideoCapture = _FakeCapture
    sys.modules.setdefault("cv2", module)
    sys.path.insert(0, os.path.abspath(EXAMPLES))
    return importlib.import_module("opencv_camera"), module


opencv_camera, cv2 = _load()


@pytest.fixture
def camera():
    cam = opencv_camera.OpenCvCamera()
    yield cam
    cam.close()


# --- the rate is the camera's, not the client's --------------------------


def test_the_frame_rate_is_not_offered_as_writable(camera):
    """
    A UVC camera advertises one frame interval per format and size, so the
    rate follows the mode rather than being a control of its own -- and
    cap.set(CAP_PROP_FPS, x) returns False for every x. Offering it as
    writable let a client set any value and be told the write worked.
    """
    assert camera.feature_set.by_name["AcquisitionFrameRate"].access == "RO"


def test_nothing_writes_the_rate_at_the_camera(camera):
    # Not merely refused upstream: the hook must not push it either, or a
    # camera that did accept the value would end up disagreeing with the
    # feature the client is told is read only.
    camera.set_camera_settings({"AcquisitionFrameRate": 5.0})
    assert camera.cap.rejected == []


# --- what it reports ------------------------------------------------------


def test_the_rate_is_unknown_until_frames_have_arrived(camera):
    assert camera.measured_frame_rate() is None
    camera._note_arrival(100.0)
    assert camera.measured_frame_rate() is None
    assert "AcquisitionFrameRate" not in camera.get_camera_settings()


def test_the_rate_is_measured_rather_than_taken_from_the_camera(camera):
    # The camera claims 30. It is delivering 20, and that is what a client
    # has to be told -- the nominal figure was wrong by a factor of two on
    # the webcam this was written against.
    for i in range(6):
        camera._note_arrival(100.0 + i * 0.05)
    assert camera.cap.get(cv2.CAP_PROP_FPS) == 30.0
    assert camera.measured_frame_rate() == pytest.approx(20.0)
    assert camera.get_camera_settings()["AcquisitionFrameRate"] == \
        pytest.approx(20.0)


def test_the_measurement_follows_a_rate_that_changes(camera):
    for i in range(opencv_camera.OpenCvCamera.RATE_WINDOW):
        camera._note_arrival(100.0 + i * 0.05)             # 20 fps
    fast = camera.measured_frame_rate()
    at = 100.0 + opencv_camera.OpenCvCamera.RATE_WINDOW * 0.05
    for i in range(opencv_camera.OpenCvCamera.RATE_WINDOW):
        at += 0.1                                          # 10 fps
        camera._note_arrival(at)
    assert fast == pytest.approx(20.0)
    assert camera.measured_frame_rate() == pytest.approx(10.0)


def test_a_stop_does_not_become_a_frame_interval(camera):
    """
    Every snap is its own start and stop, so without forgetting the history
    the idle time between them is averaged in and the camera is reported at
    a fraction of a frame per second.
    """
    for i in range(6):
        camera._note_arrival(100.0 + i * 0.05)
    assert camera.measured_frame_rate() == pytest.approx(20.0)

    camera._note_arrival(160.0)             # a minute later, next snap
    assert camera.measured_frame_rate() is None

    for i in range(1, 6):
        camera._note_arrival(160.0 + i * 0.05)
    assert camera.measured_frame_rate() == pytest.approx(20.0)


def test_one_slow_frame_is_not_a_stop(camera):
    # The threshold is relative to the rate being seen, so a camera running
    # at one frame every two seconds is not mistaken for a stopped one.
    at = 100.0
    for _ in range(6):
        camera._note_arrival(at)
        at += 2.0
    assert camera.measured_frame_rate() == pytest.approx(0.5)
    # Slow, but not five times slow, so it is folded in rather than treated
    # as a discontinuity: six intervals now span 15 s instead of 12.
    camera._note_arrival(at + 3.0)
    assert camera.measured_frame_rate() == pytest.approx(0.4)
