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
import time
import types

import pytest


class _Frame(object):
    """Stands in for the numpy array OpenCV hands back."""

    def __init__(self, tag):
        self.tag = tag

    def tobytes(self):
        return self.tag.encode()

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

    It also keeps capturing when nobody is reading, into a queue `depth`
    deep, which is the other half of what is under test here. The timing is
    what the real one does and what the code keys off: a queued frame is
    already in memory and comes back instantly, a live one costs a wait on
    the sensor.
    """

    #: Frames the driver holds. Four is what V4L2 gives OpenCV by default.
    depth = 4

    #: What the sensor takes per frame. The camera's nominal 30 is a lie, so
    #: this is deliberately not 1/30 -- the threshold has to work anyway.
    interval = 1.0 / 15.9

    def __init__(self, device=0):
        self.device = device
        self.values = {}
        self.rejected = []
        self.queued = 0
        self.stale_grabs = 0
        self.live_grabs = 0
        self.last_grabbed = None

    def leave_running(self):
        """Let the camera capture with nobody reading it."""
        self.queued = self.depth

    def grab(self):
        if self.queued > 0:
            self.queued -= 1
            self.stale_grabs += 1
            self.last_grabbed = _Frame("stale")
        else:
            self.live_grabs += 1
            self.last_grabbed = _Frame("live")
            time.sleep(self.interval)
        return True

    def retrieve(self):
        return True, self.last_grabbed

    def read(self):
        self.grab()
        return self.retrieve()

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
    # Colour conversion is not what is under test, and the stand-in frame
    # carries its tag through either branch.
    module.cvtColor = lambda frame, code: frame
    sys.modules.setdefault("cv2", module)
    sys.path.insert(0, os.path.abspath(EXAMPLES))
    return importlib.import_module("opencv_camera"), module


opencv_camera, cv2 = _load()


@pytest.fixture
def camera():
    cam = opencv_camera.OpenCvCamera()
    # next_frame() reads the geometry the device latches at AcquisitionStart,
    # so a test calling it directly has to latch it too.
    cam.latch_geometry()
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


def test_exposure_and_gain_are_the_cameras_to_decide(camera):
    """
    A webcam runs its own exposure loop, and gain is the other half of it --
    this driver pins gain at its maximum to serve the exposure it picked. A
    client writing either would be arguing with the algorithm.
    """
    for name in ("ExposureTime", "GainRaw"):
        assert camera.feature_set.by_name[name].access == "RO"


def test_the_camera_is_put_into_automatic_exposure(camera):
    """
    Written at startup whatever the camera was already doing, because manual
    exposure lives in the driver rather than in this process: it outlives the
    run, applies to every other program on the machine, and presents as black
    frames from a camera that reports no error at all.
    """
    assert camera.cap.values[cv2.CAP_PROP_AUTO_EXPOSURE] == \
        opencv_camera.AUTO_EXPOSURE_ON


def test_nothing_is_written_at_the_camera(camera):
    # Not merely refused upstream: the hook must not push these either, or a
    # camera that did accept one would end up disagreeing with the feature
    # the client is told is read only.
    before = dict(camera.cap.values)
    camera.set_camera_settings({"AcquisitionFrameRate": 5.0,
                                "ExposureTime": 5000.0, "GainRaw": 4})
    assert camera.cap.values == before
    assert camera.cap.rejected == []


def test_what_the_camera_chose_is_what_gets_reported(camera):
    camera.cap.values[cv2.CAP_PROP_EXPOSURE] = 50.0        # driver units
    camera.cap.values[cv2.CAP_PROP_GAIN] = 8
    reported = camera.get_camera_settings()
    assert reported["ExposureTime"] == 5000.0              # 100 us apiece
    assert reported["GainRaw"] == 8


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


# --- frames the driver queued while nobody was reading -------------------
#
# A camera keeps capturing between acquisitions and the driver keeps four of
# those frames. read() hands back the oldest, so without draining, the first
# frame of every acquisition is one taken when the last acquisition ended --
# and a snap is a whole acquisition, so that is the image the user sees.


def test_the_first_frame_of_a_run_is_not_one_from_the_queue(camera):
    camera.cap.leave_running()
    assert camera.next_frame() == b"live"
    assert camera.cap.stale_grabs == camera.cap.depth


def test_the_wait_that_found_the_live_frame_is_not_thrown_away(camera):
    # retrieve() decodes the frame the last grab took, so exactly one live
    # grab is spent. Grabbing again would cost another whole frame interval.
    camera.cap.leave_running()
    camera.next_frame()
    assert camera.cap.live_grabs == 1


def test_a_run_in_progress_is_not_drained_every_frame(camera):
    """
    Draining costs a frame interval to find the live one, so doing it per
    frame would halve the rate -- and there is nothing to drain anyway,
    since the stream thread is already reading as fast as frames arrive.
    """
    camera.cap.leave_running()
    camera.next_frame()
    before = camera.cap.stale_grabs
    for _ in range(4):
        camera.next_frame()
    assert camera.cap.stale_grabs == before


def test_coming_back_after_a_break_drains_again(camera):
    camera.cap.leave_running()
    for _ in range(4):
        camera.next_frame()
    drained_once = camera.cap.stale_grabs

    # The client went away; the camera kept capturing.
    camera._arrivals[-1] -= 60.0
    camera.cap.leave_running()
    assert camera.next_frame() == b"live"
    assert camera.cap.stale_grabs == drained_once + camera.cap.depth


def test_draining_is_bounded(camera):
    """
    A source whose grabs are all instant -- a video file rather than a
    camera -- must not be read to its end looking for one that waits.
    """
    camera.cap.depth = 10 ** 6
    camera.cap.leave_running()
    assert camera.next_frame() is not None
    assert camera.cap.stale_grabs == opencv_camera.OpenCvCamera.MAX_DISCARD
