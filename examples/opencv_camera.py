#
# Serve any OpenCV-readable camera as a GigE Vision camera.
#
#   python examples/opencv_camera.py --interface eth0
#   python examples/opencv_camera.py --interface eth0 --width 1280 --height 720 \
#                                    --pixel-format RGB8
#
# Works with UVC webcams on Linux, and with anything else cv2.VideoCapture
# will open.
#

import argparse
import collections
import logging
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from gige_emulator import (EmulatedCamera, FloatFeature, GigECameraServer,
                           IntFeature, netif)

log = logging.getLogger("opencv_camera")

# V4L2 reports exposure in units of 100 us. This is a convention rather than
# a guarantee -- other backends and other cameras scale differently -- so it
# is exposed as a constructor argument.
DEFAULT_EXPOSURE_SCALE = 100.0

# cv2.CAP_PROP_AUTO_EXPOSURE on the V4L2 backend: 3 is auto, 1 is manual.
AUTO_EXPOSURE_ON = 3.0
AUTO_EXPOSURE_OFF = 1.0

# OpenCV has no API for listing a camera's supported modes, so the only
# portable way to find them is to ask for each in turn and see what comes
# back. These are the common UVC sizes; a camera that supports something else
# can still be driven with an explicit --width/--height.
CANDIDATE_MODES = [
    (160, 120), (320, 180), (320, 240), (352, 288), (640, 360), (640, 480),
    (800, 600), (848, 480), (960, 540), (1024, 768), (1280, 720),
    (1280, 960), (1280, 1024), (1600, 896), (1600, 1200), (1920, 1080),
    (2560, 1440), (3840, 2160),
]


def probe_modes(device, candidates=CANDIDATE_MODES):
    """
    Ask the camera for each candidate size and record what it actually gave.

    A webcam substitutes its nearest supported mode rather than failing, so
    the set of distinct answers is the set of real modes -- but only for the
    sizes probed. This is a discovery aid, not an authoritative list; on
    Linux `v4l2-ctl --list-formats-ext` is the ground truth.
    """
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError("cannot open OpenCV device %r" % (device,))
    found = []
    try:
        for width, height in candidates:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                      int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if actual not in found and actual != (0, 0):
                found.append(actual)
    finally:
        cap.release()
    return found


class OpenCvCamera(EmulatedCamera):

    extra_features = (
        FloatFeature("ExposureTime", "Exposure time", "AcquisitionControl",
                     "RW", default=10000.0, min=1.0, max=1e6, unit="us"),
        # GainRaw, not Gain, and deliberately. The convention's Gain is a
        # float in dB, which needs the underlying value to be a linear
        # multiplier -- and cv2.CAP_PROP_GAIN is whatever V4L2 control the
        # driver happens to expose, in units it does not report. Converting
        # would be inventing information, and the range includes zero, which
        # has no dB value at all. GainRaw is the GenICam 1.x name that exists
        # for exactly this case: device-specific integer gain. The Pi example
        # does have a real multiplier and uses Gain in dB.
        IntFeature("GainRaw", "Analog gain, in the driver's own units",
                   "AnalogControl", "RW",
                   default=1, min=0, max=255),
    )

    def __init__(self, device=0, width=640, height=480, pixel_format="Mono8",
                 exposure_scale=DEFAULT_EXPOSURE_SCALE, **kwds):

        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError("cannot open OpenCV device %r" % (device,))

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Use what the camera actually gave us, not what we asked for. A
        # webcam is free to ignore the request and hand back its nearest
        # supported mode; believing the request instead would make every
        # frame the wrong length for the payload size the client was told,
        # and the client reports that as a black image rather than an error.
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_width, actual_height) != (width, height):
            log.warning("asked for %dx%d, camera supplied %dx%d",
                        width, height, actual_width, actual_height)

        frame_rate = self.cap.get(cv2.CAP_PROP_FPS) or 30.0

        self.pixel_format = pixel_format
        self.exposure_scale = exposure_scale
        self.n_grab_failures = 0

        # When each of the last few frames arrived, for the measured rate
        # reported below. Bounded because the interesting rate is the one the
        # camera is running at now, not its average since it was opened.
        self._arrivals = collections.deque(maxlen=self.RATE_WINDOW)

        super().__init__(width=actual_width, height=actual_height,
                         pixel_format=pixel_format,
                         pixel_formats=["Mono8", "RGB8"],
                         frame_rate=frame_rate, **kwds)

        # The frame rate is not ours to set. A UVC camera advertises one
        # discrete frame interval per format and size -- this webcam offers
        # YUYV 640x480 at 30 and nothing else, and YUYV 1920x1080 at 5 and
        # nothing else -- so the rate is a consequence of the mode rather
        # than a control, and cap.set(CAP_PROP_FPS, x) duly returns False for
        # every x. Declaring it writable let a client set any value it liked
        # and be told the write succeeded, since nothing ever read the rate
        # back off the camera.
        #
        # Wired here rather than in the core because it is not true of
        # cameras in general: the Pi example's rate really is writable.
        self.feature_set.by_name["AcquisitionFrameRate"].access = "RO"

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    # --- the three hooks -------------------------------------------------

    def next_frame(self):
        """
        cap.read() blocks until the sensor has a frame, so this is also what
        paces the stream -- except on the first frame of a run, where the
        queue has to be skipped first.
        """
        now = time.monotonic()
        gap = self._gap_before(now)
        if gap is None or gap > self.RATE_GAP:
            ok, frame = self._read_current()
        else:
            ok, frame = self.cap.read()
        if not ok:
            self.n_grab_failures += 1
            return None      # the stream thread simply tries again

        self._note_arrival(time.monotonic())

        # OpenCV hands back BGR. Mono8 wants one byte per pixel and RGB8
        # wants R, G, B in that order, so neither is the raw buffer.
        if self.geometry["pixel_format"] == 0x01080001:      # Mono8
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # tobytes() rather than .data, because a cvtColor result can be
        # non-contiguous and .data would then be the wrong bytes.
        return frame.tobytes()

    def set_camera_settings(self, changed):
        if "ExposureTime" in changed:
            # Auto exposure overrides anything we write, so it has to go
            # first -- and it has to go first every time, because some
            # drivers re-enable it when the stream restarts.
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, AUTO_EXPOSURE_OFF)
            raw = changed["ExposureTime"] / self.exposure_scale
            if not self.cap.set(cv2.CAP_PROP_EXPOSURE, raw):
                log.warning("camera refused exposure %g us (raw %g)",
                            changed["ExposureTime"], raw)

        if "GainRaw" in changed:
            if not self.cap.set(cv2.CAP_PROP_GAIN, float(changed["GainRaw"])):
                log.warning("camera refused gain %d", changed["GainRaw"])

        # AcquisitionFrameRate is deliberately absent: it is read only, so
        # this hook is never called for it. The rate belongs to the camera --
        # cap.read() blocking at whatever rate it settles on is what paces
        # the stream.

    #: Frames the measured rate averages over. Long enough not to jump about
    #: on one slow read, short enough to follow a rate that has genuinely
    #: changed -- at 30 fps this is half a second of history.
    RATE_WINDOW = 15

    #: A gap this many times the rate we have been seeing is taken as the
    #: stream having stopped rather than as one slow frame. Relative rather
    #: than a flat number of seconds, so it means the same thing to a 30 fps
    #: webcam and to a camera running at one frame every two seconds.
    RATE_GAP = 5.0

    def _gap_before(self, now):
        """
        How long since the last frame, counted in frames at the rate we were
        seeing, or None when there is no run to compare against.

        One measure serving both the rate window and the queue skip below,
        because they are asking the same question -- is this the next frame
        of a run, or the first after a break -- and two thresholds that
        disagreed would be a camera that forgets its rate without draining,
        or drains without forgetting.
        """
        rate = self.measured_frame_rate()
        if rate is None:
            return None
        return (now - self._arrivals[-1]) * rate

    def _note_arrival(self, now):
        """
        Record when a frame arrived, forgetting the history across a stop.

        Without this a client that stops acquiring and comes back a minute
        later gets that minute averaged in as a frame interval, and is told
        the camera runs at 0.02 fps until the window refills. Snapping does
        exactly that -- every snap is its own start and stop.
        """
        gap = self._gap_before(now)
        if gap is not None and gap > self.RATE_GAP:
            self._arrivals.clear()
        self._arrivals.append(now)

    #: Most frames to discard when picking up a camera that has been left
    #: running. Four deep on a UVC webcam, so this is a backstop against a
    #: source whose grabs are always instant -- a video file, say -- which
    #: would otherwise be drained frame by frame to its end.
    MAX_DISCARD = 32

    def _expected_interval(self):
        rate = self.measured_frame_rate() or self.cap.get(cv2.CAP_PROP_FPS)
        return 1.0 / rate if rate else 1.0 / 30.0

    def _read_current(self):
        """
        read(), less whatever the driver queued while nobody was reading.

        A camera keeps capturing between acquisitions, and those frames go
        into the driver's queue -- four deep here. read() hands back the
        oldest, so the first frames of every acquisition are the ones taken
        just after the last one ended. Measured after idling five seconds:
        four frames returned in about a millisecond each, their timestamps
        advancing 60 ms apiece while five seconds of wall clock had passed,
        and only the fifth was live.

        A snap is a whole acquisition, so this is not a cosmetic first-frame
        blemish: it shows the scene as it was when the previous snap ended.

        Telling stale from live needs no frame count, and does not have to
        trust one. A queued frame is already in memory and comes back
        instantly; a live one costs a wait on the sensor. So grab until one
        of them makes us wait, and keep that.
        """
        threshold = self._expected_interval() / 2.0
        for _ in range(self.MAX_DISCARD):
            started = time.monotonic()
            if not self.cap.grab():
                return False, None
            if (time.monotonic() - started) >= threshold:
                break
        # retrieve() decodes the frame the last grab took, which is the one
        # that waited -- so the wait is not spent and then thrown away.
        return self.cap.retrieve()

    def measured_frame_rate(self):
        """
        The rate frames are actually arriving at, or the camera's nominal one
        until enough have.

        Worth the arithmetic because the nominal figure is not just imprecise
        but wrong: this webcam reports CAP_PROP_FPS 30.0 while delivering
        15.9, auto-exposure having doubled the frame time in indoor light.
        Publishing 30 would misreport the camera by a factor of two, and hide
        the one thing about the rate a client can still influence -- shorten
        the exposure and the rate comes back up.
        """
        if len(self._arrivals) < 2:
            return None
        span = self._arrivals[-1] - self._arrivals[0]
        if span <= 0:
            return None
        return (len(self._arrivals) - 1) / span

    def get_camera_settings(self):
        out = {
            "ExposureTime": self.cap.get(cv2.CAP_PROP_EXPOSURE) * self.exposure_scale,
            "GainRaw": int(self.cap.get(cv2.CAP_PROP_GAIN)),
        }
        # Left at whatever was last measured when the stream is stopped,
        # rather than reset. A stopped camera has no rate, and the last one
        # it ran at is the more useful answer than either zero or the
        # nominal figure.
        rate = self.measured_frame_rate()
        if rate is not None:
            out["AcquisitionFrameRate"] = rate
        return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenCV -> GigE camera")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--name", default="",
                        help="user-defined camera name. Clients show this and "
                             "can select on it, so it is how you tell two "
                             "otherwise identical cameras apart")
    parser.add_argument("--vendor", default="py-gige-emulator",
                        help="vendor name, the first part of the device id. "
                             "A few real vendor names are refused because "
                             "clients apply per-vendor workarounds to them")
    parser.add_argument("--model", default="OpenCV",
                        help="model name. With the vendor and serial this "
                             "forms the device id a client lists, so two "
                             "otherwise identical cameras need different "
                             "ones here or in --serial")
    parser.add_argument("--serial", default="CV-0001",
                        help="serial number. Part of the device id, and the "
                             "usual thing to vary between two of the same "
                             "camera; some clients pin it in their config")
    parser.add_argument("--device", type=int, default=0,
                        help="OpenCV device index")
    parser.add_argument("--list-modes", action="store_true",
                        help="probe and print the sizes this camera supports, "
                             "then exit")
    parser.add_argument("--mode", default=None,
                        help="frame size as WIDTHxHEIGHT, e.g. 1280x720")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--pixel-format", default="Mono8",
                        choices=["Mono8", "RGB8"])
    parser.add_argument("--heartbeat-timeout", type=int, default=3000,
                        metavar="MS",
                        help="how long the device waits for a client to say "
                             "something before releasing control. Any command "
                             "counts, so raise this only for a client that "
                             "goes quiet for a long time, not merely a slow "
                             "one")
    parser.add_argument("--packet-size", type=int, default=1400)
    parser.add_argument("--exposure-scale", type=float,
                        default=DEFAULT_EXPOSURE_SCALE,
                        help="microseconds per unit of CAP_PROP_EXPOSURE")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    if args.list_modes:
        print("device %d supports:" % args.device)
        for width, height in probe_modes(args.device):
            print("  %dx%d" % (width, height))
        print("\nSelect one with --mode WIDTHxHEIGHT. There is no standard "
              "GigE Vision\nway to offer these as a choice to the client, so "
              "the mode is fixed here\nat startup.")
        sys.exit(0)

    # Check the interface before opening the camera, so a typo fails with a
    # readable message rather than a traceback after the hardware is live.
    try:
        netif.interface_info(args.interface)
    except netif.InterfaceError as e:
        parser.error(str(e))

    width, height = args.width, args.height
    if args.mode is not None:
        try:
            width, height = (int(v) for v in args.mode.lower().split("x"))
        except ValueError:
            parser.error("--mode wants WIDTHxHEIGHT, e.g. 1280x720")

    camera = OpenCvCamera(device=args.device, width=width, height=height,
                          pixel_format=args.pixel_format,
                          exposure_scale=args.exposure_scale)

    try:
        server = GigECameraServer(camera, interface=args.interface,
                                  vendor_name=args.vendor, model_name=args.model,
                                  serial_number=args.serial,
                                  user_defined_name=args.name,
                                  packet_size=args.packet_size,
                              heartbeat_timeout_ms=args.heartbeat_timeout)
    except ValueError as e:
        parser.error(str(e))
    print("serving %dx%d %s, ctrl-c to exit."
          % (camera.settings["Width"], camera.settings["Height"],
             camera.settings["PixelFormat"]))
    try:
        server.serve_forever()
    finally:
        camera.close()
        print("stats:", server.stats, "grab failures:", camera.n_grab_failures)
