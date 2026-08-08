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
import logging
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from gige_emulator import (EmulatedCamera, FloatFeature, GigECameraServer,
                           IntFeature)

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
        IntFeature("GainRaw", "Analog gain", "AnalogControl", "RW",
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

        super().__init__(width=actual_width, height=actual_height,
                         pixel_format=pixel_format,
                         pixel_formats=["Mono8", "RGB8"],
                         frame_rate=frame_rate, **kwds)

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    # --- the three hooks -------------------------------------------------

    def next_frame(self):
        """
        cap.read() blocks until the sensor has a frame, so this is also what
        paces the stream.
        """
        ok, frame = self.cap.read()
        if not ok:
            self.n_grab_failures += 1
            return None      # the stream thread simply tries again

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

        if "AcquisitionFrameRate" in changed:
            # The rate belongs to the camera, not to a timer in the emulator
            # -- cap.read() blocking at whatever rate the camera settles on
            # is what paces the stream.
            rate = float(changed["AcquisitionFrameRate"])
            if not self.cap.set(cv2.CAP_PROP_FPS, rate):
                log.warning("camera refused frame rate %g", rate)

    def get_camera_settings(self):
        return {
            "ExposureTime": self.cap.get(cv2.CAP_PROP_EXPOSURE) * self.exposure_scale,
            "GainRaw": int(self.cap.get(cv2.CAP_PROP_GAIN)),
        }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenCV -> GigE camera")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
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

    width, height = args.width, args.height
    if args.mode is not None:
        try:
            width, height = (int(v) for v in args.mode.lower().split("x"))
        except ValueError:
            parser.error("--mode wants WIDTHxHEIGHT, e.g. 1280x720")

    camera = OpenCvCamera(device=args.device, width=width, height=height,
                          pixel_format=args.pixel_format,
                          exposure_scale=args.exposure_scale)

    server = GigECameraServer(camera, interface=args.interface,
                              model_name="OpenCV", serial_number="CV-0001",
                              packet_size=args.packet_size)
    print("serving %dx%d %s, ctrl-c to exit."
          % (camera.settings["Width"], camera.settings["Height"],
             camera.settings["PixelFormat"]))
    try:
        server.serve_forever()
    finally:
        camera.close()
        print("stats:", server.stats, "grab failures:", camera.n_grab_failures)
