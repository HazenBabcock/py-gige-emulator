#
# Serve a Raspberry Pi camera as a GigE Vision camera.
#
# Written for the HQ camera (Sony IMX477) at full resolution, 4056x3040
# Mono16, via libcamera/Picamera2.
#
#   python examples/pi_camera.py --interface eth0
#
# NOTE: this has to run on a Pi, so unlike the OpenCV example it has not been
# exercised on the machine this was developed on. The frame handling is
# ported from a working Aravis-based bridge; the parts that are new here are
# the settings hooks.
#
# A full resolution frame is 24.7 MB, which is about 18,000 packets at the
# default 1400 byte packet size and more than a gigabit link can carry at
# 10 fps. Use jumbo frames on both ends if you can:
#
#   sudo ip link set eth0 mtu 9000          (on the Pi and the client)
#   python examples/pi_camera.py --interface eth0 --packet-size 8000
#
# and raise the receive buffer on the client:
#
#   sudo sysctl -w net.core.rmem_max=20000000
#

import argparse
import logging
import os
import sys
import threading
import time

import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import Encoder
from picamera2.outputs import Output

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from gige_emulator import (EmulatedCamera, Frame, FloatFeature,
                           GigECameraServer, IntFeature)

log = logging.getLogger("pi_camera")

# The IMX477's analogue gain register saturates at 1024/(1024-978), so
# anything above this is digital gain, which amplifies read noise rather than
# signal. Bounding the feature here means a client cannot ask for it.
IMX477_MAX_ANALOGUE_GAIN = 22

# Binning here is not binning.
#
# GenICam has no standard way to offer a client a list of discrete sensor
# modes, and neither arv-viewer nor the micro-manager Aravis adapter renders
# a custom enumeration -- both build fixed control panels. Binning is the one
# discrete control they both do render, so it is what this example uses to
# let a client pick a smaller frame.
#
# What actually happens is a change of stream size. It is only defensible
# because the IMX477's modes happen to be even divisions of the full sensor
# (4056x3040, 2028x1520, 1014x760), so "binning 2" and "half the width and
# height" land on the same numbers. Whether libcamera services that by
# binning the sensor, by skipping lines, or by scaling is its business, not
# ours -- so the pixel values are not the sum or average of a 2x2 block the
# way real binning would give. Do not rely on the photometry.
#
# The axes are also deliberately coupled: writing either BinningHorizontal or
# BinningVertical sets both, because there is one stream size and no way to
# halve only its width. Coercing a written value is legal in GenICam and
# costs nothing with real clients -- micro-manager only ever calls
# arv_camera_set_binning(cam, n, n) with matching axes, and arv-viewer's two
# spin buttons simply see the other one follow along.
#
# The usable factors are worked out from the sensor's own mode list rather
# than assumed. On an IMX477 that yields (1, 2) and nothing more: the modes
# are 4056x3040, 2028x1520, 2028x1080, 1332x990 and 4056x2160, and only
# 2028x1520 is an exact halving of both axes. A hardcoded (1, 2, 4) asks for
# 1014x760, which the sensor does not offer -- libcamera then silently
# substitutes a nearby mode and every frame comes out the wrong length.
MAX_BINNING = 8


def binning_factors(sensor_modes, full_size):
    """
    Factors n where the sensor really has a full_size/n mode on both axes.
    """
    sizes = {tuple(mode["size"]) for mode in sensor_modes}
    full_width, full_height = full_size
    factors = []
    for n in range(1, MAX_BINNING + 1):
        if full_width % n or full_height % n:
            continue
        if (full_width // n, full_height // n) in sizes:
            factors.append(n)
    return factors or [1]


class _FrameSink(Output):
    """
    Picamera2 hands encoded frames here. We keep only the most recent one --
    a stream that has fallen behind wants the newest frame, not a backlog.
    """

    def __init__(self, width, height, **kwds):
        super().__init__(**kwds)
        self.width = width
        self.height = height
        self.condition = threading.Condition()
        self.frame = None
        self.frame_count = 0
        self.timestamp_ns = 0
        self.n_superseded = 0

    def outputframe(self, frame, keyframe=True, timestamp=None, packet=None,
                    audio=False):
        # The raw buffer is padded to the sensor's stride, so it is wider
        # than the image. Reshaping to the real row length and slicing the
        # first 2*width bytes drops the padding; 2 because Mono16.
        image = np.frombuffer(frame, dtype=np.uint8)
        image = np.reshape(image, (self.height, -1))[:, :2 * self.width]

        with self.condition:
            if self.frame is not None:
                self.n_superseded += 1
            # tobytes() rather than .data: the slice above is not contiguous,
            # so .data would hand back the padded rows.
            self.frame = image.tobytes()
            self.frame_count += 1
            # picamera2's timestamp is int microseconds, rebased so the first
            # frame of the encoder run is zero.
            self.timestamp_ns = (timestamp or 0) * 1000
            self.condition.notify()

    def take(self, timeout=1.0):
        with self.condition:
            if self.frame is None:
                if not self.condition.wait(timeout):
                    return None
                if self.frame is None:
                    return None
            frame, self.frame = self.frame, None
            return frame, self.frame_count, self.timestamp_ns


class PiCamera(EmulatedCamera):

    extra_features = (
        FloatFeature("ExposureTime", "Exposure time", "AcquisitionControl",
                     "RW", default=90000.0, min=100.0, max=1e7, unit="us"),
        IntFeature("GainRaw", "Analogue gain", "AnalogControl", "RW",
                   default=1, min=1, max=IMX477_MAX_ANALOGUE_GAIN),
        IntFeature("BinningHorizontal", "Horizontal binning (see the note "
                   "at the top of this file -- it selects a smaller stream "
                   "size rather than binning the sensor)",
                   "ImageFormatControl", "RW", affects_payload=True,
                   default=1, min=1, max=MAX_BINNING),
        IntFeature("BinningVertical", "Vertical binning; always follows "
                   "BinningHorizontal", "ImageFormatControl", "RW",
                   affects_payload=True, default=1, min=1, max=MAX_BINNING),
    )

    def __init__(self, width=4056, height=3040, frame_rate=10.0,
                 raw_format="SRGGB12", **kwds):

        self.picam2 = Picamera2()
        self.full_width = width
        self.full_height = height
        self.raw_format = raw_format

        self.binning_factors = binning_factors(self.picam2.sensor_modes,
                                               (width, height))
        log.info("usable binning factors for %dx%d: %s",
                 width, height, self.binning_factors)

        self._configure(width, height)

        super().__init__(width=width, height=height, pixel_format="Mono16",
                         pixel_formats=["Mono16"], frame_rate=frame_rate,
                         **kwds)

        # Narrow the declared maximum to what this sensor can actually do, so
        # a client cannot select a factor with no matching mode.
        for name in ("BinningHorizontal", "BinningVertical"):
            self.feature_set.by_name[name].max = max(self.binning_factors)

        # AeEnable has to go off or the auto-exposure algorithm fights every
        # exposure the client sets.
        self.picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": int(self.settings["ExposureTime"]),
            "AnalogueGain": float(self.settings["GainRaw"]),
            "FrameRate": frame_rate,
        })

        self.sink = _FrameSink(width, height)
        self.encoder = Encoder()
        self.started = False

        # picam2.capture_metadata() blocks until the next frame completes,
        # which at 4056x3040 is ~85 ms and in practice sometimes far longer.
        # get_camera_settings() runs inline on the control thread, where the
        # client allows a command 500 ms before it retries and 5 retries
        # before it gives up -- so calling it there makes every feature read
        # time out. A background refresh keeps the hook to a dict lookup.
        self._metadata = {}
        self._metadata_lock = threading.Lock()
        self._metadata_thread = None

        # What the camera is actually configured for, tracked separately from
        # settings["BinningHorizontal"]. The emulator updates self.settings
        # *before* it calls set_camera_settings, so comparing the requested
        # value against the settings dict there always says "unchanged" and
        # the reconfigure silently never happens.
        self._applied_binning = 1

    def _configure(self, width, height):
        config = self.picam2.create_video_configuration(
            raw={"format": self.raw_format, "size": (width, height)})
        self.picam2.configure(config)
        self.picam2.encode_stream_name = "raw"

    def start(self):
        self.picam2.start()
        self.picam2.start_encoder(self.encoder, self.sink)
        self.started = True
        if self._metadata_thread is None:
            self._metadata_thread = threading.Thread(
                target=self._refresh_metadata, name="pi-metadata", daemon=True)
            self._metadata_thread.start()

    def close(self):
        self.started = False
        if self.picam2 is not None:
            try:
                self.picam2.stop_encoder()
                self.picam2.stop()
            except Exception:
                log.exception("error stopping the camera")

    def _refresh_metadata(self):
        while True:
            if not self.started:
                time.sleep(0.2)
                continue
            try:
                metadata = self.picam2.capture_metadata()
            except Exception:
                time.sleep(0.5)
                continue
            with self._metadata_lock:
                self._metadata = metadata
            # No need to keep up with the frame rate; the client only reads
            # these when a user is looking at them.
            time.sleep(0.5)

    def _apply_binning(self, factor):
        """
        Reconfigure to a smaller stream. Only reachable when acquisition is
        stopped -- the emulator refuses a payload-affecting write while
        streaming, because the client sized its buffers at AcquisitionStart.
        """
        width = self.full_width // factor
        height = self.full_height // factor

        was_started = self.started
        if was_started:
            self.picam2.stop_encoder()
            self.picam2.stop()
            self.started = False

        self._configure(width, height)
        self.sink = _FrameSink(width, height)

        self._applied_binning = factor
        self.settings["BinningHorizontal"] = factor
        self.settings["BinningVertical"] = factor
        self.settings["Width"] = width
        self.settings["Height"] = height

        if was_started:
            self.start()
        log.info("binning %d -> %dx%d", factor, width, height)

    # --- the three hooks -------------------------------------------------

    def next_frame(self):
        """
        Blocks until the sensor delivers a frame, which is also the pacing.
        """
        taken = self.sink.take(timeout=1.0)
        if taken is None:
            return None
        data, count, timestamp_ns = taken
        # Passing the sensor's own frame number through means a gap in the
        # client's frame ids is a frame the pipeline actually dropped, rather
        # than just one this bridge did not get to.
        return Frame(data=data, timestamp_ns=timestamp_ns, frame_id=count)

    def set_camera_settings(self, changed):
        controls = {}

        # Both axes move together; see the note at the top of this file.
        binning = changed.get("BinningHorizontal",
                              changed.get("BinningVertical"))
        if binning is not None:
            if binning not in self.binning_factors:
                raise ValueError("this sensor supports binning %s, not %r"
                                 % (self.binning_factors, binning))
            if binning != self._applied_binning:
                self._apply_binning(binning)

        if "ExposureTime" in changed:
            exposure_us = int(changed["ExposureTime"])
            controls["ExposureTime"] = exposure_us
            # A frame cannot be shorter than its exposure, so a long exposure
            # has to drag the frame rate down with it or libcamera silently
            # clamps the exposure instead.
            rate = self.settings.get("AcquisitionFrameRate", 10.0)
            if exposure_us > 0 and rate > 1e6 / exposure_us:
                controls["FrameRate"] = max(0.1, 1e6 / exposure_us)

        if "GainRaw" in changed:
            controls["AnalogueGain"] = float(changed["GainRaw"])

        if "AcquisitionFrameRate" in changed:
            controls["FrameRate"] = float(changed["AcquisitionFrameRate"])

        if controls:
            log.info("applying %s", controls)
            self.picam2.set_controls(controls)

    def get_camera_settings(self):
        # A dict lookup, deliberately. See the note by _metadata above: this
        # runs inline on the control thread and must not block.
        with self._metadata_lock:
            metadata = dict(self._metadata)
        out = {
            # Reported so a client that set one axis sees the other follow.
            "BinningHorizontal": self.settings["BinningHorizontal"],
            "BinningVertical": self.settings["BinningVertical"],
        }
        if "ExposureTime" in metadata:
            out["ExposureTime"] = float(metadata["ExposureTime"])
        if "AnalogueGain" in metadata:
            out["GainRaw"] = int(round(metadata["AnalogueGain"]))
        return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Raspberry Pi -> GigE camera")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--list-modes", action="store_true",
                        help="print the sensor modes libcamera reports, "
                             "then exit")
    parser.add_argument("--mode", default=None,
                        help="frame size as WIDTHxHEIGHT, e.g. 2028x1520")
    parser.add_argument("--width", type=int, default=4056)
    parser.add_argument("--height", type=int, default=3040)
    parser.add_argument("--frame-rate", type=float, default=10.0)
    parser.add_argument("--packet-size", type=int, default=1400,
                        help="raise this with the MTU if you have jumbo frames")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    if args.list_modes:
        # Unlike OpenCV, libcamera reports its modes directly.
        picam2 = Picamera2()
        print("sensor modes:")
        for mode in picam2.sensor_modes:
            print("  %-12s %-10s bit_depth=%s fps=%s"
                  % ("%dx%d" % mode["size"], mode.get("format"),
                     mode.get("bit_depth"), mode.get("fps")))
        picam2.close()
        print("\nSelect one with --mode WIDTHxHEIGHT, or let a client pick a "
              "binned\nsize at runtime -- see the note at the top of this "
              "file about what\nbinning really means here.")
        sys.exit(0)

    width, height = args.width, args.height
    if args.mode is not None:
        try:
            width, height = (int(v) for v in args.mode.lower().split("x"))
        except ValueError:
            parser.error("--mode wants WIDTHxHEIGHT, e.g. 2028x1520")

    camera = PiCamera(width=width, height=height, frame_rate=args.frame_rate)
    camera.start()

    server = GigECameraServer(camera, interface=args.interface,
                              model_name="PiHQ", serial_number="PI-0001",
                              packet_size=args.packet_size)
    print("serving %dx%d Mono16 (%.1f MB/frame), ctrl-c to exit."
          % (camera.settings["Width"], camera.settings["Height"],
             camera.payload_size() / 1e6))
    try:
        server.serve_forever()
    finally:
        camera.close()
        print("stats:", server.stats,
              "superseded frames:", camera.sink.n_superseded)
