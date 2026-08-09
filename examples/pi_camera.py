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
# FULL RESOLUTION NEEDS A PACKET DELAY, or it fails completely rather than
# degrading. A 4056x3040 frame is 37 MB as RGB8 and 24.7 MB as Bayer, or
# 27,120 and 16,753 packets, and the emulator sends a frame as one
# uninterrupted burst. Once that burst is larger than the client's socket
# buffer it cannot drain fast enough, and with packet resend not advertised a
# single lost packet costs the whole frame. Measured against Aravis with
# rmem_max at 16.8 MB: every full resolution frame failed, while 2028x1520 at
# 9.2 MB completed 126 of 126 with no missing packets at all.
#
# Pacing the burst fixes it and needs no root on either side:
#
#   arv-camera-test-0.8 -n <name> -a -m 5000 -y 20000
#
# which took the same 37 MB frame from 0 completed to 25 completed with zero
# missing packets, at about 1 fps -- the 20 us delay costs 0.54 s per frame,
# so tune it down until frames start failing. Raising rmem_max past the frame
# size works too, but 20 MB is not enough for a 12 MPix sensor.
#
# Jumbo frames help by cutting the packet count, if every hop supports them:
#
#   sudo ip link set eth0 mtu 9000          (on the Pi and the client)
#   python examples/pi_camera.py --interface eth0 --packet-size 8000
#

import argparse
import logging
import math
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
                           GigECameraServer, IntFeature, netif)
from gige_emulator import constants as c

log = logging.getLogger("pi_camera")

# The IMX477's analogue gain register saturates at 1024/(1024-978), so
# anything above this is digital gain, which amplifies read noise rather than
# signal. Bounding the feature here means a client cannot ask for it.
IMX477_MAX_ANALOGUE_GAIN = 1024.0 / (1024.0 - 978.0)


def gain_to_db(linear):
    """
    libcamera's AnalogueGain is a linear multiplier; the naming convention's
    Gain feature is in dB.

    Sensor gain is a signal amplitude ratio, so the factor is 20 and not 10.
    Getting that wrong halves every number and still looks entirely
    plausible, which is why it is spelled out here rather than inlined.
    """
    return 20.0 * math.log10(max(linear, 1e-6))


def db_to_gain(db):
    return 10.0 ** (db / 20.0)


IMX477_MAX_GAIN_DB = gain_to_db(IMX477_MAX_ANALOGUE_GAIN)

# This is a colour sensor. Every mode it offers is Bayer -- there is no mono
# mode at all -- so serving the raw stream as Mono is not an approximation,
# it hands the client a mosaic labelled as greyscale. A capture at 2028x1520
# splits by 2x2 phase into means of 9565 / 25297 / 25378 / 17283: the two
# equal ones are the greens, and the spread is 82% of the mean.
#
# The phase is read from the configuration rather than assumed. The sensor is
# physically RGGB, but this camera reports Rotation: 180 and libcamera hands
# back SBGGR16 -- assuming the sensor's native phase would swap the client's
# red and blue. libcamera also treats the requested format as a hint: asking
# for SRGGB12, SRGGB10 or SRGGB8 all return SBGGR16 on a Pi 5, so what was
# asked for says nothing about what arrives.
LIBCAMERA_BAYER_TO_GENICAM = {
    "SBGGR8": "BayerBG8", "SGBRG8": "BayerGB8",
    "SGRBG8": "BayerGR8", "SRGGB8": "BayerRG8",
    "SBGGR16": "BayerBG16", "SGBRG16": "BayerGB16",
    "SGRBG16": "BayerGR16", "SRGGB16": "BayerRG16",
}

# libcamera's three-byte format names describe the word, not the byte order,
# so they read backwards: "BGR888" is R,G,B in memory and is what GenICam
# calls RGB8. Verified on an IMX477 by checking which byte tracked the raw
# Bayer red plane, not taken from the documentation.
LIBCAMERA_RGB8 = "BGR888"

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


def mode_label(mode):
    return "%dx%d/%s" % (mode["size"][0], mode["size"][1],
                         str(mode.get("format")))


def unpacked_format(name):
    """
    The unpacked spelling of a sensor format.

    The sensor advertises its 10 and 12 bit modes only as _CSI2P, and asking
    for those on a Pi 5 returns BGGR_PISP_COMP1 -- a PiSP-compressed layout
    with no GenICam equivalent, so the raw Bayer option disappears and only
    RGB8 is left. Asking for the unpacked name instead selects the *same*
    sensor readout and delivers SBGGR16, which does have one: requesting
    SRGGB12 measured 101.8 fps against the 101.68 the packed mode
    advertises. So the depth is kept and the format stays labelable.
    """
    # str() because picamera2 hands back a SensorFormat object here, not a
    # plain string, and it has no string methods.
    name = str(name)
    return name[:-len("_CSI2P")] if name.endswith("_CSI2P") else name


def parse_mode(text, sensor_modes):
    """
    Resolve --mode against the sensor's own mode list.

    The format is part of the selection, not decoration. libcamera picks the
    sensor readout from it, and on an IMX477 at 1332x990 that is 147.8 fps
    for SRGGB8 against 101.8 for SRGGB12 -- measured, and matching the fps
    the mode list advertises. Taking only the size threw that away and always
    ran at the slowest depth.

    Accepts WIDTHxHEIGHT, which picks the deepest mode of that size, or
    WIDTHxHEIGHT/FORMAT for an exact one.
    """
    size, _, wanted = text.partition("/")
    try:
        width, height = (int(v) for v in size.lower().split("x"))
    except ValueError:
        raise ValueError("expected WIDTHxHEIGHT or WIDTHxHEIGHT/FORMAT, "
                         "got %r" % text)

    matches = [m for m in sensor_modes if tuple(m["size"]) == (width, height)]
    if not matches:
        raise ValueError("no %dx%d mode; this sensor offers %s"
                         % (width, height,
                            ", ".join(sorted({mode_label(m)
                                              for m in sensor_modes}))))
    if wanted:
        exact = [m for m in matches
                 if str(m.get("format", "")).lower() == wanted.lower()]
        if not exact:
            raise ValueError("no %s mode at %dx%d; that size offers %s"
                             % (wanted, width, height,
                                ", ".join(mode_label(m) for m in matches)))
        return exact[0]

    # Deepest by default: a client can always ask for less precision, but it
    # cannot recover what a shallower readout threw away.
    return max(matches, key=lambda m: m.get("bit_depth") or 0)


class _FrameSink(Output):
    """
    Picamera2 hands encoded frames here. We keep only the most recent one --
    a stream that has fallen behind wants the newest frame, not a backlog.
    """

    def __init__(self, width, height, bytes_per_pixel, **kwds):
        super().__init__(**kwds)
        self.width = width
        self.height = height
        self.row_bytes = width * bytes_per_pixel
        self.condition = threading.Condition()
        self.frame = None
        self.frame_count = 0
        self.timestamp_ns = 0
        self.n_superseded = 0

    def outputframe(self, frame, keyframe=True, timestamp=None, packet=None,
                    audio=False):
        # The buffer is padded to the stream's stride, so it is wider than
        # the image. Reshaping to the real row length and keeping the first
        # row_bytes drops the padding. That width follows the pixel format --
        # three bytes for RGB8, two for 16 bit Bayer -- and hardcoding two
        # silently truncated or overran every other format.
        image = np.frombuffer(frame, dtype=np.uint8)
        image = np.reshape(image, (self.height, -1))[:, :self.row_bytes]

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
        FloatFeature("Gain", "Analogue gain", "AnalogControl", "RW",
                     default=0.0, min=0.0, max=IMX477_MAX_GAIN_DB,
                     unit="dB"),
        IntFeature("BinningHorizontal", "Horizontal binning (see the note "
                   "at the top of this file -- it selects a smaller stream "
                   "size rather than binning the sensor)",
                   "ImageFormatControl", "RW", affects_payload=True,
                   default=1, min=1, max=MAX_BINNING),
        IntFeature("BinningVertical", "Vertical binning; always follows "
                   "BinningHorizontal", "ImageFormatControl", "RW",
                   affects_payload=True, default=1, min=1, max=MAX_BINNING),
    )

    #: --pixel-format aliases. The Bayer layout's name depends on the
    #: sensor's rotation and is only known once the camera is open, so
    #: "raw" is the only way to ask for it from a command line without
    #: running --list-modes first and reading it off.
    PIXEL_FORMAT_ALIASES = {"rgb": "RGB8", "raw": None, "bayer": None}

    def __init__(self, width=4056, height=3040, frame_rate=10.0,
                 raw_format="SRGGB12", pixel_format="RGB8", **kwds):

        self.picam2 = Picamera2()
        self.full_width = width
        self.full_height = height
        self.raw_format = raw_format

        self.binning_factors = binning_factors(self.picam2.sensor_modes,
                                               (width, height))
        log.info("usable binning factors for %dx%d: %s",
                 width, height, self.binning_factors)

        # Configure the raw stream once to find out what the pipeline
        # actually delivers, since the requested format is only a hint.
        raw_actual = self._configure_raw(width, height)
        self.bayer_format = LIBCAMERA_BAYER_TO_GENICAM.get(raw_actual["format"])
        if self.bayer_format is None:
            log.warning("raw stream is %r, which is not a Bayer layout this "
                        "example can label; offering RGB8 only",
                        raw_actual["format"])
        else:
            log.info("raw stream is %s -> %s", raw_actual["format"],
                     self.bayer_format)

        # RGB8 first, so it is the default. The ISP demosaics with the
        # sensor's own tuning file, which is a better picture than a client
        # will reconstruct, and it is what someone pointing a viewer at a
        # colour camera expects to see. Raw Bayer stays available for anyone
        # doing their own processing.
        formats = ["RGB8"] + ([self.bayer_format] if self.bayer_format else [])

        if pixel_format.lower() in self.PIXEL_FORMAT_ALIASES:
            # "raw" resolves to whatever layout this sensor turned out to
            # have, which is the point of the alias.
            resolved = self.PIXEL_FORMAT_ALIASES[pixel_format.lower()]
            pixel_format = resolved or self.bayer_format
            if pixel_format is None:
                raise ValueError(
                    "this pipeline delivers %r, which has no GenICam Bayer "
                    "equivalent, so only RGB8 is available"
                    % str(raw_actual["format"]))
        if pixel_format not in formats:
            raise ValueError("unknown pixel format %r; this camera offers %s"
                             % (pixel_format, ", ".join(formats)))

        self._applied_pixel_format = None
        self._configure(width, height, pixel_format)

        super().__init__(width=width, height=height,
                         pixel_format=pixel_format,
                         pixel_formats=formats, frame_rate=frame_rate,
                         **kwds)

        # Narrow the declared maximum to what this sensor can actually do, so
        # a client cannot select a factor with no matching mode.
        for name in ("BinningHorizontal", "BinningVertical"):
            self.feature_set.by_name[name].max = max(self.binning_factors)

        self.sink = _FrameSink(width, height, self._bytes_per_pixel())
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

    def _configure_raw(self, width, height):
        config = self.picam2.create_video_configuration(
            raw={"format": self.raw_format, "size": (width, height)})
        self.picam2.configure(config)
        return self.picam2.camera_configuration()["raw"]

    def _configure(self, width, height, pixel_format):
        """
        Point the encoder at whichever stream that format comes from.

        RGB8 is the ISP's processed output; anything else is the sensor's raw
        Bayer. They are different streams, not different encodings of one, so
        switching format means reconfiguring rather than reinterpreting.
        """
        if pixel_format == "RGB8":
            config = self.picam2.create_video_configuration(
                main={"format": LIBCAMERA_RGB8, "size": (width, height)})
            stream = "main"
        else:
            config = self.picam2.create_video_configuration(
                raw={"format": self.raw_format, "size": (width, height)})
            stream = "raw"
        self.picam2.configure(config)
        self.picam2.encode_stream_name = stream
        self._applied_pixel_format = pixel_format
        return self.picam2.camera_configuration()[stream]

    def start(self):
        self.picam2.start()
        self._apply_controls()
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

    def _apply_controls(self):
        """
        Push the current settings at libcamera.

        Controls do not survive a stop/reconfigure/start cycle, so this runs
        again after every restart. Missing it is not subtle once you know
        where to look but is invisible from the outside: a binning or pixel
        format change silently reverts to auto exposure and a free running
        sensor. On the raw stream that is 23.5 fps against the 5 asked for,
        and the stream thread then packetises ~100k packets a second on a
        Pi, starving the GVCP thread until the client's heartbeat expires --
        which looks like the camera refusing to stream rather than like a
        lost control.

        AeEnable goes off first, or the auto-exposure algorithm fights every
        exposure the client sets.
        """
        self.picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": int(self.settings["ExposureTime"]),
            "AnalogueGain": db_to_gain(self.settings["Gain"]),
            "FrameRate": float(self.settings["AcquisitionFrameRate"]),
        })

    def _bytes_per_pixel(self):
        return c.bits_per_pixel(self.pixel_format_value()) // 8

    def _apply_pixel_format(self, name):
        """
        Switch between the ISP stream and the raw one.

        Only reachable while stopped, because PixelFormat affects the payload
        and the emulator refuses those writes during acquisition.
        """
        width = self.settings["Width"]
        height = self.settings["Height"]

        was_started = self.started
        if was_started:
            self.picam2.stop_encoder()
            self.picam2.stop()
            self.started = False

        actual = self._configure(width, height, name)
        self.sink = _FrameSink(width, height, self._bytes_per_pixel())

        if was_started:
            self.start()
        log.info("pixel format -> %s (libcamera %s, stride %d)",
                 name, actual["format"], actual["stride"])

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

        self._configure(width, height, self.settings["PixelFormat"])
        self.sink = _FrameSink(width, height, self._bytes_per_pixel())

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

        if "Gain" in changed:
            controls["AnalogueGain"] = db_to_gain(changed["Gain"])

        # Same trap as binning: self.settings is updated before this hook
        # runs, so the requested value has to be compared against what the
        # camera is actually configured for.
        if ("PixelFormat" in changed
                and changed["PixelFormat"] != self._applied_pixel_format):
            self._apply_pixel_format(changed["PixelFormat"])

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
            # Reported as a float now rather than rounded to a whole
            # multiplier, which used to throw away most of the sensor's
            # resolution between 1x and 2x.
            out["Gain"] = gain_to_db(float(metadata["AnalogueGain"]))
        return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Raspberry Pi -> GigE camera")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--name", default="",
                        help="user-defined camera name. Clients show this and "
                             "can select on it, so it is how you tell two "
                             "otherwise identical cameras apart")
    parser.add_argument("--list-modes", action="store_true",
                        help="print the sensor modes libcamera reports, "
                             "then exit")
    parser.add_argument("--mode", default=None,
                        help="frame size as WIDTHxHEIGHT, e.g. 2028x1520")
    parser.add_argument("--width", type=int, default=4056)
    parser.add_argument("--height", type=int, default=3040)
    parser.add_argument("--pixel-format", default="RGB8",
                        help="what the client receives: RGB8 (the ISP's "
                             "demosaiced output), or 'raw' for this sensor's "
                             "Bayer layout, whose exact name depends on the "
                             "rotation. Not the same thing as --mode, which "
                             "picks the sensor readout")
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
        print("sensor modes (pass one of these to --mode):")
        for mode in picam2.sensor_modes:
            print("  %-26s bit_depth=%-3s max_fps=%s"
                  % (mode_label(mode), mode.get("bit_depth"),
                     mode.get("fps")))
        picam2.close()
        print("\nThe format is not decoration -- it picks the sensor readout, "
              "and the\nshallower ones are markedly faster. Giving --mode a "
              "bare WIDTHxHEIGHT\nselects the deepest mode of that size.")
        print("\nThese SRGGB names are sensor formats, and they are NOT what "
              "the client\nreceives. That is --pixel-format, which takes "
              "RGB8 (the default, the ISP's\ndemosaiced output) or 'raw' for "
              "this sensor's Bayer layout. The two are\nindependent: --mode "
              "chooses how the sensor is read, --pixel-format chooses\nwhat "
              "is sent.")
        print("\nA client can also pick a binned size at runtime, and switch "
              "pixel format\nwhile stopped; see the note at the top of this "
              "file about what binning\nreally means here.")
        sys.exit(0)

    # Check the interface before opening the camera, so a typo fails with a
    # readable message rather than a traceback after the sensor is live.
    try:
        netif.interface_info(args.interface)
    except netif.InterfaceError as e:
        parser.error(str(e))

    width, height = args.width, args.height
    raw_format = None
    if args.mode is not None:
        probe = Picamera2()
        try:
            chosen = parse_mode(args.mode, probe.sensor_modes)
        except ValueError as e:
            parser.error("--mode: %s" % e)
        finally:
            probe.close()
        width, height = chosen["size"]
        raw_format = unpacked_format(chosen["format"])
        log.info("mode %s -> requesting %s (bit_depth %s, sensor max %s fps)",
                 mode_label(chosen), raw_format, chosen.get("bit_depth"),
                 chosen.get("fps"))

    kwds = {"raw_format": raw_format} if raw_format else {}
    try:
        camera = PiCamera(width=width, height=height,
                          frame_rate=args.frame_rate,
                          pixel_format=args.pixel_format, **kwds)
    except ValueError as e:
        parser.error("--pixel-format: %s" % e)
    camera.start()

    server = GigECameraServer(camera, interface=args.interface,
                              model_name="PiHQ", serial_number="PI-0001",
                              user_defined_name=args.name,
                              packet_size=args.packet_size)
    print("serving %dx%d %s (%.1f MB/frame), ctrl-c to exit.\n"
          "pixel formats offered: %s"
          % (camera.settings["Width"], camera.settings["Height"],
             camera.settings["PixelFormat"], camera.payload_size() / 1e6,
             ", ".join(camera.feature_set.by_name["PixelFormat"].entries)))
    try:
        server.serve_forever()
    finally:
        camera.close()
        print("stats:", server.stats,
              "superseded frames:", camera.sink.n_superseded)
