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
    )

    def __init__(self, width=4056, height=3040, frame_rate=10.0,
                 raw_format="SRGGB12", **kwds):

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            raw={"format": raw_format, "size": (width, height)})
        self.picam2.configure(config)
        self.picam2.encode_stream_name = "raw"

        super().__init__(width=width, height=height, pixel_format="Mono16",
                         pixel_formats=["Mono16"], frame_rate=frame_rate,
                         **kwds)

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

    def start(self):
        self.picam2.start()
        self.picam2.start_encoder(self.encoder, self.sink)
        self.started = True

    def close(self):
        if self.started:
            self.picam2.stop_encoder()
            self.picam2.stop()
            self.started = False

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
        try:
            metadata = self.picam2.capture_metadata()
        except Exception:
            log.exception("capture_metadata() failed")
            return {}
        out = {}
        if "ExposureTime" in metadata:
            out["ExposureTime"] = float(metadata["ExposureTime"])
        if "AnalogueGain" in metadata:
            out["GainRaw"] = int(round(metadata["AnalogueGain"]))
        return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Raspberry Pi -> GigE camera")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--width", type=int, default=4056)
    parser.add_argument("--height", type=int, default=3040)
    parser.add_argument("--frame-rate", type=float, default=10.0)
    parser.add_argument("--packet-size", type=int, default=1400,
                        help="raise this with the MTU if you have jumbo frames")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    camera = PiCamera(width=args.width, height=args.height,
                      frame_rate=args.frame_rate)
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
