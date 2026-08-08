#
# A camera that generates random noise. No hardware needed, and it is what
# the emulator is tested against.
#
#   python examples/noise_camera.py --interface enxf8e43b0b54db
#

import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from gige_emulator import (EmulatedCamera, FloatFeature, GigECameraServer,
                           IntFeature)


class NoiseCamera(EmulatedCamera):

    extra_features = (
        FloatFeature("ExposureTime", "Exposure time",
                     "AcquisitionControl", "RW",
                     default=10000.0, min=1.0, max=1e6, unit="us"),
        IntFeature("GainRaw", "Analog gain", "AnalogControl", "RW",
                   default=1, min=1, max=22),
    )

    def __init__(self, **kwds):
        super().__init__(**kwds)
        self._noise = None

    def next_frame(self):
        size = self.geometry["payload"]
        # os.urandom is slower than numpy but keeps the example dependency
        # free; a real camera would return its own buffer here anyway.
        if self._noise is None or len(self._noise) != size * 2:
            self._noise = os.urandom(size * 2)
        offset = int.from_bytes(os.urandom(2), "big") % size
        return self._noise[offset:offset + size]

    def set_camera_settings(self, changed):
        print("client changed:", changed)

    def get_camera_settings(self):
        return {}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Noise -> GigE camera")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--pixel-format", default="Mono8")
    parser.add_argument("--frame-rate", type=float, default=10.0)
    parser.add_argument("--packet-size", type=int, default=1400)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    camera = NoiseCamera(width=args.width, height=args.height,
                         pixel_format=args.pixel_format,
                         pixel_formats=["Mono8", "Mono16"],
                         frame_rate=args.frame_rate)

    server = GigECameraServer(camera, interface=args.interface,
                              model_name="PyNoise", serial_number="PY-0001",
                              packet_size=args.packet_size)
    print("ctrl-c to exit.")
    try:
        server.serve_forever()
    finally:
        print("stats:", server.stats)
