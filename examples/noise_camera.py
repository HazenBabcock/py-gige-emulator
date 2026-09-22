#
# A camera that generates random noise. No hardware needed, and it is what
# the emulator is tested against.
#
#   python examples/noise_camera.py --interface eth0
#

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from gige_emulator import (EmulatedCamera, FloatFeature, GigECameraServer,
                           netif)


class NoiseCamera(EmulatedCamera):

    extra_features = (
        FloatFeature("ExposureTime", "Exposure time",
                     "AcquisitionControl", "RW",
                     default=10000.0, min=1.0, max=1e6, unit="us"),
        FloatFeature("Gain", "Analog gain", "AnalogControl", "RW",
                     default=0.0, min=0.0, max=24.0, unit="dB"),
    )

    def __init__(self, **kwds):
        super().__init__(**kwds)
        self._noise = None
        self._next_due = 0.0

    def next_frame(self):
        # Unlike a real camera this has no sensor to wait on, so it has to
        # provide its own timing. The stream thread sends frames exactly as
        # fast as next_frame() returns them, and without this it would
        # saturate a core and flood the network.
        #
        # A frame takes at least its exposure, so the frame rate is only a
        # ceiling and the longer of the two intervals wins. Pacing on the
        # rate alone left exposure doing nothing, and a client cannot
        # compensate: the usual way to let exposure set the pace is to clear
        # AcquisitionFrameRateEnable, which this camera does not have.
        interval = (self.settings.get("ExposureTime") or 0.0) * 1e-6
        rate = self.settings.get("AcquisitionFrameRate", 0.0)
        if rate and rate > 0:
            interval = max(interval, 1.0 / rate)
        if interval > 0:
            now = time.monotonic()
            if self._next_due <= 0.0:
                self._next_due = now
            self._next_due = max(self._next_due + interval, now)
            time.sleep(max(0.0, self._next_due - now))

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
    parser.add_argument("--name", default="",
                        help="user-defined camera name. Clients show this and "
                             "can select on it, so it is how you tell two "
                             "otherwise identical cameras apart")
    parser.add_argument("--vendor", default="py-gige-emulator",
                        help="vendor name, the first part of the device id. "
                             "A few real ones are warned about, because "
                             "clients apply per-vendor workarounds keyed to "
                             "them -- and because a client that checks the "
                             "name usually wants a matching --mac too")
    parser.add_argument("--model", default="PyNoise",
                        help="model name. With the vendor and serial this "
                             "forms the device id a client lists, so two "
                             "otherwise identical cameras need different "
                             "ones here or in --serial")
    parser.add_argument("--serial", default="PY-0001",
                        help="serial number. Part of the device id, and the "
                             "usual thing to vary between two of the same "
                             "camera; some clients pin it in their config")
    parser.add_argument("--mac", default="",
                        help="MAC address to report, e.g. 00:30:53:12:34:56. "
                             "Defaults to the interface's own. Nothing is "
                             "sent from it -- but its first three octets are "
                             "the vendor's IEEE OUI, and a vendor's client "
                             "may admit only devices carrying theirs")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--pixel-format", default="Mono8")
    parser.add_argument("--frame-rate", type=float, default=10.0)
    parser.add_argument("--heartbeat-timeout", type=int, default=3000,
                        metavar="MS",
                        help="how long the device waits for a client to say "
                             "something before releasing control. Any command "
                             "counts, so raise this only for a client that "
                             "goes quiet for a long time, not merely a slow "
                             "one")
    parser.add_argument("--packet-size", type=int, default=1400)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    # Check the interface before anything else, so a typo fails with a
    # readable message rather than a traceback.
    try:
        netif.interface_info(args.interface)
    except netif.InterfaceError as e:
        parser.error(str(e))

    mac = None
    if args.mac:
        try:
            mac = netif.parse_mac(args.mac)
        except ValueError as e:
            parser.error(str(e))

    camera = NoiseCamera(width=args.width, height=args.height,
                         pixel_format=args.pixel_format,
                         pixel_formats=["Mono8", "Mono16"],
                         frame_rate=args.frame_rate)

    try:
        server = GigECameraServer(camera, interface=args.interface, mac=mac,
                                  vendor_name=args.vendor, model_name=args.model,
                                  serial_number=args.serial,
                                  user_defined_name=args.name,
                                  packet_size=args.packet_size,
                              heartbeat_timeout_ms=args.heartbeat_timeout)
    except ValueError as e:
        parser.error(str(e))
    print("ctrl-c to exit.")
    try:
        server.serve_forever()
    finally:
        print("stats:", server.stats)
