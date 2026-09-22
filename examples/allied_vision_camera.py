#
# An Allied Vision camera served as a GigE Vision device.
#
#   GENICAM_GENTL64_PATH=~/VimbaX/cti \
#       python examples/allied_vision_camera.py --interface eno1
#
# The camera this was written against is a USB3 Vision Alvium, so this puts a
# camera with no network interface of its own onto the network. vmbpy finds
# its transport layers through GENICAM_GENTL64_PATH and fails at import time
# with "No TL detected" if that is unset, which is why the line above carries
# it.
#
# The reported MAC defaults to Allied Vision's OUI. That is load bearing:
# VimbaX's GigE transport layer -- which VimbaX Viewer and Micro-Manager's
# Allied Vision adapter both sit on -- enumerates a device only if its MAC
# begins 00:0a:47 or 00:0f:31, and checks nothing else. It does not care what
# the device calls itself, but the vendor, model and serial are read from the
# camera anyway, because a client showing the real camera's name is the whole
# point. See "Not showing up in a vendor's client" in the README.
#

import argparse
import contextlib
import logging
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import vmbpy

from gige_emulator import (EmulatedCamera, EnumFeature, FloatFeature,
                           GigECameraServer, IntFeature, netif)

log = logging.getLogger("allied_vision_camera")

#: Allied Vision's IEEE OUIs, from the public MA-L registry. The first is
#: the one to use; the second is the Prosilica-era block, and VimbaX accepts
#: either.
ALLIED_VISION_OUI = "00:0a:47"

#: Formats this emulator has a PFNC identifier for. The Alvium also offers
#: packed variants -- Mono10p, Mono12p -- which are a different payload size
#: under the same name, so they are dropped rather than mis-sized.
USABLE_FORMATS = ("Mono8", "Mono10", "Mono12", "Mono16", "RGB8", "BGR8",
                  "BayerGR8", "BayerRG8", "BayerGB8", "BayerBG8",
                  "BayerGR12", "BayerRG12", "BayerGB12", "BayerBG12")

#: See the note in basler_camera.py: every register read a client makes lands
#: in get_camera_settings() on the control thread, which has a latency budget
#: measured in tens of milliseconds.
SETTINGS_CACHE_SECONDS = 0.5

#: Buffers handed to the driver. Four is enough to keep the sensor fed while
#: one is being copied out, and this camera's frames are 12 MB.
BUFFER_COUNT = 4


class _FrameSink(object):
    """
    Keeps the newest complete frame and hands it over once.

    A queue would be wrong here. The GigE client sets the pace, and when it
    is slower than the sensor the right frame to send is the most recent one,
    not the oldest one still buffered -- that is a stale image presented as a
    live one.
    """

    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None
        self.n_frames = 0
        self.n_incomplete = 0
        self.n_dropped = 0

    def __call__(self, cam, stream, frame):
        if frame.get_status() == vmbpy.FrameStatus.Complete:
            with self.condition:
                if self.frame is not None:
                    self.n_dropped += 1
                self.frame = bytes(frame.get_buffer())
                self.n_frames += 1
                self.condition.notify_all()
        else:
            self.n_incomplete += 1
        cam.queue_frame(frame)

    def take(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.frame is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not self.condition.wait(remaining):
                    return None
            frame, self.frame = self.frame, None
            return frame


def entry_names(feature):
    """Enum entry names as plain strings; vmbpy hands back objects."""
    return [str(entry) for entry in feature.get_available_entries()]


class AlliedVisionCamera(EmulatedCamera):

    def __init__(self, serial=None, reset=False, **kwds):
        self._stack = contextlib.ExitStack()
        self._vmb = self._stack.enter_context(vmbpy.VmbSystem.get_instance())

        # USB only, and the exclusion is not academic: this program serves a
        # device carrying an Allied Vision OUI, so VimbaX on this same
        # machine enumerates it alongside the real hardware, with the same
        # model name. Without this, a second instance -- or a restart while
        # the first is still up -- can pick the emulator and serve it to
        # itself.
        cameras = [c for c in self._vmb.get_all_cameras()
                   if c.get_transport_layer().get_type()
                   == vmbpy.TransportLayerType.U3V]
        if not cameras:
            raise RuntimeError("no Allied Vision USB camera found")
        if serial is not None:
            cameras = [c for c in cameras if c.get_serial() == serial]
            if not cameras:
                raise RuntimeError("no Allied Vision USB camera with serial "
                                   "%r" % serial)
        elif len(cameras) > 1:
            raise RuntimeError(
                "several Allied Vision USB cameras attached, so "
                "--device-serial is needed: %s"
                % ", ".join(c.get_serial() for c in cameras))
        self.cam = self._stack.enter_context(cameras[0])

        # Not every camera has binning, and a model that lacks it still
        # carries the feature -- unreadable. Settled here, before the reset
        # below and everything after it: reading an absent feature raises,
        # and the example would refuse to start on such a camera.
        self.has_binning = (self._available("BinningHorizontal")
                            and self._available("BinningVertical"))
        if not self.has_binning:
            log.info("this camera has no binning; serving it without "
                     "BinningHorizontal or BinningVertical")

        if reset:
            self._reset_geometry()

        self.vendor_name = self._value("DeviceVendorName")
        self.model_name = self._value("DeviceModelName")
        self.serial_number = self._value("DeviceSerialNumber")

        self.sink = _FrameSink()
        self.streaming = False
        self.n_grab_failures = 0
        self._cached = {}
        self._cached_at = 0.0

        sensor_width = self._value("WidthMax")
        sensor_height = self._value("HeightMax")

        # Per instance, not on the class: every bound in them is this
        # camera's. Set before super().__init__ reads self.extra_features.
        self.extra_features = self._describe_features()

        super().__init__(width=self._value("Width"),
                         height=self._value("Height"),
                         pixel_format=self._value("PixelFormat"),
                         pixel_formats=self._pixel_formats(),
                         sensor_width=sensor_width,
                         sensor_height=sensor_height,
                         frame_rate=self._value("AcquisitionFrameRate"),
                         # The bound validate() enforces. What is reachable
                         # at the current ROI and exposure is
                         # AcquisitionFrameRateMax, wired up below.
                         max_frame_rate=10000.0,
                         roi=True,
                         offset=(self._value("OffsetX"),
                                 self._value("OffsetY")),
                         roi_bounds=self._roi_bounds(sensor_width,
                                                     sensor_height),
                         **kwds)

        features = self.feature_set.by_name
        rate = features["AcquisitionFrameRate"]
        rate.p_max = "AcquisitionFrameRateMax"
        rate.invalidated_by = self._moves_the_ceiling()
        self._refresh_frame_rate_ceiling()

        # Binning moves the geometry: the window is in binned pixels, so the
        # camera resizes it and the payload with it. A GenICam client caches
        # what it has read, so without these it goes on believing the width
        # it saw before and sizes its buffers from it.
        if self.has_binning:
            binning = ("BinningHorizontal", "BinningVertical")
            for name in ("Width", "Height", "OffsetX", "OffsetY",
                         "PayloadSize"):
                features[name].invalidated_by = (
                    tuple(features[name].invalidated_by) + binning)

        # And the axes move each other, one way: vertical binning above 1
        # forces horizontal to 2, which the camera does by itself. The
        # reverse does not happen, so only this direction is declared -- a
        # pair that invalidate each other is a cycle, and GenApi
        # implementations are not obliged to enjoy it.
        if self.has_binning:
            features["BinningHorizontal"].invalidated_by = ("BinningVertical",)

    # --- talking to vmbpy ------------------------------------------------

    def _feature(self, name):
        return self.cam.get_feature_by_name(name)

    def _available(self, name):
        """
        Whether this camera implements a feature at all.

        vmbpy answers for a feature the model does not have by carrying it
        and refusing to read it, so this asks rather than reads.
        """
        try:
            return self._feature(name).is_readable()
        except vmbpy.VmbFeatureError:
            return False

    def _value(self, name):
        value = self._feature(name).get()
        # Enums come back as objects and strings as bytes; the emulator
        # stores and compares plain str.
        if isinstance(value, bytes):
            return value.decode()
        if isinstance(value, (int, float, str)):
            return value
        return str(value)

    def _clamp(self, name, value):
        """
        Fit a value to the range the feature reports right now.

        The camera would refuse an illegal write, but a refusal reaches the
        client as a protocol error while a rounded value reaches it as a
        reading of what happened.
        """
        low, high = self._feature(name).get_range()
        value = max(low, min(high, value))
        try:
            inc = self._feature(name).get_increment()
        except Exception:
            inc = None
        if inc:
            value = low + ((value - low) // inc) * inc
        return value

    def _reset_geometry(self):
        """
        Full frame, no binning.

        Off by default, because a camera is entitled to keep the settings it
        was left in and a vendor's own viewer does not reset one either. But
        those settings outlive the process that made them, so a run that
        starts at a fraction of the sensor because an earlier experiment left
        it there looks exactly like a bug in here.
        """
        if self.has_binning:
            for name in ("BinningHorizontal", "BinningVertical"):
                self._feature(name).set(self._feature(name).get_range()[0])
        self._feature("OffsetX").set(0)
        self._feature("OffsetY").set(0)
        self._feature("Width").set(self._feature("Width").get_range()[1])
        self._feature("Height").set(self._feature("Height").get_range()[1])

    # --- what this camera can do ----------------------------------------

    def _pixel_formats(self):
        offered = entry_names(self._feature("PixelFormat"))
        usable = [name for name in offered if name in USABLE_FORMATS]
        dropped = [name for name in offered if name not in USABLE_FORMATS]
        if dropped:
            log.info("not advertising %s: no PFNC identifier for %s",
                     ", ".join(dropped),
                     "them" if len(dropped) > 1 else "it")
        current = self._value("PixelFormat")
        if current not in usable:
            # Nothing else would work: the format the camera is in decides
            # the size of every buffer it hands over.
            usable.insert(0, current)
        return usable

    def _roi_bounds(self, sensor_width, sensor_height):
        """
        (min, max, inc) for each of the four ROI features.

        The maxima are the sensor's rather than the camera's current ones.
        vmbpy reports OffsetX's range as (0, 0) while the width is at
        maximum, which is true right now and would be a lie in the XML: a
        client told 0..0 will never offer the control again, even after the
        width comes down. The camera still enforces the real interlock on
        every write.
        """
        width, height = self._feature("Width"), self._feature("Height")
        return {
            "Width": (width.get_range()[0], sensor_width,
                      width.get_increment()),
            "Height": (height.get_range()[0], sensor_height,
                       height.get_increment()),
            "OffsetX": (0, sensor_width - width.get_range()[0],
                        self._feature("OffsetX").get_increment()),
            "OffsetY": (0, sensor_height - height.get_range()[0],
                        self._feature("OffsetY").get_increment()),
        }

    def _moves_the_ceiling(self):
        """
        Features a client can change that move the reachable frame rate.

        An invalidator naming a feature this camera does not publish is
        refused by the XML validator, so a camera without binning must not
        be told that binning changes anything.
        """
        names = ["ExposureTime", "Width", "Height", "PixelFormat"]
        if self.has_binning:
            names += ["BinningHorizontal", "BinningVertical"]
        return tuple(names)

    def _describe_features(self):
        exposure = self._feature("ExposureTime")
        low, high = exposure.get_range()
        features = [
            FloatFeature("ExposureTime", "Exposure time",
                         "AcquisitionControl", "RW",
                         default=exposure.get(), min=low, max=high,
                         unit="us"),
            EnumFeature("ExposureAuto", "Automatic exposure",
                        "AcquisitionControl", "RW",
                        entries={name: index for index, name in
                                 enumerate(entry_names(
                                     self._feature("ExposureAuto")))},
                        default=self._value("ExposureAuto")),
            FloatFeature("AcquisitionFrameRateMax",
                         "Fastest frame rate the current settings sustain",
                         "AcquisitionControl", "RO",
                         invalidated_by=self._moves_the_ceiling(),
                         default=0.0, min=0.0, max=1e6, unit="Hz"),
        ]
        if self.has_binning:
            binning_h = self._feature("BinningHorizontal")
            binning_v = self._feature("BinningVertical")
            features += [
                IntFeature("BinningHorizontal", "Horizontal binning factor",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=binning_h.get(),
                           min=binning_h.get_range()[0],
                           max=binning_h.get_range()[1]),
                IntFeature("BinningVertical", "Vertical binning factor",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=binning_v.get(),
                           min=binning_v.get_range()[0],
                           max=binning_v.get_range()[1]),
            ]
        return tuple(features)

    # --- lifecycle -------------------------------------------------------

    def close(self):
        self._pause_streaming()
        self._stack.close()

    def _pause_streaming(self):
        """
        Stop the stream so a geometry write is legal.

        The emulator refuses these writes while a client is acquiring, but
        streaming here is started on demand and outlives one client's
        acquisition, so the camera can still be running when a write arrives.
        """
        if self.streaming:
            self.cam.stop_streaming()
            self.streaming = False
            with self.sink.condition:
                self.sink.frame = None

    # --- frames ----------------------------------------------------------

    def next_frame(self):
        if not self.streaming:
            self.cam.start_streaming(handler=self.sink,
                                     buffer_count=BUFFER_COUNT)
            self.streaming = True
        frame = self.sink.take()
        if frame is None:
            self.n_grab_failures += 1
        return frame

    # --- settings --------------------------------------------------------

    def _refresh_frame_rate_ceiling(self):
        """
        The live upper bound on the frame rate, which is exactly what this
        camera reports as the feature's own maximum -- it moves with the ROI,
        the exposure and binning without any coaxing.
        """
        self.settings["AcquisitionFrameRateMax"] = \
            self._feature("AcquisitionFrameRate").get_range()[1]

    def _apply_roi(self):
        """
        Move the window, offsets through zero first.

        Size and position interlock, so the same pair of numbers is accepted
        or refused depending on which is written first. Going through the
        origin costs two extra writes and is always legal.
        """
        self._pause_streaming()
        self._feature("OffsetX").set(0)
        self._feature("OffsetY").set(0)
        self._feature("Width").set(self._clamp("Width", self.settings["Width"]))
        self._feature("Height").set(self._clamp("Height",
                                                self.settings["Height"]))
        self._feature("OffsetX").set(self._clamp("OffsetX",
                                                 self.settings["OffsetX"]))
        self._feature("OffsetY").set(self._clamp("OffsetY",
                                                 self.settings["OffsetY"]))
        self._publish_geometry()

    def _publish_geometry(self):
        """
        Put back what the camera actually did. The payload size the client is
        about to read is computed from these.
        """
        for name in ("Width", "Height", "OffsetX", "OffsetY",
                     "BinningHorizontal", "BinningVertical"):
            if name in self.settings:
                self.settings[name] = self._value(name)

    def set_camera_settings(self, changed):
        if {"Width", "Height", "OffsetX", "OffsetY"} & set(changed):
            self._apply_roi()

        for name in ("BinningHorizontal", "BinningVertical"):
            if name in changed:
                self._pause_streaming()
                # Not clamped, unlike the continuous settings above. Binning
                # is a choice, not a magnitude, so the nearest legal value is
                # a different setting rather than a rounded one -- and this
                # camera moves the bound: BinningHorizontal will not go below
                # 2 while BinningVertical is above 1. A client returning to
                # 1x1 writes horizontal first, which was clamped back to 2
                # and reported as success, leaving the camera at 2x1 with
                # nothing saying so. Letting the camera's refusal through
                # reaches the client as an error on the write it got wrong.
                self._feature(name).set(changed[name])
                # Binning resizes the ROI under us, and the client sizes its
                # buffers from Width and Height.
                self._publish_geometry()

        if "PixelFormat" in changed:
            self._pause_streaming()
            self._feature("PixelFormat").set(changed["PixelFormat"])

        if "ExposureAuto" in changed:
            self._feature("ExposureAuto").set(changed["ExposureAuto"])

        if "ExposureTime" in changed:
            # Allowed under automatic exposure too, where the camera will
            # overwrite it on the next frame.
            self._feature("ExposureTime").set(
                self._clamp("ExposureTime", changed["ExposureTime"]))

        if "AcquisitionFrameRate" in changed:
            # The rate is a limit and does nothing until it is switched on.
            self._feature("AcquisitionFrameRateEnable").set(True)
            self._feature("AcquisitionFrameRate").set(
                self._clamp("AcquisitionFrameRate",
                            changed["AcquisitionFrameRate"]))

        self._refresh_frame_rate_ceiling()
        self._cached_at = 0.0

    def get_camera_settings(self):
        now = time.monotonic()
        if now - self._cached_at < SETTINGS_CACHE_SECONDS:
            return self._cached
        # Rebuilding the cache is also the moment to re-read the ceiling. It
        # moves with the exposure, and under automatic exposure the exposure
        # moves with the light, so a ceiling refreshed only when a client
        # writes something would go stale in a darkening room.
        self._refresh_frame_rate_ceiling()
        self._cached = {
            "ExposureTime": self._value("ExposureTime"),
            "ExposureAuto": self._value("ExposureAuto"),
            "AcquisitionFrameRate": self._value("AcquisitionFrameRate"),
            "Width": self._value("Width"),
            "Height": self._value("Height"),
            "OffsetX": self._value("OffsetX"),
            "OffsetY": self._value("OffsetY"),
            "AcquisitionFrameRateMax":
                self.settings["AcquisitionFrameRateMax"],
        }
        if self.has_binning:
            self._cached["BinningHorizontal"] = \
                self._value("BinningHorizontal")
            self._cached["BinningVertical"] = self._value("BinningVertical")
        self._cached_at = now
        return self._cached


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Allied Vision camera -> GigE")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--device-serial", default=None,
                        help="which camera, when more than one is attached")
    parser.add_argument("--name", default="",
                        help="user-defined camera name. Clients show this and "
                             "can select on it, so it is how you tell two "
                             "otherwise identical cameras apart")
    parser.add_argument("--mac", default="",
                        help="MAC address to report. Defaults to Allied "
                             "Vision's OUI %s with a tail derived from the "
                             "camera's serial -- VimbaX will not enumerate a "
                             "device without one. Pass --mac none for the "
                             "interface's own" % ALLIED_VISION_OUI)
    parser.add_argument("--reset", action="store_true",
                        help="put the camera back to full frame with no "
                             "binning before serving. Settings outlive the "
                             "process that made them, so an ROI left behind "
                             "by an earlier run is otherwise inherited "
                             "silently")
    parser.add_argument("--list", action="store_true",
                        help="print the attached cameras, then exit")
    parser.add_argument("--any-destination", action="store_true",
                        help="stream to whatever address the client asks "
                             "for, rather than only back to the client "
                             "holding control. Off by default: a device that "
                             "sends where it is told is an amplifier for "
                             "anyone who can forge a source address. Turn it "
                             "on to hand the images to another machine -- or "
                             "for a client on a host with two interfaces on "
                             "one network, which asks for the images on the "
                             "other one and is refused. That refusal is worth "
                             "reading before switching it off: it means the "
                             "commands and the images take different paths")
    parser.add_argument("--packet-size", type=int, default=1400)
    parser.add_argument("--heartbeat-timeout", type=int, default=3000,
                        metavar="MS")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    if args.list:
        with vmbpy.VmbSystem.get_instance() as vmb:
            for cam in vmb.get_all_cameras():
                # The transport layer is printed because anything that is
                # not U3V is listed for information and cannot be served --
                # including, confusingly, a device this program is serving
                # right now.
                print("%-26s %-16s %-8s %s"
                      % (cam.get_name(), cam.get_model(), cam.get_serial(),
                         cam.get_transport_layer().get_type()))
        sys.exit(0)

    try:
        netif.interface_info(args.interface)
    except netif.InterfaceError as e:
        parser.error(str(e))

    try:
        camera = AlliedVisionCamera(serial=args.device_serial,
                                    reset=args.reset)
    except (RuntimeError, vmbpy.VmbCameraError, vmbpy.VmbFeatureError) as e:
        parser.error(str(e))

    if args.mac.lower() in ("none", "interface"):
        mac = None
    elif args.mac:
        try:
            mac = netif.parse_mac(args.mac)
        except ValueError as e:
            parser.error(str(e))
    else:
        mac = netif.mac_from_serial(ALLIED_VISION_OUI, camera.serial_number)

    server = GigECameraServer(camera, interface=args.interface, mac=mac,
                              vendor_name=camera.vendor_name,
                              model_name=camera.model_name,
                              serial_number=camera.serial_number,
                              user_defined_name=args.name,
                              packet_size=args.packet_size,
                              heartbeat_timeout_ms=args.heartbeat_timeout,
                              allow_any_destination=args.any_destination)
    print("serving %s %s (%s) at %dx%d %s, ctrl-c to exit."
          % (camera.vendor_name, camera.model_name, camera.serial_number,
             camera.settings["Width"], camera.settings["Height"],
             camera.settings["PixelFormat"]))
    try:
        server.serve_forever()
    finally:
        camera.close()
        print("stats:", server.stats,
              "frames:", camera.sink.n_frames,
              "incomplete:", camera.sink.n_incomplete,
              "superseded:", camera.sink.n_dropped,
              "grab failures:", camera.n_grab_failures)
