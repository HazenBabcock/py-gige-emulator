#
# A Teledyne FLIR camera served as a GigE Vision device.
#
#   python examples/flir_camera.py --interface eth0
#
# The camera this was written against is a USB3 Vision Blackfly S, so this
# puts a camera with no network interface of its own onto the network. It
# needs PySpin, which Teledyne ships as a wheel matching both the installed
# Spinnaker SDK and the Python version -- not on PyPI, and not
# interchangeable between versions.
#
# Run against a Blackfly S BFS-U3-19S4M, with Spinnaker 4.4.
#
# The reported MAC defaults to FLIR's OUI. That is load bearing: Spinnaker's
# GigE transport layer -- which SpinView and Micro-Manager's Spinnaker
# adapter both sit on -- enumerates a device only if its MAC begins with one
# of five blocks belonging to FLIR, Point Grey or Teledyne DALSA, and checks
# nothing else. It does not care what the device calls itself; the vendor,
# model and serial are read from the camera anyway, because a client showing
# the real camera's name is the whole point. See "Not showing up in a
# vendor's client" in the README.
#

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import PySpin

from gige_emulator import (EmulatedCamera, EnumFeature, FloatFeature,
                           GigECameraServer, IntFeature, netif)

log = logging.getLogger("flir_camera")

#: FLIR Systems' IEEE OUI, from the public MA-L registry. Spinnaker also
#: admits Point Grey's 00:b0:9d and 2c:dd:a3, FLIR's own 00:40:7f and
#: Teledyne DALSA's 00:01:0d -- measured, not documented. It refuses every
#: other block, including FLIR Radiation's and Teledyne Technologies' own.
FLIR_OUI = "00:1b:d8"

#: Formats this emulator has a PFNC identifier for. The Blackfly also offers
#: packed variants -- Mono10p, Mono12p, Mono10Packed, Mono12Packed -- which
#: are a different payload size under the same name, so they are dropped
#: rather than mis-sized.
USABLE_FORMATS = ("Mono8", "Mono10", "Mono12", "Mono16", "RGB8", "BGR8",
                  "BayerGR8", "BayerRG8", "BayerGB8", "BayerBG8",
                  "BayerGR12", "BayerRG12", "BayerGB12", "BayerBG12")

#: See the note in basler_camera.py: every register read a client makes lands
#: in get_camera_settings() on the control thread, which has a latency budget
#: measured in tens of milliseconds.
SETTINGS_CACHE_SECONDS = 0.5

#: Buffers the driver keeps. Four is enough to keep the sensor fed while one
#: is being copied out.
BUFFER_COUNT = 4

#: How long to wait for a frame before giving up on it. Long enough for this
#: camera's 30 s maximum exposure to be a deliberate setting rather than a
#: hang, and short enough that a stalled sensor is reported rather than
#: parking the stream thread forever.
GRAB_TIMEOUT_MS = 35000


def device_info(cam, name):
    """
    Read one of the transport layer's own strings.

    These live on a nodemap that exists before Init(), which is what makes
    --list possible without taking a camera someone else is using.
    """
    node = PySpin.CStringPtr(cam.GetTLDeviceNodeMap().GetNode(name))
    return node.GetValue() if PySpin.IsReadable(node) else ""


class FlirCamera(EmulatedCamera):

    def __init__(self, serial=None, reset=False, **kwds):
        self._system = PySpin.System.GetInstance()
        self._cameras = self._system.GetCameras()
        self.cam = None

        found = [(c, device_info(c, "DeviceSerialNumber"))
                 for c in self._cameras]
        if serial is not None:
            found = [pair for pair in found if pair[1] == serial]
            if not found:
                raise RuntimeError("no FLIR camera with serial %r" % serial)
        elif not found:
            raise RuntimeError("no FLIR camera found")
        elif len(found) > 1:
            raise RuntimeError(
                "several FLIR cameras attached, so --device-serial is "
                "needed: %s" % ", ".join(pair[1] for pair in found))

        self.cam = found[0][0]
        self.vendor_name = device_info(self.cam, "DeviceVendorName")
        self.model_name = device_info(self.cam, "DeviceModelName")
        self.serial_number = found[0][1]
        self.cam.Init()

        # Not every model has binning, and one that lacks it still carries
        # the feature -- present but not available, and reading it raises.
        # Settled here, before the reset below and everything after it.
        self.has_binning = (self._available("BinningHorizontal")
                            and self._available("BinningVertical"))
        if not self.has_binning:
            log.info("this camera has no binning; serving it without "
                     "BinningHorizontal or BinningVertical")

        # Newest first, and a bounded set of buffers. The GigE client sets
        # the pace, and when it is slower than the sensor the right frame to
        # send is the one the camera sees now -- the oldest frame in a queue
        # that filled while nobody was asking is a stale image presented as
        # a live one.
        stream = self.cam.TLStream
        stream.StreamBufferHandlingMode.SetValue(
            PySpin.StreamBufferHandlingMode_NewestOnly)
        stream.StreamBufferCountMode.SetValue(
            PySpin.StreamBufferCountMode_Manual)
        stream.StreamBufferCountManual.SetValue(BUFFER_COUNT)

        if reset:
            self._reset_geometry()

        self.n_grab_failures = 0
        self.n_incomplete = 0
        self._cached = {}
        self._cached_at = 0.0

        sensor_width = self.cam.WidthMax.GetValue()
        sensor_height = self.cam.HeightMax.GetValue()

        # Per instance rather than on the class, because every bound in them
        # belongs to this particular camera. Set before super().__init__
        # reads self.extra_features.
        self.extra_features = self._describe_features()

        super().__init__(width=self.cam.Width.GetValue(),
                         height=self.cam.Height.GetValue(),
                         pixel_format=self._enum("PixelFormat"),
                         pixel_formats=self._pixel_formats(),
                         sensor_width=sensor_width,
                         sensor_height=sensor_height,
                         frame_rate=self.cam.AcquisitionResultingFrameRate
                         .GetValue(),
                         # The absolute bound validate() enforces, not the
                         # reachable one: shrinking the ROI raises what this
                         # sensor will do, so pinning it to the rate measured
                         # at startup would refuse a rate the camera can
                         # actually reach. What is reachable right now is
                         # AcquisitionFrameRateMax, wired up below.
                         max_frame_rate=10000.0,
                         roi=True,
                         offset=(self.cam.OffsetX.GetValue(),
                                 self.cam.OffsetY.GetValue()),
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

    # --- lifecycle -------------------------------------------------------

    def close(self):
        """
        Give the camera back, in the order Spinnaker insists on.

        A camera still referenced when the system is released leaves the
        library complaining at exit and, worse, the device claimed until the
        process dies.
        """
        if self.cam is not None:
            self._pause_streaming()
            self.cam.DeInit()
            del self.cam
            self.cam = None
        self._cameras.Clear()
        self._system.ReleaseInstance()

    def _pause_streaming(self):
        """
        Stop the stream so a geometry write is legal.

        The emulator refuses these writes while a client is acquiring, but
        streaming here is started on demand and outlives one client's
        acquisition, so the camera can still be running when a write arrives.
        """
        if self.cam is not None and self.cam.IsStreaming():
            self.cam.EndAcquisition()

    # --- talking to PySpin ------------------------------------------------

    def _node(self, name):
        return getattr(self.cam, name)

    def _available(self, name):
        """
        Whether this camera implements a feature at all.

        A feature the model does not have is still in the nodemap, so this
        asks rather than reads: reading raises.
        """
        node = getattr(self.cam, name, None)
        return node is not None and PySpin.IsReadable(node)

    def _enum(self, name):
        """The symbolic name of an enumeration's current entry."""
        return self._node(name).GetCurrentEntry().GetSymbolic()

    def _set_enum(self, name, value):
        """
        Select an enumeration entry by its symbolic name.

        Entries are set by their integer value, and the integer is the
        camera's own -- not the position in the list, which is why the entry
        is looked up rather than counted.
        """
        node = self._node(name)
        node.SetIntValue(node.GetEntryByName(value).GetValue())

    def _clamp(self, name, value):
        """
        Fit a value to the range the feature reports right now.

        The camera would refuse an illegal write, but a refusal reaches the
        client as a protocol error while a rounded value reaches it as a
        reading of what happened. Not used for binning, where the nearest
        legal value is a different setting rather than a rounded one.
        """
        node = self._node(name)
        value = max(node.GetMin(), min(node.GetMax(), value))
        try:
            inc = node.GetInc()
        except (AttributeError, PySpin.SpinnakerException):
            inc = None
        if inc:
            low = node.GetMin()
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
        self._pause_streaming()
        if self.has_binning:
            for name in ("BinningHorizontal", "BinningVertical"):
                node = self._node(name)
                node.SetValue(node.GetMin())
        self.cam.OffsetX.SetValue(0)
        self.cam.OffsetY.SetValue(0)
        self.cam.Width.SetValue(self.cam.Width.GetMax())
        self.cam.Height.SetValue(self.cam.Height.GetMax())

    # --- what this camera can do ----------------------------------------

    def _pixel_formats(self):
        node = self.cam.PixelFormat
        offered = [entry.GetSymbolic() for entry in
                   (PySpin.CEnumEntryPtr(x) for x in node.GetEntries())
                   if PySpin.IsReadable(entry)]
        usable = [name for name in offered if name in USABLE_FORMATS]
        dropped = [name for name in offered if name not in USABLE_FORMATS]
        if dropped:
            log.info("not advertising %s: no PFNC identifier for %s",
                     ", ".join(dropped),
                     "them" if len(dropped) > 1 else "it")
        current = self._enum("PixelFormat")
        if current not in usable:
            # Nothing else would work: the format the camera is in decides
            # the size of every buffer it hands over.
            usable.insert(0, current)
        return usable

    def _roi_bounds(self, sensor_width, sensor_height):
        """
        (min, max, inc) for each of the four ROI features.

        The maxima are the sensor's, not the camera's current ones: an
        offset's range is reported as (0, 0) while the width is at maximum,
        which is true right now and would be a lie in the XML -- a client
        told 0..0 will never offer the control again, even after the width
        comes down. The camera still enforces the real interlock on every
        write.
        """
        width, height = self.cam.Width, self.cam.Height
        return {
            "Width": (width.GetMin(), sensor_width, width.GetInc()),
            "Height": (height.GetMin(), sensor_height, height.GetInc()),
            "OffsetX": (0, sensor_width - width.GetMin(),
                        self.cam.OffsetX.GetInc()),
            "OffsetY": (0, sensor_height - height.GetMin(),
                        self.cam.OffsetY.GetInc()),
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
        exposure, gain = self.cam.ExposureTime, self.cam.Gain
        features = [
            FloatFeature("ExposureTime", "Exposure time",
                         "AcquisitionControl", "RW",
                         default=exposure.GetValue(), min=exposure.GetMin(),
                         max=exposure.GetMax(), unit="us"),
            EnumFeature("ExposureAuto", "Automatic exposure",
                        "AcquisitionControl", "RW",
                        entries=self._entries("ExposureAuto"),
                        default=self._enum("ExposureAuto")),
            FloatFeature("Gain", "Analog gain", "AnalogControl", "RW",
                         default=gain.GetValue(), min=gain.GetMin(),
                         max=gain.GetMax(), unit="dB"),
            # This camera ships with both of these on. Exposing the gain
            # without its automatic is the worst of the three states: a
            # client that sets an exposure by hand still watches the image
            # brightness wander, with nothing on the camera saying why.
            EnumFeature("GainAuto", "Automatic gain", "AnalogControl", "RW",
                        entries=self._entries("GainAuto"),
                        default=self._enum("GainAuto")),
            FloatFeature("AcquisitionFrameRateMax",
                         "Fastest frame rate the current settings sustain",
                         "AcquisitionControl", "RO",
                         invalidated_by=self._moves_the_ceiling(),
                         default=0.0, min=0.0, max=1e6, unit="Hz"),
        ]
        if self.has_binning:
            horizontal, vertical = (self.cam.BinningHorizontal,
                                    self.cam.BinningVertical)
            features += [
                IntFeature("BinningHorizontal", "Horizontal binning factor",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=horizontal.GetValue(),
                           min=horizontal.GetMin(), max=horizontal.GetMax()),
                IntFeature("BinningVertical", "Vertical binning factor",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=vertical.GetValue(),
                           min=vertical.GetMin(), max=vertical.GetMax()),
            ]
        return tuple(features)

    def _entries(self, name):
        """{symbolic name: index} for an enumeration the client may write."""
        node = self._node(name)
        return {entry.GetSymbolic(): index for index, entry in
                enumerate(PySpin.CEnumEntryPtr(x) for x in node.GetEntries())
                if PySpin.IsReadable(entry)}

    # --- frames ----------------------------------------------------------

    def next_frame(self):
        if not self.cam.IsStreaming():
            self.cam.BeginAcquisition()
        try:
            image = self.cam.GetNextImage(GRAB_TIMEOUT_MS)
        except PySpin.SpinnakerException as e:
            self.n_grab_failures += 1
            log.warning("grab failed: %s", e)
            return None
        try:
            if image.IsIncomplete():
                # Not a frame to send on: a GigE client sizes its buffer from
                # PayloadSize and would read the shortfall as packet loss.
                self.n_incomplete += 1
                return None
            return image.GetData().tobytes()
        finally:
            image.Release()

    # --- settings --------------------------------------------------------

    def _refresh_frame_rate_ceiling(self):
        """
        What the camera would run at with no rate limit applied.

        AcquisitionResultingFrameRate is the honest ceiling only while
        AcquisitionFrameRateEnable is off; once a client sets a rate, it
        reports that rate back and would drag the ceiling down to meet it.
        So the limit is lifted for the read when that is safe, and when it is
        not, the last known ceiling stands rather than being replaced by a
        number that is really just the current setting.

        Lifting it is destructive on this camera, which is worth knowing
        before copying this elsewhere: clearing AcquisitionFrameRateEnable
        puts AcquisitionFrameRate back to its maximum, so the rate a client
        asked for has to be read first and written back afterwards. Without
        that, every rate a client sets is undone by the next read of any
        setting -- which happens constantly, since this runs whenever the
        cache is rebuilt.
        """
        limited = self.cam.AcquisitionFrameRateEnable.GetValue()
        if limited and self.acquiring:
            return
        wanted = self.cam.AcquisitionFrameRate.GetValue() if limited else None
        try:
            if limited:
                self.cam.AcquisitionFrameRateEnable.SetValue(False)
            self.settings["AcquisitionFrameRateMax"] = \
                self.cam.AcquisitionResultingFrameRate.GetValue()
        finally:
            if limited:
                self.cam.AcquisitionFrameRateEnable.SetValue(True)
                self.cam.AcquisitionFrameRate.SetValue(
                    self._clamp("AcquisitionFrameRate", wanted))

    def _apply_roi(self):
        """
        Move the window, offsets through zero first.

        Size and position interlock, so the same pair of numbers is accepted
        or refused depending on which is written first. Going through the
        origin costs two extra writes and is always legal.
        """
        self._pause_streaming()
        self.cam.OffsetX.SetValue(0)
        self.cam.OffsetY.SetValue(0)
        self.cam.Width.SetValue(self._clamp("Width", self.settings["Width"]))
        self.cam.Height.SetValue(self._clamp("Height",
                                             self.settings["Height"]))
        self.cam.OffsetX.SetValue(self._clamp("OffsetX",
                                              self.settings["OffsetX"]))
        self.cam.OffsetY.SetValue(self._clamp("OffsetY",
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
                self.settings[name] = self._node(name).GetValue()

    def set_camera_settings(self, changed):
        if {"Width", "Height", "OffsetX", "OffsetY"} & set(changed):
            self._apply_roi()

        for name in ("BinningHorizontal", "BinningVertical"):
            if name in changed:
                self._pause_streaming()
                # Not clamped, unlike the continuous settings below. Binning
                # is a choice rather than a magnitude, so the nearest legal
                # value is a different setting -- and on a camera whose axes
                # constrain each other, clamping answers a client's 1x1 with
                # a silent 2x1 and calls it success. The camera's refusal
                # reaches the client as an error on the write it got wrong.
                self._node(name).SetValue(changed[name])
                # Binning resizes the ROI under us, and the client sizes its
                # buffers from Width and Height.
                self._publish_geometry()

        if "PixelFormat" in changed:
            self._pause_streaming()
            self._set_enum("PixelFormat", changed["PixelFormat"])

        for name in ("ExposureAuto", "GainAuto"):
            if name in changed:
                self._set_enum(name, changed[name])

        if "ExposureTime" in changed:
            # Allowed under automatic exposure too, where the camera will
            # overwrite it on the next frame.
            self.cam.ExposureTime.SetValue(
                self._clamp("ExposureTime", changed["ExposureTime"]))

        if "Gain" in changed:
            self.cam.Gain.SetValue(self._clamp("Gain", changed["Gain"]))

        if "AcquisitionFrameRate" in changed:
            # The rate is a limit and does nothing until it is switched on.
            # Left off, the camera free runs and a client that set 10 fps
            # would see 131 and no error.
            self.cam.AcquisitionFrameRateEnable.SetValue(True)
            self.cam.AcquisitionFrameRate.SetValue(
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
            "ExposureTime": self.cam.ExposureTime.GetValue(),
            "ExposureAuto": self._enum("ExposureAuto"),
            "Gain": self.cam.Gain.GetValue(),
            "GainAuto": self._enum("GainAuto"),
            "AcquisitionFrameRate":
                self.cam.AcquisitionResultingFrameRate.GetValue(),
            "Width": self.cam.Width.GetValue(),
            "Height": self.cam.Height.GetValue(),
            "OffsetX": self.cam.OffsetX.GetValue(),
            "OffsetY": self.cam.OffsetY.GetValue(),
            "AcquisitionFrameRateMax":
                self.settings["AcquisitionFrameRateMax"],
        }
        if self.has_binning:
            self._cached["BinningHorizontal"] = \
                self.cam.BinningHorizontal.GetValue()
            self._cached["BinningVertical"] = \
                self.cam.BinningVertical.GetValue()
        self._cached_at = now
        return self._cached


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FLIR camera -> GigE")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--device-serial", default=None,
                        help="which camera, when more than one is attached")
    parser.add_argument("--name", default="",
                        help="user-defined camera name. Clients show this and "
                             "can select on it, so it is how you tell two "
                             "otherwise identical cameras apart")
    parser.add_argument("--mac", default="",
                        help="MAC address to report. Defaults to FLIR's OUI "
                             "%s with a tail derived from the camera's serial "
                             "-- Spinnaker will not enumerate a device "
                             "without one of the blocks it knows. Pass --mac "
                             "none for the interface's own" % FLIR_OUI)
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
        system = PySpin.System.GetInstance()
        cameras = system.GetCameras()
        for cam in cameras:
            print("%-14s %-26s %s" % (device_info(cam, "DeviceVendorName"),
                                      device_info(cam, "DeviceModelName"),
                                      device_info(cam, "DeviceSerialNumber")))
        del cam
        cameras.Clear()
        system.ReleaseInstance()
        sys.exit(0)

    try:
        netif.interface_info(args.interface)
    except netif.InterfaceError as e:
        parser.error(str(e))

    try:
        camera = FlirCamera(serial=args.device_serial, reset=args.reset)
    except (RuntimeError, PySpin.SpinnakerException) as e:
        parser.error(str(e))

    if args.mac.lower() in ("none", "interface"):
        mac = None
    elif args.mac:
        try:
            mac = netif.parse_mac(args.mac)
        except ValueError as e:
            parser.error(str(e))
    else:
        mac = netif.mac_from_serial(FLIR_OUI, camera.serial_number)

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
              "incomplete:", camera.n_incomplete,
              "grab failures:", camera.n_grab_failures)
