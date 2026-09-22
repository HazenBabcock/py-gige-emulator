#
# A Basler camera served as a GigE Vision device.
#
#   python examples/basler_camera.py --interface eno1
#
# The camera this was written against is a USB3 Vision ace, so this is not
# one network protocol wrapped in another: it puts a camera that has no
# network interface of its own onto the network.
#
# The reported MAC defaults to Basler's OUI, and that is load bearing rather
# than cosmetic. pylon's GigE transport layer -- which pylon Viewer, the
# pylon API and Micro-Manager's Basler adapter all sit on -- refuses to
# enumerate a device unless its MAC begins 00:30:53 *and* it calls itself
# Basler. Both come from the camera itself here, so the emulated device
# reports what the real one reports. See "Not showing up in a vendor's
# client" in the README.
#
# Every bound below is read from the camera rather than written down: the
# exposure range, the ROI increments, which pixel formats exist. A client
# builds its controls from those, so an invented one produces a spin box
# that refuses half its own range.
#

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from pypylon import genicam, pylon

from gige_emulator import (EmulatedCamera, EnumFeature, FloatFeature,
                           GigECameraServer, IntFeature, netif)

log = logging.getLogger("basler_camera")

#: Basler's IEEE OUI, from the public MA-L registry.
BASLER_OUI = "00:30:53"

#: Only USB cameras are candidates, and the exclusion is not academic: this
#: program serves a device that calls itself a Basler GigE camera, so pylon
#: on this same machine enumerates it alongside the real hardware. Without
#: this, a second instance -- or a restart while the first is still up --
#: can pick the emulator and serve it to itself.
USB_DEVICE_CLASS = "BaslerUsb"

#: Formats this emulator has a PFNC identifier for. The camera also offers
#: packed variants -- Mono12p -- which are a different payload size for the
#: same name, so they are dropped rather than advertised and then mis-sized.
USABLE_FORMATS = ("Mono8", "Mono10", "Mono12", "Mono16", "RGB8", "BGR8",
                  "BayerGR8", "BayerRG8", "BayerGB8", "BayerBG8",
                  "BayerGR12", "BayerRG12", "BayerGB12", "BayerBG12")

#: How long a cached read of the camera's own settings stays good. Every
#: register read a client makes lands in get_camera_settings() on the control
#: thread, which has about 20 ms before a command times out, and ten GenApi
#: reads over USB do not reliably fit in that.
SETTINGS_CACHE_SECONDS = 0.5

#: What a grab is given before it is called a failure. Longer than the
#: slowest frame this camera can produce at its longest exposure would be
#: pointless -- the stream thread has nothing else to do -- but it must not
#: be shorter than one frame interval at a low frame rate.
GRAB_TIMEOUT_MS = 5000


def open_camera(serial=None):
    """
    The named camera, or the only one if there is just one.

    Failing here with the list of what *is* attached beats failing later with
    a pylon exception about a device that was never going to be found.
    """
    factory = pylon.TlFactory.GetInstance()
    devices = [d for d in factory.EnumerateDevices()
               if d.GetDeviceClass() == USB_DEVICE_CLASS]
    if not devices:
        raise RuntimeError("no Basler USB camera found")
    if serial is not None:
        for device in devices:
            if device.GetSerialNumber() == serial:
                return pylon.InstantCamera(factory.CreateDevice(device))
        raise RuntimeError(
            "no Basler USB camera with serial %r; attached: %s"
            % (serial, ", ".join("%s (%s)" % (d.GetModelName(),
                                              d.GetSerialNumber())
                                 for d in devices)))
    if len(devices) > 1:
        raise RuntimeError(
            "several Basler USB cameras attached, so --device-serial is "
            "needed: %s"
            % ", ".join(d.GetSerialNumber() for d in devices))
    return pylon.InstantCamera(factory.CreateDevice(devices[0]))


class BaslerCamera(EmulatedCamera):

    def __init__(self, serial=None, reset=False, **kwds):
        self.cam = open_camera(serial)
        self.cam.Open()

        # Not every ace has binning. The colour acA1440-220uc does not, while
        # the mono acA1440-220um this was written against does, and the node
        # is in the map either way -- present but not available. Reading it
        # raises, which is how the example used to refuse to start on that
        # camera, so this is settled before anything else touches it.
        self.has_binning = (self._available("BinningHorizontal")
                            and self._available("BinningVertical"))
        if not self.has_binning:
            log.info("this camera has no binning; serving it without "
                     "BinningHorizontal or BinningVertical")

        if reset:
            self._reset_geometry()

        self.vendor_name = self.cam.DeviceVendorName.Value
        self.model_name = self.cam.DeviceModelName.Value
        self.serial_number = self.cam.DeviceSerialNumber.Value

        self.n_grab_failures = 0
        self._cached = {}
        self._cached_at = 0.0

        sensor_width = self.cam.WidthMax.Value
        sensor_height = self.cam.HeightMax.Value

        # Declared per instance rather than on the class, because every bound
        # in them belongs to this particular camera. Assigning here shadows
        # the class attribute that __init__ reads a few lines down.
        self.extra_features = self._describe_features()

        super().__init__(width=self.cam.Width.Value,
                         height=self.cam.Height.Value,
                         pixel_format=self.cam.PixelFormat.Value,
                         pixel_formats=self._pixel_formats(),
                         sensor_width=sensor_width,
                         sensor_height=sensor_height,
                         frame_rate=self.cam.ResultingFrameRate.Value,
                         # The absolute bound validate() enforces, not the
                         # reachable one: shrinking the ROI raises what this
                         # sensor will do, so pinning it to the rate measured
                         # at startup would refuse a rate the camera can
                         # actually reach. What is reachable right now is
                         # AcquisitionFrameRateMax, wired up below.
                         max_frame_rate=10000.0,
                         roi=True,
                         offset=(self.cam.OffsetX.Value,
                                 self.cam.OffsetY.Value),
                         roi_bounds=self._roi_bounds(sensor_width,
                                                     sensor_height),
                         **kwds)

        # The ceiling moves with the ROI, the format, binning and the
        # exposure, so the frame rate's own max is a pointer to a feature
        # this class maintains rather than a number fixed at startup.
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

    def _reset_geometry(self):
        """
        Full frame, no binning.

        Off by default, because a camera is entitled to keep the settings it
        was left in and a vendor's own viewer does not reset one either. But
        those settings outlive the process that made them, so a run that
        starts 128 pixels wide because some earlier experiment left it that
        way looks exactly like a bug in here.
        """
        if self.has_binning:
            self.cam.BinningHorizontal.Value = self.cam.BinningHorizontal.Min
            self.cam.BinningVertical.Value = self.cam.BinningVertical.Min
        self.cam.OffsetX.Value = 0
        self.cam.OffsetY.Value = 0
        self.cam.Width.Value = self.cam.Width.Max
        self.cam.Height.Value = self.cam.Height.Max

    # --- what this camera can do ----------------------------------------

    def _available(self, name):
        """
        Whether this camera implements a feature at all.

        A node absent from the model is still in the node map, so asking the
        map is the only way that does not raise: reading the value of an
        unavailable node throws AccessException.
        """
        node = self.cam.GetNodeMap().GetNode(name)
        return node is not None and genicam.IsAvailable(node)

    def _pixel_formats(self):
        available = [name for name in self.cam.PixelFormat.Symbolics
                     if name in USABLE_FORMATS]
        dropped = [name for name in self.cam.PixelFormat.Symbolics
                   if name not in USABLE_FORMATS]
        if dropped:
            log.info("not advertising %s: no PFNC identifier for %s",
                     ", ".join(dropped),
                     "them" if len(dropped) > 1 else "it")
        return available

    def _roi_bounds(self, sensor_width, sensor_height):
        """
        (min, max, inc) for each of the four ROI features.

        The maxima are the sensor's, not the camera's current ones: pylon
        reports Width.Max as what fits at the *present* offset, so reading it
        now and publishing it forever would tell a client that a camera at
        offset 8 can never be more than 1448 wide even after the offset moves
        back to zero. The camera still enforces the real interlock on every
        write; this is only what the client is told up front.
        """
        return {
            "Width": (self.cam.Width.Min, sensor_width, self.cam.Width.Inc),
            "Height": (self.cam.Height.Min, sensor_height,
                       self.cam.Height.Inc),
            "OffsetX": (0, sensor_width - self.cam.Width.Min,
                        self.cam.OffsetX.Inc),
            "OffsetY": (0, sensor_height - self.cam.Height.Min,
                        self.cam.OffsetY.Inc),
        }

    def _describe_features(self):
        exposure = self.cam.ExposureTime
        features = [
            FloatFeature("ExposureTime", "Exposure time",
                         "AcquisitionControl", "RW",
                         default=exposure.Value, min=exposure.Min,
                         max=exposure.Max, unit="us"),
            EnumFeature("ExposureAuto", "Automatic exposure",
                        "AcquisitionControl", "RW",
                        entries={name: index for index, name
                                 in enumerate(self.cam.ExposureAuto.Symbolics)},
                        default=self.cam.ExposureAuto.Value),
            FloatFeature("AcquisitionFrameRateMax",
                         "Fastest frame rate the current settings sustain",
                         "AcquisitionControl", "RO",
                         invalidated_by=self._moves_the_ceiling(),
                         default=0.0, min=0.0, max=1e6, unit="Hz"),
        ]
        if self.has_binning:
            features += [
                IntFeature("BinningHorizontal", "Horizontal binning factor",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=self.cam.BinningHorizontal.Value,
                           min=self.cam.BinningHorizontal.Min,
                           max=self.cam.BinningHorizontal.Max),
                IntFeature("BinningVertical", "Vertical binning factor",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=self.cam.BinningVertical.Value,
                           min=self.cam.BinningVertical.Min,
                           max=self.cam.BinningVertical.Max),
            ]
        return tuple(features)

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

    # --- lifecycle -------------------------------------------------------

    def close(self):
        if self.cam.IsGrabbing():
            self.cam.StopGrabbing()
        self.cam.Close()

    def _pause_grabbing(self):
        """
        Stop the grab so a geometry write is legal, and let next_frame() start
        it again.

        The emulator already refuses these writes while a client is
        acquiring, but grabbing here is started on demand and outlives one
        client's acquisition, so the camera can still be streaming when the
        write arrives.
        """
        if self.cam.IsGrabbing():
            self.cam.StopGrabbing()

    # --- frames ----------------------------------------------------------

    def next_frame(self):
        if not self.cam.IsGrabbing():
            # LatestImageOnly, so a client that stops asking for a while and
            # comes back is given what the camera sees now rather than the
            # oldest frame in a queue that filled while it was away.
            self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        result = self.cam.RetrieveResult(GRAB_TIMEOUT_MS,
                                         pylon.TimeoutHandling_Return)
        if result is None or not result.IsValid():
            self.n_grab_failures += 1
            return None
        with result:
            if not result.GrabSucceeded():
                self.n_grab_failures += 1
                log.warning("grab failed: %s", result.GetErrorDescription())
                return None
            return bytes(result.GetBuffer())

    # --- settings --------------------------------------------------------

    def _refresh_frame_rate_ceiling(self):
        """
        What the camera would run at with no rate limit applied.

        ResultingFrameRate is the honest ceiling only while
        AcquisitionFrameRateEnable is off; once a client sets a rate, it
        reports that rate back and would drag the ceiling down to meet it.
        So the limit is lifted for the read when that is safe, and when it is
        not, the last known ceiling stands rather than being replaced by a
        number that is really just the current setting.
        """
        limited = self.cam.AcquisitionFrameRateEnable.Value
        if limited and self.acquiring:
            return
        try:
            if limited:
                self.cam.AcquisitionFrameRateEnable.Value = False
            self.settings["AcquisitionFrameRateMax"] = \
                self.cam.ResultingFrameRate.Value
        finally:
            if limited:
                self.cam.AcquisitionFrameRateEnable.Value = True

    def _clamp(self, node, value):
        """
        Fit a value to what the node will take right now.

        Worth doing even though the camera would refuse an illegal write:
        the refusal reaches the client as a protocol error, while a rounded
        value reaches it as a reading of what actually happened.
        """
        value = max(node.Min, min(node.Max, value))
        try:
            inc = node.Inc
        except genicam.GenericException:
            # Not every node has one, and asking one that does not is an
            # exception rather than a None -- which reached the client as
            # "AcquisitionFrameRate rejected by the camera: node does not
            # have an increment", refusing a perfectly good write.
            inc = None
        if inc and inc > 1:
            value = node.Min + ((value - node.Min) // inc) * inc
        return value

    def _apply_roi(self, changed):
        """
        Move the window, offsets first to zero.

        The camera interlocks size against position -- Width.Max is the
        sensor width less the current OffsetX -- so the same pair of numbers
        is accepted or refused depending on the order they arrive in. Going
        through the origin costs two extra writes and is always legal.
        """
        self._pause_grabbing()
        self.cam.OffsetX.Value = 0
        self.cam.OffsetY.Value = 0
        self.cam.Width.Value = self._clamp(self.cam.Width,
                                           self.settings["Width"])
        self.cam.Height.Value = self._clamp(self.cam.Height,
                                            self.settings["Height"])
        self.cam.OffsetX.Value = self._clamp(self.cam.OffsetX,
                                             self.settings["OffsetX"])
        self.cam.OffsetY.Value = self._clamp(self.cam.OffsetY,
                                             self.settings["OffsetY"])
        self._publish_geometry()

    def _publish_geometry(self):
        """
        Put back what the camera actually did.

        A camera that rounded a width to its increment, or moved an offset to
        keep the window on the sensor, has to be believed here -- the payload
        size the client is about to be told is computed from these.
        """
        for name in ("Width", "Height", "OffsetX", "OffsetY",
                     "BinningHorizontal", "BinningVertical"):
            if name in self.settings:
                self.settings[name] = getattr(self.cam, name).Value

    def set_camera_settings(self, changed):
        if {"Width", "Height", "OffsetX", "OffsetY"} & set(changed):
            self._apply_roi(changed)

        for name in ("BinningHorizontal", "BinningVertical"):
            if name in changed:
                self._pause_grabbing()
                node = getattr(self.cam, name)
                # Not clamped, unlike the continuous settings. Binning is a
                # choice rather than a magnitude, so the nearest legal value
                # is a different setting, and a camera whose axes constrain
                # each other -- the Alvium will not take horizontal 1 while
                # vertical is 2 -- would otherwise answer a client's 1x1 with
                # a silent 2x1 and call it success. The camera's refusal
                # reaches the client as an error on the write it got wrong.
                node.Value = changed[name]
                # Binning changes what the sensor's full width means, and the
                # camera resizes the ROI under us to suit. Republishing is
                # not optional: the client sizes its buffers from Width and
                # Height, and it is about to read them.
                self._publish_geometry()

        if "PixelFormat" in changed:
            self._pause_grabbing()
            self.cam.PixelFormat.Value = changed["PixelFormat"]

        if "ExposureAuto" in changed:
            self.cam.ExposureAuto.Value = changed["ExposureAuto"]

        if "ExposureTime" in changed:
            # Allowed even under automatic exposure, where the camera will
            # overwrite it on the next frame. Refusing would be defensible,
            # but a client that sets an exposure and then reads back what the
            # algorithm chose is doing something reasonable.
            self.cam.ExposureTime.Value = self._clamp(self.cam.ExposureTime,
                                                      changed["ExposureTime"])

        if "AcquisitionFrameRate" in changed:
            # The rate is a *limit* on this camera and does nothing until it
            # is switched on. Left off, the camera free runs and a client
            # that set 10 fps would see 227 and no error.
            self.cam.AcquisitionFrameRateEnable.Value = True
            self.cam.AcquisitionFrameRate.Value = self._clamp(
                self.cam.AcquisitionFrameRate, changed["AcquisitionFrameRate"])

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
            "ExposureTime": self.cam.ExposureTime.Value,
            "ExposureAuto": self.cam.ExposureAuto.Value,
            "AcquisitionFrameRate": self.cam.ResultingFrameRate.Value,
            "Width": self.cam.Width.Value,
            "Height": self.cam.Height.Value,
            "OffsetX": self.cam.OffsetX.Value,
            "OffsetY": self.cam.OffsetY.Value,
            "AcquisitionFrameRateMax":
                self.settings["AcquisitionFrameRateMax"],
        }
        if self.has_binning:
            self._cached["BinningHorizontal"] = \
                self.cam.BinningHorizontal.Value
            self._cached["BinningVertical"] = self.cam.BinningVertical.Value
        self._cached_at = now
        return self._cached


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Basler camera -> GigE")
    parser.add_argument("--interface", default="eth0",
                        help="network interface to serve on")
    parser.add_argument("--device-serial", default=None,
                        help="which camera, when more than one is attached")
    parser.add_argument("--name", default="",
                        help="user-defined camera name. Clients show this and "
                             "can select on it, so it is how you tell two "
                             "otherwise identical cameras apart")
    parser.add_argument("--mac", default="",
                        help="MAC address to report. Defaults to Basler's OUI "
                             "%s with a tail derived from the camera's serial "
                             "-- pylon will not enumerate a device without "
                             "it. Pass the interface's own with --mac none"
                             % BASLER_OUI)
    parser.add_argument("--reset", action="store_true",
                        help="put the camera back to full frame with no "
                             "binning before serving. Settings outlive the "
                             "process that made them, so an ROI left behind "
                             "by an earlier run is otherwise inherited "
                             "silently")
    parser.add_argument("--list", action="store_true",
                        help="print the attached Basler cameras, then exit")
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
        for device in pylon.TlFactory.GetInstance().EnumerateDevices():
            # The class is printed because anything that is not BaslerUsb is
            # listed for information and cannot be served -- including,
            # confusingly, a device this program is serving right now.
            print("%-16s %-18s %-12s %s" % (device.GetVendorName(),
                                            device.GetModelName(),
                                            device.GetSerialNumber(),
                                            device.GetDeviceClass()))
        sys.exit(0)

    try:
        netif.interface_info(args.interface)
    except netif.InterfaceError as e:
        parser.error(str(e))

    try:
        camera = BaslerCamera(serial=args.device_serial,
                              reset=args.reset)
    except (RuntimeError, genicam.GenericException) as e:
        parser.error(str(e))

    if args.mac.lower() in ("none", "interface"):
        mac = None
    elif args.mac:
        try:
            mac = netif.parse_mac(args.mac)
        except ValueError as e:
            parser.error(str(e))
    else:
        mac = netif.mac_from_serial(BASLER_OUI, camera.serial_number)

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
        print("stats:", server.stats, "grab failures:", camera.n_grab_failures)
