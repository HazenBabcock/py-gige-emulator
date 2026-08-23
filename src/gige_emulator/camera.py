#
# The class a user subclasses to wire in a real camera.
#
# Three hooks, and everything else is handled:
#
#   next_frame()           -- runs on the stream thread, holds no locks, and
#                             may block for as long as an exposure takes
#   set_camera_settings()  -- runs inline on the control thread when the
#                             client writes a feature
#   get_camera_settings()  -- runs inline on the control thread when the
#                             client reads a feature
#
# The two settings hooks share the control thread with GVCP, so they have a
# latency budget. Aravis gives a command 500 ms and retries five times on
# this build, but a build with fast heartbeats enabled cuts that to 25 ms and
# three retries. Treat 20 ms as the ceiling.
#

from dataclasses import dataclass

from . import constants as c
from .features import (CommandFeature, EnumFeature, FeatureSet, FloatFeature,
                       IntFeature)


@dataclass
class Frame:
    """
    Wraps frame data when the physical camera supplies its own identity.
    next_frame() may also just return bytes.
    """
    data: bytes
    timestamp_ns: int = None
    frame_id: int = None


class EmulatedCamera(object):

    #: Features beyond the core set. Declared on the subclass.
    extra_features = ()

    def __init__(self, width=640, height=480, pixel_format="Mono8",
                 pixel_formats=None, sensor_width=None, sensor_height=None,
                 frame_rate=10.0, max_frame_rate=1000.0,
                 roi=False, roi_bounds=None, offset=(0, 0)):

        if pixel_formats is None:
            pixel_formats = [pixel_format]
        if pixel_format not in pixel_formats:
            pixel_formats = [pixel_format] + list(pixel_formats)

        unknown = [name for name in pixel_formats
                   if name not in c.PIXEL_FORMAT_NAMES]
        if unknown:
            raise ValueError("unsupported pixel format(s): %s"
                             % ", ".join(unknown))

        self.feature_set = FeatureSet()
        self._build_core_features(width, height, pixel_format, pixel_formats,
                                  sensor_width or width,
                                  sensor_height or height, frame_rate,
                                  max_frame_rate, roi, roi_bounds, offset)
        self.feature_set.extend(list(self.extra_features))

        self.settings = self.feature_set.defaults()

        self.acquiring = False
        self.geometry = None      # latched at AcquisitionStart

    # --- core features ---------------------------------------------------

    def _build_core_features(self, width, height, pixel_format, pixel_formats,
                             sensor_width, sensor_height, frame_rate,
                             max_frame_rate=1000.0, roi=False,
                             roi_bounds=None, offset=(0, 0)):
        add = self.feature_set.add
        bounds = dict(roi_bounds or {})

        def limits(name, low, high):
            """(min, max, inc) for one geometry feature, from the camera if
            it said, and the widest thing that could be true if it did not."""
            low, high, inc = bounds.get(name, (low, high, 1))
            return {"min": low, "max": high, "inc": inc}

        add(IntFeature("SensorWidth", "Sensor width in pixels",
                       "ImageFormatControl", "RO", default=sensor_width,
                       min=1, max=sensor_width))
        add(IntFeature("SensorHeight", "Sensor height in pixels",
                       "ImageFormatControl", "RO", default=sensor_height,
                       min=1, max=sensor_height))
        # Read only unless the camera says it can take an ROI. A writable
        # Width on a device that cannot honour one is worse than no control:
        # arv-viewer builds a spin box from the feature and does not grey out
        # read-only ones, so the user gets something that looks adjustable
        # and answers every write with a protocol error.
        #
        # The increments matter as much as the bounds. Real sensors step
        # their ROI in fours or eights, and a client that does not know that
        # sends 1441 and gets a refusal it cannot explain.
        add(IntFeature("Width", "Image width in pixels",
                       "ImageFormatControl", "RW" if roi else "RO",
                       affects_payload=True, default=width,
                       **limits("Width", 1, sensor_width)))
        add(IntFeature("Height", "Image height in pixels",
                       "ImageFormatControl", "RW" if roi else "RO",
                       affects_payload=True, default=height,
                       **limits("Height", 1, sensor_height)))
        if roi:
            # affects_payload, although moving a window does not resize it.
            # The flag carries two behaviours and this needs both: refuse
            # while acquiring, because a client has already sized its buffers
            # and computed its packet count, and re-publish the geometry
            # afterwards, because a camera asked for an offset it cannot
            # reach at the current width may answer by moving the width too.
            add(IntFeature("OffsetX", "Left edge of the region of interest",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=offset[0],
                           **limits("OffsetX", 0, max(0, sensor_width - 1))))
            add(IntFeature("OffsetY", "Top edge of the region of interest",
                           "ImageFormatControl", "RW", affects_payload=True,
                           default=offset[1],
                           **limits("OffsetY", 0, max(0, sensor_height - 1))))
        # Writable exactly when there is something to choose. Advertising
        # several formats on a read-only feature is what this used to do, and
        # a client then draws a populated combo box that refuses every
        # selection -- arv-viewer builds its control panel from the feature
        # names and does not grey out read-only ones, so the user gets a
        # working-looking control and a write-protect error.
        #
        # Nothing else is needed to make the switch take effect:
        # payload_size() already reads the format out of settings, the bridge
        # already calls refresh_geometry() after a write, and
        # affects_payload already refuses one mid-acquisition.
        add(EnumFeature("PixelFormat", "Pixel format", "ImageFormatControl",
                        "RW" if len(pixel_formats) > 1 else "RO",
                        affects_payload=True,
                        entries={name: c.PIXEL_FORMAT_NAMES[name]
                                 for name in pixel_formats},
                        default=pixel_format))

        # PayloadSize is computed by the device and never user supplied. The
        # client sizes its receive buffer from it and then rejects any packet
        # id past what that buffer implies, with no error -- so an
        # inconsistent value here shows up as a black image, not a message.
        # TransportLayerControl, not ImageFormatControl: this is a count of
        # bytes on the stream channel rather than anything about the image's
        # shape. Checked against a vendor-authored XML, which files it the
        # same way.
        add(IntFeature("PayloadSize", "Bytes transferred per image",
                       "TransportLayerControl", "RO",
                       default=c.payload_size(width, height,
                                              c.PIXEL_FORMAT_NAMES[pixel_format]),
                       min=1, max=0xFFFFFFFF))

        # The stream channel's own registers, published as features rather
        # than left in the bootstrap page alone.
        #
        # Aravis needs neither -- it knows the bootstrap layout and writes
        # the registers directly -- but a client that works only from the XML
        # cannot. pylon looks for a GevSCPSPacketSize node, does not find
        # one, and says so before streaming with a packet size it guessed:
        #
        #   WARN: Packet size not changed because detection failed
        #   INFO: Using default packet size as there is no GevSCPSPacketSize
        #         node in 'Basler acA1440-220um#...'
        #
        # Which also means it can never negotiate a smaller MTU or use jumbo
        # frames with this device. Aravis skips any name already in the
        # document, so it uses these instead of injecting its own.
        add(IntFeature("GevSCPSPacketSize",
                       "Bytes per stream packet, including headers",
                       "TransportLayerControl", "RW", transport=True,
                       address=c.BS_SC0_PACKET_SIZE, lsb=31, msb=16,
                       default=c.DEFAULT_PACKET_SIZE, min=64, max=16384,
                       unit="B"))
        add(IntFeature("GevSCPD", "Delay between stream packets, in timestamp "
                       "ticks", "TransportLayerControl", "RW", transport=True,
                       address=c.BS_SC0_PACKET_DELAY,
                       default=0, min=0, max=0xFFFFFFFF))

        # SingleFrame is one frame per AcquisitionStart: the stream thread
        # clears self.acquiring once it has sent one, so the client has to
        # start again for the next. next_frame() sees no difference between
        # the two modes -- it is asked for a frame and returns one either
        # way -- so a camera needs to do nothing to support this.
        add(EnumFeature("AcquisitionMode", "Acquisition mode",
                        "AcquisitionControl", "RW",
                        entries={"Continuous": 1, "SingleFrame": 2},
                        default="Continuous"))
        add(CommandFeature("AcquisitionStart", "Start acquisition",
                           "AcquisitionControl", "RW"))
        add(CommandFeature("AcquisitionStop", "Stop acquisition",
                           "AcquisitionControl", "RW"))
        # max_frame_rate is the widest this device can ever go, across every
        # mode it has. It used to be a flat 1000.0 for every camera ever
        # built on this class, which a client believes: arv-viewer will
        # happily offer 500 fps on a sensor whose fastest readout is 147.
        #
        # A camera whose ceiling moves with the selected readout mode points
        # p_max at a feature it maintains, and this stays as the absolute
        # bound that validate() enforces. See examples/pi_camera.py.
        add(FloatFeature("AcquisitionFrameRate", "Frames per second",
                         "AcquisitionControl", "RW", default=float(frame_rate),
                         min=0.001, max=float(max_frame_rate), unit="Hz"))

    # --- derived ---------------------------------------------------------

    def pixel_format_value(self):
        return c.PIXEL_FORMAT_NAMES[self.settings["PixelFormat"]]

    def payload_size(self):
        return c.payload_size(self.settings["Width"], self.settings["Height"],
                              self.pixel_format_value())

    def latch_geometry(self):
        """
        Snapshot the geometry that the whole acquisition will use.

        The client computes how many packets to expect from the payload size
        it read at start, so changing width mid-stream would desynchronise it
        and every packet past the old count would be dropped silently.
        """
        self.geometry = {
            "width": self.settings["Width"],
            "height": self.settings["Height"],
            "pixel_format": self.pixel_format_value(),
            "payload": self.payload_size(),
        }
        return self.geometry

    # --- the three hooks -------------------------------------------------

    def next_frame(self):
        """
        Return the next frame as bytes, a buffer, or a Frame.

        Called on the stream thread with no locks held. Blocking is not just
        allowed, it is the point: this method sets the frame rate. The stream
        thread has no timer of its own, so it sends frames exactly as fast as
        this returns them.

        For a real camera that means blocking until the sensor delivers,
        which paces the stream for free and at the camera's true rate. A
        camera with no physical timing -- a test pattern, a file reader --
        has to pace itself, or the stream thread will saturate a core and
        flood the network.

        Return None if no frame is ready yet; the stream thread will retry.
        """
        raise NotImplementedError("subclasses must implement next_frame()")

    def set_camera_settings(self, changed):
        """
        Apply settings the client just changed. `changed` holds only the
        features that moved, not the whole dictionary.

        Note the ordering: self.settings has **already** been updated with the
        new values by the time this is called. So comparing a requested value
        against self.settings here always says "unchanged" -- if you need to
        know what the hardware is currently configured for, track that
        yourself. examples/pi_camera.py keeps an _applied_binning for exactly
        this reason.

        Raising rolls the register write back and answers the client with an
        error rather than reporting a success that did not happen.
        """

    def get_camera_settings(self):
        """
        Return current values read back from the hardware. A partial
        dictionary is fine; anything absent keeps its stored value.
        """
        return {}
