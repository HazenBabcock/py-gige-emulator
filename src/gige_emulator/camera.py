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
                 frame_rate=10.0):

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
                                  sensor_height or height, frame_rate)
        self.feature_set.extend(list(self.extra_features))

        self.settings = self.feature_set.defaults()

        self.acquiring = False
        self.geometry = None      # latched at AcquisitionStart

    # --- core features ---------------------------------------------------

    def _build_core_features(self, width, height, pixel_format, pixel_formats,
                             sensor_width, sensor_height, frame_rate):
        add = self.feature_set.add

        add(IntFeature("SensorWidth", "Sensor width in pixels",
                       "ImageFormatControl", "RO", default=sensor_width,
                       min=1, max=sensor_width))
        add(IntFeature("SensorHeight", "Sensor height in pixels",
                       "ImageFormatControl", "RO", default=sensor_height,
                       min=1, max=sensor_height))
        add(IntFeature("Width", "Image width in pixels",
                       "ImageFormatControl", "RO", default=width,
                       min=1, max=sensor_width))
        add(IntFeature("Height", "Image height in pixels",
                       "ImageFormatControl", "RO", default=height,
                       min=1, max=sensor_height))
        add(EnumFeature("PixelFormat", "Pixel format",
                        "ImageFormatControl", "RO",
                        entries={name: c.PIXEL_FORMAT_NAMES[name]
                                 for name in pixel_formats},
                        default=pixel_format))

        # PayloadSize is computed by the device and never user supplied. The
        # client sizes its receive buffer from it and then rejects any packet
        # id past what that buffer implies, with no error -- so an
        # inconsistent value here shows up as a black image, not a message.
        add(IntFeature("PayloadSize", "Bytes transferred per image",
                       "ImageFormatControl", "RO",
                       default=c.payload_size(width, height,
                                              c.PIXEL_FORMAT_NAMES[pixel_format]),
                       min=1, max=0xFFFFFFFF))

        add(EnumFeature("AcquisitionMode", "Acquisition mode",
                        "AcquisitionControl", "RW",
                        entries={"Continuous": 1, "SingleFrame": 2},
                        default="Continuous"))
        add(CommandFeature("AcquisitionStart", "Start acquisition",
                           "AcquisitionControl", "RW"))
        add(CommandFeature("AcquisitionStop", "Stop acquisition",
                           "AcquisitionControl", "RW"))
        add(FloatFeature("AcquisitionFrameRate", "Frames per second",
                         "AcquisitionControl", "RW", default=float(frame_rate),
                         min=0.001, max=1000.0, unit="Hz"))

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

        Called on the stream thread with no locks held; blocking is fine.
        Return None to indicate that no frame is ready yet.
        """
        raise NotImplementedError("subclasses must implement next_frame()")

    def set_camera_settings(self, changed):
        """
        Apply settings the client just changed. `changed` holds only the
        features that moved, not the whole dictionary; self.settings always
        holds the full current state.

        Raising rolls the register write back and answers the client with
        ACCESS_DENIED rather than reporting a success that did not happen.
        """

    def get_camera_settings(self):
        """
        Return current values read back from the hardware. A partial
        dictionary is fine; anything absent keeps its stored value.
        """
        return {}
