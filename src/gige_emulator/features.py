#
# The feature spec: one Python declaration per camera feature, from which
# both the register map and the GenICam XML are derived.
#
# The point of generating rather than hand-writing the XML is that the two
# cannot then disagree. A hand-written XML with an <Address> that does not
# match what the device actually stores fails in a way that looks like a
# client bug.
#
# Every feature occupies a whole register. Bit packing via MaskedIntReg is
# deliberately not supported: GenICam numbers mask bits from the MSB for a
# big endian register, which is a reliable source of off-by-sixteen errors,
# and no camera feature here needs it.
#

import difflib
import logging
import struct
from dataclasses import dataclass, field

from . import constants as c

log = logging.getLogger(__name__)

#: Category names from the GenICam Standard Features Naming Convention. Used
#: only to spot typos -- a category outside this set is allowed, since a
#: domain the convention does not cover is a real thing to have, and clients
#: render any category fine. It is not a whitelist.
SFNC_CATEGORIES = frozenset((
    "DeviceControl", "ImageFormatControl", "AcquisitionControl",
    "AnalogControl", "LUTControl", "ColorTransformationControl",
    "CounterAndTimerControl", "DigitalIOControl", "EventControl",
    "ChunkDataControl", "FileAccessControl", "TransportLayerControl",
    "UserSetControl", "SequencerControl", "SoftwareSignalControl",
    "ActionControl", "TestControl", "SourceControl", "ScanNDControl",
))


#: Feature names defined by the naming convention, which are emitted with
#: NameSpace="Standard". Declaring a name standard asserts the convention's
#: meaning and units for it, so a name only belongs here once that has been
#: checked -- this list is short on purpose and grows deliberately.
#:
#: `GainRaw` is deliberately absent. It is the GenICam 1.x integer form; the
#: convention's gain feature is `Gain`, a float in dB. GainRaw is still the
#: right name for a device whose gain control is in units it cannot report --
#: a webcam's is whatever V4L2 hands back, and zero has no dB value -- but it
#: is that device's own name, not the convention's.
SFNC_FEATURES = frozenset((
    "Width", "Height", "SensorWidth", "SensorHeight", "PixelFormat",
    "OffsetX", "OffsetY", "BinningHorizontal", "BinningVertical",
    "DecimationHorizontal", "DecimationVertical", "ReverseX", "ReverseY",
    "TestPattern", "PayloadSize",
    "AcquisitionMode", "AcquisitionStart", "AcquisitionStop",
    "AcquisitionFrameRate", "ExposureTime", "ExposureAuto", "ExposureMode",
    "TriggerMode", "TriggerSource", "TriggerSoftware", "TriggerSelector",
    "Gain", "GainAuto", "GainSelector", "BlackLevel", "Gamma",
    "BalanceRatio", "BalanceRatioSelector",
    "DeviceVendorName", "DeviceModelName", "DeviceVersion",
    "DeviceSerialNumber", "DeviceUserID", "DeviceReset", "DeviceTemperature",
))


#: Enumeration *entry* names the convention defines. A vendor XML marks these
#: Standard alongside the enumeration itself, and they are a separate
#: vocabulary from the feature names -- "Mono8" is a value, not a feature.
SFNC_ENUM_ENTRIES = frozenset((
    "Mono8", "Mono10", "Mono12", "Mono14", "Mono16",
    "RGB8", "RGB8Packed", "BGR8", "BayerRG8", "BayerGR8", "BayerGB8",
    "BayerBG8", "YCbCr422_8",
    "Continuous", "SingleFrame", "MultiFrame",
    "Off", "On", "Once", "Continuous",
))


class FeatureError(Exception):
    pass


@dataclass
class Feature:
    """
    One camera feature: a register, an XML node, and a key in the settings
    dict, all from this single declaration.

    `category` decides only where a client's feature tree puts this, but that
    is what someone hunting for a control actually navigates. The names come
    from the GenICam Standard Features Naming Convention, not from GigE
    Vision, which specifies the wire protocol and says nothing about feature
    names. Four questions, in order:

    1. What the camera *is* -- serial, version, temperature, reset?
       `DeviceControl`.
    2. The buffer's shape or encoding -- Width, Height, OffsetX/Y,
       PixelFormat, Binning, Decimation, TestPattern? `ImageFormatControl`.
       Note `PayloadSize` is *not* one of these despite following from them:
       it counts bytes on the stream channel, so it is
       `TransportLayerControl`.
    3. *Time* -- when, how long, how often? AcquisitionMode, Start/Stop,
       AcquisitionFrameRate, ExposureTime, and all triggering.
       `AcquisitionControl`.
    4. *Amplitude*, before digitization -- Gain, BlackLevel, Gamma,
       BalanceRatio, Sharpness? `AnalogControl`.

    **Exposure and gain are not in the same category**, which is the one that
    catches people. They are tuned together and every UI shows them side by
    side, but exposure is time and gain is amplitude, so they land in
    `AcquisitionControl` and `AnalogControl` respectively.

    Anything else the convention names is fine too -- LUTControl,
    DigitalIOControl, CounterAndTimerControl, TransportLayerControl,
    UserSetControl and so on -- and inventing one for a domain the
    convention does not cover is legitimate. `SFNC_CATEGORIES` lists the ones
    that are not warned about; a category outside it is logged once, because
    the common case is a typo silently creating a category of one.
    """

    name: str
    description: str = ""
    category: str = "DeviceControl"
    access: str = "RW"
    address: int = None          # assigned by FeatureSet.allocate()

    #: True if writing this changes how many bytes a frame occupies. The
    #: client sizes its buffers from PayloadSize when acquisition starts and
    #: silently drops any packet past the count that implies, so these are
    #: refused while streaming. GenICam calls the same idea TLParamsLocked.
    affects_payload: bool = False

    #: Names of features whose change makes this one's stored value wrong.
    #: Emitted as <pInvalidator>, which is the only thing that makes a client
    #: re-read. Without it a GUI shows the value it fetched when it built its
    #: property tree, however diligently the device updates the register --
    #: the client has no reason to ask again.
    #:
    #: This is about a value moving on its own, not about a value being
    #: writable. AcquisitionFrameRate is invalidated by ExposureTime because
    #: a long exposure drags the achievable rate down with it.
    invalidated_by: tuple = ()

    size = 4
    settable = True

    def encode(self, value):
        raise NotImplementedError

    def decode(self, raw):
        raise NotImplementedError

    def validate(self, value):
        return value


@dataclass
class IntFeature(Feature):
    default: int = 0
    min: int = 0
    max: int = 0xFFFFFFFF
    inc: int = 1
    unit: str = ""

    #: Name of another feature that supplies this one's bound at runtime.
    #: Emitted as <pMin>/<pMax> in place of the literal, because GenICam
    #: takes one or the other and never both.
    #:
    #: `min`/`max` stay meaningful when these are set: they are the widest
    #: the device can ever go, and validate() still enforces them, while the
    #: pointed-at feature carries what is reachable right now. A sensor whose
    #: fastest mode does 147 fps has max=147 forever and a pMax that follows
    #: whichever mode is selected.
    p_min: str = None
    p_max: str = None

    size = 4

    def encode(self, value):
        return struct.pack(">I", int(value) & 0xFFFFFFFF)

    def decode(self, raw):
        return struct.unpack(">I", raw)[0]

    def validate(self, value):
        value = int(value)
        if value < self.min or value > self.max:
            raise FeatureError("%s: %d outside [%d, %d]"
                               % (self.name, value, self.min, self.max))
        return value


@dataclass
class FloatFeature(Feature):
    default: float = 0.0
    min: float = 0.0
    max: float = 1e12
    unit: str = ""

    #: See IntFeature.p_min. Same meaning, same reason min/max stay set.
    p_min: str = None
    p_max: str = None

    size = 8

    def encode(self, value):
        return struct.pack(">d", float(value))

    def decode(self, raw):
        return struct.unpack(">d", raw)[0]

    def validate(self, value):
        value = float(value)
        if value < self.min or value > self.max:
            raise FeatureError("%s: %g outside [%g, %g]"
                               % (self.name, value, self.min, self.max))
        return value


@dataclass
class EnumFeature(Feature):
    entries: dict = field(default_factory=dict)
    default: str = None

    size = 4

    def __post_init__(self):
        if not self.entries:
            raise FeatureError("%s: enumeration needs entries" % self.name)
        if self.default is None:
            self.default = next(iter(self.entries))
        if self.default not in self.entries:
            raise FeatureError("%s: default %r is not an entry"
                               % (self.name, self.default))

    def encode(self, value):
        if isinstance(value, str):
            if value not in self.entries:
                raise FeatureError("%s: %r is not a valid entry"
                                   % (self.name, value))
            value = self.entries[value]
        return struct.pack(">I", int(value) & 0xFFFFFFFF)

    def decode(self, raw):
        value = struct.unpack(">I", raw)[0]
        for name, entry in self.entries.items():
            if entry == value:
                return name
        return value

    def validate(self, value):
        """
        Normalise to the entry name, which is what the rest of the device
        assumes an enumeration setting holds -- payload_size() looks the
        pixel format up by name, so an ordinal stored here surfaces as a
        KeyError raised from inside refresh_geometry(), nowhere near the
        write that caused it.

        decode() already resolves a known ordinal to its name, so an integer
        reaching here from a client write is by definition not one of the
        entries. get_camera_settings() may legitimately hand one back,
        though, so ordinals are translated rather than refused outright.
        """
        if not isinstance(value, str):
            for name, entry in self.entries.items():
                if entry == value:
                    return name
        elif value in self.entries:
            return value
        raise FeatureError("%s: %r is not one of %s"
                           % (self.name, value,
                              ", ".join("%s (%d)" % pair
                                        for pair in self.entries.items())))


@dataclass
class CommandFeature(Feature):
    """
    A GenICam Command writes its CommandValue to a register. The device
    observes the write, acts, then clears the register -- which is what
    makes the command re-executable.
    """
    command_value: int = 1

    size = 4
    settable = False

    def encode(self, value):
        return struct.pack(">I", int(value) & 0xFFFFFFFF)

    def decode(self, raw):
        return struct.unpack(">I", raw)[0]


@dataclass
class StringFeature(Feature):
    default: str = ""
    length: int = 32

    settable = True

    @property
    def size(self):
        return self.length

    def encode(self, value):
        raw = str(value).encode("utf-8")[:self.length - 1]
        return raw + b"\x00" * (self.length - len(raw))

    def decode(self, raw):
        return raw.split(b"\x00", 1)[0].decode("utf-8", "replace")


class FeatureSet(object):
    """
    Owns the feature list, the address allocation and the name lookup.
    """

    def __init__(self, arena_start=c.FEATURE_ARENA_START,
                 arena_end=c.FEATURE_ARENA_END):
        self.arena_start = arena_start
        self.arena_end = arena_end
        self.features = []
        self.by_name = {}
        self.by_address = {}
        self._next = arena_start
        self._unusual_categories = set()

    def _check_category(self, feature):
        """
        Warn once for a category the naming convention does not define.

        Not an error: a domain the convention does not cover is a legitimate
        thing to have a category for. But a misspelling produces a *new*
        category holding one feature, which looks like a stray node in a
        viewer's tree and nothing else, so it is worth a line naming the
        nearest real one.
        """
        category = feature.category
        if category in SFNC_CATEGORIES or category in self._unusual_categories:
            return
        self._unusual_categories.add(category)
        close = difflib.get_close_matches(category, SFNC_CATEGORIES, 1, 0.8)
        log.warning(
            "%r is not a standard feature category (first used by %r)%s",
            category, feature.name,
            "; did you mean %r?" % close[0] if close else "")

    def add(self, feature):
        if feature.name in self.by_name:
            raise FeatureError("duplicate feature name %r" % feature.name)

        self._check_category(feature)

        # Aravis injects its own nodes for every transport layer feature and
        # skips any name already in the document, so a collision here would
        # silently replace working plumbing with ours.
        if feature.name.startswith("Gev") or feature.name.startswith("ArvGev"):
            raise FeatureError(
                "%r collides with the transport layer nodes the client "
                "injects; camera features must not be Gev-prefixed"
                % feature.name)

        size = feature.size
        # Align each register to its own size so a 4 byte read of a float
        # cannot straddle two features.
        align = 8 if size == 8 else 4
        address = (self._next + align - 1) // align * align
        if address + size > self.arena_end:
            raise FeatureError("feature arena exhausted at %r" % feature.name)

        feature.address = address
        self._next = address + size

        self.features.append(feature)
        self.by_name[feature.name] = feature
        self.by_address[address] = feature
        return feature

    def extend(self, features):
        for feature in features:
            self.add(feature)

    def lookup_address(self, address):
        return self.by_address.get(address)

    def categories(self):
        seen = []
        for feature in self.features:
            if feature.category not in seen:
                seen.append(feature.category)
        return seen

    def defaults(self):
        out = {}
        for feature in self.features:
            if isinstance(feature, CommandFeature):
                continue
            out[feature.name] = feature.default
        return out
