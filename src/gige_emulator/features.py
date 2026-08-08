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

import struct
from dataclasses import dataclass, field

from . import constants as c


class FeatureError(Exception):
    pass


@dataclass
class Feature:
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
        if isinstance(value, str) and value not in self.entries:
            raise FeatureError("%s: %r is not a valid entry" % (self.name, value))
        return value


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

    def add(self, feature):
        if feature.name in self.by_name:
            raise FeatureError("duplicate feature name %r" % feature.name)

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
