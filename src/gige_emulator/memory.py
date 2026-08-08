#
# The device's register memory.
#
# A GigE Vision device is, from the client's point of view, a flat address
# space. Bootstrap fields live at fixed addresses near zero, the GenICam XML
# is mapped read only just above the register space, and camera features sit
# wherever the feature allocator puts them.
#
# This module is also where the user's settings hooks are dispatched from,
# and that placement is load bearing. It is tempting to fire the hooks from
# the GVCP command handler instead, on WRITE_REGISTER and READ_REGISTER --
# but the client only uses those commands for 4 byte accesses on a schema
# older than 1.1.0 (arvgcport.c, _use_legacy_endianness_mechanism). An 8 byte
# float or a string register always arrives as READ_MEMORY. Dispatching on
# (address, length) here catches every path.
#

import struct

from . import constants as c


class MemoryError_(Exception):
    """Raised for an access the device should refuse."""

    def __init__(self, message, gvcp_error):
        super().__init__(message)
        self.gvcp_error = gvcp_error


class DeviceMemory(object):

    def __init__(self, size=c.MEMORY_SIZE):
        self._mem = bytearray(size)
        self._size = size
        self._xml = b""

        # Installed by the server once features exist. Both are optional so
        # that the memory layer stays testable on its own.
        #
        # on_read(address, length) is called before a read is served, and may
        # write into memory to refresh what is about to be returned.
        #
        # on_write(address, data) is called after a write lands. Raising
        # MemoryError_ from it rolls the write back.
        self.on_read = None
        self.on_write = None

    # --- the XML blob ----------------------------------------------------

    def set_genicam_xml(self, xml_bytes):
        """
        Map the GenICam XML read only at MEMORY_SIZE.

        Padded to a 512 byte boundary because the client rounds a READMEM
        count up to a multiple of 4, so the final chunk of an odd sized blob
        reads off the end. Padding makes that a no-op rather than a short
        read that fails the client's length check.
        """
        pad = (-len(xml_bytes)) % 512
        self._xml = bytes(xml_bytes) + b"\x00" * pad
        return len(xml_bytes)

    @property
    def xml_base(self):
        return self._size

    # --- raw access ------------------------------------------------------

    def _read_raw(self, address, length):
        out = bytearray()
        if address < self._size:
            end = min(address + length, self._size)
            out += self._mem[address:end]
            if len(out) == length:
                return bytes(out)
            address = self._size
            length -= len(out)

        offset = address - self._size
        if offset < len(self._xml):
            end = min(offset + length, len(self._xml))
            out += self._xml[offset:end]

        # Anything past the end of the XML reads as zero rather than short.
        if len(out) < length:
            out += b"\x00" * (length - len(out))
        return bytes(out)

    def read(self, address, length):
        if address < 0 or length < 0:
            raise MemoryError_("negative address or length", c.ERROR_INVALID_PARAMETER)
        if self.on_read is not None:
            self.on_read(address, length)
        return self._read_raw(address, length)

    def write(self, address, data):
        if address < 0:
            raise MemoryError_("negative address", c.ERROR_INVALID_PARAMETER)
        end = address + len(data)
        if address >= self._size:
            raise MemoryError_("GenICam XML is read only", c.ERROR_WRITE_PROTECT)
        if end > self._size:
            raise MemoryError_("write crosses into read only memory",
                               c.ERROR_WRITE_PROTECT)

        previous = bytes(self._mem[address:end])
        self._mem[address:end] = data
        if self.on_write is not None:
            try:
                self.on_write(address, bytes(data))
            except Exception:
                self._mem[address:end] = previous
                raise

    # --- convenience -----------------------------------------------------

    def read_register(self, address):
        return struct.unpack(">I", self.read(address, 4))[0]

    def write_register(self, address, value):
        self.write(address, struct.pack(">I", value & 0xFFFFFFFF))

    def peek_register(self, address):
        """Read without firing the read hook. For the device's own use."""
        return struct.unpack(">I", self._read_raw(address, 4))[0]

    def poke_register(self, address, value):
        """Write without firing the write hook. For the device's own use."""
        self._mem[address:address + 4] = struct.pack(">I", value & 0xFFFFFFFF)

    def poke_bytes(self, address, data):
        self._mem[address:address + len(data)] = data

    def poke_string(self, address, text, size):
        raw = text.encode("utf-8")[:size - 1]
        self._mem[address:address + size] = raw + b"\x00" * (size - len(raw))

    def peek_string(self, address, size):
        raw = bytes(self._mem[address:address + size])
        return raw.split(b"\x00", 1)[0].decode("utf-8", "replace")
