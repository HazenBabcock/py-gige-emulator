#!/usr/bin/env python3
#
# Walk a GenTL producer (.cti) directly through the C API and report what it
# sees: transport layer, interfaces, and the devices on each. With --stream it
# goes further and grabs frames.
#
# This sits below every vendor's viewer, so it separates "the producer never
# looked" from "the producer looked and rejected the device" from "the producer
# found it and the application filtered it" -- which is the whole question when
# a camera fails to enumerate.
#
#   python3 gentl_probe.py /path/to/producer.cti
#   python3 gentl_probe.py /path/to/producer.cti --stream 10
#
# Set LD_LIBRARY_PATH to the vendor's lib directory first, or the .cti will
# fail to load its own dependencies.
#
import argparse
import ctypes
import io
import sys
import xml.etree.ElementTree as ElementTree
import zipfile

# --- GenTL enums ---------------------------------------------------------

IFACE_INFO_ID, IFACE_INFO_DISPLAYNAME, IFACE_INFO_TLTYPE = 0, 1, 2

# INFO_DATATYPE. Note STRING is 1 -- 0 is UNKNOWN, and treating it as the
# string case silently turns every short name into an integer.
INFO_DATATYPE_STRING = 1
INFO_DATATYPE_INT16, INFO_DATATYPE_UINT16 = 3, 4
INFO_DATATYPE_INT32, INFO_DATATYPE_UINT32 = 5, 6
INFO_DATATYPE_INT64, INFO_DATATYPE_UINT64 = 7, 8
INFO_DATATYPE_BOOL8, INFO_DATATYPE_SIZET = 11, 12
INFO_DATATYPE_INTS = (INFO_DATATYPE_INT16, INFO_DATATYPE_UINT16,
                      INFO_DATATYPE_INT32, INFO_DATATYPE_UINT32,
                      INFO_DATATYPE_INT64, INFO_DATATYPE_UINT64,
                      INFO_DATATYPE_BOOL8, INFO_DATATYPE_SIZET)

DEV_INFO = {0: "id", 1: "vendor", 2: "model", 3: "tltype",
            4: "displayname", 5: "access", 6: "user name", 7: "serial",
            8: "version"}
ACCESS_STATUS = {0: "unknown", 1: "readwrite", 2: "readonly",
                 3: "nonaccessible", 4: "busy", 5: "open readwrite",
                 6: "open readonly"}

DEVICE_ACCESS_CONTROL = 3
DEVICE_ACCESS_EXCLUSIVE = 4

STREAM_INFO_PAYLOAD_SIZE = 7
STREAM_INFO_TLTYPE = 10
STREAM_INFO_BUF_ANNOUNCE_MIN = 12

BUFFER_INFO_SIZE_FILLED = 9
BUFFER_INFO_IS_INCOMPLETE = 7
BUFFER_INFO_WIDTH = 10
BUFFER_INFO_HEIGHT = 11
BUFFER_INFO_FRAMEID = 16

# EVENT_TYPE. 0 is EVENT_ERROR, so registering that instead gets a clear
# "Unsupported event type" rather than a silent lack of frames.
EVENT_NEW_BUFFER = 1
ACQ_START_FLAGS_DEFAULT = 0
ACQ_STOP_FLAGS_DEFAULT = 0
ACQ_QUEUE_ALL_TO_INPUT = 0
GENTL_INFINITE = 0xFFFFFFFFFFFFFFFF


class EventNewBufferData(ctypes.Structure):
    _fields_ = [("BufferHandle", ctypes.c_void_p),
                ("pUserPointer", ctypes.c_void_p)]


# Every entry point returns GC_ERROR (int32); handles are opaque pointers.
_SIGNATURES = {
    "GCInitLib": [],
    "GCCloseLib": [],
    "GCGetLastError": [ctypes.POINTER(ctypes.c_int32), ctypes.c_char_p,
                       ctypes.POINTER(ctypes.c_size_t)],
    "GCRegisterEvent": [ctypes.c_void_p, ctypes.c_int32,
                        ctypes.POINTER(ctypes.c_void_p)],
    "GCUnregisterEvent": [ctypes.c_void_p, ctypes.c_int32],
    "GCReadPort": [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_void_p,
                   ctypes.POINTER(ctypes.c_size_t)],
    "GCWritePort": [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_void_p,
                    ctypes.POINTER(ctypes.c_size_t)],
    "GCGetPortURL": [ctypes.c_void_p, ctypes.c_char_p,
                     ctypes.POINTER(ctypes.c_size_t)],
    "EventGetData": [ctypes.c_void_p, ctypes.c_void_p,
                     ctypes.POINTER(ctypes.c_size_t), ctypes.c_uint64],
    "TLOpen": [ctypes.POINTER(ctypes.c_void_p)],
    "TLClose": [ctypes.c_void_p],
    "TLUpdateInterfaceList": [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8),
                              ctypes.c_uint64],
    "TLGetNumInterfaces": [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)],
    "TLGetInterfaceID": [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_char_p,
                         ctypes.POINTER(ctypes.c_size_t)],
    "TLGetInterfaceInfo": [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32,
                           ctypes.POINTER(ctypes.c_int32), ctypes.c_char_p,
                           ctypes.POINTER(ctypes.c_size_t)],
    "TLOpenInterface": [ctypes.c_void_p, ctypes.c_char_p,
                        ctypes.POINTER(ctypes.c_void_p)],
    "IFClose": [ctypes.c_void_p],
    "IFUpdateDeviceList": [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8),
                           ctypes.c_uint64],
    "IFGetNumDevices": [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)],
    "IFGetDeviceID": [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_char_p,
                      ctypes.POINTER(ctypes.c_size_t)],
    "IFGetDeviceInfo": [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32,
                        ctypes.POINTER(ctypes.c_int32), ctypes.c_char_p,
                        ctypes.POINTER(ctypes.c_size_t)],
    "IFOpenDevice": [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32,
                     ctypes.POINTER(ctypes.c_void_p)],
    "DevClose": [ctypes.c_void_p],
    "DevGetPort": [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)],
    "DevGetNumDataStreams": [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)],
    "DevGetDataStreamID": [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_char_p,
                           ctypes.POINTER(ctypes.c_size_t)],
    "DevOpenDataStream": [ctypes.c_void_p, ctypes.c_char_p,
                          ctypes.POINTER(ctypes.c_void_p)],
    "DSClose": [ctypes.c_void_p],
    "DSGetInfo": [ctypes.c_void_p, ctypes.c_int32,
                  ctypes.POINTER(ctypes.c_int32), ctypes.c_void_p,
                  ctypes.POINTER(ctypes.c_size_t)],
    "DSAllocAndAnnounceBuffer": [ctypes.c_void_p, ctypes.c_size_t,
                                 ctypes.c_void_p,
                                 ctypes.POINTER(ctypes.c_void_p)],
    "DSQueueBuffer": [ctypes.c_void_p, ctypes.c_void_p],
    "DSRevokeBuffer": [ctypes.c_void_p, ctypes.c_void_p,
                       ctypes.POINTER(ctypes.c_void_p),
                       ctypes.POINTER(ctypes.c_void_p)],
    "DSStartAcquisition": [ctypes.c_void_p, ctypes.c_int32, ctypes.c_uint64],
    "DSStopAcquisition": [ctypes.c_void_p, ctypes.c_int32],
    "DSFlushQueue": [ctypes.c_void_p, ctypes.c_int32],
    "DSGetBufferInfo": [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int32,
                        ctypes.POINTER(ctypes.c_int32), ctypes.c_void_p,
                        ctypes.POINTER(ctypes.c_size_t)],
}


class GenTLError(RuntimeError):
    pass


class GenTL(object):

    def __init__(self, path):
        self.lib = ctypes.CDLL(path)
        for name, argtypes in _SIGNATURES.items():
            fn = getattr(self.lib, name, None)
            if fn is None:          # optional in older producers
                continue
            fn.restype = ctypes.c_int32
            fn.argtypes = argtypes

    # --- error handling --------------------------------------------------

    def last_error(self):
        code = ctypes.c_int32(0)
        size = ctypes.c_size_t(1024)
        buf = ctypes.create_string_buffer(1024)
        self.lib.GCGetLastError(ctypes.byref(code), buf, ctypes.byref(size))
        return buf.value.decode("utf-8", "replace")

    def check(self, rc, what):
        if rc != 0:
            raise GenTLError("%s failed: rc=%d %s"
                             % (what, rc, self.last_error()))

    # --- typed getters ---------------------------------------------------

    def string_of(self, fn, *args):
        size = ctypes.c_size_t(1024)
        buf = ctypes.create_string_buffer(1024)
        if fn(*(args + (buf, ctypes.byref(size)))) != 0:
            return None
        return buf.value.decode("utf-8", "replace")

    def info_of(self, fn, *args):
        """
        The *Info entry points share a shape: (..., cmd, &type, buf, &size).

        Dispatch on the returned INFO_DATATYPE and nothing else. Guessing from
        the length instead decodes any 8 byte string as an integer, which is
        exactly what a model name of "PyNoise\\0" is.
        """
        size = ctypes.c_size_t(1024)
        buf = ctypes.create_string_buffer(1024)
        itype = ctypes.c_int32(0)
        if fn(*(args + (ctypes.byref(itype), buf, ctypes.byref(size)))) != 0:
            return None
        if itype.value == INFO_DATATYPE_STRING:
            return buf.value.decode("utf-8", "replace")
        if itype.value in INFO_DATATYPE_INTS:
            signed = itype.value in (INFO_DATATYPE_INT16, INFO_DATATYPE_INT32,
                                     INFO_DATATYPE_INT64)
            return int.from_bytes(buf.raw[:size.value], sys.byteorder,
                                  signed=signed)
        return buf.raw[:size.value]


# --- GenICam XML ---------------------------------------------------------

def fetch_xml(g, port):
    """
    Read the device's GenICam XML through the port. The URL looks like
    'Local:name.xml;<hex address>;<hex length>' -- the same string the
    bootstrap registers carry, so this works for any GigE Vision device.
    """
    url = g.string_of(g.lib.GCGetPortURL, port)
    if url is None:
        raise GenTLError("GCGetPortURL failed: %s" % g.last_error())
    url = url.split("\x00", 1)[0]
    if url.lower().startswith("file:"):
        raise GenTLError("XML is not an in-device blob: %r" % url)
    # Some producers append a query -- ImpactAcquire hands back
    # '...;10000;19e5?SchemaVersion=0.0.0' -- so strip it before parsing the
    # length, or the int() blows up on the suffix.
    path, address, length = url.split("?", 1)[0].rsplit(";", 2)
    address, length = int(address, 16), int(length, 16)

    out = bytearray()
    while len(out) < length:
        want = min(512, length - len(out))
        size = ctypes.c_size_t(want)
        buf = ctypes.create_string_buffer(want)
        g.check(g.lib.GCReadPort(port, address + len(out), buf,
                                 ctypes.byref(size)), "GCReadPort")
        if size.value == 0:
            raise GenTLError("GCReadPort returned nothing at 0x%x"
                             % (address + len(out)))
        out += buf.raw[:size.value]

    blob = bytes(out[:length])
    # Most real cameras ship the XML zipped, since the client reads it 512
    # bytes at a time. The filename in the URL is what says so.
    if path.lower().endswith(".zip"):
        with zipfile.ZipFile(io.BytesIO(blob)) as archive:
            blob = archive.read(archive.namelist()[0])
    return url, blob


def _namespace(root):
    return root.tag[:root.tag.index("}") + 1] if "}" in root.tag else ""


def _find_register(root, tag, feature_name):
    """
    Resolve a named feature to the register behind it. A feature carries a
    <pValue> naming an <IntReg>, and that register holds <Address>, <Length>
    and <Endianess>. Standard GenICam, so this is not specific to this
    emulator.
    """
    target = None
    for element in root.iter():
        if element.get("Name") != feature_name:
            continue
        p = element.find(tag + "pValue")
        if p is not None:
            target = p.text.strip()
            break
    if target is None:
        return None

    for element in root.iter(tag + "IntReg"):
        if element.get("Name") != target:
            continue
        address = element.find(tag + "Address")
        if address is None:
            return None
        length = element.find(tag + "Length")
        endian = element.find(tag + "Endianess")
        return {"address": int(address.text.strip(), 0),
                "length": int(length.text.strip(), 0) if length is not None
                else 4,
                "big": (endian is None
                        or endian.text.strip().lower() == "bigendian")}
    return None


def find_command_register(xml, command_name):
    """Returns (register, value_to_write) for a <Command>, or None."""
    root = ElementTree.fromstring(xml)
    tag = _namespace(root)
    reg = _find_register(root, tag, command_name)
    if reg is None:
        return None
    value = 1
    for element in root.iter(tag + "Command"):
        if element.get("Name") == command_name:
            v = element.find(tag + "CommandValue")
            if v is not None:
                value = int(v.text.strip(), 0)
            break
    return reg, value


def write_register(g, port, reg, value):
    """
    Write an integer to a device register, honouring its declared endianness.

    GCWritePort passes the bytes through untouched, so handing it a native
    ctypes integer writes little endian on x86 -- a 1 arrives at a big endian
    GigE Vision device as 0x01000000, and the command silently never fires.
    """
    raw = int(value).to_bytes(reg["length"], "big" if reg["big"] else "little")
    size = ctypes.c_size_t(len(raw))
    buf = ctypes.create_string_buffer(raw, len(raw))
    return g.lib.GCWritePort(port, reg["address"], buf, ctypes.byref(size))


def read_int_feature(g, port, xml, feature_name):
    """
    Read an integer feature straight off the device port.

    Needed because a producer is allowed to report no payload size of its
    own -- GenTL says the consumer then takes it from the device's own
    PayloadSize feature, which is what this is for.
    """
    root = ElementTree.fromstring(xml)
    reg = _find_register(root, _namespace(root), feature_name)
    if reg is None:
        return None
    size = ctypes.c_size_t(reg["length"])
    buf = ctypes.create_string_buffer(reg["length"])
    if g.lib.GCReadPort(port, reg["address"], buf, ctypes.byref(size)) != 0:
        return None
    return int.from_bytes(buf.raw[:size.value],
                          "big" if reg["big"] else "little")


# --- streaming -----------------------------------------------------------

def stream(g, device, frames, timeout_ms):
    port = ctypes.c_void_p()
    g.check(g.lib.DevGetPort(device, ctypes.byref(port)), "DevGetPort")

    url, xml = fetch_xml(g, port)
    print("      XML: %s (%d bytes)" % (url, len(xml)))

    start = find_command_register(xml, "AcquisitionStart")
    stop = find_command_register(xml, "AcquisitionStop")
    if start is None:
        raise GenTLError("no AcquisitionStart command in the device XML")
    print("      AcquisitionStart -> write 0x%x = %d"
          % (start[0]["address"], start[1]))

    n = ctypes.c_uint32(0)
    g.check(g.lib.DevGetNumDataStreams(device, ctypes.byref(n)),
            "DevGetNumDataStreams")
    if n.value == 0:
        raise GenTLError("device reports no data streams")
    stream_id = g.string_of(g.lib.DevGetDataStreamID, device, 0)

    handle = ctypes.c_void_p()
    g.check(g.lib.DevOpenDataStream(device, stream_id.encode(),
                                    ctypes.byref(handle)),
            "DevOpenDataStream")
    print("      data stream: %s" % stream_id)

    buffers = []
    event = ctypes.c_void_p()
    try:
        payload = g.info_of(g.lib.DSGetInfo, handle, STREAM_INFO_PAYLOAD_SIZE)
        least = g.info_of(g.lib.DSGetInfo, handle,
                          STREAM_INFO_BUF_ANNOUNCE_MIN)
        if not isinstance(payload, int) or payload <= 0:
            # DEFINES_PAYLOADSIZE false: the producer does not size buffers
            # itself and expects us to take it from the device.
            payload = read_int_feature(g, port, xml, "PayloadSize")
            print("      producer gave no payload size; PayloadSize feature "
                  "says %r" % (payload,))
        if not isinstance(payload, int) or payload <= 0:
            raise GenTLError("no usable payload size (%r)" % (payload,))
        count = max(least if isinstance(least, int) else 1, 4)
        print("      payload %d bytes, announcing %d buffers"
              % (payload, count))

        for _ in range(count):
            b = ctypes.c_void_p()
            g.check(g.lib.DSAllocAndAnnounceBuffer(handle, payload, None,
                                                   ctypes.byref(b)),
                    "DSAllocAndAnnounceBuffer")
            buffers.append(b)
            g.check(g.lib.DSQueueBuffer(handle, b), "DSQueueBuffer")

        g.check(g.lib.GCRegisterEvent(handle, EVENT_NEW_BUFFER,
                                      ctypes.byref(event)), "GCRegisterEvent")
        g.check(g.lib.DSStartAcquisition(handle, ACQ_START_FLAGS_DEFAULT,
                                         GENTL_INFINITE),
                "DSStartAcquisition")

        g.check(write_register(g, port, start[0], start[1]),
                "GCWritePort(AcquisitionStart)")

        good = incomplete = 0
        for i in range(frames):
            data = EventNewBufferData()
            size = ctypes.c_size_t(ctypes.sizeof(data))
            rc = g.lib.EventGetData(event, ctypes.byref(data),
                                    ctypes.byref(size), timeout_ms)
            if rc != 0:
                print("      frame %d: EventGetData rc=%d %s"
                      % (i, rc, g.last_error()))
                break
            b = ctypes.c_void_p(data.BufferHandle)
            filled = g.info_of(g.lib.DSGetBufferInfo, handle, b,
                               BUFFER_INFO_SIZE_FILLED)
            bad = g.info_of(g.lib.DSGetBufferInfo, handle, b,
                            BUFFER_INFO_IS_INCOMPLETE)
            width = g.info_of(g.lib.DSGetBufferInfo, handle, b,
                              BUFFER_INFO_WIDTH)
            height = g.info_of(g.lib.DSGetBufferInfo, handle, b,
                               BUFFER_INFO_HEIGHT)
            frame_id = g.info_of(g.lib.DSGetBufferInfo, handle, b,
                                 BUFFER_INFO_FRAMEID)
            if bad:
                incomplete += 1
            else:
                good += 1
            print("      frame %-3d id=%-6s %sx%s  %s bytes%s"
                  % (i, frame_id, width, height, filled,
                     "  INCOMPLETE" if bad else ""))
            g.lib.DSQueueBuffer(handle, b)

        if stop is not None:
            write_register(g, port, stop[0], stop[1])
        g.lib.DSStopAcquisition(handle, ACQ_STOP_FLAGS_DEFAULT)
        print("      complete %d, incomplete %d" % (good, incomplete))
        return good, incomplete
    finally:
        if event:
            g.lib.GCUnregisterEvent(handle, EVENT_NEW_BUFFER)
        g.lib.DSFlushQueue(handle, ACQ_QUEUE_ALL_TO_INPUT)
        for b in buffers:
            p1, p2 = ctypes.c_void_p(), ctypes.c_void_p()
            g.lib.DSRevokeBuffer(handle, b, ctypes.byref(p1), ctypes.byref(p2))
        g.lib.DSClose(handle)


# --- main ----------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Probe a GenTL producer and optionally grab frames.")
    parser.add_argument("cti", help="path to the .cti producer")
    parser.add_argument("--discovery-timeout", type=int, default=2000,
                        metavar="MS", help="per list update, default 2000")
    parser.add_argument("--stream", type=int, default=0, metavar="N",
                        help="open the first device found and grab N frames")
    parser.add_argument("--frame-timeout", type=int, default=5000,
                        metavar="MS", help="per frame, default 5000")
    parser.add_argument("--exclusive", action="store_true",
                        help="open the device exclusively rather than for "
                             "control, which refuses to share it")
    args = parser.parse_args()

    g = GenTL(args.cti)
    print("producer: %s" % args.cti)
    g.check(g.lib.GCInitLib(), "GCInitLib")

    tl = ctypes.c_void_p()
    g.check(g.lib.TLOpen(ctypes.byref(tl)), "TLOpen")

    changed = ctypes.c_uint8(0)
    g.check(g.lib.TLUpdateInterfaceList(tl, ctypes.byref(changed),
                                        args.discovery_timeout),
            "TLUpdateInterfaceList")
    n = ctypes.c_uint32(0)
    g.check(g.lib.TLGetNumInterfaces(tl, ctypes.byref(n)),
            "TLGetNumInterfaces")
    print("interfaces: %d" % n.value)

    total = 0
    streamed = False
    for i in range(n.value):
        iface_id = g.string_of(g.lib.TLGetInterfaceID, tl, i)
        key = iface_id.encode()
        tltype = g.info_of(g.lib.TLGetInterfaceInfo, tl, key,
                           IFACE_INFO_TLTYPE)
        display = g.info_of(g.lib.TLGetInterfaceInfo, tl, key,
                            IFACE_INFO_DISPLAYNAME)
        print("\n  [%d] %s  (type=%s, %s)" % (i, iface_id, tltype, display))

        handle = ctypes.c_void_p()
        if g.lib.TLOpenInterface(tl, key, ctypes.byref(handle)) != 0:
            print("      TLOpenInterface failed: %s" % g.last_error())
            continue
        try:
            if g.lib.IFUpdateDeviceList(handle, ctypes.byref(changed),
                                        args.discovery_timeout) != 0:
                print("      IFUpdateDeviceList failed: %s" % g.last_error())
                continue
            dn = ctypes.c_uint32(0)
            g.lib.IFGetNumDevices(handle, ctypes.byref(dn))
            print("      devices: %d" % dn.value)
            total += dn.value

            for d in range(dn.value):
                dev_id = g.string_of(g.lib.IFGetDeviceID, handle, d)
                print("      - %s" % dev_id)
                for cmd in sorted(DEV_INFO):
                    if cmd == 0:
                        continue
                    v = g.info_of(g.lib.IFGetDeviceInfo, handle,
                                  dev_id.encode(), cmd)
                    if v is None:
                        continue
                    if DEV_INFO[cmd] == "access":
                        v = "%s (%s)" % (v, ACCESS_STATUS.get(v, "?"))
                    print("          %-12s %s" % (DEV_INFO[cmd], v))

                if args.stream and not streamed:
                    streamed = True
                    flags = (DEVICE_ACCESS_EXCLUSIVE if args.exclusive
                             else DEVICE_ACCESS_CONTROL)
                    device = ctypes.c_void_p()
                    try:
                        g.check(g.lib.IFOpenDevice(handle, dev_id.encode(),
                                                   flags,
                                                   ctypes.byref(device)),
                                "IFOpenDevice")
                        try:
                            stream(g, device, args.stream, args.frame_timeout)
                        finally:
                            g.lib.DevClose(device)
                    except GenTLError as e:
                        print("      streaming failed: %s" % e)
        finally:
            g.lib.IFClose(handle)

    print("\ntotal devices: %d" % total)
    g.lib.TLClose(tl)
    g.lib.GCCloseLib()


if __name__ == "__main__":
    main()
