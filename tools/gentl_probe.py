#!/usr/bin/env python3
#
# Walk a GenTL producer (.cti) directly through the C API and report what it
# sees: transport layer, interfaces, and the devices on each.
#
# This sits below every vendor's viewer, so it separates "the producer never
# looked" from "the producer looked and rejected the device" -- which is the
# whole question when a camera fails to enumerate.
#
#   python3 gentl_probe.py /path/to/producer.cti [discovery_timeout_ms]
#
import ctypes
import sys

# GenTL info commands. Interface and device share the numbering below only by
# coincidence; they are separate enums in the spec.
IFACE_INFO_ID, IFACE_INFO_DISPLAYNAME, IFACE_INFO_TLTYPE = 0, 1, 2
DEV_INFO = {0: "id", 1: "vendor", 2: "model", 3: "tltype",
            4: "displayname", 5: "access", 6: "user name", 7: "serial",
            8: "version"}
ACCESS_STATUS = {0: "unknown", 1: "readwrite", 2: "readonly",
                 3: "nonaccessible", 4: "busy", 5: "open readwrite",
                 6: "open readonly"}


class GenTL(object):

    def __init__(self, path):
        self.lib = ctypes.CDLL(path)
        for name, argtypes in (
                ("GCInitLib", []),
                ("GCCloseLib", []),
                ("GCGetLastError", [ctypes.POINTER(ctypes.c_int32),
                                    ctypes.c_char_p,
                                    ctypes.POINTER(ctypes.c_size_t)]),
                ("TLOpen", [ctypes.POINTER(ctypes.c_void_p)]),
                ("TLClose", [ctypes.c_void_p]),
                ("TLUpdateInterfaceList", [ctypes.c_void_p,
                                           ctypes.POINTER(ctypes.c_uint8),
                                           ctypes.c_uint64]),
                ("TLGetNumInterfaces", [ctypes.c_void_p,
                                        ctypes.POINTER(ctypes.c_uint32)]),
                ("TLGetInterfaceID", [ctypes.c_void_p, ctypes.c_uint32,
                                      ctypes.c_char_p,
                                      ctypes.POINTER(ctypes.c_size_t)]),
                ("TLGetInterfaceInfo", [ctypes.c_void_p, ctypes.c_char_p,
                                        ctypes.c_int32,
                                        ctypes.POINTER(ctypes.c_int32),
                                        ctypes.c_char_p,
                                        ctypes.POINTER(ctypes.c_size_t)]),
                ("TLOpenInterface", [ctypes.c_void_p, ctypes.c_char_p,
                                     ctypes.POINTER(ctypes.c_void_p)]),
                ("IFClose", [ctypes.c_void_p]),
                ("IFUpdateDeviceList", [ctypes.c_void_p,
                                        ctypes.POINTER(ctypes.c_uint8),
                                        ctypes.c_uint64]),
                ("IFGetNumDevices", [ctypes.c_void_p,
                                     ctypes.POINTER(ctypes.c_uint32)]),
                ("IFGetDeviceID", [ctypes.c_void_p, ctypes.c_uint32,
                                   ctypes.c_char_p,
                                   ctypes.POINTER(ctypes.c_size_t)]),
                ("IFGetDeviceInfo", [ctypes.c_void_p, ctypes.c_char_p,
                                     ctypes.c_int32,
                                     ctypes.POINTER(ctypes.c_int32),
                                     ctypes.c_char_p,
                                     ctypes.POINTER(ctypes.c_size_t)])):
            fn = getattr(self.lib, name)
            fn.restype = ctypes.c_int32
            fn.argtypes = argtypes

    def check(self, rc, what):
        if rc == 0:
            return
        code = ctypes.c_int32(0)
        size = ctypes.c_size_t(1024)
        buf = ctypes.create_string_buffer(1024)
        self.lib.GCGetLastError(ctypes.byref(code), buf, ctypes.byref(size))
        raise RuntimeError("%s failed: rc=%d %s"
                           % (what, rc, buf.value.decode("utf-8", "replace")))

    def _string(self, fn, *args):
        size = ctypes.c_size_t(1024)
        buf = ctypes.create_string_buffer(1024)
        rc = fn(*(args + (buf, ctypes.byref(size))))
        if rc != 0:
            return None
        return buf.value.decode("utf-8", "replace")

    def _info(self, fn, handle, key, cmd):
        size = ctypes.c_size_t(1024)
        buf = ctypes.create_string_buffer(1024)
        itype = ctypes.c_int32(0)
        rc = fn(handle, key, cmd, ctypes.byref(itype), buf,
                ctypes.byref(size))
        if rc != 0:
            return None
        # Type 0 is a NUL terminated string; the small integer types are what
        # ACCESS_STATUS and TLTYPE come back as.
        if itype.value == 0:
            return buf.value.decode("utf-8", "replace")
        if size.value == 4:
            return int.from_bytes(buf.raw[:4], sys.byteorder)
        return buf.raw[:size.value]


def main():
    path = sys.argv[1]
    timeout = int(sys.argv[2]) if len(sys.argv) > 2 else 1000

    g = GenTL(path)
    print("producer: %s" % path)
    g.check(g.lib.GCInitLib(), "GCInitLib")

    tl = ctypes.c_void_p()
    g.check(g.lib.TLOpen(ctypes.byref(tl)), "TLOpen")

    changed = ctypes.c_uint8(0)
    g.check(g.lib.TLUpdateInterfaceList(tl, ctypes.byref(changed), timeout),
            "TLUpdateInterfaceList")
    n = ctypes.c_uint32(0)
    g.check(g.lib.TLGetNumInterfaces(tl, ctypes.byref(n)), "TLGetNumInterfaces")
    print("interfaces: %d" % n.value)

    total_devices = 0
    for i in range(n.value):
        iface_id = g._string(g.lib.TLGetInterfaceID, tl, i)
        key = iface_id.encode()
        tltype = g._info(g.lib.TLGetInterfaceInfo, tl, key, IFACE_INFO_TLTYPE)
        display = g._info(g.lib.TLGetInterfaceInfo, tl, key,
                          IFACE_INFO_DISPLAYNAME)
        print("\n  [%d] %s  (type=%s, %s)" % (i, iface_id, tltype, display))

        h = ctypes.c_void_p()
        rc = g.lib.TLOpenInterface(tl, key, ctypes.byref(h))
        if rc != 0:
            print("      TLOpenInterface failed rc=%d" % rc)
            continue
        try:
            rc = g.lib.IFUpdateDeviceList(h, ctypes.byref(changed), timeout)
            if rc != 0:
                print("      IFUpdateDeviceList failed rc=%d" % rc)
                continue
            dn = ctypes.c_uint32(0)
            g.lib.IFGetNumDevices(h, ctypes.byref(dn))
            print("      devices: %d" % dn.value)
            total_devices += dn.value
            for d in range(dn.value):
                dev_id = g._string(g.lib.IFGetDeviceID, h, d)
                print("      - %s" % dev_id)
                for cmd, label in sorted(DEV_INFO.items()):
                    if cmd == 0:
                        continue
                    v = g._info(g.lib.IFGetDeviceInfo, h, dev_id.encode(),
                                cmd)
                    if v is None:
                        continue
                    if label == "access":
                        v = "%s (%s)" % (v, ACCESS_STATUS.get(v, "?"))
                    print("          %-12s %s" % (label, v))
        finally:
            g.lib.IFClose(h)

    print("\ntotal devices: %d" % total_devices)
    g.lib.TLClose(tl)
    g.lib.GCCloseLib()


if __name__ == "__main__":
    main()
