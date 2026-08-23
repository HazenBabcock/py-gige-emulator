#
# Network interface introspection, stdlib only.
#
# A GigE Vision device has to publish its own IP, subnet and MAC in the
# bootstrap registers, because that is what the client connects to -- the
# UDP source address of the discovery reply is not used.
#

import fcntl
import hashlib
import os
import socket
import struct

SIOCGIFADDR = 0x8915
SIOCGIFNETMASK = 0x891B
SIOCGIFHWADDR = 0x8927


class InterfaceError(Exception):
    pass


def _ioctl_addr(sock, name, request):
    packed = struct.pack("256s", name.encode()[:15])
    try:
        result = fcntl.ioctl(sock.fileno(), request, packed)
    except OSError as e:
        raise InterfaceError("cannot query interface '%s': %s" % (name, e)) from e
    return socket.inet_ntoa(result[20:24])


def _ioctl_mac(sock, name):
    packed = struct.pack("256s", name.encode()[:15])
    try:
        result = fcntl.ioctl(sock.fileno(), SIOCGIFHWADDR, packed)
    except OSError as e:
        raise InterfaceError("cannot query interface '%s': %s" % (name, e)) from e
    return bytes(result[18:24])


def list_interfaces():
    """
    Interfaces that have an IPv4 address, as [(name, ip), ...].

    Used to make a bad --interface say what the choices are; the default of
    'eth0' is wrong on any machine using predictable interface names.
    """
    found = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for name in sorted(os.listdir("/sys/class/net")):
            try:
                found.append((name, _ioctl_addr(sock, name, SIOCGIFADDR)))
            except InterfaceError:
                continue
    except OSError:
        pass
    finally:
        sock.close()
    return found


def interface_info(name):
    """
    Returns (ip, netmask, mac_bytes) for a named interface.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ip = _ioctl_addr(sock, name, SIOCGIFADDR)
        netmask = _ioctl_addr(sock, name, SIOCGIFNETMASK)
        mac = _ioctl_mac(sock, name)
    except InterfaceError as e:
        available = list_interfaces()
        if available:
            raise InterfaceError(
                "%s\navailable interfaces: %s"
                % (e, ", ".join("%s (%s)" % pair for pair in available))) from e
        raise
    finally:
        sock.close()
    return ip, netmask, mac


def bind_to_device(sock, name):
    """
    Restrict a socket to one network interface. Returns True on success.

    This is the only way to honour "serve on eth0" for the control socket.
    Binding to the interface's *address* would not do it: a socket bound to a
    unicast address does not receive broadcast on Linux, and broadcast is how
    GigE Vision discovery arrives. Binding to the device keeps broadcast and
    still drops anything from another interface.

    Not fatal if it fails -- SO_BINDTODEVICE needed CAP_NET_RAW on older
    kernels and does not exist off Linux. The emulator then listens
    everywhere, which is what it did before, so a caller should warn rather
    than give up.
    """
    if not hasattr(socket, "SO_BINDTODEVICE"):
        return False
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                        name.encode() + b"\x00")
        return True
    except OSError:
        return False


def broadcast_address(ip, netmask):
    ip_int = struct.unpack("!I", socket.inet_aton(ip))[0]
    mask_int = struct.unpack("!I", socket.inet_aton(netmask))[0]
    return socket.inet_ntoa(struct.pack("!I", ip_int | (~mask_int & 0xFFFFFFFF)))


def format_mac(mac):
    return ":".join("%02x" % b for b in mac)


def parse_mac(text):
    """
    The inverse of format_mac. Accepts colons or dashes, since the IEEE
    registry and every vendor's documentation write the same six bytes both
    ways.
    """
    parts = text.replace("-", ":").split(":")
    if len(parts) != 6:
        raise ValueError("a MAC address is six octets, e.g. 00:30:53:12:34:56;"
                         " got %r" % text)
    try:
        octets = [int(part, 16) for part in parts]
    except ValueError:
        raise ValueError("%r is not hexadecimal" % text) from None
    if any(octet < 0 or octet > 0xFF for octet in octets):
        raise ValueError("%r has an octet outside 0x00..0xff" % text)
    return bytes(octets)


def mac_from_serial(oui, serial):
    """
    A stable MAC for a camera that has no MAC of its own -- a USB camera
    being served as a network device, say.

    The first three octets are the vendor OUI, which is what a client may
    insist on seeing. The last three are a digest of the camera's serial, so
    two cameras of the same make never collide and a given camera keeps the
    same address across restarts, which matters because some clients
    remember a device by it.

    Nothing is ever sent from this address; it is reported and nothing else.
    """
    parts = oui.replace("-", ":").split(":")
    if len(parts) != 3:
        raise ValueError("an OUI is three octets, e.g. 00:30:53; got %r" % oui)
    prefix = parse_mac(":".join(parts + ["00", "00", "00"]))[:3]
    return prefix + hashlib.sha1(serial.encode("utf-8")).digest()[:3]
