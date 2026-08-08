#
# Network interface introspection, stdlib only.
#
# A GigE Vision device has to publish its own IP, subnet and MAC in the
# bootstrap registers, because that is what the client connects to -- the
# UDP source address of the discovery reply is not used.
#

import fcntl
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


def broadcast_address(ip, netmask):
    ip_int = struct.unpack("!I", socket.inet_aton(ip))[0]
    mask_int = struct.unpack("!I", socket.inet_aton(netmask))[0]
    return socket.inet_ntoa(struct.pack("!I", ip_int | (~mask_int & 0xFFFFFFFF)))


def format_mac(mac):
    return ":".join("%02x" % b for b in mac)
