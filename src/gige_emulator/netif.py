#
# Network interface introspection, stdlib only.
#
# A GigE Vision device has to publish its own IP, subnet and MAC in the
# bootstrap registers, because that is what the client connects to -- the
# UDP source address of the discovery reply is not used.
#

import fcntl
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


def interface_info(name):
    """
    Returns (ip, netmask, mac_bytes) for a named interface.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ip = _ioctl_addr(sock, name, SIOCGIFADDR)
        netmask = _ioctl_addr(sock, name, SIOCGIFNETMASK)
        mac = _ioctl_mac(sock, name)
    finally:
        sock.close()
    return ip, netmask, mac


def broadcast_address(ip, netmask):
    ip_int = struct.unpack("!I", socket.inet_aton(ip))[0]
    mask_int = struct.unpack("!I", socket.inet_aton(netmask))[0]
    return socket.inet_ntoa(struct.pack("!I", ip_int | (~mask_int & 0xFFFFFFFF)))


def format_mac(mac):
    return ":".join("%02x" % b for b in mac)
