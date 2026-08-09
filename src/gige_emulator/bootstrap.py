#
# Populates the GigE Vision bootstrap register page.
#
# Kept separate from the server so it can be checked without opening a
# socket -- the discovery reply is a verbatim copy of the first 248 bytes of
# this page, so getting a field offset wrong here is invisible until a client
# fails to find the camera.
#

import socket
import struct
from dataclasses import dataclass, field

from . import constants as c


@dataclass
class DeviceInfo:
    ip: str
    netmask: str = "255.255.255.0"
    mac: bytes = b"\x00\x00\x00\x00\x00\x00"
    gateway: str = "0.0.0.0"
    manufacturer_name: str = "py-gige-emulator"
    model_name: str = "PyGigE"
    device_version: str = "0.1.0"
    manufacturer_info: str = "pure python GigE Vision emulator"
    serial_number: str = "PY-0001"
    user_defined_name: str = ""
    xml_filename: str = "camera.xml"
    heartbeat_timeout_ms: int = 3000
    packet_size: int = c.DEFAULT_PACKET_SIZE
    #: Advertise packet resend. Off makes the device behave as it did before
    #: resend existed, which is the comparison to make when a client
    #: misbehaves rather than merely loses packets.
    packet_resend: bool = True

    def __post_init__(self):
        if self.manufacturer_name in c.RESERVED_VENDOR_NAMES:
            raise ValueError(
                "vendor name %r triggers client side per-vendor workarounds; "
                "pick another" % self.manufacturer_name)


def _ip_to_u32(text):
    return struct.unpack("!I", socket.inet_aton(text))[0]


def init_bootstrap(memory, info, xml_size):
    """
    Write the bootstrap page. `xml_size` is the unpadded length of the
    GenICam XML, which goes into the URL string the client parses.
    """
    memory.poke_register(c.BS_VERSION, 0x00010002)
    memory.poke_register(c.BS_DEVICE_MODE,
                         c.DEVICE_MODE_BIG_ENDIAN | c.DEVICE_MODE_CHARSET_UTF8)

    # The client reads the 6 MAC bytes from offsets 0x0a..0x0f, i.e. the low
    # half of the "high" word and all of the "low" word. Writing them at
    # 0x08 instead gives every emulated camera a MAC of 00:00:xx:xx:xx:xx.
    mac = info.mac.ljust(6, b"\x00")[:6]
    memory.poke_register(c.BS_MAC_HIGH, struct.unpack(">H", mac[0:2])[0])
    memory.poke_register(c.BS_MAC_LOW, struct.unpack(">I", mac[2:6])[0])

    memory.poke_register(c.BS_SUPPORTED_IP_CONFIG, 0x80000007)
    memory.poke_register(c.BS_CURRENT_IP_CONFIG, 0x80000005)

    # This, not the UDP source address, is where the client will connect.
    memory.poke_register(c.BS_CURRENT_IP_ADDRESS, _ip_to_u32(info.ip))
    memory.poke_register(c.BS_CURRENT_SUBNET_MASK, _ip_to_u32(info.netmask))
    memory.poke_register(c.BS_CURRENT_GATEWAY, _ip_to_u32(info.gateway))

    memory.poke_string(c.BS_MANUFACTURER_NAME, info.manufacturer_name,
                       c.BS_MANUFACTURER_NAME_SIZE)
    memory.poke_string(c.BS_MODEL_NAME, info.model_name, c.BS_MODEL_NAME_SIZE)
    memory.poke_string(c.BS_DEVICE_VERSION, info.device_version,
                       c.BS_DEVICE_VERSION_SIZE)
    memory.poke_string(c.BS_MANUFACTURER_INFO, info.manufacturer_info,
                       c.BS_MANUFACTURER_INFO_SIZE)
    memory.poke_string(c.BS_SERIAL_NUMBER, info.serial_number,
                       c.BS_SERIAL_NUMBER_SIZE)
    memory.poke_string(c.BS_USER_DEFINED_NAME, info.user_defined_name,
                       c.BS_USER_DEFINED_NAME_SIZE)

    # "Local:///name.xml;<hex address>;<hex size>" -- lower case hex, no 0x.
    url = "Local:///%s;%x;%x" % (info.xml_filename, memory.xml_base, xml_size)
    memory.poke_string(c.BS_XML_URL_0, url, c.BS_XML_URL_SIZE)

    memory.poke_register(c.BS_N_NETWORK_INTERFACES, 1)
    memory.poke_register(c.BS_N_MESSAGE_CHANNELS, 0)
    memory.poke_register(c.BS_N_STREAM_CHANNELS, 1)

    # Advertising packet resend is what makes a client ask for one. Aravis
    # reads this register once at open and, finding the bit clear, sets the
    # stream to PACKET_RESEND_NEVER for the whole session (arvgvdevice.c) --
    # so with it off no amount of device-side support is ever exercised.
    #
    # It matters most exactly where it is least optional. A full resolution
    # frame is 16,984 packets; at a measured 0.03% loss essentially every
    # frame arrives with a hole, and without resend a single hole discards
    # all 24.7 MB of it.
    capability = c.GVCP_CAPABILITY_PACKET_RESEND if info.packet_resend else 0
    memory.poke_register(c.BS_GVCP_CAPABILITY, capability)

    memory.poke_register(c.BS_HEARTBEAT_TIMEOUT, info.heartbeat_timeout_ms)

    # Timestamps are nanoseconds, so the tick frequency is 1 GHz.
    memory.poke_register(c.BS_TIMESTAMP_TICK_FREQUENCY_HIGH, 0)
    memory.poke_register(c.BS_TIMESTAMP_TICK_FREQUENCY_LOW, 1000000000)

    memory.poke_register(c.BS_CONTROL_CHANNEL_PRIVILEGE, 0)

    memory.poke_register(c.BS_SC0_PORT, 0)
    memory.poke_register(c.BS_SC0_PACKET_SIZE, info.packet_size)
    memory.poke_register(c.BS_SC0_PACKET_DELAY, 0)
    memory.poke_register(c.BS_SC0_IP_ADDRESS, 0)

    # Left at zero so the viewer's unconditional multipart probe finds the
    # capability bit clear and moves on.
    memory.poke_register(c.BS_SC0_CAPABILITY, 0)
    memory.poke_register(c.BS_SC0_CONFIGURATION, 0)


def discovery_page(memory):
    return memory._read_raw(0, c.DISCOVERY_DATA_SIZE)
