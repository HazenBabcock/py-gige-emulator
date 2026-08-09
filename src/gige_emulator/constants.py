#
# GigE Vision protocol constants.
#
# Values here were checked against the Aravis implementation, which is the
# reference this emulator is tested with. Where a constant is easy to get
# wrong, the reason is recorded next to it rather than in a commit message.
#

# --- GVCP, the control channel -------------------------------------------

GVCP_PORT = 3956

# Byte 0 of every packet. A device must ignore anything that is not CMD.
PACKET_TYPE_ACK = 0x00
PACKET_TYPE_CMD = 0x42
PACKET_TYPE_ERROR = 0x80
PACKET_TYPE_UNKNOWN_ERROR = 0x8F

# Byte 1 of a command packet.
CMD_FLAGS_ACK_REQUIRED = 0x01
CMD_FLAGS_EXTENDED_IDS = 0x10

CMD_DISCOVERY = 0x0002
ACK_DISCOVERY = 0x0003
CMD_BYE = 0x0004
ACK_BYE = 0x0005
CMD_PACKET_RESEND = 0x0040
CMD_READ_REGISTER = 0x0080
ACK_READ_REGISTER = 0x0081
CMD_WRITE_REGISTER = 0x0082
ACK_WRITE_REGISTER = 0x0083
CMD_READ_MEMORY = 0x0084
ACK_READ_MEMORY = 0x0085
CMD_WRITE_MEMORY = 0x0086
ACK_WRITE_MEMORY = 0x0087
ACK_PENDING = 0x0089

# Byte 1 of an error packet. Bytes 0 and 1 together are the GEV status word,
# so 0x80 0x06 reads as 0x8006, GEV_STATUS_ACCESS_DENIED.
ERROR_NOT_IMPLEMENTED = 0x01
ERROR_INVALID_PARAMETER = 0x02
ERROR_INVALID_ACCESS = 0x03
ERROR_WRITE_PROTECT = 0x04
ERROR_BAD_ALIGNMENT = 0x05
ERROR_ACCESS_DENIED = 0x06
ERROR_BUSY = 0x07
ERROR_GENERIC = 0xFF

# The client never asks for more than this in one READMEM, and chunks
# anything larger itself.
GVCP_DATA_SIZE_MAX = 512

# Bits in the GVCP capability register at 0x0934.
#
# Numbered from the LSB, which is the opposite of how the GigE Vision tables
# read: the spec calls packet resend "bit 29", counting from the MSB of a big
# endian word, and that is the same bit as 1 << 2 here. Aravis's own constants
# agree (arvgvcpprivate.h, ARV_GVBS_GVCP_CAPABILITY_PACKET_RESEND is 1 << 2)
# because it reads the register as a u32 and masks. Taking the spec's number
# literally sets bit 29, which advertises nothing this device does and leaves
# resend off -- a client then never asks, and a lost packet still costs a
# whole frame.
GVCP_CAPABILITY_CONCATENATION = 1 << 0
GVCP_CAPABILITY_WRITE_MEMORY = 1 << 1
GVCP_CAPABILITY_PACKET_RESEND = 1 << 2

# --- Bootstrap register map ----------------------------------------------

BS_VERSION = 0x0000
BS_DEVICE_MODE = 0x0004
BS_MAC_HIGH = 0x0008
BS_MAC_LOW = 0x000C
BS_SUPPORTED_IP_CONFIG = 0x0010
BS_CURRENT_IP_CONFIG = 0x0014
BS_CURRENT_IP_ADDRESS = 0x0024
BS_CURRENT_SUBNET_MASK = 0x0034
BS_CURRENT_GATEWAY = 0x0044
BS_MANUFACTURER_NAME = 0x0048
BS_MODEL_NAME = 0x0068
BS_DEVICE_VERSION = 0x0088
BS_MANUFACTURER_INFO = 0x00A8
BS_SERIAL_NUMBER = 0x00D8
BS_USER_DEFINED_NAME = 0x00E8
BS_XML_URL_0 = 0x0200
BS_XML_URL_1 = 0x0400
BS_N_NETWORK_INTERFACES = 0x0600
BS_N_MESSAGE_CHANNELS = 0x0900
BS_N_STREAM_CHANNELS = 0x0904
BS_GVCP_CAPABILITY = 0x0934
BS_HEARTBEAT_TIMEOUT = 0x0938
BS_TIMESTAMP_TICK_FREQUENCY_HIGH = 0x093C
BS_TIMESTAMP_TICK_FREQUENCY_LOW = 0x0940
BS_CONTROL_CHANNEL_PRIVILEGE = 0x0A00
BS_SC0_PORT = 0x0D00
BS_SC0_PACKET_SIZE = 0x0D04
BS_SC0_PACKET_DELAY = 0x0D08
BS_SC0_IP_ADDRESS = 0x0D18
BS_SC0_SOURCE_PORT = 0x0D1C
BS_SC0_CAPABILITY = 0x0D20
BS_SC0_CONFIGURATION = 0x0D24

BS_MANUFACTURER_NAME_SIZE = 32
BS_MODEL_NAME_SIZE = 32
BS_DEVICE_VERSION_SIZE = 32
BS_MANUFACTURER_INFO_SIZE = 48
BS_SERIAL_NUMBER_SIZE = 16
BS_USER_DEFINED_NAME_SIZE = 16
BS_XML_URL_SIZE = 512

# Device mode fields. Bit 31 declares the endianness of the bootstrap
# registers, and it is not optional decoration -- every register here is
# served big endian, so leaving it clear advertises the opposite of what the
# device does. GenICam numbers this bit from the MSB, hence 1 << 31 rather
# than the bit 0 the spec table appears to name.
DEVICE_MODE_BIG_ENDIAN = 1 << 31
DEVICE_MODE_CHARSET_UTF8 = 0x0001

# The discovery ack payload is a verbatim copy of memory [0, 0xf8).
DISCOVERY_DATA_SIZE = 0xF8

CCP_EXCLUSIVE = 1 << 0
CCP_CONTROL = 1 << 1

# The stream port and packet size live in the LOW 16 bits.
#
# This is worth stating loudly because the obvious reading is wrong. Aravis
# describes these registers with <LSB>31</LSB><MSB>16</MSB>, which looks like
# bits 31..16. But GenICam numbers bits from the MSB for a big endian
# register, and arvgcregisternode.c computes lsb = 8*length - register_lsb - 1
# = 0 for a 4 byte register. Aravis's own C constants agree: PACKET_SIZE_MASK
# is 0x0000ffff at POS 0. Reading these as >> 16 yields port 0, and the
# resulting failure -- a camera that is controlled but never streams -- gives
# no clue where to look.
SC_PORT_MASK = 0x0000FFFF
SC_PACKET_SIZE_MASK = 0x0000FFFF

# The top two bits of the packet size register are flags, not size. A client
# sizes its packets by writing a candidate with FIRE_TEST set and seeing
# whether a packet of that size comes back; ignoring the bit is not inert,
# because the probe then fails at every size and the client falls back to the
# 576 byte minimum. DO_NOT_FRAGMENT is what makes the probe mean anything --
# without it an oversized test packet is fragmented, arrives, and the client
# picks a size the path cannot actually carry.
SC_PACKET_SIZE_FIRE_TEST = 1 << 31
SC_PACKET_SIZE_DO_NOT_FRAGMENT = 1 << 30

# Bit positions in the packet size register, in real C bit numbering.
SC_PACKET_BIG_ENDIAN = 1 << 29
SC_PACKET_DO_NOT_FRAGMENT = 1 << 30
SC_PACKET_FIRE_TEST = 1 << 31

# --- Device memory layout ------------------------------------------------

MEMORY_SIZE = 0x10000       # register space; the XML is mapped just above it
BOOTSTRAP_END = 0x1000      # never allocate camera features below this
FEATURE_ARENA_START = 0x8000
FEATURE_ARENA_END = MEMORY_SIZE

# --- GVSP, the stream channel --------------------------------------------

GVSP_PACKET_TYPE_OK = 0x0000
GVSP_PACKET_TYPE_RESEND = 0x0100
GVSP_PACKET_TYPE_UNAVAILABLE = 0x800C

GVSP_CONTENT_LEADER = 0x01
GVSP_CONTENT_TRAILER = 0x02
GVSP_CONTENT_PAYLOAD = 0x03

GVSP_PACKET_ID_MASK = 0x00FFFFFF

PAYLOAD_TYPE_IMAGE = 0x0001

# IP (20) + UDP (8) + GVSP status (2) + GVSP header (6). The client sizes its
# receive buffer at packet_size - 28 and silently truncates anything larger,
# so a datagram must never exceed that.
GVSP_UDP_OVERHEAD = 20 + 8
GVSP_PROTOCOL_OVERHEAD = 20 + 8 + 2 + 6      # == 36

DEFAULT_PACKET_SIZE = 1400

# --- Pixel formats -------------------------------------------------------

PIXEL_FORMAT_MONO8 = 0x01080001
PIXEL_FORMAT_MONO10 = 0x01100003
PIXEL_FORMAT_MONO12 = 0x01100005
PIXEL_FORMAT_MONO16 = 0x01100007
PIXEL_FORMAT_RGB8 = 0x02180014
PIXEL_FORMAT_BGR8 = 0x02180015

# Bayer. A colour sensor's raw output is a mosaic, and labelling it Mono is
# not a harmless approximation -- the client renders a checkerboard and has no
# way to know there is colour to recover. The two letters are the top-left 2x2
# phase, so they change with sensor rotation and must be read from whatever
# the camera reports rather than assumed. Values checked against Aravis's
# arvenums.h.
PIXEL_FORMAT_BAYER_GR8 = 0x01080008
PIXEL_FORMAT_BAYER_RG8 = 0x01080009
PIXEL_FORMAT_BAYER_GB8 = 0x0108000A
PIXEL_FORMAT_BAYER_BG8 = 0x0108000B
PIXEL_FORMAT_BAYER_GR12 = 0x01100010
PIXEL_FORMAT_BAYER_RG12 = 0x01100011
PIXEL_FORMAT_BAYER_GB12 = 0x01100012
PIXEL_FORMAT_BAYER_BG12 = 0x01100013
PIXEL_FORMAT_BAYER_GR16 = 0x0110002E
PIXEL_FORMAT_BAYER_RG16 = 0x0110002F
PIXEL_FORMAT_BAYER_GB16 = 0x01100030
PIXEL_FORMAT_BAYER_BG16 = 0x01100031

PIXEL_FORMAT_NAMES = {
    "Mono8": PIXEL_FORMAT_MONO8,
    "Mono10": PIXEL_FORMAT_MONO10,
    "Mono12": PIXEL_FORMAT_MONO12,
    "Mono16": PIXEL_FORMAT_MONO16,
    "RGB8": PIXEL_FORMAT_RGB8,
    "BGR8": PIXEL_FORMAT_BGR8,
    "BayerGR8": PIXEL_FORMAT_BAYER_GR8,
    "BayerRG8": PIXEL_FORMAT_BAYER_RG8,
    "BayerGB8": PIXEL_FORMAT_BAYER_GB8,
    "BayerBG8": PIXEL_FORMAT_BAYER_BG8,
    "BayerGR12": PIXEL_FORMAT_BAYER_GR12,
    "BayerRG12": PIXEL_FORMAT_BAYER_RG12,
    "BayerGB12": PIXEL_FORMAT_BAYER_GB12,
    "BayerBG12": PIXEL_FORMAT_BAYER_BG12,
    "BayerGR16": PIXEL_FORMAT_BAYER_GR16,
    "BayerRG16": PIXEL_FORMAT_BAYER_RG16,
    "BayerGB16": PIXEL_FORMAT_BAYER_GB16,
    "BayerBG16": PIXEL_FORMAT_BAYER_BG16,
}


def bits_per_pixel(pixel_format):
    """
    Bits per pixel is byte 2 of the PFNC identifier.
    """
    return (pixel_format >> 16) & 0xFF


def payload_size(width, height, pixel_format):
    return width * height * bits_per_pixel(pixel_format) // 8


# Aravis branches on the vendor name to apply per-vendor workarounds, so an
# emulator must not claim to be one of these.
RESERVED_VENDOR_NAMES = (
    "Basler", "Prosilica", "DALSA", "FLIR", "Point Grey Research",
    "XIMEA GmbH", "MATRIX VISION GmbH", "Ricoh Company, Ltd.",
    "The Imaging Source Europe GmbH",
)
