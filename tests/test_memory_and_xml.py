import logging
import struct
import xml.etree.ElementTree as ElementTree

import pytest

from gige_emulator import bootstrap, genicam_xml
from gige_emulator import constants as c
from gige_emulator.camera import EmulatedCamera
from gige_emulator.features import (SFNC_CATEGORIES, FeatureError, FeatureSet,
                                    FloatFeature, IntFeature)
from gige_emulator.memory import DeviceMemory, MemoryError_


class DummyCamera(EmulatedCamera):
    extra_features = (
        FloatFeature("ExposureTime", "", "AcquisitionControl", "RW",
                     default=10000.0, min=1.0, max=1e6, unit="us"),
        IntFeature("GainRaw", "", "AnalogControl", "RW", default=1,
                   min=1, max=22),
    )

    def next_frame(self):
        return b""


# --- memory --------------------------------------------------------------

def test_registers_are_big_endian():
    memory = DeviceMemory()
    memory.write_register(0x8000, 0x01020304)
    assert memory.read(0x8000, 4) == b"\x01\x02\x03\x04"
    assert memory.read_register(0x8000) == 0x01020304


def test_the_xml_region_is_read_only():
    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    with pytest.raises(MemoryError_) as info:
        memory.write(memory.xml_base, b"nope")
    assert info.value.gvcp_error == c.ERROR_WRITE_PROTECT


def test_a_write_straddling_the_xml_boundary_is_refused():
    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    with pytest.raises(MemoryError_):
        memory.write(c.MEMORY_SIZE - 2, b"\x00\x00\x00\x00")


def test_the_xml_is_padded_so_an_over_read_still_succeeds():
    """
    The client rounds a READMEM count up to a multiple of four, so the last
    chunk of an odd sized blob reads off the end. Without padding that comes
    back short and the client's length check fails the whole download.
    """
    xml = b"<a/>" * 7          # 28 bytes, not a multiple of 512
    memory = DeviceMemory()
    size = memory.set_genicam_xml(xml)
    assert size == 28
    data = memory.read(memory.xml_base, 32)
    assert len(data) == 32
    assert data[:28] == xml


def test_reads_far_past_the_xml_return_zeros_not_an_error():
    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    assert memory.read(memory.xml_base + 100000, 16) == b"\x00" * 16


def test_a_read_crossing_from_registers_into_the_xml_is_stitched():
    memory = DeviceMemory()
    memory.poke_bytes(c.MEMORY_SIZE - 4, b"ABCD")
    memory.set_genicam_xml(b"WXYZ")
    assert memory.read(c.MEMORY_SIZE - 4, 8) == b"ABCDWXYZ"


# --- bootstrap -----------------------------------------------------------

def test_the_discovery_page_carries_the_ip_the_client_will_connect_to():
    memory = DeviceMemory()
    info = bootstrap.DeviceInfo(ip="192.168.1.224", netmask="255.255.255.0",
                                mac=bytes.fromhex("f8e43b0b54db"))
    memory.set_genicam_xml(b"<x/>")
    bootstrap.init_bootstrap(memory, info, 4)
    page = bootstrap.discovery_page(memory)
    assert len(page) == c.DISCOVERY_DATA_SIZE
    assert page[0x24:0x28] == bytes([192, 168, 1, 224])


def test_the_mac_lands_at_offsets_0x0a_to_0x0f():
    """
    The client reads the six MAC bytes from 0x0a, not 0x08. Writing them at
    0x08 gives every emulated camera the same physical id.
    """
    memory = DeviceMemory()
    mac = bytes.fromhex("f8e43b0b54db")
    info = bootstrap.DeviceInfo(ip="10.0.0.1", mac=mac)
    memory.set_genicam_xml(b"<x/>")
    bootstrap.init_bootstrap(memory, info, 4)
    page = bootstrap.discovery_page(memory)
    assert page[0x0A:0x10] == mac


def test_the_xml_url_is_hex_without_a_prefix():
    memory = DeviceMemory()
    info = bootstrap.DeviceInfo(ip="10.0.0.1", xml_filename="camera.xml")
    memory.set_genicam_xml(b"x" * 0x3F53)
    bootstrap.init_bootstrap(memory, info, 0x3F53)
    url = memory.peek_string(c.BS_XML_URL_0, c.BS_XML_URL_SIZE)
    assert url == "Local:///camera.xml;10000;3f53"


def test_resend_and_multipart_capability_bits_stay_clear():
    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    bootstrap.init_bootstrap(memory, bootstrap.DeviceInfo(ip="10.0.0.1"), 4)
    assert memory.peek_register(c.BS_GVCP_CAPABILITY) == 0
    assert memory.peek_register(c.BS_SC0_CAPABILITY) == 0


def test_device_mode_declares_the_endianness_it_actually_serves():
    """
    Every register is written big endian, so bit 31 has to say so. It was
    clear for a while, which advertised a little endian device serving big
    endian registers -- Aravis never checks, so nothing failed visibly.
    """
    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    bootstrap.init_bootstrap(memory, bootstrap.DeviceInfo(ip="10.0.0.1"), 4)
    mode = memory.peek_register(c.BS_DEVICE_MODE)
    assert mode & c.DEVICE_MODE_BIG_ENDIAN
    assert mode & 0xFFFF == c.DEVICE_MODE_CHARSET_UTF8

    # And it has to survive into the ack, which is the only place a client
    # ever sees it.
    page = bootstrap.discovery_page(memory)
    assert struct.unpack_from(">I", page, c.BS_DEVICE_MODE)[0] == mode


def test_a_reserved_vendor_name_is_refused():
    with pytest.raises(ValueError):
        bootstrap.DeviceInfo(ip="10.0.0.1", manufacturer_name="Basler")


# --- features ------------------------------------------------------------

def test_features_land_in_the_arena_and_are_aligned():
    camera = DummyCamera()
    for feature in camera.feature_set.features:
        assert c.FEATURE_ARENA_START <= feature.address < c.FEATURE_ARENA_END
        assert feature.address % (8 if feature.size == 8 else 4) == 0


def test_the_arena_never_reaches_the_xml_url_fields():
    camera = DummyCamera()
    for feature in camera.feature_set.features:
        assert not (c.BS_XML_URL_0 <= feature.address < c.BS_XML_URL_1 + 512)


def test_a_misspelled_category_is_named_and_corrected(caplog):
    """
    A typo does not fail, it quietly creates a category holding one feature.
    In a viewer that is a stray node and nothing else, so the warning has to
    carry the nearest real name to be worth anything.
    """
    features = FeatureSet()
    with caplog.at_level(logging.WARNING, logger="gige_emulator.features"):
        features.add(IntFeature("Foo", "", "AqcuisitionControl", "RW"))
    assert "AqcuisitionControl" in caplog.text
    assert "did you mean 'AcquisitionControl'" in caplog.text


def test_an_unusual_category_warns_once_not_once_per_feature(caplog):
    features = FeatureSet()
    with caplog.at_level(logging.WARNING, logger="gige_emulator.features"):
        for i in range(4):
            features.add(IntFeature("Foo%d" % i, "", "FRETControl", "RW"))
    assert caplog.text.count("FRETControl") == 1
    # Invented categories are legitimate, so no suggestion is fabricated and
    # the features are still added.
    assert "did you mean" not in caplog.text
    assert features.categories() == ["FRETControl"]


def test_standard_categories_are_silent(caplog):
    features = FeatureSet()
    with caplog.at_level(logging.WARNING, logger="gige_emulator.features"):
        for i, category in enumerate(sorted(SFNC_CATEGORIES)):
            features.add(IntFeature("Foo%d" % i, "", category, "RW"))
    assert caplog.text == ""


def test_the_built_in_features_use_standard_categories():
    """
    Every camera inherits these, so a wrong category here is inherited too --
    and the rule in Feature's docstring is only worth writing down if the
    features shipped with the library follow it.
    """
    camera = DummyCamera()
    for feature in camera.feature_set.features:
        assert feature.category in SFNC_CATEGORIES, feature.name


def test_category_namespace_follows_where_the_name_came_from():
    """
    NameSpace says whether the *name* is the convention's or this device's
    own, so a convention name declared Custom claims authorship of a name it
    did not invent. No client checks, but it is backwards.
    """
    class Mixed(EmulatedCamera):
        extra_features = (
            IntFeature("GainRaw", "", "AnalogControl", "RW", default=1,
                       min=1, max=22),
            IntFeature("Ratio", "", "FRETControl", "RW", default=1,
                       min=0, max=100),
        )

        def next_frame(self):
            return b""

    camera = Mixed(width=64, height=48, pixel_format="Mono8")
    xml = genicam_xml.build_xml(camera.feature_set, "Mixed", "vendor")
    root = ElementTree.fromstring(xml)
    tag = root.tag[:root.tag.index("}") + 1]

    spaces = {e.get("Name"): e.get("NameSpace")
              for e in root.iter(tag + "Category")}
    assert spaces["Root"] == "Standard"
    assert spaces["AnalogControl"] == "Standard"
    assert spaces["AcquisitionControl"] == "Standard"
    # Invented, so it really is this device's own name.
    assert spaces["FRETControl"] == "Custom"


def test_feature_namespace_follows_where_the_name_came_from():
    """
    Checked against a vendor-authored GenICam XML: feature nodes and enum
    entries carry Standard when the convention defines the name, and the
    register behind each feature is named here so it stays Custom.
    """
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "vendor")
    root = ElementTree.fromstring(xml)
    tag = root.tag[:root.tag.index("}") + 1]

    spaces = {}
    for node in root.iter():
        if node.get("Name") and node.get("NameSpace"):
            spaces[(node.tag[len(tag):], node.get("Name"))] = node.get("NameSpace")

    assert spaces[("Integer", "Width")] == "Standard"
    assert spaces[("Enumeration", "PixelFormat")] == "Standard"
    assert spaces[("EnumEntry", "Mono8")] == "Standard"
    assert spaces[("Command", "AcquisitionStart")] == "Standard"
    assert spaces[("Float", "ExposureTime")] == "Standard"
    # The register is this generator's own name, not the convention's.
    assert spaces[("IntReg", "WidthReg")] == "Custom"


def test_gainraw_is_not_claimed_as_a_standard_name():
    """
    The convention's gain feature is `Gain`, a float in dB with a
    GainSelector. GainRaw is the GenICam 1.x integer form, kept because it is
    what the examples already use -- but it is this device's own name, and
    declaring it Standard would assert units it does not have.
    """
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "vendor")
    root = ElementTree.fromstring(xml)
    tag = root.tag[:root.tag.index("}") + 1]
    node = [e for e in root.iter(tag + "Integer")
            if e.get("Name") == "GainRaw"][0]
    assert node.get("NameSpace") == "Custom"


def test_payload_size_is_a_transport_layer_feature():
    """
    It follows from the geometry but it counts bytes on the stream channel,
    which is where a vendor XML files it too.
    """
    features = DummyCamera().feature_set.by_name
    assert features["PayloadSize"].category == "TransportLayerControl"


def test_exposure_and_gain_land_in_the_two_categories_people_confuse():
    """
    Pinning the case the docstring calls out: they are tuned together and
    shown side by side, but exposure is time and gain is amplitude.
    """
    features = DummyCamera().feature_set.by_name
    assert features["ExposureTime"].category == "AcquisitionControl"
    assert features["GainRaw"].category == "AnalogControl"


def test_a_gev_prefixed_feature_is_refused():
    """
    The client injects its own transport layer nodes and skips any name
    already present, so ours would silently replace working plumbing.
    """
    features = FeatureSet()
    with pytest.raises(FeatureError):
        features.add(IntFeature("GevSCPSPacketSize"))


def test_duplicate_feature_names_are_refused():
    features = FeatureSet()
    features.add(IntFeature("Width"))
    with pytest.raises(FeatureError):
        features.add(IntFeature("Width"))


def test_payload_size_matches_the_geometry():
    camera = DummyCamera(width=640, height=480, pixel_format="Mono16")
    expected = 640 * 480 * 2
    assert camera.payload_size() == expected
    assert camera.settings["PayloadSize"] == expected
    camera.latch_geometry()
    assert camera.geometry["payload"] == expected


def test_payload_size_tracks_the_pixel_format():
    mono8 = DummyCamera(width=640, height=480, pixel_format="Mono8")
    mono16 = DummyCamera(width=640, height=480, pixel_format="Mono16")
    assert mono16.settings["PayloadSize"] == 2 * mono8.settings["PayloadSize"]


# --- generated XML -------------------------------------------------------

def test_the_generated_xml_is_well_formed_and_self_consistent():
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    assert genicam_xml.validate_xml(xml, camera.feature_set) == []
    ElementTree.fromstring(xml)


def test_the_xml_declares_a_device_port():
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    assert b'<Port Name="Device"' in xml


def test_the_xml_never_declares_the_fire_test_packet_feature():
    """
    Declaring it makes the client binary search packet sizes by firing test
    packets this device does not answer.
    """
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    assert b"FireTestPacket" not in xml


def test_every_address_in_the_xml_matches_the_allocator():
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    root = ElementTree.fromstring(xml)
    tag = "{%s}" % genicam_xml.SCHEMA_NS
    found = 0
    for kind in ("IntReg", "FloatReg", "StringReg"):
        for element in root.iter(tag + kind):
            address = int(element.find(tag + "Address").text, 0)
            assert camera.feature_set.lookup_address(address) is not None
            found += 1
    assert found == len(camera.feature_set.features)


def test_the_schema_is_1_0_1():
    """
    Below 1.1.0 the client uses READ_REGISTER for 4 byte accesses, which is
    what the reference device does and what this is tested against.
    """
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    root = ElementTree.fromstring(xml)
    assert root.get("SchemaMajorVersion") == "1"
    assert root.get("SchemaMinorVersion") == "0"
    assert root.get("SchemaSubMinorVersion") == "1"
