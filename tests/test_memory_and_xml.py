import logging
import struct
import xml.etree.ElementTree as ElementTree

import pytest

from gige_emulator import bootstrap, genicam_xml
from gige_emulator import constants as c
from gige_emulator.camera import EmulatedCamera
from gige_emulator.features import (SFNC_CATEGORIES, CommandFeature,
                                    EnumFeature, FeatureError,
                                    FeatureSet, FloatFeature, IntFeature,
                                    StringFeature)
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


def test_packet_resend_is_advertised_and_nothing_else_is():
    """
    Aravis reads this register once at open and, finding the resend bit
    clear, sets the stream to PACKET_RESEND_NEVER for the whole session --
    so device-side support that is not advertised here is never reached.

    The other bits stay clear because this device does not implement them,
    and a claimed capability is worse than a missing one: the client uses it
    and gets silence.
    """
    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    bootstrap.init_bootstrap(memory, bootstrap.DeviceInfo(ip="10.0.0.1"), 4)
    capability = memory.peek_register(c.BS_GVCP_CAPABILITY)
    assert capability == c.GVCP_CAPABILITY_PACKET_RESEND
    # Counted from the LSB, which is the opposite of the spec's tables. Bit
    # 29 rather than bit 2 would advertise nothing this device does.
    assert capability == 0x00000004
    # The multipart bit lives on the stream channel and stays clear.
    assert memory.peek_register(c.BS_SC0_CAPABILITY) == 0


def test_packet_resend_can_be_turned_off():
    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    bootstrap.init_bootstrap(
        memory, bootstrap.DeviceInfo(ip="10.0.0.1", packet_resend=False), 4)
    assert memory.peek_register(c.BS_GVCP_CAPABILITY) == 0


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


def test_a_reserved_vendor_name_is_served_but_warned_about(caplog):
    """
    Refusing these outright was too strong. pylon's GigE transport layer will
    not enumerate a device at all unless it calls itself Basler, so the name
    has a real use -- but it also changes the behaviour of clients that were
    working, which is what the warning is for.
    """
    with caplog.at_level(logging.WARNING, logger="gige_emulator.bootstrap"):
        info = bootstrap.DeviceInfo(ip="10.0.0.1", manufacturer_name="Basler")
    assert info.manufacturer_name == "Basler"
    assert "Basler" in caplog.text

    memory = DeviceMemory()
    memory.set_genicam_xml(b"<x/>")
    bootstrap.init_bootstrap(memory, info, 4)
    page = bootstrap.discovery_page(memory)
    assert page[0x48:0x4E] == b"Basler"


def test_an_ordinary_vendor_name_says_nothing(caplog):
    with caplog.at_level(logging.WARNING, logger="gige_emulator.bootstrap"):
        bootstrap.DeviceInfo(ip="10.0.0.1", manufacturer_name="py-gige-emulator")
    assert caplog.text == ""


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


def test_an_enumeration_stores_the_entry_name_not_the_ordinal():
    """
    Everything downstream looks an enumeration up by name -- payload_size()
    indexes PIXEL_FORMAT_NAMES with it -- so an ordinal that reached the
    settings dict would only surface much later, as a KeyError from inside
    refresh_geometry().
    """
    feature = EnumFeature("PixelFormat", entries={"Mono8": 0x01080001,
                                                  "Mono16": 0x01100007})
    assert feature.validate("Mono16") == "Mono16"
    assert feature.validate(0x01100007) == "Mono16"


def test_an_enumeration_refuses_a_value_outside_its_entries():
    feature = EnumFeature("PixelFormat", entries={"Mono8": 0x01080001,
                                                  "Mono16": 0x01100007})
    with pytest.raises(FeatureError):
        feature.validate("Mono12")
    with pytest.raises(FeatureError):
        feature.validate(12345)


def test_a_refused_enumeration_says_what_the_choices_were():
    """
    A bare "12345 is not valid" leaves someone reading a device log with
    nothing to compare against, and the ordinals are what a client actually
    wrote.
    """
    feature = EnumFeature("PixelFormat", entries={"Mono8": 0x01080001})
    with pytest.raises(FeatureError) as excinfo:
        feature.validate(12345)
    assert "Mono8" in str(excinfo.value)
    assert str(0x01080001) in str(excinfo.value)


def test_a_string_register_is_padded_to_its_full_length():
    """
    The register is a fixed window, so a short value has to fill it. Leaving
    the tail untouched would serve whatever the previous value left there.
    """
    feature = StringFeature("DeviceUserID", length=16)
    raw = feature.encode("bench-left")
    assert len(raw) == 16
    assert raw == b"bench-left" + b"\x00" * 6
    assert feature.decode(raw) == "bench-left"


def test_an_oversized_string_keeps_its_terminator():
    """
    Truncating to the full length rather than length - 1 would fill the
    window with no NUL in it, and a client reading a string register scans
    for one -- so it would run on into whatever feature was allocated next.
    """
    feature = StringFeature("DeviceUserID", length=16)
    raw = feature.encode("a-name-far-too-long-for-sixteen")
    assert len(raw) == 16
    assert raw.endswith(b"\x00")
    assert len(feature.decode(raw)) == 15


def test_a_string_feature_is_allocated_and_declared_at_one_address():
    """
    A StringReg is both the feature node and its register, unlike an Integer
    with a separate IntReg behind it, so there is only one address and the
    XML has to agree with the allocator about it.
    """
    features = FeatureSet()
    features.add(IntFeature("Width", default=640))
    string = features.add(StringFeature("DeviceUserID", length=16))
    assert string.size == 16
    assert features.lookup_address(string.address) is string

    xml = genicam_xml.build_xml(features, "Model", "Vendor")
    assert genicam_xml.validate_xml(xml, features) == []

    root = ElementTree.fromstring(xml)
    tag = "{%s}" % genicam_xml.SCHEMA_NS
    nodes = [e for e in root.iter(tag + "StringReg")
             if e.get("Name") == "DeviceUserID"]
    assert len(nodes) == 1
    assert int(nodes[0].find(tag + "Address").text, 0) == string.address
    assert int(nodes[0].find(tag + "Length").text) == 16


def _build(features):
    return ElementTree.fromstring(
        genicam_xml.build_xml(features, "Model", "Vendor"))


def test_a_dynamic_bound_replaces_the_literal_rather_than_joining_it():
    """
    GenICam takes <Max> or <pMax>, never both -- a node carrying the pair is
    invalid against the schema, and a client that rejects the document
    presents as a camera that cannot be opened at all.
    """
    features = FeatureSet()
    features.add(FloatFeature("AcquisitionFrameRateMax", default=147.91,
                              access="RO"))
    features.add(FloatFeature("AcquisitionFrameRate", default=10.0,
                              min=0.001, max=147.91,
                              p_max="AcquisitionFrameRateMax"))
    tag = "{%s}" % genicam_xml.SCHEMA_NS
    node = [e for e in _build(features).iter(tag + "Float")
            if e.get("Name") == "AcquisitionFrameRate"][0]

    assert node.find(tag + "pMax").text == "AcquisitionFrameRateMax"
    assert node.find(tag + "Max") is None
    # The static minimum is untouched; only the bound that was pointed at
    # becomes a reference.
    assert node.find(tag + "Min") is not None


def test_the_static_max_survives_as_the_absolute_bound():
    """
    p_max says what is reachable now; max stays what the device can ever do,
    and is what validate() enforces. Dropping it would let a client write a
    rate no mode supports whenever the pointed-at value was briefly stale.
    """
    feature = FloatFeature("AcquisitionFrameRate", default=10.0, min=0.001,
                           max=147.91, p_max="AcquisitionFrameRateMax")
    assert feature.validate(120.0) == 120.0
    with pytest.raises(FeatureError):
        feature.validate(500.0)


def test_an_invalidator_lands_on_the_register_not_the_feature():
    """
    The cached value lives in the register node, so that is what has to be
    invalidated. Marking the feature node alone leaves the register cache
    intact and the client serves the same stale number straight back out.
    """
    features = FeatureSet()
    features.add(FloatFeature("ExposureTime", default=10000.0))
    features.add(FloatFeature("AcquisitionFrameRate", default=10.0,
                              invalidated_by=("ExposureTime",)))
    tag = "{%s}" % genicam_xml.SCHEMA_NS
    root = _build(features)

    reg = [e for e in root.iter(tag + "FloatReg")
           if e.get("Name") == "AcquisitionFrameRateReg"][0]
    assert reg.find(tag + "pInvalidator").text == "ExposureTimeReg"

    node = [e for e in root.iter(tag + "Float")
            if e.get("Name") == "AcquisitionFrameRate"][0]
    assert node.find(tag + "pInvalidator") is None


def test_a_dangling_dependency_is_caught_at_startup():
    """
    A mistyped name produces a document the client rejects at parse time,
    which looks like a camera that cannot be opened rather than like one
    feature being wrong.
    """
    features = FeatureSet()
    features.add(FloatFeature("AcquisitionFrameRate", default=10.0,
                              p_max="NoSuchFeature",
                              invalidated_by=("AlsoMissing",)))
    problems = genicam_xml.validate_xml(
        genicam_xml.build_xml(features, "Model", "Vendor"), features)
    assert any("NoSuchFeature" in p for p in problems)
    assert any("AlsoMissing" in p for p in problems)


def test_a_bound_pointing_at_something_valueless_is_caught():
    features = FeatureSet()
    features.add(CommandFeature("AcquisitionStart"))
    features.add(FloatFeature("AcquisitionFrameRate", default=10.0,
                              p_max="AcquisitionStart"))
    problems = genicam_xml.validate_xml(
        genicam_xml.build_xml(features, "Model", "Vendor"), features)
    assert any("carries no numeric value" in p for p in problems)


def test_a_camera_can_state_its_real_frame_rate_ceiling():
    """
    The default used to be a flat 1000 Hz for every camera ever built on this
    class, and a client believes it -- arv-viewer offers 500 fps on a sensor
    whose fastest readout is 147.
    """
    camera = DummyCamera(width=64, height=64, max_frame_rate=147.91)
    assert camera.feature_set.by_name["AcquisitionFrameRate"].max == 147.91


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


# --- the zipped XML blob -------------------------------------------------

def test_the_zipped_xml_round_trips():
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    blob = genicam_xml.zip_xml(xml, "dummy.xml")
    assert genicam_xml.unzip_xml(blob) == xml


def test_the_archive_holds_exactly_one_entry():
    """
    The client asks for the first name it finds and inflates that one, so a
    second entry is at best ignored and at worst the one it picks.
    """
    import io
    import zipfile
    blob = genicam_xml.zip_xml(b"<x/>", "dummy.xml")
    with zipfile.ZipFile(io.BytesIO(blob)) as archive:
        assert archive.namelist() == ["dummy.xml"]


def test_the_same_feature_set_always_zips_to_the_same_bytes():
    """
    The entry's timestamp is pinned. Left to the clock, the blob's length
    would wander and with it the size in the XML url.
    """
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    assert genicam_xml.zip_xml(xml, "d.xml") == genicam_xml.zip_xml(xml, "d.xml")


def test_zipping_saves_most_of_the_download_round_trips():
    """
    The client reads the blob 512 bytes at a time and cannot be told to read
    more, so what the download costs is chunks, not bytes.
    """
    camera = DummyCamera()
    xml = genicam_xml.build_xml(camera.feature_set, "Dummy", "test-vendor")
    blob = genicam_xml.zip_xml(xml, "dummy.xml")
    plain_chunks = -(-len(xml) // 512)
    zipped_chunks = -(-len(blob) // 512)
    assert plain_chunks >= 8
    assert zipped_chunks * 3 < plain_chunks


def test_deflate_that_does_not_shrink_makes_a_bigger_archive():
    """
    Documents why the server compares the two sizes before serving the
    archive. The client decides an entry is compressed by comparing the
    stored sizes, so deflate output that grew would be copied out raw and
    parsed as garbage -- and an archive is its entry plus about 120 bytes of
    headers, so "the archive got smaller" is exactly the right test.
    """
    import random
    incompressible = random.Random(20250808).randbytes(4096)
    assert len(genicam_xml.zip_xml(incompressible, "x")) > len(incompressible)
