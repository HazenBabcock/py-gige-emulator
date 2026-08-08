#
# End to end over loopback, with no Aravis and no second machine.
#

import time
import xml.etree.ElementTree as ElementTree

import pytest

from fakeclient import FakeClient, FakeClientError
from gige_emulator import (EmulatedCamera, FloatFeature, GigECameraServer,
                           IntFeature)
from gige_emulator import constants as c

WIDTH, HEIGHT = 64, 48
PORT = 13956


class PatternCamera(EmulatedCamera):
    """
    Returns a deterministic frame so a round trip can be checked byte for
    byte rather than just "something arrived".
    """

    extra_features = (
        FloatFeature("ExposureTime", "", "AcquisitionControl", "RW",
                     default=10000.0, min=1.0, max=1e6, unit="us"),
        IntFeature("GainRaw", "", "AnalogControl", "RW", default=1,
                   min=1, max=22),
        # Stands in for the Pi example's binning: writing it changes the
        # frame size, so it must be refused while acquiring.
        IntFeature("Binning", "", "ImageFormatControl", "RW",
                   affects_payload=True, default=1, min=1, max=4),
    )

    def __init__(self, **kwds):
        super().__init__(**kwds)
        self.applied = []
        self.reads = 0
        self.frame_index = 0

    def pattern(self, index):
        size = self.payload_size()
        return bytes(((i * 7 + index * 13) & 0xFF) for i in range(size))

    def next_frame(self):
        # No sensor to wait on, so pace here -- the stream thread has no
        # timer and would otherwise spin as fast as this returns.
        rate = self.settings.get("AcquisitionFrameRate", 0.0)
        if rate and rate > 0:
            time.sleep(1.0 / rate)
        frame = self.pattern(self.frame_index)
        self.frame_index += 1
        return frame

    def set_camera_settings(self, changed):
        self.applied.append(dict(changed))
        if "Binning" in changed:
            # Same shape as the Pi example: the camera changes its own
            # geometry from inside the hook, and the emulator republishes it.
            factor = changed["Binning"]
            self.settings["Width"] = WIDTH // factor
            self.settings["Height"] = HEIGHT // factor

    def get_camera_settings(self):
        self.reads += 1
        return {}


@pytest.fixture
def server():
    camera = PatternCamera(width=WIDTH, height=HEIGHT, pixel_format="Mono8",
                           pixel_formats=["Mono8", "Mono16"], frame_rate=50.0)
    srv = GigECameraServer(camera, ip="127.0.0.1", netmask="255.0.0.0",
                           mac=bytes.fromhex("020000000001"),
                           model_name="PatternCam", serial_number="TEST-1",
                           gvcp_port=PORT, bind_address="127.0.0.1",
                           heartbeat_timeout_ms=1000)
    srv.start()
    yield srv
    srv.stop()


@pytest.fixture
def client(server):
    with FakeClient(("127.0.0.1", PORT)) as c_:
        yield c_


def test_discovery_reports_the_device_identity(client):
    info = client.discover()
    assert info["ip"] == "127.0.0.1"
    assert info["model"] == "PatternCam"
    assert info["serial"] == "TEST-1"
    assert info["mac"] == bytes.fromhex("020000000001")


def test_the_genicam_xml_downloads_and_parses(client, server):
    url, xml = client.fetch_genicam_xml()
    assert url.startswith("Local:///")
    assert xml == server.xml
    root = ElementTree.fromstring(xml)
    assert root.get("ModelName") == "PatternCam"


def test_control_is_taken_and_released(client, server):
    assert not server.control.has_control()
    client.take_control()
    assert server.control.has_control()
    assert client.heartbeat() & c.CCP_CONTROL
    client.release_control()
    assert not server.control.has_control()


def test_a_frame_round_trips_byte_for_byte(client, server):
    camera = server.camera
    client.take_control()
    packet_size = client.open_stream()
    start = camera.feature_set.by_name["AcquisitionStart"].address
    client.write_register(start, 1)

    block_id, leader, data = client.receive_frame(packet_size)

    assert block_id != 0, "frame id 0 is not valid"
    assert leader["width"] == WIDTH
    assert leader["height"] == HEIGHT
    assert leader["pixel_format"] == c.PIXEL_FORMAT_MONO8
    assert len(data) == WIDTH * HEIGHT
    # The frame must be one the camera actually produced.
    assert data in [camera.pattern(i) for i in range(camera.frame_index + 1)]


def test_several_frames_arrive_with_increasing_ids(client, server):
    client.take_control()
    packet_size = client.open_stream()
    start = server.camera.feature_set.by_name["AcquisitionStart"].address
    client.write_register(start, 1)

    ids = []
    for _ in range(4):
        block_id, leader, data = client.receive_frame(packet_size)
        ids.append(block_id)
    assert all(i != 0 for i in ids)
    assert ids == sorted(ids), "frame ids must strictly increase"
    assert len(set(ids)) == len(ids)


def test_acquisition_stop_halts_the_stream(client, server):
    client.take_control()
    packet_size = client.open_stream()
    features = server.camera.feature_set.by_name
    client.write_register(features["AcquisitionStart"].address, 1)
    client.receive_frame(packet_size)

    client.write_register(features["AcquisitionStop"].address, 1)
    assert not server.camera.acquiring
    time.sleep(0.2)
    with pytest.raises(FakeClientError):
        client.receive_frame(packet_size, timeout=1.0)


def test_a_command_register_self_clears(client, server):
    client.take_control()
    features = server.camera.feature_set.by_name
    client.write_register(features["AcquisitionStart"].address, 1)
    # A GenICam command must read back as zero, or it can only fire once.
    assert client.read_register(features["AcquisitionStart"].address) == 0


def test_writing_a_feature_reaches_set_camera_settings(client, server):
    camera = server.camera
    client.take_control()
    client.write_register(camera.feature_set.by_name["GainRaw"].address, 7)
    assert {"GainRaw": 7} in camera.applied
    assert camera.settings["GainRaw"] == 7
    assert client.read_register(camera.feature_set.by_name["GainRaw"].address) == 7


def test_a_float_feature_round_trips_through_read_memory(client, server):
    """
    Floats are 8 bytes, so they arrive as READ_MEMORY rather than
    READ_REGISTER. Hooks keyed off the command id would never fire here.
    """
    camera = server.camera
    client.take_control()
    feature = camera.feature_set.by_name["ExposureTime"]
    import struct
    client.write_memory(feature.address, struct.pack(">d", 25000.0))
    assert {"ExposureTime": 25000.0} in camera.applied
    raw = client.read_memory(feature.address, 8)
    assert struct.unpack(">d", raw)[0] == 25000.0


def test_an_out_of_range_value_is_rejected_and_rolled_back(client, server):
    camera = server.camera
    client.take_control()
    feature = camera.feature_set.by_name["GainRaw"]
    client.write_register(feature.address, 5)
    with pytest.raises(FakeClientError):
        client.write_register(feature.address, 999)     # max is 22
    assert camera.settings["GainRaw"] == 5
    assert client.read_register(feature.address) == 5


def test_a_payload_feature_is_refused_while_acquiring(client, server):
    """
    The client sized its buffers from PayloadSize at AcquisitionStart, so a
    geometry change now would leave it dropping every packet past the old
    count with nothing reported. It has to be refused, not silently latched.
    """
    camera = server.camera
    binning = camera.feature_set.by_name["Binning"]
    client.take_control()

    client.write_register(binning.address, 2)      # allowed while stopped
    assert camera.settings["Binning"] == 2

    packet_size = client.open_stream()
    client.write_register(
        camera.feature_set.by_name["AcquisitionStart"].address, 1)
    client.receive_frame(packet_size)

    with pytest.raises(FakeClientError):
        client.write_register(binning.address, 4)
    assert camera.settings["Binning"] == 2
    assert client.read_register(binning.address) == 2

    client.write_register(
        camera.feature_set.by_name["AcquisitionStop"].address, 1)
    client.write_register(binning.address, 4)      # allowed again
    assert camera.settings["Binning"] == 4


def test_a_payload_feature_republishes_the_geometry(client, server):
    camera = server.camera
    client.take_control()
    binning = camera.feature_set.by_name["Binning"]
    width = camera.feature_set.by_name["Width"]
    payload = camera.feature_set.by_name["PayloadSize"]

    client.write_register(binning.address, 2)
    # The camera changed Width from inside set_camera_settings, so the
    # registers must have followed without anyone writing them.
    assert client.read_register(width.address) == WIDTH // 2
    assert client.read_register(payload.address) == (WIDTH // 2) * (HEIGHT // 2)


def test_a_read_only_feature_is_refused(client, server):
    camera = server.camera
    client.take_control()
    feature = camera.feature_set.by_name["Width"]
    with pytest.raises(FakeClientError):
        client.write_register(feature.address, 32)
    assert client.read_register(feature.address) == WIDTH


def test_a_non_controller_cannot_write(server, client):
    client.take_control()
    with FakeClient(("127.0.0.1", PORT), timeout=0.4) as intruder:
        # Reads are always served.
        assert intruder.read_register(c.BS_CONTROL_CHANNEL_PRIVILEGE) & c.CCP_CONTROL
        with pytest.raises(FakeClientError):
            intruder.write_register(
                server.camera.feature_set.by_name["GainRaw"].address, 3)


def test_control_expires_when_the_client_goes_quiet(server):
    with FakeClient(("127.0.0.1", PORT)) as first:
        first.take_control()
        assert server.control.has_control()
    # heartbeat_timeout_ms is 1000 for this fixture
    time.sleep(1.4)
    assert not server.control.has_control()
    assert not server.camera.acquiring

    with FakeClient(("127.0.0.1", PORT)) as second:
        second.take_control()
        assert server.control.has_control()


def test_get_camera_settings_is_called_on_a_feature_read(client, server):
    camera = server.camera
    before = camera.reads
    client.read_register(camera.feature_set.by_name["GainRaw"].address)
    assert camera.reads > before


def test_reading_a_bootstrap_register_does_not_call_the_hook(client, server):
    camera = server.camera
    before = camera.reads
    client.read_register(c.BS_N_STREAM_CHANNELS)
    assert camera.reads == before


def test_a_duplicate_command_id_is_answered_twice(client, server):
    """
    The client reuses the same packet id when it retries, so a device that
    deduplicates would hang a retrying client forever.
    """
    import struct
    payload = struct.pack(">I", c.BS_N_STREAM_CHANNELS)
    first = client._command(c.CMD_READ_REGISTER, payload,
                            c.ACK_READ_REGISTER, packet_id=4242)
    second = client._command(c.CMD_READ_REGISTER, payload,
                             c.ACK_READ_REGISTER, packet_id=4242)
    assert first == second == struct.pack(">I", 1)
