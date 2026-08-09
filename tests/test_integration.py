#
# End to end over loopback, with no Aravis and no second machine.
#

import struct
import time
import xml.etree.ElementTree as ElementTree

import pytest

from fakeclient import FakeClient, FakeClientError
from gige_emulator import (EmulatedCamera, FloatFeature, GigECameraServer,
                           IntFeature, StringFeature)
from gige_emulator import constants as c
from gige_emulator import stream as stream_module

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
        return self.pattern_for(index, self.payload_size())

    @staticmethod
    def pattern_for(index, size):
        """The same pattern at an explicit size, so a test can regenerate it
        for a frame it has reassembled without guessing the geometry."""
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


def test_the_user_defined_name_reaches_the_discovery_ack():
    """
    It is the one identity field a client can select on but does not list --
    the device id is always vendor-model-serial -- so a name that is working
    is indistinguishable from one that was ignored unless you look here.
    """
    camera = PatternCamera(width=8, height=8, pixel_format="Mono8")
    srv = GigECameraServer(camera, ip="127.0.0.1", netmask="255.0.0.0",
                           model_name="PatternCam", serial_number="TEST-1",
                           user_defined_name="bench-left",
                           gvcp_port=PORT + 2, bind_address="127.0.0.1")
    srv.start()
    try:
        with FakeClient(("127.0.0.1", PORT + 2)) as named:
            info = named.discover()
        assert info["user_defined_name"] == "bench-left"
        # And it is genuinely separate from the id a client would list.
        assert info["serial"] == "TEST-1"
        assert info["model"] == "PatternCam"
    finally:
        srv.stop()


def test_an_unset_user_defined_name_is_empty_not_missing(client):
    # The shared fixture sets no name, and the field must still be there and
    # readable rather than absent or full of stale bytes.
    assert client.discover()["user_defined_name"] == ""


def test_the_genicam_xml_downloads_and_parses(client, server):
    url, xml = client.fetch_genicam_xml()
    assert url.startswith("Local:///")
    assert xml == server.xml
    root = ElementTree.fromstring(xml)
    assert root.get("ModelName") == "PatternCam"


def test_the_xml_is_served_zipped_and_the_url_says_so(client, server):
    """
    The client reads the blob in fixed 512 byte chunks, so what the download
    costs is round trips. Compressing it is most of the time it takes to
    open a camera on a slow link -- measured at 16 chunks against 3 on the
    Pi example, out of 28 round trips for the whole open.

    The filename is the only thing that tells the client to inflate, so the
    two have to be checked together: a .zip url over plain xml, or the other
    way round, is a device that opens nowhere.
    """
    url, xml = client.fetch_genicam_xml()
    path = url.rsplit(";", 2)[0]
    assert path.endswith(".xml.zip")
    assert server.xml_blob != server.xml
    assert server.xml_blob.startswith(b"PK\x03\x04")

    # And the size in the url is the blob's, not the xml's -- the client
    # reads exactly that many bytes before trying to inflate them.
    size = int(url.rsplit(";", 1)[1], 16)
    assert size == len(server.xml_blob)
    assert xml == server.xml


def test_the_served_blob_is_never_larger_than_the_xml(server):
    assert len(server.xml_blob) <= len(server.xml)


def test_xml_compression_can_be_turned_off(server):
    """
    Kept as an escape hatch for a client that cannot inflate. Nothing in
    the standard makes it optional, but this device exists to be pointed at
    clients that turn out to be strange.
    """
    camera = PatternCamera(width=8, height=8, pixel_format="Mono8")
    srv = GigECameraServer(camera, ip="127.0.0.1", netmask="255.0.0.0",
                           model_name="PatternCam", serial_number="TEST-1",
                           gvcp_port=PORT + 3, bind_address="127.0.0.1",
                           compress_xml=False)
    srv.start()
    try:
        with FakeClient(("127.0.0.1", PORT + 3)) as plain:
            url, xml = plain.fetch_genicam_xml()
        assert url.rsplit(";", 2)[0].endswith(".xml")
        assert not url.rsplit(";", 2)[0].endswith(".zip")
        assert srv.xml_blob == srv.xml
        assert xml == srv.xml
    finally:
        srv.stop()


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


def test_single_frame_mode_delivers_one_frame_and_stops(client, server):
    """
    The mode was advertised and never read, so a client selecting it got a
    continuous stream -- a wrong answer rather than a missing feature, and
    one it cannot tell from the device ignoring the request.
    """
    features = server.camera.feature_set.by_name
    client.take_control()
    client.write_register(features["AcquisitionMode"].address, 2)
    assert server.camera.settings["AcquisitionMode"] == "SingleFrame"

    packet_size = client.open_stream()
    client.write_register(features["AcquisitionStart"].address, 1)
    client.receive_frame(packet_size)

    # The stream thread clears this once the frame is out, so give it the
    # one poll interval it needs rather than racing the assertion.
    deadline = time.monotonic() + 2.0
    while server.camera.acquiring and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not server.camera.acquiring
    assert server.stats["frames"] == 1

    with pytest.raises(FakeClientError):
        client.receive_frame(packet_size, timeout=1.0)


def test_single_frame_mode_can_be_started_again(client, server):
    """
    Stopping after one frame must leave the device startable, not merely
    idle -- a snapshot client takes many single frames in a row.
    """
    features = server.camera.feature_set.by_name
    client.take_control()
    client.write_register(features["AcquisitionMode"].address, 2)
    packet_size = client.open_stream()

    for expected in (1, 2, 3):
        client.write_register(features["AcquisitionStart"].address, 1)
        client.receive_frame(packet_size)
        deadline = time.monotonic() + 2.0
        while server.camera.acquiring and time.monotonic() < deadline:
            time.sleep(0.01)
        assert server.stats["frames"] == expected


def test_continuous_mode_is_unaffected(client, server):
    features = server.camera.feature_set.by_name
    client.take_control()
    assert server.camera.settings["AcquisitionMode"] == "Continuous"

    packet_size = client.open_stream()
    client.write_register(features["AcquisitionStart"].address, 1)
    for _ in range(3):
        client.receive_frame(packet_size)
    assert server.camera.acquiring
    client.write_register(features["AcquisitionStop"].address, 1)


def test_a_resent_packet_completes_the_frame_byte_for_byte(client, server):
    """
    The case resend exists for. A full resolution frame is ~17,000 packets,
    and at a measured 0.03% loss essentially every one arrives with a hole;
    without resend a single hole discards the whole frame.
    """
    camera = server.camera
    client.take_control()
    # Mono16 so the frame spans enough packets for a dropped one to be in the
    # middle rather than at an edge. Before AcquisitionStart, because the
    # geometry is latched there and the device refuses it afterwards.
    client.write_register(
        camera.feature_set.by_name["PixelFormat"].address,
        c.PIXEL_FORMAT_MONO16)
    packet_size = client.open_stream()
    client.write_register(
        camera.feature_set.by_name["AcquisitionStart"].address, 1)

    block, leader, data = client.receive_frame_dropping(packet_size, [2, 3])

    assert len(data) == WIDTH * HEIGHT * 2
    # Byte for byte against the pattern the camera generated. A resent packet
    # written at the wrong offset would still give the right length, and the
    # client would call the frame complete -- so length alone proves nothing.
    index = None
    for candidate in range(camera.frame_index + 1):
        if data == camera.pattern_for(candidate, len(data)):
            index = candidate
            break
    assert index is not None, "reassembled frame matches no generated pattern"
    assert server.stats["resent_packets"] >= 2
    client.write_register(
        camera.feature_set.by_name["AcquisitionStop"].address, 1)


def test_stopping_releases_the_frame_held_for_resends(client, server):
    """
    A frame is kept after sending so a resend can be answered from it. That
    retention must end when the stream goes quiet, or a camera left idle
    holds its last frame for as long as the process lives -- 24.7 MB at full
    resolution, for a request that is never coming.
    """
    camera = server.camera
    client.take_control()
    packet_size = client.open_stream()
    client.write_register(
        camera.feature_set.by_name["AcquisitionStart"].address, 1)
    client.receive_frame(packet_size)

    client.write_register(
        camera.feature_set.by_name["AcquisitionStop"].address, 1)

    deadline = time.monotonic() + 2.0
    while server.stream._retained and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not server.stream._retained


def test_restarting_delivers_a_fresh_frame_not_the_retained_one(client, server):
    """
    The retained frame is only ever read to answer a resend naming its own
    id, never sent as a new frame -- so a client that stops, waits, and
    starts again gets a live image rather than whatever was in flight when
    it left. Worth pinning down: the two paths both end in sendto() on the
    same socket, and nothing but this test says which frame each one sends.
    """
    camera = server.camera
    client.take_control()
    packet_size = client.open_stream()
    start = camera.feature_set.by_name["AcquisitionStart"].address
    stop = camera.feature_set.by_name["AcquisitionStop"].address

    client.write_register(start, 1)
    for _ in range(2):
        _, _, before = client.receive_frame(packet_size)
    client.write_register(stop, 1)

    def pattern_index(data):
        for candidate in range(camera.frame_index + 2):
            if data == camera.pattern_for(candidate, len(data)):
                return candidate
        return None

    index_before = pattern_index(before)
    assert index_before is not None

    # Long enough that a device replaying its buffer would be obvious.
    time.sleep(0.5)

    client.write_register(start, 1)
    _, _, after = client.receive_frame(packet_size)
    index_after = pattern_index(after)

    assert index_after is not None, "frame after restart matches no pattern"
    assert index_after > index_before, (
        "restart replayed frame %s; expected something newer than %s"
        % (index_after, index_before))
    client.write_register(stop, 1)


def test_a_resend_larger_than_the_frame_can_afford_is_refused_whole(client,
                                                                    server):
    """
    A request for a large run means the client lost hundreds of packets, not
    one. Repairing that costs bandwidth the next frame needs, and at full
    link utilisation there is none spare -- so the repair starves the next
    frame, which then needs repairing too, and the stream never recovers.

    Serving part of it is the worst of both: the frame still cannot complete
    without the rest, so every packet sent for it is wasted at exactly the
    moment the link is most oversubscribed. That is what a truncating
    version did, and it showed up as the client timing out on frames the
    device had just spent its bandwidth on.
    """
    camera = server.camera
    client.take_control()
    packet_size = client.open_stream()
    client.write_register(
        camera.feature_set.by_name["AcquisitionStart"].address, 1)
    block, _, _ = client.receive_frame(packet_size)

    before_sent = server.stats["resent_packets"]
    before_refused = server.stats["resend_refused"]
    # Far more than the frame has, so certainly over the fraction.
    client.request_resend(block, 1, 10000)

    deadline = time.monotonic() + 2.0
    while (server.stats["resend_refused"] == before_refused
           and time.monotonic() < deadline):
        time.sleep(0.01)
    assert server.stats["resend_refused"] == before_refused + 1
    # Refused, not partly served: nothing was spent on a frame that could
    # not have completed.
    assert server.stats["resent_packets"] == before_sent
    client.write_register(
        camera.feature_set.by_name["AcquisitionStop"].address, 1)


def test_the_stream_leaves_the_link_some_room(server):
    """
    The remainder is what resends travel in. Sending flat out means a lost
    run can only be repaired by taking bandwidth from the next frame.
    """
    assert 0 < server.stream.link_utilisation < 1.0
    # A frame that took 100 ms of wire time is followed by a pause, not by
    # the next frame immediately.
    started = time.monotonic()
    server.stream._pace(0.1)
    assert time.monotonic() - started >= 0.01


def test_a_resend_for_a_frame_already_released_says_so(client, server):
    """
    Silence would cost the client its whole retention timeout before it gave
    up on a frame the device cannot complete anyway. The unavailable status
    is what makes Aravis stop asking and move on.
    """
    camera = server.camera
    client.take_control()
    packet_size = client.open_stream()
    client.write_register(
        camera.feature_set.by_name["AcquisitionStart"].address, 1)
    block, _, _ = client.receive_frame(packet_size)
    client.write_register(
        camera.feature_set.by_name["AcquisitionStop"].address, 1)
    time.sleep(0.2)

    before = server.stats["resend_unavailable"]
    # A frame id that was never sent, so certainly not retained.
    client.request_resend((block + 500) & 0xFFFF, 1, 4)
    deadline = time.monotonic() + 2.0
    while server.stats["resend_unavailable"] == before and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server.stats["resend_unavailable"] == before + 1

    status, block_id, infos = struct.unpack_from(
        ">HHI", client.stream_socket.recv(packet_size), 0)
    assert status == c.GVSP_PACKET_TYPE_UNAVAILABLE


def test_a_packet_size_probe_is_answered_at_the_requested_size(client, server):
    """
    A client sizes its receive path by asking the device to fire a packet of a
    candidate size. Ignoring the request is not inert: every probe times out,
    the client walks down and settles on the 576 byte minimum, which is what
    ImpactAcquire was measured doing.
    """
    client.take_control()
    packet_size = client.open_stream(packet_size=1400)
    client.write_register(c.BS_SC0_PACKET_SIZE,
                          c.SC_PACKET_SIZE_FIRE_TEST | packet_size)

    data, _ = client.stream_socket.recvfrom(4096)
    assert len(data) == packet_size - 28          # less the IP and UDP headers
    assert server.stream.n_test_packets == 1


def test_the_fire_bit_clears_but_the_size_survives(client, server):
    """
    The fire bit is a trigger, not state. Left set, a client reading the
    register back sees a probe still pending; cleared along with everything
    else, it loses the settings the stream is about to run with.
    """
    client.take_control()
    client.open_stream(packet_size=1400)
    client.write_register(
        c.BS_SC0_PACKET_SIZE,
        c.SC_PACKET_SIZE_FIRE_TEST | c.SC_PACKET_SIZE_DO_NOT_FRAGMENT | 1400)
    client.stream_socket.recvfrom(4096)

    value = client.read_register(c.BS_SC0_PACKET_SIZE)
    assert not value & c.SC_PACKET_SIZE_FIRE_TEST
    assert value & c.SC_PACKET_SIZE_DO_NOT_FRAGMENT
    assert value & c.SC_PACKET_SIZE_MASK == 1400


def test_a_probe_does_not_disturb_the_frames_that_follow(client, server):
    """
    The test packet goes out on the stream socket carrying a GVSP header, so
    the risk is that it lands in the client's reassembler. Block id zero is
    what keeps it out.
    """
    client.take_control()
    packet_size = client.open_stream(packet_size=1400)
    client.write_register(c.BS_SC0_PACKET_SIZE,
                          c.SC_PACKET_SIZE_FIRE_TEST | packet_size)
    client.stream_socket.recvfrom(4096)

    features = server.camera.feature_set.by_name
    client.write_register(features["AcquisitionStart"].address, 1)
    block_id, _leader, data = client.receive_frame(packet_size)
    assert block_id == 1
    assert len(data) == server.camera.geometry["payload"]


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


def test_pixel_format_is_writable_when_there_is_a_choice(client, server):
    """
    Advertising several formats on a read-only feature draws a populated
    combo box in a viewer that refuses every selection. Either the choice is
    real or it should not be offered.
    """
    camera = server.camera
    pixel_format = camera.feature_set.by_name["PixelFormat"]
    payload = camera.feature_set.by_name["PayloadSize"]
    client.take_control()

    assert pixel_format.access == "RW"
    assert client.read_register(payload.address) == WIDTH * HEIGHT

    client.write_register(pixel_format.address, c.PIXEL_FORMAT_MONO16)
    assert camera.settings["PixelFormat"] == "Mono16"
    # Two bytes a pixel now, and the client must see that without asking.
    assert client.read_register(payload.address) == WIDTH * HEIGHT * 2


def test_a_bogus_pixel_format_is_refused_and_leaves_the_device_working(
        client, server):
    """
    An ordinal outside the enumeration used to be stored as-is, and every
    later call that looked the format up by name then raised a KeyError --
    including latch_geometry(), so the camera could no longer start at all.
    The write has to be refused at the point it arrives.

    The error code is checked rather than just the failure, because the
    failure this replaces was a *timeout*: the exception escaped the handler,
    nothing was sent, and the client gave up after its retries. Both raise
    FakeClientError, and only the status byte tells them apart.
    """
    camera = server.camera
    pixel_format = camera.feature_set.by_name["PixelFormat"]
    client.take_control()

    with pytest.raises(FakeClientError) as excinfo:
        client.write_register(pixel_format.address, 12345)
    assert excinfo.value.error == c.ERROR_INVALID_PARAMETER

    assert camera.settings["PixelFormat"] == "Mono8"
    assert client.read_register(pixel_format.address) == c.PIXEL_FORMAT_MONO8

    # The half that actually regressed: the device still works afterwards.
    packet_size = client.open_stream()
    client.write_register(
        camera.feature_set.by_name["AcquisitionStart"].address, 1)
    block_id, leader, data = client.receive_frame(packet_size)
    assert leader["pixel_format"] == c.PIXEL_FORMAT_MONO8
    assert len(data) == WIDTH * HEIGHT
    client.write_register(
        camera.feature_set.by_name["AcquisitionStop"].address, 1)


def test_a_device_side_bug_is_answered_rather_than_ignored(client, server,
                                                           monkeypatch):
    """
    A handler that raises something the dispatcher does not expect must still
    produce an ack. Sending nothing makes the client burn its retry budget
    and report a timeout, which reads as a network fault and sends whoever is
    debugging it to the wrong side of the link.
    """
    def boom():
        raise RuntimeError("simulated device bug")

    monkeypatch.setattr(server.bridge, "refresh_geometry", boom)
    client.take_control()

    with pytest.raises(FakeClientError) as excinfo:
        client.write_register(
            server.camera.feature_set.by_name["PixelFormat"].address,
            c.PIXEL_FORMAT_MONO16)
    assert excinfo.value.error == c.ERROR_GENERIC


def test_switching_pixel_format_changes_the_frames_that_follow(client, server):
    camera = server.camera
    client.take_control()
    client.write_register(
        camera.feature_set.by_name["PixelFormat"].address,
        c.PIXEL_FORMAT_MONO16)

    packet_size = client.open_stream()
    client.write_register(
        camera.feature_set.by_name["AcquisitionStart"].address, 1)
    block_id, leader, data = client.receive_frame(packet_size)

    assert leader["pixel_format"] == c.PIXEL_FORMAT_MONO16
    assert len(data) == WIDTH * HEIGHT * 2
    client.write_register(
        camera.feature_set.by_name["AcquisitionStop"].address, 1)


def test_pixel_format_is_read_only_with_nothing_to_choose():
    """
    A camera that can deliver one format has a genuinely read-only feature,
    and saying so is better than accepting a write that changes nothing.
    """
    class OneFormat(EmulatedCamera):
        def next_frame(self):
            return b""

    camera = OneFormat(width=8, height=8, pixel_format="Mono16",
                       pixel_formats=["Mono16"])
    assert camera.feature_set.by_name["PixelFormat"].access == "RO"


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


def test_a_string_feature_round_trips_through_write_memory():
    """
    A string register is the one feature type a client can never reach with
    WRITE_REGISTER -- four bytes will not carry it -- so it exercises the
    memory path end to end, which is exactly why the bridge keys on
    (address, length) rather than on the GVCP command.
    """
    class NamedCamera(PatternCamera):
        extra_features = PatternCamera.extra_features + (
            StringFeature("DeviceUserID", "User programmable id",
                          "DeviceControl", "RW", default="bench-left",
                          length=16),
        )

    camera = NamedCamera(width=8, height=8, pixel_format="Mono8")
    srv = GigECameraServer(camera, ip="127.0.0.1", netmask="255.0.0.0",
                           model_name="PatternCam", serial_number="TEST-1",
                           gvcp_port=PORT + 4, bind_address="127.0.0.1")
    srv.start()
    try:
        with FakeClient(("127.0.0.1", PORT + 4)) as named:
            named.take_control()
            feature = camera.feature_set.by_name["DeviceUserID"]

            raw = named.read_memory(feature.address, feature.size)
            assert raw.split(b"\x00", 1)[0] == b"bench-left"

            named.write_memory(feature.address, feature.encode("bench-right"))
            assert camera.settings["DeviceUserID"] == "bench-right"
            # The hook has to fire for a string exactly as it does for an
            # integer; a camera cannot act on a name it is never told about.
            assert {"DeviceUserID": "bench-right"} in camera.applied

            raw = named.read_memory(feature.address, feature.size)
            assert raw.split(b"\x00", 1)[0] == b"bench-right"
    finally:
        srv.stop()


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


def test_a_busy_client_keeps_control_without_ever_heartbeating(server):
    """
    The failure this prevents is a client being dropped mid-startup.

    A client serialises control access behind one mutex and heartbeats on a
    one second period, so a burst of feature reads -- enumerating everything
    to build a property tree -- starves its own heartbeat. Counting only the
    privilege read released control after three missed turns, on a client
    that was plainly alive and talking to us the whole time.
    """
    features = server.camera.feature_set.by_name
    with FakeClient(("127.0.0.1", PORT)) as busy:
        busy.take_control()
        # Well past the fixture's 1000 ms, and never once reading CCP.
        deadline = time.monotonic() + 1.6
        while time.monotonic() < deadline:
            busy.read_register(features["GainRaw"].address)
            time.sleep(0.05)
        assert server.control.has_control(), (
            "a client issuing register reads was treated as dead")
        # And it really still owns it, rather than merely not being expired.
        busy.write_register(features["GainRaw"].address, 9)
        assert server.camera.settings["GainRaw"] == 9


def test_silence_still_releases_control(server):
    """
    The other half: activity has to mean actual packets, or a crashed client
    would hold the camera forever. Same duration as the busy client above.
    """
    with FakeClient(("127.0.0.1", PORT)) as quiet:
        quiet.take_control()
        assert server.control.has_control()
        time.sleep(1.6)
        assert not server.control.has_control()


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


def test_stopping_during_a_long_exposure_does_not_kill_the_stream_thread():
    """
    stop() will not wait out an exposure -- a camera can be asked for ten
    seconds and blocking the caller that long is worse. So the thread comes
    back from next_frame() to a socket that has been closed and dropped, and
    it has to notice rather than dereference it.

    This is built without the shared fixtures because it needs a next_frame()
    slower than the join, and a heartbeat long enough that the controller is
    not dropped mid-exposure -- expiry clears `acquiring`, which skips the
    send and hides the bug.
    """
    import threading

    exposure = stream_module.SHUTDOWN_JOIN_TIMEOUT + 2.0

    class SlowCamera(EmulatedCamera):
        def next_frame(self):
            time.sleep(exposure)
            return b"\x40" * self.geometry["payload"]

        def set_camera_settings(self, changed):
            pass

        def get_camera_settings(self):
            return {}

    camera = SlowCamera(width=32, height=24, pixel_format="Mono8")
    srv = GigECameraServer(camera, ip="127.0.0.1", netmask="255.0.0.0",
                           model_name="Slow", serial_number="SLOW-1",
                           gvcp_port=PORT + 1, bind_address="127.0.0.1",
                           heartbeat_timeout_ms=60000)
    srv.start()

    failures = []
    previous = threading.excepthook
    threading.excepthook = failures.append
    try:
        with FakeClient(("127.0.0.1", PORT + 1)) as slow_client:
            slow_client.take_control()
            slow_client.open_stream(packet_size=1400)
            slow_client.write_register(
                camera.feature_set.by_name["AcquisitionStart"].address, 1)
            time.sleep(0.5)                       # well inside the exposure
            srv.stop()
        # Let the orphaned thread finish its exposure and return.
        time.sleep(exposure + 0.5)
    finally:
        threading.excepthook = previous

    assert not [f.exc_type.__name__ for f in failures], (
        "stream thread raised on shutdown: %s"
        % [f.exc_value for f in failures])
    assert not any(t.name == "gvsp-stream" for t in threading.enumerate())
