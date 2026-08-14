#
# Wires a camera, its generated XML, the register memory and the two
# protocol threads into one object.
#

import logging
import threading
import time

from . import bootstrap, genicam_xml, netif
from . import constants as c
from .bridge import FeatureBridge
from .control import ControlChannel
from .memory import DeviceMemory
from .stream import StreamChannel

log = logging.getLogger(__name__)


class GigECameraServer(object):

    def __init__(self, camera, interface=None, ip=None, netmask=None, mac=None,
                 model_name=None, vendor_name="py-gige-emulator",
                 serial_number="PY-0001", user_defined_name="",
                 gvcp_port=c.GVCP_PORT, bind_address="",
                 packet_size=c.DEFAULT_PACKET_SIZE,
                 heartbeat_timeout_ms=3000, validate=True,
                 compress_xml=True, packet_resend=True, resend_guard=None,
                 retain_frames=None, link_utilisation=None):

        if interface is not None:
            # The address has to be the interface's or the client cannot
            # reach the device, but the MAC is only ever *reported* -- and
            # some clients admit a device only if its first three octets are
            # their own vendor's OUI, so a caller who supplies one keeps it.
            ip, netmask, found_mac = netif.interface_info(interface)
            mac = mac or found_mac
        if ip is None:
            raise ValueError("give either an interface name or an ip address")

        self.camera = camera
        self.interface = interface
        self.ip = ip
        self.lock = threading.RLock()
        self.memory = DeviceMemory()

        model_name = model_name or type(camera).__name__

        self.xml = genicam_xml.build_xml(camera.feature_set, model_name,
                                         vendor_name)
        if validate:
            problems = genicam_xml.validate_xml(self.xml, camera.feature_set)
            if problems:
                raise ValueError("generated GenICam XML is not usable:\n  "
                                 + "\n  ".join(problems))

        # The URL's filename is what tells the client whether to inflate, so
        # the name and the blob have to be decided together.
        xml_filename = "%s.xml" % model_name.lower()
        self.xml_blob = self.xml
        if compress_xml:
            packed = genicam_xml.zip_xml(self.xml, xml_filename)
            # Only if it actually helps. A blob that grew would also be
            # unreadable rather than merely pointless: the client decides an
            # entry is compressed by comparing the two sizes in the central
            # directory, so deflate output that did not shrink gets copied
            # out raw. An archive is its entry plus about 120 bytes of
            # headers, so this one test covers both.
            if len(packed) < len(self.xml):
                self.xml_blob = packed
                xml_filename += ".zip"
            else:
                log.debug("genicam xml does not compress usefully (%d -> %d "
                          "bytes); serving it as is", len(self.xml),
                          len(packed))

        xml_size = self.memory.set_genicam_xml(self.xml_blob)

        self.info = bootstrap.DeviceInfo(
            ip=ip, netmask=netmask or "255.255.255.0",
            mac=mac or b"\x00" * 6,
            manufacturer_name=vendor_name, model_name=model_name,
            serial_number=serial_number, user_defined_name=user_defined_name,
            xml_filename=xml_filename,
            heartbeat_timeout_ms=heartbeat_timeout_ms,
            packet_size=packet_size, packet_resend=packet_resend)
        bootstrap.init_bootstrap(self.memory, self.info, xml_size)

        self.bridge = FeatureBridge(camera, self.memory, self.lock)
        self.bridge.sync_all_to_memory()
        self.bridge.refresh_geometry()

        # Only restrict to a device when we were given one by name. Starting
        # from a bare ip= -- which is how the loopback tests run -- leaves
        # both sockets unrestricted, as before.
        self.control = ControlChannel(
            self.memory, self.lock, port=gvcp_port, bind_address=bind_address,
            bridge=self.bridge, on_control_change=self._on_control_change,
            interface=interface, on_test_packet=self._on_test_packet,
            on_packet_resend=self._on_packet_resend)
        self.stream = StreamChannel(camera, self.memory, self.lock, ip,
                                    control=self.control, interface=interface,
                                    resend_guard=resend_guard,
                                    retain_frames=retain_frames,
                                    link_utilisation=link_utilisation)

    def _on_test_packet(self, packet_size, do_not_fragment):
        # Bound late rather than passed as self.stream.send_test_packet,
        # because the control channel is built before the stream channel
        # exists.
        self.stream.send_test_packet(packet_size, do_not_fragment)

    def _on_packet_resend(self, frame_id, first_id, last_id):
        # Bound late, like _on_test_packet: the control channel is built
        # before the stream channel that owns the frame and the socket.
        self.stream.resend(frame_id, first_id, last_id)

    def _on_control_change(self, has_control):
        if not has_control:
            # Losing the controller must stop acquisition, or a client that
            # crashed leaves the camera streaming into a socket nobody owns.
            with self.lock:
                self.camera.acquiring = False

    # --- lifecycle -------------------------------------------------------

    def start(self):
        port = self.control.start()
        self.stream.start()
        # The user-defined name is reported separately because it is not part
        # of the device id, which is always vendor-model-serial. Clients list
        # the id, so a name that is working looks like a name that was
        # ignored unless something says otherwise.
        log.info("%s listening on %s:%d as %s-%s-%s%s",
                 type(self.camera).__name__, self.ip, port,
                 self.info.manufacturer_name, self.info.model_name,
                 self.info.serial_number,
                 (", user-defined name %r (select with this, not the id)"
                  % self.info.user_defined_name)
                 if self.info.user_defined_name else "")
        return self

    def stop(self):
        self.stream.stop()
        self.control.stop()

    def serve_forever(self):
        self.start()
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False

    # --- introspection ---------------------------------------------------

    @property
    def stats(self):
        return {
            "commands": self.control.n_commands,
            "command_errors": self.control.n_errors,
            "frames": self.stream.n_frames,
            "packets": self.stream.n_packets,
            "test_packets": self.stream.n_test_packets,
            "resend_requests": self.stream.n_resend_requests,
            "resent_packets": self.stream.n_resent_packets,
            "resend_unavailable": self.stream.n_resend_unavailable,
            "resend_refused": self.stream.n_resend_refused,
            "send_errors": self.stream.n_send_errors,
            "has_control": self.control.has_control(),
            "acquiring": self.camera.acquiring,
        }
