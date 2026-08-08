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
                 heartbeat_timeout_ms=3000, validate=True):

        if interface is not None:
            ip, netmask, mac = netif.interface_info(interface)
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

        xml_size = self.memory.set_genicam_xml(self.xml)

        self.info = bootstrap.DeviceInfo(
            ip=ip, netmask=netmask or "255.255.255.0",
            mac=mac or b"\x00" * 6,
            manufacturer_name=vendor_name, model_name=model_name,
            serial_number=serial_number, user_defined_name=user_defined_name,
            xml_filename="%s.xml" % model_name.lower(),
            heartbeat_timeout_ms=heartbeat_timeout_ms,
            packet_size=packet_size)
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
            interface=interface)
        self.stream = StreamChannel(camera, self.memory, self.lock, ip,
                                    control=self.control, interface=interface)

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
        log.info("%s listening on %s:%d as %s-%s-%s",
                 type(self.camera).__name__, self.ip, port,
                 self.info.manufacturer_name, self.info.model_name,
                 self.info.serial_number)
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
            "send_errors": self.stream.n_send_errors,
            "has_control": self.control.has_control(),
            "acquiring": self.camera.acquiring,
        }
