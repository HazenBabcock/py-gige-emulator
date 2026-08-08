#
# The GVCP control channel: one thread, one UDP socket on port 3956.
#
# Binding to ('', 3956) covers unicast, 255.255.255.255 and the subnet
# broadcast in a single socket. Aravis opens three sockets for this because
# GLib binds to specific addresses; the stdlib has no such constraint.
#
# Every reply goes back to the (address, port) the datagram came from. The
# client's control socket is on an ephemeral port, so replying to a fixed
# port would work for exactly nobody.
#

import logging
import socket
import threading
import time

from . import bootstrap
from . import constants as c
from . import gvcp
from . import netif
from .memory import MemoryError_

log = logging.getLogger(__name__)


class ControlChannel(object):

    def __init__(self, memory, lock, port=c.GVCP_PORT, bind_address="",
                 on_control_change=None, bridge=None, interface=None,
                 on_test_packet=None):
        self.memory = memory
        self.lock = lock
        self.port = port
        self.bind_address = bind_address
        self.interface = interface
        self.on_control_change = on_control_change
        # Called with (packet_size, do_not_fragment) when a client asks the
        # device to fire a test packet. Wired to the stream channel, which is
        # the only thing here holding a GVSP socket.
        self.on_test_packet = on_test_packet

        # The bridge runs user code, so it is always called with the lock
        # released -- a slow camera must not be able to stall the stream
        # thread's per frame snapshot.
        self.bridge = bridge

        self.socket = None
        self.thread = None
        self.running = False

        # The controller is identified by its (ip, port) pair, which is the
        # client's ephemeral control socket.
        self.controller = None
        self.controller_time = 0.0

        self.n_commands = 0
        self.n_errors = 0

    # --- lifecycle -------------------------------------------------------

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # The bind below is to INADDR_ANY, because a socket bound to a
        # specific address does not receive broadcast and broadcast is how
        # discovery arrives. Restrict to the chosen interface at the device
        # level instead, or a machine with two interfaces answers discovery
        # on both while advertising only one of its addresses -- which works
        # by luck when they share a subnet and fails confusingly otherwise.
        if self.interface is not None:
            if not netif.bind_to_device(self.socket, self.interface):
                log.warning("cannot restrict the control socket to %s; it "
                            "will answer discovery on every interface",
                            self.interface)

        self.socket.bind((self.bind_address, self.port))
        self.socket.settimeout(0.2)
        self.running = True
        self.thread = threading.Thread(target=self._run, name="gvcp-control",
                                       daemon=True)
        self.thread.start()
        return self.socket.getsockname()[1]

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            self.thread = None
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def _run(self):
        while self.running:
            try:
                data, address = self.socket.recvfrom(2048)
            except socket.timeout:
                self._expire_controller()
                continue
            except OSError:
                break
            try:
                reply = self.handle(data, address)
            except Exception:
                log.exception("control packet handler failed")
                continue
            if reply:
                try:
                    self.socket.sendto(reply, address)
                except OSError:
                    log.exception("failed to send control reply")

    # --- controller / heartbeat -----------------------------------------

    def _heartbeat_timeout(self):
        return self.memory.peek_register(c.BS_HEARTBEAT_TIMEOUT) / 1000.0

    def _expire_controller(self):
        """
        Drop a controller that has gone quiet.

        Aravis's device only evaluates this when a packet arrives, so a
        client that dies silently leaves the camera controlled -- and
        streaming -- forever. Calling this from the receive timeout as well
        costs nothing and makes a crashed viewer recoverable.
        """
        with self.lock:
            if self.controller is None:
                return False
            if (time.monotonic() - self.controller_time) <= self._heartbeat_timeout():
                return False
            log.info("heartbeat timeout, releasing controller %s", (self.controller,))
            self.controller = None
            self.memory.poke_register(c.BS_CONTROL_CHANNEL_PRIVILEGE, 0)
        if self.on_control_change is not None:
            self.on_control_change(False)
        return True

    def has_control(self):
        with self.lock:
            return self.controller is not None

    # --- dispatch --------------------------------------------------------

    def handle(self, data, address):
        """
        Returns the reply datagram, or b"" for commands that get no answer.

        Split out from the socket loop so the whole control channel can be
        exercised from a test without any network at all.
        """
        try:
            command = gvcp.parse_command(data)
        except gvcp.MalformedPacket:
            return b""
        if command is None:
            return b""

        self.n_commands += 1
        self._expire_controller()

        with self.lock:
            write_access = (self.controller is None
                            or self.controller == address)

        try:
            reply = self._dispatch(command, address, write_access)
        except MemoryError_ as e:
            self.n_errors += 1
            return gvcp.encode_error(command.command + 1, command.packet_id,
                                     e.gvcp_error)
        except gvcp.MalformedPacket:
            self.n_errors += 1
            return gvcp.encode_error(command.command + 1, command.packet_id,
                                     c.ERROR_INVALID_PARAMETER)

        self._update_controller(address)
        return reply

    def _dispatch(self, command, address, write_access):
        cmd = command.command

        if cmd == c.CMD_DISCOVERY:
            with self.lock:
                page = bootstrap.discovery_page(self.memory)
            return gvcp.encode_discovery_ack(command.packet_id, page)

        if cmd == c.CMD_READ_REGISTER:
            addresses = gvcp.decode_read_register(command.payload)
            if self.bridge is not None:
                for a in addresses:
                    self.bridge.before_read(a, 4)
            with self.lock:
                # A read of the privilege register is the client's heartbeat.
                if c.BS_CONTROL_CHANNEL_PRIVILEGE in addresses:
                    if self.controller == address:
                        self.controller_time = time.monotonic()
                values = [self.memory.read_register(a) for a in addresses]
            return gvcp.encode_read_register_ack(command.packet_id, values)

        if cmd == c.CMD_WRITE_REGISTER:
            if not write_access:
                raise MemoryError_("not the controller", c.ERROR_ACCESS_DENIED)
            pairs = gvcp.decode_write_register(command.payload)
            with self.lock:
                for addr, value in pairs:
                    self.memory.write_register(addr, value)
            if self.bridge is not None:
                for addr, value in pairs:
                    self.bridge.after_write(addr)
            for addr, value in pairs:
                if (addr == c.BS_SC0_PACKET_SIZE
                        and value & c.SC_PACKET_SIZE_FIRE_TEST):
                    self._fire_test_packet(value)
            return gvcp.encode_write_register_ack(command.packet_id, len(pairs))

        if cmd == c.CMD_READ_MEMORY:
            addr, count = gvcp.decode_read_memory(command.payload)
            if count > c.GVCP_DATA_SIZE_MAX:
                raise MemoryError_("read too large", c.ERROR_INVALID_PARAMETER)
            if self.bridge is not None:
                self.bridge.before_read(addr, count)
            with self.lock:
                data = self.memory.read(addr, count)
            return gvcp.encode_read_memory_ack(command.packet_id, addr, data)

        if cmd == c.CMD_WRITE_MEMORY:
            if not write_access:
                raise MemoryError_("not the controller", c.ERROR_ACCESS_DENIED)
            addr, payload = gvcp.decode_write_memory(command.payload)
            with self.lock:
                self.memory.write(addr, payload)
            if self.bridge is not None:
                self.bridge.after_write(addr)
            return gvcp.encode_write_memory_ack(command.packet_id, addr)

        if cmd == c.CMD_PACKET_RESEND:
            # Never requested, because the resend capability bit is clear.
            # Arriving here means a client ignored that, and the honest
            # answer is silence rather than a bogus ack.
            return b""

        raise MemoryError_("unimplemented command 0x%04x" % cmd,
                           c.ERROR_NOT_IMPLEMENTED)

    def _fire_test_packet(self, value):
        """
        Honour a packet size probe: send one packet of the requested size.

        The fire bit is a trigger rather than state, so it is cleared here --
        a client that reads the register back must not see a test it already
        asked for still pending. The size and the don't-fragment bit stay,
        since those are the settings the stream will run with.
        """
        with self.lock:
            self.memory.poke_register(c.BS_SC0_PACKET_SIZE,
                                      value & ~c.SC_PACKET_SIZE_FIRE_TEST)
        if self.on_test_packet is None:
            return
        self.on_test_packet(value & c.SC_PACKET_SIZE_MASK,
                            bool(value & c.SC_PACKET_SIZE_DO_NOT_FRAGMENT))

    def _update_controller(self, address):
        """
        Adopt or release the controller based on the privilege register,
        which the client sets by writing to it.
        """
        changed = None
        with self.lock:
            privilege = self.memory.peek_register(c.BS_CONTROL_CHANNEL_PRIVILEGE)
            if self.controller is None and privilege != 0:
                log.info("control taken by %s", (address,))
                self.controller = address
                self.controller_time = time.monotonic()
                changed = True
            elif self.controller is not None and privilege == 0:
                log.info("control released by %s", (address,))
                self.controller = None
                changed = False
        if changed is not None and self.on_control_change is not None:
            self.on_control_change(changed)
