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


def _claims_control(command):
    """
    True if this command is a write to the control channel privilege
    register, the one write an uncontrolled device must accept.
    """
    if command.command == c.CMD_WRITE_REGISTER:
        try:
            pairs = gvcp.decode_write_register(command.payload)
        except gvcp.MalformedPacket:
            return False
        return any(address == c.BS_CONTROL_CHANNEL_PRIVILEGE
                   for address, _ in pairs)
    return False


class ControlChannel(object):

    def __init__(self, memory, lock, port=c.GVCP_PORT, bind_address="",
                 on_control_change=None, bridge=None, interface=None,
                 on_test_packet=None, on_packet_resend=None):
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
        # Called with (frame_id, first_packet_id, last_packet_id). Wired to
        # the stream channel, which holds both the frame and the socket.
        self.on_packet_resend = on_packet_resend

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
            # Holding control is what earns the right to write. The one
            # exception is the privilege register itself, which is how
            # control is claimed in the first place -- refusing that would
            # leave no way in.
            #
            # This used to grant write access to anyone whenever the device
            # was idle, which is looser than the standard and looser than it
            # sounds: the address is a UDP source, so a few packets with a
            # forged one could point the stream anywhere and start it, with
            # no reply ever needed by whoever sent them.
            write_access = (self.controller == address
                            or (self.controller is None
                                and _claims_control(command)))
            # Any command from the controller counts as a heartbeat, not just
            # a read of the privilege register.
            #
            # The heartbeat exists to notice a client that has *died*, and one
            # sending register reads plainly has not. Counting only the
            # privilege read drops a client that is merely busy: a client
            # serialises control access behind one mutex and heartbeats on a
            # one second period, so while it enumerates features to build its
            # property tree the heartbeat is queued behind that burst. Three
            # missed turns and control is released mid-initialisation, which
            # is what micro-manager hit on startup. Crash detection is
            # unaffected, since a crashed client sends nothing at all.
            if self.controller == address:
                self.controller_time = time.monotonic()

        try:
            reply = self._dispatch(command, address, write_access)
        except MemoryError_ as e:
            self.n_errors += 1
            # Say so. A refused write is otherwise invisible from the device:
            # the reply is an error ack the client may not put in front of
            # anyone -- arv-viewer does not -- and nothing reaches the log,
            # because the refusal happens here, before the feature layer that
            # does the logging. What that leaves is a control whose changes
            # quietly do nothing, indistinguishable from a write that never
            # arrived, which is the wrong question to be left holding.
            log.warning("refused command 0x%04x from %s: %s",
                        command.command, address, e)
            return gvcp.encode_error(command.command + 1, command.packet_id,
                                     e.gvcp_error)
        except gvcp.MalformedPacket:
            self.n_errors += 1
            return gvcp.encode_error(command.command + 1, command.packet_id,
                                     c.ERROR_INVALID_PARAMETER)
        except Exception:
            # Anything arriving here is a bug in the device rather than a
            # request it is entitled to refuse -- but the client still has to
            # be answered. Letting it fall through to the socket loop's
            # handler sends nothing at all, and silence is the one failure a
            # client cannot attribute: it burns its full retry budget and
            # then reports a timeout, which points at the network rather than
            # at the device that actually failed. The traceback is logged
            # here, so answering hides nothing.
            self.n_errors += 1
            log.exception("dispatching command 0x%04x failed", command.command)
            return gvcp.encode_error(command.command + 1, command.packet_id,
                                     c.ERROR_GENERIC)

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
                # The privilege read is the client's nominal heartbeat, but
                # handle() already refreshed the clock for any command from
                # the controller, so there is nothing extra to do for it.
                values = [self.memory.read_register(a) for a in addresses]
            return gvcp.encode_read_register_ack(command.packet_id, values)

        if cmd == c.CMD_WRITE_REGISTER:
            pairs = gvcp.decode_write_register(command.payload)
            if not write_access:
                raise MemoryError_(self._denied([a for a, _ in pairs]),
                                   c.ERROR_ACCESS_DENIED)
            with self.lock:
                for addr, value in pairs:
                    self.memory.write_register(addr, value)
            if self.bridge is not None:
                for addr, _ in pairs:
                    self.bridge.after_write(addr)
            for addr, value in pairs:
                if (addr == c.BS_SC0_PACKET_SIZE
                        and value & c.SC_PACKET_SIZE_FIRE_TEST):
                    self._fire_test_packet(value)
            return gvcp.encode_write_register_ack(command.packet_id, len(pairs))

        if cmd == c.CMD_READ_MEMORY:
            addr, count = gvcp.decode_read_memory(command.payload)
            if count > c.GVCP_READMEM_MAX:
                raise MemoryError_("read of %d bytes is larger than this "
                                   "device answers in one ack (%d)"
                                   % (count, c.GVCP_READMEM_MAX),
                                   c.ERROR_INVALID_PARAMETER)
            if self.bridge is not None:
                self.bridge.before_read(addr, count)
            with self.lock:
                data = self.memory.read(addr, count)
            return gvcp.encode_read_memory_ack(command.packet_id, addr, data)

        if cmd == c.CMD_WRITE_MEMORY:
            addr, payload = gvcp.decode_write_memory(command.payload)
            if not write_access:
                raise MemoryError_(self._denied([addr]),
                                   c.ERROR_ACCESS_DENIED)
            with self.lock:
                self.memory.write(addr, payload)
            if self.bridge is not None:
                self.bridge.after_write(addr)
            return gvcp.encode_write_memory_ack(command.packet_id, addr)

        if cmd == c.CMD_PACKET_RESEND:
            # No ack, ever. Aravis sends this command without the
            # ack-required flag (arvgvcp.c), so it is waiting for the packets
            # themselves on the stream channel, not for a reply here --
            # answering on the control channel would be one more datagram it
            # ignores while the frame it needs times out.
            frame_id, first_id, last_id = gvcp.decode_packet_resend(
                command.payload)
            if self.on_packet_resend is not None:
                self.on_packet_resend(frame_id, first_id, last_id)
            return b""

        raise MemoryError_("unimplemented command 0x%04x" % cmd,
                           c.ERROR_NOT_IMPLEMENTED)

    def _feature_name(self, address):
        """The feature at an address, for a message a person has to act on."""
        if self.bridge is not None:
            feature = self.bridge.features.lookup_address(address)
            if feature is not None:
                return feature.name
        return "0x%x" % address

    def _denied(self, addresses):
        """
        Why a write was refused, naming what was being written and who is
        holding the channel.

        The holder is the useful half. "Not the controller" says a client is
        being refused; it does not say that some other client -- often one
        the user has forgotten is still open -- is the reason, which is the
        thing that has to change for the write to succeed.
        """
        with self.lock:
            controller = self.controller
        return ("write to %s refused: control is held by %s"
                % (", ".join(self._feature_name(a) for a in addresses),
                   controller if controller is not None else "nobody"))

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
