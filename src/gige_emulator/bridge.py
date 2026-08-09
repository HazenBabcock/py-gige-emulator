#
# Connects client register access to the user's settings hooks.
#
# Dispatch is on (address, length) rather than on the GVCP command, and that
# is deliberate. The client only uses READ_REGISTER / WRITE_REGISTER for
# 4 byte accesses on a schema older than 1.1.0; an 8 byte float or a string
# register always arrives as READ_MEMORY. Keying off the command would give
# hooks that fire for integers and silently never fire for floats.
#
# The hooks run with the device lock released. They are user code calling
# real hardware, and holding the lock across them would let a slow camera
# stall the stream thread's per-frame snapshot.
#

import logging

from . import constants as c
from .features import CommandFeature
from .memory import MemoryError_

log = logging.getLogger(__name__)


class FeatureBridge(object):

    def __init__(self, camera, memory, lock):
        self.camera = camera
        self.memory = memory
        self.lock = lock
        self.features = camera.feature_set

    # --- reads -----------------------------------------------------------

    def before_read(self, address, length):
        """
        Refresh whatever the client is about to read. Called with no lock.
        """
        feature = self.features.lookup_address(address)
        if feature is None:
            return

        try:
            current = self.camera.get_camera_settings()
        except Exception:
            log.exception("get_camera_settings() raised; serving cached values")
            return
        if not current:
            return

        with self.lock:
            for name, value in current.items():
                target = self.features.by_name.get(name)
                if target is None or isinstance(target, CommandFeature):
                    continue
                try:
                    self.camera.settings[name] = target.validate(value)
                    self.memory.poke_bytes(target.address,
                                           target.encode(self.camera.settings[name]))
                except Exception:
                    log.exception("cannot store %r from get_camera_settings()",
                                  name)

    # --- writes ----------------------------------------------------------

    def after_write(self, address):
        """
        Called with no lock, after the bytes have landed in memory. Raising
        MemoryError_ rolls the register back and answers the client with an
        error rather than a success that did nothing.
        """
        feature = self.features.lookup_address(address)
        if feature is None:
            return

        if isinstance(feature, CommandFeature):
            self._run_command(feature)
            return

        if feature.access == "RO":
            self._restore(feature)
            raise MemoryError_("%s is read only" % feature.name,
                               c.ERROR_WRITE_PROTECT)

        # The client sized its buffers from PayloadSize when acquisition
        # started, so moving the geometry now would leave it dropping every
        # packet past the old count without reporting anything. Refusing is
        # the only answer it can act on.
        if feature.affects_payload and self.camera.acquiring:
            self._restore(feature)
            raise MemoryError_(
                "%s cannot be changed while acquiring" % feature.name,
                c.ERROR_BUSY)

        with self.lock:
            raw = self.memory._read_raw(feature.address, feature.size)
        value = feature.decode(raw)

        try:
            value = feature.validate(value)
        except Exception as e:
            self._restore(feature)
            raise MemoryError_(str(e), c.ERROR_INVALID_PARAMETER)

        previous = self.camera.settings.get(feature.name)
        if value == previous:
            return

        self.camera.settings[feature.name] = value
        try:
            self.camera.set_camera_settings({feature.name: value})
        except Exception as e:
            self.camera.settings[feature.name] = previous
            self._restore(feature)
            raise MemoryError_("%s rejected by the camera: %s"
                               % (feature.name, e), c.ERROR_INVALID_PARAMETER)

        # A geometry change may have moved Width and Height too -- binning is
        # the usual case -- so re-publish everything derived from it rather
        # than just the payload size.
        if feature.affects_payload:
            self.refresh_geometry()

    def _restore(self, feature):
        """Put the stored value back into the register."""
        with self.lock:
            value = self.camera.settings.get(feature.name)
            if value is None:
                return
            try:
                self.memory.poke_bytes(feature.address, feature.encode(value))
            except Exception:
                log.exception("cannot restore %r", feature.name)

    # --- commands --------------------------------------------------------

    def _run_command(self, feature):
        with self.lock:
            raw = self.memory._read_raw(feature.address, feature.size)
            fired = feature.decode(raw) == feature.command_value
            # GenICam commands are self clearing, which is what makes them
            # executable more than once.
            self.memory.poke_register(feature.address, 0)
        if not fired:
            return

        if feature.name == "AcquisitionStart":
            with self.lock:
                self.camera.latch_geometry()
                self.camera.acquiring = True
            log.info("acquisition started, %r", self.camera.geometry)
        elif feature.name == "AcquisitionStop":
            with self.lock:
                self.camera.acquiring = False
            log.info("acquisition stopped")

    # --- device side bookkeeping ----------------------------------------

    def refresh_geometry(self):
        """
        Re-publish everything derived from the geometry.

        A camera is free to change Width and Height from inside
        set_camera_settings -- selecting a binned sensor mode does exactly
        that -- so the registers have to be brought back in step with
        self.settings afterwards, not just PayloadSize.
        """
        with self.lock:
            self.camera.settings["PayloadSize"] = self.camera.payload_size()
            for name in ("Width", "Height", "PixelFormat", "PayloadSize"):
                feature = self.features.by_name.get(name)
                if feature is None:
                    continue
                self.memory.poke_bytes(
                    feature.address,
                    feature.encode(self.camera.settings[name]))

    def sync_all_to_memory(self):
        """Write every stored setting into its register. Called at startup."""
        with self.lock:
            for feature in self.features.features:
                if isinstance(feature, CommandFeature):
                    self.memory.poke_register(feature.address, 0)
                    continue
                value = self.camera.settings.get(feature.name, feature.default)
                self.memory.poke_bytes(feature.address, feature.encode(value))
