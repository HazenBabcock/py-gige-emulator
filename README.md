### py-gige-emulator ###

A pure Python GigE Vision camera emulator. It speaks GVCP and GVSP directly over
UDP, so any GigE Vision client on the network — `arv-viewer`, micro-manager,
anything built on [Aravis](https://github.com/AravisProject/aravis) — sees a real
camera. No compiler, no libaravis, no C.

Wire in a physical camera by subclassing `EmulatedCamera` and implementing three
hooks.

#### Usage ####

```python
from gige_emulator import EmulatedCamera, GigECameraServer, FloatFeature, IntFeature

class MyCamera(EmulatedCamera):

    # Width, Height, PixelFormat, PayloadSize, SensorWidth, SensorHeight,
    # AcquisitionMode, AcquisitionStart/Stop and AcquisitionFrameRate are
    # provided. Declare anything else you want the client to see.
    extra_features = (
        FloatFeature("ExposureTime", default=10000.0, min=1.0, max=1e6, unit="us"),
        IntFeature("GainRaw", default=1, min=1, max=22),
    )

    def next_frame(self):
        """Runs on the stream thread. Blocking is fine. Return bytes."""
        return my_hardware.grab()

    def set_camera_settings(self, changed):
        """Runs inline when the client writes a feature. `changed` holds only
        what moved; self.settings always holds the full state."""
        if "ExposureTime" in changed:
            my_hardware.set_exposure(changed["ExposureTime"])

    def get_camera_settings(self):
        """Runs inline when the client reads a feature. A partial dict is fine."""
        return {"ExposureTime": my_hardware.get_exposure()}

camera = MyCamera(width=640, height=480, pixel_format="Mono8")
GigECameraServer(camera, interface="eth0",
                 model_name="MyCam", serial_number="0001").serve_forever()
```

Try it with no hardware at all:

```
$ python examples/noise_camera.py --interface eth0
```

then point any GigE Vision client at it.

#### Design notes ####

* **The GenICam XML is generated from the feature declarations**, not hand
  written. The register map and the XML come from the same source, so they
  cannot drift — an `<Address>` that disagrees with what the device stores
  fails in a way that looks like a client bug.
* **Two threads.** Control and streaming are separate, unlike the Aravis
  reference implementation, because `next_frame()` is a real camera grab. A one
  second exposure on a single thread would block GVCP past the client's command
  timeout and cost you control mid-acquisition.
* **The settings hooks run with the device lock released**, so a slow camera
  cannot stall the stream thread. They do share the control thread with GVCP,
  though, so keep them under about 20 ms — a client built with fast heartbeats
  gives a command only 25 ms and three retries.
* **Hooks dispatch on (address, length), not on the GVCP command.** A client
  only uses `READ_REGISTER` for 4 byte accesses; an 8 byte float or a string
  register always arrives as `READ_MEMORY`. Keying off the command gives hooks
  that fire for integers and silently never fire for floats.
* **Geometry is latched at `AcquisitionStart`.** The client sizes its buffer
  from `PayloadSize` and then drops any packet past the count that implies,
  with no error, so changing width mid-stream would produce black frames and
  no message.

#### Dropped packets ####

Packet resend is deliberately not advertised, so a lost packet costs a whole
frame. If you see failures, raise the receive buffer on the client machine:

```
$ sudo sysctl -w net.core.rmem_max=20000000
$ sudo sysctl -w net.core.rmem_default=20000000
```

Large frames want jumbo frames on both ends and a matching `GevSCPSPacketSize`.
The device honours the packet delay register if a client sets one.

#### Tests ####

```
$ PYTHONPATH=src:tests python -m pytest tests/
```

The suite needs no network and no Aravis: `tests/fakeclient.py` is a minimal
GigE Vision client that drives the whole handshake over loopback. Where it
reassembles a frame it uses the same rule the real client uses — byte offset
derived from the packet id — rather than trusting anything the device claims.

#### Status ####

Verified against Aravis 0.8 (`arv-tool-0.8`, `arv-camera-test-0.8`): discovery,
GenICam feature tree, control privilege and heartbeat, and continuous
acquisition with zero size mismatches, timeouts or missing frames.

Not implemented: packet resend, multipart payloads, chunk data, message
channels, extended (64 bit) frame ids, and more than one stream channel.
