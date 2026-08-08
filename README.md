### py-gige-emulator ###

A pure Python GigE Vision camera emulator. It speaks GVCP and GVSP directly over
UDP, so any GigE Vision client on the network — `arv-viewer` and anything built
on [Aravis](https://github.com/AravisProject/aravis) — sees a real camera.

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

#### Examples ####

```
$ python examples/noise_camera.py  --interface eth0     # no hardware needed
$ python examples/opencv_camera.py --interface eth0     # any cv2.VideoCapture camera
$ python examples/pi_camera.py     --interface eth0     # Raspberry Pi HQ camera
```

then point any GigE Vision client at it. All three take `--interface` to pick
which network interface the camera appears on — naming one that does not exist
prints the ones that do — and `--name` to set the camera's user-defined name:

```
$ python examples/pi_camera.py --interface eth0 --name bench-left
$ arv-tool-0.8 -n bench-left control Width Height
```

That is the GigE Vision field intended for a human-chosen label (bootstrap
register 0x00e8). Clients index it, so it is how you select one of two
otherwise identical cameras without typing the full
`vendor-model-serial` device id.

`noise_camera.py` needs nothing but Python and is the quickest way to check the
emulator reaches your client. `opencv_camera.py` serves a UVC webcam and shows
the settings hooks driving real hardware, including reading back what the
camera actually did with a request. `pi_camera.py` serves an IMX477 over
libcamera/Picamera2 at 4056x3040 Mono16, and shows passing the sensor's own
frame number and timestamp through so a gap in the client's frame ids means a
frame the pipeline genuinely dropped.

#### Design notes ####

* **The GenICam XML is generated from the feature declarations**, not hand
  written. The register map and the XML come from the same source, so they
  cannot drift — an `<Address>` that disagrees with what the device stores
  fails in a way that looks like a client bug.
* **Two threads.** Control and streaming are separate because `next_frame()`
  is a real camera grab. A one second exposure on a single thread would
  block GVCP past the client's command timeout and cost you control
  mid-acquisition.
* **`next_frame()` sets the frame rate.** There is no timer in the stream
  thread — it sends frames exactly as fast as your hook returns them, so a
  real camera blocking until the sensor delivers paces the stream for free and
  at its true rate. A camera with no physical timing must pace itself, or the
  stream thread will saturate a core; `examples/noise_camera.py` shows the
  pattern. `AcquisitionFrameRate` is therefore something you push at your
  hardware in `set_camera_settings`, not something the emulator enforces.
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

Some clients size their own packets by asking the device to fire a test packet
at a candidate size and seeing whether it arrives. The device answers those
probes, including the don't-fragment bit, which is what makes an oversized
probe fail rather than quietly fragment — so such a client finds the path MTU
on its own and raising the MTU at both ends moves it up with nothing set by
hand. ImpactAcquire does this, and settles at 1488 on a plain 1500 byte link.

**Aravis does not.** It reads the packet size register and uses whatever it
finds, and never writes it — not even with `--packet-size-adjustment=always`,
which only ever reduces on a failure. For Aravis the emulator's own
`--packet-size` is the setting that matters, so match it to the link.

Note the register is device state that outlives the client that set it: once a
probing client has negotiated 1488, a later non-probing client sees 1488 too.
That makes back-to-back measurements easy to misread — restart the emulator
between them.

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

Also verified against a second, independent commercial stack — Balluff's
`mvGenTLProducer.cti` — which enumerates the camera with full identity and
`readwrite` access and streams complete frames from it. `tools/gentl_probe.py`
drives any GenTL producer through the C API to check this, with `--stream N` to
grab frames; it needs no SDK and no bindings to compile. Use it before blaming
the emulator, because a vendor viewer cannot tell you whether its producer
never looked, looked and rejected the device, or found it and filtered it at
the application layer — and all three happen.

Not implemented: packet resend, multipart payloads, chunk data, message
channels, extended (64 bit) frame ids, and more than one stream channel.
