### py-gige-emulator ###

[![tests](https://github.com/HazenBabcock/py-gige-emulator/actions/workflows/tests.yml/badge.svg)](https://github.com/HazenBabcock/py-gige-emulator/actions/workflows/tests.yml)

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
    # provided, as are the stream channel's own GevSCPSPacketSize and GevSCPD.
    # Pass roi=True for a writable Width and Height plus OffsetX and OffsetY.
    # Declare anything else you want the client to see.
    # The category is where a client's feature tree files this, and it is
    # worth getting right: exposure is a time and gain is an amplitude, so
    # they belong in different ones however often they are tuned together.
    extra_features = (
        FloatFeature("ExposureTime", "Exposure time", "AcquisitionControl",
                     "RW", default=10000.0, min=1.0, max=1e6, unit="us"),
        FloatFeature("Gain", "Analog gain", "AnalogControl", "RW",
                     default=0.0, min=0.0, max=24.0, unit="dB"),
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
$ python examples/basler_camera.py --interface eth0     # Basler USB camera, via pypylon
$ python examples/allied_vision_camera.py --interface eth0   # Alvium USB, via vmbpy
```

then point any GigE Vision client at it. They all take `--interface` to pick
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

That device id is separately settable with `--vendor`, `--model` and
`--serial`, and it is what a client *lists* — `--name` does not appear there.
So two of the same camera need different ids, and a client that pins one in
its configuration needs the id to match:

```
$ python examples/pi_camera.py --interface eth0 --serial GV01
INFO ... listening on 192.168.1.225:3956 as py-gige-emulator-PiHQ-GV01
```

`opencv_camera.py` and `pi_camera.py` also take `--list-modes` and
`--mode`, since both sit on hardware that offers a fixed set:

```
$ python examples/pi_camera.py --list-modes
  1332x990/SRGGB8            bit_depth=8    max_fps=147.91
  2028x1520/SRGGB12_CSI2P    bit_depth=12   max_fps=45.19
  ...
$ python examples/pi_camera.py --interface eth0 --mode 1332x990/SRGGB8
```

For the Pi the format is part of the selection, not decoration — libcamera
picks the sensor readout from it, and the shallower ones are markedly faster.
At 1332x990 that is a measured 147.8 fps against 101.8. A bare `WIDTHxHEIGHT`
takes the deepest readout of that size. A webcam has no equivalent, so
`opencv_camera.py` takes a size alone.

The two vendor examples are a different shape from the others. Both wrap a
**USB3 Vision** camera, so they are not one network protocol repackaged as
another: they put a camera with no network interface of its own onto the
network. Both read their vendor, model and serial off the camera at startup
and report those, and both default their MAC to their vendor's OUI, because
neither vendor's client will enumerate a device without one — see "Not
showing up in a vendor's client" below. Pass `--mac none` to report the
interface's own address instead.

They expose the region of interest, exposure and its automatic mode, the
frame rate, the pixel format and binning, with every bound read from the
camera rather than written down: the exposure range, the ROI increments, the
formats that exist. A client builds its controls from those, so an invented
bound produces a control that refuses half its own range.

```
$ python examples/basler_camera.py --list
Basler           acA1440-220um      40272323     BaslerUsb
$ python examples/basler_camera.py --interface eno1 --reset
serving Basler acA1440-220um (40272323) at 1456x1088 Mono12, ctrl-c to exit.
```

`--reset` puts the camera back to full frame with no binning first. It is off
by default, because a camera is entitled to keep the settings it was left in
and a vendor's own viewer does not reset one either — but those settings
outlive the process that made them, so an ROI left behind by an earlier run
is otherwise inherited in silence.

`allied_vision_camera.py` needs `GENICAM_GENTL64_PATH` pointing at VimbaX's
`cti` directory; vmbpy fails with "No TL detected" without it.

**`--mode` and `--pixel-format` are independent**, which is the easy thing to
trip over. `--mode` chooses how the *sensor* is read — those are the `SRGGB…`
names, and none of them is what goes on the wire. `--pixel-format` chooses
what the *client* receives:

```
$ python examples/pi_camera.py --interface eth0 --mode 2028x1520 --pixel-format raw
```

`raw` is spelled that way on purpose: the Bayer layout's name depends on the
sensor's rotation, so it is not knowable until the camera is open. The exact
name works too once `--list-modes` or the startup line has told you what it
is.

`noise_camera.py` needs nothing but Python and is the quickest way to check the
emulator reaches your client. `opencv_camera.py` serves a UVC webcam and shows
the settings hooks driving real hardware, including reading back what the
camera actually did with a request. `pi_camera.py` serves an IMX477 over
libcamera/Picamera2, and shows passing the sensor's own frame number and
timestamp through so a gap in the client's frame ids means a frame the
pipeline genuinely dropped.

#### Pixel formats ####

Declare what a camera can deliver with `pixel_formats`, and the client picks:

```python
camera = MyCamera(width=640, height=480,
                  pixel_format="Mono8", pixel_formats=["Mono8", "Mono16"])
```

`PixelFormat` is writable when that list holds more than one entry and
read-only when it does not, so a camera with one format says so rather than
accepting a write that changes nothing. `PayloadSize` follows automatically —
your `next_frame()` only has to return `self.geometry["payload"]` bytes of
whatever is currently selected. It is payload-affecting, so a client cannot
change it mid-acquisition.

Mono8/10/12/16, RGB8, BGR8 and the twelve Bayer layouts (`BayerRG8`,
`BayerBG16`, …) are recognised. **A colour sensor's raw output is Bayer, and
calling it Mono is not a harmless approximation** — the client renders a
checkerboard and has no way to know there is colour to recover. The two
letters are the top-left 2x2 phase, and they change with sensor rotation, so
read them from whatever your camera reports rather than assuming: the Pi HQ
camera's sensor is RGGB, but it reports a 180° rotation and libcamera hands
back `SBGGR16` accordingly, so hardcoding the sensor's phase would swap the
client's red and blue.

`pi_camera.py` shows the whole pattern. It offers `RGB8` from the ISP's
demosaiced stream and the raw Bayer layout it detects at startup, and
switching between them reconfigures the sensor. RGB8 is the default: the ISP
demosaics with the sensor's own tuning file, which beats anything a client
will reconstruct.

#### Design notes ####

* **The GenICam XML is generated from the feature declarations**, not hand
  written. The register map and the XML come from the same source, so they
  cannot drift — an `<Address>` that disagrees with what the device stores
  fails in a way that looks like a client bug.
* **The XML is served zipped**, as real cameras do, and the URL's `.xml.zip`
  filename is what tells the client to inflate it. The download costs round
  trips rather than bandwidth, and how many depends on the client: Aravis
  hardcodes 512 byte reads, pylon asks for 1256, and this device answers up to
  1460 — what fits in one datagram. A typical feature set is 8 kB of XML that
  deflates to under 1.5 kB, so at Aravis's chunk size that is 16 round trips
  against 3, out of about 28 for the whole open. Invisible on a fast
  link and most of the wait on a slow one. Pass `compress_xml=False` if you
  meet a client that cannot inflate; the device also falls back on its own if
  the archive would not be smaller, because a client decides an entry is
  compressed by comparing the two stored sizes and would copy non-shrinking
  deflate output out raw.
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
* **Any command counts as a heartbeat, not just the privilege read.** A
  client that is talking to the device has obviously not crashed, and the
  heartbeat exists to catch one that has. Counting only the nominal heartbeat
  drops a client that is merely busy: clients serialise control access behind
  one mutex and heartbeat on a one second period, so a burst of feature reads
  at startup starves the heartbeat and control is released mid-initialisation.
  Genuine silence still releases it, which is the case that matters.
  `heartbeat_timeout_ms` (3000 by default) covers a client that goes quiet for
  a long time on purpose.
* **Bounds can be computed, and changes can be pushed.** A feature's `min` and
  `max` are what the device can *ever* do; `p_max="OtherFeature"` emits
  `<pMax>` so the client's range tracks what is reachable now. `EmulatedCamera`
  takes `max_frame_rate` for the common case of a sensor whose ceiling is
  simply not 1000 fps. Separately, `invalidated_by=("ExposureTime",)` emits
  `<pInvalidator>` on the register node, which is the only thing that makes a
  client re-read: without it a GUI shows the value it fetched when it built
  its tree, however faithfully the device updates the register. Use it when a
  feature's value moves *on its own* — a long exposure dragging the frame rate
  down — not merely when one is writable.
* **`AcquisitionMode` is honoured.** `SingleFrame` delivers one frame per
  `AcquisitionStart` and then stops, so a snapshot client starts again for
  each one. Your `next_frame()` sees no difference between the two modes.
* **Geometry is latched at `AcquisitionStart`.** The client sizes its buffer
  from `PayloadSize` and then drops any packet past the count that implies,
  with no error, so changing width mid-stream would produce black frames and
  no message.

#### Dropped packets ####

Packet resend is implemented and advertised, so a lost packet need not cost
the frame: the client notices the gap, asks for the missing range, and the
device serves it from the handful of frames it keeps for the purpose.

Most of what this section used to report was the device's own doing. The
stream socket asked for an 8 MB send buffer, which at gigabit is 65 ms of
video queued inside the sender -- deep enough to overrun the host's own
transmit scheduling, and deep enough that a resend left the machine long
after the client had given up on the frame it was meant to repair. That
buffer is now 256 KB (`SEND_BUFFER_SIZE` in `stream.py`). Measured against an
IMX477, Aravis with `-a`, 20 s per row, the client's `rmem_max` at 16.8 MB:

| frame | packets | completed | failed | missing packets | resend requests |
|---|---|---|---|---|---|
| 2028x1520 RGB8, 9.2 MB | 6,282 | 200 | 0 | 0 | 0 |
| 4056x3040 BayerBG16, 24.6 MB | 16,753 | 78 | 0 | 0 | 0 |
| 4056x3040 RGB8, 37.0 MB | 27,120 | 50 | 0 | 0 | 0 |

Every one of those rows used to lose packets, the bottom two badly: 63,741
and 27,121 missing, and 343,271 resend requests behind the middle row alone.

So the advice this section used to give -- raise the client's receive buffer
past the frame size, or pace the burst with the packet delay -- is no longer
the first thing to reach for. A 37 MB frame is more than twice the receive
buffer above and arrives whole. The packet delay still works and still costs
what it did: the same row with 20 us between packets
(`arv-camera-test -a -m 5000 -y 20000`) falls from 50 frames to 19, now for
nothing that needed fixing. Both remain worth trying on a link that really
does lose packets, since resend cannot rescue what never reached the socket:

```
$ sudo sysctl -w net.core.rmem_max=20000000
$ sudo sysctl -w net.core.rmem_default=20000000
```

**A client that completes every frame is not evidence the device is
behaving.** Aravis repairs quietly and reports success: on the old 8 MB
buffer the 37 MB row still completed every frame, with 542 resend requests
behind it. VimbaX gives up on a frame sooner, and on a second bench that same
buffer cost it three quarters of its frames -- while Aravis, on that bench and
that buffer, still reported no failures at all. Read the resend counts in
`server.stats` alongside the failures.

Jumbo frames help by cutting the packet count, if every hop supports them and
`GevSCPSPacketSize` matches.

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

#### Slow to open ####

If a client takes seconds to open the camera while the device is answering
promptly, suspect the route before the device. Opening is round trip bound —
about 28 commands, over half of them the XML download — so a link with a
100 ms round trip turns a 15 ms open into three seconds.

The trap is two interfaces on one subnet. A client broadcasts discovery from
**every** interface it has; if your wired and wireless interfaces are both on,
say, `192.168.1.0/24`, the wireless broadcast still reaches the camera over
the wired segment, the camera answers it, and the client may keep that
answer and run the whole session over WiFi. Measured here: 0.44 ms round trip
over ethernet, 37 ms over WiFi to the same camera.

`--interface` does not help, and cannot: it restricts which of the *device's*
interfaces answer, which it does — verified by capturing each one — but the
duplicate is on the client's side, and a request arriving on the permitted
interface from a host on that subnet is indistinguishable from any other. A
real camera behaves the same way. Give the camera its own subnet.

To tell the two apart, capture at the device and look at which side the gaps
are on. Time between a request arriving and the answer going out is the
device; time between an answer and the next request is the link.

#### Not showing up in a vendor's client ####

Some transport layers only admit their own manufacturer's cameras, and they
decide from the **MAC address** in the discovery ack — the first three octets
are the vendor's IEEE OUI. Nothing is ever sent from the reported MAC, so
`--mac` sets it to whatever a client wants to see:

```
$ python examples/noise_camera.py --interface eth0 \
      --mac 00:30:53:12:34:56 --vendor Basler --model acA1300-30gm
```

Measured on this bench, with Aravis listing the same camera in every run as a
control, and again with a matching MAC but the emulator's own vendor name:

| client | enumerates with our own identity | what it wants |
|---|---|---|
| Aravis | yes | nothing |
| ImpactAcquire (Balluff) | yes | nothing |
| VimbaX (Allied Vision) | no | one of exactly two OUIs, `00:0a:47` or `00:0f:31` |
| pylon (Basler) | no | a Basler OUI, `00:30:53`, **and** the vendor name `Basler` |

Model and serial are free in both cases. Vendor names that are known to
change client behaviour are logged as a warning rather than refused, because
this is what they are for — but Aravis switches its own per-vendor workarounds
on the same string, so borrowing one is not free.

VimbaX's two are the whole list rather than the two that happened to be
tried: its GigE transport layer tests the top three octets against two fixed
values, with no table behind them and no setting that widens the check. So
borrow one of those rather than matching some particular camera — Allied
Vision's newer blocks are not accepted by this build, and neither is anyone
else's. The vendors also check separately, not against some shared list of
camera makers: with Basler's `00:30:53`, VimbaX reports zero devices.

Both OUIs above come from the IEEE MA-L registry, which is public — on
Debian, `/usr/share/ieee-data/oui.txt`.

Two things worth being clear about. An OUI is assigned by the IEEE to a real
company, so a borrowed one belongs in a deliberate flag on a private network
and never in a default. And enumerating is not the same as working: a vendor's
client may go on to ask for features only its own cameras have.

Wanting a bigger read than Aravis does is a separate trap in the same area.
pylon downloads the XML in 1256 byte reads where Aravis uses 512, and the
device used to refuse anything over the smaller number — an open that failed
with `Failed to read memory at 0x10000, 0x4e8 bytes` and looked like a
corrupt XML. The limit is now what fits in one datagram.

#### Exposure ####

Running this puts a UDP listener on port 3956 of the host. **GigE Vision has
no authentication**, by design and in every implementation: anyone who can
reach that port can take control of the camera, change its settings and start
a stream. That is the protocol rather than anything specific to this device,
and a real camera behaves the same way — but a real camera sits on a camera
network, while this runs on a general purpose machine with a real uplink,
which makes it a much better amplifier if it is left somewhere it can be
reached.

Two defaults exist to keep it from being an easy one:

* **The stream goes only to the client that asked for it.** The destination is
  a register the client writes, so a device that honours it unconditionally
  will send megabytes per second wherever it is told — and since the client's
  address is a UDP source, a handful of forged packets naming a third party
  would do it, with no reply ever going back to whoever sent them. The write
  that names another address is **refused, with an error the client reports**,
  rather than accepted and then ignored when the stream starts: a client told
  nothing simply waits out its grab timeout with no frames and no reason.
  Pass `allow_any_destination=True`, or `--any-destination` to any of the
  examples, if handing the images to another machine is what you want.

  This also catches something worth knowing about even when it is harmless. A
  client on a host with two interfaces on one network may control the camera
  over one and ask for the images on the other — pylon does exactly this here,
  controlling over the wire and asking for the stream at its own WiFi address.
  It works, in the sense that images arrive, but the commands and the images
  are then taking different paths, and a camera on a wire ends up delivering
  over WiFi. The refusal says so; `--any-destination` accepts it deliberately,
  and taking the second interface down avoids it.
* **Writing requires holding control**, with one exception: the write that
  claims control. Accepting writes from anyone whenever the device happened to
  be idle is looser than the standard, and it removed the need for an attacker
  to complete any handshake at all.

What remains is inherent to the protocol. Discovery answers whoever asks —
eight bytes in, 256 back — so the device is a small reflector to anyone able
to forge a source address, exactly as a real camera is. A client that does
hold control can ask for a full rate stream at any time.

So: give it `--interface` to pin it to one network, do not route it anywhere
untrusted, and do not forward 3956 through anything. It listens on an
unprivileged port and writes no files, so it never needs root — do not give it
any.

#### Tests ####

```
$ pip install -e ".[test]"
$ python -m pytest
```

`PYTHONPATH=src python -m pytest` works too — pytest puts `tests/` on the path
itself — but installing exercises the packaging as well, so a `pyproject.toml`
that no longer builds fails here rather than on someone's first `pip install`.
The `test` extra pulls in numpy, which `tests/test_pi_modes.py` needs: it
imports the Pi example, and while picamera2 is stubbed there, numpy is not.

The suite needs no network and no Aravis: `tests/fakeclient.py` is a minimal
GigE Vision client that drives the whole handshake over loopback. Where it
reassembles a frame it uses the same rule the real client uses — byte offset
derived from the packet id — rather than trusting anything the device claims.

GitHub Actions runs it on every push, on Python 3.10 through 3.13. Linux only,
and deliberately: `netif.py` imports `fcntl` at module scope, so the package
does not import at all on Windows, and `SO_BINDTODEVICE` is Linux specific.

#### Status ####

Verified against Aravis 0.8 (`arv-tool-0.8`, `arv-camera-test-0.8`): discovery,
GenICam feature tree, control privilege and heartbeat, and continuous
acquisition with zero size mismatches, timeouts or missing frames.

Most recently on real hardware — an IMX477 on a Raspberry Pi 5 serving
1332x990 RGB8 over `examples/pi_camera.py`, read by Aravis 0.8.36 on a second
machine across a wired link:

```
$ arv-camera-test-0.8 -n bench-hq -a --duration 20
n_completed_buffers    = 201        n_missing_packets      = 0
n_failures             = 0          n_size_mismatch_errors = 0
n_missing_frames       = 0          n_received_packets     = 583503
```

800 MB at 39.6 MiB/s, no errors of any kind. Note `--duration` is what stops
the tool: `-m` is frame retention and `-y` the packet delay, and neither
bounds the run, so without it the tool streams until interrupted.

Also verified against a second, independent commercial stack — Balluff's
`mvGenTLProducer.cti` — which enumerates the camera with full identity and
`readwrite` access and streams complete frames from it. `tools/gentl_probe.py`
drives any GenTL producer through the C API to check this, with `--stream N` to
grab frames; it needs no SDK and no bindings to compile. Use it before blaming
the emulator, because a vendor viewer cannot tell you whether its producer
never looked, looked and rejected the device, or found it and filtered it at
the application layer — and all three happen.

Not implemented: multipart payloads, chunk data, message channels, extended
(64 bit) frame ids, and more than one stream channel.
