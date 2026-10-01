# 14. Camera frames travel as a second, timestamped downlink of JPEG chunks over UDP

## Status

Accepted. The two questions left open in the first draft are recorded under "Resolved questions".

## Context

Root `CLAUDE.md` is explicit that **nothing in this system observes the environment today**: the
robot reports a 57-byte `RobotStateFrame` (`core/Teleop.Core/Pipeline/RobotStateFrameCodec.cs`) and
nothing else. That is no longer true of the hardware. The JetRover carries an **Orbbec DaBai DCW**
depth camera whose colour stream enumerates as an ordinary UVC device, `/dev/video0`, and
`just robot-camera-check` captured a 640×480 frame from it on 2026-10-01. Nothing streams it yet.

Two things want that stream:

1. **An operator view.** The immediate ask: show the robot's camera in the Unity scene.
2. **The research direction.** `docs/proposals/2026-09-29-learned-model-mediated-teleoperation.md`
   puts this ADR first in its plan. Its slow loop needs camera frames whose capture time is known on
   the same clock as every pose, and its classical-predictive-display baseline (delayed video with
   the predicted arm overlaid) needs the same thing.

Requirement 2 is what rules out the easy options. A video player in Unity that shows "the feed"
would satisfy 1 and produce no number. This project's rule is that a figure which cannot be traced
to a stamped, recorded measurement does not count, and `docs/metrics.md` §1 already defines the
stamps (`t_capture`, `t_send`, `t_recv`, `t_playout`, `t_render`, `t_photon`) every sample must
carry. A camera frame is a sample.

Facts about the existing system that shape the decision:

- **The ports in use.** `Teleop.RobotHost` listens on UDP 6000 (Cartesian commands) and 6002
  (joint-angle commands, uplink only); the operator receives state replies on 6001.
- **Reply targets.** As of branch `robothost-reply-to-sender` (not yet on `main`), RobotHost replies
  to the source address of the latest command instead of a configured `--remote-host`. This ADR
  reuses that model rather than inventing a second one.
- **The robot clock.** `Teleop.RobotHost/Time/MonotonicClock.cs` is `Stopwatch`, which on Linux is
  `CLOCK_MONOTONIC` at 1 GHz. V4L2 drivers stamp each captured buffer on `CLOCK_MONOTONIC` as well
  (`V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC`). A frame's capture time is therefore available in exactly
  the clock domain `ClockSync` already relates to the operator's, without a second synchronisation.
- **Clock sync is a property of the pose path.** `Time/ClockSync.cs` builds its offset estimate
  from four-timestamp round trips on the command/state exchange (ADR 0002, ADR 0008). A one-way
  video stream produces no round trips of its own.
- **`.tlog` is a line-oriented text format** with a 512-byte line limit
  (`Recording/RecordFormat.cs`, `Recording/CLAUDE.md`). Readers skip unknown tags while still
  folding them into the checksum, so adding a tag is forward-compatible. A JPEG does not fit in a
  line and should not try.
- **Core may not do I/O, threads or allocation on the hot path** (invariants 3 and 8). Sockets and
  texture upload belong to the hosts; anything pure (wire encoding, reassembly) can live in Core and
  be tested headlessly.

## Decision

### 1. A separate stream on its own port, never multiplexed into `RobotStateFrame`

Frames go robot → operator on their own UDP flow. The pose downlink stays exactly as it is.

The pose stream is 57 bytes per reply and every latency figure in the project rides on it. Putting
tens of kilobytes per frame on the same flow would make pose loss and pose delay depend on picture
content, and would break the reply-per-command invariant `RobotEndpoint` relies on. Separate flows
keep each stream's loss and delay attributable to itself.

**Ports:** the robot-side camera sender listens on **UDP 6003**. The operator binds a local port of
its choosing (default **6004**) and sends from it.

### 2. The operator subscribes with a keepalive; the sender streams to its source

The operator sends a small fixed-size **subscribe datagram** to robot port 6003 once a second. The
sender streams frames to the source address and port of the most recent subscribe, and stops when
none has arrived for **3 s**.

This is the same reply-to-sender model RobotHost now uses, for the same reasons: no configured
operator address, NAT-correct by construction, and the Quest or the PC can be the viewer without a
redeploy. The timeout matters more here than for poses: an unsubscribed sender would otherwise keep
pushing megabits per second at an operator who has gone, possibly through a Tailscale relay.

The subscribe datagram carries a requested maximum frame rate, so a viewer on a constrained link
can ask for less.

### 3. JPEG frames, split into chunks of at most 1200 bytes

Each frame is one JPEG, split into chunks small enough to never fragment at the IP layer. 1200
bytes leaves room for IPv4/UDP headers and for WireGuard's overhead when the path is Tailscale
(1280-byte inner MTU).

Every chunk carries a fixed little-endian binary header, then its slice of the JPEG:

| Field | Type | Meaning |
|---|---|---|
| `magic` / `version` | `u16` / `u8` | rejects anything that is not this protocol, and old versions |
| `frameId` | `u32` | increments per frame; wraps |
| `chunkIndex` / `chunkCount` | `u16` / `u16` | position of this slice within the frame |
| `captureTicks` | `i64` | **driver** buffer timestamp, robot domain (see §4) |
| `sendTicks` | `i64` | when the frame's first chunk was handed to the socket, robot domain |
| `ticksPerSecond` | `i64` | robot clock rate, per ADR 0008 |
| `width` / `height` | `u16` / `u16` | decoded size, so the receiver can allocate once |
| `payloadBytes` | `u16` | bytes of JPEG in this chunk |

There is no retransmission. A frame missing any chunk when a newer frame completes is discarded and
counted. The receiver only ever moves forward: a completed frame older than the one already shown is
dropped rather than displayed. For a live view, a late frame is worse than no frame.

**The camera produces the JPEGs itself.** `just robot-camera-check` lists the DaBai's colour stream
on 2026-10-01 as offering `MJPG` at 640×480 through 1920×1080, every size at 5–30 fps (and raw
`YUYV` only at 640×480 for anything above 5 fps). The sender therefore forwards the camera's own
compressed frames without decoding or re-encoding them: there is no encode stage on the Nano to cost
CPU or add latency. The default is **640×480 at 30 fps**. The timestamp spike (resolved question 1) showed the
camera's own latency is about one frame period, so 30 fps halves it (32 ms against 68 ms at 15 fps)
for twice the bandwidth, which the lab LAN carries easily.
UVC cameras sometimes omit the Huffman tables (`DHT`) from MJPEG frames, which some decoders refuse.
An early acceptance check is that a raw frame from the device decodes in `Texture2D.LoadImage` on
the Quest; if it does not, the sender inserts the standard tables, which is a byte-level splice and
not a re-encode.

JPEG rather than H.264/WebRTC because every frame is independently decodable, so one lost chunk
costs one frame and not a group of pictures. Each frame also carries its own timestamps, and the
receiving side is `Texture2D.LoadImage` in Unity with no plugin or Android codec negotiation. The
price is bandwidth: this camera's frames measured about 42 KB at 640×480, so about 10 Mbit/s at
30 fps (5 Mbit/s at 15 fps). That is fine on the lab LAN and is the reason H.264 stays an option
for later (see "Not decided here").

### 4. The capture stamp is the driver's buffer timestamp, or it is unset

`captureTicks` is the V4L2 buffer timestamp, which the driver takes at capture on
`CLOCK_MONOTONIC`. It is **not** the time the sender process happened to read the buffer: that
would fold queueing inside the capture stack into a number labelled "capture".

A capture implementation that cannot obtain the driver timestamp must send `captureTicks` as unset
(the same `LatencyTrace.Unset` sentinel, `long.MinValue`) rather than substituting its own read time. A
frame with an unset capture stamp is still displayed. It simply contributes to no capture-based
metric. This is invariant 10 applied to data: a stand-in that looks like a measurement is worse
than an honest gap.

Before any capture-based number is cited, the sender's clock domain must be checked on the device:
its `CLOCK_MONOTONIC` and RobotHost's `Stopwatch` must agree. They should by construction; the check
is cheap and the failure would be silent.

### 5. Operator-side conversion reuses the pose path's `ClockSync`

Robot-domain stamps on a frame are converted to the operator's timebase with the **existing**
`ClockSync` estimate maintained by `OperatorEndpoint` on the pose path. The video stream gets no
synchronisation of its own.

Consequences, stated here so nobody discovers them in a plot: frame latency figures exist only while
the pose path is running and synced (`ClockSyncDiagnostics.IsSynced`), are only as accurate as that
estimate, and carry its uncertainty. Frames received before sync, or while it is lost, are shown but
produce no latency samples.

### 6. Pure parts in Core, I/O in the hosts

- **Core: a new `Camera/` folder** with the chunk codec (encode and decode one chunk header plus
  payload, allocation-free, caller-owned spans like `ICommandCodec`) and a **frame reassembler**:
  a fixed number of preallocated frame slots sized for the declared maximum frame (256 KB), newest
  wins, counting incomplete and out-of-order frames. Both are deterministic and headlessly testable,
  including against loss, duplication and reordering from the existing `EmulatedTransport`
  impairments.
- **No `Contracts/` interface and no registry entry yet.** There is one wire shape and one
  reassembly rule, not a family of competing implementations. The moment there is a second way to
  decide which frame to show (a jitter buffer for video, say), that becomes a `Contracts/`
  interface and a research axis like `Buffering/`, through its own ADR.
- **Robot side: `core/Teleop.CameraHost/`**, a sibling .NET project to `Teleop.RobotHost`. It
  captures the camera's MJPEG frames as they are, stamps, chunks, and serves subscriptions. See
  resolved question 1 for why it is .NET and what has to be proven first.
- **Unity: a new `Bridge/CameraFeedBridge.cs`.** It owns the socket, sends the keepalive, stamps
  `t_recv` at socket dequeue (`docs/metrics.md` §1 warns what happens otherwise), feeds the Core
  reassembler, decodes with `Texture2D.LoadImage`, and draws on a panel in the scene. It is the
  first `Bridge/` code that uploads textures every frame, so it **needs human review and an IL2CPP
  Quest build**; `just bridge-check` will compile it and catch nothing else.

### 7. The panel is world-locked, not head-locked

The feed is drawn on a quad fixed in the scene, near the arm proxy, never attached to the head.
Video that moves with the head while its content lags the world is a textbook sensory conflict, and
this project's human-factors survey (`docs/literature/human-factors.md`) is explicit about how
little is known about what operators tolerate. Placement is configurable. The default is beside
the arm proxy (resolved question 2).

### 8. Recording: metadata in the `.tlog`, pixels in a sidecar

- **A new `.tlog` tag, `CV`**, one line per frame with its id, chunk count, byte count, completeness,
  and every stamp the frame acquired (`t_capture`, `t_send`, first and last chunk `t_recv`,
  decode end, `t_render`), unset where absent. The tag is additive, so the format version does not
  change and existing readers skip it.
- **JPEG payloads go to a sidecar file next to the session** (`<session>.frames`: length-prefixed
  JPEGs in arrival order, keyed by `frameId`), listed in the run manifest with its checksum.
  `verify` keeps operating on the `.tlog` alone. A replay that needs pixels reads the sidecar.

### 9. Metrics, defined in `docs/metrics.md` by the PR that first emits them

- `camera_owd_ms`: last-chunk `t_recv` minus `t_send`, ClockSync-converted.
- `camera_capture_to_render_ms`: `t_render` minus `t_capture`. The software estimate of
  glass-to-glass latency, the video counterpart of M2P.
- `camera_frame_dropped`: one sample per frame discarded as incomplete, superseded or late.
- A stage breakdown: capture→send, transit, reassembly, decode, render. It must sum to
  `camera_capture_to_render_ms` within tolerance, as `docs/metrics.md` §2 requires of M2P's.

`camera_capture_to_render_ms` gets one physical validation before it is cited, as M2P does
(ADR 0003): the camera films a millisecond counter rendered in the headset or on the operator's
monitor, and the photographed difference is compared with the software figure.

## Resolved questions

Decided 2026-10-01 by the project owner.

1. **The robot-side sender is a new sibling .NET project, `core/Teleop.CameraHost/`**, deployed
   like `Teleop.RobotHost` with its own `just` recipe and systemd unit. That gives one toolchain, one
   deploy story and one clock implementation (`Stopwatch`) on the robot. It was chosen over a Python
   sender because `robot/` is documentation-only and the ROS workspace's repository target is
   unsettled (the existing `jetrover-teleop-ros` should not receive new pushes until a destination
   is chosen).

   The cost is that .NET has no V4L2 API, so reading driver buffer timestamps needs P/Invoke on the
   V4L2 ioctls (`VIDIOC_REQBUFS`, `VIDIOC_QBUF`, `VIDIOC_DQBUF`, `mmap`). **The first
   implementation step is a spike proving that `VIDIOC_DQBUF` timestamps can be read from .NET on
   the Jetson** and that they agree with `Stopwatch.GetTimestamp()` (§4). If the spike fails, §4
   still holds: the sender ships with `captureTicks` unset, and this decision is revisited rather
   than worked around with a read-time stamp.

   **Spike result, 2026-10-01: passed** (`just camera-timestamp-spike`, on the Jetson over the LAN):
   - `Stopwatch.Frequency` is 1 GHz, and a direct `clock_gettime(CLOCK_MONOTONIC)` fell between two
     `Stopwatch` reads in 1000 of 1000 samples: Stopwatch is `CLOCK_MONOTONIC` on this machine.
   - Every frame (`uvcvideo`, "Orbbec DaBai DCW RGB Camera") carried
     `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` with source **start of exposure**.
   - Age at dequeue (Stopwatch now minus driver stamp) is about **one frame period**: p50 68 ms at
     15 fps, p50 32 ms at 30 fps. Because the stamp marks the start of exposure, that age is the
     camera's own exposure, readout and USB transfer, before any software sees the frame. The
     camera, not the network, is the first and largest stage of the breakdown in §9, and it
     shrinks with frame rate.
   - Measured frame interval 68 ms at 15 fps and 32–36 ms at 30 fps; one dropped frame per run, at
     stream start.
   - Frames are about **42 KB** at 640×480 at either rate (about 5 Mbit/s at 15 fps, 10 Mbit/s at
     30 fps), and **every frame includes Huffman tables**, so §3's DHT-splice fallback is not needed
     for this camera.
2. **The panel sits beside the arm proxy**, world-locked, at roughly the proxy's height. The
   backdrop layout (delayed video behind the proxy, predicted arm on top, the classical predictive
   display) is deferred. It needs the camera's extrinsic calibration to line up with the proxy, and
   that calibration is its own piece of work, most likely together with depth.

## Not decided here

- **Depth.** The DaBai's depth stream needs Orbbec's SDK, not V4L2. It belongs to a later ADR, most
  likely together with camera-to-base calibration.
- **H.264 or WebRTC.** Revisit if JPEG bandwidth or Quest decode time turns out to be the limiting
  stage in the breakdown. The breakdown is what decides, not an argument made now.
- **Emulated impairment of the video flow** in live Unity sessions. The Core reassembler is tested
  against impairments headlessly from the start; wiring a camera flow through Unity's impairment
  controller can follow once the real stream exists.
- **Any consumer other than the display.** Frames reaching a world model, a perception stack, or a
  predictor is the proposal's later work and goes through its own ADR.

## Consequences

- The operator can see the robot's workspace, and every frame shown carries a capture time on the
  synced clock, so the first camera-latency figures are produced by the same instruments as every
  other latency figure here.
- Root `CLAUDE.md`'s statement that nothing observes the environment, and the corresponding lines in
  `README.md` and `core/Teleop.Core/Plant/CLAUDE.md`, become false once this lands and must be
  updated in that PR. The world-model direction paragraph stays as it is: a camera feed shown to a
  human is not a model of the world.
- The robot gains a second long-running process, `Teleop.CameraHost`, with its own systemd unit and
  deploy recipe, independent of `Teleop.RobotHost` so that a camera fault cannot stop arm control.
- The video stream's latency is only measurable while the pose path is synced (§5). A session that
  only watches the camera has no latency numbers by design.
- Bandwidth is about 10 Mbit/s at the defaults (640×480, 30 fps); a viewer can request less (§2). Fine on the lab LAN; poor through a Tailscale DERP
  relay. Camera latency figures must record which path they used, as pose figures already must.
- `Bridge/CameraFeedBridge.cs` is new code under human review, and only an IL2CPP Quest build
  verifies it.
