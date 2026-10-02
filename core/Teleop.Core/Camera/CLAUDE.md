# Camera

The pure half of the camera downlink in `docs/adr/0014-camera-frame-downlink.md`: the wire format
and frame reassembly. The robot-side sender (`core/Teleop.CameraHost/`) and the Unity viewer
(`Bridge/CameraFeedBridge.cs`, not yet written) own the sockets, the capture device and the
texture; everything here is a deterministic function of the bytes and ticks it is given.

## Contents

| File | What it is |
|---|---|
| `CameraChunkCodec.cs` | one chunk: a 41-byte little-endian header plus at most 1159 bytes of JPEG, in a datagram of at most 1200 bytes (ADR 0014 §3) |
| `CameraSubscribeCodec.cs` | the viewer's 13-byte keepalive (ADR 0014 §2) |
| `CameraFrameReassembler.cs` | chunks back into frames, newest wins, every drop counted |
| `CameraLatencyRecorder.cs` | one displayed frame's stamps into the `camera_*` metrics of `docs/metrics.md` §9, through the pose path's `ClockSync` |
| `CameraFrameStamp.cs`, `CameraChunkHeader.cs`, `CameraFrameInfo.cs`, `CameraChunkOutcome.cs`, `CameraReassemblerDiagnostics.cs`, `CameraLatencySample.cs` | the value types those use |

## Why there is no `Contracts/` interface or registry entry

There is one wire shape and one rule for which frame to show, not a family of competing
implementations, so there is nothing to swap and nothing to benchmark against (ADR 0014 §6). The
day there is a second way to choose a frame (a jitter buffer for video, say), that choice becomes a
`Contracts/` interface and a research axis like `Buffering/`, through its own ADR.

## Rules specific to this folder

- **Ticks are carried, never read.** Stamps on a chunk are in the robot's clock domain; arrival,
  decode and render ticks are whatever the host passed in. Converting between the two is
  `ClockSync`'s job (ADR 0014 §5): `CameraLatencyRecorder` reads the pose path's estimate and never
  updates it. Nothing here reads a clock.
- **Newest wins, and every frame that is not shown is counted.** A reassembler that quietly
  discards frames hides the loss the camera-latency metrics need to report.
- **A chunk may only ever write into its own frame.** Every chunk claiming a frame id must agree on
  that frame's stamp and chunk count; one that does not is rejected rather than merged.
- **Metrics come only from `CameraLatencyRecorder`,** and every name it emits is defined in
  `docs/metrics.md` §9. The reassembler counts in its diagnostics struct and emits nothing; the host
  passes the increase in its drop counters to `RecordDropped`.

## What the sender and viewer must do (found by review, not yet built)

- **Newest wins forever, so a sender restart freezes the view.** The reassembler only moves
  forward: if the sender restarts its frame ids at 0 after the viewer has shown frame 5000, every
  new frame is `Late` until the counter passes 5000 again (minutes at 30 fps). One forged
  far-future frame id does the same, and a forged incomplete one can hold a slot. The fix belongs
  to the hosts: the viewer calls `Reset()` whenever it (re)subscribes or sees a long run of `Late`,
  and the sender does not restart at a fixed id. The protocol has no authentication, like the pose
  path, so forged datagrams are a known limit rather than something this folder can stop.
- **Every chunk of a frame carries the same stamp, `sendTicks` included.** `sendTicks` is when the
  frame's first chunk was handed to the socket (ADR 0014 §3), not each chunk's own send time. A
  sender that stamps chunks individually gets every chunk after the first rejected as
  `Inconsistent`.
- **`Teleop.Core.Camera` shadows `UnityEngine.Camera`** for code inside a `Teleop.Core.*`
  namespace. Bridge code lives in `Teleop.Bridge`, so a bare `Camera.main` resolves normally there;
  keep it that way, or qualify it.
