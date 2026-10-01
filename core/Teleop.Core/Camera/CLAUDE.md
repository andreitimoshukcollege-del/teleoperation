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
| `CameraFrameStamp.cs`, `CameraChunkHeader.cs`, `CameraFrameInfo.cs`, `CameraChunkOutcome.cs`, `CameraReassemblerDiagnostics.cs` | the value types those use |

## Why there is no `Contracts/` interface or registry entry

There is one wire shape and one rule for which frame to show, not a family of competing
implementations, so there is nothing to swap and nothing to benchmark against (ADR 0014 §6). The
day there is a second way to choose a frame (a jitter buffer for video, say), that choice becomes a
`Contracts/` interface and a research axis like `Buffering/`, through its own ADR.

## Rules specific to this folder

- **Ticks are carried, never interpreted.** Stamps on a chunk are in the robot's clock domain;
  arrival ticks are whatever the host passed in. Converting between the two is `ClockSync`'s job,
  done by the consumer (ADR 0014 §5). Nothing here reads a clock.
- **Newest wins, and every frame that is not shown is counted.** A reassembler that quietly
  discards frames hides the loss the camera-latency metrics need to report.
- **A chunk may only ever write into its own frame.** Every chunk claiming a frame id must agree on
  that frame's stamp and chunk count; one that does not is rejected rather than merged.
- **No metric names are emitted here.** The diagnostics struct counts; the host that owns the
  display turns those into `camera_*` metrics, defined in `docs/metrics.md` in the same PR.
