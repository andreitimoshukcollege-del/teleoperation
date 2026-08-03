# Transport

`ITransport` implementations, `ICommandCodec` implementations, and the network emulator.

Core contains **no real I/O.** The transports here are in-process: loopback, and the emulator
that wraps another transport. Sockets live in `Bridge/UdpTransport.cs` because I/O is a host
concern, not because `System.Net` is unavailable (it is available under IL2CPP). Keeping Core
I/O-free is what makes replay bit-deterministic.

## Implemented

| Name | File | Idea |
|---|---|---|
| loopback | `LoopbackTransport.cs` | zero-impairment baseline: fixed-capacity FIFO ring, `arrivalTicks == sendTicks`, full ring returns false |
| emulated | `EmulatedTransport.cs` | decorator: fixed delay + uniform jitter + Gilbert-Elliott burst loss + explicit reorder knob, all from an injected `SeededRng` |

`loopback` has a `Registry/Registries.cs` entry (`Transports["loopback"]`). `EmulatedTransport`
deliberately does not: it is a decorator over another `ITransport` plus a `NetworkProfile` and a
`SeededRng`, a materially different constructor shape than `LoopbackTransport`'s
`(maxPayloadBytes, capacity)` -- see `Registry/CLAUDE.md` for the full reasoning. It needs an
entry (its own shape, or a small builder type) the moment a sweep needs to select a transport by
name rather than wiring one directly, as every current test does.

## EmulatedTransport

A **decorator** — it wraps any `ITransport` and injects delay, jitter, loss, and reordering
from a `NetworkProfile` or a captured trace. Most of the research runs through it, including
in Unity: wrap `UdpTransport` on a LAN and you get a reproducible impairment on a real socket.

Requirements:
- All impairment driven by the injected seeded RNG. Same seed + same input => same output.
- Trace-driven mode replays recorded one-way delays sample by sample, no resampling.
- Loss must model **bursts**, not just a Bernoulli rate. Real links lose runs of packets, and
  burst length is what breaks jitter buffers.

Status against those requirements: the first and third are met (`Types/NetworkProfile.cs`, a
2-state Gilbert-Elliott chain with expected burst length `1 / (1 - LossProbabilityAfterLost)`).
**Trace-driven mode is not implemented** — `core/testdata/traces/` does not exist yet and the
standard profile set is frozen behind an ADR; `EmulatedTransport` is parametric-only for now.

How the delay is applied, since it constrains anything built on top: `ITransport.Send` has no
future-delivery parameter and Core has no threads, so delay lives entirely on the receive side.
`TryReceive` drains the wrapped transport, stamps each drained datagram with
`innerArrivalTicks + delay` — the tick the wrapped transport *reported*, never the poll tick —
and holds it in a fixed-capacity min-heap until due. Consequences worth knowing:

- Impairment is **additive** over the wrapped transport, so wrapping a real socket gives real
  delay plus the profile, not the profile alone.
- Reordering needs no special case: delivery pops by earliest synthetic arrival.
- The jitter and reorder draws happen at drain time while the loss draw happens at send time, so
  the two interleave in the RNG stream according to the poll schedule. Deterministic for a fixed
  schedule; a *different* poll schedule with the same seed is a different realization.
- Every datagram consumes exactly one draw at send and two at drain regardless of the profile's
  values, so two profiles differing in one knob and sharing a seed keep common random numbers on
  the others.
- When `maxInFlight` is exhausted the wrapped transport is simply not drained (back-pressure);
  the emulator never destroys a datagram the profile did not say to lose.

## Codecs

`ICommandCodec` turns a `CommandFrame` into bytes. Genuinely underrated lever — the wire
format changes what mitigation is even possible downstream.

| Name | File | Idea |
|---|---|---|
| `raw` | `RawPoseCodec.cs` | baseline: instantaneous pose, uncompressed |
| `delta-quant` | `DeltaQuantizedCodec.cs` | delta against last acked + quantization |
| `trajectory` | `TrajectorySplineCodec.cs` | ~200 ms of *intended future motion* per frame |
| `redundant` | `NFrameRedundantCodec.cs` | N-frame redundancy; bandwidth for loss tolerance |

`trajectory` is the interesting one: sending intent rather than position means the robot always
has a plan to follow through a lost packet. It converts a latency problem into an
intent-transmission problem, and it interacts with the predictor — benchmark the pair.

Only `raw` is implemented so far (fixed 73-byte little-endian binary, `Pipeline/`'s Phase-4
zero-mitigation baseline codec — binary, not text like `Recording/`'s `.tlog`, because this
crosses a bounded datagram on the per-frame hot path and is never committed or diffed).
`delta-quant`, `trajectory`, `redundant` remain planned; their rows describe the design, not
the disk.

## Network profiles

Frozen in `core/testdata/traces/`. **Do not edit or add to the standard set** without an ADR;
changing the benchmark suite destroys comparability with every result already recorded.

`lan` · `50ms-5j` · `150ms-20j-0.5loss` · `300ms-60j-2loss-bursty` · `cellular-congested`
(trace) · `leo-satellite` (trace, periodic reconfiguration spikes) · `long-haul` (real capture)
