# Transport

`ITransport` implementations, `ICommandCodec` implementations, and the network emulator.

Core contains **no real I/O.** The transports here are in-process: loopback, and the emulator
that wraps another transport. Sockets live in `Bridge/UdpTransport.cs` because I/O is a host
concern, not because `System.Net` is unavailable (it is available under IL2CPP). Keeping Core
I/O-free is what makes replay bit-deterministic.

## EmulatedTransport

A **decorator** — it wraps any `ITransport` and injects delay, jitter, loss, and reordering
from a `NetworkProfile` or a captured trace. Most of the research runs through it, including
in Unity: wrap `UdpTransport` on a LAN and you get a reproducible impairment on a real socket.

Requirements:
- All impairment driven by the injected seeded RNG. Same seed + same input => same output.
- Trace-driven mode replays recorded one-way delays sample by sample, no resampling.
- Loss must model **bursts**, not just a Bernoulli rate. Real links lose runs of packets, and
  burst length is what breaks jitter buffers.

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

## Network profiles

Frozen in `core/testdata/traces/`. **Do not edit or add to the standard set** without an ADR;
changing the benchmark suite destroys comparability with every result already recorded.

`lan` · `50ms-5j` · `150ms-20j-0.5loss` · `300ms-60j-2loss-bursty` · `cellular-congested`
(trace) · `leo-satellite` (trace, periodic reconfiguration spikes) · `long-haul` (real capture)
