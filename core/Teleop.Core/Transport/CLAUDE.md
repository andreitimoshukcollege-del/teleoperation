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
| emulated | `EmulatedTransport.cs` | decorator: applies a caller-supplied set of `INetworkImpairment` objects, each with its own seeded substream |

`loopback` has a `Registry/Registries.cs` entry (`Transports["loopback"]`). `EmulatedTransport`
deliberately does not: it is a decorator over another `ITransport` plus an impairment set and a
seed, a materially different constructor shape than `LoopbackTransport`'s
`(maxPayloadBytes, capacity)` -- see `Registry/CLAUDE.md` for the full reasoning. It needs an
entry (its own shape, or a small builder type) the moment a sweep needs to select a transport by
name rather than wiring one directly, as every current test does.

## EmulatedTransport

A **decorator** — it wraps any `ITransport` and applies a **set of impairments the caller supplies**
(`docs/adr/0013-composable-network-impairments.md`). Most of the research runs through it, including
in Unity: wrap `UdpTransport` on a LAN and you get a reproducible impairment on a real socket.

It holds no impairment parameters of its own and knows nothing about profile names. Adding a new
kind of impairment is a new file in `Impairments/` implementing `Contracts/INetworkImpairment.cs` —
not a widening of a struct plus an edit at every construction site.

Requirements:
- All impairment driven by injected seeded RNG. Same seed + same set + same input => same output.
- Trace replay is sample by sample, no resampling (`TraceDelayImpairment`).
- Loss must model **bursts**, not just a Bernoulli rate. Real links lose runs of packets, and
  burst length is what breaks jitter buffers (`GilbertElliottLossImpairment`).

All three are met.

### The impairment set

`Impairments/`: `FixedDelay` (0 draws/datagram), `UniformJitter` (1), `GilbertElliottLoss` (1, send
stage), `Reorder` (1), `TraceDelay` (0). Each owns its parameters, validates them in its own
constructor, and owns one RNG substream.

**Two stages, because a datagram has two.** Loss is decided in `Send`, so a lost datagram never
reaches the wrapped transport — a real link does not put a dropped packet on the wire, and
`ITransport.Send` returning false is contractual. Delay is applied at drain time, because `Send` has
no future-delivery parameter and Core has no threads. An impairment declares its stages and is never
called at one it did not declare.

**Composition is order-independent**, enforced by two rules on `DatagramFate`: delay is additive
(`+=`, never `=`) and `Dropped` is monotone. Permuting the array is therefore not a configuration
change. An order-sensitive impairment would make order part of the configuration and must say so.

**The trace/parametric contradiction is gone.** There is no trace "mode" and no cross-field
validation. "A trace supersedes delay and jitter" is simply which impairments the caller included;
including both is legal and means a recorded link plus an extra fixed hop, with the delays summing.

### Determinism — read this before touching the RNG

The old contract was "exactly 3 draws per datagram from one shared stream, in a fixed order", with
jitter and reorder drawn even at zero so that profiles sharing a seed stayed aligned. That is gone,
because it cannot survive composition: removing an axis removes a draw and shifts every subsequent
one. The replacement:

> **Stream isolation.** Each impairment draws only from its own substream, derived from the trial
> seed and its axis name, and consumes a constant number of draws per datagram per stage — fixed by
> its type, independent of its parameter values, including at neutral values.

That buys strictly more than it replaces. Changing an axis's magnitude still leaves others
untouched; now so does **adding or removing** one, and a neutral axis is observationally identical
to its absence. The last property is what lets the catalog emit only non-zero axes while
reproducing ADR 0004's frozen numbers exactly.

Substream seeds come from a hand-rolled FNV-1a over the axis name — **never `string.GetHashCode()`**,
which .NET Core randomises per process and would silently destroy cross-run reproducibility while
every in-process test still passed. `ImpairmentStreamsTests` pins the derived values to hardcoded
literals precisely so a rename, a hash change, or an accidental `GetHashCode` fails loudly.

Other consequences worth knowing:

- Impairment is **additive** over the wrapped transport, so wrapping a real socket gives real delay
  plus the set, not the set alone.
- Reordering needs no special case: delivery pops by earliest synthetic arrival.
- Loss draws at send while delay draws at drain, so the two interleave according to the poll
  schedule. Deterministic for a fixed schedule; a *different* poll schedule with the same seed is a
  different realization.
- When `maxInFlight` is exhausted the wrapped transport is simply not drained (back-pressure); the
  emulator never destroys a datagram no impairment asked to lose.
- `audit` checks every `INetworkImpairment` implementation is constructed by name in
  `NetworkProfileCatalog.cs`. That is this axis's substitute for a `Registries.cs` entry, and it
  doubles as an IL2CPP reachability check.

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

Per `docs/adr/0004-network-profile-suite.md`: the four parametric names plus a new
`synthetic-burst` trace-driven profile are implemented, with exact parametric values recorded
there. `cellular-congested`/`leo-satellite`/`long-haul` remain reserved names, honestly
unimplemented pending an actual network capture — not faked. Resolution by name lives in
`core/Teleop.Eval/Sweep/NetworkProfileCatalog.cs`, not here (loading a trace file is I/O).

Extended by `docs/adr/0005-isolated-impairment-profiles.md`: `jitter-<N>ms` / `delay-<N>ms` /
`loss-<N>pct`, each isolating one `NetworkProfile` parameter with the other two held fixed —
answers "how sensitive is this to one variable," not "does it survive a realistic bad link,"
which is what the frozen five above are for. Resolved by pattern
(`NetworkProfileCatalog.TryResolveIsolatedAxisProfile`), not one named case per point; that ADR's
own rule governs adding more points to an existing family, and a new ADR is still required for a
genuinely new family (different fixed companions, or reintroducing burst shape on the loss axis).

Extended again by `docs/adr/0006-combined-impairment-profiles.md`: `combo__delay-<N>ms__jitter-
<N>ms__loss-<N>pct`, any 2-or-3-axis subset, values chosen by the caller rather than isolated
against a fixed companion — answers "what happens as the whole link degrades at once," a third
question distinct from both families above. An axis absent from the name is 0, not a baseline,
unlike the isolated family. Resolved by pattern (`NetworkProfileCatalog.TryResolveCombinedProfile`);
`analysis/experiment_builder.py`'s `combined_points` generates these names as a **lockstep**
walk (point *i* takes the i-th value of every checked axis) rather than a cross product, so the
whole sweep can be plotted as one line chart (`analysis/teleop_analysis/figures/combined_response.py`).
