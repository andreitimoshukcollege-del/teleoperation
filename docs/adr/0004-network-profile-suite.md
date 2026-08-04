# 4. The network profile suite: trace format, catalog, and the five profiles for Gate 5

## Status

Accepted.

## Context

Gate 5 (`docs/setup.md`) needs "a p50/p95/p99 table across five network profiles." Before this
ADR, none of the supporting infrastructure existed: `Types/NetworkProfile.cs` has no named-preset
concept (purely parametric fields), `EmulatedTransport` was parametric-only despite
`Transport/CLAUDE.md` already describing a trace-driven mode as the intended design, and the
"frozen" 7-name profile set (`lan`, `50ms-5j`, `150ms-20j-0.5loss`, `300ms-60j-2loss-bursty`,
`cellular-congested`, `leo-satellite`, `long-haul`) existed only as prose, naming things that had
never been built. Three of those seven names — `cellular-congested`, `leo-satellite`,
`long-haul` — specifically imply a real network capture, which no agent working on this repo can
produce.

## Decision

### Five profiles, honestly

Gate 5 needs five profiles, not seven. This pass implements:
- The four parametric profiles from the frozen set, as literal `NetworkProfile` values.
- One new profile, `synthetic-burst`, trace-driven, demonstrating the trace mechanism works.

`cellular-congested`, `leo-satellite`, and `long-haul` remain reserved names in the frozen set,
explicitly **not implemented and not faked**. `Sweep/NetworkProfileCatalog.cs` reports them as a
distinct "reserved, pending a real capture" failure, never silently as "unknown profile" — the
distinction matters because the fix for one is "learn the correct name" and the fix for the other
is "go measure a real link," and conflating them would waste someone's time on the wrong one.
Someone with the actual capture hardware can complete the set later.

### The four parametric profiles' exact values

Recorded here so the numbers in a manifest and the numbers in this document never drift apart:

| Name | Base delay | Jitter (±) | Loss (after-delivered / after-lost) | Notes |
|---|---|---|---|---|
| `lan` | 2 ms | 1 ms | 0 / 0 | A good but real link, not `LoopbackTransport`'s literal zero |
| `50ms-5j` | 50 ms | 5 ms | 0 / 0 | |
| `150ms-20j-0.5loss` | 150 ms | 20 ms | 0.5% / 0.5% | Equal probabilities degenerate the Gilbert-Elliott chain to plain Bernoulli loss — the name has no "bursty" qualifier, so it shouldn't have burst behavior |
| `300ms-60j-2loss-bursty` | 300 ms | 60 ms | 0.612% / 70% | Tuned so steady-state loss ≈ 2% (`p/(p+(1-0.7)) = 0.02` ⟹ `p ≈ 0.00612`) with expected burst length `1/(1-0.7) ≈ 3.33` |

None carry reordering — none of the four names mention it, so none is added silently.

### Trace file format, new

A recorded one-way-delay trace (`core/testdata/traces/*.trace`, `Sweep/TraceFile.cs`): a header
line `TRACE|<version>|<ticksPerSecond>`, then one non-negative tick integer per line, one sample
per datagram, consumed in order. Text, not binary — the same "research data is text, diff it"
reasoning `Recording/RecordFormat.cs` gives for `.tlog` — but a dedicated minimal format rather
than reusing `.tlog`'s spec wholesale: a delay trace has none of `.tlog`'s need for multiple
record types or a running checksum.

Trace files are generated deterministically via `Teleop.Eval -- gen-trace`
(`Tooling/SyntheticTraceBuilder.cs` for `synthetic-burst`), the same discipline `gen-golden`
already established for the golden `.tlog` fixture: never hand-author committed research data.

### `EmulatedTransport` trace-driven mode

A new constructor overload (`core/Teleop.Core/Transport/EmulatedTransport.cs`) takes a delay-tick
array in place of `NetworkProfile.BaseDelayTicks`/`JitterTicks` (which must both be zero in this
mode — the trace already represents the true recorded delay, so synthetic jitter on top would
double-model variance). Burst loss and reordering remain independent, profile-driven knobs in
both modes. The trace wraps back to its start when exhausted rather than resampling or throwing:
a sweep trial may run longer than the trace itself, and repeating the recorded sequence verbatim
is what "no resampling" (`Transport/CLAUDE.md`) actually means. Full reasoning and test coverage
in that file and `EmulatedTransportTests.cs`.

### `synthetic-burst`: what it's for and why it isn't pretending to be real

`SyntheticTraceBuilder` generates a bimodal delay pattern — a tight low-delay baseline punctuated
by periodic multi-sample congestion bursts at clearly elevated delay — a shape no combination of
`BaseDelayTicks` + uniform jitter can express. Its purpose is narrowly to prove the trace-driven
mechanism actually changes measured behavior versus the parametric profiles, not to model any
particular real network. Its name says exactly that.

### Named profile catalog lives in `Teleop.Eval`, not Core

`Sweep/NetworkProfileCatalog.cs` resolves a name to either a `NetworkProfile` literal or a loaded
trace. It is deliberately not a `Registry/Registries.cs` table: loading a trace file is I/O, which
is a host concern, and `NetworkProfile` is not a `Contracts/` interface implementation the way
everything else in `Registries.cs` is.

## Consequences

- Closing the rest of the frozen 7-name set (`cellular-congested`, `leo-satellite`, `long-haul`)
  requires an actual network capture. That work is out of scope for any agent and stays reserved.
- The four parametric profiles' exact numeric values are now a citable, committed decision, not
  arbitrary per-run choices — changing them later needs its own ADR, per `Transport/CLAUDE.md`'s
  "do not edit the standard set without an ADR" rule, which this document is itself an instance of.
- `synthetic-burst` should not be cited as evidence about any real link's behavior — only as
  evidence the trace-driven mechanism works.
