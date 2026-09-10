
---

## Hypothesis (written before any code)

**Mechanism.** A `MultipathTransport` `ITransport` decorator over N inner transports: `Send` puts
the datagram on every path; `TryReceive` returns whichever copy arrived first and suppresses the
later copies. Each path is an independently seeded `EmulatedTransport` carrying its own
Gilbert-Elliott loss chain, so a burst on path A does not coincide with a burst on path B.

**H1 (transport level).** Duplication over two *independent* paths reduces the loss rate seen by
the receiving endpoint from `π` to approximately `π²` and collapses the expected loss-burst length
towards 1. Metrics: **loss rate** and **loss burst-length distribution**, both `docs/metrics.md`
§3, **direction: down**. Neither is emitted by any component in Core today, so both are counted
directly in my harness — labelled harness-grade, not `results/`-grade.

**H2 (plant level — the one that actually matters).** Because a copy on path B arrives at *the
same step* at which path A's copy would have arrived had it not been lost, the plant receives, at
that step, a `CommandFrame` it would otherwise not have had. So the plant's pose trajectory under
multipath differs from single-path, and it is *closer* to the operator's commanded trajectory
during bursts. This is the claim that separates path diversity from payload redundancy, which is a
provable no-op because `RigidBodyPlant.Command` rejects any frame with
`CaptureTicks <= _lastAcceptedCaptureTicks` and `ICommandCodec.TryDecode` surfaces only the newest
frame in a datagram — so a *late* copy of an old frame can never change the trajectory.

**H3 (metric level).** H2 propagates to `prediction_position_error_mm` (§4) p95 **down**, with
`correction_magnitude_mm` (§5) p95 **not up**. Reported together, always, per §8 rule 2.
Baseline row present: predictor `none`, reconciler `snap`, playout `immediate`.

**Costs stated in advance, to be reported alongside any benefit, not after it.**
- 2× datagrams on the wire in each direction. `docs/metrics.md` §3 defines goodput as
  application-useful bytes/s **excluding redundancy**, so *goodput is unchanged by construction*
  and the entire cost lands outside it — the honest statement is "same goodput at twice the
  offered load", i.e. halved bytes-efficiency, and §3 has no metric that says so.
- Guaranteed reordering: two paths with different delay draws invert constantly. §3's reordering
  rate and max displacement are defined and emitted by nothing.
- Duplicate suppression costs O(K · maxPayloadBytes) of retained state.
- Two genuinely independent paths is an infrastructure claim this repo cannot make good on. It is
  the *best case*, not an estimate. Recorded for a human.

### Falsifiers, cheapest first. Each is run before the one after it.

| # | Falsifier | If it fires |
|---|---|---|
| **F1** | With independent per-path seeds, the plant's per-step pose trajectory under multipath is **identical** to single-path. | H2 is false: duplicate delivery on a second path changes nothing at step granularity, the payload-redundancy no-op generalizes to path diversity, and the candidate is closed. **This is a complete result and I stop there.** |
| **F2** | With *identical* per-path seeds (perfectly correlated paths), the trajectory is **not** identical to single-path. | My harness or my dedup is wrong — perfectly correlated duplication must be an exact no-op. Harness bug; halt and report, do not tune. |
| **F3** | Across N seeds, the p95 difference in `prediction_position_error_mm` between single-path and multipath lies inside the seed-to-seed spread of that percentile. | H3 is not resolvable at this trial length / impairment set. Negative result on the metric even if H1 and H2 hold at the transport and plant. §8 rule 3. |

F1 and F2 are deterministic single-seed structural tests and cost nothing. F3 needs N seeds.

---

## Feasibility assessment — the two changed facts, verified against source

### Fact 1: loss is now a two-state Gilbert-Elliott chain. **VERIFIED — TRUE.**

`core/Teleop.Core/Transport/Impairments/GilbertElliottLossImpairment.cs` holds
`_afterDelivered` / `_afterLost` and a `_previousWasLost` bit; `ApplyOnSend` picks
`_previousWasLost ? _afterLost : _afterDelivered` and makes exactly one draw per datagram. So loss
is temporally correlated whenever `r != p`. `ExpectedBurstLength => 1/(1-r)`.

`Transport/NetworkProfileCatalog.cs` confirms which profiles are actually bursty:

| Profile | `p` (after delivered) | `r` (after lost) | steady-state `π = p/(p+1-r)` | expected burst `1/(1-r)` |
|---|---|---|---|---|
| `300ms-60j-2loss-bursty` | 0.00612 | 0.70 | 2.00% | 3.33 |
| `150ms-20j-0.5loss` | 0.005 | 0.005 | 0.50% | 1.005 (degenerate → Bernoulli) |
| `lan`, `50ms-5j` | 0 | 0 | 0 | — |

So the brief's premise holds: correlated loss exists, and it is a different regime from the
bounded-uniform jitter the previous analysis closed the candidate on. **But the loss-burst
evidence is one profile wide**, exactly as the brief warned — the `loss-<N>pct` and `combo__`
families set `r = p`, i.e. memoryless.

`GilbertElliottLossImpairment` also runs at the **send** stage and drops *before* the wrapped
transport is asked. That matters for me: on a lost datagram the inner transport never sees the
bytes, so a per-path loss consumes no capacity, and multipath's per-path draws are independent
conditional only on their seeds.

### Fact 2: per-impairment RNG substreams derived from (trial seed, axis name). **VERIFIED — TRUE, and it cuts exactly as the brief warned.**

`Transport/Impairments/ImpairmentStreams.cs`: `DeriveForAxis(trialSeed, axisName) =
SplitMix64Finalizer(trialSeed + FNV1a64(axisName) * φ)`. `EmulatedTransport`'s constructor binds
each impairment with that. Both inputs are deterministic, and the axis names are fixed by the
impairment classes (`"loss"`, `"jitter"`, `"delay"`, `"reorder"`).

**Implication, stated plainly: two `EmulatedTransport` instances built with the same trial seed and
the same impairment set draw the same numbers in the same order.** Since both paths are handed the
same sequence of `Send` calls, their draw indices stay aligned, so they drop exactly the same
datagrams and add exactly the same delays. **Same-seed multipath is a provable no-op.** If I had
built two paths that way, I would have measured zero and concluded, wrongly, that path diversity
does nothing.

**Are distinct per-path seeds available, legitimate, and reproducible?**

- **Available:** yes. `seed` is an ordinary `ulong` constructor parameter of `EmulatedTransport`;
  nothing derives it from a global.
- **Legitimate:** yes, and it is the *established* idiom, not a hack. `SweepCommand.RunTrial`
  already builds the uplink with `seed` and the downlink with `unchecked(seed + 1)` for precisely
  this reason — "the two directions are decorrelated by the seed". A second path is another
  physically distinct link and gets the same treatment. `ImpairmentStreams.Derive`'s own doc calls
  out that adjacent trial seeds must produce uncorrelated substreams, because the sweep relies on
  it.
- **Reproducible:** yes. The seeds are a pure function of the trial seed (`seed`, `seed+2`,
  `seed+4`, ... in my harness so uplink/downlink offsets do not collide with path offsets), the
  hash is hand-rolled integer arithmetic explicitly chosen over `string.GetHashCode()` to survive
  .NET's per-process string-hash randomisation, and `Reset()` returns each substream to its bind
  seed.

**Decision, recorded as an assumption:** paths are seeded `seed + 2·k` for path index `k`, with the
downlink direction offset by `+1` as the sweep does. This models **statistically independent
paths, which is the best case and an upper bound on the benefit, not an estimate.** The repo has no
correlated two-path impairment model and adding one is `NetworkProfile`-shaped, i.e. an ADR, which
is out of my scope. To keep myself honest I run the *fully correlated* case as a control (identical
seeds), which brackets reality between "exact no-op" and "independent-path upper bound" — and which
doubles as falsifier F2.

### Contract check: does a multipath decorator satisfy `ITransport` unchanged?

Re-verified clause by clause against `Contracts/ITransport.cs` (the previous log did this for a
non-deduplicating variant; I dedup, which changes two rows):

| Clause | My decorator |
|---|---|
| "may be delayed, dropped, **duplicated**, or reordered" | Duplication is named as allowed channel behaviour. |
| `MaxPayloadBytes` | `min` over paths — every datagram must fit on every path to be duplicated on all of them. |
| `Send -> bool` | Send on all paths, return true iff **any** accepted. "False means will not be delivered" is preserved: false only when no path took it. |
| `TryReceive` "must never return the same datagram twice" | I **suppress duplicates**, so I satisfy this under *both* readings — the strict one (dedup required) and the previous log's permissive one (two copies are two datagrams). I did not want the reading to be load-bearing. |
| `arrivalTicks` is `t_recv`, not poll time | Propagated verbatim from the inner transport that produced the copy. Never overwritten with `nowTicks`. This is the named source of wrong numbers here. |
| "returned in arrival order" | One-slot lookahead per path, emit the minimum `arrivalTicks` among filled slots. |
| too-short `destination` → false, `byteCount` = required, datagram stays queued | Held in my own slot, so it survives the refusal. |
| `Reset()` — "a decorator resets the transport it wraps" | Resets every path, clears slots and the dedup ring. |
| no clock, no I/O, no threads, no allocation after construction | All slots, the heap-free slot table and the dedup ring are preallocated. |

**Verdict: decorator, no contract change.** Confirms the previous log's Q1.

### Registry: can `Func<int, int, ITransport>` express it? **No — and audit does not care.**

`Registries.Transports` is `Func<int maxPayloadBytes, int capacity, ITransport>`. A decorator over
N inner transports cannot be built from two ints; it is the same shape mismatch that keeps
`EmulatedTransport` out (`Registry/CLAUDE.md`). **So I add no registry line, per my brief.**

The gate consequence, checked before writing code rather than discovered by a failing gate:
`Teleop.Eval/Verification/AuditCommand.cs`'s `RegistryCompletenessAxes` lists `IPredictor`,
`IReconciler`, `ICommandCodec`, `IPlayoutPolicy`, `IAutonomyArbiter` — **`ITransport` is
deliberately excluded**, with a comment naming `EmulatedTransport` as the accepted gap. So a second
unregistered `ITransport` passes `audit` and does not weaken any gate.

**This is itself a finding.** `Transport/` now has three `ITransport` implementations and exactly
one of them is selectable by name. The axis has no by-name selection mechanism, `audit` has been
configured not to notice, and `SweepCommand` hardcodes its wiring — so *no transport work on this
axis is reachable from an experiment YAML at all*. Priced in "Left undone".

---

## What I built

| File | What it is |
|---|---|
| `core/Teleop.Core/Transport/MultipathTransport.cs` (+ `.cs.meta`, fresh guid `671d6fa517aa4b30bcf6000900c1610b`, checked for collision against the whole tree) | `ITransport` decorator over N inner transports. Sends on all; delivers the earliest-arriving copy; suppresses later copies by payload byte-equality against a preallocated ring. One lookahead slot per path so delivery is min-over-paths by `arrivalTicks` rather than round-robin. Allocation-free after construction, no clock, no I/O, no RNG of its own. |
| `core/Teleop.Core.Tests/Transport/MultipathTransportTests.cs` | 18 contract tests: `MaxPayloadBytes` = min over paths, aliased-path rejection, defensive clone, pass-through at one path, duplicate suppression, earliest-arrival delivery carrying that path's `t_recv`, cross-path arrival ordering, too-short-destination behaviour, `Send` true-iff-any, survival of a total outage on one path, `Reset` determinism, and two allocation assertions. |
| `core/Teleop.Core.Tests/Transport/MultipathLossHarnessTests.cs` | The measurement harness: falsifiers F1/F2, the §3 transport-level loss study, and the §4/§5 metric study over four impairment configurations x 20 seeds x 2000 steps. |

**No `Registries.cs` line.** `Func<int, int, ITransport>` cannot express a decorator over N
transports. `audit` excludes `Transports` from registry completeness, so this is an accepted gap,
not a hidden failure. **No `Transport/CLAUDE.md` row** either -- axis `CLAUDE.md` files are outside
my file scope this run. Both are deviations from invariant 9 and both are recorded here rather than
worked around; the human merging this should add the row.

## Results

All numbers below are **harness measurements**: no `manifest.json`, no SHA, not in `results/`, not
citable. They came from `dotnet test --filter FullyQualifiedName~MultipathLossHarnessTests
--logger "console;verbosity=detailed"`. Percentiles are linear-interpolation (pandas
`Series.quantile` default, matching `analysis/teleop_analysis/percentiles.py`).

Common to every arm: predictor `none`, reconciler `snap`, playout `immediate` -- the baseline row
(§8 rule 1). 20 seeds, 2000 steps at 10 ms, first 100 steps discarded from every metric population
in **both** arms (see "startup transient" below). The *only* thing that differs between the
single-path and multipath arms is the number of paths and their seeds.

### F2 (control) -- PASSED. Identically-seeded paths are an exact no-op.

`differing steps = 0` over 500 steps on `300ms-60j-2loss-bursty`, and identical accepted-command
counts. Two `EmulatedTransport`s built with the same trial seed and the same impairment set drop
exactly the same datagrams, so multipath over them changes nothing at all. **The harness is not
manufacturing a difference**, and the per-path seed decision is load-bearing exactly as predicted.

### F1 (the claim that separates path diversity from FEC) -- DID NOT FIRE.

500 steps, `300ms-60j-2loss-bursty`, seed 1000:

```
differing steps = 232 / 500      mean |delta position| = 17.6 mm
uplink datagrams reaching the plant:
  single    459  (accepted 168, rejected as stale 291)
  multipath 472  (accepted 218, rejected as stale 254)   of 500 submitted
```

**A second independent path does change the plant's trajectory at step granularity.** The
payload-redundancy no-op does *not* generalize to path diversity, and the reason is the one stated
in the hypothesis: a copy on path B arrives at the step at which path A's lost copy would have,
which is information delivered earlier, not merely delivered again. H2 holds.

### §3, transport level -- loss rate and burst-length distribution

Frozen `300ms-60j-2loss-bursty` parameters, 20 000 datagrams x 10 seeds, no pipeline in the way.

| §3 metric | single path | two independent paths |
|---|---|---|
| **loss rate** p50 | 2.0500% | **0.0475%** |
| loss rate p95 | 2.2960% | 0.0905% |
| **loss burst length** -- events | 1184 | **50** |
| burst length mean | 3.436 | 2.060 |
| burst length max | 30 | 8 |
| burst histogram | `{1:327, 2:263, 3:177, 4:128, 5:83, 6:57, 7:45, 8:35, 9:22, 10:14, 11:6, 12:8, 13:4, 14:4, 15:4, 17:1, 19:2, 20:1, 22:1, 25:1, 30:1}` | `{1:22, 2:14, 3:8, 4:4, 5:1, 8:1}` |

A **43x loss-rate reduction** and a burst distribution with the long tail cut off at 8 instead of
30. This is an empirical confirmation of the previous log's purely analytic prediction (pi-squared
= 0.04%, joint burst ~ 1.96) -- arrived at independently, by counting, and it lands within a few
percent. Both metrics are §3 definitions used unchanged; neither is emitted by any component, so
both were counted in the harness.

### §4 and §5 -- the four configurations, and what they attribute to what

`prediction_position_error_mm` (§4) and `correction_magnitude_mm` (§5), always together (§8 rule 2).

#### (a) LOSS ONLY -- GE(0.00612, 0.7), 50 ms fixed delay, **zero jitter**. *Not in the frozen suite.*

The clean arm. Zero jitter makes both copies of a send arrive at the same tick, so min-over-paths
is a no-op on delay and the only thing diversity can do is recover a lost datagram. 50 ms keeps the
round trip well inside the 64-slot in-flight ring.

| | single path | two independent paths |
|---|---|---|
| `prediction_position_error_mm` p50 | 19.14 | 19.15 |
| p90 | 25.97 | 25.78 |
| p95 | 26.76 | 26.43 |
| **p99** | **37.41** | **27.02** |
| max | 93.66 | 42.39 |
| per-seed p95: median / min / max / spread | 26.73 / 26.47 / 26.92 / **0.45** | 26.43 / 26.42 / 26.44 / 0.01 |
| per-seed p99: median / min / max / spread | 35.88 / **27.11** / 46.27 / 19.16 | 27.00 / 27.00 / **27.06** / 0.06 |
| `correction_magnitude_mm` n | 7269 | 7296 (ratio 1.00, **comparable**) |
| `correction_magnitude_mm` p50 | 5.17 | 5.16 |
| p90 | 5.41 | 5.38 |
| **p95** | **7.87** | **5.41** |
| **p99** | **25.66** | **5.42** |
| max | 72.50 | 21.12 |
| uplink delivered to plant | 39 013 (2.467% missing) | 39 872 (0.320% missing) |
| rejected as stale | 0 | 0 |

**This is the finding, and it is a change in the shape of a tradeoff rather than a few percent.**

- **The median does not move at all** (19.14 -> 19.15 mm, and 5.17 -> 5.16 mm). p95 moves by 0.33 mm
  against a single-path per-seed p95 spread of 0.45 mm -- **inside run-to-run variance, so F3 fires
  at p95**: §8 rule 3 says that is not a result and I am not reporting it as one.
- **p99 moves by 28%, and it is cleanly resolvable.** Multipath's *worst* per-seed p99 (27.06) is
  below single-path's *best* per-seed p99 (27.11) -- the two distributions of per-seed p99 do not
  overlap across 20 seeds. There is no seed on which the baseline wins.
- **Correction cost falls with it, hard**: p95 -31%, p99 -79%, on populations of equal size (7269
  vs 7296, ratio 1.00). This is the unusual part. Prediction error and correction cost normally
  trade against each other, and the whole reason `docs/metrics.md` insists they be reported
  together is to catch a predictor that buys error with micro-corrections. Here they move the same
  way, because path diversity is not a prediction-aggressiveness knob -- it is an
  information-availability knob, and having the frame at all is better on both axes at once.
- **Multipath's p99 (27.02) is essentially its own p95 (26.43): the distribution has no tail
  left.** The single-path tail is entirely loss-gap staleness, and it is gone.

Physical sanity check, because a 79% p99 improvement deserves one before it is believed. The
synthetic operator moves at up to ~0.54 m/s (0.5·sin t plus 0.3·cos 0.7t), i.e. 5.4 mm per
10 ms step. At 50 ms delay the steady-state error should be ~27 mm -- observed p95 is 26.76. A lost
burst of *k* datagrams adds k x 5.4 mm; a 3-datagram burst predicts ~43 mm, and observed
single-path p99 is 37.4 with a max of 93.7 (a ~12-long burst, which the §3 histogram says happens).
Every number is where the arithmetic says it should be, and the multipath max of 42.4 mm is the
residual double-loss. Not an artifact.

#### (b) JITTER ONLY, attribution control -- 300 ms, +/-60 ms, **zero loss**

| | single path | two independent paths |
|---|---|---|
| `prediction_position_error_mm` p50 | 127.04 | 103.88 |
| p95 | 244.27 | 158.50 |
| p99 | 286.89 | 233.02 |
| uplink delivered / accepted / **rejected as stale** | 39 381 / 14 717 / **24 664** | 39 418 / 18 652 / 20 766 |
| `correction_magnitude_mm` n | 3658 | 8367 (ratio 2.29, **POPULATIONS DIFFER**) |

#### (c) `300ms-60j-2loss-bursty`, the frozen suite's only bursty-loss profile -- loss and jitter **confounded**

| | single path | two independent paths |
|---|---|---|
| `prediction_position_error_mm` p50 | 126.99 | 104.38 |
| p95 | 245.70 | 160.73 |
| p99 | 284.70 | 238.57 |
| uplink delivered / accepted / rejected as stale | 38 500 / 14 490 / 24 010 | 39 398 / 18 504 / 20 894 |
| `correction_magnitude_mm` n | 3605 | 8268 (ratio 2.29, **POPULATIONS DIFFER**) |

**Put (b) and (c) side by side and the transferable claim dies.** Turning the loss chain off
changes the single-path p95 from 245.70 to 244.27 -- 1.4 mm, less than the per-seed spread. The
multipath improvement is 35% with loss and 35% without it. **On the frozen suite's only
bursty-loss profile, 2% bursty loss is invisible in `prediction_position_error_mm`, and
essentially all of multipath's apparent benefit there is the min-of-two-paths *jitter* effect --
a different axis, already analysed and closed.** Reporting (c) as a loss result would have been
the single easiest way to flatter this candidate, and it would have been wrong.

Why loss is invisible there: **62% of everything that arrives is rejected by the plant as stale**
(24 664 of 39 381) purely from +/-60 ms jitter reordering against a 10 ms cadence. Against that,
losing another 2% of datagrams is noise. The `correction_magnitude_mm` comparison on (b) and (c) is
**not usable at all** -- multipath accepts 27% more commands, so the plant moves more, so more
disagreements are registered and the sample count changes by 2.29x. Those percentiles compare
populations, not algorithms, and the harness prints that warning itself rather than leaving it to
be noticed.

#### (d) HETEROGENEOUS paths, pricing the reordering cost -- A = 100 ms +/-40 ms vs B = 160 ms +/-40 ms, zero loss

| | single path (A alone) | A + B |
|---|---|---|
| `prediction_position_error_mm` p50 / p95 / p99 | 30.41 / 59.20 / 70.28 | 30.41 / 59.18 / 70.12 |
| accepted / **rejected as stale** | 18 587 / **21 209** | 18 620 / **21 176** |

**A near-exact no-op, and it confirms the previous log's prediction empirically:** with supports
[60,140] and [120,200] overlapping only in [120,140], min-over-paths is almost always path A, and
the scheme degenerates to "use the faster path" for twice the bytes.

### The cost I predicted and did not find

I stated in advance that this decorator would manufacture reordering, and the brief warned the same.
**That was wrong, and I am recording it as wrong.** Stale rejections at the plant went *down* in
every configuration measured: 24 664 -> 20 766 (jitter-only), 24 010 -> 20 894 (frozen bursty),
21 209 -> 21 176 (heterogeneous). The reason is elementary once seen: the minimum of two draws from
the same distribution has *smaller* spread than one draw, so identically-distributed paths reduce
arrival-order inversion rather than causing it. Reordering could only increase for paths whose
delay supports overlap partially -- and configuration (d), built to find exactly that, degenerates
before it can. I could not construct a configuration in which this decorator adds reordering.

### Costs that are real and stand

- **2x datagrams in each direction.** §3 defines goodput as excluding redundancy, so goodput is
  *unchanged by construction* and the entire cost falls outside every metric this project has. The
  honest phrasing is "the same goodput at twice the offered load"; there is no metric that says it.
- **Independence is an assumption, not a measurement.** Everything above is the best case. Real
  second paths share last miles and backhaul; under perfect correlation the whole effect is exactly
  zero, which is what F2 measures. Reality is between 43x and 1x, and this repo has no correlated
  two-path impairment model to say where.
- **Suppression state**: 64 x `MaxPayloadBytes` bytes, plus a linear byte-comparison scan per
  delivered datagram. Allocation-free but not free.
- **Byte-equality dedup is exact for this workload and wrong in general** -- every `RawPoseCodec`
  frame carries a distinct `CaptureTicks`, so two byte-identical datagrams can only be copies. An
  application that legitimately repeats a payload would lose the repeat.

## Prior work consulted (candidate generation only -- never evidence about this system)

- **"Exploiting the Path Propagation Time Differences in Multipath Transmission with FEC" (Kurant,
  arXiv:0901.1479)** -- https://arxiv.org/abs/0901.1479. Proposes *Spread*, scheduling FEC packets
  across paths to exploit propagation-time differences; claims a **two- to five-fold reduction in
  effective loss rate** over state-of-the-art multipath FEC, evaluated analytically and by
  trace-driven simulation, with the stated operating point that path propagation times "typically
  differ by 10-100 ms". Two things it gave me: it independently supports my heterogeneous arm
  (d)'s 60 ms difference being realistic rather than contrived, and it makes the overhead point
  sharply -- their 2-5x comes from *spreading FEC*, at far less than 2x bytes, whereas the 43x I
  measured is *full duplication* at exactly 2x bytes. Their scheme is the better point on the
  overhead curve and this repo cannot express it: `ICommandCodec.TryDecode` returns only the newest
  frame in a datagram, so intra-datagram coding has nowhere to land.
- **"Online multipath convolutional coding for real-time transmission" (arXiv:1204.1428)** --
  https://arxiv.org/pdf/1204.1428. Same family; not read past the abstract, since coding schemes
  are closed here (payload redundancy/FEC was rejected as a provable no-op against `TryDecode`
  plus `RigidBodyPlant`'s monotonic filter, and that argument kills convolutional coding across
  datagrams too).
- **Search that found nothing useful:** I looked for prior work specifically on *duplication*
  (not coding) across paths under Gilbert-Elliott loss for closed-loop *control* rather than media
  streaming, and found only media/streaming work and generic GE-model references. The distinctive
  thing about this system -- that a stale frame is discarded by the plant, so redundancy only helps
  when it arrives *earlier* -- does not appear in the literature I could reach. Recorded so the
  next run does not repeat the search.

## Verification

All three gates plus `bridge-check`, on the full tree, after the last code change.

```
$ cd core && dotnet test
Passed!  - Failed: 0, Passed: 666, Total: 666  (Teleop.Core.Tests)
Passed!  - Failed: 0, Passed:  36, Total:  36  (Teleop.RobotHost.Tests)
Passed!  - Failed: 0, Passed:  28, Total:  28  (Teleop.RobotArm.Tests)
Passed!  - Failed: 0, Passed:   3, Total:   3  (Teleop.Eval.Tests)
   (run three times consecutively, 666/666 each time)

$ dotnet run --project Teleop.Eval -- verify
verify: PASS -- basic-session.tlog replays byte-identical across two independent passes,
                and matches the original file exactly.                        (exit 0)

$ dotnet run --project Teleop.Eval -- audit
audit: PASS -- no invariant violations found.                                 (exit 0)

$ cd unity/BridgeCheck && dotnet build
0 Errors, 1 pre-existing warning (CS8632 in JetRoverOperatorBridge.cs, not mine)
```

### A flaky gate I hit, did not cause, and did not touch

My first full-suite run failed 2 of 666 with
`EasedExponentialReconcilerTests.Reconcile_WhileConverged_Allocates_Zero_Bytes` -- *"7760 bytes
allocated (0.776 bytes/call)"* -- and `MotionMathTests.ToRotationVector_Allocates_Zero_Bytes`.
Fractional bytes per call is impossible for a real allocation (the smallest object is 24 bytes),
and it is precisely the tiering artifact `TestSupport/AllocationAssert.cs`'s own doc comment
describes having been fixed once already.

I checked whether I caused it instead of assuming I had not. **Moving both of my test files out of
the tree entirely and re-running three times still failed 1-2 allocation tests per run** (0.736,
0.721, 0.638 bytes/call, a different test each time). It is pre-existing at `76b2a14` on this box.
With my files restored, three consecutive full runs then passed 666/666, and all 81 allocation
tests including my two pass in isolation. The most likely cause is CPU contention -- three
researchers are running on this box concurrently this session, and contention is exactly what makes
tier-1 promotion land inside the measured window.

**I changed nothing about it.** Weakening an allocation assertion to make a suite green is the
worst available move. Recorded for a human: the allocation gate is load-sensitive on this machine
and will fail intermittently in any parallel or shared-runner setting, which makes it unreliable in
exactly the situation where a gate matters.

## How this result could flatter itself

Six ways, listed because I would rather find them than have a reader find them.

1. **Independent paths are the best case and I cannot bound how far the real case is from it.**
   F2 measures the other extreme: perfectly correlated paths are an exact zero. The truth is
   somewhere in between and this repo has no model that can say where. **If a reader takes one
   caveat from this log, take this one.**
2. **The clean arm (a) is not in the frozen suite, and the frozen suite's arm (c) shows nothing
   attributable to loss.** Configuration (a) is where the result lives, and it is a profile I
   constructed. Nothing here transfers to the benchmark until the suite has a bursty-loss profile
   whose jitter does not swamp it. That is a real limit on the claim, not a formality.
3. **The startup transient nearly produced a fake p99.** Before I trimmed it, p99 on the 300 ms
   profiles read 1300.18 mm -- which is the initial operator-to-plant offset, not the network. At
   20 seeds x 2000 steps a 35-step transient is 1.75% of the pooled population, i.e. it sits above
   p98. I trim 100 steps identically in every arm; anyone re-running with a different trim will get
   different p99s on those profiles, and any *existing* result in this repo that pools from step 0
   on a 300 ms profile has the same contamination.
4. **`correction_magnitude_mm` is only comparable in arms (a) and (d).** In (b) and (c) multipath
   accepts 27% more commands, so the sample count changes by 2.29x and the percentiles compare
   populations. I print the ratio and a warning rather than quietly reporting the improvement -- it
   would have looked like a -79% correction-cost win.
5. **`owd_uplink_ms` is survivorship-biased and I deliberately do not report it.** It is emitted
   only when a reply is matched to an in-flight trace, so a lost packet emits nothing -- and my
   candidate changes exactly which packets survive. On (c) the sample counts are 24 937 vs 35 952
   for the same 40 000 submissions: pooling those percentiles would compare two different
   populations and would show multipath "winning" partly by resurrecting slow round trips that
   the baseline never sampled. The counts are printed so the trap is visible.
6. **`OperatorEndpoint.InsertInFlight` overwrites an occupied ring slot without checking**, and on
   the 300 ms profiles my harness is exposed to it: `InFlightCapacity = 64` at 10 ms is a 640 ms
   window against a 480-720 ms round trip. Quantified rather than assumed: on (b), zero-loss by
   construction, only 26 040 of 40 000 submissions produced an `owd_uplink_ms` sample -- **35% of
   round trips are silently lost to ring eviction with no network loss at all**, and the loss is
   delay-correlated, so every recorded p95/p99 for `owd_*` on a 300 ms profile in this repo is
   biased low. This is a defect in the measurement, not in any algorithm; I did not fix it (a human
   owns the comparability question) and configuration (a) at 50 ms avoids it, which is one more
   reason (a) is the arm to trust.

## Left undone / for a human

1. **The `Transport/CLAUDE.md` "Implemented" row and the axis's `Registries.cs` story.** Outside my
   file scope. `MultipathTransport` needs a row; more importantly, `Transport/` now has three
   `ITransport` implementations of which exactly one is selectable by name, `audit` is configured
   not to notice, and `SweepCommand` hardcodes its wiring -- **no transport candidate on this axis
   can be selected from an `experiments/*.yaml` at all.**
2. **Price of wiring a transport axis into the sweep, since that is a deliverable.** Four changes,
   none large, one of them an ADR: (i) a transport factory shape that can express a decorator --
   either a second dictionary keyed to a small builder type, or a `NetworkProfileCatalog`-style
   by-name catalog, which `Registry/CLAUDE.md` already argues is the better analogue; (ii) a
   `transport:` (and paths / per-path seed) axis in the experiment schema and `ExperimentConfig`,
   mirroring how `stacks` was added for reconcilers; (iii) `SweepCommand.RunTrial`'s hardcoded
   `new EmulatedTransport(...)` pair replaced by a lookup -- `Pipeline/CLAUDE.md`'s standing
   instruction is that a hardcoded line gets *replaced*, not wrapped; (iv) an ADR, because
   per-path seeding is a reproducibility contract and the manifest has to record it or the run is
   not reproducible. Estimate: a day, of which the ADR is the slow part. **Nothing here is blocked
   on Core.**
3. **§3's network metrics are still emitted by nothing.** Loss rate, burst-length distribution,
   reordering rate/displacement and goodput are all defined in `docs/metrics.md` and emitted by no
   component, which is why the biggest effect this candidate has (43x loss rate) had to be counted
   in a test rather than read from a sink. I may not edit `docs/metrics.md` and did not. The
   blocker is not the definitions -- it is that `ITransport` has no `IMetricSink` and no
   `Diagnostics` property, so a transport has nowhere to emit. That is a `Contracts/` change and
   therefore an ADR.
4. **A correlated two-path impairment model.** The single largest fidelity gap. Needs at minimum a
   shared-loss-event component and a shared-delay component between paths. `NetworkProfile`-shaped,
   so an ADR. Until it exists, every multipath number in this repo is an upper bound.
5. **A bursty-loss profile whose jitter does not swamp the loss.** Result (c) shows the frozen
   suite cannot currently measure *any* loss-mitigation candidate: on its only bursty-loss profile,
   2% bursty loss moves `prediction_position_error_mm` p95 by 1.4 mm, inside seed spread, because
   +/-60 ms jitter already causes 62% stale rejection. Something like `50ms-0j-2loss-bursty` would
   make loss work measurable. Frozen suite, so an ADR, and not mine.
6. **The `InsertInFlight` silent-overwrite defect** (item 6 above), now quantified: 35% of round
   trips unsampled on a 300 ms zero-loss profile. Not mine to fix, and it affects the
   interpretation of existing recorded results, not just future ones.
7. **A correction to a closed result, offered without reopening it.** The jitter half of path
   diversity was closed on an OWD-percentile argument: min-of-two buys 0.414·J at p50 and 0.180·J
   at p99, shrinking into the tail. That arithmetic is right, and I did not re-derive it. But arm
   (b) shows the *mechanism it did not model* is much larger: min-of-two also **narrows the
   arrival-time distribution**, which cuts the plant's stale-frame rejection from 62% to 53% and
   moves `prediction_position_error_mm` p95 by 35% -- an order of magnitude more than the
   delivered-delay effect alone predicts. The closed verdict is about a different quantity than the
   one that turned out to matter. A human should decide whether that is worth reopening; I have
   not, and no work of mine depends on it.
8. **Blocked on a human -- real two-path hardware.** Two NICs, Wi-Fi plus cellular, sockets,
   `Bridge/UdpTransport.cs`, `unity/`, the Windows box. Off-limits from here by rule, not
   attempted, and the only way to replace assumption 1 with a measurement.

## Verdict

**Built, and the loss half of path diversity is real but narrow.** Duplication over two independent
paths cuts §3 loss rate 43x and the loss-burst tail from 30 to 8; at the plant that shows up as **no
change at the median and a collapsed tail** -- `prediction_position_error_mm` p99 -28% and
`correction_magnitude_mm` p99 -79%, both moving the same direction, on comparable populations, with
per-seed p99 distributions that do not overlap across 20 seeds. p95 is inside seed spread and is
not a result.

**But the benefit is invisible on the frozen benchmark**, where +/-60 ms jitter causes 62% stale
rejection and 2% bursty loss cannot be seen at all; and it rests entirely on an independence
assumption this repo cannot check, whose opposite extreme (F2) is an exact zero.
