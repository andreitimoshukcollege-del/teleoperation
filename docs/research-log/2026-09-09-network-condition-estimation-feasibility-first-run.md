# Network condition estimation and adaptation — feasibility assessment (first run)

**Agent:** candidate 3 of 4, Transport-axis survey run — **first attempt**.
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a1b3e14c5d640e8e1` (deleted).
**Branch:** `worktree-agent-a1b3e14c5d640e8e1` (deleted).
**HEAD at start:** `06c0ab5b6f570efa028002897c856c566041663b`.

## Provenance — read this first

**This log was reconstructed by the research-organizer from the researcher's final report message,
not copied from its worktree, because no file ever existed to copy.** The researcher hit a silent
failure where a heredoc write reported success but left no file, and then — while it was still
running — the organizer wrongly judged it dead (its transcript had been silent for 90 minutes and
its worktree held only a scratch file) and removed its worktree as part of end-of-run cleanup. It
lost filesystem access and could not write its log. Its entire deliverable arrived as its return
message and is reproduced below essentially verbatim. **The organizer's cleanup error is recorded
in `2026-09-09-network-transport-survey-decisions.md`.**

There are **two** logs for this one candidate, both legitimate and independent:

- this one — the first attempt, which ran to completion and reached a bottom line;
- `2026-09-09-network-condition-estimation-feasibility.md` — a relaunch the organizer started when
  it believed this attempt had died, which was itself truncated after section 4.

They agree on the contract determination and on the inelastic-sender answer, and were produced
without knowledge of each other. Where they differ in emphasis, both are preserved rather than
reconciled.

Claims the organizer independently re-verified against the source at `06c0ab5`, because they are
load-bearing and new: `SyntheticTraceBuilder.BurstStartProbabilityPerSample = 0.01` fired by
`if (burstRemaining == 0 && rng.NextDouble() < BurstStartProbabilityPerSample)` — a memoryless
Bernoulli trigger, while that file's own XML doc (line 13) calls the result "periodic congestion
bursts"; and `Registries.PlayoutPolicies` is typed
`Func<PlayoutPolicyConfig, IMetricSink, IPlayoutPolicy<Pose>>` with no `ITimeAuthority`, while
`PlayoutPolicyConfig.MaxAdaptationRatePerSecond` is specified per second of wall time. Both hold.

**No code, tests, registry entries, YAML, ADRs, `results/` output, or shared-file edits were
produced or attempted by this researcher.**

---

## Seed self-check — PASS

| Check | Expected | Observed | Verdict |
|---|---|---|---|
| `git rev-parse HEAD` | `06c0ab5b6f570efa028002897c856c566041663b` | identical | PASS |
| Worktree | own, not main | `/home/andrei/.../worktrees/agent-a1b3e14c5d640e8e1` | PASS |
| Branch | not `main` | `worktree-agent-a1b3e14c5d640e8e1` | PASS |
| `just core-check` | 3 gates green | `dotnet test` 558 + 36 + 28 + 3 passed / 0 failed; `verify: PASS`; `audit: PASS` | PASS |

---

## 1. Estimator / consumer split, and the ranked consumers

**(a) Estimator** — observes arrivals (interarrival deltas, sequence gaps, OWD trend, burst
structure) and produces a live estimate or forecast. **(b) Consumer** — changes behaviour in
response. An estimator with no consumer is unmeasurable, so the consumer decides everything.

Ranked, with honest existence status:

1. **Adaptive playout buffer.** `Contracts/IPlayoutPolicy.cs` is written; `PlayoutPolicyConfig`
   already carries `TargetPercentile`, `DelayProcessNoise`, `DelayMeasurementNoise`,
   `MaxAdaptationRatePerSecond`; `PlayoutPolicyDiagnostics` mandates delay budget / occupancy /
   late-arrival rate; `Registries.PlayoutPolicies` exists and is empty. The contract was clearly
   designed for exactly this candidate. **Nothing implements it, and `OperatorEndpoint` hardcodes
   `t_playout = t_operatorRecv`.** The only consumer with a written home.
2. **Adaptive prediction horizon.** No home. `IPredictor.Predict(nowTicks)` takes an absolute
   instant; horizon is implicit in what the caller asks for, and the only caller is
   `OperatorEndpoint.EstimateRobotState(nowTicks)`, which always asks for "now".
   `PredictorConfig.MaxHorizonTicks` is a clamp, not a live knob. Making the horizon link-adaptive
   is a `Pipeline/` change, and it is **mutually exclusive with #1 in any sweep** — both change
   effective staleness, and `Buffering/CLAUDE.md` forbids moving both.
3. **Adaptive redundancy depth.** Three stacked blockers: `NFrameRedundantCodec` is not built (and
   is candidate 1's territory), the sweep cannot vary the codec, and `SweepCommand` sizes the
   uplink transport from the fixed `RawPoseCodec.EncodedSize` so a variable-size codec breaks the
   wiring. Not feasible now.
4. **Adaptive send rate / pacing.** Effectively out of scope for Core. Cadence is host
   orchestration by `Pipeline/CLAUDE.md` requirement 2 ("no third driver class"), `ITransport.Send`
   has no pacing concept, and Core may not own a loop.

---

## 2. Contract determination — unambiguous

**No new `Contracts/` interface. Yes an ADR — for the `Pipeline/` wiring, not for the contract.
Then an ordinary `Buffering/` implementation. In that order.**

The interface half is genuinely done: `IPlayoutPolicy<TState>` declares
`Enqueue`/`TryDequeue`/`Reset`/`Diagnostics`, the config and diagnostics types exist, the registry
table is typed. An estimator belongs *inside* a policy as a private component, not as a peer
contract — a standalone `INetworkEstimator` would be a contract with exactly one consumer and no
way to score it.

But "the interface is declared" does **not** make the wiring ordinary work. `Pipeline/CLAUDE.md`
says "replace that line, not add around it", which under-prices what actually has to change in
`OperatorEndpoint.TryReceiveState`:

- Today `TryReceiveState` does `.WithPlayoutTicks(arrivalTicks)` and immediately calls
  `ObserveRobotState(...)`, then returns the completed trace. With a real policy, receive and
  playout are different instants. The `LatencyTrace` for a sequence must be **held between Enqueue
  and TryDequeue** so `WithPlayoutTicks` can carry the true playout instant, and `ObserveRobotState`
  must move to the dequeue path — otherwise the predictor observes samples the buffer has not
  released, and the buffer does nothing.
- That changes `TryReceiveState`'s return contract (a trace can be network-complete but not yet
  played out), and it adds a second drain loop the host must call. Both bind future code. That is
  what an ADR is for.

The ADR must decide, specifically: where Enqueue/TryDequeue sit in the step order; whether
`TryReceiveState` still returns a completed trace or splits into two methods; where
`ObserveRobotState` moves; and whether the `PlayoutPolicies` factory signature gains an
`ITimeAuthority`.

**That last one is a concrete, overlooked cost.** The factory is
`(PlayoutPolicyConfig, IMetricSink) -> IPlayoutPolicy<Pose>` — **no clock** — while
`PlayoutPolicyConfig.MaxAdaptationRatePerSecond` is documented as "ticks of budget per *second* of
wall time". Converting needs `TicksPerSecond`. `Predictors` takes `(config, clock)` and
`Reconcilers` takes `(config, sink, clock)`, so precedent favours adding it — but that edits
`Registry/Registries.cs`, a named shared-conflict file.

Separately: `ExperimentConfig` has no `playoutPolicies` field and `SweepCommand` never constructs
one, so a finished policy still cannot be swept. Same class of constraint as the codec one, and it
must be fixed for this axis (unlike the codec one, which I was told not to touch).

Build order: **ADR → `ImmediatePlayout` (byte-identical to today's stand-in, proven) → sweep
plumbing → `FixedDelayPlayout` → `PercentileTrackingPlayout` → only then anything adaptive.**
`immediate` must come first and be proven identical to the hardcoded line, because that is the only
way to show the wiring change did not itself move the numbers.

---

## 3. The inelastic-sender challenge — answered

**Yes. On this system only the receive side has a meaningful knob, and "predict the network"
reduces to "schedule playout better". That does reorder the run's recommendation: this candidate
is a `Buffering/` candidate, not a `Transport/` one.**

Why, specifically here:

- The uplink is one 73-byte `RawPoseCodec` frame per tick at a cadence Core is forbidden to own.
  There is no bitrate to lower, no resolution to drop, no frame rate to shed, and no queue to drain
  faster. The entire classical RTC actuator set has no analogue.
- The only sender-side analogues that exist *in principle* — redundancy depth and payload content
  (intent instead of pose) — are the other two researchers' candidates, and both are blocked on a
  variable-size codec plus the sweep constraint.
- At 73 bytes per tick this flow is almost certainly not causing the congestion it would be
  regulating. A congestion *controller* here would be a controller with no authority over the
  disturbance.
- The **estimator** half survives this unscathed. Observing the link is meaningful even when
  controlling the send side is not. It simply has to live inside the receiver.

One honest exception: the *host* could change tick rate. That is outside Core and outside this axis.

---

## 4. Mechanism, sub-variants, and what prior art claims (with measurement conditions)

**Mechanism in one paragraph.** The receiver maintains a running model of the downlink's delay
distribution from arrival stamps it already has (`t_recv` per sequence, plus sequence gaps for
loss). From that model it sets a *delay budget* — the target gap between a sample's capture and its
playout — and holds arriving samples in a reorder-tolerant buffer until their budgeted instant. A
larger budget absorbs more jitter and reordering, so fewer samples are discarded as late and the
predictor sees a cleaner, in-order, evenly-spaced observation stream with fewer gaps to extrapolate
through; a smaller budget adds less latency. The estimator's whole job is to sit at the best point
on that curve as the link changes, instead of at a fixed point chosen offline.

Sub-variants, in ascending ambition:

- **Percentile tracking.** Keep a window/histogram of recent one-way delays; set the budget to a
  target quantile; add spike detection so a sudden delay step is followed fast rather than averaged.
  *Prior art:* Moon, Kurose & Towsley, "Packet audio playout delay adjustment: performance bounds
  and algorithms", Multimedia Systems 6:17–28, 1998
  (<https://link.springer.com/article/10.1007/s005300050073>, PDF at
  <https://citeseerx.ist.psu.edu/document?doi=a9f2a0f2d90a4ef171ba88c7321f320e33c6d29f&repid=rep1&type=pdf>);
  antecedent Ramjee, Kurose, Towsley & Schulzrinne, "Adaptive playout mechanisms for packetized
  audio applications in wide-area networks", INFOCOM 1994
  (<https://www.semanticscholar.org/paper/598f080acc304b1869f75c700b58b90526b6aa9a>). *Claims:*
  tight upper and lower bounds on the optimal average playout delay for a given loss count, and that
  their percentile+spike-detection algorithm performs **close to that theoretical optimum** and
  beats the then-standard EWMA mean+variance estimators. *Measured under:* recorded 1990s Internet
  audio delay traces; ~20 ms packetization; adjustment only at talkspurt boundaries; receive-side
  only, no feedback channel; the actuator is silence-period stretching. Accessed 2026-09-09.
- **Kalman / filter-based.** Filter delay mean and variance (or delay *gradient*) and set the budget
  from the filtered state. *Prior art:* Google Congestion Control, `draft-ietf-rmcat-gcc-02`
  (<https://datatracker.ietf.org/doc/html/draft-ietf-rmcat-gcc-02>) and Carlucci, De Cicco, Holmer
  & Mascolo, MMSys 2016 (<https://c3lab.poliba.it/images/6/65/Gcc-analysis.pdf>) — originally a 2-D
  Kalman filter over one-way delay *variation*, later a scalar Kalman, later a
  trendline/linear-regression filter, with an overuse detector firing when the gradient exceeds an
  adaptive threshold for >100 ms. *Measured under:* elastic video, per-frame or transport-wide-cc
  feedback, actuator = target bitrate. **The estimator is portable to us; the consumer is not.**
- **Production adaptive buffering.** WebRTC NetEQ: `DelayManager` reads a target delay off a
  *forgetting histogram* at a **high percentile** — since 2022 of *relative* delay anchored to the
  fastest recent packet rather than raw interarrival — plus a second histogram (the reorder
  optimizer) for out-of-order and retransmitted packets. Sources:
  <https://chromium.googlesource.com/external/webrtc/+/master/modules/audio_coding/neteq/g3doc/index.md>,
  <https://webrtchacks.com/how-webrtcs-neteq-jitter-buffer-provides-smooth-audio/>, and "Improved
  Jitter Buffer Management for WebRTC", ACM TOMM 2021
  (<https://dl.acm.org/doi/fullHtml/10.1145/3410449>). *Measured under:* 20 ms audio frames with
  time-scale modification (expand/accelerate) available as an actuator — **a knob we do not have for
  pose**. Accessed 2026-09-09.
- **Genuine forecasting.** Predict the *next* delays, not the current distribution — the only
  variant that beats a percentile tracker in principle, and only where structure is forecastable.

**Does forecasting beat percentile tracking? The honest answer is: no evidence, and on our fixture,
provably not.** The production state of the art (NetEQ) is a percentile read off a histogram. The
strongest academic result in the area says percentile tracking is *near the theoretical optimum*.
What is marketed as "predictive" jitter buffering (e.g. the Kalman-based filing US 12483753) is
one-step-ahead filtering — estimation with a lag correction, not multi-step forecasting of
structure. **Negative search result worth recording so it is not repeated:** I found no work
reporting that a multi-step delay forecaster beats a well-tuned percentile tracker for playout
scheduling on general Internet traces. If someone wants to claim that win, they will be making it,
not inheriting it.

Also surveyed and not useful here: RFC 8868 (evaluation criteria for interactive RTC congestion
control) and RFC 8698 (NADA) — both assume an elastic sender, which is exactly the assumption this
system violates.

---

## 5. The `leo-satellite` lead — and the finding underneath it

`Transport/CLAUDE.md` advertises `leo-satellite` with "periodic reconfiguration spikes". **It does
not exist.** `docs/adr/0004-network-profile-suite.md` reserves `cellular-congested`,
`leo-satellite`, and `long-haul` as honestly-unimplemented names pending a real capture, and
`Teleop.Eval/Sweep/NetworkProfileCatalog.cs` keeps them in a `ReservedPendingRealCapture` set that
reports them distinctly from "unknown profile". The strongest possible argument for forecasting
over absorption therefore has no vehicle.

Worse — and this is the finding I would most want a human to see:

**The only trace fixture in the repo has no periodic structure at all, despite the docs saying it
does.** `core/testdata/traces/synthetic-burst.trace` is 2000 samples at 10 MHz. I measured it:
baseline 20.03 ms (sd 1.13), burst plateau 200.5 ms (sd 30.2), 13 burst runs of length 6–15, and
inter-burst-start gaps of **77, 516, 133, 188, 52, 34, 49, 81, 173, 74, 88, 394** samples. That is
not a period. Reading the generator confirms why — `Teleop.Eval/Tooling/SyntheticTraceBuilder.cs`
uses:

```csharp
private const double BurstStartProbabilityPerSample = 0.01; // ~once per 100 samples
```

a **memoryless Bernoulli trigger**. Given no burst is running, the probability of onset is exactly
0.01 regardless of history, so **no forecaster can beat the constant 0.01 on onset prediction.**
What *is* learnable is burst *persistence* (length uniform 5–15), which is regime detection, not
forecasting. On the only structured material this repo owns, "predict the link" collapses to
"detect the regime fast", which is what a percentile tracker with spike detection already does.

Both `docs/adr/0004` and `SyntheticTraceBuilder`'s own XML doc describe these as "periodic
congestion bursts". That word is wrong, and it is precisely the word that makes the forecasting
pitch sound plausible. **A human should correct it** (I may not edit ADRs).

**And the trace profile makes seeds inert.** `NetworkProfileCatalog.TryResolve` gives
`synthetic-burst` a profile with zero base delay, zero jitter, zero loss, zero reorder — the trace
supplies delay, nothing else. Every RNG draw in `EmulatedTransport.DrawDelayTicks`/`Send` on that
profile is a no-op (jitter span 1 → always 0; loss and reorder probability 0 → never fire). **So all
five seeds in `exp-001` produce bit-identical trials on `synthetic-burst`.** The one profile with
exploitable structure has n = 1, not n = 5, and `docs/metrics.md` §8.3's seed-spread requirement is
vacuous there. On top of that, `trialSteps: 500` at `stepIntervalTicks: 100000` (10 ms) consumes
only the **first 500 trace samples — exactly two burst episodes** (index 37, length 13; index 114,
length 9; 22 of 500 samples). Any adaptive-vs-fixed result on that profile rests on two events.

This is a code-derived prediction, not a measurement (I was not permitted to sweep). It is trivially
falsifiable: run the same experiment twice with different seed lists restricted to `synthetic-burst`
and diff the CSVs.

**Requirement, not a request:** this direction needs a profile with genuine periodic or otherwise
forecastable structure before "forecasting" can be distinguished from "tracking". That means either
a real capture (`leo-satellite`) or a new ADR authorising a new synthetic trace family. I must not
add or edit a profile, so this is blocked on a human.

---

## 6. Which existing metric moves, in which direction, on which profile — and the instrumentation gap

I verified the artifact handed to me. The complete set of `Record(` call sites outside tests is:

```
core/Teleop.Core/Pipeline/OperatorEndpoint.cs:227   owd_uplink_ms
core/Teleop.Core/Pipeline/OperatorEndpoint.cs:232   owd_downlink_ms
core/Teleop.Eval/Sweep/SweepCommand.cs:304          prediction_position_error_mm
core/Teleop.Eval/Sweep/SweepCommand.cs:305          prediction_orientation_error_deg
```

plus `correction_magnitude_mm/_deg`, `time_to_convergence_ms`, `jerk_mm_s3` from the reconcilers.
**Everything in `docs/metrics.md` §3 — jitter, loss rate, burst-length distribution, reordering
rate, goodput — is defined and emitted by nothing.** The axis whose premise is "the link's condition
is observable and worth acting on" currently measures none of the link's condition.

**The blocking gap is worse than §3, and it is not §3.** A playout policy's headline result — delay
budget vs. induced late-arrival rate — is reported through `PlayoutPolicyDiagnostics`, a **struct
property, not `IMetricSink`**. Those numbers never reach `metrics.csv`, and their names are not in
`docs/metrics.md`. So the policy's own operating point is unreportable until a human adds
definitions. `IPlayoutPolicy`'s doc says a policy that does not report its operating point "is not
evaluable" — that is currently true of any policy anyone could write.

What must be emitted, by which component, before an adaptive policy is evaluable:

| Quantity | Emitter | Status |
|---|---|---|
| Delay budget (ms) | the `IPlayoutPolicy` implementation | needs a metric definition (human) |
| Induced late-arrival rate | same | needs a definition (human) |
| Underrun count/rate | same; `Buffering/CLAUDE.md` requires it be pushed to `IMetricSink` | needs a definition (human) |
| Buffer occupancy | same | needs a definition (human) |
| Loss rate, burst-length distribution (§3) | a sequence-gap tracker in `Metrics/`, fed by `OperatorEndpoint` | defined, no emitter |
| RFC 3550 jitter / OWD IQR (§3) | same | defined, no emitter |

**Identifiability limit worth stating in the ADR:** because `RobotEndpoint` replies once per
*received* datagram, a lost uplink command produces no reply at all. From the operator side, uplink
loss and downlink loss are indistinguishable on a round-trip-matched stream. A §3 loss metric
emitted at `OperatorEndpoint` measures round-trip survival, not per-direction loss, and must be
named and defined as such or it will be misread.

**Direction of the existing metrics, if a buffer is wired:**

- `owd_uplink_ms` / `owd_downlink_ms`: **will not move.** They are `t_recv − t_send`; the buffer
  acts after `t_recv`. **The buffer's entire latency cost is invisible to every currently emitted
  metric.** This is the single most important instrumentation fact in this assessment.
- `prediction_position_error_mm` / `prediction_orientation_error_deg`: expected to **rise** (samples
  are older when the predictor observes them, so it extrapolates further) — but partly offset by a
  cleaner, in-order, gap-free observation stream.
- `correction_magnitude_mm/_deg`, `jerk_mm_s3`, `time_to_convergence_ms`: expected to **fall** on
  jittery/bursty profiles (fewer out-of-order and late shocks).

That is exactly the tradeoff shape this run is looking for, and both halves are already measurable.
**But reported on today's metrics, a buffering win would look free, and it would not be.** Combined
with constraint 5 (no displayed-pose accuracy metric), a policy that smooths the display while
degrading its fidelity would post improved `jerk_mm_s3` with nothing showing the cost. Do not let a
buffering result ship on the current instrument set.

**By profile:** `lan` (2 ms ± 1 ms, no loss) — expect a flat null, nothing to absorb; `50ms-5j` —
near-null; `150ms-20j-0.5loss` and the `jitter-Nms` family — jitter absorption should show;
`300ms-60j-2loss-bursty` (expected burst length 3.33) — strongest parametric case; `synthetic-burst`
— the only bimodal/regime case and the only place adaptivity could beat a fixed budget, subject to
the n=1 problem above.

---

## 7. Population traps a sweep on this axis must be read against

- **OWD is conditioned on survival** (round-trip matched by `Sequence`); a lost packet emits
  nothing. Worse for us than stated: a policy that discards late arrivals also removes samples from
  `ObserveRobotState`, so it changes the **predictor's** input stream. The comparison across
  policies therefore compares different sample sets on *every* metric, not just OWD. Report counts
  alongside percentiles, and prefer per-frame metrics (`jerk_mm_s3`) over per-event ones
  (`correction_magnitude_mm`) — the reconciliation axis already learned this exact lesson and
  `docs/metrics.md` §5 records why.
- **Seeds are inert on `synthetic-burst`** (§5). Report the spread as "zero by construction", never
  as "tight".
- **The synthetic operator motion is a smooth analytic sinusoid** with `Vector3.Zero` for both
  velocity fields. This matters less here than for the intent codec — but not zero. A C-infinity
  signal makes a late or dropped sample *cheap*, because `const-vel`/`double-exp` extrapolate a
  sinusoid well over 20–200 ms. So the buffer's central benefit ("fewer gaps to extrapolate
  through") is systematically **understated** by this workload. That biases against the candidate,
  which is the safe direction, but a null result on this motion is not a null result in general.
- Non-issue, checked: `_inFlightSequences` is 64 deep; at `300ms-60j-2loss-bursty` with 10 ms steps
  that is ~60 in flight, close but sufficient. A playout buffer sits *after* `TryReceiveState`'s
  match, so it adds no in-flight pressure.

> **Organizer's note on the last bullet.** This researcher judged the 64-deep in-flight ring
> "close but sufficient". Candidate 4 and the candidate-3 relaunch both reached the opposite
> conclusion, and the relaunch reported confirming it empirically against recorded `results/` row
> counts. The organizer verified the *mechanism* (`InsertInFlight` overwrites an occupied slot
> without checking `_inFlightOccupied`, and an evicted trace makes `TryReceiveState` skip the frame
> before `ObserveRobotState`). The disagreement is about whether the window is actually exceeded in
> practice, and it is flagged in the decision record as needing a measurement rather than a third
> opinion.

---

## 8. Build cost, in phases

| Phase | Work | Cost | Blocked? |
|---|---|---|---|
| 0a | ADR for the `Pipeline/` playout wiring (Enqueue/TryDequeue placement, `TryReceiveState` contract, `ObserveRobotState` move, factory signature) | — | **human** |
| 0b | Metric definitions in `docs/metrics.md` for the playout operating point | — | **human** |
| 1 | `ImmediatePlayout` + registry + tests; replace the hardcoded line; prove a sweep is byte-identical to the pre-change baseline | ~1 unit | after 0a |
| 2 | Sweep plumbing: `ExperimentConfig.playoutPolicies`, `SweepCommand` construction, registry factory signature (`Registries.cs`, shared file) | ~1 unit | |
| 3 | `FixedDelayPlayout` — the second baseline; gives the latency/loss curve its x-axis | ~0.5 unit | |
| 4 | `PercentileTrackingPlayout` (Moon/Kurose/Towsley-style, with spike detection) — the first real "adapt to the estimated link" | ~1 unit | |
| 5 | A real estimator: Kalman over delay, or Gilbert-Elliott regime inference (`kalman-jitter` / `adaptive`) | ~1–2 units | |
| 6 | A profile with genuine forecastable structure | — | **human** (real capture or new ADR) |

First *interesting* result: Phase 0 (human) + ~3.5 agent units. First result that answers "does
forecasting beat tracking": +2 units **and a profile that does not exist**.

---

## 9. The falsifier

**Cheapest experiment that could kill the whole direction (needs Phases 0–3 only, no estimator at
all):** hold predictor at `none` and reconciler at `snap`; sweep `fixed` over a grid of delay
budgets against `percentile` over a grid of target percentiles, on `300ms-60j-2loss-bursty` and
`synthetic-burst`; plot induced late-arrival rate against delay budget. **If the adaptive percentile
tracker does not sit strictly inside the `fixed` sweep's latency/loss frontier on any profile, the
direction is dead at the consumer and no better estimator can rescue it** — because an estimator can
only move a policy *along* that frontier, never off it. Run this before building any estimator.

**Cheaper still, and unblocked today:** an offline analysis in `analysis/` (Python, reads the delay
trace, no Core code, no ADR, no metric) computing Moon/Kurose/Towsley-style optimum-vs-achievable
playout-delay bounds on `synthetic-burst`'s delay sequence. If the gap in milliseconds between the
offline optimum and a well-chosen fixed budget is small on the only trace we have, there is nothing
for any policy to win and the whole run can stop there. **This is the single next thing I would do.**

---

## 10. Is it distinguishable by measurement from something already implemented?

Nothing implements `IPlayoutPolicy`, so the only available comparison is against
`OperatorEndpoint`'s hardcoded `t_playout = t_operatorRecv`, i.e. the not-yet-built `immediate`.

That comparison **can** show whether buffering changes prediction error and correction cost, and it
is a genuine baseline (it is what every current result was measured against). It **cannot** show:
the latency the buffer costs (no emitted metric reads `t_playout`); whether induced late-arrival
loss exceeds network loss (no emitted §3 loss metric); or whether displayed-pose accuracy degraded
(constraint 5 — no such metric exists). Those three absences all point the same way: **the current
instrument set can only report the upside.**

---

## 11. How this assessment could flatter itself

- I ranked adaptive playout first partly because it is the consumer with a written contract. "There
  is already an interface for it" is an argument about repo convenience, not about the operator. Had
  the contract been written for adaptive redundancy, I would probably have ranked that first.
- **I measured nothing.** Every directional claim ("buffer lowers jerk, raises prediction error") is
  mechanism reasoning, and the causal coupling through `ObserveRobotState` could invert it.
- The seeds-are-inert claim on `synthetic-burst` is code-derived, not observed. Falsify it in two
  minutes by diffing CSVs from two seed lists on that profile.
- My "unforecastable" finding rests on one 2000-sample synthetic file. That is a fact about our
  fixture, **not about real links**, and it must not be allowed to read as "delay forecasting does
  not work". LEO links genuinely do have periodic reconfiguration structure; we simply have no
  capture of one.
- The "percentile tracking is near-optimal" bound comes from 1990s audio traces at ~20 ms
  packetization with talkspurt-granularity adjustment. Our profiles reach 300 ms + 60 ms jitter + 2%
  bursty loss, well outside where that result was measured. I used it to argue *against* my own
  candidate's most ambitious variant, which is the direction that makes it safe to lean on — but it
  is still an extrapolation.
- I had an incentive to conclude "this needs an ADR", because the brief told me that is a successful
  outcome. My guard against that was to name exactly which four decisions the ADR must make. If I
  could not name them, "needs an ADR" would have been evasion.

---

## Bottom line

**Build it — but not yet, and not in `Transport/`.** The estimator has no home until a consumer
exists; the only viable consumer is `IPlayoutPolicy`; that is blocked on an ADR for the `Pipeline/`
playout wiring and on metric definitions for the playout operating point, neither of which an agent
may write. On this system "predict the network" reduces to "schedule playout better", which makes
this a `Buffering/` candidate. **The correct next unit of work is the offline trace-bounds analysis
in `analysis/`** — unblocked today, needs no Core code, and can kill the entire direction for a
fraction of the cost of building anything.

## Left undone / for a human

1. ~~Recreate my log from this text~~ — done by the organizer; this file.
2. Write the ADR for the `Pipeline/` playout wiring (four decisions named in §2).
3. Add playout-operating-point metric definitions to `docs/metrics.md`.
4. Correct "periodic congestion bursts" in `docs/adr/0004-network-profile-suite.md` and in
   `core/Teleop.Eval/Tooling/SyntheticTraceBuilder.cs`'s XML doc — the generator is a memoryless
   Bernoulli trigger and the word "periodic" is load-bearing in the wrong direction.
5. Decide whether to authorise a forecastable-structure trace family (new ADR) or leave
   `leo-satellite` reserved.
6. Note that `Registries.PlayoutPolicies`' factory lacks the `ITimeAuthority` that
   `PlayoutPolicyConfig.MaxAdaptationRatePerSecond` requires — a shared-file change the first
   adaptive policy will force.
