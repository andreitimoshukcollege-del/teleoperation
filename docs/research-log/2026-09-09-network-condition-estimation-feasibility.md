# Network condition estimation and adaptation — feasibility assessment

**Agent:** Claude Opus 5 (candidate 3 of 4, Transport-axis survey run)
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-af92076684e411fa1`
**Branch:** `worktree-agent-af92076684e411fa1`
**HEAD at start:** `06c0ab5b6f570efa028002897c856c566041663b`

**Assignment:** survey and feasibility only. No `.cs`, no test, no registry entry, no experiment
YAML, no sweep, no `results/` output. This log is the only artifact.

**Candidate:** "estimate and predict the link, then adapt to it" — the user's framing of
*predicting* network conditions rather than merely absorbing them.

**Incident during the run:** the worktree became unreachable mid-write (bash and Read both
reported it removed) and then became writable again. An earlier appended copy of this log was
lost and this file is a full rewrite. Per `docs/research-log/CLAUDE.md`, **copy this file out of
the worktree immediately** — the loss window is real and was hit once already.

## Seed self-check

| Check | Expected | Actual | Verdict |
|---|---|---|---|
| `git rev-parse HEAD` | `06c0ab5b6f570efa028002897c856c566041663b` | `06c0ab5b6f570efa028002897c856c566041663b` | PASS |
| Worktree path | own worktree | `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-af92076684e411fa1` | PASS |
| Branch | own branch | `worktree-agent-af92076684e411fa1` | PASS |
| Three gates | skipped by instruction | not run — three other researchers confirmed green at this sha (558 tests, `verify` PASS, `audit` PASS) | N/A |

---

## Hypothesis (written before any investigation)

**Claim under test:** a component that *estimates and forecasts* link conditions (delay, jitter,
loss), feeding a consumer that adapts (most plausibly a playout buffer), lowers
`prediction_position_error_mm` / `prediction_orientation_error_deg` and/or
`correction_magnitude_mm` / `jerk_mm_s3` relative to the current hardcoded immediate playout, on
the profiles whose delay process has exploitable structure.

**What would falsify it, stated up front:**

1. No profile in the frozen suite has structure a forecaster can exploit beyond what a fixed
   percentile tracker already captures. (A bounded-uniform jitter process is memoryless; its
   optimal fixed budget is its known upper bound.)
2. The consumer knob does not exist and cannot be wired without an architecture change larger
   than the expected gain.
3. The metrics cannot see the effect: a playout buffer changes *when* a sample is used, and if
   nothing measures displayed-pose accuracy or end-to-end playout latency, the win is invisible
   and the cost is invisible too.
4. The measurement harness is biased in a way that makes any before/after comparison on the
   relevant profile untrustworthy.

**Outcome: all four fired against the *forecasting* claim.** A fifth mechanism, not in my original
hypothesis, survived and is strong — see section 5. Falsifier 4 fired hardest and produced the most
valuable output of this assessment.

---

## Prior finding verification (assigned)

### The in-flight ring censors OWD — CONFIRMED, and worse than reported

`Pipeline/OperatorEndpoint.cs` keeps in-flight `LatencyTrace`s in a fixed ring written round-robin
by `InsertInFlight`, which **overwrites unconditionally**:

```csharp
_inFlightSequences[_inFlightNextIndex] = sequence;
_inFlightTraces[_inFlightNextIndex] = trace;
_inFlightOccupied[_inFlightNextIndex] = true;
_inFlightNextIndex = (_inFlightNextIndex + 1) % _inFlightSequences.Length;
```

`SweepCommand` sets `InFlightCapacity = 64` and submits exactly one command per 10 ms step, so
sequence *n*'s slot is reused by the `SubmitCommand` of step *n+64* — which runs **before** that
step's receive drain. The trace therefore survives receive drains at steps *n* … *n+63*: a
**630 ms** budget, not 640.

Step accounting: the operator sends at step *n*; the robot drains it at step
*n* + ceil(d_up/Δ); the reply is drained at step *n* + ceil(d_up/Δ) + ceil(d_down/Δ). On
`300ms-60j-2loss-bursty`, d ~ U(240, 360) ms, so each ceil term is ~uniform on {25…36}. The round
trip survives iff the two terms sum to ≤ 63. P(sum ≤ 63) for two iid uniform{25…36} = 99/144 =
**0.6875**. So **~31% of round trips are evicted before their reply arrives**, and the evicted ones
are exactly the slowest.

**Verified against recorded data**, not just derived.
`results/exp-001-predictor-baseline/20260908-034948Z/none/*/metrics.csv`, `owd_uplink_ms` row counts
over 5 seeds × 500 steps = 2500 possible round trips:

| Profile | `owd_uplink_ms` rows | Expected w/o ring eviction | Expected w/ ring eviction |
|---|---|---|---|
| `lan` | 2490 | 2490 (5×2 still in flight at end) | same |
| `50ms-5j` | 2444 | ~2445 | same |
| `150ms-20j-0.5loss` | 2328 | ~2340 | same (32 steps ≪ 63) |
| `300ms-60j-2loss-bursty` | **1446** | ~2096 | **1441** |
| `synthetic-burst` | 2475 | ~2475 | same |

The model predicts 1441; the recorded file contains 1446. **0.35% agreement.** No longer an estimate.

Consequence on the recorded distribution, same files:

| | recorded | true process (incl. the +U(0,10) ms step quantization) |
|---|---|---|
| `owd_uplink_ms` mean | 287.2 | ~305 |
| p50 | 284.2 | ~305 |
| p95 | 338.2 | ~356 |
| p99 | 350.0 | ~359 |
| max | 363.1 | 370 |

**Every recorded `owd_uplink_ms`/`owd_downlink_ms` figure on `300ms-60j-2loss-bursty` is biased low
by roughly 18 ms at the mean and at p95.** Confirmed as candidate 4 suspected.

### The larger consequence candidate 4 did not name

Eviction does not merely drop a *measurement*. In `TryReceiveState`:

```csharp
if (!TryTakeInFlight(stateFrame.Sequence, out LatencyTrace trace)) { continue; }
...
ObserveRobotState(stateFrame.Pose, downlinkSendOperatorDomain);
```

`ObserveRobotState` is what feeds `IPredictor.Observe` and `IReconciler.Observe`. A state frame
whose sequence has been evicted is discarded **before** the predictor or reconciler ever sees it.

So on `300ms-60j-2loss-bursty` the harness injects a **~31% artificial downlink loss on top of the
profile's 2%**, invisible in the profile definition, affecting `prediction_position_error_mm`,
`correction_magnitude_mm`, `time_to_convergence_ms` and `jerk_mm_s3` — every metric the Prediction
and Reconciliation axes have ever reported on the hardest profile. This is not a metrics artifact;
it changes what the algorithms are fed.

It is a harness defect in `core/Teleop.Eval/`, which I am forbidden to edit, and it is not in my
candidate's lane. Recorded here for someone to own. The fix is cheap (raise `InFlightCapacity`, or
make `InsertInFlight` refuse to overwrite an occupied slot and count the refusal) but it invalidates
comparability with every existing `results/` directory, so it needs a human decision.

### `leo-satellite` — DOES NOT EXIST

`core/Teleop.Eval/Sweep/NetworkProfileCatalog.cs`:

```csharp
private static readonly HashSet<string> ReservedPendingRealCapture = new HashSet<string>(
    StringComparer.Ordinal) { "cellular-congested", "leo-satellite", "long-haul" };
```

`TryResolve` returns **false** for it with a distinct "reserved for a real network capture, not yet
available" error. `Transport/CLAUDE.md`'s "periodic reconfiguration spikes" describes an intended
capture, not a file. This is honest and correctly implemented (invariant 10 in spirit), and it
removes the single strongest argument for this candidate: **the one profile with advertised
periodic — i.e. genuinely forecastable — structure is not in the suite.**

### `synthetic-burst` — the only heavy tail, and it has ZERO seed variance

Analyzed `core/testdata/traces/synthetic-burst.trace` (read-only; 2000 samples at 10 MHz):

| | ms |
|---|---|
| min / p50 / p75 / p90 | 18.0 / 20.2 / 21.2 / 21.8 |
| p95 / p99 / max | **171.0** / 238.1 / 250.0 |

Bimodal: a ~20 ms baseline punctuated by **13 delay bursts** of 6–15 consecutive samples reaching
170–250 ms. Burst onsets at sample indices 37, 114, 630, 763, 951, 1003, 1037, 1086, 1167, 1340,
1414, 1502, 1896 — inter-onset gaps 77, 516, 133, 188, 52, 34, 49, 81, 173, 74, 88, 394. **Not
periodic.** Poisson-like onsets with a persistent high state.

Autocorrelation of the delay sequence: lag-1 **0.869**, lag-2 0.764, lag-3 0.669, decaying
monotonically to ~0 by lag 12. No secondary peak anywhere in lags 1–399. The structure present is
*persistence*, not *periodicity* — a two-regime hidden state, exactly what a Gilbert-Elliott-style
state estimator or a NetEQ-style occupancy tracker reacts to, and exactly what a *periodic*
forecaster has nothing to work with.

**The trial window destroys most of it.** `EmulatedTransport` consumes the trace one sample per
datagram from index 0 (`NextTraceSample`; `_traceIndex = 0` at construction and at `Reset`), and a
trial is 500 steps. Only samples 0–499 are ever used, containing exactly **two** bursts, at indices
37–49 and 114–122 — i.e. **both inside the first 1.25 s of a 5 s trial**, squarely in the startup
transient where `ClockSync` has not converged and predictor history is still filling. 22 of 500
samples (4.4%) are in a burst.

**And every seed produces the identical realization.** In trace mode the profile's base delay and
jitter must be zero, and `synthetic-burst`'s loss and reorder probabilities are all 0.0, so the
`SeededRng` — the only source of randomness in a trial — has no outcome to affect. Verified
empirically: splitting
`results/exp-001-predictor-baseline/20260908-034948Z/none/synthetic-burst/metrics.csv` into its five
per-seed trials gives five **byte-identical** 2208-line blocks.

`docs/metrics.md` §8 rule 3 ("never declare a winner from a single seed, and always state the
observed seed spread") is therefore **unsatisfiable on `synthetic-burst`**: the seed spread is
identically zero by construction and pooling five seeds pools five copies of one trial. n = 1.

This lands directly on my candidate: `synthetic-burst` is the *only* profile where an adaptive
buffer has anything to win, and it is the profile where the evidence is weakest.

### Bounded uniform jitter — confirmed, and decisive here

`Transport/NetworkProfileCatalog.TryResolveParametric` builds every parametric profile as
`Base + Uniform(−J, +J)` with hard bounds. For a *bounded* delay distribution the optimal fixed
playout budget is a known constant — `Base + J` — reachable by a two-line `fixed` policy with no
estimator at all. There is nothing for an adaptive scheme to track that a constant does not already
get right, and nothing for a forecaster to forecast that a percentile does not already bound.
Falsifier 1 fires on `lan`, `50ms-5j`, `150ms-20j-0.5loss`, `300ms-60j-2loss-bursty`, and the whole
`jitter-`/`delay-`/`loss-`/`combo__` grid.

---

## 1. The estimator / consumer split

**(a) The estimator.** A pure function of the arriving stream — interarrival times, sequence gaps,
one-way-delay series — producing a live estimate or short-horizon forecast of delay, jitter and
loss. Structurally a *perfect* fit for Core: no I/O, no wall clock (arrival ticks arrive as
parameters), no allocation, deterministic, seeded. It could be written today and unit-tested against
a synthetic delay series without touching the pipeline at all.

**(b) The consumer.** Something whose behaviour changes as a function of (a). Without one, (a) is an
unused number. Ranked by value-per-unit-cost **in this repo**:

| Rank | Consumer | Exists? | Verdict |
|---|---|---|---|
| 1 | **Adaptive playout buffer** (`IPlayoutPolicy`) | contract + config + diagnostics types exist; **no implementation**, and `Pipeline/` does not call it | The only consumer with a real knob and a real place to live. Also the only one whose operating point the repo already insists on reporting. |
| 2 | **Adaptive prediction horizon** | `IPredictor.Predict(nowTicks)` takes an absolute time, not a horizon; horizon is implicit in `nowTicks − lastObservation`. `PredictorConfig.MaxHorizonTicks` is a static clamp. | There is no per-call horizon knob to adapt. Creating one is a `Contracts/` change to a stable, three-implementation interface — far more invasive than adding a playout policy. **Deprioritized.** |
| 3 | **Adaptive redundancy depth** | `NFrameRedundantCodec` is planned, not built; and `SweepCommand` hardcodes `new RawPoseCodec()` in three places, one sizing the transport by `RawPoseCodec.EncodedSize`. | Blocked twice over: no scheme to control, no sweep axis to vary. Candidate 1's lane; my answer to "is an adaptive controller over it feasible" is **not yet, and not for reasons an estimator can fix**. |
| 4 | **Adaptive send rate / pacing** | `ITransport.Send` is called once per host step by `OperatorEndpoint.SubmitCommand`; cadence is host orchestration and `Pipeline/CLAUDE.md` requirement 2 forbids Core from owning a loop. | See section 3. Nothing to adapt. |

The honest ranking collapses to: **consumer 1 or nothing.**

## 2. The contract determination

**Determination: (i) — an implementation of the existing `IPlayoutPolicy`, plus a Pipeline wiring
change that needs an ADR. Not a new `Contracts/` interface. Not "both, in a build order." One
interface, one ADR, and the ADR is about *wiring*, not about a new abstraction.**

1. **The estimator does not need its own interface.** `IPlayoutPolicy.Enqueue(sequence, sample,
   arrivalTicks)` receives exactly the inputs an estimator needs — arrival time and sequence, from
   which delay, jitter, gaps and reordering are all derived. A delay estimator is therefore a
   *private collaborator of a playout policy*, not a peer contract. Declaring `IDelayEstimator` in
   `Contracts/` would create an interface with one consumer, requiring an ADR, a registry table and
   a `Pipeline/` wiring change, to buy composability nobody has asked for. Root `CLAUDE.md`: "new
   approach to an existing question → new file in the matching Core folder." A percentile tracker
   and a Kalman delay filter are two answers to the *same* question ("when should this sample play
   out?"), which is exactly what `Buffering/`'s planned rows encode — `percentile` and
   `kalman-jitter` are listed as sibling policies, not as an estimator plus a policy. **The repo has
   already made this decision; I am agreeing with it.**

2. **`PlayoutPolicyConfig` already carries the estimator's parameters.** `TargetPercentile`,
   `DelayProcessNoise`, `DelayMeasurementNoise`, `MaxAdaptationRatePerSecond` are all estimator
   knobs sitting in the playout config, each documented as "used by `percentile`" / "used by
   `kalman-jitter`". Whoever wrote these types made the same call.

3. **`PlayoutPolicyDiagnostics` is already the estimator's output surface.** `DelayBudgetTicks` is
   the estimate; `LateArrivalRate` is the estimate's realized error rate. Nothing is missing.

4. **The wiring *is* an architecture change and does need an ADR.** `Pipeline/CLAUDE.md` says the
   hardcoded `t_playout = t_operatorRecv` line must be **replaced**, not added around. Doing that
   means `OperatorEndpoint` gains an `IPlayoutPolicy<Pose>` constructor dependency and a two-phase
   receive (arrival-drain, then playout-drain), which changes when `ObserveRobotState` runs relative
   to when a datagram arrives — i.e. it changes the input timing of *every* predictor and
   reconciler, and therefore the meaning of every existing result. That is squarely "Pipeline learns
   to wire a new contract," root `CLAUDE.md`'s stated ADR trigger. The ADR's subject is the wiring
   and its comparability consequences, not the interface.

**Corollary the organizer should note:** `immediate` is *also* unbuilt, and it is what every future
playout result must be measured against. `ResolvedStack` already writes `"playoutPolicy":
"immediate"` into every `manifest.json` (`StandInPlayoutPolicy = "immediate"`), so **existing
results already assert a playout policy that has no implementation.** Honest, but it means the first
Buffering PR is not "an adaptive policy" — it is `immediate` + `fixed` + the `OperatorEndpoint`
rewire, with any adaptive policy strictly afterwards. That ordering is not optional: without
`fixed`, an adaptive policy has no non-trivial control to beat, and beating `immediate` is trivial
(any buffer beats no buffer on late-arrival rate, loses on latency, and the tradeoff between the two
is the entire point).

## 3. The inelastic-sender challenge — answered

**Yes: only the receive side has a meaningful knob, and the congestion-control literature is
therefore largely inapplicable to the sender here.**

`RawPoseCodec` is a fixed 73-byte frame sent once per 10 ms step: **58.4 kbit/s before headers**,
~96 kbit/s with UDP/IP. Every mechanism in the delay-based-CC family — GCC, BBR and relatives —
exists to answer "what bitrate should I send at?" That presupposes:

- a sender that *can* change its rate (a video encoder's quantizer, a bulk transfer's window);
- a rate high enough that the flow is itself the cause of the queue;
- an application that benefits from more bytes.

None hold. Dropping a pose sample to relieve congestion means not commanding the robot for 10 ms,
strictly worse than sending 73 bytes. Raising the rate does nothing — there is no higher fidelity to
spend bandwidth on. And decisively for this harness: `EmulatedTransport`'s delay is drawn from a
`NetworkProfile` or replayed from a trace and is **completely independent of send rate**. A
send-rate controller here would be provably a no-op *by construction*, not merely ineffective — the
emulator has no queue model for it to act on. Testing one would measure nothing.

Two narrower sender-side knobs survive, both owned by other candidates:

- **Redundancy depth** (candidate 1) is a genuine sender-side rate knob: at 73 bytes/frame, N=3
  costs ~190 kbit/s, still trivial, and buys burst-loss tolerance. An estimator could drive N from
  observed burst structure. But per §1 rank 3 it is doubly blocked, and the right first question is
  whether *fixed* N=2/3 helps at all — that needs no estimator, and if fixed redundancy does not
  help, adaptive redundancy cannot.
- **Pacing/cadence** is host orchestration by `Pipeline/CLAUDE.md` requirement 2 and cannot live in
  Core.

**Consequence for the run's recommendation:** every sender-side framing of "predict the network and
adapt" should be struck. The candidate reduces to receive-side playout, and the estimator is an
implementation detail inside it. That is a genuine narrowing and it should reorder the run.

## 4. Mechanism, sub-variants, and what the prior art actually claims

**One paragraph.** Each arriving downlink state frame carries a capture stamp and an arrival tick;
their difference (offset by an unknown clock skew, which cancels if you use *relative* delay) is a
one-way-delay sample. A policy maintains a running estimate of that series' distribution and holds
each sample until `capture + budget`, releasing it at `t_playout`. Increasing the budget converts
jitter into a uniform, predictable added delay and shrinks the late-arrival rate; decreasing it does
the reverse. That is the entire latency/loss trade `Buffering/CLAUDE.md` names. The candidate's
specific claim is that a smarter estimate of "what the delay will be for the *next* sample" sets a
better budget than a static one.

### Sub-variants, cheapest first

| Variant | Estimator | Size |
|---|---|---|
| `fixed` | none — constant budget | the control; trivial |
| `percentile` | running quantile of a forgetting histogram of relative delay | ~40 lines, no matrix algebra |
| `kalman-jitter` | scalar Kalman over delay mean + variance | ~80 lines; `DelayProcessNoise`/`DelayMeasurementNoise` already in config |
| `adaptive` (NetEQ-style) | occupancy-driven expand/contract | needs an *elastic* consumer — see below |
| genuine forecasting | ARMA / GARCH / regime inference over the delay series | 200+ lines, and the only variant that is actually "predicting the network" |

### Prior art, with measurement conditions

**Zhang, Fay, Kilmartin & Moore, "A Garch-based adaptive playout delay algorithm for VoIP",
Computer Networks 54 (2010) 3108–3122.**
https://www.cl.cam.ac.uk/~awm22/publications/zhang2010garch.pdf (accessed 2026-09-09). The most
directly on-point source found. Proposes ARMA(1,0)+GARCH(1,1) forecasting of the jitter time series,
with a "Direct Garch" parameter-estimation variant whose cost function targets a desired packet-loss
rate *and* penalises consecutive drops.
*Conditions:* three real VoIP traces captured 2007 — NUI Galway to U. Tokyo (jitter ~30 ms), to UNSW
Sydney (<30 ms), to Chengdu (~25 ms), 6.5–21.5 h each; G.729B, **20 ms packets, 80-byte payloads**,
RTP/UDP; traces described as self-similar and bursty; evaluated in a *simulated* network driven by
the recorded jitter; scored on PLR-versus-additional-buffering-delay and PESQ MOS.
*Claim:* Fig. 6 shows Standard Garch, Direct Garch and an MLP neural predictor with **very similar
curves, all noticeably better than the Linear Recursive Filter** (Ramjee et al.) — the gap reads as
roughly 1–2 ms of additional buffering delay at equal PLR, in the 18–32 ms delay / 1–6 % PLR region.
**The part that matters most here, and it is a negative:** the histogram/percentile family (Concord
— described in the paper's own §2 as the "distribution-based" class that estimates the
packet-delay-distribution curve for a desired PLR) **is not on the tradeoff plot at all.** Concord
appears only in Fig. 4 (raw prediction SSE, where Concord1/2/3 sit modestly above the Garch models
and far below the MLP at long horizons) and Fig. 5 (where Concord is described as producing "a
comparatively smooth prediction" that fails to capture burstiness). **This paper demonstrates
forecasting beating a reactive exponential filter. It does not demonstrate forecasting beating a
percentile tracker on the delay/loss tradeoff.**

**NetEQ (libwebrtc `modules/audio_coding/neteq`).** Sources: WebRTC's own g3doc
https://chromium.googlesource.com/external/webrtc/+/master/modules/audio_coding/neteq/g3doc/index.md
and webrtcHacks' detailed walkthrough
https://webrtchacks.com/how-webrtcs-neteq-jitter-buffer-provides-smooth-audio/ (both accessed
2026-09-09). The reference production "adaptive" jitter buffer. `DelayManager` tracks **relative
delay** (each packet's transit versus the fastest packet in a rolling window — the 2022 change away
from inter-arrival, made precisely because inter-arrival misses *accumulating* delay), bins it into
a **forgetting histogram** with forget factor 0.983 (~3.5 s memory), and reads the target delay off
that histogram at the **0.95 quantile**. A second "reorder optimizer" histogram applies a
`delay_ms + 20 ms × loss_percent` cost, and the larger of the two targets wins.
**This confirms the assignment's suspicion from the source: the canonical production "adaptive"
jitter buffer is a percentile tracker with a decay, not a forecaster.** Its *conditions* matter too:
it assumes audio is **elastic** — accelerate/expand are WSOLA-style time-stretches of the waveform —
and assumes regular cadence at a configured ptime. A pose stream is not time-stretchable; there is
no "play this pose 5 ms faster." So `Buffering/CLAUDE.md`'s `adaptive` row ("NetEQ-style:
expands/contracts against buffer occupancy") **cannot be ported literally.** What survives the port
is the delay manager; what does not is the actuator. That row should be rewritten or dropped.

**Google Congestion Control (delay-based half).** IETF draft
https://datatracker.ietf.org/doc/html/draft-ietf-rmcat-gcc-02 ; analysis by Carlucci, De Cicco,
Holmer & Mascolo, MMSys '16, https://c3lab.poliba.it/images/6/65/Gcc-analysis.pdf (accessed
2026-09-09). Receiver (later sender, via transport-wide-cc feedback) estimates the one-way *delay
gradient* — inter-arrival delta minus inter-departure delta — through a Kalman filter (originally
2-D, later scalar) or a trendline filter, compares it against an adaptive threshold, and drives an
over-use detector that throttles the **sending bitrate**. *Conditions:* elastic video, hundreds of
kbit/s to Mbit/s, the flow itself causing the queue, bottleneck links with competing TCP. **Every
one of those conditions fails here** (§3). One narrow idea transfers: the *gradient* formulation
cancels clock offset without requiring clock sync, worth having because `ClockSync`'s offset
estimate is itself noisy early in a trial. Nothing else transfers.

**BBR.** Models the path (bottleneck bandwidth × round-trip propagation time) rather than reacting
to loss, and is the cleanest example of "model the link, don't just absorb it." But its entire
output is a pacing rate and a cwnd, both meaningless for a fixed 73-byte/10 ms stream. Recorded here
as *considered and rejected on applicability*, so nobody proposes it again.

**RFC 3550 interarrival jitter.** `docs/metrics.md` §3 already names it. An exponentially smoothed
mean absolute inter-arrival deviation — a *scale* estimate, not a distribution and not a forecast.
Cheap enough that any policy should compute it, but on its own it sets a budget only via an
arbitrary multiplier.

**Searches that found nothing useful, recorded so they are not repeated.** I looked specifically for
a published head-to-head in which *forecasting* delay beats *percentile/histogram tracking* on the
playout delay-versus-late-loss curve. I did not find one. What exists is (a) forecasting beats
*reactive filters* (Zhang 2010 versus LRF), (b) forecasting beats *nothing* (RBF-prediction playout,
IEEE 2010), and (c) modern probabilistic delay forecasting in 5G scored on **NLL/MAE of the forecast
itself**, not on any downstream buffer tradeoff (https://arxiv.org/html/2503.15297v1, accessed
2026-09-09) — exactly the substitution to watch for: a better forecast is not the same claim as a
better buffer. Telehaptic/teleoperation searches surfaced only adaptive *media* rate allocation with
the haptic stream prioritized, i.e. the elastic-sender case again, not an inelastic-control-stream
playout result. **The marginal value of genuine forecasting over percentile tracking appears to be
an open question in the literature, not a settled win.**

---

## 5. Which existing metric moves, in which direction, on which profiles

This section changed my verdict, so it is the important one.

### 5a. The instruments that do not exist

- **`t_playout` is stamped into `LatencyTrace` and no metric is emitted from it.** There is no
  playout-latency metric name anywhere. A playout buffer's *cost* — the delay it adds — is
  **unmeasurable with the current metric set.** I may not add one, and I have not.
- **`owd_uplink_ms` / `owd_downlink_ms` are computed from `arrivalTicks`, not `t_playout`.** A
  playout buffer therefore moves them **not at all**. The headline latency metrics are blind to this
  entire axis.
- **No displayed-pose accuracy metric** (the assignment's constraint 5, confirmed). If a buffer
  degrades what the operator sees, nothing catches it.
- **`docs/metrics.md` §3 in its entirety** — jitter, loss rate, burst-length distribution,
  reordering rate, goodput — is defined and emitted by nothing. Every input an estimator would
  report is undefined-in-practice.
- **`PredictorDiagnostics.RejectedObservations` is not emitted as a metric either**, which matters
  because of 5c.

### 5b. Why a buffer should *hurt* on the metrics that do exist

`ObserveRobotState` folds each sample into `IPredictor.Observe` as soon as it arrives. Buffering
delays that fold by the delay budget, so the predictor's extrapolation distance grows by the budget
on every sample. `prediction_position_error_mm` and `correction_magnitude_mm` should therefore both
get **worse**, monotonically in budget, on every profile. The classical benefit of a jitter buffer —
delivering a regularly-cadenced stream to a consumer that needs one — is already supplied here by
the predictor/reconciler pair, which is timestamp-driven and does not care about arrival cadence.
`Buffering/CLAUDE.md`'s own "Interaction note" says as much: buffer and predictor are substitutes.

On the naive reading, the candidate is not merely unpromising — it is *expected to lose*.

### 5c. The mechanism that survives: capture-order repair, and it is large

`IPlayoutPolicy` contract item 2 requires releasing samples **in capture-time order regardless of
arrival order**. That is not jitter absorption; it is reordering repair, and it has a real consumer.

**Reordering is pervasive, despite `reorderProbability = 0.0` in every profile.** `EmulatedTransport`
delivers by earliest *synthetic* arrival from a min-heap, so jitter alone reorders: a sample sent
10 ms later with 100 ms less delay overtakes its predecessor. Measured from
`results/exp-001-predictor-baseline/20260908-034948Z/none/*/metrics.csv` by reconstructing each
completed trip's capture tick as `arrival − (owd_up + owd_down)` and checking monotonicity in
arrival order (approximate — `RobotEndpoint` replies in the same step so robot-side processing is
zero, and the method correctly reports 0 on `lan`, which validates it):

| Profile | completed trips | adjacent arrivals out of capture order | max displacement |
|---|---|---|---|
| `lan` | 2490 | 0 (0.0%) | 0 |
| `50ms-5j` | 2444 | 264 (10.8%) | 2 |
| `150ms-20j-0.5loss` | 2328 | 785 (**33.8%**) | 5 |
| `300ms-60j-2loss-bursty` | 1446 | 545 (**37.8%**) | 8 |
| `synthetic-burst` | 2475 | 155 (6.3%) | **38** |

(The `300ms` row is itself computed on the ring-censored 69%, so treat it as indicative.)

**And one of the two real predictors rejects every out-of-order sample outright.**
`DoubleExponentialPredictor.Observe`:

```csharp
if (_hasState && obs.CaptureTicks <= _lastAcceptedTicks)
{
    _rejectedObservations++;
    return;
}
```

with a type doc that states the policy explicitly: "a late sample is therefore rejected outright
rather than applied out of sequence … That is a real behavioural difference between the two
predictors on a reordering trace." `ConstantVelocityPredictor`, by contrast, splices late samples
into a stamp-ordered window and derives its rate from the newest pair only — deliberately
order-independent.

So on `150ms-20j-0.5loss` and `300ms-60j-2loss-bursty`, **`double-exp` is throwing away roughly a
third of its input while `const-vel` keeps all of it**, and that difference is currently attributed
to predictor quality. The signature is visible in the existing results
(`results/exp-001-predictor-baseline/20260908-034948Z/`, `prediction_position_error_mm`, mm):

| Profile | out-of-order | `none` p50/p95 | `const-vel` p50/p95 | `double-exp` p50/p95 |
|---|---|---|---|---|
| `lan` | 0.0% | 3.67 / 5.25 | 0.06 / 0.15 | 1.32 / 1.73 |
| `50ms-5j` | 10.8% | 19.08 / 31.64 | 4.33 / 29.28 | 20.78 / **592.24** |
| `150ms-20j-0.5loss` | 33.8% | 54.92 / 88.99 | 32.83 / 103.00 | 14.71 / **315.80** |
| `300ms-60j-2loss-bursty` | 37.8% | 100.90 / 1300.04 | 80.97 / 1300.06 | 48.89 / 1300.04 |
| `synthetic-burst` | 6.3% (disp. 38) | 6.54 / 21.00 | 3.02 / 19.71 | 5.19 / **379.70** |

`double-exp` has the **best p50 on three profiles and a catastrophic p95 on four** — precisely the
signature of a filter that is accurate while fed and goes stale during rejection runs.
`synthetic-burst` is the clincher: only 6.3% adjacent inversions but a max displacement of 38,
because a 250 ms delay burst lets ~23 later packets overtake it — one long rejection run, and a p95
of 380 mm against `const-vel`'s 19.7 mm.

*(The repeated exact `1300.0x` values are the trajectory's own extent — |x| ≤ 0.5 m, z ∈ [0.7, 1.3] m
— i.e. "the estimate is at the origin." A saturation artifact worth someone's attention; not mine.)*

**So there is a real, named, large effect a playout policy addresses — and it is not the candidate's
effect.** It requires no estimator, no forecaster, no adaptivity: a `fixed` buffer of roughly 2×
jitter reorders everything correctly. The candidate's *distinctive* ingredient (estimation, let
alone forecasting) contributes nothing to it.

### 5d. Summary of expected movement

| Metric | Direction from a `fixed` capture-ordering buffer | Profiles where it should show |
|---|---|---|
| `prediction_position_error_mm` p95, `double-exp` | **down, large** (rejection runs eliminated) | `50ms-5j`, `150ms-20j-0.5loss`, `synthetic-burst` |
| `prediction_position_error_mm` p50, all predictors | **up, small** (every sample is staler by the budget) | all |
| `prediction_position_error_mm`, `const-vel` | **up only** — already order-tolerant, so it pays the staleness and gains nothing | all |
| `correction_magnitude_mm` / `_deg` | mixed: down for `double-exp` (fewer stale-jump corrections), up for `const-vel` | as above |
| `jerk_mm_s3`, `time_to_convergence_ms` | follows correction magnitude | as above |
| `owd_uplink_ms` / `owd_downlink_ms` | **no change, by construction** | none |
| the buffer's own added latency | **no instrument exists** | — |

Profiles that will show **nothing**: `lan` (zero reordering, zero jitter) and the entire
`combo__delay-0.1ms…` grid in `exp-gui-sweep.yaml`. Profiles with exploitable structure for a
*forecaster* specifically: **none that exist.** `synthetic-burst` has regime persistence but two
events per trial and zero seed variance; `leo-satellite`, the only advertised periodic profile, is
unimplemented.

## 6. Build cost, in phases

Phase 0 is not optional and is not mine.

| Phase | Work | Who | Cost |
|---|---|---|---|
| **0** | Decide and fix the in-flight-ring censoring in `SweepCommand`/`OperatorEndpoint`, and decide what happens to existing `results/`. Blocks any Buffering result on the 300 ms profile from being trustworthy. | human + whoever owns `Teleop.Eval` | small code, large decision |
| **1** | ADR: wire `IPlayoutPolicy<Pose>` into `OperatorEndpoint`, replacing `t_playout = t_operatorRecv`; state the comparability break explicitly. | — | 1 ADR |
| **2** | `Buffering/ImmediatePlayout.cs` + `FixedDelayPlayout.cs`, registry entries, unit tests (reorder, duplicate, underrun, `Reset`), `OperatorEndpoint` rewire, allocation tests. `immediate` must be behaviourally identical to today's hardcoded line — that identity is the migration's own gate. | Core | the bulk of the work |
| **3** | Sweep support: `ExperimentConfig` needs a `playoutPolicies` axis; `ResolvedStack` must stop hardcoding `StandInPlayoutPolicy`. This is the *same* gap MEMORY.md records for the reconciler — the `stacks` schema exists in Python, `ExperimentConfig` does not honour it for playout. | `Teleop.Eval` | moderate |
| **4** | The experiment: `immediate` vs `fixed` at several budgets, predictor **held fixed**, one run per predictor. Per `Buffering/CLAUDE.md`'s interaction note, never vary both. | — | 2 sweeps |
| **5 (only if 4 wins)** | `PercentileTrackingPlayout.cs` — the NetEQ-style forgetting histogram at a configured quantile. | Core | ~40 lines + tests |
| **6 (only if 5 beats 4's best fixed budget)** | `KalmanJitterPlayout.cs`, then a genuine forecaster. | Core | ~80, then 200+ lines |

**Phases 5 and 6 are the candidate.** Phases 0–4 are prerequisites belonging to other people's axes,
and phase 4 answers the interesting question by itself.

## 7. The falsifier — cheapest experiment that could kill it

**Falsifier A (kills the *forecasting* claim; costs nothing and is already answered by reading):**
does any profile in the frozen suite have a delay process a forecaster can exploit? Answer: **no.**
Four parametric families are bounded-uniform white noise (optimal budget is the known constant
`Base + J`); `synthetic-burst` has regime persistence but two events per trial, both in the startup
transient, and *zero* seed variance; `leo-satellite` does not exist. A forecaster cannot beat
`Base + J` on a bounded uniform, and cannot be shown to beat anything on an n = 1 trace. **This
falsifier has already fired.**

**Falsifier B (the experiment worth actually running — it tests the surviving mechanism, not the
candidate):** after phases 0–3, run `immediate` vs `fixed` at budgets {0, 10, 20, 40, 80} ms,
predictor held at `double-exp`, reconciler `snap`, on `lan` / `50ms-5j` / `150ms-20j-0.5loss` /
`synthetic-burst`, ≥5 seeds. Report `prediction_position_error_mm` and `correction_magnitude_mm`
together with the `none`/`snap` baseline row present.
*Kill condition:* if `double-exp`'s p95 error does not fall materially at a budget of ~2× the
profile's jitter, then capture-order repair is worth nothing — and since capture-order repair is the
*only* mechanism by which a buffer helps here, the whole Buffering axis is worth less than its
wiring cost.
*The interesting shape either way:* the same sweep with `const-vel` should show **pure degradation**
(already order-tolerant, so it pays the staleness and gains nothing). A result where the buffer
helps one predictor and hurts the other is a tradeoff-shape finding, not a percentage.

**Falsifier C (the candidate's own, only if B passes):** does `percentile` at 0.95 beat the best
`fixed` budget from B on any profile? On bounded-uniform profiles it provably cannot beat `Base + J`,
so the only place it can win is `synthetic-burst` — where n = 1. **This is why I do not expect the
candidate to be decidable on the current profile suite at all.**

## 8. Is it distinguishable by measurement from something already implemented?

Nothing implements `IPlayoutPolicy`, so the only available comparison is against
`OperatorEndpoint`'s hardcoded `t_playout = t_operatorRecv`. What that can and cannot show:

- **Can show:** whether *any* buffering beats *no* buffering on `prediction_position_error_mm` and
  `correction_magnitude_mm` — specifically whether capture-order repair recovers `double-exp`'s p95
  tail (§5c). A genuine, large, currently-unmeasured effect.
- **Cannot show:** the latency the buffer costs. There is no metric derived from `t_playout`. Every
  comparison against the hardcoded stand-in will therefore be **one-sided**: all of the buffer's
  benefit visible, none of its cost. That is a trap, and any Buffering result must state it
  prominently or it will read as a free win. Reporting `PlayoutPolicyDiagnostics.DelayBudgetTicks`
  in the log next to the metrics is a partial mitigation and needs no new metric.
- **Cannot distinguish** `percentile` from a well-chosen `fixed` on any bounded-uniform profile:
  the optimum is a constant and both will find it. Measurement-equivalent there by construction.
- **Cannot distinguish** any two adaptive policies on `synthetic-burst` with meaningful confidence,
  because every seed replays the same 500 trace samples containing the same two bursts.

## 9. Bottom line

**Do not build the estimator/forecaster. Build `immediate` + `fixed` first, for a different reason
than this candidate proposed, and revisit estimation only if a real capture with forecastable
structure ever lands.**

On the frozen suite, delay is bounded-uniform white noise, so the optimal playout budget is a known
constant and there is nothing to estimate; the one profile with persistence structure has two events
per trial and zero seed variance; and the one profile advertised as *periodic* — `leo-satellite`,
the only case where forecasting could beat percentile-tracking on principle — does not exist.
Meanwhile the strongest published source (Zhang 2010) shows forecasting beating a *reactive filter*,
not a percentile tracker, and the reference production "adaptive" buffer (NetEQ) turns out to be a
percentile tracker with a decay. The candidate's distinctive ingredient is the one thing this
platform cannot currently reward.

## 10. How this assessment could flatter itself

1. **The §5c reordering result is the most exciting thing here and it is derived, not measured
   directly.** Capture ticks were *reconstructed* as `arrival − (owd_up + owd_down)` from a pooled
   CSV with no sequence column. The method is validated only by reporting exactly 0.0% on `lan`. If
   `ClockSync`'s conversion introduces noise correlated with delay, inversion counts are inflated.
   The clean check is `PredictorDiagnostics.RejectedObservations`, which exists in code and is
   emitted nowhere — someone should read it directly rather than trust my arithmetic.
2. **I ran no experiment.** Every number is from reading source or re-analyzing
   `results/exp-001-predictor-baseline/20260908-034948Z/` (git sha `31bf9e6`, **not** my seed sha
   `06c0ab5`). The `double-exp` rejection policy I quoted is from `06c0ab5`; the error table is from
   `31bf9e6`. **Different commits; I did not verify the predictor was unchanged between them.**
   Anyone acting on §5c must re-run.
3. **A negative verdict is the comfortable verdict for a survey agent.** "Do not build" costs me
   nothing and cannot be falsified by a failing test. The specific way this could be wrong: I
   assessed the candidate against the *frozen profile suite* and concluded the suite has no
   exploitable structure. A defender would say the suite is the problem, not the candidate — partly
   right. My honest position is that on this platform, with these profiles, the candidate is not
   decidable; that is a statement about the pairing, not a proof that forecasting is worthless.
4. **The in-flight-ring finding is dramatic and sanity-checked only once.** 1441 vs 1446 is strong,
   but the model has a semi-free parameter (63 versus 64 steps changes the prediction ~2%). The
   conclusion — eviction at scale on that profile — is robust to that; the exact 31% is not.
5. **I may have over-read `Buffering/CLAUDE.md` as endorsing my contract determination.** Its six
   planned rows are consistent with "estimator internal to the policy," but they are a planning
   table, not a decision record. Someone wanting a composable `IDelayEstimator` shared between a
   playout policy and an adaptive-redundancy controller would have a real argument, and my §1
   ranking dismissed adaptive redundancy partly on grounds (the codec sweep gap) that are contingent
   and fixable.
6. **I did not consider that a buffer's real beneficiary might be the reconciler, not the
   predictor.** `IReconciler.Observe` takes `predictedAtCapture`; out-of-order truth could produce
   spurious corrections. I reasoned about the predictor because that is where the explicit rejection
   code is, and may have missed a second mechanism on the reconciliation side.

## Verification

No gates run — no code was written, by instruction. Nothing under `core/`, `experiments/`,
`results/`, `unity/` or `robot/` was modified. No `git` state-changing command was issued. All
analysis of `results/` and `core/testdata/` was read-only. One temporary Python scratch file was
created inside the worktree for the §5c analysis; the worktree was destroyed and recreated around
that point, so **check for and delete `scratch_predanalysis.py` at the worktree root** if it
survived.

## Left undone / for a human

1. **The in-flight-ring censoring needs an owner.** It affects the Prediction and Reconciliation
   axes far more than Transport. Fixing it invalidates comparability with all existing `results/`,
   which is a human call.
2. **`synthetic-burst`'s zero seed variance** means `docs/metrics.md` §8 rule 3 cannot be satisfied
   on it. Either the trace should be offset per trial, or results on it should be labelled n = 1.
   The profile suite is frozen by ADR 0004, so this is a human call too.
3. **`Buffering/CLAUDE.md`'s `adaptive` row is mis-specified**: NetEQ's expand/contract actuator is
   waveform time-stretching, which has no analogue for a pose stream. The row should be rewritten or
   dropped. Not edited here (shared file, out of scope).
4. **`PredictorDiagnostics.RejectedObservations` should be emitted**, or at least read once, to
   confirm §5c directly rather than by reconstruction. That needs a metric definition in
   `docs/metrics.md`, which I may not touch.
5. **No metric derived from `t_playout` exists.** Any Buffering work will produce one-sided results
   until one is defined. That definition is a human's to write.
6. **Worktree instability:** this worktree became unreachable mid-run and the first copy of this log
   was lost. Copy this file out before the worktree is reaped.

---

## Organizer's note on provenance (added by research-organizer, not by the researcher)

This log was **recovered after its worktree had already been removed.** The organizer wrongly judged
this researcher dead — its transcript had been silent for 18 minutes and its log had stopped growing
mid-structure — and removed the worktree as part of end-of-run cleanup while it was still running.
The researcher lost its checkout, then rewrote this log in full into the leftover directory, and the
file was recovered from there intact. An earlier, truncated 402-line snapshot the organizer had
copied mid-run was discarded in favour of this complete version. The cleanup error is recorded in
`2026-09-09-network-transport-survey-decisions.md`.

**There are two independent assessments of this one candidate.** The other is
`2026-09-09-network-condition-estimation-feasibility-first-run.md`, from a first attempt the
organizer had also misjudged as dead. They were produced without knowledge of each other and
**agree** on the two determinations that matter: the contract determination (an implementation of
the existing `IPlayoutPolicy`, with an ADR for the `Pipeline/` wiring rather than for a new
interface), and the inelastic-sender answer (only the receive side has a knob). They also
independently reached the same negative on forecasting-versus-percentile-tracking, from different
literature.

They **disagree** on one point, left unreconciled deliberately: this log confirms the in-flight ring
censors OWD on `300ms-60j-2loss-bursty` and reports checking the prediction against a recorded
`metrics.csv` (predicted 1441 rows, recorded 1446), while the first-run log judged the 64-deep ring
"close but sufficient" on arithmetic alone. The organizer verified the *mechanism* — `InsertInFlight`
overwrites an occupied slot without checking `_inFlightOccupied`, and `TryReceiveState` skips the
frame before `ObserveRobotState` — but did not re-derive the rates. Treat the row-count agreement as
the stronger evidence and confirm it before acting.

Claims in this log that the organizer independently re-verified against the source at `06c0ab5`:
`leo-satellite` is in `ReservedPendingRealCapture` so `TryResolve` returns false;
`SyntheticTraceBuilder.BurstStartProbabilityPerSample = 0.01` is a memoryless Bernoulli trigger
despite that file's own doc calling the result "periodic"; and `Registries.PlayoutPolicies` is typed
without an `ITimeAuthority` that `PlayoutPolicyConfig.MaxAdaptationRatePerSecond` would require. All
hold. The `double-exp` reordering-rejection analysis is flagged by the researcher itself as resting
on reconstructed capture ticks and on a different sha (`31bf9e6`), and should be treated as a lead,
not a result.
