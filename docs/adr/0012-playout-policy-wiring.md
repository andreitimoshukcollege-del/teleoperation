# 12. `OperatorEndpoint` gains a two-phase receive so `IPlayoutPolicy` can be wired in

## Status

Accepted.

## Context

`Buffering/` has been contracts-and-types only since the project started: `Contracts/IPlayoutPolicy.cs`,
`Types/PlayoutPolicyConfig.cs` and `Types/PlayoutPolicyDiagnostics.cs` are written, but the folder
holds no `.cs` implementation and `Registry/Registries.cs`'s `PlayoutPolicies` table is empty.
`Pipeline/OperatorEndpoint.cs` stands in for the missing axis with one line:

```csharp
.WithPlayoutTicks(arrivalTicks) // Phase-4 stand-in for IPlayoutPolicy -- see type doc.
```

`Pipeline/CLAUDE.md` says to replace that line rather than add around it once a real policy exists.

`docs/research-log/2026-09-09-playout-bounds-decisions.md` is why one now should. Offline
(`analysis/playout_bounds.py`), on `core/testdata/traces/synthetic-burst.trace`, a causal policy no
cleverer than "budget = max of the last `w` delays" holds the same late-loss rate as the best
possible constant budget while buffering **47-61% less**. That reverses the transport survey, which
had recommended closing this axis. Building `percentile`/`adaptive` is the point; this ADR is about
the wiring they all need first, and about the two baselines that make any of them interpretable.

The wiring is not mechanical, because a playout policy is the first thing in the pipeline that
**holds a sample across time**. Everything `OperatorEndpoint` does today happens in one pass:
`TryReceiveState` drains the transport, decodes, runs `ClockSync`, emits one-way-delay metrics,
stamps `t_playout`, folds the sample into the predictor/reconciler, and returns the completed
`LatencyTrace` — all at arrival. A buffer breaks that pass in half. The sample arrives at one
instant and is *used* at a later one, and `t_playout` is by definition the later one.

`IPlayoutPolicy`'s own doc already anticipates the shape: `TryDequeue` returns the released
sample's sequence "specifically so the pipeline can attribute the exact playout instant back to the
`LatencyTrace` for that sequence via `WithPlayoutTicks`."

## Decision

### 1. `TryReceiveState` splits into arrival work and playout work

`TryReceiveState(nowTicks, out LatencyTrace)` keeps everything that is genuinely about *arrival* —
decode, `ClockSync.AddRoundTrip`, `owd_uplink_ms`/`owd_downlink_ms` — and then `Enqueue`s the
sample into the policy instead of observing it. It no longer stamps `t_playout` and no longer calls
`ObserveRobotState`.

A new `TryPlayoutState(nowTicks, out LatencyTrace)` drains the policy: each `TryDequeue` releases a
sample, stamps `t_playout` on that sequence's trace, folds the sample into the predictor and
reconciler, and returns the now-complete trace. Same "call in a loop until false" shape as
`TryReceiveState` and `ITransport.TryReceive`.

Hosts call receive-then-playout-then-`EstimateRobotState` once per step. `docs/setup.md`'s
callback-placement table already puts the drain and the frame tick in that order; playout goes
between them.

### 2. A second ring holds traces between arrival and playout

The in-flight ring frees a slot when the reply arrives. With a buffer, the trace must survive from
arrival to release, so `OperatorEndpoint` gains a second fixed-capacity ring — same
insert-and-take-by-sequence shape, sized from the policy's `HistoryCapacity` — keyed by the
sequence `TryDequeue` hands back.

A trace evicted from that ring before its sample plays out is the same ordinary outcome as an
unmatched reply: the sample still reaches the predictor and reconciler, only the latency
bookkeeping is lost. This is deliberate, and it is the shape the in-flight ring was *fixed* into
(commit "fix the in-flight ring"): **state must never be censored by a bookkeeping structure.**

### 3. `immediate` and `fixed` first; `percentile` and `adaptive` after

`Buffering/CLAUDE.md` calls these two the baselines, and `docs/metrics.md` §8 rule 1 requires the
baseline in every comparison. An adaptive policy's whole claim is a *comparison* against the best
fixed budget at matched loss, so `fixed` is not optional scaffolding — it is the denominator of the
result. Neither of them adapts, which is also why they are the right pair to prove the wiring on:
any behaviour change they cause is attributable to the wiring, not to an estimator.

Both are thin policies over one shared `internal sealed PlayoutSampleBuffer`, on exactly the
reasoning `Reconciliation/DisplayedJerkEstimator.cs` records for itself: ordering, duplicate
rejection and late-arrival counting are the terms policies are *compared on*, so two copies would
agree in the common case and diverge in the corners, and the comparison would become one of buffer
implementations as much as of policies.

### 4. "Late" is defined by order, not by arithmetic on the budget

The obvious definition — a sample is late iff `t_recv > t_capture + budget` — is wrong for
`immediate`, whose budget is zero and against which every sample would score late.

The definition used instead: **a sample is late iff it can no longer be released in capture-time
order, i.e. its capture stamp is not newer than the last released sample's.** For a dense stream
the two coincide exactly (if a sample's delay exceeds the budget, the buffer has already released
something newer by the time it lands), so this is the same latency/loss curve
`analysis/playout_bounds.py` measures, stated in a way that also holds at zero budget and on a
sparse or bursty stream. A sample rejected because the buffer is full is counted the same way,
which is what `PlayoutPolicyConfig.HistoryCapacity`'s doc already says that field means.

### 5. "Underrun" is a drain that released nothing

`TryDequeue` returning false is explicitly "the common case and not an error" — it is how a drain
loop terminates — so it cannot itself be an underrun. Two candidate definitions were rejected:
*buffer is empty* scores `immediate` at 100% underrun, since a zero buffer is always empty by
construction; *buffer is non-empty but nothing due* scores `fixed` at 100%, since that is the
steady state it is designed to be in.

An underrun is therefore counted when `TryDequeue` returns false, **nothing has been released since
the previous false return, and the buffer is empty** — the pipeline asked and there was nothing to
have. Both halves are load-bearing: without the second, `fixed` scores an underrun on the step after
every release, because holding a not-yet-due sample looks identical to giving nothing. Both policies
score zero on a healthy stream and count exactly one per starved step.

### 6. Five new metric names, and `immediate` makes the buffer stage visible

`docs/metrics.md` §2's stage breakdown has always named `buffer` as a stage with no metric behind
it, and the survey's finding stands: `owd_*` is stamped at arrival, so a buffer's entire latency
cost is invisible and a buffering win would read as free. `playout_delay_ms` (`t_playout - t_recv`)
is that stage. `playout_budget_ms` and `playout_occupancy` are the operating point
`Buffering/CLAUDE.md` requirement 1 demands; `playout_late` and `playout_underrun` are its induced
loss. All five are defined in `docs/metrics.md` in the same change that first emits them, per
`IMetricSink`'s own rule.

### 7. The factory signature gains `ITimeAuthority` now, not later

`PlayoutPolicies` becomes `(PlayoutPolicyConfig, IMetricSink, ITimeAuthority) -> IPlayoutPolicy<Pose>`,
matching `Reconcilers` exactly. Neither policy in this change reads the clock.
`PlayoutPolicyConfig.MaxAdaptationRatePerSecond` is specified per second of wall time and is
unusable without one, so `adaptive` will need it — and `Registries.cs` is a named cross-machine
conflict file in root `CLAUDE.md`, so changing its shape twice costs more than carrying one unused
parameter. `Predictors` already records this exact tradeoff for `PassthroughPredictor`.

## Consequences

**`immediate` is not byte-identical to the stand-in it replaces, on a reordering link.** The
hardcoded line fed every decoded frame straight to the predictor in *arrival* order. `immediate`
enforces capture order per `IPlayoutPolicy` clause 2 and discards samples that arrive out of order.
Reordering is pervasive in existing runs even at `reorderProbability = 0.0`, because the emulator
delivers by earliest synthetic arrival — so this changes measured numbers on impaired profiles.

That is a fix, not a regression: today `ConstantVelocityPredictor` splices out-of-order samples in
while `DoubleExponentialPredictor` rejects them outright, so the predictor comparison has been
partly a comparison of two undocumented reordering policies. After this change, ordering is one
policy's job and every predictor sees the same stream. It is the second harness defect on the same
axis as the in-flight-ring censoring, and it is recorded here rather than fixed silently.

**Existing `results/` directories are not comparable to new ones on any profile where reordering
occurs.** `ResolvedStack.StandInPlayoutPolicy` and `analysis/teleop_analysis/manifest.py`'s
`LEGACY_PLAYOUT_POLICY` both record the string `immediate` for every pre-Buffering run, which now
names a real registry key with different behaviour. Both constants are renamed to
`legacy-inline-playout` so a manifest cannot claim a run used a policy that did not exist when it
ran. Old manifests on disk are untouched and keep reading correctly through `manifest.py`'s
existing legacy path; only the label changes.

**The `owd_*` metrics keep their current meaning.** They are arrival-side and are emitted from
`TryReceiveState` exactly as before. Nothing about this change moves them, which is what keeps the
buffer's cost a separate, addable stage rather than a silent inflation of one-way delay.

**A host that forgets to call `TryPlayoutState` sees a frozen robot estimate.** Nothing is folded
into the predictor any more at arrival. This is the same failure mode as forgetting
`EstimateRobotState`, which cost this project the entire Reconciliation axis being unmeasurable by
sweep — so both are asserted by `LoopbackPipelineIntegrationTests` rather than left to a comment.

**This does not make the 47-61% result reproducible by sweep.** A 500-step trial contains ~2 burst
episodes on `synthetic-burst`, so the sweep still cannot resolve an adaptive policy's advantage.
Fixing that means a longer trial or a finer step, which changes what every existing `results/`
directory means, and remains a human's decision. This ADR builds the apparatus; it does not settle
that question.
