# 2026-09-09 — decision record: building the Buffering axis, first two policies

**Organizer:** none — built directly in the main session, by user instruction ("now start building
the buffering axis"). No delegation, no judge panel: one implementation task with a known shape,
not a question with competing answers. **HEAD at start:** `9dfa728`.

Argument and alternatives: `docs/adr/0012-playout-policy-wiring.md`. This file records what was
decided and what the first sweep actually said.

## Why now

`2026-09-09-playout-bounds-decisions.md` reopened this axis: offline, a causal adaptive policy
buffers 47-61% less than the best possible constant budget at matched loss. The axis had contracts
and types but no implementation and no wiring, so nothing could be built on that finding without
first making `IPlayoutPolicy` reachable from `Pipeline/`.

## What was built, and what deliberately was not

`immediate` and `fixed` — the two baselines — over one shared `PlayoutSampleBuffer`, plus the
two-phase receive they need, five metric names, sweep support, and `exp-003-playout-baselines`.

**`percentile`/`adaptive` were not built, and that is the whole point of stopping here.** An
adaptive policy's claim is "same loss, less delay" — a *comparison*, whose denominator is the best
fixed budget. Building the adaptive one first would have produced a number with nothing to state it
against. `fixed` is not scaffolding on the way to the interesting policy; it is the interesting
policy's y-axis.

## Approaches considered and not pursued

- **Keeping a backward-compatible `OperatorEndpoint` constructor** so `unity/` would still compile.
  Rejected twice over. It would have to default the policy internally, which contradicts this
  class's own stated rule that the zero-mitigation configuration must be visible at the call site.
  Worse, it would compile and then fail *silently*: a host that never calls `TryPlayoutState` feeds
  the predictor nothing at all. A compile error is the honest failure here, and the two Unity call
  sites are proposed for the Windows box rather than edited from this one.
- **Defining "late" as `t_recv > t_capture + budget`.** The obvious arithmetic, and wrong: at a zero
  budget it scores `immediate` at 100% late. Replaced with an order-based rule that coincides with
  it on a dense stream — ADR §4.
- **Two simpler underrun rules**, each of which scores one baseline at 100% on a healthy stream.
  The surviving rule needs *both* halves and it took a failing test to find the second — ADR §5.
- **A second ring for traces held between arrival and playout.** Planned in the ADR, then dropped
  during implementation: keeping the trace in its existing in-flight slot until playout does the
  same job with no new parameter and no second eviction policy to reason about. It did surface one
  real consequence — a duplicate reply now finds the slot still occupied, where before the slot was
  freed on arrival, so `TryReceiveState` gained an explicit duplicate guard. Without it a duplicate
  would have fed `ClockSync` twice and emitted a second pair of `owd_*` samples.
- **Sweeping `playoutBudgetMs` as a list.** `fixed` is a family of operating points, not one, so the
  curve is the right artifact. Deferred: it means budget-named stacks and a schema change, and the
  first sweep's job was to prove the wiring, not to produce a citable curve.
- **Tuning the budget per profile so exp-003 looked better.** Explicitly rejected — see below.
- **A new trace family with forecastable structure.** Same rejection as in the bounds run: a trace
  built to reward adaptation proves nothing about adaptation.

## Course changes mid-run

- **I hoisted a `ClockSync.ToOperatorTicks` call above `AddRoundTrip` while restructuring, and it
  silently changed a converted timestamp** — the conversion reads the current offset estimate, so
  doing it before the round trip is folded in uses the previous one. Caught by an existing ADR-0008
  test asserting an exact rescaled stamp. The conversion is now duplicated into both branches, with
  a comment saying why it must not be shared. Worth recording because the refactor looked purely
  mechanical and was not.
- **Three of my first Buffering tests asserted the wrong thing, not the code.** Two were my own
  misreading of the underrun rule; the third built a stream where a sample arrived before it was
  captured. The underrun one was worth the trouble: fixing the test exposed that the rule as
  written counted `fixed`'s designed steady state as starvation.

## What the first sweep actually said

`exp-003-playout-baselines`, 2 predictors x 2 policies x 5 profiles x 5 seeds, 40 ms budget.

**The mechanism works and is measured.** On `synthetic-burst`, `fixed` played out 2355 samples
against `immediate`'s 1885 — it recovers 470 reordered samples `immediate` must discard — and
charges ~20 ms p50 `playout_delay_ms` for them. That is the latency/loss trade, in a sweep, for the
first time.

**And a fixed budget below the link's one-way delay does nothing at all.** The budget is measured
from capture, so on `50ms-5j`, `150ms-20j-0.5loss` and `300ms-60j-2loss-bursty` every sample already
arrives past its own due instant and is released on sight: `fixed` at 40 ms is byte-identical to
`immediate` on all three, down to the late and underrun counts. This is the textbook property of a
fixed playout budget rather than a defect, but it has a sharp consequence — **one budget cannot be
shared across profiles with different base delays**, so a single-budget comparison across a profile
suite is not a comparison of policies.

I left the budget at 40 rather than tuning it per profile, because tuning it would hide precisely
that. It is also the strongest available argument for `percentile` as the next row: a policy that
*derives* its budget from observed delay is the direct answer to a budget that cannot be transferred.

## What was left open, and what is blocked on a human

- **`unity/` does not compile until two call sites are updated on the Windows box.**
  `TeleopOperatorBridge.cs` and `JetRoverOperatorBridge.cs` each need the new constructor argument
  and a `TryPlayoutState` drain. Proposed in the PR, not edited from here (root CLAUDE.md).
- **Existing `results/` are not comparable to new ones on any profile where reordering occurs.**
  `immediate` enforces capture order; the inline stand-in it replaced did not, and reordering is
  pervasive even at `reorderProbability = 0`. Whether to re-run anything is a human's call. The
  manifest label `legacy-inline-playout` marks which side of the line a run is on.
- **The 47-61% adaptive result is still not reproducible by sweep**, for the reason the bounds run
  gave: ~2 burst episodes per 500-step trial. Unchanged by this work, and still a trial-length
  decision that moves every existing result's meaning.
- **Budget-swept stacks** are the next schema change, and they are what turns `fixed` from a row
  into the curve `percentile` and `adaptive` are judged against.

## Addendum, 2026-09-10: re-verified after ADR 0013

`docs/adr/0013-composable-network-impairments.md` rebuilt the impairment pipeline in Core and
warns that "two runs of `300ms-60j-2loss-bursty` at the same seed, one before and one after this
change, produce different per-datagram outcomes." Every number above was measured before it.

The ADR reasons that this comparability break is "worth zero today, because no recorded result
exists" -- true of the repository, since `results/` is gitignored, but **not** true of this log,
which cites numbers from runs that exist only on the Linux box. That gap is invisible from the
machine that wrote the ADR. So the experiments were re-run rather than assumed to hold.

**Every conclusion survives, and the drift is exactly the shape the ADR predicts.**

- `lan` and `synthetic-burst` are **identical to the digit**, before and after. Neither consumes a
  parametric impairment draw -- `lan` has no impairment at all, and `synthetic-burst` is
  trace-driven with base delay, jitter, loss and reorder all zeroed (which is also why the network
  survey found its five seeds inert). There is no realization for the rewrite to change.
- The three parametric profiles moved by under one percentage point of loss and under 0.3 ms of
  mean buffering delay: `immediate` on `50ms-5j` 2.78% -> 3.20%, on `150ms-20j-0.5loss`
  22.08% -> 22.72%, on `300ms-60j-2loss-bursty` 57.06% -> 56.63%; `percentile` 3.32% -> 2.84% and
  8.63% -> 8.44% at unchanged delay. Same distribution, different realization.
- Every **structural** claim holds unchanged, because none of them depended on a realization:
  `fixed` at 40 ms is still byte-identical to `immediate` wherever the budget is below the link's
  one-way delay, and the matched-loss headline rests on `synthetic-burst`, which did not move.

Nothing here is retracted. The one thing worth carrying forward: the ADR's "worth zero exactly
once" only stays true if local, gitignored results are counted, and from the other machine they
cannot be seen. A committed number is a recorded result even when the run behind it is not.

## Addendum, 2026-09-10: the reordering these numbers rest on is a harness artifact — and I first
## described the mechanism wrongly

**Correction first.** An earlier version of this addendum said reordering is "a joint property of
jitter width and the harness's poll schedule", implying the polling permutes delivery. That is
wrong. `EmulatedTransport` delivers in synthetic-arrival order at *any* poll rate; a coarser poll
batches arrivals into one drain but the heap still pops them in arrival order. Polling reorders
nothing at the transport.

The mechanism is one layer up and is now **measured rather than argued**.
`net_roundtrip_reorder_displacement` was split into three attributed quantities
(`docs/research-log/2026-09-10-harness-repair-decisions.md`), and on `jitter-5ms` at the 10 ms step
this sweep uses:

| | total | uplink | downlink | batched |
|---|---|---|---|---|
| `jitter-5ms`, 10 ms step | 327 | **0** | **0** | 620 |

Neither leg inverted a single datagram. Every inversion came from `RobotEndpoint.Step` replying to
every command drained in one poll with the same `nowTicks`: a batch leaves with identical send
stamps and independent downlink jitter then shuffles it. (620 frames shared a send stamp with the
arrival before them; 327 of those pairs actually came out inverted.) At a 20 ms step nothing lands
in the same poll and all four counts are zero, which is why the rate looked poll-dependent.

**What this means for the numbers above is unchanged from the earlier addendum, and the reason is
now firmer.** `immediate`'s late-arrival rate *is* the stream's out-of-order rate by construction,
so on the parametric jitter profiles its measured loss is inflated by an artifact of the harness's
reply batching, not by anything the link did:

| profile | reorder rate at the 10 ms step | attributable to |
|---|---|---|
| `50ms-5j` | 13.0% | robot reply batching, entirely |
| `150ms-20j-0.5loss` | 43.5% | batching plus both transit legs |
| `300ms-60j-2loss-bursty` | 70.1% | batching plus both transit legs |
| `synthetic-burst` | 28.1% | the trace's 20 ms → 250 ms delay step |

**The 56-59% headline is unaffected.** It rests on `synthetic-burst`, where a ~230 ms delay step
inverts arrival order regardless of poll interval or batching. What is qualified is the *size* of
`immediate`'s loss on the three parametric profiles — and the conclusion there was that `fixed` at
40 ms is byte-identical to `immediate` and that a tuned `fixed` budget beats `percentile`, neither
of which depends on the absolute rate.

Step size was swept on `jitter-*`, not on `synthetic-burst`, so poll-independence there remains an
argument about the mechanism rather than a measurement.
