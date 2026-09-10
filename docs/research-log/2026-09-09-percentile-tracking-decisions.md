# 2026-09-09 — decision record: `percentile`, and where tracking actually wins

**Organizer:** none — built directly in the main session, by user instruction ("now build
percentile"). **HEAD at start:** `967069c`. Implementation:
`core/Teleop.Core/Buffering/PercentileTrackingPlayout.cs`. Runs:
`results/exp-004-percentile-tracking/`, plus 15 scratch `fixed`-budget runs and two scratch window
runs under `results/scratch-*` (left in place — `results/` is append-only).

## The question

`exp-003` showed a `fixed` budget cannot be transferred: measured from capture, one value
degenerates to `immediate` on every profile whose one-way delay exceeds it. `percentile` derives its
budget from observed delay instead. Does deriving beat being told, and does the offline 47-61% from
`2026-09-09-playout-bounds-decisions.md` survive contact with the pipeline?

## The answer, and it has two halves

| profile family | result at matched loss |
|---|---|
| bursty / bimodal (`synthetic-burst`, `50ms-5j`) | `percentile` buffers **56-59% less** |
| bounded-uniform (`150ms-20j-0.5loss`, `300ms-60j-2loss-bursty`) | a per-profile-tuned `fixed` budget is slightly better on **both** axes |

**Both halves were predicted, by different documents, and each was right only about its own half.**
The transport survey said a bounded-uniform profile has an optimal constant `Base + J` and nothing
to estimate — true, and a tracker on a stationary uniform distribution is a noisy, lagging estimate
of a constant that was already known. The bounds analysis said a trace with persistent burst
structure is trackable — also true, and 56-59% brackets the 47-61% it predicted.

**The case for `percentile` is transferability, not dominance,** and that distinction is the whole
result. Every `fixed` budget that beat it was chosen by sweeping 15 budgets against that one
profile, which no deployment can do; the per-profile optima range from 60 ms to 350 ms, and the
160 ms that wins on `150ms-20j-0.5loss` is catastrophic on `300ms-60j-2loss-bursty`. `percentile` is
the only policy in the set that behaves sensibly on all five profiles without being told anything.

## Course changes mid-run

- **My first `fixed` curve had six points and produced a headline that was wrong.** At budgets
  {20, 60, 100, 150, 200, 300} ms, `percentile` appeared to beat `fixed` on *every* impaired
  profile, several by more than the offline analysis predicted. Refining to fifteen points found
  that `150ms-20j-0.5loss` has a cliff between 150 ms (8.78% loss) and 200 ms (0.00%) with 160 ms
  sitting in it at 0.65% loss and 10.9 ms mean — better than `percentile` on both axes. The coarse
  curve had bracketed the cliff instead of sampling it. I had the wrong conclusion written down
  before the refinement, and the only reason it was caught is that the six-point brackets were too
  wide to state a number honestly.
- **A large part of `percentile`'s measured loss turned out to be warm-up, not policy.** The window
  must fill before tracking starts, and 64 samples of a 500-step trial is 13% of it — spent at
  `playoutBudgetMs`, which the experiment reused from `fixed` at 40 ms, far below the base delay of
  three of the five profiles. Shrinking the window to 16 dropped loss from 3.32% to 0.99% on
  `150ms-20j-0.5loss` and 8.63% to 4.58% on `300ms-60j-2loss-bursty` at essentially unchanged mean
  delay. Measured rather than assumed, because the alternative reading — "tracking is simply worse
  here" — would have been a wrong verdict on the policy.

## Approaches considered and not pursued

- **A mean-plus-k-sigma or Kalman estimator instead of an order statistic.** Rejected on the data:
  the delay distribution this project owns is bimodal with a 128 ms empty band, and a mean of that
  sample sits inside the band describing a delay no packet ever had. An order statistic makes no
  shape assumption. `kalman-jitter` stays a separate planned row so that assumption is tested
  rather than smuggled in, and `OnABimodalDelayDistribution_...` pins the reasoning as a test.
- **Rate-limiting the budget with `MaxAdaptationRatePerSecond`** to satisfy the no-oscillation
  requirement. Rejected: a sliding-window order statistic has no feedback path — the budget never
  influences the delays it is estimated from — so it moves monotonically and settles within exactly
  `w` observations. Capping the rate could only add lag to a response that cannot overshoot. The
  requirement is met structurally and asserted as monotonicity plus a settling bound, not by a knob.
- **Re-scheduling already-buffered samples when the budget rises.** Rejected as scope: expanding a
  live buffer against occupancy is `adaptive`'s mechanism, and folding it in here would make two
  mechanisms unmeasurable separately. Each sample's playout instant is fixed at `Enqueue`.
- **Reusing `HistoryCapacity` as the delay window.** Rejected; `DelayWindowSamples` was added to
  `PlayoutPolicyConfig` instead. Capacity is constrained by the budget and the step interval, while
  the window sets responsiveness, and the two must be swept independently — the offline analysis
  moved 47% to 61% across windows of 64 down to 24, so pinning them together would have made the
  policy's main knob unreachable.
- **Excluding the arriving sample from its own window**, matching the offline analysis exactly.
  Rejected: including it is still strictly causal — the sample is in hand, nothing about the future
  is read — and it lets the first sample of a burst be caught by the rise it itself triggers. The
  offline form is the weaker one and its own caveats say so.
- **Tuning the experiment until `percentile` won everywhere.** The temptation was concrete: a
  smaller window, or a warm-up budget chosen per profile, would have done it. Not done, because the
  split between the two profile families is the finding.

## What was left open, and what is blocked on a human

- **The warm-up budget is shared with `fixed`'s.** `playoutBudgetMs` serves as both the constant for
  `fixed` and the pre-window value for `percentile`, and 40 ms is a bad choice for a 300 ms link —
  it guarantees a burst of late loss at every trial start. Splitting them is a config field; whether
  a deriving policy should instead start *pessimistic* (at `MaxDelayBudgetTicks`, safe but wasteful)
  is a design question worth deciding before it is coded.
- **Budget-swept stacks.** The 15-point `fixed` curve was produced by 15 separate scratch sweeps
  because `playoutBudgetMs` is a scalar. That is how a matched-loss comparison has to be made, so it
  should be first-class rather than a shell loop — already flagged in
  `2026-09-09-buffering-axis-first-policies-decisions.md`.
- **Window and percentile are unswept.** Only `w = 64/32/16` at `p = 1.0` were tried, and only as a
  warm-up diagnostic. `p` was never varied. A real `percentile` study varies one of them, and
  neither number here should be read as tuned.
- **500-step trials still bound everything.** ~2 burst episodes on `synthetic-burst`, and now also
  a 13% warm-up fraction at `w = 64`. Both shrink if the trial gets longer, and that changes what
  every existing `results/` directory means.
- **`unity/` still does not compile** until the two bridges are updated on the Windows box (from the
  previous PR). This change adds one argument to the `PlayoutPolicyConfig` in that proposal.

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

## Addendum, 2026-09-10: the reordering these numbers rest on is partly a harness artifact

`docs/research-log/2026-09-09-network-efficiency-decisions.md` instrumented `docs/metrics.md` §3
and found that **reordering is a joint property of jitter width and the sweep's poll interval, not
a property of the link.** On `jitter-5ms`, the same ±5 ms jitter produces a 19.7% / 13.0% / 0.0%
reorder rate at 5 / 10 / 20 ms steps: at a poll coarser than the jitter width, no two datagrams can
swap across a boundary at all.

That lands directly on this axis, because `immediate`'s late-arrival rate **is** the stream's
out-of-order rate by construction — that is how the policy is defined here. So on the parametric
jitter profiles, `immediate`'s measured loss is inflated by the 10 ms step this sweep happens to
use, and an operator running at a different frame rate would not see the same figure:

| profile | reorder rate at the 10 ms step | driven by |
|---|---|---|
| `50ms-5j` | 13.0% | ±5 ms jitter vs the poll — vanishes at a 20 ms step |
| `150ms-20j-0.5loss` | 43.9% | ±20 ms jitter vs the poll |
| `300ms-60j-2loss-bursty` | 70.1% | ±60 ms jitter vs the poll |
| `synthetic-burst` | 28.1% | the trace's 20 ms → 250 ms delay step |

**The headline is unaffected and the qualification is real.** The 56-59% matched-loss result rests
on `synthetic-burst`, where reordering comes from a ~230 ms delay step that inverts arrival order at
any poll interval, not from jitter narrower than the poll. What is qualified is the *size* of
`immediate`'s loss on the three parametric profiles — the conclusion there was that `fixed` at 40 ms
is byte-identical to `immediate` and that a tuned `fixed` budget beats `percentile`, and none of
that depends on the absolute rate.

Stated as a limit rather than a fix: step size was swept on `jitter-*`, not on `synthetic-burst`, so
"poll-independent" is an argument about the mechanism here, not a measurement. Sweeping it would
settle it.
