# 2026-09-09 — decision record: the Buffering axis does not close

**Organizer:** none — run directly in the main session, single-step, no delegation and no judge
panel. **Run type:** offline analysis of a committed trace fixture. **HEAD:** `03f7e26`.
Detail: `2026-09-09-playout-bounds.md`.

## The question

The network/transport survey (`2026-09-09-network-transport-survey-decisions.md`) recommended one
cheap next step and predicted its outcome: measure the gap between the theoretical optimum playout
delay and a well-chosen fixed budget, and if the gap is small, **close the Buffering direction**.

It was the right experiment to run and it returned the opposite answer.

## The decision

**Buffering is now the strongest remaining research axis, not a closed one.** On
`synthetic-burst`, the crudest causal adaptive policy imaginable — "budget = max of the last `w`
delays" — beats a matched-loss fixed budget by 47–61% of buffering delay. Building `fixed` and
`percentile` and comparing them is worth doing.

## Why the survey's prediction was wrong

Not an arithmetic error; an inference error, and worth naming because it is easy to repeat. The
survey read the trace generator's memoryless Bernoulli burst *onset* and concluded the delay
process is unforecastable. Onset is unforecastable. But a burst has a **duration** — 6–15 samples —
so `P(burst | burst) = 0.90`. A causal policy never needs to predict onset; it needs to detect that
one has begun and hold while it lasts. "Memoryless onset" and "no exploitable structure" are
different claims and only the first is true here.

The survey's other half stands unchallenged: on the *parametric* profiles delay really is bounded
uniform, the optimal budget really is the constant `Base + J`, and there really is nothing to
estimate. The reversal is specific to the trace-driven profile.

## Approaches considered and not pursued

- **Building a playout policy in the same change.** Rejected. `Pipeline/OperatorEndpoint.cs`
  hardcodes `t_playout = t_operatorRecv`, and replacing that is an architecture change requiring an
  ADR (`Buffering/CLAUDE.md`, `Pipeline/CLAUDE.md`). The point of an offline bound is to decide
  whether that ADR is worth writing. It now is; that is the next run, not this one.
- **Comparing against a smarter adaptive policy** (percentile tracker, NetEQ-style, Kalman).
  Deliberately not done. A crude policy that wins by 47% is a stronger argument than a tuned one
  that wins by 70%, because nobody can attribute the crude one's margin to tuning. The number is a
  lower bound on purpose.
- **Generating a new, genuinely-periodic trace family** to make the effect easier to see. Rejected
  as circular: a trace built to reward adaptation proves nothing about whether adaptation helps. It
  also needs an ADR. The existing fixture already carries the structure.
- **Adding the playout operating point to `docs/metrics.md`** so the result could be produced by a
  sweep rather than offline. Correct, and out of scope — metric definitions are the user's call
  (already open from the survey). It is why this had to be an offline analysis at all.
- **Extending `trialSteps` so a sweep could resolve this.** Rejected here: it changes what every
  existing `results/` directory means, which is a comparability decision belonging to a human.
  Recorded as the blocking item.
- **Running the judge panel.** Not applicable: one analysis, no candidates to rank, no sweep to
  audit, no contract to check.

## Course changes mid-run

- **My first burst-persistence number was wrong and I corrected it before writing anything down as
  a result.** I thresholded "elevated" at p90 and got `P = 0.608` over 78 episodes. p90 (21.8 ms)
  falls *inside* the baseline cluster, so most of those episodes were single-sample baseline
  jitter. Splitting at the widest gap in a sharply bimodal distribution gives 13 episodes and
  `P = 0.90`. The corrected figure is stronger, so the conclusion did not depend on the error — but
  a quantile threshold on a bimodal distribution is a trap and the code now has a test pinning it.

## What was left open, and what is blocked on a human

- **The effect is real in the trace and invisible to the sweep.** A 500-step trial contains exactly
  2 burst episodes, which is precisely what the survey said. Resolving it needs a longer trial or a
  finer step, and either changes existing results' comparability. **Blocked on a human.**
- **`Buffering/` requirement 1 — "report the operating point" — is unsatisfiable today.** The two
  numbers this entire analysis is built on (delay budget, induced late-loss) cannot be emitted:
  `PlayoutPolicyDiagnostics` never reaches `IMetricSink`, and `owd_*` is stamped before playout, so
  a buffer's whole latency cost is invisible. A buffering win would currently read as free.
  **Blocked on a human** (metric definitions).
- **We still own no real delay capture.** Everything above is conditional on a real link having
  burst structure at all. Nothing here should be quoted as a property of the network.
