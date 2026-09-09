# 2026-09-09 — playout delay/loss bounds on `synthetic-burst`

**Agent:** none — run directly in the main session, by user instruction ("run the playout bounds
analysis"). **Worktree:** none. **Branch:** `buffering-playout-bounds-analysis`.
**HEAD at start:** `03f7e26`.

Artifact: `analysis/playout_bounds.py` (+ `analysis/tests/test_playout_bounds.py`). Pure Python,
reads one committed trace fixture, writes nothing. No Core change, no ADR, no new metric, no
`results/` directory.

## Why this ran

`2026-09-09-network-transport-survey-decisions.md` named this as the single cheapest next step and
predicted it would **close** the Buffering direction:

> delay on every parametric profile is bounded uniform, so the optimal budget is the known
> constant `Base + J` and there is nothing to estimate; the only persistent-structure profile
> yields two events per trial at n = 1

The reasoning was that if a well-chosen fixed budget is close to the theoretical optimum, an
adaptive policy has nothing to buy and the whole axis can be deprioritised. That is a good test and
it was worth running before anyone wrote a line of `Buffering/`.

## Hypothesis (written before any code)

**H:** on `synthetic-burst`, a fixed budget at matched late-loss is within ~15% of what a causal
adaptive policy achieves, so `percentile`/`adaptive`/`kalman-jitter` are not worth building.
**Falsifier:** an adaptive policy no more sophisticated than "max of the last w delays" beats a
matched-loss fixed budget by a margin too large to attribute to tuning.

**The falsifier fired, decisively.** H is rejected.

## Method

Three policies scored on the same 2000-sample trace, all on one curve (mean buffering delay
against late-arrival loss), because a delay number without its loss number is meaningless here:

- `oracle` — budget = the sample's own delay. Unachievable; the floor.
- `fixed` — one constant budget for the whole run.
- `adaptive` — budget = max of the previous `w` delays. Strictly causal: index `i` never reads
  `delays[i]` or later. Deliberately the crudest policy that could work, so the numbers are a
  **lower** bound on the adaptive family, not a best case.

Comparison is always at *matched loss*: the adaptive policy's achieved loss is measured first,
then the fixed budget is set to whatever constant produces the same loss.

## Results

```
trace: core/testdata/traces/synthetic-burst.trace  (2000 samples)
  p50 20.2   p95 171.5   p99 238.4   max 250.0 ms
  oracle floor: 31.9 ms mean budget at 0.00% loss

      loss      fixed   adaptive   saving
     2.30%   219.4 ms   116.6 ms      47%
     2.80%   209.8 ms    99.1 ms      53%
     4.05%   187.2 ms    78.4 ms      58%
     5.00%   171.5 ms    67.7 ms      61%
```

47–61% of buffering delay, at identical loss, from the least sophisticated causal policy available.
The mechanism is simple: bursts occupy ~7% of samples, and a fixed budget pays for them 100% of the
time.

### Burst structure — the number the question turns on

```
split at 86 ms, in a 128 ms empty band
  P(burst)         0.066
  P(burst | burst) 0.901
  13 episodes, 6-15 samples each
```

The survey read `SyntheticTraceBuilder.BurstStartProbabilityPerSample = 0.01` — a memoryless
Bernoulli draw — and concluded the process is unforecastable. That is true of the **onset** and
false of the **state**. A burst has a duration of 6–15 samples, so once one is under way the next
sample is elevated with probability 0.90. A causal policy does not need to predict onset; it needs
to notice it has started and to hold while it lasts. That is what the 47–61% is.

### A methodological error I made and corrected

The first version thresholded "elevated" at p90 and reported `P(elevated | elevated) = 0.608` over
78 episodes. That number is wrong — or rather, it measures the wrong thing. p90 on this trace is
21.8 ms, which sits *inside* the baseline cluster, so 63 of those 78 "episodes" were single-sample
baseline jitter and diluted the persistence estimate. The delay distribution is sharply bimodal
with a 128 ms empty band between 22.0 and 150.4 ms; splitting at the widest gap gives 13 real
episodes and 0.901. Recorded because the corrected number is *stronger* than the one I first
reported, and because a quantile threshold on a bimodal distribution will mislead the next person
the same way. `burst_threshold` is tested against a fixture where p90 lands in the baseline.

## Verification

`analysis/tests/test_playout_bounds.py`, 7 tests, hand-computable values per `analysis/CLAUDE.md` —
tick conversion, the strictly-greater-than late rule, causality of `adaptive_budgets` (it must not
read the current sample), the one-sample step-change lag that causality costs, the fixed-budget
quantile, gap-versus-quantile thresholding, and run-length counting including a trailing run.

`cd analysis && ./.venv/bin/python -m pytest -q` → **80 passed, 1 skipped**, with `test_gui.py` and
`tests/test_test_gui.py` excluded: this box has no `tkinter` (the GUI is a Windows-side tool per
`analysis/CLAUDE.md`). That collection error is pre-existing and unrelated. No `core/` gate was run
because no `core/` file was touched.

## How this result could flatter itself

Stated plainly, because three of these are load-bearing:

1. **One synthetic trace, generated by our own builder.** We own no real capture. This shows the
   mechanism is exploitable *when burst structure exists*; it does not show a real Tailscale path
   has that structure. Nothing here should be quoted as a property of the network.
2. **Scored over all 2000 samples; a sweep trial consumes 500.** The first 500 samples contain
   exactly 2 burst episodes — which *corroborates* the survey's "two events per trial". So the
   effect is real in the trace and simultaneously unmeasurable by the sweep as currently
   configured. This is a trial-length problem, not a trace problem, and it is the main thing
   standing between this analysis and a citable result.
3. **The fixed baseline is the honest one, and it is still the weakest link.** `fixed` is scored as
   the best possible constant *chosen with full knowledge of the trace* — a real deployment tunes
   it worse. So the 47–61% understates the gap against a realistically-tuned fixed policy, and
   overstates it against nothing.
4. **Late loss is not the same as packet loss.** These samples all arrive; the policy discards
   them for being late. Whether an operator prefers 100 ms less delay at 2.8% late-discard is a
   perceptual question this analysis cannot answer, and there is no metric for it.

## Left undone / for a human

- **Trial length.** Making this measurable needs `trialSteps` large enough to see ~13 episodes, or
  a shorter step interval. Both change what every existing `results/` directory means.
- **`Buffering/CLAUDE.md` requirement 1 is still unsatisfiable.** `PlayoutPolicyDiagnostics` never
  reaches `IMetricSink` and none of its fields are in `docs/metrics.md`, so a policy cannot report
  its operating point — the exact pair of numbers this analysis is built on. A buffering win would
  currently read as free, because `owd_*` is stamped before playout.
- **The "periodic" wording** in `docs/adr/0004` and `SyntheticTraceBuilder`'s XML doc is still
  wrong (onset is Bernoulli). It is also not the reason the survey's conclusion missed, so fixing
  the word does not fix the inference.
