#!/usr/bin/env python3
"""Bound what a playout buffer could buy, before anyone builds one.

`Buffering/` has contracts and config types but no implementation, and wiring `IPlayoutPolicy`
into `Pipeline/` is an architecture change needing an ADR. This answers the cheaper question
first: on the delay data we actually own, how much is there to win, and does an *adaptive*
policy beat a well-chosen *fixed* one by enough to justify the work?

Deliberately outside `teleop_analysis/`: that package reads `results/`, and this reads a
committed trace fixture. It is data, not Core behaviour -- no C# is read, no metric recomputed.

    ./analysis/.venv/bin/python analysis/playout_bounds.py

Three policies, all scored on the same trace:

  oracle    per-sample budget equal to the sample's own delay. Unachievable -- it requires
            knowing the future -- but it is the floor any real policy is measured against.
  fixed     one constant budget for the whole run. What `fixed` in Buffering/CLAUDE.md would be.
  adaptive  budget = max of the last w delays, using only samples already received. Deliberately
            the crudest causal policy that could work, so its result is a *lower* bound on what
            an adaptive family achieves -- `percentile` or `adaptive` (NetEQ-style) in that same
            table have strictly more to work with.

A sample whose delay exceeds its budget arrives too late to be played and counts as loss. The
tradeoff is therefore buffering delay against late-loss rate, and policies are only comparable
at matched loss.
"""

import pathlib
import statistics

REPO = pathlib.Path(__file__).resolve().parent.parent
TRACE = REPO / "core" / "testdata" / "traces" / "synthetic-burst.trace"

# One sweep trial consumes this many samples (experiments/*.yaml `trialSteps`), which is far
# fewer than the trace holds. Reported because it bounds what a sweep could currently observe.
TRIAL_STEPS = 500


def load_delays_ms(path):
    """One-way delays in milliseconds. Format is `TRACE|<version>|<ticksPerSecond>` then one
    tick count per line -- see core/Teleop.Eval/Sweep/TraceFile.cs."""
    lines = path.read_text().splitlines()
    ticks_per_second = int(lines[0].split("|")[2])
    return [int(v) / ticks_per_second * 1000.0 for v in lines[1:] if v.strip()]


def score(budgets, delays):
    """Mean buffering delay, and the percentage of samples arriving after their budget."""
    late = sum(1 for budget, delay in zip(budgets, delays) if delay > budget)
    return statistics.mean(budgets), late / len(delays) * 100.0


def adaptive_budgets(delays, window):
    """Causal: sample i is budgeted by the largest delay seen in the previous `window` samples.
    Never reads delays[i] or later, which is what makes it implementable."""
    return [max(delays[max(0, i - window):i]) if i else delays[0] for i in range(len(delays))]


def fixed_budget_for_loss(delays, loss_percent):
    """The smallest constant budget achieving at most `loss_percent` late loss."""
    ordered = sorted(delays)
    return ordered[min(len(ordered) - 1, int((1.0 - loss_percent / 100.0) * len(ordered)))]


def burst_threshold(delays):
    """Split baseline from burst at the midpoint of the widest gap in the sorted delays.

    A quantile does not work here and choosing one is how this analysis first went wrong: at p90
    the "elevated" set is mostly single-sample baseline jitter, which drags the persistence
    estimate down and understates the structure. The distribution is sharply bimodal -- a
    baseline cluster and a burst cluster with a wide empty band between -- so the gap locates the
    split without a tuned constant.
    """
    ordered = sorted(delays)
    gap, low, high = max(
        (ordered[i + 1] - ordered[i], ordered[i], ordered[i + 1]) for i in range(len(ordered) - 1)
    )
    return (low + high) / 2.0, gap


def episodes(flags):
    """Lengths of the maximal runs of True -- one entry per burst."""
    lengths, run = [], 0
    for flag in flags:
        if flag:
            run += 1
        elif run:
            lengths.append(run)
            run = 0
    return lengths + ([run] if run else [])


def main():
    delays = load_delays_ms(TRACE)
    ordered = sorted(delays)
    n = len(delays)

    def q(p):
        return ordered[min(n - 1, int(p * n))]

    print(f"trace: {TRACE.relative_to(REPO)}  ({n} samples)")
    print(f"  p50 {q(.50):.1f}   p95 {q(.95):.1f}   p99 {q(.99):.1f}   max {max(delays):.1f} ms")

    threshold, gap = burst_threshold(delays)
    in_burst = [d > threshold for d in delays]
    lengths = episodes(in_burst)
    pairs = sum(1 for a, b in zip(in_burst, in_burst[1:]) if a and b)

    print(f"\nburst structure (split at {threshold:.0f} ms, in a {gap:.0f} ms empty band)")
    print(f"  P(burst)         {sum(in_burst) / n:.3f}")
    print(f"  P(burst | burst) {pairs / max(1, sum(in_burst[:-1])):.3f}")
    print(f"  {len(lengths)} episodes, {min(lengths)}-{max(lengths)} samples each")
    print("  -> onset is memoryless, but a burst *persists*, and persistence is what a causal")
    print("     policy tracks. Forecasting the onset is not required and is not claimed here.")
    print(f"  but only {len(episodes(in_burst[:TRIAL_STEPS]))} of them fall in the first "
          f"{TRIAL_STEPS} samples, which is all one sweep trial consumes")

    oracle_delay, oracle_loss = score(delays, delays)
    print(f"\noracle floor: {oracle_delay:.1f} ms mean budget at {oracle_loss:.2f}% loss")

    print("\nfixed vs adaptive, compared at matched late-loss")
    print(f"  {'loss':>8} {'fixed':>10} {'adaptive':>10} {'saving':>8}")
    for window in (64, 48, 32, 24):
        a_delay, a_loss = score(adaptive_budgets(delays, window), delays)
        f_delay, _ = score([fixed_budget_for_loss(delays, a_loss)] * n, delays)
        print(f"  {a_loss:7.2f}% {f_delay:7.1f} ms {a_delay:7.1f} ms "
              f"{(1 - a_delay / f_delay) * 100:7.0f}%")

    print(
        "\nA fixed budget must permanently pay for a burst that is present ~7% of the time.\n"
        "Caveats that bound this result:\n"
        "  - One synthetic trace from our own generator, not a real capture. It shows the\n"
        "    mechanism is exploitable, not that a real link behaves this way.\n"
        "  - `max of last w` is the crudest causal policy, so the adaptive column is a lower\n"
        "    bound, and the window is not tuned per trace.\n"
        "  - Scored over all 2000 samples. A 500-step trial sees ~2 episodes, so a sweep as\n"
        "    currently configured could not resolve this difference; that is a trial-length\n"
        "    problem, not a trace problem.\n"
        "  - Late loss is unmeasurable in the pipeline today: no playout policy is wired, and\n"
        "    owd_* is stamped before playout would occur."
    )


if __name__ == "__main__":
    main()
