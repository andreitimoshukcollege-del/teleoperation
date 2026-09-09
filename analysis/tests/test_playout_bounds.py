from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from playout_bounds import (  # noqa: E402
    adaptive_budgets,
    burst_threshold,
    episodes,
    fixed_budget_for_loss,
    load_delays_ms,
    score,
)


def test_load_delays_converts_ticks_to_ms(tmp_path):
    trace = tmp_path / "t.trace"
    trace.write_text("TRACE|1|10000000\n100000\n250000\n\n")
    assert load_delays_ms(trace) == [10.0, 25.0]


def test_score_counts_only_delays_strictly_over_budget():
    # budgets 10/10/10 against delays 5/10/15 -> mean budget 10, one late out of three.
    mean_budget, loss = score([10.0, 10.0, 10.0], [5.0, 10.0, 15.0])
    assert mean_budget == 10.0
    assert loss == pytest.approx(100 / 3)


def test_adaptive_budgets_never_read_the_current_sample():
    # window 2: [d0, max(d0), max(d0,d1), max(d1,d2)] -- index i only ever sees i-1 and earlier.
    assert adaptive_budgets([1.0, 9.0, 2.0, 3.0], window=2) == [1.0, 1.0, 9.0, 9.0]


def test_adaptive_budget_lags_a_step_change_by_one_sample():
    """The cost of being causal, made explicit: the first sample of a burst is always late."""
    delays = [10.0] * 4 + [100.0] * 4
    budgets = adaptive_budgets(delays, window=4)
    assert budgets[4] == 10.0  # the step is not yet visible
    assert budgets[5] == 100.0  # and is fully absorbed one sample later
    assert score(budgets, delays)[1] == pytest.approx(12.5)  # exactly 1 of 8 late


def test_fixed_budget_for_loss_picks_the_matching_quantile():
    delays = [float(v) for v in range(1, 101)]
    assert fixed_budget_for_loss(delays, 0.0) == 100.0
    assert fixed_budget_for_loss(delays, 10.0) == 91.0


def test_burst_threshold_splits_the_widest_gap_not_a_quantile():
    """Baseline cluster near 20, burst cluster near 200. A p90 cut would land inside the
    baseline (this is the mistake the docstring warns about); the gap cut lands between."""
    delays = [20.0 + i / 10 for i in range(19)] + [200.0]
    threshold, gap = burst_threshold(delays)
    assert 21.8 < threshold < 200.0
    assert gap == pytest.approx(178.2)
    # p90 lands inside the baseline cluster, well below the real split.
    assert sorted(delays)[int(0.9 * len(delays))] < threshold


def test_episodes_counts_maximal_runs_including_a_trailing_one():
    assert episodes([False, True, True, False, True, False, True, True]) == [2, 1, 2]
    assert episodes([False, False]) == []
