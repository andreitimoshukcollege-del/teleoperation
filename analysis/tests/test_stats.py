from __future__ import annotations

import pandas as pd

from teleop_analysis import stats


def test_seed_spread_returns_none_without_seed_column():
    df = pd.DataFrame({"stack": ["a"], "profile": ["lan"], "name": ["m"], "value": [1.0]})
    assert stats.seed_spread(df, ["stack", "profile", "name"]) is None


def test_seed_spread_computed_when_seed_present():
    df = pd.DataFrame({
        "stack": ["a"] * 4,
        "profile": ["lan"] * 4,
        "name": ["m"] * 4,
        "seed": [1, 1, 2, 2],
        "value": [10.0, 10.0, 20.0, 20.0],
    })
    spread = stats.seed_spread(df, ["stack", "profile", "name"])
    assert spread is not None
    assert spread.iloc[0]["p50_seed_spread"] == 10.0


def test_compare_to_baseline_states_the_assumption():
    df = pd.DataFrame({
        "profile": ["lan"] * 8,
        "name": ["m"] * 8,
        "stack": ["baseline"] * 4 + ["other"] * 4,
        "value": [1.0, 2.0, 3.0, 4.0, 10.0, 20.0, 30.0, 40.0],
    })
    result = stats.compare_to_baseline(df, baseline_stack="baseline")
    assert len(result) == 1
    row = result.iloc[0]
    assert row["stack"] == "other"
    assert "Mann-Whitney" in row["assumption"]
