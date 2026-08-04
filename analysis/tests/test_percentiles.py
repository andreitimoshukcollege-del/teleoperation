from __future__ import annotations

import pandas as pd

from teleop_analysis import percentiles


def test_summarize_matches_hand_computed_median():
    df = pd.DataFrame({"stack": ["a"] * 10, "value": list(range(1, 11))})
    table = percentiles.summarize(df, ["stack"])
    row = table.iloc[0]
    assert row["stack"] == "a"
    assert row["p50"] == 5.5
    assert row["n"] == 10
    assert row["p50"] <= row["p95"] <= row["p99"]


def test_summarize_empty_input_returns_empty_frame_with_expected_columns():
    df = pd.DataFrame(columns=["stack", "value"])
    table = percentiles.summarize(df, ["stack"])
    assert list(table.columns) == ["stack", "p50", "p95", "p99", "n"]
    assert table.empty


def test_summarize_groups_independently():
    df = pd.DataFrame({
        "stack": ["a", "a", "a", "b", "b", "b"],
        "value": [1, 2, 3, 10, 20, 30],
    })
    table = percentiles.summarize(df, ["stack"]).set_index("stack")
    assert table.loc["a", "p50"] == 2
    assert table.loc["b", "p50"] == 20


def test_summarize_never_calls_mean_no_such_column_by_construction():
    # No "mean" column exists in the output -- this is a structural guard against
    # accidentally reintroducing a mean-only aggregation later.
    df = pd.DataFrame({"stack": ["a"] * 3, "value": [1, 2, 3]})
    table = percentiles.summarize(df, ["stack"])
    assert "mean" not in table.columns
