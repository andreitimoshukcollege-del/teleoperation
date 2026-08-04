from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pandas as pd
import pytest

ANALYSIS_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = ANALYSIS_DIR.parent
RUN_DIR = REPO_ROOT / "results" / "exp-001-predictor-baseline" / "20260804-020431Z"


@pytest.mark.skipif(
    not RUN_DIR.exists(),
    reason="committed exp-001-predictor-baseline result is not present in this checkout",
)
def test_cli_end_to_end_matches_hand_computed_percentile():
    result = subprocess.run(
        [sys.executable, "-m", "teleop_analysis.cli", str(RUN_DIR), "--figures", "table"],
        cwd=ANALYSIS_DIR,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr

    figures_dir = RUN_DIR / "figures"
    table_path = figures_dir / "summary_table.csv"
    assert table_path.exists()

    table = pd.read_csv(table_path)
    row = table[
        (table["profile"] == "lan")
        & (table["name"] == "owd_uplink_ms")
        & (table["stack"] == "none")
    ]
    assert len(row) == 1
    row = row.iloc[0]

    raw = pd.read_csv(RUN_DIR / "none" / "lan" / "metrics.csv")
    expected_p50 = raw.loc[raw["name"] == "owd_uplink_ms", "value"].quantile(0.5)
    assert row["p50"] == pytest.approx(expected_p50)
