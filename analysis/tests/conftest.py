from __future__ import annotations

import json
from pathlib import Path

import pandas as pd
import pytest


@pytest.fixture
def synthetic_run(tmp_path) -> Path:
    """A tiny two-stack, one-profile sweep run: hand-computable values, no seed column
    (mirrors the pre-Phase-6 metrics.csv format actually committed under results/).
    """
    run_dir = tmp_path / "exp-999-synthetic" / "20260101-000000Z"
    manifest = {
        "experimentId": "exp-999-synthetic",
        "gitSha": "deadbeefcafef00d1234",
        "seeds": [1, 2],
        "predictors": ["none", "fast"],
        "reconciler": "snap",
        "networkProfiles": ["lan"],
        "trialSteps": 10,
        "stepIntervalTicks": 100000,
        "configPath": "experiments/exp-999-synthetic.yaml",
        "machine": "TESTHOST",
        "command": "dotnet run --project core/Teleop.Eval -- sweep experiments/exp-999-synthetic.yaml",
        "generatedAtUtc": "2026-01-01T00:00:00Z",
    }
    run_dir.mkdir(parents=True)
    (run_dir / "manifest.json").write_text(json.dumps(manifest))

    none_dir = run_dir / "none" / "lan"
    none_dir.mkdir(parents=True)
    pd.DataFrame({
        "name": ["prediction_position_error_mm"] * 10 + ["correction_magnitude_mm"] * 10,
        "value": [float(v) for v in range(1, 11)] + [float(v) for v in range(11, 21)],
        "ticks": list(range(0, 1000, 100)) * 2,
    }).to_csv(none_dir / "metrics.csv", index=False)

    fast_dir = run_dir / "fast" / "lan"
    fast_dir.mkdir(parents=True)
    pd.DataFrame({
        "name": ["prediction_position_error_mm"] * 10 + ["correction_magnitude_mm"] * 10,
        "value": [v / 2 for v in range(1, 11)] + [v / 2 for v in range(11, 21)],
        "ticks": list(range(0, 1000, 100)) * 2,
    }).to_csv(fast_dir / "metrics.csv", index=False)

    return run_dir
