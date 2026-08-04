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


@pytest.fixture
def synthetic_run_two_profiles(tmp_path) -> Path:
    """A tiny two-stack, two-profile sweep run, where both profiles ("lan", "50ms-5j") have a
    known scalar jitter value -- for testing figures that plot a metric against network jitter.
    """
    run_dir = tmp_path / "exp-998-synthetic-jitter" / "20260101-000000Z"
    manifest = {
        "experimentId": "exp-998-synthetic-jitter",
        "gitSha": "cafef00ddeadbeef1234",
        "seeds": [1, 2],
        "predictors": ["none", "fast"],
        "reconciler": "snap",
        "networkProfiles": ["lan", "50ms-5j"],
        "trialSteps": 10,
        "stepIntervalTicks": 100000,
        "configPath": "experiments/exp-998-synthetic-jitter.yaml",
        "machine": "TESTHOST",
        "command": "dotnet run --project core/Teleop.Eval -- sweep experiments/exp-998-synthetic-jitter.yaml",
        "generatedAtUtc": "2026-01-01T00:00:00Z",
    }
    run_dir.mkdir(parents=True)
    (run_dir / "manifest.json").write_text(json.dumps(manifest))

    for stack, scale in (("none", 1.0), ("fast", 0.5)):
        for profile in ("lan", "50ms-5j"):
            profile_dir = run_dir / stack / profile
            profile_dir.mkdir(parents=True)
            pd.DataFrame({
                "name": ["prediction_position_error_mm"] * 10 + ["correction_magnitude_mm"] * 10,
                "value": [v * scale for v in range(1, 11)] + [v * scale for v in range(11, 21)],
                "ticks": list(range(0, 1000, 100)) * 2,
            }).to_csv(profile_dir / "metrics.csv", index=False)

    return run_dir


@pytest.fixture
def synthetic_run_two_loss_profiles(tmp_path) -> Path:
    """A tiny two-stack, two-profile sweep run using the isolated "loss-<N>pct" family
    (docs/adr/0005-isolated-impairment-profiles.md) -- the legacy presets have no clean loss
    value, so the loss-axis figures need profiles from this family specifically.
    """
    run_dir = tmp_path / "exp-997-synthetic-loss" / "20260101-000000Z"
    manifest = {
        "experimentId": "exp-997-synthetic-loss",
        "gitSha": "1234deadbeefcafef00d",
        "seeds": [1, 2],
        "predictors": ["none", "fast"],
        "reconciler": "snap",
        "networkProfiles": ["loss-0pct", "loss-5pct"],
        "trialSteps": 10,
        "stepIntervalTicks": 100000,
        "configPath": "experiments/exp-997-synthetic-loss.yaml",
        "machine": "TESTHOST",
        "command": "dotnet run --project core/Teleop.Eval -- sweep experiments/exp-997-synthetic-loss.yaml",
        "generatedAtUtc": "2026-01-01T00:00:00Z",
    }
    run_dir.mkdir(parents=True)
    (run_dir / "manifest.json").write_text(json.dumps(manifest))

    for stack, scale in (("none", 1.0), ("fast", 0.5)):
        for profile in ("loss-0pct", "loss-5pct"):
            profile_dir = run_dir / stack / profile
            profile_dir.mkdir(parents=True)
            pd.DataFrame({
                "name": ["prediction_position_error_mm"] * 10 + ["correction_magnitude_mm"] * 10,
                "value": [v * scale for v in range(1, 11)] + [v * scale for v in range(11, 21)],
                "ticks": list(range(0, 1000, 100)) * 2,
            }).to_csv(profile_dir / "metrics.csv", index=False)

    return run_dir
