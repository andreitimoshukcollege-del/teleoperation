from __future__ import annotations

import json
from pathlib import Path

import pandas as pd
import pytest


def _write_manifest(run_dir: Path, **overrides) -> None:
    manifest = {
        "gitSha": "deadbeefcafef00d1234",
        "seeds": [1, 2],
        "predictors": ["none", "fast"],
        "reconciler": "snap",
        "trialSteps": 10,
        "stepIntervalTicks": 100000,
        "machine": "TESTHOST",
        **overrides,
    }
    manifest.setdefault("configPath", f"experiments/{manifest['experimentId']}.yaml")
    manifest.setdefault(
        "command", f"dotnet run --project core/Teleop.Eval -- sweep {manifest['configPath']}"
    )
    manifest.setdefault("generatedAtUtc", "2026-01-01T00:00:00Z")
    run_dir.mkdir(parents=True)
    (run_dir / "manifest.json").write_text(json.dumps(manifest))


def _write_metrics_csv(profile_dir: Path, scale: float) -> None:
    """10 prediction_position_error_mm rows (1..10) + 10 correction_magnitude_mm rows (11..20),
    each multiplied by `scale` -- hand-computable values, no seed column (mirrors the
    pre-Phase-6 metrics.csv format actually committed under results/).
    """
    profile_dir.mkdir(parents=True)
    pd.DataFrame({
        "name": ["prediction_position_error_mm"] * 10 + ["correction_magnitude_mm"] * 10,
        "value": [v * scale for v in range(1, 11)] + [v * scale for v in range(11, 21)],
        "ticks": list(range(0, 1000, 100)) * 2,
    }).to_csv(profile_dir / "metrics.csv", index=False)


@pytest.fixture
def synthetic_run(tmp_path) -> Path:
    """A tiny two-stack, one-profile sweep run: hand-computable values, no seed column."""
    run_dir = tmp_path / "exp-999-synthetic" / "20260101-000000Z"
    _write_manifest(run_dir, experimentId="exp-999-synthetic", networkProfiles=["lan"])

    _write_metrics_csv(run_dir / "none" / "lan", scale=1.0)
    _write_metrics_csv(run_dir / "fast" / "lan", scale=0.5)

    return run_dir


@pytest.fixture
def synthetic_run_two_profiles(tmp_path) -> Path:
    """A tiny two-stack, two-profile sweep run, where both profiles ("lan", "50ms-5j") have a
    known scalar jitter value -- for testing figures that plot a metric against network jitter.
    """
    run_dir = tmp_path / "exp-998-synthetic-jitter" / "20260101-000000Z"
    _write_manifest(
        run_dir, experimentId="exp-998-synthetic-jitter", networkProfiles=["lan", "50ms-5j"]
    )

    for stack, scale in (("none", 1.0), ("fast", 0.5)):
        for profile in ("lan", "50ms-5j"):
            _write_metrics_csv(run_dir / stack / profile, scale=scale)

    return run_dir


@pytest.fixture
def synthetic_run_two_loss_profiles(tmp_path) -> Path:
    """A tiny two-stack, two-profile sweep run using the isolated "loss-<N>pct" family
    (docs/adr/0005-isolated-impairment-profiles.md) -- the legacy presets have no clean loss
    value, so the loss-axis figures need profiles from this family specifically.
    """
    run_dir = tmp_path / "exp-997-synthetic-loss" / "20260101-000000Z"
    _write_manifest(
        run_dir, experimentId="exp-997-synthetic-loss", networkProfiles=["loss-0pct", "loss-5pct"]
    )

    for stack, scale in (("none", 1.0), ("fast", 0.5)):
        for profile in ("loss-0pct", "loss-5pct"):
            _write_metrics_csv(run_dir / stack / profile, scale=scale)

    return run_dir
