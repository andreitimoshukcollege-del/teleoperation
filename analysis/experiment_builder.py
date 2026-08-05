"""Builds an experiments/*.yaml for the isolated jitter/delay/loss profile families
(docs/adr/0005-isolated-impairment-profiles.md), for the GUI's Experiment tab.

Pure string/list generation -- no file I/O here, the caller decides where to write the result.
Kept outside teleop_analysis/ on purpose: that package only reads results/ and produces figures
(analysis/CLAUDE.md); generating a new experiment config and (elsewhere, in test_gui.py) shelling
out to `dotnet run -- sweep` is a different kind of action, same reasoning that already puts
run_tests.py/test_gui.py outside that package.
"""
from __future__ import annotations

from typing import List

_POINT_EPSILON = 1e-9


def _points(min_value: float, max_value: float, step: float) -> List[float]:
    if step <= 0:
        raise ValueError(f"step must be positive, got {step}")
    if max_value < min_value:
        raise ValueError(f"max ({max_value}) must be >= min ({min_value})")

    points = []
    value = min_value
    while value <= max_value + _POINT_EPSILON:
        points.append(round(value, 10))
        value += step
    return points


def _format_number(value: float) -> str:
    """41 -> "41", 0.25 -> "0.25" -- matches the "no trailing .0" convention
    NetworkProfileCatalog's regex resolver and labels.py's parser both already expect.
    """
    text = f"{value:.10f}".rstrip("0").rstrip(".")
    return text or "0"


def jitter_points(min_ms: float, max_ms: float, step_ms: float) -> List[str]:
    return [f"jitter-{_format_number(v)}ms" for v in _points(min_ms, max_ms, step_ms)]


def delay_points(min_ms: float, max_ms: float, step_ms: float) -> List[str]:
    return [f"delay-{_format_number(v)}ms" for v in _points(min_ms, max_ms, step_ms)]


def loss_points(min_pct: float, max_pct: float, step_pct: float) -> List[str]:
    return [f"loss-{_format_number(v)}pct" for v in _points(min_pct, max_pct, step_pct)]


def build_experiment_yaml(
    experiment_id: str,
    predictors: List[str],
    seeds: List[int],
    profiles: List[str],
    reconciler: str = "snap",
    trial_steps: int = 500,
    step_interval_ticks: int = 100000,
) -> str:
    """Returns YAML text matching experiments/CLAUDE.md's schema, ready to write to a file."""
    if not predictors:
        raise ValueError("at least one predictor is required")
    if not profiles:
        raise ValueError("at least one network profile is required")
    if not seeds:
        raise ValueError("at least one seed is required")

    lines = [
        f"id: {experiment_id}",
        f"seeds: [{', '.join(str(s) for s in seeds)}]",
        "predictors:",
        *(f"  - {p}" for p in predictors),
        f"reconciler: {reconciler}",
        "networkProfiles:",
        *(f"  - {p}" for p in profiles),
        f"trialSteps: {trial_steps}",
        f"stepIntervalTicks: {step_interval_ticks}",
    ]
    return "\n".join(lines) + "\n"
