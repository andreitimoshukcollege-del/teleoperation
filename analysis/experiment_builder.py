"""Builds an experiments/*.yaml for the isolated jitter/delay/loss profile families
(docs/adr/0005-isolated-impairment-profiles.md) and the combined multi-axis family
(docs/adr/0006-combined-impairment-profiles.md), for the GUI's Experiment tab.

Pure string/list generation -- no file I/O here, the caller decides where to write the result.
Kept outside teleop_analysis/ on purpose: that package only reads results/ and produces figures
(analysis/CLAUDE.md); generating a new experiment config and (elsewhere, in test_gui.py) shelling
out to `dotnet run -- sweep` is a different kind of action, same reasoning that already puts
run_tests.py/test_gui.py outside that package.
"""
from __future__ import annotations

from itertools import product
from typing import List, Sequence

_POINT_EPSILON = 1e-9


def axis_points(min_value: float, max_value: float, step: float) -> List[float]:
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
    return [f"jitter-{_format_number(v)}ms" for v in axis_points(min_ms, max_ms, step_ms)]


def delay_points(min_ms: float, max_ms: float, step_ms: float) -> List[str]:
    return [f"delay-{_format_number(v)}ms" for v in axis_points(min_ms, max_ms, step_ms)]


def loss_points(min_pct: float, max_pct: float, step_pct: float) -> List[str]:
    return [f"loss-{_format_number(v)}pct" for v in axis_points(min_pct, max_pct, step_pct)]


def combined_points(
    delay_ms: Sequence[float] = (),
    jitter_ms: Sequence[float] = (),
    loss_pct: Sequence[float] = (),
) -> List[str]:
    """Cartesian product of the given per-axis value lists into "combo__" profile names
    (docs/adr/0006-combined-impairment-profiles.md) -- e.g. delay_ms=[100, 150], jitter_ms=[20]
    produces ["combo__delay-100ms__jitter-20ms", "combo__delay-150ms__jitter-20ms"]. An axis
    passed as empty is omitted from every generated name entirely (resolves to 0 at sweep time,
    not a value of 0 in this product), not one of the values being combined.

    Requires at least 2 non-empty axes -- combining exactly one axis is just the isolated family
    (jitter_points/delay_points/loss_points above), which already exists and isolates its
    companions properly for sensitivity charts; this function is for genuine combinations only.
    """
    axes = [
        ("delay", "ms", delay_ms),
        ("jitter", "ms", jitter_ms),
        ("loss", "pct", loss_pct),
    ]
    populated = [(label, unit, values) for label, unit, values in axes if values]
    if len(populated) < 2:
        raise ValueError("combined profiles need at least 2 non-empty axes")

    combos = []
    for combo in product(*(values for _, _, values in populated)):
        segments = [
            f"{label}-{_format_number(value)}{unit}"
            for (label, unit, _), value in zip(populated, combo)
        ]
        combos.append("combo__" + "__".join(segments))
    return combos


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
