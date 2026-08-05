from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from teleop_analysis import percentiles
from teleop_analysis.figures.captions import build_caption_multi_profile
from teleop_analysis.labels import combined_profile_axes, friendly_stack_name
from teleop_analysis.manifest import Manifest

CORRECTION_METRIC = "correction_magnitude_mm"
PREDICTION_ERROR_METRIC = "prediction_position_error_mm"

_CORRECTION_YLABEL = 'Size of the visible "snap" when truth arrives (mm) -- lower is better'
_PREDICTION_ERROR_YLABEL = "Distance from predicted to true position (mm) -- lower is better"

# Canonical axis order everywhere else in this codebase (NetworkProfileCatalog,
# experiment_builder, labels.py's friendly-name formatter).
_AXIS_ORDER = ("delay", "jitter", "loss")
_AXIS_UNIT = {"delay": "ms", "jitter": "ms", "loss": "%"}

# A dense combined sweep (e.g. a 300-point delay range, ADR 0006's "hold the shorter axis"
# behavior) has far too many steps to label every one without the labels overwriting each other,
# and dots that are fine 10 apart become a solid blob 300 apart on the same figure width -- these
# thresholds keep the chart legible regardless of how many steps the sweep actually has.
_MAX_TICK_LABELS = 12
_MARKER_CUTOFF = 20


def _tick_label(axes: Dict[str, float]) -> str:
    return ", ".join(f"{axis}={axes[axis]:g}{_AXIS_UNIT[axis]}" for axis in _AXIS_ORDER if axis in axes)


def _thinned_tick_indices(step_count: int) -> List[int]:
    """Which of the `step_count` x positions get a tick label. Every step up to
    _MAX_TICK_LABELS; past that, an evenly spaced subset (always including the last step) so a
    dense sweep's labels don't overwrite each other into an unreadable smear.
    """
    if step_count <= _MAX_TICK_LABELS:
        return list(range(step_count))
    stride = -(-step_count // _MAX_TICK_LABELS)  # ceil division
    indices = list(range(0, step_count, stride))
    if indices[-1] != step_count - 1:
        indices.append(step_count - 1)
    return indices


def _marker_style(step_count: int) -> Tuple[Optional[str], Optional[int]]:
    """A marker at every step is legible up to a couple dozen points; past that the dots
    overlap into a solid blob, so drop them and let the line alone carry a dense sweep's shape.
    """
    if step_count <= _MARKER_CUTOFF:
        return "o", 5
    return None, None


def _plot_metric_vs_combined(
    df: pd.DataFrame,
    manifest: Manifest,
    out_dir: Path,
    metric: str,
    title: str,
    ylabel: str,
    filename: str,
) -> Optional[Path]:
    """One line per stack across a co-varying combined sweep
    (docs/adr/0006-combined-impairment-profiles.md) -- every axis checked in the GUI's "Combined
    impairments" section marches forward together (experiment_builder.combined_points is a
    lockstep zip, not a cross product), so there's exactly one point per step, all on one graph,
    not a separate chart per combined profile. x position is step index; the tick label spells
    out every axis's value at that step, since a combined profile has no single scalar the way
    an isolated-axis one does (labels.axis_value returns None for it on every axis).
    """
    available_profiles = df["profile"].unique().tolist()
    parsed = {p: combined_profile_axes(p) for p in available_profiles}
    combined_profiles = [p for p, axes in parsed.items() if axes]
    if len(combined_profiles) < 2:
        print(
            f"Not enough combined ('combo__') profiles to plot {filename} "
            f"(need at least 2, have {len(combined_profiles)}) -- skipping."
        )
        return None

    # The lockstep construction means every populated axis increases together, so any one axis
    # that's present in every combined profile in this run gives the correct step order.
    axis_keys = set.intersection(*(set(parsed[p].keys()) for p in combined_profiles))
    common_axis = next((a for a in _AXIS_ORDER if a in axis_keys), None)
    if common_axis is None:
        print(
            f"Combined profiles in this run don't share a common axis to order by -- "
            f"skipping {filename}."
        )
        return None
    ordered = sorted(combined_profiles, key=lambda p: parsed[p][common_axis])
    step_count = len(ordered)

    x_positions = list(range(step_count))
    marker, markersize = _marker_style(step_count)

    fig_width = max(9.0, min(20.0, 9.0 + 0.05 * step_count))
    fig, ax = plt.subplots(figsize=(fig_width, 5.5))
    for stack in sorted(df["stack"].unique()):
        stack_df = df[(df["stack"] == stack) & (df["name"] == metric) & (df["profile"].isin(ordered))]
        table = percentiles.summarize(stack_df, ["profile"]).set_index("profile")
        p50_values = [table.loc[p, "p50"] if p in table.index else float("nan") for p in ordered]
        p95_values = [table.loc[p, "p95"] if p in table.index else float("nan") for p in ordered]

        line = ax.plot(
            x_positions, p50_values, marker=marker, markersize=markersize,
            label=f"{friendly_stack_name(stack)} (typical)",
        )[0]
        ax.plot(
            x_positions, p95_values, marker=marker, markersize=markersize, linestyle="--",
            alpha=0.6, color=line.get_color(),
            label=f"{friendly_stack_name(stack)} (occasional worst case)",
        )

    tick_indices = _thinned_tick_indices(step_count)
    ax.set_xticks(tick_indices)
    ax.set_xticklabels(
        [_tick_label(parsed[ordered[i]]) for i in tick_indices],
        fontsize=8, rotation=30, ha="right",
    )
    ax.set_xlabel("Combined impairment (every checked axis stepped together)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=8)

    caption = build_caption_multi_profile(manifest)
    fig.text(
        0.5, 0.01,
        f"{caption}\nSolid = p50 (typical case), dashed = p95 (occasional worst case, "
        "about 1 run in 20)",
        ha="center", fontsize=8, wrap=True,
    )
    fig.tight_layout(rect=(0, 0.16, 1, 1))

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / filename
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path


def plot_correction_vs_combined(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_combined(
        df, manifest, out_dir,
        metric=CORRECTION_METRIC,
        title="Correction cost across a combined impairment sweep",
        ylabel=_CORRECTION_YLABEL,
        filename="combined__correction.png",
    )


def plot_prediction_error_vs_combined(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_combined(
        df, manifest, out_dir,
        metric=PREDICTION_ERROR_METRIC,
        title="Prediction error across a combined impairment sweep",
        ylabel=_PREDICTION_ERROR_YLABEL,
        filename="combined__prediction_error.png",
    )
