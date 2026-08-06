from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
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


def _thinned_tick_indices_in_range(step_count: int, xlo: float, xhi: float) -> List[int]:
    """Which x positions in the visible range [xlo, xhi] get a tick label. Every visible step up
    to _MAX_TICK_LABELS; past that, an evenly spaced subset (always including the last visible
    step) so a dense sweep's labels don't overwrite each other into an unreadable smear.
    Range-aware (rather than always thinning the *whole* sweep) so the GUI's live, zoomable view
    can re-thin to just the currently visible steps as the user zooms in -- otherwise a zoomed-in
    view could land entirely between two of the whole-sweep's chosen labels and show none at all.
    """
    lo = max(0, math.floor(xlo))
    hi = min(step_count - 1, math.ceil(xhi))
    if lo > hi:
        return []
    visible_count = hi - lo + 1
    if visible_count <= _MAX_TICK_LABELS:
        return list(range(lo, hi + 1))
    stride = -(-visible_count // _MAX_TICK_LABELS)  # ceil division
    indices = list(range(lo, hi + 1, stride))
    if indices[-1] != hi:
        indices.append(hi)
    return indices


def _thinned_tick_indices(step_count: int) -> List[int]:
    """The whole-sweep case of _thinned_tick_indices_in_range -- what the static saved PNG uses,
    since it never changes its own view.
    """
    return _thinned_tick_indices_in_range(step_count, 0, step_count - 1)


def _marker_style(step_count: int) -> Tuple[Optional[str], Optional[int]]:
    """A marker at every step is legible up to a couple dozen points; past that the dots
    overlap into a solid blob, so drop them and let the line alone carry a dense sweep's shape.
    """
    if step_count <= _MARKER_CUTOFF:
        return "o", 5
    return None, None


def _build_metric_vs_combined_figure(
    df: pd.DataFrame,
    manifest: Manifest,
    metric: str,
    title: str,
    ylabel: str,
) -> Optional[Tuple[Figure, str]]:
    """Builds the figure and its caption without saving anything -- split out of
    _plot_metric_vs_combined so the GUI's live figure view (test_gui.py) can embed the same
    figure directly instead of loading a saved PNG back off disk. One line per stack across a
    co-varying combined sweep (docs/adr/0006-combined-impairment-profiles.md) -- every axis
    checked in the GUI's "Combined impairments" section marches forward together
    (experiment_builder.combined_points is a lockstep zip, not a cross product), so there's
    exactly one point per step, all on one graph, not a separate chart per combined profile. x
    position is step index; the tick label spells out every axis's value at that step, since a
    combined profile has no single scalar the way an isolated-axis one does (labels.axis_value
    returns None for it on every axis).

    Registers an `xlim_changed` callback that re-thins the visible tick labels to whatever range
    is currently in view -- inert for the static saved PNG (its xlim never changes), but what
    keeps a live, zoomed-in view of a dense sweep from landing on a stretch with no labels at
    all, since the whole-sweep thinning only accounted for the full-width view.
    """
    available_profiles = df["profile"].unique().tolist()
    parsed = {p: combined_profile_axes(p) for p in available_profiles}
    combined_profiles = [p for p, axes in parsed.items() if axes]
    if len(combined_profiles) < 2:
        print(
            f"Not enough combined ('combo__') profiles to plot "
            f"(need at least 2, have {len(combined_profiles)}) -- skipping."
        )
        return None

    # The lockstep construction means every populated axis increases together, so any one axis
    # that's present in every combined profile in this run gives the correct step order.
    axis_keys = set.intersection(*(set(parsed[p].keys()) for p in combined_profiles))
    common_axis = next((a for a in _AXIS_ORDER if a in axis_keys), None)
    if common_axis is None:
        print("Combined profiles in this run don't share a common axis to order by -- skipping.")
        return None
    ordered = sorted(combined_profiles, key=lambda p: parsed[p][common_axis])
    step_count = len(ordered)

    x_positions = list(range(step_count))
    marker, markersize = _marker_style(step_count)

    fig_width = max(9.0, min(20.0, 9.0 + 0.05 * step_count))
    fig, ax = plt.subplots(figsize=(fig_width, 5.5))

    # Filter and group once, not once per stack -- df can be tens of millions of rows for a dense
    # sweep, and `stack`/`profile` are deliberately plain string columns (io_utils.py), so a
    # filter pass is a full string-hash scan every time it reruns.
    metric_df = df[(df["name"] == metric) & (df["profile"].isin(ordered))]
    table = percentiles.summarize(metric_df, ["stack", "profile"]).set_index(["stack", "profile"])
    known_stacks = set(table.index.get_level_values("stack"))

    for stack in sorted(df["stack"].unique()):
        if stack in known_stacks:
            stack_table = table.xs(stack, level="stack").reindex(ordered)
        else:
            stack_table = pd.DataFrame(index=ordered, columns=["p50", "p95"], dtype=float)
        p50_values = stack_table["p50"].to_numpy()
        p95_values = stack_table["p95"].to_numpy()

        line = ax.plot(
            x_positions, p50_values, marker=marker, markersize=markersize,
            label=f"{friendly_stack_name(stack)} (typical)",
        )[0]
        ax.plot(
            x_positions, p95_values, marker=marker, markersize=markersize, linestyle="--",
            alpha=0.6, color=line.get_color(),
            label=f"{friendly_stack_name(stack)} (occasional worst case)",
        )

    def _apply_ticks(ax) -> None:
        xlo, xhi = ax.get_xlim()
        tick_indices = _thinned_tick_indices_in_range(step_count, xlo, xhi)
        ax.set_xticks(tick_indices)
        ax.set_xticklabels(
            [_tick_label(parsed[ordered[i]]) for i in tick_indices],
            fontsize=8, rotation=30, ha="right",
        )

    _apply_ticks(ax)
    ax.callbacks.connect("xlim_changed", _apply_ticks)
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
    return fig, caption


def _plot_metric_vs_combined(
    df: pd.DataFrame,
    manifest: Manifest,
    out_dir: Path,
    metric: str,
    title: str,
    ylabel: str,
    filename: str,
) -> Optional[Path]:
    result = _build_metric_vs_combined_figure(df, manifest, metric, title, ylabel)
    if result is None:
        return None
    fig, caption = result
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / filename
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path


def build_correction_vs_combined_figure(df: pd.DataFrame, manifest: Manifest) -> Optional[Tuple[Figure, str]]:
    return _build_metric_vs_combined_figure(
        df, manifest,
        metric=CORRECTION_METRIC,
        title="Correction cost across a combined impairment sweep",
        ylabel=_CORRECTION_YLABEL,
    )


def plot_correction_vs_combined(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_combined(
        df, manifest, out_dir,
        metric=CORRECTION_METRIC,
        title="Correction cost across a combined impairment sweep",
        ylabel=_CORRECTION_YLABEL,
        filename="combined__correction.png",
    )


def build_prediction_error_vs_combined_figure(df: pd.DataFrame, manifest: Manifest) -> Optional[Tuple[Figure, str]]:
    return _build_metric_vs_combined_figure(
        df, manifest,
        metric=PREDICTION_ERROR_METRIC,
        title="Prediction error across a combined impairment sweep",
        ylabel=_PREDICTION_ERROR_YLABEL,
    )


def plot_prediction_error_vs_combined(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_combined(
        df, manifest, out_dir,
        metric=PREDICTION_ERROR_METRIC,
        title="Prediction error across a combined impairment sweep",
        ylabel=_PREDICTION_ERROR_YLABEL,
        filename="combined__prediction_error.png",
    )
