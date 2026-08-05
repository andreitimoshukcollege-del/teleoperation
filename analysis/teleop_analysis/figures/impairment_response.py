from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from teleop_analysis import percentiles
from teleop_analysis.figures.captions import build_caption_multi_profile
from teleop_analysis.labels import (
    AXIS_DESCRIPTIONS,
    AXIS_UNITS,
    axis_value,
    friendly_profile_name,
    friendly_stack_name,
    ordered_profiles_by_axis,
)
from teleop_analysis.manifest import Manifest

CORRECTION_METRIC = "correction_magnitude_mm"
PREDICTION_ERROR_METRIC = "prediction_position_error_mm"

# How each axis reads in a chart title: "Correction cost as {phrase}".
_AXIS_PHRASES = {
    "jitter": "the connection gets choppier",
    "delay": "delay increases",
    "loss": "packet loss increases",
}

_CORRECTION_YLABEL = 'Size of the visible "snap" when truth arrives (mm) -- lower is better'
_PREDICTION_ERROR_YLABEL = "Distance from predicted to true position (mm) -- lower is better"


_EXCLUDED_NAME_LIMIT = 6


def _excluded_note(excluded: List[str], axis: str) -> str:
    """" | excluded (no single jitter value): ..." -- naming every profile is fine for a
    handful, but a dense sweep can exclude hundreds (every point from the *other* two axes),
    which would bury the chart in caption text -- fall back to a count past a small threshold.
    """
    if not excluded:
        return ""
    if len(excluded) <= _EXCLUDED_NAME_LIMIT:
        detail = ", ".join(friendly_profile_name(p) for p in excluded)
    else:
        detail = f"{len(excluded)} profiles"
    return f" | excluded (no single {axis} value): {detail}"


def _plot_metric_vs_impairment(
    df: pd.DataFrame,
    manifest: Manifest,
    out_dir: Path,
    metric: str,
    axis: str,
    title: str,
    ylabel: str,
    filename: str,
) -> Optional[Path]:
    """One line per stack: the typical (p50, solid) and occasional-worst-case (p95, dashed)
    value of `metric`, plotted against each profile's value on `axis` ("jitter" | "delay" |
    "loss"). Answers "as this one variable gets worse, how much does this technique's output
    degrade" directly, rather than needing to flip between separate bar charts per profile.

    Only profiles with one clean scalar value on `axis` are placed on it (see
    labels.axis_value) -- a profile like "synthetic-burst" replays a recorded trace and has no
    single value on any axis, and the legacy bundled presets have no clean value on the loss axis
    specifically (docs/adr/0005-isolated-impairment-profiles.md: they'd conflate loss rate with
    burst length). Excluded profiles are named in the caption rather than forced onto a
    misleading position.
    """
    available_profiles = df["profile"].unique().tolist()
    ordered = ordered_profiles_by_axis(available_profiles, axis)
    if len(ordered) < 2:
        print(
            f"Not enough {axis}-comparable network profiles to plot {filename} "
            f"(need at least 2, have {len(ordered)}) -- skipping."
        )
        return None

    excluded = [p for p in available_profiles if p not in ordered]
    x_values = [axis_value(p, axis) for p in ordered]

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for stack in sorted(df["stack"].unique()):
        stack_df = df[(df["stack"] == stack) & (df["name"] == metric) & (df["profile"].isin(ordered))]
        table = percentiles.summarize(stack_df, ["profile"]).set_index("profile")
        p50_values = [table.loc[p, "p50"] if p in table.index else float("nan") for p in ordered]
        p95_values = [table.loc[p, "p95"] if p in table.index else float("nan") for p in ordered]

        line = ax.plot(
            x_values, p50_values, marker="o", label=f"{friendly_stack_name(stack)} (typical)"
        )[0]
        ax.plot(
            x_values, p95_values, marker="o", linestyle="--", alpha=0.6,
            color=line.get_color(), label=f"{friendly_stack_name(stack)} (occasional worst case)",
        )

    ax.set_xlabel(f"Network {axis} ({AXIS_UNITS[axis]}) -- {AXIS_DESCRIPTIONS[axis]}")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=8)

    caption = build_caption_multi_profile(manifest)
    note = _excluded_note(excluded, axis)
    fig.text(
        0.5, 0.01,
        f"{caption}{note}\nSolid = p50 (typical case), dashed = p95 (occasional worst case, "
        "about 1 run in 20)",
        ha="center", fontsize=8, wrap=True,
    )
    fig.tight_layout(rect=(0, 0.1, 1, 1))

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / filename
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path


def plot_correction_vs_jitter(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_impairment(
        df, manifest, out_dir,
        metric=CORRECTION_METRIC, axis="jitter",
        title=f"Correction cost as {_AXIS_PHRASES['jitter']}",
        ylabel=_CORRECTION_YLABEL,
        filename="impairment__correction_vs_jitter.png",
    )


def plot_prediction_error_vs_jitter(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_impairment(
        df, manifest, out_dir,
        metric=PREDICTION_ERROR_METRIC, axis="jitter",
        title=f"Prediction error as {_AXIS_PHRASES['jitter']}",
        ylabel=_PREDICTION_ERROR_YLABEL,
        filename="impairment__prediction_error_vs_jitter.png",
    )


def plot_correction_vs_delay(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_impairment(
        df, manifest, out_dir,
        metric=CORRECTION_METRIC, axis="delay",
        title=f"Correction cost as {_AXIS_PHRASES['delay']}",
        ylabel=_CORRECTION_YLABEL,
        filename="impairment__correction_vs_delay.png",
    )


def plot_prediction_error_vs_delay(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_impairment(
        df, manifest, out_dir,
        metric=PREDICTION_ERROR_METRIC, axis="delay",
        title=f"Prediction error as {_AXIS_PHRASES['delay']}",
        ylabel=_PREDICTION_ERROR_YLABEL,
        filename="impairment__prediction_error_vs_delay.png",
    )


def plot_correction_vs_loss(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_impairment(
        df, manifest, out_dir,
        metric=CORRECTION_METRIC, axis="loss",
        title=f"Correction cost as {_AXIS_PHRASES['loss']}",
        ylabel=_CORRECTION_YLABEL,
        filename="impairment__correction_vs_loss.png",
    )


def plot_prediction_error_vs_loss(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_impairment(
        df, manifest, out_dir,
        metric=PREDICTION_ERROR_METRIC, axis="loss",
        title=f"Prediction error as {_AXIS_PHRASES['loss']}",
        ylabel=_PREDICTION_ERROR_YLABEL,
        filename="impairment__prediction_error_vs_loss.png",
    )
