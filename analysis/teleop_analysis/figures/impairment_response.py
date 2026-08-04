from __future__ import annotations

from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from teleop_analysis import percentiles
from teleop_analysis.figures.captions import build_caption_multi_profile
from teleop_analysis.labels import (
    PROFILE_JITTER_MS,
    friendly_profile_name,
    friendly_stack_name,
    ordered_profiles_by_jitter,
)
from teleop_analysis.manifest import Manifest

CORRECTION_METRIC = "correction_magnitude_mm"
PREDICTION_ERROR_METRIC = "prediction_position_error_mm"


def _plot_metric_vs_jitter(
    df: pd.DataFrame,
    manifest: Manifest,
    out_dir: Path,
    metric: str,
    title: str,
    ylabel: str,
    filename: str,
) -> Optional[Path]:
    """One line per stack: the typical (p50, solid) and occasional-worst-case (p95, dashed)
    value of `metric`, plotted against each profile's jitter. Answers "as the connection gets
    choppier, how much does this technique's output degrade" directly, rather than needing to
    flip between separate bar charts per profile.

    Only profiles with one fixed scalar jitter value are placed on this axis (see
    labels.PROFILE_JITTER_MS) -- a profile like "synthetic-burst" replays a recorded trace and
    has no single jitter number, so it's excluded and named in the caption instead of being
    forced onto a misleading position.
    """
    available_profiles = df["profile"].unique().tolist()
    ordered = ordered_profiles_by_jitter(available_profiles)
    if len(ordered) < 2:
        print(
            f"Not enough jitter-comparable network profiles to plot {filename} "
            f"(need at least 2, have {len(ordered)}) -- skipping."
        )
        return None

    excluded = [p for p in available_profiles if p not in ordered]
    jitter_values = [_jitter_of(p) for p in ordered]

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for stack in sorted(df["stack"].unique()):
        stack_df = df[(df["stack"] == stack) & (df["name"] == metric) & (df["profile"].isin(ordered))]
        table = percentiles.summarize(stack_df, ["profile"]).set_index("profile")
        p50_values = [table.loc[p, "p50"] if p in table.index else float("nan") for p in ordered]
        p95_values = [table.loc[p, "p95"] if p in table.index else float("nan") for p in ordered]

        line = ax.plot(
            jitter_values, p50_values, marker="o", label=f"{friendly_stack_name(stack)} (typical)"
        )[0]
        ax.plot(
            jitter_values, p95_values, marker="o", linestyle="--", alpha=0.6,
            color=line.get_color(), label=f"{friendly_stack_name(stack)} (occasional worst case)",
        )

    ax.set_xlabel("Network jitter (ms) -- higher = choppier connection")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=8)

    caption = build_caption_multi_profile(manifest)
    note = ""
    if excluded:
        excluded_names = ", ".join(friendly_profile_name(p) for p in excluded)
        note = f" | excluded (no single jitter value): {excluded_names}"
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


def _jitter_of(profile: str) -> float:
    return PROFILE_JITTER_MS[profile]


def plot_correction_vs_impairment(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_jitter(
        df, manifest, out_dir,
        metric=CORRECTION_METRIC,
        title="Correction cost as the connection gets choppier",
        ylabel="Size of the visible \"snap\" when truth arrives (mm) -- lower is better",
        filename="impairment__correction_vs_jitter.png",
    )


def plot_prediction_error_vs_impairment(df: pd.DataFrame, manifest: Manifest, out_dir: Path) -> Optional[Path]:
    return _plot_metric_vs_jitter(
        df, manifest, out_dir,
        metric=PREDICTION_ERROR_METRIC,
        title="Prediction error as the connection gets choppier",
        ylabel="Distance from predicted to true position (mm) -- lower is better",
        filename="impairment__prediction_error_vs_jitter.png",
    )
