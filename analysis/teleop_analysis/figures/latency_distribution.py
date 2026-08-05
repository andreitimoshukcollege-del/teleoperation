from __future__ import annotations

from pathlib import Path
from typing import Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import pandas as pd

from teleop_analysis import percentiles
from teleop_analysis.figures._bars import plot_percentile_bars
from teleop_analysis.figures.captions import build_caption
from teleop_analysis.labels import PERCENTILE_EXPLANATION, friendly_stack_name
from teleop_analysis.manifest import Manifest

# docs/metrics.md §2: uplink and downlink OWD are frequently asymmetric and averaging them hides
# that -- always plotted as two separate panels, never combined into one number.
UPLINK_METRIC = "owd_uplink_ms"
DOWNLINK_METRIC = "owd_downlink_ms"


def build_latency_distribution_figure(df: pd.DataFrame, manifest: Manifest, profile: str) -> Tuple[Figure, str]:
    """Builds the figure and its caption without saving anything -- split out of
    plot_latency_distribution so the GUI's live figure view (test_gui.py) can embed the same
    figure directly instead of loading a saved PNG back off disk. One-way delay in each
    direction, kept separate because they're frequently asymmetric -- averaging them would hide
    that.
    """
    subset = df[df["profile"] == profile]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    uplink_table = percentiles.summarize(subset[subset["name"] == UPLINK_METRIC], ["stack"])
    plot_percentile_bars(
        axes[0],
        uplink_table,
        "stack",
        "Uplink delay: operator command → robot (lower is better)",
        "One-way delay (ms)",
        label_fn=friendly_stack_name,
    )

    downlink_table = percentiles.summarize(subset[subset["name"] == DOWNLINK_METRIC], ["stack"])
    plot_percentile_bars(
        axes[1],
        downlink_table,
        "stack",
        "Downlink delay: robot state → operator (lower is better)",
        "One-way delay (ms)",
        label_fn=friendly_stack_name,
    )

    caption = build_caption(manifest, profile)
    fig.text(0.5, 0.01, f"{caption}\n{PERCENTILE_EXPLANATION}", ha="center", fontsize=8, wrap=True)
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    return fig, caption


def plot_latency_distribution(df: pd.DataFrame, manifest: Manifest, profile: str, out_dir: Path) -> Path:
    fig, caption = build_latency_distribution_figure(df, manifest, profile)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{profile}__latency.png"
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path
