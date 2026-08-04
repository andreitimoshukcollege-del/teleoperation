from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from teleop_analysis import percentiles
from teleop_analysis.figures._bars import plot_percentile_bars
from teleop_analysis.figures.captions import build_caption
from teleop_analysis.manifest import Manifest

# docs/metrics.md §2: uplink and downlink OWD are frequently asymmetric and averaging them hides
# that -- always plotted as two separate panels, never combined into one number.
UPLINK_METRIC = "owd_uplink_ms"
DOWNLINK_METRIC = "owd_downlink_ms"


def plot_latency_distribution(df: pd.DataFrame, manifest: Manifest, profile: str, out_dir: Path) -> Path:
    subset = df[df["profile"] == profile]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    uplink_table = percentiles.summarize(subset[subset["name"] == UPLINK_METRIC], ["stack"])
    plot_percentile_bars(axes[0], uplink_table, "stack", "Uplink one-way delay", "ms")

    downlink_table = percentiles.summarize(subset[subset["name"] == DOWNLINK_METRIC], ["stack"])
    plot_percentile_bars(axes[1], downlink_table, "stack", "Downlink one-way delay", "ms")

    caption = build_caption(manifest, profile)
    fig.text(0.5, 0.01, caption, ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.05, 1, 1))

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{profile}__latency.png"
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path
