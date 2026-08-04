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

# docs/metrics.md §5 / §8 rule 2: prediction error and correction cost must always be reported
# together, in the same figure, never as two separately-citable halves.
POSITION_ERROR_METRIC = "prediction_position_error_mm"
CORRECTION_METRIC = "correction_magnitude_mm"


def plot_error_vs_cost(df: pd.DataFrame, manifest: Manifest, profile: str, out_dir: Path) -> Path:
    subset = df[df["profile"] == profile]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    error_table = percentiles.summarize(subset[subset["name"] == POSITION_ERROR_METRIC], ["stack"])
    plot_percentile_bars(axes[0], error_table, "stack", "Prediction position error", "mm")

    cost_table = percentiles.summarize(subset[subset["name"] == CORRECTION_METRIC], ["stack"])
    plot_percentile_bars(axes[1], cost_table, "stack", "Correction magnitude", "mm")

    caption = build_caption(manifest, profile)
    fig.text(0.5, 0.01, caption, ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.05, 1, 1))

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{profile}__error_vs_cost.png"
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path
