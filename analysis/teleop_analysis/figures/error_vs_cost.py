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

# docs/metrics.md §5 / §8 rule 2: prediction error and correction cost must always be reported
# together, in the same figure, never as two separately-citable halves.
POSITION_ERROR_METRIC = "prediction_position_error_mm"
CORRECTION_METRIC = "correction_magnitude_mm"


def build_error_vs_cost_figure(df: pd.DataFrame, manifest: Manifest, profile: str) -> Tuple[Figure, str]:
    """Builds the figure and its caption without saving anything -- split out of
    plot_error_vs_cost so the GUI's live figure view (test_gui.py) can embed the same figure
    directly instead of loading a saved PNG back off disk. Left: how far off the prediction was.
    Right: how big the visible correction was when truth disagreed with it. Shown together
    deliberately -- an aggressive predictor can look great on the left and terrible on the right
    (frequent, large snaps), which is exactly the tradeoff this pair of panels is meant to make
    visible.
    """
    subset = df[df["profile"] == profile]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    error_table = percentiles.summarize(subset[subset["name"] == POSITION_ERROR_METRIC], ["stack"])
    plot_percentile_bars(
        axes[0],
        error_table,
        "stack",
        "Prediction error (lower is better)",
        "Distance from predicted to true position (mm)",
        label_fn=friendly_stack_name,
    )

    cost_table = percentiles.summarize(subset[subset["name"] == CORRECTION_METRIC], ["stack"])
    plot_percentile_bars(
        axes[1],
        cost_table,
        "stack",
        "Correction cost (lower is better)",
        "Size of the visible \"snap\" when truth arrives (mm)",
        label_fn=friendly_stack_name,
    )

    caption = build_caption(manifest, profile)
    fig.text(0.5, 0.01, f"{caption}\n{PERCENTILE_EXPLANATION}", ha="center", fontsize=8, wrap=True)
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    return fig, caption


def plot_error_vs_cost(df: pd.DataFrame, manifest: Manifest, profile: str, out_dir: Path) -> Path:
    fig, caption = build_error_vs_cost_figure(df, manifest, profile)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{profile}__error_vs_cost.png"
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path
