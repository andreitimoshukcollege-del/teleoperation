from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import pandas as pd

from teleop_analysis import percentiles
from teleop_analysis.axis_diff import classify_stacks
from teleop_analysis.figures._bars import plot_percentile_bars
from teleop_analysis.figures.captions import build_caption
from teleop_analysis.labels import PERCENTILE_EXPLANATION, friendly_stack_name
from teleop_analysis.manifest import Manifest

DEFAULT_METRIC = "prediction_position_error_mm"


def _best_by_p50(table: pd.DataFrame, exclude: Iterable[str]) -> Optional[str]:
    candidates = table[~table["stack"].isin(list(exclude))]
    if candidates.empty:
        return None
    return candidates.loc[candidates["p50"].idxmin(), "stack"]


def build_stack_comparison_figure(
    df: pd.DataFrame,
    manifest: Manifest,
    profile: str,
    metric: str = DEFAULT_METRIC,
) -> Tuple[Figure, str]:
    """Builds the figure and its caption without saving anything -- split out of
    plot_stack_comparison so the GUI's live figure view (test_gui.py) can embed the same figure
    directly instead of loading a saved PNG back off disk. Two panels: mitigations tested
    separately (single-axis vs baseline) and tested together (multi-axis combinations vs
    baseline and the best single-axis result) -- this is what makes "separately and together" a
    literal, readable comparison rather than one pooled chart.
    """
    subset = df[(df["profile"] == profile) & (df["name"] == metric)]
    classification = classify_stacks(manifest)
    baseline_names = [n for n, c in classification.items() if c == "baseline"]
    single_names = [n for n, c in classification.items() if c in ("baseline", "single-axis")]
    combined_names = [n for n, c in classification.items() if c == "combined"]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ylabel = f"{metric.replace('_mm', '').replace('_', ' ').title()} (lower is better)"

    single_table = percentiles.summarize(subset[subset["stack"].isin(single_names)], ["stack"])
    plot_percentile_bars(
        axes[0],
        single_table,
        "stack",
        "Each mitigation tried on its own, vs. no mitigation",
        ylabel,
        label_fn=friendly_stack_name,
    )

    if combined_names:
        best_single = _best_by_p50(single_table, exclude=baseline_names)
        names = list(dict.fromkeys(baseline_names + combined_names + ([best_single] if best_single else [])))
        combined_table = percentiles.summarize(subset[subset["stack"].isin(names)], ["stack"])
        plot_percentile_bars(
            axes[1],
            combined_table,
            "stack",
            "Mitigations combined, vs. baseline and the best single one",
            ylabel,
            label_fn=friendly_stack_name,
        )
    else:
        axes[1].axis("off")
        axes[1].text(
            0.5, 0.5,
            "No multi-axis mitigation stacks in this manifest yet.",
            ha="center", va="center",
        )

    caption = build_caption(manifest, profile)
    fig.text(
        0.5, 0.01,
        f"{caption} | metric={metric}\n{PERCENTILE_EXPLANATION}",
        ha="center", fontsize=8, wrap=True,
    )
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    return fig, caption


def plot_stack_comparison(
    df: pd.DataFrame,
    manifest: Manifest,
    profile: str,
    out_dir: Path,
    metric: str = DEFAULT_METRIC,
) -> Path:
    fig, caption = build_stack_comparison_figure(df, manifest, profile, metric)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{profile}__stack_comparison.png"
    fig.savefig(out_path, metadata={"Description": caption})
    plt.close(fig)
    return out_path
