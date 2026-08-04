from __future__ import annotations

from typing import Callable, Optional

import numpy as np
import pandas as pd


def plot_percentile_bars(
    ax,
    table: pd.DataFrame,
    category_col: str,
    title: str,
    ylabel: str,
    label_fn: Optional[Callable[[str], str]] = None,
) -> None:
    """Grouped p50/p95/p99 bars, one group of three per category. Never a mean-only bar.

    `label_fn`, if given, maps each category's raw registry key (e.g. "double-exp") to a
    human-readable display string (e.g. "Double exponential") for the x-tick labels only --
    the underlying data/grouping is unaffected.
    """
    if table.empty:
        ax.axis("off")
        ax.text(0.5, 0.5, "no data for this metric/profile", ha="center", va="center")
        return

    categories = table[category_col].tolist()
    display_categories = [label_fn(c) for c in categories] if label_fn else categories
    x = np.arange(len(categories))
    width = 0.25

    ax.bar(x - width, table["p50"], width, label="p50 (typical)")
    ax.bar(x, table["p95"], width, label="p95 (occasional worst case)")
    ax.bar(x + width, table["p99"], width, label="p99 (rare worst case)")
    ax.set_xticks(x)
    ax.set_xticklabels(display_categories, rotation=30, ha="right")
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.legend(fontsize=8)
