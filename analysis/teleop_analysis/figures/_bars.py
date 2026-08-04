from __future__ import annotations

import numpy as np
import pandas as pd


def plot_percentile_bars(ax, table: pd.DataFrame, category_col: str, title: str, ylabel: str) -> None:
    """Grouped p50/p95/p99 bars, one group of three per category. Never a mean-only bar."""
    if table.empty:
        ax.axis("off")
        ax.text(0.5, 0.5, "no data for this metric/profile", ha="center", va="center")
        return

    categories = table[category_col].tolist()
    x = np.arange(len(categories))
    width = 0.25

    ax.bar(x - width, table["p50"], width, label="p50")
    ax.bar(x, table["p95"], width, label="p95")
    ax.bar(x + width, table["p99"], width, label="p99")
    ax.set_xticks(x)
    ax.set_xticklabels(categories, rotation=30, ha="right")
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.legend()
