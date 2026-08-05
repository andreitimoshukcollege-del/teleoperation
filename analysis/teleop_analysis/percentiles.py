from __future__ import annotations

from typing import List

import pandas as pd

# docs/metrics.md and analysis/CLAUDE.md both require p50/p95/p99, never a mean alone -- this is
# the one place in the package allowed to aggregate a distribution down to a scalar.
QUANTILES = (0.5, 0.95, 0.99)
QUANTILE_LABELS = {0.5: "p50", 0.95: "p95", 0.99: "p99"}


def summarize(df: pd.DataFrame, group_cols: List[str], value_col: str = "value") -> pd.DataFrame:
    """Return one row per group with p50/p95/p99/n columns. Never call .mean() on df[value_col]."""
    if df.empty:
        return pd.DataFrame(columns=[*group_cols, "p50", "p95", "p99", "n"])

    # observed=True: a categorical group column (e.g. "name", read as category in io_utils.py)
    # otherwise keeps every category it was ever declared with, including ones filtered out of
    # this particular `df` -- silently adding empty (all-NaN) rows to the result. This is also
    # where pandas' own default is headed (observed=False is deprecated).
    grouped = df.groupby(group_cols, observed=True)[value_col]
    table = grouped.quantile(list(QUANTILES)).unstack(level=-1)
    table = table.rename(columns=QUANTILE_LABELS)[["p50", "p95", "p99"]]
    table["n"] = grouped.count()
    return table.reset_index()
