from __future__ import annotations

from typing import List, Optional

import pandas as pd
from scipy.stats import mannwhitneyu


def seed_spread(df: pd.DataFrame, group_cols: List[str], value_col: str = "value") -> Optional[pd.DataFrame]:
    """Per-group p50 spread across seeds (max - min of per-seed p50).

    docs/metrics.md §8 rule 3: never declare a winner from a single seed, always state the
    observed spread. Returns None -- not a fabricated zero -- when the metrics.csv predates the
    seed column, so the caller can say honestly that spread is unavailable for this run.
    """
    if "seed" not in df.columns or df["seed"].isna().all():
        return None

    with_seed = df.dropna(subset=["seed"])
    per_seed = (
        with_seed.groupby([*group_cols, "seed"])[value_col]
        .quantile(0.5)
        .reset_index()
    )
    spread = (
        per_seed.groupby(group_cols)[value_col]
        .agg(lambda s: s.max() - s.min())
        .reset_index()
        .rename(columns={value_col: "p50_seed_spread"})
    )
    return spread


def compare_to_baseline(
    df: pd.DataFrame,
    baseline_stack: str,
    stack_col: str = "stack",
    value_col: str = "value",
) -> pd.DataFrame:
    """Mann-Whitney U test of each non-baseline stack against the baseline, per (profile, metric).

    Mann-Whitney U is used because it makes no normality assumption about the (heavy-tailed)
    latency/error distributions this project reports -- analysis/CLAUDE.md requires stating the
    test and its assumptions. Assumption checked here: independent samples (true across seeds/
    trials within one sweep) and at least 2 observations per side.
    """
    results = []
    for (profile, metric), group in df.groupby(["profile", "name"]):
        baseline_vals = group.loc[group[stack_col] == baseline_stack, value_col].dropna()
        if len(baseline_vals) < 2:
            continue
        for stack_name, sub in group.groupby(stack_col):
            if stack_name == baseline_stack:
                continue
            vals = sub[value_col].dropna()
            if len(vals) < 2:
                continue
            statistic, p_value = mannwhitneyu(vals, baseline_vals, alternative="two-sided")
            results.append({
                "profile": profile,
                "metric": metric,
                "stack": stack_name,
                "u_statistic": statistic,
                "p_value": p_value,
                "assumption": "independent samples; distribution-free (Mann-Whitney U, two-sided)",
            })
    return pd.DataFrame(results)
