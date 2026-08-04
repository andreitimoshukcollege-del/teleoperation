from __future__ import annotations

from pathlib import Path

import pandas as pd

from teleop_analysis import percentiles


def build_summary_table(df: pd.DataFrame) -> pd.DataFrame:
    table = percentiles.summarize(df, ["profile", "name", "stack"])
    return table.sort_values(["profile", "name", "stack"]).reset_index(drop=True)


def write_summary_table(table: pd.DataFrame, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "summary_table.csv"
    table.to_csv(out_path, index=False)
    return out_path
