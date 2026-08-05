from __future__ import annotations

import concurrent.futures
from pathlib import Path
from typing import List, Tuple

import pandas as pd

from teleop_analysis.manifest import Manifest

# "name" has a small, fixed set of real values (currently 6 metric names) known upfront --
# declaring it categorical at read time skips pandas' per-file type-sniffing and makes every
# later groupby/equality check against it an integer comparison instead of a string one.
# "stack"/"profile" are deliberately left as plain strings: they're often filtered down to a
# subset and then grouped by (see figures/*.py), and a categorical column keeps *all* of its
# original categories after filtering rows out unless every groupby call site remembers
# `observed=True` -- an easy way to silently reintroduce empty rows into a percentile table.
_METRICS_DTYPES = {"name": "category", "value": "float64", "ticks": "int64"}


def _read_metrics_csv(csv_path: Path, stack_name: str, profile_name: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path, dtype=_METRICS_DTYPES)
    df["stack"] = stack_name
    df["profile"] = profile_name
    if "seed" not in df.columns:
        df["seed"] = pd.NA
    return df


def discover_run(run_dir: Path) -> Tuple[Manifest, pd.DataFrame]:
    """Load a run's manifest and every metrics.csv under it into one tidy DataFrame.

    Columns: stack, profile, seed (NA if the CSV predates the seed column), name, value, ticks.
    Handles both the sweep layout (<stack>/<profile>/metrics.csv, many rows) and the single-session
    layout a live Unity or real-deployment recording writes (metrics.csv directly under run_dir) --
    the only thing that differs structurally between the three data-collection modes.
    """
    run_dir = Path(run_dir)
    manifest = Manifest.load(run_dir)
    frames: List[pd.DataFrame] = []

    if manifest.source == "sweep":
        tasks = []
        for stack in manifest.stacks:
            for profile in manifest.network_profiles:
                csv_path = run_dir / stack.name / profile / "metrics.csv"
                if csv_path.exists():
                    tasks.append((csv_path, stack.name, profile))

        # I/O + C-parser bound -- pandas' C engine releases the GIL while parsing, so a thread
        # pool gives real parallelism here without a process pool's setup/pickling overhead. A
        # dense sweep can mean 1000+ small files; reading them one at a time in a Python loop
        # was the dominant cost at that scale.
        with concurrent.futures.ThreadPoolExecutor() as executor:
            frames = list(executor.map(lambda task: _read_metrics_csv(*task), tasks))
    else:
        csv_path = run_dir / "metrics.csv"
        if csv_path.exists():
            stack_name = manifest.stacks[0].name if manifest.stacks else "session"
            profile_name = manifest.network_profiles[0] if manifest.network_profiles else "unknown"
            frames.append(_read_metrics_csv(csv_path, stack_name, profile_name))

    if not frames:
        raise FileNotFoundError(
            f"no metrics.csv found under {run_dir} for experiment {manifest.experiment_id!r} "
            f"-- check the manifest's stacks/networkProfiles match the directories on disk"
        )

    return manifest, pd.concat(frames, ignore_index=True)
