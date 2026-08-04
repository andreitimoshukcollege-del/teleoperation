from __future__ import annotations

from pathlib import Path
from typing import Tuple

import pandas as pd

from teleop_analysis.manifest import Manifest


def discover_run(run_dir: Path) -> Tuple[Manifest, pd.DataFrame]:
    """Load a run's manifest and every metrics.csv under it into one tidy DataFrame.

    Columns: stack, profile, seed (NA if the CSV predates the seed column), name, value, ticks.
    Handles both the sweep layout (<stack>/<profile>/metrics.csv, many rows) and the single-session
    layout a live Unity or real-deployment recording writes (metrics.csv directly under run_dir) --
    the only thing that differs structurally between the three data-collection modes.
    """
    run_dir = Path(run_dir)
    manifest = Manifest.load(run_dir)
    frames = []

    if manifest.source == "sweep":
        for stack in manifest.stacks:
            for profile in manifest.network_profiles:
                csv_path = run_dir / stack.name / profile / "metrics.csv"
                if not csv_path.exists():
                    continue
                df = pd.read_csv(csv_path)
                df["stack"] = stack.name
                df["profile"] = profile
                if "seed" not in df.columns:
                    df["seed"] = pd.NA
                frames.append(df)
    else:
        csv_path = run_dir / "metrics.csv"
        if csv_path.exists():
            df = pd.read_csv(csv_path)
            df["stack"] = manifest.stacks[0].name if manifest.stacks else "session"
            df["profile"] = manifest.network_profiles[0] if manifest.network_profiles else "unknown"
            if "seed" not in df.columns:
                df["seed"] = pd.NA
            frames.append(df)

    if not frames:
        raise FileNotFoundError(
            f"no metrics.csv found under {run_dir} for experiment {manifest.experiment_id!r} "
            f"-- check the manifest's stacks/networkProfiles match the directories on disk"
        )

    return manifest, pd.concat(frames, ignore_index=True)
