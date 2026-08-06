from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import List, Optional, Sequence, Set

import pandas as pd

from teleop_analysis import io_utils, stats
from teleop_analysis.baseline import find_baseline
from teleop_analysis.figures import (
    combined_response,
    error_vs_cost,
    impairment_response,
    latency_distribution,
    stack_comparison,
    summary_table,
)
from teleop_analysis.manifest import Manifest

ALL_FIGURES = (
    "error-cost", "latency", "stack-comparison", "impairment-response", "combined-response",
    "table",
)

_BAR_CHART_KINDS = ("error-cost", "latency", "stack-comparison")


def _generate_profile_figures(
    profile_df: pd.DataFrame,
    manifest: Manifest,
    profile: str,
    figures_dir: Path,
    bar_kinds_requested: Set[str],
) -> List[Path]:
    """Runs in a worker process (see main()'s ProcessPoolExecutor) -- generates whichever
    per-profile bar-chart kinds were requested for exactly one profile, using `profile_df` (an
    already-filtered slice of the run's dataframe, not the whole thing) so each worker's task
    payload stays small regardless of how many profiles or rows the full sweep has.
    """
    written = []
    if "error-cost" in bar_kinds_requested:
        written.append(error_vs_cost.plot_error_vs_cost(profile_df, manifest, profile, figures_dir))
    if "latency" in bar_kinds_requested:
        written.append(latency_distribution.plot_latency_distribution(profile_df, manifest, profile, figures_dir))
    if "stack-comparison" in bar_kinds_requested:
        written.append(stack_comparison.plot_stack_comparison(profile_df, manifest, profile, figures_dir))
    return written


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Generate captioned figures and a p50/p95/p99 table from a Teleop results run "
            "directory (results/<experiment>/<timestamp>), regardless of whether it came from "
            "a sweep, a live Unity session, or a real-world deployment."
        )
    )
    parser.add_argument("run_dir", type=Path, help="results/<experiment>/<timestamp> directory")
    parser.add_argument(
        "--figures",
        default=",".join(ALL_FIGURES),
        help=f"comma-separated subset of {{{','.join(ALL_FIGURES)}}}",
    )
    args = parser.parse_args(argv)

    requested = set(args.figures.split(","))
    unknown = requested - set(ALL_FIGURES)
    if unknown:
        parser.error(f"unknown figure kind(s) {sorted(unknown)}; choose from {ALL_FIGURES}")

    run_dir = args.run_dir.resolve()
    manifest, df = io_utils.discover_run(run_dir)
    figures_dir = run_dir / "figures"

    baseline = find_baseline(manifest)
    if baseline is None:
        print(
            f"WARNING: no baseline stack found for {manifest.experiment_id!r} -- every "
            f"comparison below is missing the reference point that makes it interpretable.",
            file=sys.stderr,
        )

    written = []
    bar_kinds_requested = requested & set(_BAR_CHART_KINDS)
    if bar_kinds_requested:
        # A dense sweep means hundreds of profiles, each independently filtering the *entire*
        # dataframe down to itself inside plot_error_vs_cost/etc. -- pre-splitting once here
        # turns that into a cheap dict lookup per task, and running the tasks across processes
        # (matplotlib rendering is CPU/Python-bound, unlike io_utils.py's I/O-bound CSV reads,
        # so a thread pool wouldn't give real parallelism here) is what actually cuts wall time
        # on a multi-core machine.
        # dict(df.groupby(...)) -- without the iter() -- breaks: GroupBy has a `.keys`
        # *attribute* (whatever was passed to `by=`, here the string "profile"), and dict()
        # decides an arg is a mapping by checking hasattr(arg, "keys"), then calls it as
        # obj.keys() -- "profile"() -- "'str' object is not callable". iter() forces the
        # iterable-of-(name, group)-pairs protocol instead.
        profile_frames = dict(iter(df.groupby("profile", observed=True)))
        empty_frame = df.iloc[0:0]  # same columns/dtypes, for a profile with no rows at all
        with ProcessPoolExecutor() as executor:
            futures = [
                executor.submit(
                    _generate_profile_figures,
                    profile_frames.get(profile, empty_frame),
                    manifest, profile, figures_dir, bar_kinds_requested,
                )
                for profile in manifest.network_profiles
            ]
            for future in futures:
                written.extend(future.result())

    if "impairment-response" in requested:
        for path in (
            impairment_response.plot_correction_vs_jitter(df, manifest, figures_dir),
            impairment_response.plot_prediction_error_vs_jitter(df, manifest, figures_dir),
            impairment_response.plot_correction_vs_delay(df, manifest, figures_dir),
            impairment_response.plot_prediction_error_vs_delay(df, manifest, figures_dir),
            impairment_response.plot_correction_vs_loss(df, manifest, figures_dir),
            impairment_response.plot_prediction_error_vs_loss(df, manifest, figures_dir),
        ):
            if path is not None:
                written.append(path)

    if "combined-response" in requested:
        for path in (
            combined_response.plot_correction_vs_combined(df, manifest, figures_dir),
            combined_response.plot_prediction_error_vs_combined(df, manifest, figures_dir),
        ):
            if path is not None:
                written.append(path)

    if "table" in requested:
        table = summary_table.build_summary_table(df)
        written.append(summary_table.write_summary_table(table, figures_dir))
        print(table.to_string(index=False))

    spread = stats.seed_spread(df, ["stack", "profile", "name"])
    if spread is None:
        print(
            "NOTE: this run's metrics.csv has no 'seed' column -- seed-to-seed spread is not "
            "available for it (pre-Phase-6 sweep format). Do not infer stability from a single "
            "seed on the strength of this run alone."
        )
    else:
        print("Seed spread (max p50 - min p50 across seeds), by stack/profile/metric:")
        print(spread.to_string(index=False))

    for path in written:
        print(f"wrote {path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
