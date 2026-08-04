from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

from teleop_analysis import io_utils, stats
from teleop_analysis.baseline import find_baseline
from teleop_analysis.figures import (
    error_vs_cost,
    impairment_response,
    latency_distribution,
    stack_comparison,
    summary_table,
)

ALL_FIGURES = ("error-cost", "latency", "stack-comparison", "impairment-response", "table")


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
    for profile in manifest.network_profiles:
        if "error-cost" in requested:
            written.append(error_vs_cost.plot_error_vs_cost(df, manifest, profile, figures_dir))
        if "latency" in requested:
            written.append(latency_distribution.plot_latency_distribution(df, manifest, profile, figures_dir))
        if "stack-comparison" in requested:
            written.append(stack_comparison.plot_stack_comparison(df, manifest, profile, figures_dir))

    if "impairment-response" in requested:
        for path in (
            impairment_response.plot_correction_vs_impairment(df, manifest, figures_dir),
            impairment_response.plot_prediction_error_vs_impairment(df, manifest, figures_dir),
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
