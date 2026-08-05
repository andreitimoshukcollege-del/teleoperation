from __future__ import annotations

from pathlib import Path

from teleop_analysis import io_utils
from teleop_analysis.figures import error_vs_cost, latency_distribution, stack_comparison, summary_table


def test_discover_run_produces_tidy_frame_with_no_seed_column(synthetic_run: Path):
    manifest, df = io_utils.discover_run(synthetic_run)
    assert manifest.experiment_id == "exp-999-synthetic"
    assert set(df["stack"].unique()) == {"none", "fast"}
    assert set(df["profile"].unique()) == {"lan"}
    assert df["seed"].isna().all()


def test_figures_generate_without_crashing_on_a_two_stack_run(synthetic_run: Path, tmp_path: Path):
    manifest, df = io_utils.discover_run(synthetic_run)
    out_dir = tmp_path / "figures"

    p1 = error_vs_cost.plot_error_vs_cost(df, manifest, "lan", out_dir)
    p2 = latency_distribution.plot_latency_distribution(df, manifest, "lan", out_dir)
    # No multi-axis "combined" stack exists in this fixture -- the right panel must degrade
    # gracefully (an explanatory placeholder), not crash or fabricate data.
    p3 = stack_comparison.plot_stack_comparison(df, manifest, "lan", out_dir)

    for path in (p1, p2, p3):
        assert path.exists()
        assert path.stat().st_size > 0

    table = summary_table.build_summary_table(df)
    assert not table.empty
    written = summary_table.write_summary_table(table, out_dir)
    assert written.exists()


def test_build_figure_functions_return_a_live_figure_without_touching_disk(synthetic_run: Path):
    # The GUI's live figure view (test_gui.py) calls these directly instead of the plot_* wrappers
    # above, specifically to embed the Figure without ever writing/reloading a PNG.
    manifest, df = io_utils.discover_run(synthetic_run)

    fig1, caption1 = error_vs_cost.build_error_vs_cost_figure(df, manifest, "lan")
    fig2, caption2 = latency_distribution.build_latency_distribution_figure(df, manifest, "lan")
    fig3, caption3 = stack_comparison.build_stack_comparison_figure(df, manifest, "lan")

    for fig, caption in ((fig1, caption1), (fig2, caption2), (fig3, caption3)):
        assert len(fig.axes) == 2  # each of these is a two-panel comparison figure
        assert isinstance(caption, str) and caption
