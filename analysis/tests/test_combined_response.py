from __future__ import annotations

from pathlib import Path

from teleop_analysis import io_utils
from teleop_analysis.figures import combined_response


def test_thinned_tick_indices_labels_every_step_below_the_threshold():
    assert combined_response._thinned_tick_indices(5) == [0, 1, 2, 3, 4]
    assert combined_response._thinned_tick_indices(combined_response._MAX_TICK_LABELS) == list(
        range(combined_response._MAX_TICK_LABELS)
    )


def test_thinned_tick_indices_thins_a_dense_sweep_and_keeps_the_last_step():
    indices = combined_response._thinned_tick_indices(301)
    assert len(indices) <= combined_response._MAX_TICK_LABELS + 1  # +1 for the appended last step
    assert indices[-1] == 300
    assert indices == sorted(indices)  # still left-to-right


def test_thinned_tick_indices_in_range_labels_only_the_visible_window():
    # Zoomed into steps 100-105 of a 301-step sweep -- well under _MAX_TICK_LABELS, so every
    # visible step should get a label, and nothing outside the window.
    indices = combined_response._thinned_tick_indices_in_range(301, 100, 105)
    assert indices == list(range(100, 106))


def test_thinned_tick_indices_in_range_thins_a_dense_visible_window():
    # Zoomed out to steps 0-300 of a 301-step sweep -- same as the whole-sweep case.
    indices = combined_response._thinned_tick_indices_in_range(301, 0, 300)
    assert indices == combined_response._thinned_tick_indices(301)


def test_thinned_tick_indices_in_range_clamps_to_the_step_bounds():
    # A view scrolled/zoomed past the sweep's own edges must not request out-of-range indices.
    indices = combined_response._thinned_tick_indices_in_range(10, -5, 3)
    assert indices == [0, 1, 2, 3]
    assert combined_response._thinned_tick_indices_in_range(10, 12, 20) == []


def test_marker_style_shows_markers_below_the_cutoff_and_hides_them_above_it():
    marker, size = combined_response._marker_style(combined_response._MARKER_CUTOFF)
    assert marker is not None and size is not None

    marker, size = combined_response._marker_style(combined_response._MARKER_CUTOFF + 1)
    assert marker is None and size is None


def test_plots_generate_as_one_chart_across_the_whole_combined_sweep(
    synthetic_run_combined_profiles: Path, tmp_path: Path
):
    manifest, df = io_utils.discover_run(synthetic_run_combined_profiles)
    out_dir = tmp_path / "figures"

    correction_path = combined_response.plot_correction_vs_combined(df, manifest, out_dir)
    error_path = combined_response.plot_prediction_error_vs_combined(df, manifest, out_dir)

    for path in (correction_path, error_path):
        assert path is not None
        assert path.exists()
        assert path.stat().st_size > 0

    # One chart per metric for the whole 3-step sweep, not one per combined profile.
    assert correction_path.name == "combined__correction.png"
    assert error_path.name == "combined__prediction_error.png"


def test_returns_none_when_fewer_than_two_combined_profiles(synthetic_run: Path, tmp_path: Path):
    # synthetic_run's fixture only has the "lan" profile -- not a combo__ name at all.
    manifest, df = io_utils.discover_run(synthetic_run)
    out_dir = tmp_path / "figures"

    assert combined_response.plot_correction_vs_combined(df, manifest, out_dir) is None
    assert combined_response.plot_prediction_error_vs_combined(df, manifest, out_dir) is None


def test_returns_none_when_isolated_axis_profiles_are_not_combined_profiles(
    synthetic_run_two_profiles: Path, tmp_path: Path
):
    # "lan"/"50ms-5j" aren't combo__ names -- must not be swept into a combined chart.
    manifest, df = io_utils.discover_run(synthetic_run_two_profiles)
    out_dir = tmp_path / "figures"

    assert combined_response.plot_correction_vs_combined(df, manifest, out_dir) is None


def test_build_correction_vs_combined_figure_returns_a_live_figure(
    synthetic_run_combined_profiles: Path,
):
    manifest, df = io_utils.discover_run(synthetic_run_combined_profiles)

    result = combined_response.build_correction_vs_combined_figure(df, manifest)

    assert result is not None
    fig, caption = result
    assert isinstance(caption, str) and caption
    assert len(fig.axes) == 1


def test_build_prediction_error_vs_combined_figure_returns_a_live_figure(
    synthetic_run_combined_profiles: Path,
):
    manifest, df = io_utils.discover_run(synthetic_run_combined_profiles)

    result = combined_response.build_prediction_error_vs_combined_figure(df, manifest)

    assert result is not None


def test_zooming_the_live_figure_rethins_ticks_to_the_visible_range(
    synthetic_run_combined_profiles: Path,
):
    # synthetic_run_combined_profiles has 3 steps (well under _MAX_TICK_LABELS), so simulate a
    # dense sweep's zoom behavior by shrinking the threshold instead of building a huge fixture.
    original_max = combined_response._MAX_TICK_LABELS
    combined_response._MAX_TICK_LABELS = 1
    try:
        manifest, df = io_utils.discover_run(synthetic_run_combined_profiles)
        fig, _caption = combined_response.build_correction_vs_combined_figure(df, manifest)
        ax = fig.axes[0]

        full_ticks = list(ax.get_xticks())
        ax.set_xlim(2.0, 2.9)  # zoom into just the last step -- triggers the xlim_changed callback
        zoomed_ticks = list(ax.get_xticks())

        assert zoomed_ticks != full_ticks
        assert zoomed_ticks == [2]
    finally:
        combined_response._MAX_TICK_LABELS = original_max
