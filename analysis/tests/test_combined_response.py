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
