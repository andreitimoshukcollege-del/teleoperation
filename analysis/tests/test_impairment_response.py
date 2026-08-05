from __future__ import annotations

from pathlib import Path

from teleop_analysis import io_utils
from teleop_analysis.figures import impairment_response


def test_jitter_plots_generate_with_two_known_jitter_profiles(
    synthetic_run_two_profiles: Path, tmp_path: Path
):
    manifest, df = io_utils.discover_run(synthetic_run_two_profiles)
    out_dir = tmp_path / "figures"

    correction_path = impairment_response.plot_correction_vs_jitter(df, manifest, out_dir)
    error_path = impairment_response.plot_prediction_error_vs_jitter(df, manifest, out_dir)

    for path in (correction_path, error_path):
        assert path is not None
        assert path.exists()
        assert path.stat().st_size > 0


def test_build_jitter_figure_functions_return_a_live_figure(
    synthetic_run_two_profiles: Path,
):
    # The GUI's live figure view (test_gui.py) calls these directly instead of the plot_*
    # wrappers above, specifically to embed the Figure without ever writing/reloading a PNG.
    manifest, df = io_utils.discover_run(synthetic_run_two_profiles)

    correction = impairment_response.build_correction_vs_jitter_figure(df, manifest)
    error = impairment_response.build_prediction_error_vs_jitter_figure(df, manifest)

    for result in (correction, error):
        assert result is not None
        fig, caption = result
        assert len(fig.axes) == 1
        assert isinstance(caption, str) and caption


def test_build_jitter_figure_returns_none_for_the_same_case_the_plot_wrapper_skips(
    synthetic_run: Path,
):
    manifest, df = io_utils.discover_run(synthetic_run)
    assert impairment_response.build_correction_vs_jitter_figure(df, manifest) is None


def test_delay_plots_generate_with_the_same_two_profiles(
    synthetic_run_two_profiles: Path, tmp_path: Path
):
    # "lan" and "50ms-5j" have known delay values too (2ms, 50ms) -- same fixture, different axis.
    manifest, df = io_utils.discover_run(synthetic_run_two_profiles)
    out_dir = tmp_path / "figures"

    correction_path = impairment_response.plot_correction_vs_delay(df, manifest, out_dir)
    error_path = impairment_response.plot_prediction_error_vs_delay(df, manifest, out_dir)

    for path in (correction_path, error_path):
        assert path is not None
        assert path.exists()
        assert path.stat().st_size > 0


def test_loss_plots_generate_with_two_isolated_loss_profiles(
    synthetic_run_two_loss_profiles: Path, tmp_path: Path
):
    manifest, df = io_utils.discover_run(synthetic_run_two_loss_profiles)
    out_dir = tmp_path / "figures"

    correction_path = impairment_response.plot_correction_vs_loss(df, manifest, out_dir)
    error_path = impairment_response.plot_prediction_error_vs_loss(df, manifest, out_dir)

    for path in (correction_path, error_path):
        assert path is not None
        assert path.exists()
        assert path.stat().st_size > 0


def test_loss_plots_return_none_for_legacy_profiles_with_no_clean_loss_value(
    synthetic_run_two_profiles: Path, tmp_path: Path
):
    # "lan"/"50ms-5j" have no loss-axis value at all (0% for both, and more importantly not a
    # deliberately isolated point) -- the loss chart must not silently plot them.
    manifest, df = io_utils.discover_run(synthetic_run_two_profiles)
    out_dir = tmp_path / "figures"

    assert impairment_response.plot_correction_vs_loss(df, manifest, out_dir) is None
    assert impairment_response.plot_prediction_error_vs_loss(df, manifest, out_dir) is None


def test_returns_none_when_fewer_than_two_jitter_comparable_profiles(
    synthetic_run: Path, tmp_path: Path
):
    # synthetic_run's fixture only has the "lan" profile -- not enough to draw a line.
    manifest, df = io_utils.discover_run(synthetic_run)
    out_dir = tmp_path / "figures"

    assert impairment_response.plot_correction_vs_jitter(df, manifest, out_dir) is None
    assert impairment_response.plot_prediction_error_vs_jitter(df, manifest, out_dir) is None


def test_excluded_note_names_a_handful_of_profiles():
    note = impairment_response._excluded_note(["lan", "synthetic-burst"], "jitter")
    assert note == " | excluded (no single jitter value): LAN (near-ideal), Recorded burst trace"


def test_excluded_note_falls_back_to_a_count_past_the_threshold():
    excluded = [f"delay-{n}ms" for n in range(50)]
    note = impairment_response._excluded_note(excluded, "jitter")
    assert note == " | excluded (no single jitter value): 50 profiles"
    # Never a wall of names, even though it could technically render one.
    assert "delay-0ms" not in note


def test_excluded_note_returns_empty_string_when_nothing_excluded():
    assert impairment_response._excluded_note([], "jitter") == ""
