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
