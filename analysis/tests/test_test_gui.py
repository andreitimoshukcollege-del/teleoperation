from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pytest

from teleop_analysis import io_utils
from teleop_analysis.figures import combined_response, impairment_response

# test_gui.py lives at analysis/ (one level above tests/), not inside the teleop_analysis
# package -- add it to sys.path explicitly rather than making it importable as a package module.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_gui import (  # noqa: E402
    _ZOOM_STEP,
    _figure_builder_for_filename,
    _zoom_axes_around_point,
    build_report_command,
    build_sweep_command,
    delete_run,
    discover_runs,
    figures_for_run,
)


def test_discover_runs_finds_only_directories_with_a_manifest(tmp_path):
    results_dir = tmp_path / "results"
    run_a = results_dir / "exp-a" / "20260101-000000Z"
    run_b = results_dir / "exp-b" / "20260102-000000Z"
    run_a.mkdir(parents=True)
    run_b.mkdir(parents=True)
    (run_a / "manifest.json").write_text("{}")
    (run_b / "manifest.json").write_text("{}")
    # A directory with no manifest.json is not a citable run (results/CLAUDE.md) and must not
    # be offered as one.
    (results_dir / "exp-c" / "not-a-run").mkdir(parents=True)

    assert set(discover_runs(results_dir)) == {run_a, run_b}


def test_discover_runs_returns_empty_list_when_results_dir_missing(tmp_path):
    assert discover_runs(tmp_path / "does-not-exist") == []


def test_figures_for_run_lists_only_png_files(tmp_path):
    run_dir = tmp_path / "run"
    figures_dir = run_dir / "figures"
    figures_dir.mkdir(parents=True)
    (figures_dir / "b.png").write_bytes(b"")
    (figures_dir / "a.png").write_bytes(b"")
    (figures_dir / "summary_table.csv").write_bytes(b"")

    assert [p.name for p in figures_for_run(run_dir)] == ["a.png", "b.png"]


def test_figures_for_run_returns_empty_list_when_no_figures_dir(tmp_path):
    assert figures_for_run(tmp_path / "run-without-figures") == []


def test_build_report_command_uses_absolute_run_path():
    command = build_report_command(Path("results/exp-001/20260101-000000Z"))
    assert command[:3] == [sys.executable, "-m", "teleop_analysis.cli"]
    assert Path(command[3]).is_absolute()


def test_build_report_command_omits_figures_flag_by_default():
    command = build_report_command(Path("results/exp-001/20260101-000000Z"))
    assert "--figures" not in command


def test_build_report_command_appends_figures_flag_when_given():
    command = build_report_command(Path("results/exp-001/20260101-000000Z"), figures="table")
    assert command[-2:] == ["--figures", "table"]


def test_delete_run_removes_the_run_directory_and_its_contents(tmp_path):
    run_dir = tmp_path / "exp-a" / "20260101-000000Z"
    (run_dir / "figures").mkdir(parents=True)
    (run_dir / "manifest.json").write_text("{}")
    (run_dir / "figures" / "a.png").write_bytes(b"")

    delete_run(run_dir)

    assert not run_dir.exists()


def test_zoom_axes_around_point_zoom_in_shrinks_the_range_and_keeps_the_point_fixed():
    fig, ax = plt.subplots()
    try:
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        fig.canvas.draw()

        data_x, data_y = 30, 40
        x_px, y_px = ax.transData.transform((data_x, data_y))

        _zoom_axes_around_point(ax, x_px, y_px, zoom_in=True)

        new_xlo, new_xhi = ax.get_xlim()
        new_ylo, new_yhi = ax.get_ylim()
        assert (new_xhi - new_xlo) == pytest.approx(100 / _ZOOM_STEP)
        assert (new_yhi - new_ylo) == pytest.approx(100 / _ZOOM_STEP)

        # The same pixel position must still land on the same data point -- that's the whole
        # point of "zoom into where the mouse is pointing" rather than just shrinking the range.
        new_data_x, new_data_y = ax.transData.inverted().transform((x_px, y_px))
        assert new_data_x == pytest.approx(data_x)
        assert new_data_y == pytest.approx(data_y)
    finally:
        plt.close(fig)


def test_zoom_axes_around_point_zoom_out_grows_the_range():
    fig, ax = plt.subplots()
    try:
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        fig.canvas.draw()

        x_px, y_px = ax.transData.transform((50, 50))
        _zoom_axes_around_point(ax, x_px, y_px, zoom_in=False)

        new_xlo, new_xhi = ax.get_xlim()
        assert (new_xhi - new_xlo) == pytest.approx(100 * _ZOOM_STEP)
    finally:
        plt.close(fig)


def test_figure_builder_for_filename_maps_fixed_impairment_and_combined_names():
    assert (
        _figure_builder_for_filename("impairment__correction_vs_jitter.png")
        is impairment_response.build_correction_vs_jitter_figure
    )
    assert (
        _figure_builder_for_filename("combined__prediction_error.png")
        is combined_response.build_prediction_error_vs_combined_figure
    )


def test_figure_builder_for_filename_returns_none_for_an_unknown_name():
    # "table"'s summary_table.csv isn't a .png and never reaches this, but a defensive check
    # against an unrecognized name is still correct behavior.
    assert _figure_builder_for_filename("summary_table.csv") is None
    assert _figure_builder_for_filename("something-unrelated.png") is None


def test_figure_builder_for_filename_extracts_the_profile_for_per_profile_kinds(synthetic_run: Path):
    manifest, df = io_utils.discover_run(synthetic_run)
    builder = _figure_builder_for_filename("lan__error_vs_cost.png")
    assert builder is not None

    result = builder(df, manifest)
    assert result is not None
    fig, caption = result
    assert "LAN" in caption  # build_caption embeds the friendly name of the extracted profile
    plt.close(fig)


def test_build_sweep_command_uses_absolute_yaml_path_and_dotnet_sweep_args():
    command = build_sweep_command(Path("experiments/exp-gui-sweep.yaml"))
    assert command[0] == "dotnet"
    assert command[1:3] == ["run", "--project"]
    assert command[4:6] == ["--", "sweep"]
    assert Path(command[3]).is_absolute()  # --project path
    assert Path(command[6]).is_absolute()  # yaml path
