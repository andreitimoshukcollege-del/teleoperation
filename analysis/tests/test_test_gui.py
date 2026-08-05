from __future__ import annotations

import sys
from pathlib import Path

# test_gui.py lives at analysis/ (one level above tests/), not inside the teleop_analysis
# package -- add it to sys.path explicitly rather than making it importable as a package module.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_gui import (  # noqa: E402
    _ZOOM_MAX,
    _ZOOM_MIN,
    _clamp_zoom,
    _zoom_scroll_fraction,
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


def test_clamp_zoom_passes_through_values_within_range():
    assert _clamp_zoom(1.0) == 1.0
    assert _clamp_zoom(2.5) == 2.5


def test_clamp_zoom_clamps_to_the_configured_min_and_max():
    assert _clamp_zoom(_ZOOM_MIN / 2) == _ZOOM_MIN
    assert _clamp_zoom(_ZOOM_MAX * 2) == _ZOOM_MAX


def test_zoom_scroll_fraction_keeps_the_same_center_when_zooming_in():
    # Viewing the middle of the image, now only 20% of the (larger) image fits on screen --
    # the top-left of the view should sit at 0.4 so the centered point (0.5) stays centered.
    assert _zoom_scroll_fraction(center_frac=0.5, visible_frac=0.2) == 0.4


def test_zoom_scroll_fraction_clamps_at_the_image_edges():
    # Near the left edge, centering exactly would go negative -- clamp to 0 instead.
    assert _zoom_scroll_fraction(center_frac=0.05, visible_frac=0.2) == 0.0
    # Near the right edge, centering exactly would overshoot past 1.0 -- clamp so the view's
    # trailing edge lands exactly on the image's right edge instead.
    assert _zoom_scroll_fraction(center_frac=0.95, visible_frac=0.2) == 0.8


def test_zoom_scroll_fraction_clamps_to_zero_when_the_whole_image_already_fits():
    # visible_frac > 1 means the image is smaller than the canvas (zoomed out, or a tiny
    # figure) -- there's nowhere to scroll to, so this must not go negative.
    assert _zoom_scroll_fraction(center_frac=0.5, visible_frac=1.5) == 0.0


def test_build_sweep_command_uses_absolute_yaml_path_and_dotnet_sweep_args():
    command = build_sweep_command(Path("experiments/exp-gui-sweep.yaml"))
    assert command[0] == "dotnet"
    assert command[1:3] == ["run", "--project"]
    assert command[4:6] == ["--", "sweep"]
    assert Path(command[3]).is_absolute()  # --project path
    assert Path(command[6]).is_absolute()  # yaml path
