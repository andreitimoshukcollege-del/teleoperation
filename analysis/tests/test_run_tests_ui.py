from __future__ import annotations

import sys
from pathlib import Path

# run_tests.py lives at analysis/ (one level above tests/), not inside the teleop_analysis
# package -- add it to sys.path explicitly rather than making it importable as a package module.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_tests import collect_test_ids, group_by_file, humanize_test_name  # noqa: E402
from test_gui import build_pytest_command, find_summary_line  # noqa: E402


def test_collect_test_ids_finds_known_tests():
    node_ids = collect_test_ids()
    assert any("test_percentiles.py::test_summarize_matches_hand_computed_median" in n for n in node_ids)
    assert any("test_baseline.py::test_find_baseline_returns_exact_no_mitigation_stack" in n for n in node_ids)
    # Every collected id must actually be a node id, not a warning/summary line.
    assert all("::" in n for n in node_ids)


def test_group_by_file_groups_synthetic_ids_correctly():
    ids = [
        "tests/test_a.py::test_one",
        "tests/test_a.py::test_two",
        "tests/test_b.py::test_three",
    ]
    grouped = group_by_file(ids)
    assert set(grouped.keys()) == {"tests/test_a.py", "tests/test_b.py"}
    assert grouped["tests/test_a.py"] == ["tests/test_a.py::test_one", "tests/test_a.py::test_two"]
    assert grouped["tests/test_b.py"] == ["tests/test_b.py::test_three"]


def test_group_by_file_handles_empty_input():
    assert group_by_file([]) == {}


def test_humanize_test_name_strips_prefix_and_spaces_words():
    assert (
        humanize_test_name("test_find_baseline_returns_exact_no_mitigation_stack")
        == "Find baseline returns exact no mitigation stack"
    )


def test_humanize_test_name_handles_name_without_test_prefix():
    assert humanize_test_name("already_a_name") == "Already a name"


def test_humanize_test_name_handles_empty_string():
    assert humanize_test_name("") == ""


def test_build_pytest_command_includes_selected_ids_and_verbose_flag():
    import sys

    command = build_pytest_command(["tests/test_a.py::test_one", "tests/test_b.py::test_two"])
    assert command[0] == sys.executable
    assert command[1:4] == ["-m", "pytest", "tests/test_a.py::test_one"]
    assert command[-2:] == ["tests/test_b.py::test_two", "-v"]


def test_build_pytest_command_handles_no_selected_ids():
    import sys

    assert build_pytest_command([]) == [sys.executable, "-m", "pytest", "-v"]


def test_find_summary_line_extracts_passing_summary():
    lines = [
        "collecting ...\n",
        "tests/test_a.py::test_one PASSED\n",
        "==================== 17 passed, 14 warnings in 3.21s ====================\n",
    ]
    assert find_summary_line(lines) == "17 passed, 14 warnings in 3.21s"


def test_find_summary_line_extracts_failing_summary():
    lines = [
        "tests/test_a.py::test_one FAILED\n",
        "=================== 1 failed, 16 passed in 2.05s ===================\n",
    ]
    assert find_summary_line(lines) == "1 failed, 16 passed in 2.05s"


def test_find_summary_line_returns_none_when_no_summary_present():
    assert find_summary_line(["just some output\n", "no summary line here\n"]) is None


def test_find_summary_line_returns_none_for_empty_input():
    assert find_summary_line([]) is None
