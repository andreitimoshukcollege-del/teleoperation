"""Test picker for analysis/tests/.

Run: <venv python> run_tests.py           -- opens a GUI window to pick which tests to run
     <venv python> run_tests.py --all     -- run everything, no window (scriptable; this is what
                                              CI or an agent verifying "did the tests pass" should
                                              use instead -- see analysis/CLAUDE.md's Testing
                                              section, which documents `pytest -v` directly for
                                              exactly that reason)
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Dict, List

import pytest

TESTS_DIR = Path(__file__).resolve().parent / "tests"


def collect_test_ids(tests_dir: Path = TESTS_DIR) -> List[str]:
    """Ask pytest itself which tests exist, so this list can never drift from reality."""
    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "--collect-only", "-q", str(tests_dir)],
        capture_output=True,
        text=True,
    )
    return [line.strip() for line in proc.stdout.splitlines() if "::" in line]


def group_by_file(node_ids: List[str]) -> Dict[str, List[str]]:
    """node id 'tests/test_x.py::test_y' -> {'tests/test_x.py': ['tests/test_x.py::test_y', ...]}"""
    grouped: Dict[str, List[str]] = {}
    for node_id in node_ids:
        file_part = node_id.split("::", 1)[0]
        grouped.setdefault(file_part, []).append(node_id)
    return grouped


def humanize_test_name(test_name: str) -> str:
    """'test_find_baseline_returns_exact_no_mitigation_stack' -> 'find baseline returns exact
    no mitigation stack' -- the raw snake_case name is the real pytest node id (unchanged, still
    used to actually run the test); this is display text only, for the checklist to scan quickly.
    """
    name = test_name[len("test_"):] if test_name.startswith("test_") else test_name
    name = name.replace("_", " ")
    return name[:1].upper() + name[1:] if name else name


def run_all() -> int:
    return pytest.main([str(TESTS_DIR), "-v"])


def run_gui() -> int:
    # Imported lazily so `run_tests.py --all` (the scriptable/CI path) never needs tkinter or a
    # display at all -- only opening the picker window does.
    import test_gui

    return test_gui.launch(collect_test_ids()) or 0


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--all", action="store_true", help="run every test without prompting (non-interactive)"
    )
    args = parser.parse_args(argv)

    if args.all:
        return run_all()
    return run_gui()


if __name__ == "__main__":
    sys.exit(main())
