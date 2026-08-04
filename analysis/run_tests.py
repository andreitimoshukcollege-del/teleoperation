"""Interactive test picker for analysis/tests/.

Run: <venv python> run_tests.py           -- pick which tests to run from a checklist
     <venv python> run_tests.py --all     -- run everything, no prompt (scriptable; this is what
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


def run_all() -> int:
    return pytest.main([str(TESTS_DIR), "-v"])


def run_interactive() -> int:
    import questionary

    node_ids = collect_test_ids()
    if not node_ids:
        print("No tests collected -- check that analysis/tests/ exists and pytest can import it.")
        return 1

    choices = []
    for file_path, ids_in_file in group_by_file(node_ids).items():
        choices.append(questionary.Separator(f"-- {file_path} --"))
        for node_id in ids_in_file:
            test_name = node_id.split("::", 1)[1]
            choices.append(questionary.Choice(title=test_name, value=node_id, checked=True))

    selected = questionary.checkbox(
        "Select tests to run (space to toggle, enter to run, ctrl-c to cancel):",
        choices=choices,
    ).ask()

    if selected is None:
        print("Cancelled.")
        return 130
    if not selected:
        print("Nothing selected.")
        return 0

    print(f"\nRunning {len(selected)} test(s)...\n")
    return pytest.main([*selected, "-v"])


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--all", action="store_true", help="run every test without prompting (non-interactive)"
    )
    args = parser.parse_args(argv)

    if args.all:
        return run_all()
    return run_interactive()


if __name__ == "__main__":
    sys.exit(main())
