"""Entry point for analysis/tests/.

Run: <venv python> run_tests.py           -- opens the analysis/ GUI (configure and run a sweep,
                                              view its figures; see test_gui.py)
     <venv python> run_tests.py --all     -- run the analysis/ pytest suite, no window
                                              (scriptable; this is what CI or an agent verifying
                                              "did the tests pass" should use -- see
                                              analysis/CLAUDE.md's Testing section, which
                                              documents `pytest -v` directly for exactly that
                                              reason)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pytest

TESTS_DIR = Path(__file__).resolve().parent / "tests"


def run_all() -> int:
    return pytest.main([str(TESTS_DIR), "-v"])


def run_gui() -> int:
    # Imported lazily so `run_tests.py --all` (the scriptable/CI path) never needs tkinter or a
    # display at all -- only opening the window does.
    import test_gui

    return test_gui.launch() or 0


def main(argv: "list[str] | None" = None) -> int:
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
