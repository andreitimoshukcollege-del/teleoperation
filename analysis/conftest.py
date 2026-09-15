"""Pytest configuration for `analysis/`.

Exists for one reason: `test_gui.py` is the tkinter GUI application, not a test module, but its
name matches pytest's default `test_*.py` discovery pattern, so pytest imports it during
collection. On a headless box without the `tkinter` system package that import raises
`ModuleNotFoundError`, and a collection error aborts the **entire** suite rather than skipping one
file -- so `just test` and `just check` failed outright on a machine that could run every test
that matters.

That was papered over for a while by passing `--ignore` flags on the command line, which is worse
than it sounds: the flags lived in people's shell history rather than in the repo, so the
documented command was simply wrong, and a new contributor's first `just check` failed.

`tkinter` is deliberately *not* added as a requirement. `analysis/CLAUDE.md` describes the GUI as
a human-facing convenience on the Windows box, and the scriptable path -- `pytest`, which is what
CI and agents use -- has never needed a display. Requiring a GUI toolkit to run headless unit
tests would be the wrong fix.
"""
from __future__ import annotations

import importlib.util

collect_ignore: list[str] = []

if importlib.util.find_spec("tkinter") is None:
    collect_ignore.append("test_gui.py")
