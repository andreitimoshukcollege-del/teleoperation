"""What a sweep can be configured with, read from Core rather than hardcoded here.

The problem this solves. `test_gui.py` used to carry its own tuple of predictor names, with a
comment conceding there was "no runtime way to query the C# registry from Python. Update both
places by hand." Nobody did. By the time this module was written Core had fifteen registered
implementations and the GUI offered three -- it silently omitted every reconciler and every playout
policy, so a researcher using the GUI could not sweep most of the algorithms this project has
built, and nothing anywhere said so.

Python cannot load a .NET assembly, but it can run a process and read JSON. `Teleop.Eval catalog`
prints the live `Registries` tables and `NetworkProfileCatalog` contents, so this module has no
list of its own to fall out of date. Add a predictor to `Registries.cs` and it appears in the GUI
on the next launch, with no Python change at all.

Deliberately no fallback list. If the catalog cannot be read, callers get an exception and the GUI
says so, because a stale hardcoded list presented as current is exactly the failure being fixed --
and an empty GUI that explains itself is more useful than a confident one that lies.
"""

from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CORE_EVAL_DIR = REPO_ROOT / "core" / "Teleop.Eval"


class CatalogUnavailable(RuntimeError):
    """`Teleop.Eval catalog` could not be run or did not return usable JSON."""


@dataclass(frozen=True)
class IsolatedAxis:
    """One isolated-impairment family: its name, and the unit suffix its profile names use."""

    name: str
    unit_suffix: str

    def profile_name(self, value: str) -> str:
        """`delay-100ms`, `loss-1pct` -- the same construction Core's `IsolatedAxis` does."""
        return f"{self.name}-{value}{self.unit_suffix}"


@dataclass(frozen=True)
class Catalog:
    """Everything a sweep can select, as of the build that produced it."""

    predictors: List[str] = field(default_factory=list)
    reconcilers: List[str] = field(default_factory=list)
    playout_policies: List[str] = field(default_factory=list)
    codecs: List[str] = field(default_factory=list)
    transports: List[str] = field(default_factory=list)
    arbiters: List[str] = field(default_factory=list)
    named_profiles: List[str] = field(default_factory=list)
    trace_profiles: List[str] = field(default_factory=list)
    isolated_axes: List[IsolatedAxis] = field(default_factory=list)

    @property
    def algorithm_axes(self) -> Dict[str, List[str]]:
        """The selectable algorithm axes, in the order a sweep composes a stack.

        Codecs and transports are omitted on purpose: `sweep` does not vary them (there is one
        codec and `EmulatedTransport` is wired directly, not selected by name), so offering them
        would be a control that does nothing. Arbiters are omitted while the table is empty.
        """
        axes = {
            "predictor": self.predictors,
            "reconciler": self.reconcilers,
            "playout": self.playout_policies,
        }
        return {name: values for name, values in axes.items() if values}


def load(eval_dir: Path = CORE_EVAL_DIR, timeout_seconds: float = 180.0) -> Catalog:
    """Runs `Teleop.Eval catalog` and parses its JSON.

    Slow on a cold build -- it may compile Core and Eval first -- which is why callers should do
    this once at startup rather than per interaction.

    Invoked with an absolute `--project` path and no `cwd`, matching `test_gui.build_sweep_command`
    exactly. That shape is already proven on this project's Windows box, where the GUI runs under
    the Windows Python and `dotnet` resolves to the Windows SDK.
    """
    try:
        completed = subprocess.run(
            ["dotnet", "run", "--project", str(eval_dir), "--", "catalog"],
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
        )
    except FileNotFoundError as exc:
        raise CatalogUnavailable(
            "`dotnet` was not found. The catalog of available algorithms is read from Core at "
            "startup, so the .NET SDK is required even to configure a sweep."
        ) from exc
    except subprocess.TimeoutExpired as exc:
        raise CatalogUnavailable(
            f"`Teleop.Eval catalog` did not finish within {timeout_seconds:.0f}s. A cold build can "
            "be slow; try `cd core && dotnet build` first."
        ) from exc

    if completed.returncode != 0:
        raise CatalogUnavailable(
            "`Teleop.Eval catalog` failed "
            f"(exit {completed.returncode}). Is `core/` building?\n{completed.stderr.strip()}"
        )

    return parse(completed.stdout)


def parse(text: str) -> Catalog:
    """Parses the verb's JSON.

    Split from `load` so the parsing is testable without a .NET toolchain -- the analysis test
    suite runs on machines and CI jobs that may not have one.

    Tolerates a build banner or MSBuild noise before the JSON by starting at the first `{`, which
    `dotnet run` is entirely capable of emitting on a cold build.
    """
    start = text.find("{")
    if start < 0:
        raise CatalogUnavailable(
            f"`Teleop.Eval catalog` produced no JSON object. Output was:\n{text.strip()[:500]}"
        )

    try:
        raw = json.loads(text[start:])
    except json.JSONDecodeError as exc:
        raise CatalogUnavailable(
            f"`Teleop.Eval catalog` produced unparseable JSON: {exc}"
        ) from exc

    axes = [
        IsolatedAxis(name=entry["name"], unit_suffix=entry["unitSuffix"])
        for entry in raw.get("isolatedAxes", [])
    ]

    catalog = Catalog(
        predictors=list(raw.get("predictors", [])),
        reconcilers=list(raw.get("reconcilers", [])),
        playout_policies=list(raw.get("playoutPolicies", [])),
        codecs=list(raw.get("codecs", [])),
        transports=list(raw.get("transports", [])),
        arbiters=list(raw.get("arbiters", [])),
        named_profiles=list(raw.get("namedProfiles", [])),
        trace_profiles=list(raw.get("traceProfiles", [])),
        isolated_axes=axes,
    )

    if not catalog.predictors:
        raise CatalogUnavailable(
            "`Teleop.Eval catalog` reported no predictors. That cannot be right -- `none` is always "
            "registered -- so the output is probably not what this expects."
        )

    return catalog
