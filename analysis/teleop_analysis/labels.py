from __future__ import annotations

import re
from typing import Dict, Iterable, List, Optional, Pattern, Tuple

# Human-readable names for the algorithms/stacks that actually exist. New predictors/stacks fall
# back to a title-cased, hyphen-to-space version of their registry key rather than needing an
# entry here -- this list is a readability nicety, not a completeness requirement.
FRIENDLY_STACK_NAMES: Dict[str, str] = {
    "none": "No prediction (baseline)",
    "const-vel": "Constant velocity",
    "double-exp": "Double exponential",
}


def friendly_stack_name(name: str) -> str:
    if name in FRIENDLY_STACK_NAMES:
        return FRIENDLY_STACK_NAMES[name]
    return name.replace("-", " ").replace("_", " ").title()


# Human-readable names for the network profiles that actually exist
# (core/Teleop.Eval/Sweep/NetworkProfileCatalog.cs). New profiles fall back to their raw name.
FRIENDLY_PROFILE_NAMES: Dict[str, str] = {
    "lan": "LAN (near-ideal)",
    "50ms-5j": "50ms delay, 5ms jitter",
    "150ms-20j-0.5loss": "150ms delay, 20ms jitter, 0.5% loss",
    "300ms-60j-2loss-bursty": "300ms delay, 60ms jitter, ~2% bursty loss",
    "synthetic-burst": "Recorded burst trace",
}


def friendly_profile_name(name: str) -> str:
    return FRIENDLY_PROFILE_NAMES.get(name, name)


# Per-axis scalar value lookup, mirroring core/Teleop.Eval/Sweep/NetworkProfileCatalog.cs by hand
# for the legacy bundled presets (kept here only for chart-axis positioning, not as a metric Core
# should be emitting itself -- analysis/CLAUDE.md: "if a plot needs a metric that Core doesn't
# emit, add the metric to Core rather than recomputing it here"; this is presentation metadata,
# not a recomputed metric), and by regex for the isolated single-variable families from
# docs/adr/0005-isolated-impairment-profiles.md ("jitter-<N>ms", "delay-<N>ms", "loss-<N>pct") --
# parsed rather than hand-listed so a new point in an existing family never needs a labels.py edit.
#
# Deliberately no legacy loss table: the two legacy profiles with nonzero loss have different
# burst lengths (ExpectedBurstLength ~1 vs ~3.3), so a loss-rate axis built from them would
# conflate rate with burst shape -- exactly the confound ADR 0005 exists to avoid. Only the
# isolated "loss-<N>pct" family (Bernoulli, burst length ~1 at every point) is loss-axis-clean.
_LEGACY_JITTER_MS: Dict[str, float] = {
    "lan": 1,
    "50ms-5j": 5,
    "150ms-20j-0.5loss": 20,
    "300ms-60j-2loss-bursty": 60,
}
_LEGACY_DELAY_MS: Dict[str, float] = {
    "lan": 2,
    "50ms-5j": 50,
    "150ms-20j-0.5loss": 150,
    "300ms-60j-2loss-bursty": 300,
}
_JITTER_RE = re.compile(r"^jitter-(\d+(?:\.\d+)?)ms$")
_DELAY_RE = re.compile(r"^delay-(\d+(?:\.\d+)?)ms$")
_LOSS_RE = re.compile(r"^loss-(\d+(?:\.\d+)?)pct$")

_AXES: Dict[str, Tuple[Dict[str, float], Pattern[str]]] = {
    "jitter": (_LEGACY_JITTER_MS, _JITTER_RE),
    "delay": (_LEGACY_DELAY_MS, _DELAY_RE),
    "loss": ({}, _LOSS_RE),
}

# x-axis label text per axis, used by figures/impairment_response.py.
AXIS_UNITS: Dict[str, str] = {"jitter": "ms", "delay": "ms", "loss": "%"}
AXIS_DESCRIPTIONS: Dict[str, str] = {
    "jitter": "higher = choppier connection",
    "delay": "higher = longer round trip",
    "loss": "higher = more dropped packets",
}


def axis_value(profile: str, axis: str) -> Optional[float]:
    """`profile`'s scalar value on `axis` ("jitter" | "delay" | "loss"), or None if this profile
    doesn't have one clean value on that axis (e.g. `synthetic-burst` on any axis, or a legacy
    preset on the loss axis).
    """
    legacy_table, pattern = _AXES[axis]
    if profile in legacy_table:
        return legacy_table[profile]
    match = pattern.match(profile)
    return float(match.group(1)) if match else None


def ordered_profiles_by_axis(profile_names: Iterable[str], axis: str) -> List[str]:
    """Only the profiles in `profile_names` with a known scalar value on `axis`, ascending."""
    values = {p: axis_value(p, axis) for p in profile_names}
    known = [p for p, v in values.items() if v is not None]
    return sorted(known, key=lambda p: values[p])


def ordered_profiles_by_jitter(profile_names: Iterable[str]) -> List[str]:
    """Back-compat alias for ordered_profiles_by_axis(profile_names, "jitter")."""
    return ordered_profiles_by_axis(profile_names, "jitter")


PERCENTILE_EXPLANATION = (
    "Solid line/darker bar = p50 (the typical case). Lighter/dashed = p95 (about 1 run in 20 is "
    "this bad or worse) -- the tail is what an operator actually notices, which is why it's "
    "shown alongside the typical case rather than replacing it with a single average."
)
