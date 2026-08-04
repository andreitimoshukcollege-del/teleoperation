from __future__ import annotations

from typing import Dict, Iterable, List, Optional

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


# Nominal jitter (ms) for every profile that has one fixed scalar value for it. This mirrors
# core/Teleop.Eval/Sweep/NetworkProfileCatalog.cs's parametric profiles by hand -- kept here only
# for chart-axis positioning, not as a metric Core should be emitting itself (analysis/CLAUDE.md:
# "if a plot needs a metric that Core doesn't emit, add the metric to Core rather than
# recomputing it here" -- this is presentation metadata, not a recomputed metric). Deliberately
# excludes "synthetic-burst": it replays a recorded trace, so it has no single jitter number to
# place on a numeric axis.
PROFILE_JITTER_MS: Dict[str, float] = {
    "lan": 1,
    "50ms-5j": 5,
    "150ms-20j-0.5loss": 20,
    "300ms-60j-2loss-bursty": 60,
}


def ordered_profiles_by_jitter(profile_names: Iterable[str]) -> List[str]:
    """Only the profiles in `profile_names` that have a known scalar jitter value, ascending."""
    known = [p for p in profile_names if p in PROFILE_JITTER_MS]
    return sorted(known, key=lambda p: PROFILE_JITTER_MS[p])


PERCENTILE_EXPLANATION = (
    "Solid line/darker bar = p50 (the typical case). Lighter/dashed = p95 (about 1 run in 20 is "
    "this bad or worse) -- the tail is what an operator actually notices, which is why it's "
    "shown alongside the typical case rather than replacing it with a single average."
)
