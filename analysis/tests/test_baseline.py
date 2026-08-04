from __future__ import annotations

import warnings

from teleop_analysis.baseline import find_baseline
from teleop_analysis.manifest import Manifest, ResolvedStack


def _manifest_with_stacks(stacks):
    return Manifest(
        experiment_id="exp-test",
        git_sha="abc123",
        seeds=[1],
        stacks=stacks,
        network_profiles=["lan"],
        trial_steps=1,
        step_interval_ticks=1,
        config_path="",
        machine="",
        command="",
        generated_at_utc="",
        source="sweep",
        path=None,
    )


def test_find_baseline_returns_exact_no_mitigation_stack():
    stacks = [
        ResolvedStack("none", "none", "snap", "immediate", "direct"),
        ResolvedStack("double-exp", "double-exp", "snap", "immediate", "direct"),
    ]
    baseline = find_baseline(_manifest_with_stacks(stacks))
    assert baseline is not None
    assert baseline.name == "none"


def test_find_baseline_warns_loudly_and_returns_none_when_missing():
    stacks = [ResolvedStack("double-exp", "double-exp", "snap", "immediate", "direct")]
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        baseline = find_baseline(_manifest_with_stacks(stacks))
    assert baseline is None
    assert any("no baseline stack" in str(w.message) for w in caught)


def test_find_baseline_warns_but_still_returns_when_playout_or_arbiter_nonstandard():
    stacks = [ResolvedStack("none", "none", "snap", "fixed", "ladder")]
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        baseline = find_baseline(_manifest_with_stacks(stacks))
    assert baseline is not None
    assert baseline.name == "none"
    assert any("not a true no-mitigation" in str(w.message) for w in caught)
