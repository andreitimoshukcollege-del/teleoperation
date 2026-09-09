from __future__ import annotations

import warnings

import pytest

from teleop_analysis.baseline import find_baseline
from teleop_analysis.manifest import LEGACY_PLAYOUT_POLICY, Manifest, ResolvedStack


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


def test_a_legacy_run_still_finds_its_baseline_after_the_playout_rename():
    """`legacy-inline-playout` and `immediate` are both "no buffer".

    A manifest written before docs/adr/0012 records the former, and it is still that run's correct
    reference point -- refusing to recognise it would emit a "no baseline" warning on every result
    already on disk.
    """
    manifest = _manifest_with_stacks([
        ResolvedStack("none", "none", "snap", LEGACY_PLAYOUT_POLICY, "direct"),
        ResolvedStack("const-vel", "const-vel", "snap", LEGACY_PLAYOUT_POLICY, "direct"),
    ])

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert find_baseline(manifest).name == "none"


def test_a_buffered_stack_is_not_mistaken_for_the_baseline():
    """`fixed` is a mitigation on the buffering axis, so a run whose only `none`-predictor stack is
    buffered has no true no-mitigation-on-every-axis reference and must say so."""
    manifest = _manifest_with_stacks([ResolvedStack("none__fixed", "none", "snap", "fixed", "direct")])

    with pytest.warns(UserWarning, match="unbuffered playoutPolicy"):
        assert find_baseline(manifest).name == "none__fixed"
