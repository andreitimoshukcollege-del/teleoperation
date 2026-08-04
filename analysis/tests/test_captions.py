from __future__ import annotations

from teleop_analysis.figures.captions import build_caption, build_caption_multi_profile
from teleop_analysis.manifest import Manifest


def _manifest(**overrides) -> Manifest:
    defaults = dict(
        experiment_id="exp-test",
        git_sha="0123456789abcdef",
        seeds=[1, 2, 3],
        stacks=[],
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
    defaults.update(overrides)
    return Manifest(**defaults)


def test_caption_contains_profile_seeds_and_sha():
    caption = build_caption(_manifest(), "lan")
    # Profile names are shown human-readably, not as the raw registry key.
    assert "profile=LAN" in caption
    assert "1,2,3" in caption
    assert "0123456789ab" in caption  # truncated sha, not the full hash


def test_caption_falls_back_to_raw_name_for_unknown_profile():
    caption = build_caption(_manifest(), "some-future-profile")
    assert "profile=some-future-profile" in caption


def test_multi_profile_caption_has_no_profile_field_but_keeps_seeds_and_sha():
    caption = build_caption_multi_profile(_manifest())
    assert "profile=" not in caption
    assert "1,2,3" in caption
    assert "0123456789ab" in caption
