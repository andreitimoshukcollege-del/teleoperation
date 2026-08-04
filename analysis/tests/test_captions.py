from __future__ import annotations

from teleop_analysis.figures.captions import build_caption
from teleop_analysis.manifest import Manifest


def test_caption_contains_profile_seeds_and_sha():
    manifest = Manifest(
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
    caption = build_caption(manifest, "lan")
    assert "profile=lan" in caption
    assert "1,2,3" in caption
    assert "0123456789ab" in caption  # truncated sha, not the full hash
