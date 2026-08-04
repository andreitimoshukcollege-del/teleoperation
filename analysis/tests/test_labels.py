from __future__ import annotations

from teleop_analysis.labels import friendly_profile_name, friendly_stack_name, ordered_profiles_by_jitter


def test_friendly_stack_name_uses_known_mapping():
    assert friendly_stack_name("double-exp") == "Double exponential"
    assert friendly_stack_name("none") == "No prediction (baseline)"


def test_friendly_stack_name_falls_back_to_title_case_for_unknown_names():
    assert friendly_stack_name("some-new-thing") == "Some New Thing"


def test_friendly_profile_name_uses_known_mapping():
    assert friendly_profile_name("lan") == "LAN (near-ideal)"


def test_friendly_profile_name_falls_back_to_raw_name_for_unknown_profiles():
    assert friendly_profile_name("custom-profile") == "custom-profile"


def test_ordered_profiles_by_jitter_sorts_ascending_and_drops_unknown():
    ordered = ordered_profiles_by_jitter(
        ["300ms-60j-2loss-bursty", "lan", "synthetic-burst", "50ms-5j"]
    )
    assert ordered == ["lan", "50ms-5j", "300ms-60j-2loss-bursty"]


def test_ordered_profiles_by_jitter_handles_empty_input():
    assert ordered_profiles_by_jitter([]) == []
