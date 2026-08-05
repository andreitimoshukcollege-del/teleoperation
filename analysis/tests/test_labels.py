from __future__ import annotations

from teleop_analysis.labels import (
    axis_value,
    friendly_profile_name,
    friendly_stack_name,
    ordered_profiles_by_axis,
    ordered_profiles_by_jitter,
)


def test_friendly_stack_name_uses_known_mapping():
    assert friendly_stack_name("double-exp") == "Double exponential"
    assert friendly_stack_name("none") == "No prediction (baseline)"


def test_friendly_stack_name_falls_back_to_title_case_for_unknown_names():
    assert friendly_stack_name("some-new-thing") == "Some New Thing"


def test_friendly_profile_name_uses_known_mapping():
    assert friendly_profile_name("lan") == "LAN (near-ideal)"


def test_friendly_profile_name_falls_back_to_raw_name_for_unknown_profiles():
    assert friendly_profile_name("custom-profile") == "custom-profile"


def test_friendly_profile_name_formats_combined_profiles_in_canonical_axis_order():
    # jitter listed before delay in the name -- output must still be delay, jitter, loss order.
    name = "combo__jitter-20ms__delay-150ms__loss-0.5pct"
    assert friendly_profile_name(name) == "150ms delay, 20ms jitter, 0.5% loss (combined)"


def test_friendly_profile_name_formats_combined_profiles_with_omitted_axes():
    assert friendly_profile_name("combo__jitter-30ms__loss-1pct") == "30ms jitter, 1% loss (combined)"


def test_friendly_profile_name_falls_back_to_raw_name_for_malformed_combined_profile():
    assert friendly_profile_name("combo__not-a-real-axis") == "combo__not-a-real-axis"


def test_axis_value_returns_none_for_every_axis_on_a_combined_profile():
    name = "combo__delay-150ms__jitter-20ms__loss-0.5pct"
    assert axis_value(name, "delay") is None
    assert axis_value(name, "jitter") is None
    assert axis_value(name, "loss") is None


def test_ordered_profiles_by_jitter_sorts_ascending_and_drops_unknown():
    ordered = ordered_profiles_by_jitter(
        ["300ms-60j-2loss-bursty", "lan", "synthetic-burst", "50ms-5j"]
    )
    assert ordered == ["lan", "50ms-5j", "300ms-60j-2loss-bursty"]


def test_ordered_profiles_by_jitter_handles_empty_input():
    assert ordered_profiles_by_jitter([]) == []


def test_axis_value_legacy_presets_have_jitter_and_delay_but_not_loss():
    assert axis_value("50ms-5j", "jitter") == 5
    assert axis_value("50ms-5j", "delay") == 50
    assert axis_value("150ms-20j-0.5loss", "loss") is None  # deliberately not clean (ADR 0005)


def test_axis_value_parses_isolated_axis_family_names():
    assert axis_value("jitter-15ms", "jitter") == 15
    assert axis_value("delay-250ms", "delay") == 250
    assert axis_value("loss-0.5pct", "loss") == 0.5
    assert axis_value("loss-5pct", "loss") == 5


def test_axis_value_returns_none_for_unrecognized_profile():
    assert axis_value("synthetic-burst", "jitter") is None
    assert axis_value("some-future-profile", "delay") is None


def test_ordered_profiles_by_axis_sorts_ascending_and_drops_unknown():
    ordered = ordered_profiles_by_axis(
        ["loss-5pct", "synthetic-burst", "loss-0pct", "loss-1.5pct"], "loss"
    )
    assert ordered == ["loss-0pct", "loss-1.5pct", "loss-5pct"]


def test_ordered_profiles_by_axis_delay_matches_jitter_axis_ordering_for_legacy_presets():
    # For the 4 legacy presets, delay and jitter happen to increase together -- same order.
    profiles = ["300ms-60j-2loss-bursty", "lan", "150ms-20j-0.5loss", "50ms-5j"]
    assert ordered_profiles_by_axis(profiles, "delay") == ordered_profiles_by_axis(profiles, "jitter")
