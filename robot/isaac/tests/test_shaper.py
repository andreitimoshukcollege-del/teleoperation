"""The bridge's shaper against Core's, bit for bit (golden/joint-shaper-v1.json, the trainer's golden, written by Core)."""

from conftest import golden, profile_path
from teleop_isaac import profile
from teleop_isaac.shaper import ShapedState, TrajectoryShaperConfig, shaper_for_profile, step


def test_every_shaper_step_matches_core_bit_for_bit():
    g = golden("joint-shaper-v1.json")
    shaper = TrajectoryShaperConfig.from_golden(g["shaper"])
    assert len(g["cases"]) == len(shaper.joints) == 5
    for case in g["cases"]:
        limits = shaper.joints[shaper.index_of(case["motorId"])]
        s = ShapedState.at_rest(case["start"])
        for i, row in enumerate(case["steps"]):
            step(s, row[0], limits, shaper)
            where = f"motor {case['motorId']} step {i}"
            assert s.q == row[1], f"{where}: q {s.q!r} != {row[1]!r}"
            if limits.is_finger:
                assert int(s.moving) == row[2] and s.to == row[3], where
            else:
                assert s.v == row[2] and s.a == row[3], f"{where}: v/a {s.v!r}/{s.a!r} != {row[2]!r}/{row[3]!r}"


def test_the_profiles_shaper_is_in_robothosts_motor_order():
    p = profile.load(profile_path())
    c = shaper_for_profile(p)
    assert [j.motor_id for j in c.joints] == p.motor_ids() == [1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18]
    assert [j.is_finger for j in c.joints] == [False] * 7 + [True] + [False] * 7 + [True]
    hand = c.joints[7]
    assert hand.finger_speed == 0.2 / 0.04  # the Franka Hand: 0.2 m/s over 4 cm of travel, as a fraction per second
