"""A take into its episode frames (docs/adr/0036 decisions 2-4), without LeRobot."""

import json
import os

import pytest

from conftest import MS, REPO, finish, fr3_take, jetrover_take, relay_pulse
from teleop_lerobot.takes import TakeError, raw_events, read_take, verify_checksums


def test_a_jetrover_frame_holds_the_newest_reading_and_command_at_the_time_it_shows(scenarios):
    take = read_take(jetrover_take(scenarios))
    assert (take.robot, take.source, take.dataset_group, take.scenario) == ("jetrover", "isaac", "jetrover-isaac", "reach")
    assert take.task == "reach the red cube"
    assert take.names == ["servo_1", "servo_2", "servo_3", "servo_4", "servo_5", "servo_10"]
    for f in take.frames:
        t = f.timing["sensed"]
        k_ri = (t - 905 * MS) // (20 * MS)  # the newest RI at or before t (RI at 0.905 + 0.020 k s)
        k_ro = (t - 900 * MS) // (20 * MS)
        # Motor 3's reading k=60 was invalid: the one before stands.
        k3 = k_ri - 1 if k_ri == 60 else k_ri
        assert f.state[:5] == [401 + k_ri, 402 + k_ri, 403 + k3, 404 + k_ri, 405 + k_ri]
        assert f.action == [501.5 + k_ro, 502.5 + k_ro, 503.5 + k_ro, 504.5 + k_ro, 505.5 + k_ro, 510.5 + k_ro]
        assert f.state[5] == f.action[5]  # the gripper's slot: its last command, as the lab's recorder has it
        assert f.timing["stateAge"] == t - (905 * MS + 20 * MS * min(k_ri, k3))  # the oldest reading it holds


def test_an_episode_runs_from_the_first_to_the_last_accepted_command_at_15_fps(scenarios):
    take = read_take(jetrover_take(scenarios))
    sensed = [f.timing["sensed"] for f in take.frames]
    assert sensed[0] >= 1000 * MS and sensed[-1] <= 3000 * MS
    # 30 fps recorded, 15 written: every other frame, from the first inside the window; 1.0 to 3.0 s is 31.
    assert [f.timing["frame"] for f in take.frames][:4] == [15, 17, 19, 21]
    assert len(take.frames) == 31 and not any(f.timing["filled"] for f in take.frames)
    assert all(b - a == pytest.approx(1e9 / 15, abs=1.5e6) for a, b in zip(sensed, sensed[1:]))


def test_frames_that_wobble_in_time_are_still_taken_evenly(scenarios):
    # The twin's frames wobble by a few ms (2026-11-15): taking the newest frame at most 1 ms past each point picked frames
    # 1, 2 or 3 apart. The nearest frame keeps them 2 apart, every step within a frame's wobble of 1/15 s.
    take = read_take(jetrover_take(scenarios, jitter=4.0))
    numbers = [f.timing["frame"] for f in take.frames]
    assert all(b - a == 2 for a, b in zip(numbers, numbers[1:])), numbers
    assert all(abs(f.timing["sensed"] - f.timing["grid"]) <= 5 * MS for f in take.frames)
    assert not any(f.timing["filled"] for f in take.frames)


def test_a_gap_in_the_recording_repeats_the_last_frame_and_says_so(scenarios):
    take = read_take(jetrover_take(scenarios, gap=(2.0, 2.3)))
    filled = [f for f in take.frames if f.timing["filled"]]
    assert 3 <= len(filled) <= 5
    assert all(f.timing["frame"] == take.frames[take.frames.index(f) - 1].timing["frame"] for f in filled)


def test_an_fr3_frame_joins_its_state_exactly_and_hands_become_finger_travel(scenarios):
    take = read_take(fr3_take(scenarios))
    assert (take.robot, take.dataset_group, take.task) == ("fr3_duo", "fr3_duo-isaac", "handover")  # no description
    assert take.names[7] == "left_gripper" and take.names[15] == "right_gripper" and len(take.names) == 16
    for f in take.frames:
        t = f.timing["sensed"]
        k = t // (4 * MS)
        assert f.state[0] == pytest.approx(0.01 + 0.001 * k) and f.state[8] == pytest.approx(0.11 + 0.001 * k)
        assert f.state[7] == pytest.approx(0.02) and f.state[15] == pytest.approx(0.02)  # 0.5 opening x 0.04 m
        assert f.action[7] == pytest.approx(0.04) and f.timing["stateAge"] == 0
    assert 99 not in [f.timing["frame"] for f in take.frames]


def test_the_frames_images_are_the_recorded_jpegs(scenarios):
    take = read_take(jetrover_take(scenarios))
    f = take.frames[0]
    assert {c.name: take.jpeg(f, c) for c in take.cameras} == {
        "third-left": f"third-left-{f.timing['frame']}".encode(), "wrist": f"wrist-{f.timing['frame']}".encode()}


def test_the_real_robots_camera_is_front(scenarios):
    take = read_take(jetrover_take(scenarios, source="robot"))
    assert take.dataset_group == "jetrover-real" and [c.name for c in take.cameras] == ["front"]


def test_a_take_that_does_not_match_its_checksums_is_refused(scenarios):
    path = jetrover_take(scenarios)
    verify_checksums(path)
    with open(os.path.join(path, "isaac", "wrist.rgb.frames"), "r+b") as f:
        f.seek(20)
        f.write(b"X")
    with pytest.raises(TakeError, match="does not match"):
        verify_checksums(path)


def test_a_session_that_is_not_a_take_or_has_no_frames_is_refused(scenarios):
    path = jetrover_take(scenarios)
    import json
    meta_path = os.path.join(path, "session.json")
    meta = json.load(open(meta_path))
    json.dump({**meta, "scenario": None}, open(meta_path, "w"))
    with pytest.raises(TakeError, match="not a take"):
        read_take(path)
    json.dump({**meta, "source": "sim-host"}, open(meta_path, "w"))
    with pytest.raises(TakeError, match="records no frames"):
        read_take(path)


def test_a_cut_off_frames_log_is_refused(scenarios):
    path = jetrover_take(scenarios)
    log = os.path.join(path, "isaac.jlog")
    data = open(log, "rb").read()
    open(log, "wb").write(data[: data.rfind(b"END|")])
    finish(path)
    with pytest.raises(ValueError, match="trailer"):
        read_take(path)


# The JetRover's action is the command sent (docs/adr/0038 "M, motion", 2026-11-24). RobotHost refused 146 of a source take's 814
# commands for 3.0 s (episode 23 of jetrover_20261124_021454_cb1e21: its collision backstop judges the servo path from where the
# arm is measured, docs/adr/0020) and 32 of its replay's, which sent the same 814 commands; the action, then the newest RO,
# differed between the two by 258 servo units at p95.

MOTORS = (1, 2, 3, 4, 5, 10)


def motion(ms):
    """A joint command's targets ms into the take: every joint moving, the gripper closing."""
    s = (ms - 1000) / 1000
    return {1: -0.5 + 0.4 * s, 2: 0.3 + 0.1 * s, 3: -1.2 + 0.2 * s, 4: 0.1 * s, 10: min(1.0, 0.25 * s), 5: -0.05 + 0.02 * s}


def commands(refused=(), stale=(), parked=()):
    """Every 20 ms from 1.007 to 2.987 s (no frame within 30 us of one), each ``motion``; the outcome by its ms."""
    out = []
    for ms in range(1007, 3001, 20):
        if ms in stale:
            out.append((ms, "S", {m: 0.9 for m in MOTORS}))  # an old command arriving late: nothing like the motion
            continue
        outcome = "R" if any(a <= ms < b for a, b in refused) else "P" if ms in parked else "A"
        out.append((ms, outcome, motion(ms)))
    return out


def newest(cmds, t, outcomes=("A", "R")):
    """The command in force at t: an accepted one from its RO, 30 us after it arrived; a refused one from its arrival."""
    best = None
    for ms, outcome, values in cmds:
        at = ms * MS + (30_000 if outcome == "A" else 0)
        if outcome in outcomes and at <= t:
            best = (at, values)
    return best


def test_a_command_robothost_refused_is_still_the_action(scenarios):
    cmds = commands(refused=[(1500, 2200)])
    take = read_take(jetrover_take(scenarios, commands=cmds))
    inside = 0
    for f in take.frames:
        t = f.timing["sensed"]
        at, values = newest(cmds, t)
        assert f.action == pytest.approx([relay_pulse(m, values[m]) for m in MOTORS], abs=1e-6)
        assert f.timing["actionAge"] == t - at
        # The gripper's slot is the relay's: the gripper as RobotHost last told it, which a refused command never reached.
        _, accepted = newest(cmds, t, ("A",))
        assert f.state[5] == pytest.approx(relay_pulse(10, accepted[10]))
        inside += 1500 * MS < t < 2200 * MS
    assert inside >= 9  # 0.7 s at 15 fps
    refused = [f for f in take.frames if 1600 * MS < f.timing["sensed"] < 2200 * MS]
    assert len({tuple(f.action) for f in refused}) == len(refused)  # it moves on, as the operator's commands did
    assert all(f.state[5] != f.action[5] for f in refused)


def test_a_replay_has_its_sources_action_whatever_robothost_refused(scenarios):
    # The same commands, refused for 1.0 s in one take and for 0.2 s in the other, as in the pair above.
    source = read_take(jetrover_take(scenarios, name="20261124T021149Z-take013", commands=commands(refused=[(1400, 2400)])))
    replay = read_take(jetrover_take(scenarios, name="20261124T021216Z-take014", commands=commands(refused=[(1400, 1600)])))
    assert [f.timing["sensed"] for f in source.frames] == [f.timing["sensed"] for f in replay.frames]
    for a, b in zip(source.frames, replay.frames):
        assert a.action == pytest.approx(b.action, abs=1e-6)
    # The relay targets alone, the action until then, differ by tens of units for most of the source's refusals.
    held = [abs(a.state[5] - b.state[5]) for a, b in zip(source.frames, replay.frames)]
    assert max(held) > 20


def test_the_action_of_a_refused_command_is_the_plants_own_conversion(scenarios):
    # Pinned by hand (GenericArmPlantConfig.Default; core/RobotProfiles/jetrover.json): 238.732 pulses a radian from 500, the
    # shoulder's offset -0.64494798 rad added; the gripper 116 open, 645 closed; the range 0..1000.
    pinned = {1: 0.0, 2: 0.64494798, 3: 0.5, 4: 3.0, 5: -3.0, 10: 0.5}
    cmds = [(ms, "A", motion(ms)) for ms in range(1007, 2000, 20)] + [(2007, "R", pinned)]
    cmds += [(ms, "A", motion(ms)) for ms in range(2507, 3001, 20)]
    take = read_take(jetrover_take(scenarios, commands=cmds))
    after = [f for f in take.frames if 2008 * MS < f.timing["sensed"] < 2500 * MS]
    assert after and all(f.action == pytest.approx([500.0, 500.0, 619.3662, 1000.0, 0.0, 380.5], abs=1e-4) for f in after)


def test_a_stale_command_is_not_the_action(scenarios):
    cmds = commands(stale=(2007,))
    take = read_take(jetrover_take(scenarios, commands=cmds))
    assert all(f.action[0] != pytest.approx(relay_pulse(1, 0.9)) for f in take.frames)  # a newer command had replaced it
    for f in take.frames:
        _, values = newest(cmds, f.timing["sensed"])
        assert f.action == pytest.approx([relay_pulse(m, values[m]) for m in MOTORS], abs=1e-6)


def test_while_the_arm_parks_the_action_is_robothosts_park_not_the_commands_it_ignored(scenarios):
    # ROBOT OFF mid-take, then ROBOT ON (docs/adr/0023 decision 5): RobotHost parks the arm through the relay (RO of its own) and
    # records the operator's commands meanwhile as P. Were those the action too, it would alternate between the two every few ms.
    parked = tuple(range(1707, 2400, 20))
    cmds = commands(parked=parked)
    rest = {1: 495.0, 2: 870.0, 3: 180.0, 4: 30.0, 5: 520.0, 10: 116.0}
    park = [((1710 + 20 * k) * MS + 7_000, {m: rest[m] + 60 - k for m in MOTORS}) for k in range(0, 31)]
    extra = [["RO", t, ";".join(f"{m}:{v!r}:300" for m, v in values.items())] for t, values in park]
    take = read_take(jetrover_take(scenarios, commands=cmds, extra=extra))
    events = sorted([(ms * MS + 30_000, {m: relay_pulse(m, v[m]) for m in MOTORS}) for ms, o, v in cmds if o == "A"] + park,
                    key=lambda e: e[0])
    during = 0
    for f in take.frames:
        t = f.timing["sensed"]
        values = [v for at, v in events if at <= t][-1]
        assert f.action == pytest.approx([values[m] for m in MOTORS], abs=1e-6)
        during += 1720 * MS < t < 2400 * MS
    assert during >= 9
    actions = [e["jetson_mono_ns"] for e in raw_events(take) if e["type"] == "action"]
    assert actions == [at for at, _ in events]  # none at an ignored command's time


def test_an_episode_runs_to_the_last_command_robothost_refused(scenarios):
    # The episode spans the operator's commands that make the action: a take whose last commands RobotHost refused keeps its
    # last frames (until 2026-11-24 it ended at the last accepted command), so a replay spans what its source does.
    refused = read_take(jetrover_take(scenarios, commands=commands(refused=[(2400, 3100)])))
    accepted = read_take(jetrover_take(scenarios, name="20261112T100000Z-take002", commands=commands()))
    assert [f.timing["sensed"] for f in refused.frames] == [f.timing["sensed"] for f in accepted.frames]
    assert refused.frames[-1].timing["sensed"] > 2900 * MS
    for a, b in zip(refused.frames, accepted.frames):
        assert a.action == pytest.approx(b.action, abs=1e-6)


FR3_ORDER = list(range(1, 9)) + list(range(11, 19))  # each arm's seven joints, then its hand


def _fr3_middles():
    with open(os.path.join(REPO, "core", "RobotProfiles", "fr3-duo.json"), encoding="utf-8") as f:
        profile = json.load(f)
    return {j["motorId"]: (j["softLower"] + j["softUpper"]) / 2 for arm in profile["arms"] for j in arm["joints"]}


FR3_MIDDLES = _fr3_middles()


def fr3_motion(ms):
    """An FR3 Duo command's targets ms into the take: every joint moving inside its soft limits, both hands closing."""
    s = (ms - 1000) / 1000
    return {m: (0.3 + 0.5 * s if m in (8, 18) else FR3_MIDDLES[m] + 0.01 * m - 0.1 * s) for m in FR3_ORDER}


def test_an_fr3_command_robothost_refused_is_still_the_action_as_its_plant_would_send_it(scenarios):
    # The same rule for both robots. SerialChainPlant refuses a command whose pose would collide; its action is the command,
    # within the soft limits as the plant would have sent it, a hand's fraction as finger travel (0.04 m).
    cmds = [(ms, "R" if 1300 <= ms < 1600 else "A", fr3_motion(ms)) for ms in range(1002, 2000, 20)]
    cmds[20] = (cmds[20][0], "R", {**fr3_motion(cmds[20][0]), 1: 5.0, 8: 1.7})  # beyond joint 1's soft limit, the hand past 1
    take = read_take(fr3_take(scenarios, commands=cmds))
    inside = 0
    for f in take.frames:
        t = f.timing["sensed"]
        values = [v for ms, o, v in cmds if o in "AR" and ms * MS <= t][-1]
        expect = [values[m] * (0.04 if m in (8, 18) else 1.0) for m in FR3_ORDER]
        if values[1] == 5.0:
            expect[0], expect[7] = 2.9007400166666666, 0.04
        assert f.action == pytest.approx(expect, abs=1e-6)
        inside += 1300 * MS <= t < 1600 * MS
    assert inside >= 4
    actions = [e for e in raw_events(take) if e["type"] == "action"]
    assert [e["jetson_mono_ns"] for e in actions] == [ms * MS for ms, o, _ in cmds if o in "AR"]


def test_a_take_whose_relay_targets_are_not_its_commands_converted_is_refused(scenarios):
    # RobotHost's conversion changed (another gripper calibration, say): the action of the commands it did not pass on cannot be
    # built, and saying so beats guessing.
    off = lambda m, v: relay_pulse(m, v) + (1.0 if m == 3 else 0.0)
    path = jetrover_take(scenarios, commands=commands(refused=[(2000, 2100)]), relay=off)
    with pytest.raises(TakeError, match="conversion has changed"):
        read_take(path)
    # With every command passed on, nothing is converted: the action is the RO, as it always was.
    ok = read_take(jetrover_take(scenarios, name="20261112T100000Z-take002", commands=commands(), relay=off))
    assert ok.frames and all(f.action[2] == pytest.approx(off(3, newest(commands(), f.timing["sensed"])[1][3])) for f in ok.frames)


def test_a_take_with_refused_commands_and_no_profile_is_refused(scenarios):
    path = jetrover_take(scenarios, commands=commands(refused=[(2000, 2100)]))
    os.remove(os.path.join(path, "robot-profile.json"))
    finish(path)
    with pytest.raises(TakeError, match="robot-profile.json"):
        read_take(path)


def test_the_raw_events_actions_are_the_action_columns_commands(scenarios):
    cmds = commands(refused=[(2000, 2100)])
    take = read_take(jetrover_take(scenarios, commands=cmds))
    actions = [e for e in raw_events(take) if e["type"] == "action"]
    sent = [(ms * MS + (30_000 if o == "A" else 0), [relay_pulse(m, v[m]) for m in MOTORS]) for ms, o, v in cmds if o in "AR"]
    assert [e["jetson_mono_ns"] for e in actions] == [t for t, _ in sent]
    assert all(e["values"] == pytest.approx(v, abs=1e-6) for e, (_, v) in zip(actions, sent))
