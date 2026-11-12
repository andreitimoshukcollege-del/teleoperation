"""A take into its episode frames (docs/adr/0036 decisions 2-4), without LeRobot."""

import os

import pytest

from conftest import MS, finish, fr3_take, jetrover_take
from teleop_lerobot.takes import TakeError, read_take, verify_checksums


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
