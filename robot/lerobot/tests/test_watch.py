"""The watcher (docs/adr/0036 decision 6), with a stand-in for LeRobot that keeps LeRobot's episode count."""

import json
import os
import shutil
import time

from conftest import fr3_take, jetrover_take
from teleop_lerobot.watch import Converter
from teleop_lerobot.writer import read_ledger


class FakeWriter:
    def __init__(self):
        self.appended = []

    def append(self, root, repo, take):
        os.makedirs(os.path.join(root, "meta"), exist_ok=True)
        info = os.path.join(root, "meta", "info.json")
        n = json.load(open(info))["total_episodes"] if os.path.exists(info) else 0
        json.dump({"total_episodes": n + 1, "repo": repo}, open(info, "w"))
        self.appended.append((root, os.path.basename(take.path), len(take.frames)))
        return n


def converter(scenarios, out, later=100.0):
    clock = {"now": time.time() + later}
    conv = Converter([scenarios], out, FakeWriter(), settle=60.0, log=lambda s: None, clock=lambda: clock["now"])
    return conv, clock


def test_a_take_is_converted_once_after_it_has_settled(scenarios, tmp_path):
    jetrover_take(scenarios)
    out = str(tmp_path / "lerobot")
    conv, clock = converter(scenarios, out, later=0.0)
    assert conv.pass_once() == 0  # just completed: a DISCARD LAST may still come
    clock["now"] += 61
    assert conv.pass_once() == 1
    assert conv.pass_once() == 0
    root = os.path.join(out, "jetrover-isaac", "reach")
    ledger = read_ledger(root)
    assert [e["episode"] for e in ledger] == [0] and ledger[0]["frames"] == 31 and ledger[0]["task"] == "reach the red cube"
    timing = open(os.path.join(root, "meta", "teleop", "episode_000000.timing.jsonl")).read().splitlines()
    assert len(timing) == 31 and json.loads(timing[0])["frameIndex"] == 0


def test_scenarios_and_robots_get_datasets_of_their_own_and_takes_append_in_order(scenarios, tmp_path):
    jetrover_take(scenarios, name="20261112T100500Z-take002")
    jetrover_take(scenarios, name="20261112T100000Z-take001")
    fr3_take(scenarios)
    out = str(tmp_path / "lerobot")
    conv, _ = converter(scenarios, out)
    assert conv.pass_once() == 3
    assert [(os.path.relpath(r, out), t) for r, t, _ in conv.writer.appended] == [
        (os.path.join("fr3_duo-isaac", "handover"), "20261112T110000Z-take001"),
        (os.path.join("jetrover-isaac", "reach"), "20261112T100000Z-take001"),
        (os.path.join("jetrover-isaac", "reach"), "20261112T100500Z-take002")]


def test_a_take_set_aside_after_conversion_rebuilds_its_dataset_without_it(scenarios, tmp_path):
    jetrover_take(scenarios, name="20261112T100000Z-take001")
    second = jetrover_take(scenarios, name="20261112T100500Z-take002")
    out = str(tmp_path / "lerobot")
    conv, _ = converter(scenarios, out)
    conv.pass_once()
    discarded = os.path.join(scenarios, "_discarded", "reach")
    os.makedirs(discarded)
    shutil.move(second, discarded)
    conv.writer.appended.clear()
    assert conv.pass_once() == 1
    root = os.path.join(out, "jetrover-isaac", "reach")
    assert [os.path.basename(e["take"]) for e in read_ledger(root)] == ["20261112T100000Z-take001"]
    assert json.load(open(os.path.join(root, "meta", "info.json")))["total_episodes"] == 1
    assert not [n for n in os.listdir(os.path.dirname(root)) if ".old-" in n or ".rebuild-" in n]


def test_a_dataset_whose_write_died_is_rebuilt(scenarios, tmp_path):
    jetrover_take(scenarios)
    out = str(tmp_path / "lerobot")
    conv, _ = converter(scenarios, out)
    conv.pass_once()
    info = os.path.join(out, "jetrover-isaac", "reach", "meta", "info.json")
    json.dump({"total_episodes": 2}, open(info, "w"))  # LeRobot saved an episode the ledger never got
    assert conv.pass_once() == 1
    assert json.load(open(info))["total_episodes"] == 1


def test_a_bad_take_is_skipped_once_with_its_reason(scenarios, tmp_path):
    path = jetrover_take(scenarios)
    with open(os.path.join(path, "robothost.jlog"), "ab") as f:
        f.write(b"x")
    out = str(tmp_path / "lerobot")
    conv, _ = converter(scenarios, out)
    assert conv.pass_once() == 0 and conv.pass_once() == 0
    skipped = open(os.path.join(out, "skipped.jsonl")).read().splitlines()
    assert len(skipped) == 1 and "does not match" in json.loads(skipped[0])["reason"]
    # A restarted watcher remembers it.
    again, _ = converter(scenarios, out)
    assert again.pass_once() == 0 and len(open(os.path.join(out, "skipped.jsonl")).read().splitlines()) == 1


def test_discarded_and_open_takes_are_never_candidates(scenarios, tmp_path):
    jetrover_take(os.path.join(scenarios, "_discarded"), scenario="reach")
    open_take = jetrover_take(scenarios, name="20261112T120000Z-take003")
    os.remove(os.path.join(open_take, "files.sha256"))
    conv, _ = converter(scenarios, str(tmp_path / "lerobot"))
    assert conv.pass_once() == 0


LAB_TIMING_FIELDS = {"episode_saved_so_far", "frame_in_episode", "pc_sample_mono_ns", "jetson_state_wall_ns",
                     "jetson_action_wall_ns", "pc_state_recv_mono_ns", "pc_action_recv_mono_ns", "pc_camera_read_mono_ns"}


def test_the_lab_recorders_two_files_sit_beside_the_dataset_in_its_format(scenarios, tmp_path):
    jetrover_take(scenarios, name="20261112T100000Z-take001")
    jetrover_take(scenarios, name="20261112T100500Z-take002")
    out = str(tmp_path / "lerobot")
    conv, _ = converter(scenarios, out)
    conv.pass_once()
    root = os.path.join(out, "jetrover-isaac", "reach")

    timing = [json.loads(l) for l in open(os.path.join(root, "capture_timing.jsonl"))]
    assert {frozenset(t) for t in timing} == {frozenset(LAB_TIMING_FIELDS)}  # record_jetrover.py's fields, no more
    assert [t["episode_saved_so_far"] for t in timing] == [0] * 31 + [1] * 31
    assert [t["frame_in_episode"] for t in timing[:3]] == [0, 1, 2]
    first = timing[0]
    assert first["jetson_state_wall_ns"] is None and first["jetson_action_wall_ns"] is None
    assert first["pc_state_recv_mono_ns"] <= first["pc_camera_read_mono_ns"] and first["pc_action_recv_mono_ns"] <= first["pc_camera_read_mono_ns"]

    events = [json.loads(l) for l in open(os.path.join(root, "raw_ros_events.jsonl"))]
    readies = [e for e in events if e["type"] == "ready"]
    assert len(readies) == 2 and readies[0]["servo_ids"] == [1, 2, 3, 4, 5, 10]  # one per take, as the bridge says ready
    actions = [e for e in events if e["type"] == "action"]
    states = [e for e in events if e["type"] == "state"]
    assert {frozenset(e) for e in actions + states} == {frozenset({"type", "jetson_wall_ns", "jetson_mono_ns", "servo_ids", "values", "pc_received_mono_ns"})}
    assert actions[0]["servo_ids"] == [1, 2, 3, 4, 5, 10] and actions[0]["values"] == [501.5, 502.5, 503.5, 504.5, 505.5, 510.5]
    assert states[0]["servo_ids"] == [1, 2, 3, 4, 5] and states[0]["values"] == [401.0, 402.0, 403.0, 404.0, 405.0]
    # The bridge emits a state only when every arm servo read: the one RI with motor 3 invalid is not a state.
    assert len(states) == 2 * 115 and len(actions) == 2 * 116
    times = [e["jetson_mono_ns"] for e in events[1:117 + 115]]
    assert times == sorted(times)


def test_an_fr3_dataset_gets_them_too_with_its_sixteen_motors(scenarios, tmp_path):
    fr3_take(scenarios)
    out = str(tmp_path / "lerobot")
    conv, _ = converter(scenarios, out)
    conv.pass_once()
    root = os.path.join(out, "fr3_duo-isaac", "handover")
    events = [json.loads(l) for l in open(os.path.join(root, "raw_ros_events.jsonl"))]
    state = next(e for e in events if e["type"] == "state")
    assert state["servo_ids"] == list(range(1, 9)) + list(range(11, 19))
    assert state["values"][7] == 0.02  # the hand's 0.5 opening as finger travel, as in observation.state
    frames = read_ledger(root)[0]["frames"]
    assert len(open(os.path.join(root, "capture_timing.jsonl")).readlines()) == frames
