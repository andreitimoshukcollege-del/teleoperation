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
