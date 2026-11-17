"""Safety datasets from the twin's takes, labelled by physics (docs/adr/0036 amendment 1): frame labels from WC contacts,
each category's check, and the lab's label files."""

import json
import math
import os

import pytest

from conftest import MS, jetrover_take
from teleop_lerobot import safety
from teleop_lerobot.takes import read_take
from teleop_lerobot.watch import complete_takes

UPRIGHT = (1.0, 0.0, 0.0, 0.0)
ON_ITS_SIDE = (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0)  # 90 degrees about x


def wc(t_ms, pairs=""):
    return ["WC", t_ms * MS + MS, t_ms * MS, pairs]


def wo(t_ms, poses):
    """poses: {object: (x, y, z, (qw, qx, qy, qz))}."""
    text = ";".join(f"{o}:{x}:{y}:{z}:{q[0]}:{q[1]}:{q[2]}:{q[3]}" for o, (x, y, z, q) in poses.items())
    return ["WO", t_ms * MS + MS, t_ms * MS, 1, text]


def take(root, category, extra, name="20261114T100000Z-take001"):
    return jetrover_take(root, name=name, scenario=category, extra=extra)


def verdict(path):
    t = read_take(path)
    stretches = safety.contacts(path)
    labels = safety.frame_labels(t, stretches)
    return t, labels, safety.check(t, labels, stretches, safety.object_poses(path))


def test_a_touch_labels_the_frames_it_spans_and_recovering_passes(scenarios):
    standing = {2: (0.24, -0.08, -0.15, UPRIGHT)}
    path = take(scenarios, "collide_and_recover", [wo(1000, standing), wc(1500, "0:5:2:1.5"), wc(1620), wo(2900, standing)])
    t, labels, v = verdict(path)
    assert v.ok, v.reason
    assert v.touched == (2,)
    unsafe = [i for i, x in enumerate(labels) if x]
    # Frames every 1/15 s from 1.0 s: the touch 1.50-1.62 s spans frames 7..9 (their periods reach +-33 ms).
    times = [t.frames[i].timing["sensed"] / 1e9 for i in unsafe]
    assert unsafe == list(range(unsafe[0], unsafe[-1] + 1)) and min(times) <= 1.5 + 0.034 and max(times) >= 1.62 - 0.034
    assert labels[-1] == 0


def test_a_touch_shorter_than_a_frame_still_marks_one(scenarios):
    # 20 ms, between two frame instants: the frame whose period holds it is unsafe.
    path = take(scenarios, "collide_and_recover", [wc(1510, "0:6:1:0.3"), wc(1530)])
    _, labels, v = verdict(path)
    assert sum(labels) >= 1 and v.ok


def test_recovering_that_never_lets_go_or_tips_its_object_fails(scenarios):
    path = take(scenarios, "collide_and_recover", [wc(1500, "0:5:2:1.5")], name="20261114T100000Z-take001")
    assert "no recovery" in verdict(path)[2].reason
    tipped = take(scenarios, "collide_and_recover", [wo(1000, {2: (0.24, -0.08, -0.15, UPRIGHT)}), wc(1500, "0:5:2:1.5"), wc(1600),
                                                     wo(2500, {2: (0.26, -0.08, -0.18, ON_ITS_SIDE)})], name="20261114T100100Z-take002")
    v = verdict(tipped)[2]
    assert not v.ok and "tipped object 2 by 90" in v.reason


def turned(q, angle):
    """``q`` turned ``angle`` about the vertical (world z)."""
    c, s = math.cos(angle / 2), math.sin(angle / 2)
    w, x, y, z = q
    return (c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)


def test_an_object_lying_on_its_side_that_a_touch_only_spins_has_not_tipped(scenarios):
    # docs/adr/0037 amendment 6: a layout lays some objects on a side, and a touch may spin one on the table. Its own z axis is
    # level then: a 60-degree spin about the vertical turns that axis 60 degrees, but tips the object not at all.
    lying = {2: (0.24, -0.08, -0.16, ON_ITS_SIDE)}
    spun = {2: (0.25, -0.07, -0.16, turned(ON_ITS_SIDE, math.radians(60)))}
    assert verdict(take(scenarios, "collide_and_recover", [wo(1000, lying), wc(1500, "0:5:2:1.5"), wc(1620), wo(2900, spun)]))[2].ok
    # Rolled from its side onto its end, it has tipped; and a standing object spun about the vertical has not.
    up = {2: (0.25, -0.07, -0.13, UPRIGHT)}
    v = verdict(take(scenarios, "collide_and_recover", [wo(1000, lying), wc(1500, "0:5:2:1.5"), wc(1620), wo(2900, up)],
                     name="20261114T100100Z-take002"))[2]
    assert not v.ok and "tipped object 2 by 90" in v.reason
    standing = {2: (0.24, -0.08, -0.15, UPRIGHT)}
    spun_up = {2: (0.24, -0.08, -0.15, turned(UPRIGHT, 1.0))}
    assert verdict(take(scenarios, "collide_and_recover", [wo(1000, standing), wc(1500, "0:5:2:1.5"), wc(1620), wo(2900, spun_up)],
                        name="20261114T100200Z-take003"))[2].ok
    assert safety._tilt_degrees((0, 0, 0, *ON_ITS_SIDE), (0, 0, 0, *turned(ON_ITS_SIDE, 1.0))) == pytest.approx(0.0, abs=1e-4)
    assert safety._tilt_degrees((0, 0, 0, *UPRIGHT), (0, 0, 0, *ON_ITS_SIDE)) == pytest.approx(90.0, abs=1e-6)


def test_a_full_collision_must_knock_its_object_over(scenarios):
    knocked = take(scenarios, "collide_with_obstacle", [wo(1000, {3: (0.17, -0.02, -0.16, UPRIGHT)}), wc(1400, "0:4:3:6.0"),
                                                        wc(1900), wo(2500, {3: (0.21, -0.05, -0.18, ON_ITS_SIDE)})])
    assert verdict(knocked)[2].ok
    grazed = take(scenarios, "collide_with_obstacle", [wo(1000, {3: (0.17, -0.02, -0.16, UPRIGHT)}), wc(1400, "0:4:3:0.5"),
                                                       wc(1450), wo(2500, {3: (0.171, -0.02, -0.16, UPRIGHT)})], name="20261114T100100Z-take002")
    v = verdict(grazed)[2]
    assert not v.ok and "tipped none" in v.reason


def test_no_contact_categories_fail_on_any_touch(scenarios):
    assert verdict(take(scenarios, "stop_before_collision", []))[2].ok
    assert verdict(take(scenarios, "reach_no_obstacle", [], name="20261114T100100Z-take002"))[2].ok
    v = verdict(take(scenarios, "stop_before_collision", [wc(2000, "0:6:4:0.2"), wc(2040)], name="20261114T100200Z-take003"))[2]
    assert not v.ok and "touched object(s) [4]" in v.reason
    assert not verdict(take(scenarios, "collide_with_obstacle", [], name="20261114T100300Z-take004"))[2].ok  # never touched


def test_the_dataset_take_has_his_front_camera_and_category_task(scenarios):
    t = safety.as_safety(read_take(take(scenarios, "collide_and_recover", [])))
    assert [c.name for c in t.cameras] == ["third-left", "front"]
    assert all("front" in f.parts and "wrist" not in f.parts for f in t.frames)
    assert t.task == "collide_and_recover" and t.scenario == "collide_and_recover"


def test_his_label_files(tmp_path):
    episodes = [(0, "reach_no_obstacle", [0, 0, 0]), (1, "collide_and_recover", [0, 1, 1, 0, 1, 0]),
                (2, "collide_with_obstacle", [0, 0, 1, 1])]
    assert safety.ranges([0, 1, 1, 0, 1, 0]) == [[1, 2], [4, 4]]
    assert safety.events(1, "collide_and_recover", [0, 1, 1, 0, 1, 0]) == [
        {"episode": 1, "kind": "contact", "frame_in_episode": 1, "source": "physics"},
        {"episode": 1, "kind": "clear", "frame_in_episode": 5, "source": "physics"}]
    assert [e["kind"] for e in safety.events(2, "collide_with_obstacle", [0, 0, 1, 1])] == ["contact"]  # no recovery: no clear
    rows = safety.label_rows(1, "collide_and_recover", [0, 1])
    assert rows[1] == {"episode_index": 1, "frame_index": 1, "label": 1, "category": "collide_and_recover",
                       "effective_category": "collide_and_recover", "overridden": False, "source": "physics"}
    pd = pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    safety.write_labels(str(tmp_path), episodes)
    # His manual JSON: string episode keys, inclusive ranges, all-safe episodes left out.
    assert json.loads((tmp_path / "manual_safety_labels.json").read_text()) == {"1": [[1, 2], [4, 4]], "2": [[2, 3]]}
    lines = [json.loads(x) for x in (tmp_path / "safety_events.jsonl").read_text().splitlines()]
    assert [(e["episode"], e["kind"]) for e in lines] == [(1, "contact"), (1, "clear"), (2, "contact")]
    df = pd.read_parquet(tmp_path / "meta" / "safety_labels.parquet")
    assert list(df.columns) == ["episode_index", "frame_index", "label", "category", "effective_category", "overridden", "source"]
    assert len(df) == 13 and int(df["label"].sum()) == 5


def test_a_failing_take_is_set_aside_with_its_reason(scenarios):
    path = take(scenarios, "stop_before_collision", [wc(2000, "0:6:4:0.2"), wc(2040)])
    v = verdict(path)[2]
    dest = safety.set_aside(path, v)
    assert dest == os.path.join(scenarios, "_discarded", "stop_before_collision", os.path.basename(path))
    assert not os.path.exists(path)
    assert json.load(open(os.path.join(dest, "safety-check.json")))["ok"] is False


def test_the_watcher_never_rebuilds_a_safety_dataset(scenarios, tmp_path):
    # 2026-11-15: the watcher took jetrover-isaac-safety/<batch> for one of its datasets, saw every take in its ledger as set
    # aside (it skips the safety scenarios), and "rebuilt" it empty, deleting it.
    from teleop_lerobot.watch import Converter

    path = take(scenarios, "collide_with_obstacle", [])
    out = tmp_path / "lerobot"
    batch = out / "jetrover-isaac-safety" / "jetrover_20261115_020534_abcdef"
    (batch / "meta" / "teleop").mkdir(parents=True)
    (batch / "meta" / "info.json").write_text(json.dumps({"total_episodes": 1}))
    (batch / "meta" / "teleop" / "takes.jsonl").write_text(json.dumps({"episode": 0, "take": path, "filesSha256": "x"}) + "\n")

    class NoWriter:
        def __getattr__(self, name):
            raise AssertionError(f"the watcher wrote ({name}) though it had nothing of its own to write")

    conv = Converter([scenarios], str(out), NoWriter(), settle=0, log=lambda m: None)
    assert str(batch) not in conv.known_datasets()
    conv.pass_once()
    assert (batch / "meta" / "teleop" / "takes.jsonl").exists()


def test_the_video_encoding_override_reaches_lerobots_encoder_and_is_put_back(monkeypatch):
    # LeRobot 0.5.1 takes no CRF from LeRobotDataset (2026-11-15): the writer hands its encoder one while an episode is saved.
    import sys
    import types

    from teleop_lerobot.writer import video_encoding

    calls = []

    def encode_video_frames(imgs_dir, video_path, fps, vcodec="libsvtav1", crf=30, preset=None, **rest):
        calls.append((crf, preset))

    fake = types.ModuleType("lerobot.datasets.dataset_writer")
    fake.encode_video_frames = encode_video_frames
    package = types.ModuleType("lerobot.datasets")
    package.dataset_writer = fake
    monkeypatch.setitem(sys.modules, "lerobot", types.ModuleType("lerobot"))
    monkeypatch.setitem(sys.modules, "lerobot.datasets", package)
    monkeypatch.setitem(sys.modules, "lerobot.datasets.dataset_writer", fake)

    with video_encoding(18, 8):
        fake.encode_video_frames("imgs", "out.mp4", 15, vcodec="libsvtav1")
    fake.encode_video_frames("imgs", "out.mp4", 15)
    with video_encoding():  # nothing asked: LeRobot's own
        fake.encode_video_frames("imgs", "out.mp4", 15)
    assert calls == [(18, 8), (30, None), (30, None)]
    assert fake.encode_video_frames is encode_video_frames

    del fake.encode_video_frames
    with pytest.raises(RuntimeError):
        with video_encoding(18, 8):
            pass


def test_the_watcher_leaves_the_safety_scenarios_alone(scenarios):
    take(scenarios, "collide_with_obstacle", [])
    jetrover_take(scenarios, scenario="free-play")
    found = complete_takes([scenarios], now=1e12, settle=0)
    assert [os.path.basename(os.path.dirname(t)) for t in found] == ["free-play"]


def test_the_knock_report_says_what_each_touched_object_weighed_and_whether_it_went_over(scenarios):
    # docs/adr/0037 P4: what can be knocked over, by mass at size, from the takes themselves.
    path = take(scenarios, "collide_with_obstacle", [wo(1000, {3: (0.17, -0.02, -0.16, UPRIGHT), 4: (0.25, 0.0, -0.17, UPRIGHT)}),
                                                     wc(1400, "0:4:3:6.0;0:4:4:1.0"), wc(1900),
                                                     wo(2500, {3: (0.21, -0.05, -0.18, ON_ITS_SIDE), 4: (0.251, 0.0, -0.17, UPRIGHT)})])
    with open(os.path.join(path, "scene.json"), "w") as f:
        json.dump({"objects": [{"id": 3, "name": "can", "kind": "movable", "shape": "cylinder", "mass": 0.04},
                               {"id": 4, "name": "wall", "kind": "fixed", "shape": "box", "mass": 0}]}, f)
    rows = {r["object"]: r for r in safety.knocks(path)}
    assert rows[3]["knocked"] and rows[3]["tiltDeg"] == pytest.approx(90.0, abs=0.1) and rows[3]["massKg"] == 0.04
    assert rows[3]["scale"] == 1.0  # no SZ in this take's isaac.jlog: the file's size
    assert not rows[4]["knocked"] and rows[4]["movedCm"] == pytest.approx(0.1, abs=0.01)
    summary = safety.knock_summary(list(rows.values()))
    assert any("1/1 knocked" in line and "can" in line for line in summary)
    assert any(line.startswith("fixed: 0/1") for line in summary)


def test_the_sizes_come_from_the_renderers_first_sz_record():
    # The record lists hold the fields after the tag: SZ|58|1:1.5625;2:1.0 is ["58", "1:1.5625;2:1.0"].
    assert safety.sizes([["58", "1:1.5625;2:1.0;31:1.0"], ["59", "1:0.8"]]) == {1: 1.5625, 2: 1.0, 31: 1.0}
    assert safety.sizes([]) == {} and safety.sizes([["58"]]) == {}
