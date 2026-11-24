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


def take(root, category, extra, name="20261114T100000Z-take001", at_s=0.0):
    return jetrover_take(root, name=name, scenario=category, extra=extra, at_s=at_s)


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


def test_the_dataset_take_has_his_front_camera_and_a_neutral_task_or_its_category(scenarios):
    t = safety.as_safety(read_take(take(scenarios, "collide_and_recover", [])))
    assert [c.name for c in t.cameras] == ["third-left", "front"]
    assert all("front" in f.parts and "wrist" not in f.parts for f in t.frames)
    # docs/adr/0038 decision 6: the caption no longer names the scenario; the old one on request (rebuild comparisons).
    assert t.task == "move the arm" and t.scenario == "collide_and_recover"
    old = safety.as_safety(read_take(take(scenarios, "collide_and_recover", [], name="20261114T100100Z-take002")), caption="category")
    assert old.task == "collide_and_recover"


def test_his_label_files(tmp_path):
    episodes = [(0, "reach_no_obstacle", [0, 0, 0]), (1, "collide_and_recover", [0, 1, 1, 0, 1, 0]),
                (2, "collide_with_obstacle", [0, 0, 1, 1])]
    assert safety.ranges([0, 1, 1, 0, 1, 0]) == [[1, 2], [4, 4]]
    # A pair a range (docs/adr/0036 amendment 5: several events a take), so a lead window never spans the free movement between.
    assert safety.events(1, "collide_and_recover", [0, 1, 1, 0, 1, 0]) == [
        {"episode": 1, "kind": "contact", "frame_in_episode": 1, "source": "physics"},
        {"episode": 1, "kind": "clear", "frame_in_episode": 3, "source": "physics"},
        {"episode": 1, "kind": "contact", "frame_in_episode": 4, "source": "physics"},
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
    assert [(e["episode"], e["kind"]) for e in lines] == [(1, "contact"), (1, "clear"), (1, "contact"), (1, "clear"), (2, "contact")]
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
    take(scenarios, "mixed", [], name="20261122T100000Z-take001")  # docs/adr/0038's script: a batch's dataset too
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


def se(t_ms, index, drawn, kind, obj, final=0.5, contact=0.7, end=0.9, clear=1.1):
    """A planned event (SE), its times in seconds after its approach began at ``t_ms``."""
    return ["SE", t_ms * MS, index, drawn, kind, obj, final, contact, end, clear]


def test_several_touches_each_backed_off_pass_and_each_is_its_own_unsafe_range(scenarios, tmp_path):
    # docs/adr/0036 amendment 5: three touches in one take, on three objects, each ending before the next.
    up = {2: (0.24, -0.08, -0.15, UPRIGHT), 3: (0.17, -0.02, -0.16, UPRIGHT), 4: (0.29, 0.03, -0.17, UPRIGHT)}
    rows = [wo(1000, up), wc(1200, "0:5:2:1.5"), wc(1300), wc(1800, "0:6:3:1.0"), wc(1900), wc(2400, "0:5:4:2.0"), wc(2500), wo(2900, up)]
    t, labels, v = verdict(take(scenarios, "collide_and_recover", rows))
    assert v.ok, v.reason
    assert v.touched == (2, 3, 4) and v.contacts == 3 and "3 touches" in v.reason
    assert len(safety.ranges(labels)) == 3  # several unsafe ranges an episode, exact
    assert all(r[1] - r[0] <= 4 for r in safety.ranges(labels))
    episodes = [(0, "collide_and_recover", labels)]
    assert [e["kind"] for e in safety.events(0, "collide_and_recover", labels)] == ["contact", "clear"] * 3
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    safety.write_labels(str(tmp_path), episodes)
    assert json.loads((tmp_path / "manual_safety_labels.json").read_text()) == {"0": safety.ranges(labels)}


def test_several_touches_fail_if_any_does_not_back_off_or_any_object_tips(scenarios):
    up = {2: (0.24, -0.08, -0.15, UPRIGHT), 3: (0.17, -0.02, -0.16, UPRIGHT)}
    held = take(scenarios, "collide_and_recover", [wo(1000, up), wc(1200, "0:5:2:1.5"), wc(1300), wc(2400, "0:5:3:2.0")])
    assert "no recovery" in verdict(held)[2].reason  # the last touch never ended
    tipped = {2: (0.24, -0.08, -0.15, UPRIGHT), 3: (0.19, -0.02, -0.18, ON_ITS_SIDE)}
    v = verdict(take(scenarios, "collide_and_recover", [wo(1000, up), wc(1200, "0:5:2:1.5"), wc(1300), wc(1800, "0:6:3:1.0"), wc(1900),
                                                        wo(2900, tipped)], name="20261114T100100Z-take002"))[2]
    assert not v.ok and "tipped object 3 by 90" in v.reason


def test_several_knocks_pass_with_one_object_over_and_fail_with_none(scenarios):
    before = {3: (0.17, -0.02, -0.16, UPRIGHT), 4: (0.29, 0.03, -0.17, UPRIGHT), 5: (0.22, 0.10, -0.17, UPRIGHT)}
    rows = [wo(1000, before), wc(1200, "0:4:3:0.6"), wc(1250), wc(1800, "0:4:4:6.0"), wc(1950), wc(2400, "0:4:5:0.4"), wc(2450)]
    after = {3: (0.171, -0.02, -0.16, UPRIGHT), 4: (0.33, 0.06, -0.18, ON_ITS_SIDE), 5: (0.221, 0.10, -0.17, UPRIGHT)}
    v = verdict(take(scenarios, "collide_with_obstacle", rows + [wo(2900, after)]))[2]
    assert v.ok and v.contacts == 3 and "knocked [4]" in v.reason
    grazed = {3: (0.171, -0.02, -0.16, UPRIGHT), 4: (0.291, 0.03, -0.17, UPRIGHT), 5: (0.221, 0.10, -0.17, UPRIGHT)}
    v = verdict(take(scenarios, "collide_with_obstacle", rows + [wo(2900, grazed)], name="20261114T100100Z-take002"))[2]
    assert not v.ok and "tipped none" in v.reason


def test_each_touched_object_is_measured_from_its_own_first_touch(scenarios):
    # The first knock sent object 3 into object 4, shoving it 4 cm before the arm ever touched it; the second knock then only
    # grazed 4. Measured from its own first touch, 4 was not knocked by the arm (3 was, so the take passes on 3 alone).
    before = {3: (0.17, -0.02, -0.16, UPRIGHT), 4: (0.29, 0.03, -0.17, UPRIGHT)}
    shoved = {3: (0.24, 0.00, -0.18, ON_ITS_SIDE), 4: (0.33, 0.03, -0.17, UPRIGHT)}
    rows = [wo(1000, before), wc(1200, "0:4:3:6.0"), wc(1300), wo(1500, shoved), wc(2400, "0:4:4:0.4"), wc(2450), wo(2900, shoved)]
    path = take(scenarios, "collide_with_obstacle", rows)
    v = verdict(path)[2]
    assert v.ok and "knocked [3]" in v.reason
    first = safety.first_touches(safety.contacts(path))
    assert first == {3: 1200 * MS, 4: 2400 * MS}


def test_the_planned_events_and_where_physics_put_them_come_from_the_take(scenarios):
    # docs/adr/0036 amendment 5: joint-drive sends each event as its approach begins; RobotHost records it as SE.
    rows = [se(1100, 1, 3, "touch", 2, contact=0.15), wc(1260, "0:5:2:1.5"), wc(1330),
            se(1700, 2, 3, "touch", 3, contact=0.2), wc(1910, "0:6:3:1.0"), wc(1990),
            se(2300, 3, 3, "touch", 4, contact=0.25)]
    path = take(scenarios, "collide_and_recover", rows)
    planned = safety.planned_events(path)
    assert [(e["index"], e["drawn"], e["kind"], e["object"]) for e in planned] == [(1, 3, "touch", 2), (2, 3, "touch", 3), (3, 3, "touch", 4)]
    assert planned[0]["approach"] == 1100 * MS and planned[0]["contact"] == 1250 * MS and planned[0]["clear"] == 2200 * MS
    t = read_take(path)
    timing = safety.event_timing(t, safety.contacts(path), planned)
    start = t.frames[0].timing["sensed"] / 1e9
    assert timing[0]["planned"] == pytest.approx(1.25 - start, abs=1e-3) and timing[0]["contact"] == pytest.approx(1.26 - start, abs=1e-3)
    assert timing[1]["touched"] == [3] and timing[1]["contact"] == pytest.approx(1.91 - start, abs=1e-3)
    assert timing[2]["contact"] is None and timing[2]["touched"] == []  # the third never touched
    assert safety.planned_events(take(scenarios, "collide_and_recover", [], name="20261114T100100Z-take002")) == []  # an older take
    stop = take(scenarios, "stop_before_collision", [se(1500, 1, 1, "stop", 4, contact=float("nan"))], name="20261114T100200Z-take003")
    assert safety.planned_events(stop)[0]["contact"] is None


class RootWriter:
    """A LeRobot stand-in that numbers each dataset's episodes from 0, as LeRobot does (its meta/info.json)."""

    def __init__(self):
        self.episodes = []

    def append(self, root, repo, t):
        n = sum(1 for r, _, _ in self.episodes if r == root)
        self.episodes.append((root, repo, t.path))
        os.makedirs(os.path.join(root, "meta"), exist_ok=True)
        with open(os.path.join(root, "meta", "info.json"), "w") as f:
            json.dump({"total_episodes": n + 1}, f)
        return n


def test_two_twins_takes_become_a_dataset_each_and_never_one(tmp_path):
    # docs/adr/0037 amendment 7 built the two twins' kept takes, in the order they were recorded, as one dataset. The twins
    # record at once on the GPU machine's one CLOCK_MONOTONIC, so their takes overlap on it, and a reader that picks
    # raw_ros_events.jsonl's commands by time took a third of its checkpoints from the other twin's arm (world-models' review
    # of jetrover_20261124_021454_cb1e21, docs/adr/0038 L). One dataset of both is refused; each twin's is its own.
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    first, second = tmp_path / "jetrover-isaac", tmp_path / "jetrover-isaac-twin1"
    first.mkdir(), second.mkdir()
    a = take(str(first), "collide_and_recover", [wc(1500, "0:5:2:1.5"), wc(1620)], name="20261121T100000Z-take001")
    b = take(str(second), "reach_no_obstacle", [], name="20261121T100001Z-take001", at_s=1)  # while a records
    c = take(str(first), "stop_before_collision", [], name="20261121T100100Z-take002", at_s=60)
    takes = [read_take(p) for p in (a, b, c)]
    w = RootWriter()
    out = str(tmp_path / "out" / "jetrover_20261121_100000_abcdef")
    with pytest.raises(safety.OverlapError) as refused:
        safety.build(w, out, takes, log=lambda m: None)
    message = str(refused.value)
    assert message.startswith(f"{out}: 1 pair(s) of the 3 takes overlap in time on the clock raw_ros_events.jsonl and ")
    assert ("One dataset holds one collection stream (docs/adr/0038 L): they come from 2 takes roots, and safety-build "
            "(build_streams) writes each root's takes, each twin's, as a dataset of its own.") in message
    # A take's span is every time the two lab files hold for it: here its first command (RO, 0.9 s) to its last reading
    # (RI, 3.205 s), around its frames (1.0 to 3.0 s); seconds from the earliest.
    from teleop_lerobot.writer import clock_span

    assert [clock_span(t) for t in takes] == [(900 * MS, 3205 * MS), (1900 * MS, 4205 * MS), (60900 * MS, 63205 * MS)]
    assert f"episode 0 ({a}) (0.00-2.31 s) and episode 1 ({b}) (1.00-3.31 s) overlap by 1.30 s" in message
    assert "episode 2" not in message  # c, a minute later, overlaps nothing
    assert w.episodes == [] and not os.path.exists(out)  # refused before anything was written
    with pytest.raises(safety.OverlapError):
        safety.build_split(w, out, takes, 2, log=lambda m: None)
    assert not any(os.path.exists(r) for r in safety.split_roots(out, 2))

    logged = []
    roots = safety.build_streams(w, out, takes, log=logged.append)
    assert roots == [out + "_twin0", out + "_twin1"]
    assert [(os.path.basename(r), p) for r, _, p in w.episodes] == [(os.path.basename(out) + "_twin0", a),
                                                                     (os.path.basename(out) + "_twin0", c),
                                                                     (os.path.basename(out) + "_twin1", b)]
    ledgers = [[(e["episode"], e["take"]) for e in map(json.loads, open(os.path.join(r, "meta", "teleop", "takes.jsonl")))]
               for r in roots]
    assert ledgers == [[(0, a), (1, c)], [(0, b)]]  # each dataset numbers its own episodes, in the order recorded
    assert json.load(open(os.path.join(roots[0], "manual_safety_labels.json"))).keys() == {"0"}
    assert json.load(open(os.path.join(roots[1], "manual_safety_labels.json"))) == {}
    assert f"[stream] {os.path.basename(out)}_twin0: 2 take(s) from {os.path.realpath(first)}" in logged
    assert f"[stream] {os.path.basename(out)}_twin1: 1 take(s) from {os.path.realpath(second)}" in logged
    # Every raw command and reading names its episode, so nothing need place it by time (docs/adr/0038 L).
    raw = [json.loads(x) for x in open(os.path.join(roots[0], "raw_ros_events.jsonl"))]
    assert sorted({e["episode_index"] for e in raw}) == [0, 1] and [e["episode_index"] for e in raw] == sorted(e["episode_index"] for e in raw)
    from teleop_lerobot.writer import episode_spans

    spans, unplaced = episode_spans(roots[0])
    assert unplaced == [] and sorted(spans) == [0, 1] and spans[0][1] < spans[1][0]


def test_overlapping_spans_are_every_pair_that_shares_a_moment():
    from teleop_lerobot.writer import overlap_lines, overlapping

    spans = [(0, 10), (10, 20), (21, 30), (5, 6), (31, 31), (25, 40)]
    assert overlapping(spans) == [(0, 1), (0, 3), (2, 5), (4, 5)]  # ends included: 10 is in both of the first two
    assert overlapping([(0, 1)]) == [] and overlapping([]) == []
    assert overlapping([(5, 9), (5, 9)]) == [(0, 1)]  # a take given twice
    lines = overlap_lines([f"e{i}" for i in range(6)], [(s * 10 ** 9, e * 10 ** 9) for s, e in spans], overlapping(spans), listed=2)
    assert lines == ["e0 (0.00-10.00 s) and e1 (10.00-20.00 s) overlap by 0.00 s",
                     "e0 (0.00-10.00 s) and e3 (5.00-6.00 s) overlap by 1.00 s", "and 2 more overlapping pair(s)"]


def test_a_stream_is_the_takes_root_and_the_twins_roots_are_named_by_twin(tmp_path):
    assert safety.stream_of(str(tmp_path / "jetrover-isaac-twin1" / "mixed" / "20261124T020651Z-take001")) == \
        os.path.realpath(str(tmp_path / "jetrover-isaac-twin1"))
    twins = ["/d/takes/jetrover-isaac", "/d/takes/jetrover-isaac-twin1"]
    assert safety.stream_names(twins) == {twins[0]: "twin0", twins[1]: "twin1"}
    assert safety.stream_names(["/d/takes/jetrover-isaac-twin1"]) == {"/d/takes/jetrover-isaac-twin1": "twin1"}
    assert safety.stream_names(["/d/takes/jetrover-isaac", "/d/takes/jetrover-real"]) == {
        "/d/takes/jetrover-isaac": "jetrover-isaac", "/d/takes/jetrover-real": "jetrover-real"}  # no twins: folder names
    with pytest.raises(ValueError, match="would both be named 'jetrover-isaac'"):
        safety.stream_names(["/a/jetrover-isaac", "/b/jetrover-isaac"])
    assert safety.stream_roots("/o/x", ["twin0"]) == ["/o/x"]  # one stream: the name asked for, as before
    assert safety.stream_roots("/o/x", ["twin0", "twin1"], 2) == ["/o/x_twin0_1of2", "/o/x_twin0_2of2", "/o/x_twin1_1of2",
                                                                  "/o/x_twin1_2of2"]


def test_the_build_command_refuses_takes_that_overlap_and_writes_nothing(scenarios, tmp_path, capsys, monkeypatch):
    # One takes root whose takes overlap in time (a twin's clock gone wrong, or one take given twice): exit 3, every pair named.
    from teleop_lerobot import writer
    from teleop_lerobot.__main__ import main

    class NoWriter:
        def __init__(self, *args, **kwargs):
            pass

        def append(self, *args, **kwargs):
            raise AssertionError("a refused build wrote an episode")
    monkeypatch.setattr(writer, "LeRobotWriter", NoWriter)
    a = take(scenarios, "reach_no_obstacle", [], name="20261124T100000Z-take001")
    b = take(scenarios, "reach_no_obstacle", [], name="20261124T100002Z-take002", at_s=2)
    out = str(tmp_path / "ds")
    assert main(["safety-build", "--jobs", "1", "--geometry-dir", "", "--out", out, a, b]) == 3
    text = capsys.readouterr().out
    assert f"[refused] {out}: 1 pair(s) of the 2 takes overlap in time" in text
    assert "they come from one takes root, whose takes overlap only when one is given twice or two recorders wrote into it." in text
    assert f"[refused]   episode 0 ({a}) (0.00-2.31 s) and episode 1 ({b}) (2.00-4.30 s) overlap by 0.30 s" in text
    assert not os.path.exists(out)
    assert main(["safety-build", "--jobs", "1", "--geometry-dir", "", "--split", "2", "--out", out, a, a]) == 3
    assert "take 0 (" in capsys.readouterr().out and not os.path.exists(out + "_1of2")


class PreparingWriter:
    """A writer with workers' half (docs/adr/0036 amendment 6): its job 'prepares' an episode into the folder it is given,
    the later ones faster than the earlier, so they finish out of order; ``append`` records what it was handed."""

    def __init__(self, fail_at=None, total=3):
        import threading

        self.episodes, self.fail_at, self.total = [], fail_at, total
        self.lock, self.running, self.most_running, self.started = threading.Lock(), 0, 0, []

    def prepare_job(self):
        def job(t, episode, folder):
            import time

            from teleop_lerobot.writer import Prepared

            with self.lock:
                self.started.append(episode)
                self.running += 1
                self.most_running = max(self.most_running, self.running)
            try:
                os.makedirs(folder)
                with open(os.path.join(folder, "frame.png"), "w") as f:
                    f.write(t.path)
                time.sleep(0.02 * (self.total - episode))
                if episode == self.fail_at:
                    raise RuntimeError(f"episode {episode} could not be prepared")
                return Prepared(episode, folder, {"observation.images.front": [os.path.join(folder, "frame.png")]}, {})
            finally:
                with self.lock:
                    self.running -= 1
        return job

    def append(self, root, repo, t, prepared=None):
        assert prepared is not None and os.path.exists(prepared.images["observation.images.front"][0])
        with open(prepared.images["observation.images.front"][0]) as f:
            assert f.read() == t.path  # its own take's
        self.episodes.append((prepared.episode, t.path, [c.name for c in t.cameras]))
        return len(self.episodes) - 1


def test_a_parallel_build_appends_what_the_workers_prepared_in_the_takes_order(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    from concurrent.futures import ThreadPoolExecutor

    paths = [take(scenarios, c, [wc(1500, "0:5:2:1.5"), wc(1620)] if c == "collide_and_recover" else [], name=f"20261121T1000{i}0Z-take00{i}",
                  at_s=10 * i)
             for i, c in enumerate(("collide_and_recover", "reach_no_obstacle", "stop_before_collision", "collide_and_recover"))]
    takes = [read_take(p) for p in paths]
    w = PreparingWriter(total=4)
    out = str(tmp_path / "out" / "jetrover_20261121_100000_abcdef")
    with ThreadPoolExecutor(4) as pool:
        written = safety.build(w, out, takes, log=lambda m: None, jobs=2, executor=pool)
    assert [(ep, p) for ep, p, _ in w.episodes] == [(0, paths[0]), (1, paths[1]), (2, paths[2]), (3, paths[3])]
    assert all(cams == ["third-left", "front"] for _, _, cams in w.episodes)  # the safety take: wrist named front
    assert w.most_running >= 2  # prepared side by side
    assert not os.path.exists(out + ".prepare")  # every prepared file gone

    class Sequential:
        def __init__(self):
            self.n = 0

        def append(self, root, repo, t):
            self.n += 1
            return self.n - 1
    alone = str(tmp_path / "out" / "alone")
    assert safety.build(Sequential(), alone, takes, log=lambda m: None) == written  # the same episodes and labels
    for name in ("manual_safety_labels.json", "safety_events.jsonl", "capture_timing.jsonl", "raw_ros_events.jsonl"):
        assert open(os.path.join(out, name)).read() == open(os.path.join(alone, name)).read()


def test_the_workers_run_at_most_ahead_of_the_writer(scenarios, tmp_path):
    from concurrent.futures import ThreadPoolExecutor

    from teleop_lerobot.writer import prepared_in_order

    takes = [read_take(take(scenarios, "reach_no_obstacle", [], name=f"20261121T1000{i:02d}Z-take0{i:02d}")) for i in range(7)]
    w = PreparingWriter(total=7)
    root = str(tmp_path / "ds")
    seen = []
    with ThreadPoolExecutor(7) as pool:
        stream = prepared_in_order(w, root, takes, 2, executor=pool, ahead=3)
        for i, (t, prepared) in enumerate(stream):
            assert prepared.episode == i and t is takes[i]
            assert len(w.started) <= i + 4  # this one and the three after it, at most
            seen.append(prepared.folder)
            assert os.path.exists(prepared.folder)
    assert len(seen) == 7 and not os.path.exists(root + ".prepare")


def test_a_worker_that_fails_stops_the_build_and_leaves_no_prepared_files(scenarios, tmp_path):
    from concurrent.futures import ThreadPoolExecutor

    takes = [read_take(take(scenarios, "reach_no_obstacle", [], name=f"20261121T1000{i}0Z-take00{i}", at_s=10 * i)) for i in range(4)]
    w = PreparingWriter(fail_at=1, total=4)
    out = str(tmp_path / "out" / "failed")
    with ThreadPoolExecutor(2) as pool, pytest.raises(RuntimeError, match="episode 1 could not be prepared"):
        safety.build(w, out, takes, log=lambda m: None, jobs=2, executor=pool)
    assert [ep for ep, _, _ in w.episodes] == [0]  # what came before it was written; nothing after
    assert not os.path.exists(out + ".prepare") and not os.path.exists(os.path.join(out, "manual_safety_labels.json"))


def test_one_job_prepares_nothing(scenarios, tmp_path):
    class Writer:
        def prepare_job(self):
            raise AssertionError("jobs=1 builds in this process, as before")

        def append(self, root, repo, t):
            return 0
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    t = read_take(take(scenarios, "reach_no_obstacle", []))
    safety.build(Writer(), str(tmp_path / "one"), [t], log=lambda m: None, jobs=1)
    assert not os.path.exists(str(tmp_path / "one") + ".prepare")


def test_a_take_and_the_prepare_job_go_to_a_spawned_worker(scenarios):
    # The workers are spawned, not forked (LeRobot brings torch): what they are handed must pickle, and come back the same.
    import functools
    import pickle

    from teleop_lerobot.writer import prepare_episode

    t = safety.as_safety(read_take(take(scenarios, "collide_and_recover", [wc(1500, "0:5:2:1.5"), wc(1620)])))
    job = functools.partial(prepare_episode, vcodec="libsvtav1", crf=18, preset=8)
    again = pickle.loads(pickle.dumps(job))
    assert again.func is prepare_episode and again.keywords == {"vcodec": "libsvtav1", "crf": 18, "preset": 8}
    back = pickle.loads(pickle.dumps(t))
    assert back == t and back.jpeg(back.frames[3], back.cameras[1]) == t.jpeg(t.frames[3], t.cameras[1])


def _fake_dataset(root, episodes, stamp="2026-11-21T10:00:00Z", flip=None):
    """A dataset's files as the safety build lays them out (one file per episode and kind), without LeRobot."""
    def put(rel, data):
        p = os.path.join(root, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "wb" if isinstance(data, bytes) else "w") as f:
            f.write(data)
    put("meta/info.json", json.dumps({"total_episodes": episodes}))
    put("meta/stats.json", json.dumps({"episodes": episodes}))
    for ep in range(episodes):
        put(f"data/chunk-000/file-{ep:03d}.parquet", bytes([ep, 1, 2, 3]) if ep != flip else bytes([ep, 1, 2, 4]))
        put(f"videos/observation.images.front/chunk-000/file-{ep:03d}.mp4", bytes([ep]) * 9)
        put(f"meta/teleop/episode_{ep:06d}.timing.jsonl", json.dumps({"frameIndex": 0, "ep": ep}) + "\n")
    put("capture_timing.jsonl", "".join(json.dumps({"episode_saved_so_far": ep, "frame_in_episode": i}) + "\n"
                                        for ep in range(episodes) for i in range(2)))
    put("raw_ros_events.jsonl", "".join(json.dumps({"type": "state", "n": n}) + "\n" for n in range(3 * episodes)))
    put("safety_events.jsonl", "".join(json.dumps({"episode": ep, "kind": "contact"}) + "\n" for ep in range(episodes) if ep % 2))
    put("meta/teleop/takes.jsonl", "".join(json.dumps({"episode": ep, "take": f"t{ep}", "convertedUtc": stamp}) + "\n"
                                           for ep in range(episodes)))
    put("manual_safety_labels.json", json.dumps({str(ep): [[1, 2]] for ep in range(episodes) if ep % 2}))


def test_a_rebuild_of_the_first_episodes_compares_file_for_file(tmp_path):
    ref, same, prefix, other = (str(tmp_path / n) for n in ("ref", "same", "prefix", "other"))
    _fake_dataset(ref, 4)
    _fake_dataset(same, 4, stamp="2026-11-22T09:00:00Z")  # written another time: the same
    _fake_dataset(prefix, 2)  # the first two episodes: the totals are its own
    _fake_dataset(other, 3, flip=1)
    with open(os.path.join(same, "meta", "stats.json"), "w") as f:
        f.write('{\n  "episodes": 4\n}')  # the same values, written otherwise
    assert safety.compare_datasets(ref, same) == []
    assert safety.compare_datasets(ref, prefix) == []
    assert safety.compare_datasets(ref, other) == [os.path.join("data", "chunk-000", "file-001.parquet") + ": differs"]
    with open(os.path.join(same, "manual_safety_labels.json"), "w") as f:
        f.write(json.dumps({"1": [[1, 3]], "3": [[1, 2]]}))
    assert safety.compare_datasets(ref, same) == ["manual_safety_labels.json: differs (episodes [1])"]
    _fake_dataset(str(tmp_path / "more"), 5)
    assert "more than" in safety.compare_datasets(ref, str(tmp_path / "more"))[0]
