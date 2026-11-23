"""Safety data where the scene decides (docs/adr/0038 decision 6): the table is object 0 and never a contact, the mixed script
is set aside only when broken, every episode's outcome and sidecars come from the take, labels in world-models' format, and a
batch splits into datasets by layout."""

import hashlib
import json
import math
import os
import re
import sys
import types

import pytest

from conftest import MS, REPO, jetrover_take
from teleop_lerobot import safety, sidecars
from teleop_lerobot.takes import read_take

UPRIGHT = (1.0, 0.0, 0.0, 0.0)
ON_ITS_SIDE = (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0)
SCENE_SHA = "ab" * 32
# The two cameras' poses as a take of jetrover-bench recorded them (2026-11-22, take175 of collide_with_obstacle).
THIRD_LEFT = ["CM", "third-left", 64, 48, 465.6, 465.6, 32, 24, 0.02, 6.0, 0.88, 0.51, 0.32, 0.3808798, 0.2935164, 0.5352036,
              0.694504, "", "rgb;depth;mask", "base"]
WRIST = ["CM", "wrist", 64, 48, 337.2, 337.2, 32, 24, 0.01, 6.0, 0.1, 0.0, 0.05, -0.5811892, -0.4027644, 0.4027644, 0.5811892,
         "wrist_link", "rgb;depth;mask", "mount"]
SCENE = {
    "name": "test-bench",
    "objects": [
        {"id": 2, "name": "can", "shape": "mesh", "asset": "ycb/can", "size": [0.06, 0.06, 0.10], "kind": "movable", "mass": 0.2},
        {"id": 3, "name": "cube", "shape": "box", "size": [0.05, 0.05, 0.05], "kind": "movable", "mass": 0.1},
        {"id": 4, "name": "vase", "shape": "mesh", "asset": "polyhaven/vase", "size": [0.2, 0.2, 0.4], "kind": "fixed"},
        {"id": 5, "name": "mug", "shape": "mesh", "asset": "ycb/mug", "size": [0.1, 0.08, 0.08], "kind": "movable", "mass": 0.12},
        {"id": 6, "name": "spare", "shape": "box", "size": [0.05, 0.05, 0.05], "kind": "movable", "mass": 0.1}],
    "surfaces": [{"id": "table", "center": [0.3, 0.0, -0.2], "size": [0.4, 1.6, 0.04], "collides": True}],
    "pool": {"edgeRegions": [-0.2, -0.12, 0.4, 0.62, -0.2, -0.12, -0.62, -0.4]},
    "cameras": [{"id": "third-left", "position": [0.88, 0.51, 0.32], "lookAt": [0.08, 0.0, 0.07]},
                {"id": "wrist", "mount": "wrist_link", "mountPosition": [0.1, 0.0, 0.05], "mountLookAt": [0.26, 0.0, -0.01],
                 "mountUp": [0.0, 0.0, 1.0]}]}
STANDING = {2: (0.24, -0.08, -0.15, UPRIGHT), 3: (0.17, -0.02, -0.17, UPRIGHT), 4: (0.0, -0.35, -0.0, UPRIGHT),
            5: (-0.16, 0.5, -0.16, UPRIGHT), 6: (17.0, 1.0, -4.97, UPRIGHT)}


def wc(t_ms, pairs=""):
    return ["WC", t_ms * MS + MS, t_ms * MS, pairs]


def wo(t_ms, poses):
    text = ";".join(f"{o}:{x}:{y}:{z}:{q[0]}:{q[1]}:{q[2]}:{q[3]}" for o, (x, y, z, q) in poses.items())
    return ["WO", t_ms * MS + MS, t_ms * MS, 1, text]


def world(extra=(), poses=None, ids="2;3;4;5", seed=639309532987986191, sizes="2:1;3:1.25;4:1;5:1;6:1"):
    """The records a take of a scene opens with: its layout, sizes and poses, then ``extra``."""
    return [["WL", 900 * MS, 89, seed, ids], ["WZ", 900 * MS, 0, sizes], wo(900, poses or STANDING)] + list(extra)


def take(root, scenario, extra, name="20261122T100000Z-take001", **kw):
    kw.setdefault("scene", SCENE)
    kw.setdefault("session", {"scene": {"name": "test-bench", "file": "scene.json", "sha256": SCENE_SHA, "seed": 1}})
    kw.setdefault("cm", lambda c: {"third-left": THIRD_LEFT, "wrist": WRIST}.get(c))
    return jetrover_take(root, name=name, scenario=scenario, extra=extra, **kw)


def episode(path, **options):
    return sidecars.episode(read_take(path), sidecars.Options(**options))


class FakeClearance:
    """Workstream C's interface (docs/adr/0038): ``compute(take, geometry_dir) -> Clearance``, here a constant per link
    plus ``dip`` mm off link 0 at frame ``at``."""

    def __init__(self, base=80.0, dip=0.0, at=5, nearest=3):
        self.base, self.dip, self.at, self.nearest, self.calls = base, dip, at, nearest, []

    def module(self):
        fake = types.ModuleType("teleop_lerobot.clearance")

        class ClearanceUnavailable(Exception):
            pass

        def compute(take, geometry_dir):
            self.calls.append(geometry_dir)
            n = len(take.frames)
            obstacle = [(self.base - (self.dip if i == self.at else 0.0), self.base + 10.0) for i in range(n)]
            return types.SimpleNamespace(links=("gripper_link", "wrist_link"), obstacle_mm=obstacle,
                                         obstacle_id=[(self.nearest, 4)] * n, surface_mm=[(40.0, 90.0)] * n, cutoff_mm=200.0,
                                         method="fake")
        fake.compute, fake.ClearanceUnavailable = compute, ClearanceUnavailable
        return fake


@pytest.fixture
def clearance(monkeypatch, tmp_path):
    """Installs a :class:`FakeClearance` as teleop_lerobot.clearance; returns it with a geometry folder."""
    import teleop_lerobot

    def install(fake):
        module = fake.module()
        monkeypatch.setitem(sys.modules, "teleop_lerobot.clearance", module)
        monkeypatch.setattr(teleop_lerobot, "clearance", module, raising=False)
        geometry = tmp_path / "geometry"
        geometry.mkdir(exist_ok=True)
        return str(geometry)
    return install


# --- the table, and the old categories --------------------------------------------------------------------------------------

def test_the_table_is_never_a_contact_for_the_old_checks_and_unsafe_in_the_labels_unless_built_table_safe(scenarios, capsys):
    from teleop_lerobot.__main__ import main

    # docs/adr/0038 decision 4: the bridge reports the arm touching the table as WC object 0. A reach take that only brushes the
    # table is still a reach take, kept as before. Since 2026-11-23 (world-models' review: on the real robot a hard table hit
    # is a collision) the labels count the brush; table_safe leaves it out, as builds did before.
    path = take(scenarios, "reach_no_obstacle", world([wc(1500, "0:5:0:2.0"), wc(1700)]))
    t = read_take(path)
    assert safety.contacts(path) == [] and len(safety.contacts(path, table=True)) == 1
    labels = safety.frame_labels(t, safety.contacts(path))
    assert sum(labels) == 0 and safety.check(t, labels, safety.contacts(path), safety.object_poses(path)).ok
    assert main(["safety-check", path]) == 0  # the verdict, and its line's counts, as before: obstacles only
    line = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert line["ok"] and line["reason"] == "no contact" and line["unsafeFrames"] == 0 and line["tableContact"]
    e = episode(path)
    assert sum(e.labels) > 0 and e.labels == [int(v) for v in e.frames["in_contact_table"]]  # the table's frames, no others
    assert not any(e.frames["in_contact"]) and e.frames["unsafe"] == e.labels and e.line["unsafeRanges"] == 1
    assert sum(episode(path, table_safe=True).labels) == 0
    # The brush ends, so its range has a clear as a recover's does (docs/adr/0038, 2026-11-23); a knock's never has one.
    assert [x["kind"] for x in safety.events(0, "reach_no_obstacle", e.labels)] == ["contact", "clear"]
    assert [x["kind"] for x in safety.events(0, "collide_with_obstacle", e.labels)] == ["contact"]
    # A touch on an object while the gripper also rests on the table: the object's stretch ends when its pair goes, though
    # the table's carries on to the episode's end. A recover take is judged by its obstacle touches alone: it backed off.
    rec = take(scenarios, "collide_and_recover", world([wc(1500, "0:5:2:1.5;0:6:0:3.0"), wc(1620, "0:6:0:3.0")]),
               name="20261122T100100Z-take002")
    stretches = safety.contacts(rec)
    assert [(c.start, c.end, c.objects) for c in stretches] == [(1500 * MS, 1620 * MS, (2,))]
    t = read_take(rec)
    v = safety.check(t, safety.frame_labels(t, stretches), stretches, safety.object_poses(rec))
    assert v.ok and v.touched == (2,) and v.contacts == 1
    assert [s.object for s in sidecars.object_stretches(sidecars.read_records(rec).host)] == [0, 2]
    e = episode(rec)
    assert e.labels[-1] == 1 and e.frames["in_contact_table"][-1] and not e.frames["in_contact"][-1]  # the table, to the end
    assert episode(rec, table_safe=True).labels == safety.frame_labels(t, stretches)


def test_the_jetrover_links_are_the_urdfs_in_order():
    with open(os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.urdf"), encoding="utf-8") as f:
        assert tuple(re.findall(r'<link name="([^"]+)"', f.read())) == sidecars.JETROVER_LINKS


# --- the mixed script: kept unless broken ------------------------------------------------------------------------------------

def test_a_mixed_take_is_kept_whatever_happened_and_set_aside_only_when_broken(scenarios):
    knocked = world([wc(1400, "0:5:3:6.0"), wc(1900), wo(2500, {**STANDING, 3: (0.21, -0.05, -0.18, ON_ITS_SIDE)})])
    path = take(scenarios, "mixed", knocked)
    t = read_take(path)
    v = safety.check(t, safety.frame_labels(t, safety.contacts(path)), safety.contacts(path), safety.object_poses(path))
    assert v.ok and "knocked [3]" in v.reason
    assert safety.broken(path, t) == []
    assert safety.check(read_take(take(scenarios, "mixed", world(), name="20261122T100100Z-take002")), [0], [], {}).ok  # nothing
    assert safety.broken(path, t, drive_exit=4) == [] and safety.broken(path, t, drive_exit=0) == []
    assert "stalled" in safety.broken(path, t, drive_exit=2)[0] and "stream failed" in safety.broken(path, t, drive_exit=3)[0]
    assert "by hand" in safety.broken(path, t, drive_exit=130)[0]
    # Frames that ended early, a writer that dropped records, no world at all, a renderer that stalled for over a second.
    er = take(scenarios, "mixed", world(), name="20261122T100200Z-take003", isaac_extra=[["ER", 2900 * MS, "disk-low"]])
    assert any("ended early" in r and "disk-low" in r for r in safety.broken(er, read_take(er)))
    dropped = take(scenarios, "mixed", world(), name="20261122T100300Z-take004", dropped=3)
    assert any("dropped 3" in r for r in safety.broken(dropped, read_take(dropped)))
    bare = take(scenarios, "mixed", [], name="20261122T100400Z-take005")
    assert any("no object poses" in r for r in safety.broken(bare, read_take(bare)))
    stalled = take(scenarios, "mixed", world(), name="20261122T100500Z-take006", gap=(1.4, 2.8))
    assert any("repeat one render" in r for r in safety.broken(stalled, read_take(stalled)))
    short_gap = take(scenarios, "mixed", world(), name="20261122T100600Z-take007", gap=(1.5, 1.9))
    assert safety.broken(short_gap, read_take(short_gap)) == []  # under a second: kept


def test_safety_check_sets_a_broken_mixed_take_aside_and_says_what_happened_in_a_whole_one(scenarios, capsys):
    from teleop_lerobot.__main__ import main

    whole = take(scenarios, "mixed", world([wc(1500, "0:5:2:1.5"), wc(1620), ["RP", 950 * MS, "7.1", "source", "none"]]))
    assert main(["safety-check", "--set-aside", whole]) == 0
    line = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert line["ok"] and line["broken"] == [] and line["outcome"] == "contact" and line["group"] == "7.1"
    assert line["setAside"] is None and os.path.exists(whole)
    cut = take(scenarios, "mixed", world([wc(1500, "0:5:2:1.5"), wc(1620)]), name="20261122T100100Z-take002")
    assert main(["safety-check", "--set-aside", "--drive-exit", "2", cut]) == 1
    line = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert not line["ok"] and line["reason"].startswith("broken: ") and "stalled" in line["reason"]
    assert line["setAside"] == os.path.join(scenarios, "_discarded", "mixed", os.path.basename(cut))
    # One that cannot be read at all (its robothost.jlog cut off) gets a verdict too, and is set aside.
    unreadable = take(scenarios, "mixed", world(), name="20261122T100200Z-take003")
    with open(os.path.join(unreadable, "robothost.jlog"), "rb+") as f:
        f.truncate(os.path.getsize(os.path.join(unreadable, "robothost.jlog")) - 30)
    assert main(["safety-check", "--set-aside", unreadable]) == 1
    line = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert not line["ok"] and "cannot be read" in line["reason"] and line["frames"] == 0 and line["setAside"]
    # The four old categories keep their checks: an unreadable one still raises, as before.
    old = take(scenarios, "reach_no_obstacle", world(), name="20261122T100300Z-take004")
    with open(os.path.join(old, "robothost.jlog"), "rb+") as f:
        f.truncate(os.path.getsize(os.path.join(old, "robothost.jlog")) - 30)
    with pytest.raises(Exception):
        main(["safety-check", old])


# --- the outcome ------------------------------------------------------------------------------------------------------------

def test_each_episode_has_an_outcome_and_unknown_when_no_clearance_tells_a_near_miss(scenarios, clearance):
    knock = take(scenarios, "mixed", world([wc(1400, "0:5:3:6.0"), wc(1900), wo(2500, {**STANDING, 3: (0.21, -0.05, -0.18, ON_ITS_SIDE)})]))
    assert episode(knock).line["outcome"] == "knock" and episode(knock).line["knocked"] == [3]
    touch = take(scenarios, "mixed", world([wc(1500, "0:5:2:1.5"), wc(1620)]), name="20261122T100100Z-take002")
    e = episode(touch)
    assert e.line["outcome"] == "contact" and e.line["touched"] == [2] and e.line["knocked"] == []
    assert e.line["contacts"] == 1 and not e.line["tableContact"]
    table = take(scenarios, "mixed", world([wc(1500, "0:6:0:4.0"), wc(1700)]), name="20261122T100200Z-take003")
    e = episode(table)
    assert e.line["outcome"] == "unknown" and e.line["tableContact"] and e.line["tableContacts"] == 1
    assert e.line["minClearanceMm"] is None and e.line["clearanceUnavailable"] == "no geometry folder given"
    assert sum(e.labels) > 0 and sum(episode(table, table_safe=True).labels) == 0  # counted; the builder flag leaves it out
    # With clearance (workstream C's module, faked): 20 mm at its closest is a near miss, 40 mm no contact.
    geometry = clearance(FakeClearance(base=45.0, dip=25.0))
    e = episode(table, geometry_dir=geometry)
    assert e.line["outcome"] == "near_miss" and e.line["minClearanceMm"] == 20.0 and e.line["minClearanceObject"] == 3
    assert e.line["clearanceMethod"] == "fake" and e.line["clearanceCutoffMm"] == 200.0 and e.line["clearanceUnavailable"] is None
    geometry = clearance(FakeClearance(base=45.0, dip=5.0))
    assert episode(table, geometry_dir=geometry).line["outcome"] == "no_contact"
    far = episode(table, geometry_dir=clearance(FakeClearance(base=150.0, nearest=-1)))  # C's cutoff, nothing within it
    assert far.line["outcome"] == "no_contact" and far.line["minClearanceObject"] is None
    assert set(far.frames["clearance_id.gripper_link"]) == {None}
    assert episode(touch, geometry_dir=geometry).line["outcome"] == "contact"  # a contact needs no clearance
    assert episode(table, geometry_dir=str(scenarios) + "/none").line["clearanceUnavailable"].startswith("no geometry at")


def test_a_clearance_module_that_fails_leaves_nan_and_says_why(scenarios, monkeypatch, tmp_path):
    import teleop_lerobot

    fake = types.ModuleType("teleop_lerobot.clearance")

    class ClearanceUnavailable(Exception):
        pass

    def compute(take, geometry_dir):
        raise ClearanceUnavailable("no VQ in this take")
    fake.compute, fake.ClearanceUnavailable = compute, ClearanceUnavailable
    monkeypatch.setitem(sys.modules, "teleop_lerobot.clearance", fake)
    monkeypatch.setattr(teleop_lerobot, "clearance", fake, raising=False)
    path = take(scenarios, "mixed", world())
    e = episode(path, geometry_dir=str(tmp_path))
    assert e.line["clearanceUnavailable"] == "ClearanceUnavailable: no VQ in this take" and e.line["outcome"] == "unknown"
    assert all(v is None for v in e.frames["min_clearance_mm"])


# --- contacts.jsonl, frames.parquet's columns ---------------------------------------------------------------------------------

def test_each_contact_stretch_is_a_line_with_its_links_force_and_what_it_did_to_its_object(scenarios):
    rows = world([wc(1400, "0:5:2:1.0;0:7:2:2.5"), wc(1450, "0:5:2:4.0"), wc(1500), wc(1800, "0:4:0:3.0"), wc(1900),
                  wc(2100, "0:5:2:0.5"), wc(2150), wo(1700, {**STANDING, 2: (0.30, -0.08, -0.15, UPRIGHT)}),
                  wo(2600, {**STANDING, 2: (0.31, -0.08, -0.18, ON_ITS_SIDE)})])
    e = episode(take(scenarios, "mixed", rows))
    first, table, second = e.contacts
    assert (first["object"], first["name"], first["kind"]) == (2, "can", "object")
    assert first["links"] == ["gripper_link", "right_finger"] and first["linkIndices"] == [5, 7]
    assert first["peakForceN"] == 4.0  # the larger of 1.0 + 2.5 and 4.0
    assert first["startS"] == pytest.approx(0.4) and first["endS"] == pytest.approx(0.5) and not first["open"]
    assert first["startFrame"] == 6 and first["endFrame"] == 8  # frames every 1/15 s from 1.0 s, each +-33 ms: 1.400-1.533 s
    # Moved 6 cm before the next touch began, so the first pushed it; the second tipped it, measured to the episode's end.
    assert first["moved"] and not first["tipped"] and first["movedCm"] == pytest.approx(6.0, abs=0.01)
    assert second["tipped"] and second["tiltDeg"] == pytest.approx(90.0, abs=0.1)
    assert (table["object"], table["kind"], table["links"], table["moved"]) == (0, "table", ["wrist_link"], None)
    assert e.line["contacts"] == 2 and e.line["tableContacts"] == 1 and e.line["outcome"] == "knock"
    # Still touching when the take ended: open, with no end time.
    held = episode(take(scenarios, "mixed", world([wc(2900, "0:3:4:1.0")]), name="20261122T100100Z-take002"))
    assert held.contacts[0]["open"] and held.contacts[0]["endS"] is None and held.contacts[0]["kind"] == "fixture"
    assert held.line["timingFlags"] == ["still touching at the end"]


def test_every_frame_has_its_flags_labels_time_to_contact_fill_and_skews(scenarios, clearance):
    path = take(scenarios, "mixed", world([wc(1500, "0:5:2:1.5"), wc(1620), wc(2000, "0:6:0:3.0"), wc(2100)]), gap=(2.3, 2.45))
    t = read_take(path)
    e = episode(path)
    f = e.frames
    n = len(t.frames)
    assert f["frame_index"] == list(range(n)) and f["sensed_ns"] == [x.timing["sensed"] for x in t.frames]
    assert any(f["in_contact_table"]) and not any(a and b for a, b in zip(f["in_contact"], f["in_contact_table"]))
    assert f["unsafe"] == [int(a or b) for a, b in zip(f["in_contact"], f["in_contact_table"])]  # an obstacle or the table
    safe = episode(path, table_safe=True).frames  # the table left out: the obstacles' frames alone, the check's rule
    assert safe["unsafe"] == safety.frame_labels(t, safety.contacts(path)) == [int(v) for v in f["in_contact"]]
    assert safe["in_contact_table"] == f["in_contact_table"]  # the kinds stay apart either way
    first = f["in_contact"].index(True)
    assert f["time_to_contact_s"][first] == 0.0
    assert f["time_to_contact_s"][first - 1] == pytest.approx((1500 * MS - f["sensed_ns"][first - 1]) / 1e9, abs=1e-6)
    assert f["time_to_contact_s"][-1] is None  # no contact follows; a table contact is no obstacle contact
    assert f["filled"] == [bool(x.timing["filled"]) for x in t.frames] and any(f["filled"])  # the gap repeats a render
    assert f["state_skew_ms"] == [x.timing["stateAge"] / 1e6 for x in t.frames]
    assert f["action_skew_ms"] == [x.timing["actionAge"] / 1e6 for x in t.frames]
    assert all(v is None for v in f["min_clearance_mm"])  # no geometry: NaN
    g = episode(path, geometry_dir=clearance(FakeClearance(base=60.0, dip=30.0, at=2))).frames
    assert g["clearance_mm.gripper_link"][2] == 30.0 and g["clearance_mm.wrist_link"][2] == 70.0
    assert g["clearance_id.gripper_link"][0] == 3 and g["surface_mm.gripper_link"][0] == 40.0
    assert g["min_clearance_mm"][2] == 30.0 and g["min_clearance_id"][2] == 3 and g["min_surface_mm"][0] == 40.0


# --- the layout, the scene, the cameras ---------------------------------------------------------------------------------------

def test_the_layout_hash_is_its_documented_recipe(scenarios):
    objects = [(3, [0.05, 0.05, 0.05], (0.17, -0.02, -0.17, 1.0, 0.0, 0.0, 0.0)),
               (2, [0.06, 0.06, 0.1], (0.24, -0.08, -0.15) + ON_ITS_SIDE)]
    # The recipe, written out again here from its docstring: ints only, sorted by id, keys sorted, no spaces.
    rows = [[2, [60, 60, 100], [240, -80, -150], [1000, 0, 0, 0, 0, -1000, 0, 1000, 0]],
            [3, [50, 50, 50], [170, -20, -170], [1000, 0, 0, 0, 1000, 0, 0, 0, 1000]]]
    text = json.dumps({"objects": rows, "scene": SCENE_SHA, "v": 1}, separators=(",", ":"))
    assert sidecars.layout_hash(SCENE_SHA, objects) == hashlib.sha256(text.encode()).hexdigest()
    h = sidecars.layout_hash(SCENE_SHA, objects)
    flipped = [(2, [0.06, 0.06, 0.1], (0.24, -0.08, -0.15) + tuple(-q for q in ON_ITS_SIDE)), objects[0]]
    assert sidecars.layout_hash(SCENE_SHA, flipped) == h  # q and -q, in any order: the same layout
    nudged = [objects[0], (2, [0.06, 0.06, 0.1], (0.2402, -0.08, -0.15) + ON_ITS_SIDE)]
    assert sidecars.layout_hash(SCENE_SHA, nudged) == h  # 0.2 mm
    for other in ([objects[0], (2, [0.06, 0.06, 0.1], (0.242, -0.08, -0.15) + ON_ITS_SIDE)],  # 2 mm
                  [objects[0], (2, [0.06, 0.06, 0.12], (0.24, -0.08, -0.15) + ON_ITS_SIDE)],  # another size
                  [objects[0]]):  # one fewer
        assert sidecars.layout_hash(SCENE_SHA, other) != h
    assert sidecars.layout_hash("cd" * 32, objects) != h
    c, s = math.cos(math.radians(0.1)), math.sin(math.radians(0.1))  # turned 0.2 degrees about z
    assert sidecars.layout_hash(SCENE_SHA, [objects[1], (3, [0.05, 0.05, 0.05], (0.17, -0.02, -0.17, c, 0.0, 0.0, s))]) != h
    # In an episode: the objects WL put on the table, at their WZ sizes, where they stood at the first frame.
    e = episode(take(scenarios, "mixed", world()))
    starts = {o: (x, y, z) + q for o, (x, y, z, q) in STANDING.items()}
    want = sidecars.layout_hash(SCENE_SHA, [(2, [0.06, 0.06, 0.1], starts[2]), (3, [0.0625, 0.0625, 0.0625], starts[3]),
                                            (4, [0.2, 0.2, 0.4], starts[4]), (5, [0.1, 0.08, 0.08], starts[5])])
    assert e.line["layoutHash"] == want and e.scene["layoutHash"] == want
    assert e.line["layoutSeed"] == 639309532987986191 and e.line["layout"] == 89 and e.line["sceneSha256"] == SCENE_SHA


def test_the_scene_line_has_every_object_on_the_table_the_surfaces_and_the_cameras(scenarios):
    moved = {**STANDING, 2: (0.26, -0.08, -0.15, UPRIGHT)}
    edits = [["WE", 950 * MS, 7, "park", 3, "17:1:-4.97:1:0:0:0", "pending"], ["WE", 960 * MS, 7, "park", 3, "17:1:-4.97:1:0:0:0", "applied"],
             ["WP", 960 * MS, 7, "park", 3, "17:1:-4.97:1:0:0:0", "applied"]]
    e = episode(take(scenarios, "mixed", world([wo(1500, moved), wo(1600, moved)] + edits, ids="2;3;4;5;6")))
    assert [(x["op"], x["id"], x["outcome"]) for x in e.scene["edits"]] == [("park", 3, "pending"), ("park", 3, "applied")]
    assert e.scene["placements"] == [{"t": -0.04, "cause": "park", "id": 3, "outcome": "applied"}]
    objects = {o["id"]: o for o in e.scene["objects"]}
    assert sorted(objects) == [2, 3, 4, 5]  # 6 is in WL but on the parking shelf
    cube, vase, mug, can = objects[3], objects[4], objects[5], objects[2]
    assert cube["dimensionsM"] == [0.0625, 0.0625, 0.0625] and cube["sizeFactor"] == 1.25
    assert cube["massKg"] == pytest.approx(0.1 * 1.25 ** 3) and cube["primitive"] == "box" and cube["asset"] is None
    assert vase["kind"] == "fixture" and vase["massKg"] is None and vase["asset"] == "polyhaven/vase"
    assert mug["reach"] == "distractor" and can["reach"] == "in_reach"  # the mug stands by the bench's edge
    assert can["start"][:3] == [0.24, -0.08, -0.15] and can["end"][:3] == [0.26, -0.08, -0.15]
    assert [c[:4] for c in can["changes"]] == [[0.5, 0.26, -0.08, -0.15]] and cube["changes"] == []
    assert e.scene["surfaces"] == [{"id": "table", "center": [0.3, 0.0, -0.2], "size": [0.4, 1.6, 0.04], "collides": True}]
    cams = {c["name"]: c for c in e.scene["cameras"]}
    assert set(cams) == {"third-left", "front"} and cams["front"]["recorded"] == "wrist" and cams["front"]["poseFrame"] == "mount"
    assert cams["third-left"]["fx"] == 465.6 and cams["third-left"]["pose"][:3] == [0.88, 0.51, 0.32]
    # Where the scene file puts them, to the record's 7 digits: no jitter.
    assert all(j["mm"] < 0.01 and j["deg"] < 0.01 for j in e.line["cameraJitter"].values())


def test_a_mixed_pools_roles_say_what_is_in_reach(scenarios):
    # docs/adr/0038 decision 3: a mixed pool gives each object a role; a distractor is out of reach wherever it stands, and an
    # obstacle in reach even by the bench's edge (that rule is for scenes without roles).
    roles = json.loads(json.dumps(SCENE))
    for o in roles["objects"]:
        o["role"] = {2: "distractor", 5: "obstacle", 4: "stand"}.get(o["id"], "obstacle")
    e = episode(take(scenarios, "mixed", world(), scene=roles))
    by = {o["id"]: o for o in e.scene["objects"]}
    assert (by[2]["reach"], by[2]["role"]) == ("distractor", "distractor") and by[5]["reach"] == "in_reach"
    assert (by[4]["reach"], by[4]["role"], by[4]["kind"]) == ("in_reach", "stand", "fixture")


def test_camera_jitter_is_the_recorded_pose_against_the_scene_files(scenarios):
    shifted = list(THIRD_LEFT)
    shifted[10] = 0.885  # 5 mm along x
    e = episode(take(scenarios, "mixed", world(), cm=lambda c: shifted if c == "third-left" else WRIST))
    assert e.line["cameraJitter"]["third-left"]["mm"] == pytest.approx(5.0, abs=0.01)
    # The real scene file and the poses a take of it recorded agree, whichever sign the quaternion takes.
    with open(os.path.join(REPO, "core", "Scenes", "jetrover-bench.json"), encoding="utf-8") as f:
        bench = json.load(f)
    cams = sidecars.cameras(types.SimpleNamespace(tag=lambda name: [r[1:] for r in (THIRD_LEFT, WRIST)] if name == "CM" else []),
                            {"wrist": "front"})
    jitter = sidecars.camera_jitter(bench, cams)
    assert set(jitter) == {"third-left", "front"} and all(j["deg"] < 0.001 and j["mm"] < 0.001 for j in jitter.values())


def test_the_room_looks_clutter_moves_and_replay_go_into_the_episode_line(scenarios):
    isaac = [["EN", 800 * MS, 87, 1622916560, "royal_esplanade", "2bd8", 7.752, 1493.182, 18, 116.51,
              "third-left:1.0;wrist:0.974", "5600"],
             ["LK", 87, "bench:black-resin;table:fabric-denim"], ["MT", 87, "3:wood_planks"],
             ["CL", 87, "a4:0.43:0.36:2.95;a4:0.41:0.37:2.91;bag:0.33:0.45:1.84"],
             ["LT", 86, 4000, "", ""],  # an earlier change's: not this take's
             ["LT", 87, 5200, 110, "35.0:40.0:2.5:4500:5.0"],
             ["CJ", 87, "third-left:0.004:-0.002:0.001:0.5:-0.3:0.1;wrist:0.001:0:0:0.2:0:0"],
             ["OA", 87, "2:omnipbr:0.9:0.2:1.02:0.99:0.97"]]
    host = world([["RP", 950 * MS, "13.4", "replay", "shift:2:0.03:-0.01"],
                  ["MV", 1000 * MS, 1, 3, "aim", 2, -0.02, 0.031, 0.1, 0.9, 0.95, 1.4, "later"],
                  ["MV", 1000 * MS, 2, 3, "free", 0, "", "", 1.4, "", "", 2.0],
                  ["MV", 1000 * MS, 3, 3, "aim", 3, 0.01, 0.02, 2.0, 2.5, 2.5, 2.9]])
    e = episode(take(scenarios, "mixed", host, isaac_extra=isaac,
                     session={"conditions": {"speedSliderRadPerSec": 3.8, "operator": "tool 10.0.0.2:5000"},
                              "scene": {"name": "test-bench", "sha256": SCENE_SHA}}))
    line = e.line
    assert (line["group"], line["role"], line["edit"]) == ("13.4", "replay", "shift:2:0.03:-0.01")
    assert line["editParsed"] == {"op": "shift", "object": 2, "dx": 0.03, "dy": -0.01}
    room = line["room"]
    assert room["hdri"] == "royal_esplanade" and room["yawDeg"] == 7.752 and room["gains"] == {"third-left": 1.0, "wrist": 0.974}
    assert room["change"] == 87 and room["extra"] == ["5600"]  # a later field (workstream A's light) kept as it came
    assert line["looks"] == {"bench": "black-resin", "table": "fabric-denim"} and line["materials"] == {"3": "wood_planks"}
    assert line["clutter"] == {"pieces": 3, "kinds": {"a4": 2, "bag": 1},
                               "items": [["a4", 0.43, 0.36, 2.95], ["a4", 0.41, 0.37, 2.91], ["bag", 0.33, 0.45, 1.84]]}
    # Which change each look came from: a replay group shares one look, so its takes' numbers match.
    assert line["lookChanges"] == {"EN": 87, "LK": 87, "MT": 87, "CL": 87, "LT": 87, "CJ": 87, "OA": 87}
    moves = line["moves"]
    assert moves["count"] == 3 and moves["kinds"] == {"aim": 2, "free": 1}
    assert moves["records"][0]["kind"] == "aim" and moves["records"][0]["object"] == "2" and moves["records"][0]["extra"] == ["later"]
    assert moves["records"][0]["offsetMm"] == "-0.02" and moves["records"][0]["endS"] == "1.4"  # SESSION_FORMAT.md's names
    assert line["lighting"] == {"domeKelvin": 5200.0, "exposure": 110.0,
                                "key": {"azimuthDeg": 35.0, "elevationDeg": 40.0, "intensity": 2.5, "kelvin": 4500.0, "angleDeg": 5.0}}
    assert line["cameraJitter"]["front"]["pitchDeg"] == 0.2 and line["cameraJitter"]["third-left"]["dx"] == 0.004
    assert "mm" in line["cameraJitter"]["third-left"]  # the pose against the scene file's, beside the drawn offset
    assert line["objectColours"] == {"2": {"method": "omnipbr", "brightness": 0.9, "desaturation": 0.2, "tint": [1.02, 0.99, 0.97]}}
    assert line["speedSlider"] == 3.8 and line["scenario"] == "mixed" and line["events"] == []


def test_the_replay_record_is_read_exactly_as_adr_0038_defines_it():
    assert sidecars.parse_edit("none") == {"op": "none"}
    assert sidecars.parse_edit("remove:52") == {"op": "remove", "object": 52}
    assert sidecars.parse_edit("move:52:0.31:-0.12:1.5708") == {"op": "move", "object": 52, "x": 0.31, "y": -0.12, "yawRad": 1.5708}
    for bad in ("shift:52:0.1", "remove:x", "spin:52", "none:1", ""):
        with pytest.raises(ValueError):
            sidecars.parse_edit(bad)
    host = types.SimpleNamespace(tag=lambda name: [["5", "g", "source", "none"]] if name == "RP" else [])
    assert sidecars.replay(host)["role"] == "source"
    with pytest.raises(ValueError):
        sidecars.replay(types.SimpleNamespace(tag=lambda name: [["5", "g", "copy", "none"]] if name == "RP" else []))
    assert sidecars.replay(types.SimpleNamespace(tag=lambda name: []))["group"] is None  # an older take


# --- item 24, item 17 ---------------------------------------------------------------------------------------------------------

def test_contacts_too_near_either_end_are_reported_not_set_aside(scenarios):
    early = take(scenarios, "mixed", world([wc(1300, "0:5:2:1.5"), wc(1400), wc(2500, "0:5:3:1.0"), wc(2600)]))
    e = episode(early)
    assert e.line["firstContactS"] == pytest.approx(0.3) and e.line["afterLastContactS"] == pytest.approx(0.4)
    assert len(e.line["timingFlags"]) == 2 and "under 1.5 s" in e.line["timingFlags"][0] and "under 1.0 s" in e.line["timingFlags"][1]
    assert safety.broken(early, read_take(early)) == []
    # No contact, nothing to flag.
    assert episode(take(scenarios, "mixed", world(), name="20261122T100100Z-take002")).line["timingFlags"] == []


def test_unavoidable_ranges_come_by_default_for_the_twins_three_stop_times(scenarios, tmp_path):
    # Item 17: stopped, the arm goes on for the stop time of its speed, so the frames within that time before a contact can
    # no longer avoid it. Since 2026-11-23 by default, for the twin's fast, median and slow stops; the labels stay contact.
    from teleop_lerobot.__main__ import main

    path = take(scenarios, "mixed", world([wc(2000, "0:5:2:1.5"), wc(2100), wc(2600, "0:4:4:0.5"), wc(2650)]))
    e = episode(path)
    # Frames every 1/15 s from 1.0 s: the contacts span frames 15-17 and 24-25. Before each, the frames within the stop time:
    # 0.17 s from 1.867 and 2.467 s, 0.245 s from 1.8 and 2.4 s, 0.31 s from 1.733 and 2.333 s (the frames before those are
    # 0.2, 0.267 and 0.333 s out).
    assert [i for i, v in enumerate(e.frames["in_contact"]) if v] == [15, 16, 17, 24, 25] and sum(e.labels) == 5
    assert e.unavoidable == {"fast": [[13, 14], [22, 23]], "median": [[12, 14], [21, 23]], "slow": [[11, 14], [20, 23]]}
    assert episode(path, stop=sidecars.StopModel.given(0.35)).unavoidable == {"given": [[10, 14], [19, 23]]}
    assert episode(path, stop=None).unavoidable is None
    # Before the contacts the labels count: a table brush too, unless built table_safe.
    table = take(scenarios, "mixed", world([wc(2000, "0:6:0:3.0"), wc(2100)]), name="20261122T100100Z-take002")
    assert episode(table).unavoidable["median"] == [[12, 14]]
    assert episode(table, table_safe=True).unavoidable == {"fast": [], "median": [], "slow": []}
    for bad in (0.0, -0.1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            sidecars.StopModel.given(bad)
    for bad in ((), (("fast", 0.2), ("fast", 0.3)), (("", 0.2),)):  # none, a name twice, no name
        with pytest.raises(ValueError):
            sidecars.StopModel(bad)
    for argv in (["--stop-time-s", "0.3", "--no-unavoidable"], ["--stop-time-s", "0"]):  # refused before anything is read
        with pytest.raises(SystemExit):
            main(["safety-build", "--out", str(tmp_path / "refused"), *argv, path])
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    root, given, plain = (str(tmp_path / n) for n in ("ds", "given", "plain"))
    safety.build(Writer(), root, [read_take(path), read_take(table)], log=lambda m: None)
    written = json.load(open(os.path.join(root, "meta", "teleop", "unavoidable_ranges.json")))
    assert list(written) == ["source", "method", "stopTimes", "unavoidable_ranges"]
    assert written["source"] == "twin stop-latency 2026-11-23, provisional until the real arm is measured"
    assert written["stopTimes"] == {"fast": 0.17, "median": 0.245, "slow": 0.31} and "the labels stay contact" in written["method"]
    assert written["unavoidable_ranges"] == {"fast": {"0": [[13, 14], [22, 23]], "1": [[13, 14]]},
                                             "median": {"0": [[12, 14], [21, 23]], "1": [[12, 14]]},
                                             "slow": {"0": [[11, 14], [20, 23]], "1": [[11, 14]]}}
    report = json.load(open(os.path.join(root, "meta", "teleop", "report.json")))
    assert report["options"]["stopModel"] == {"stopTimes": {"fast": 0.17, "median": 0.245, "slow": 0.31}}
    assert json.load(open(os.path.join(root, "safety_ranges.json")))["unsafe_ranges"] == {"0": [[15, 17], [24, 25]], "1": [[15, 17]]}
    safety.build(Writer(), given, [read_take(path)], log=lambda m: None, options=sidecars.Options(stop=sidecars.StopModel.given(0.35)))
    written = json.load(open(os.path.join(given, "meta", "teleop", "unavoidable_ranges.json")))
    assert written["stopTimes"] == {"given": 0.35} and written["unavoidable_ranges"] == {"given": {"0": [[10, 14], [19, 23]]}}
    assert written["source"] == "given to the build (--stop-time-s)"
    safety.build(Writer(), plain, [read_take(path)], log=lambda m: None, options=sidecars.Options(stop=None))
    assert not os.path.exists(os.path.join(plain, "meta", "teleop", "unavoidable_ranges.json"))  # --no-unavoidable


# --- splitting a batch ---------------------------------------------------------------------------------------------------------

def test_a_split_never_parts_a_replay_group_or_a_layout():
    lines = [{"group": "1", "layoutHash": "a", "sceneSha256": "s", "layoutSeed": 1, "contacts": 1},
             {"group": "1", "layoutHash": "b", "sceneSha256": "s", "layoutSeed": 1, "contacts": 0},  # its replay
             {"group": "2", "layoutHash": "c", "sceneSha256": "s", "layoutSeed": 2, "contacts": 0},
             {"group": None, "layoutHash": "c", "sceneSha256": "s", "layoutSeed": 9, "contacts": 0},  # c again
             {"group": None, "layoutHash": "d", "sceneSha256": "s", "layoutSeed": 2, "contacts": 1},  # seed 2 again
             {"group": None, "layoutHash": "e", "sceneSha256": "s", "layoutSeed": 5, "contacts": 1},
             {"group": None, "layoutHash": "f", "sceneSha256": "s", "layoutSeed": 6, "contacts": 0}]
    bins = sidecars.split(lines, 3)
    assert sorted(i for b in bins for i in b) == list(range(7))
    where = {i: k for k, b in enumerate(bins) for i in b}
    assert where[0] == where[1] and where[2] == where[3] == where[4]
    assert sorted(len(b) for b in bins) == [2, 2, 3]
    assert sidecars.split(lines, 3) == bins  # deterministic
    with pytest.raises(ValueError, match="not enough for 5"):
        sidecars.split(lines, 5)


# --- the dataset ----------------------------------------------------------------------------------------------------------------

class Writer:
    """A LeRobot stand-in: records each take's task, and writes meta/info.json as LeRobot would (its episode count)."""

    def __init__(self):
        self.episodes = []

    def append(self, root, repo, t):
        self.episodes.append((repo, t.path, t.task))
        os.makedirs(os.path.join(root, "meta"), exist_ok=True)
        n = sum(1 for r, _, _ in self.episodes if r == repo)
        with open(os.path.join(root, "meta", "info.json"), "w") as f:
            json.dump({"total_episodes": n}, f)
        return n - 1


def batch(scenarios):
    """Four mixed takes: a source and its replay (one group), and two more on their own layouts."""
    return [take(scenarios, "mixed", world([wc(2000, "0:5:2:1.5"), wc(2100), ["RP", 950 * MS, "3.1", "source", "none"]]),
                 name="20261122T100000Z-take001"),
            take(scenarios, "mixed", world([["RP", 950 * MS, "3.1", "replay", "remove:2"]], ids="3;4;5"), name="20261122T100100Z-take002"),
            take(scenarios, "mixed", world([wc(1600, "0:5:0:2.0"), wc(1700)], seed=11, poses=ELSEWHERE), name="20261122T100200Z-take003"),
            take(scenarios, "mixed", world([wc(2000, "0:4:3:6.0"), wc(2300), wo(2600, {**THIRD, 3: (0.2, 0.0, -0.18, ON_ITS_SIDE)})],
                                           seed=12, poses=THIRD), name="20261122T100300Z-take004")]


ELSEWHERE = {**STANDING, 2: (0.20, 0.10, -0.15, UPRIGHT)}  # other layouts: the can stands elsewhere
THIRD = {**STANDING, 2: (0.28, 0.05, -0.15, UPRIGHT)}


def test_a_build_writes_world_models_labels_neutral_captions_and_every_sidecar(scenarios, tmp_path):
    pd = pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    paths = batch(scenarios)
    w, logged = Writer(), []
    out = str(tmp_path / "out" / "jetrover_20261122_100000_abcdef")
    written = safety.build(w, out, [read_take(p) for p in paths], log=logged.append)
    assert {task for _, _, task in w.episodes} == {"move the arm"}  # docs/adr/0038: no caption names the scenario
    ranges = json.load(open(os.path.join(out, "safety_ranges.json")))
    assert list(ranges) == ["source", "notes", "unsafe_ranges"] and ranges["source"] == "physics"  # exactly world-models' keys
    assert ranges["unsafe_ranges"] == json.load(open(os.path.join(out, "manual_safety_labels.json")))
    # The table counts (episode 2 only brushed it), and the notes say why and what it costs (docs/adr/0038, 2026-11-23).
    assert set(ranges["unsafe_ranges"]) == {"0", "2", "3"} and ranges["notes"] == safety.NOTES_TABLE
    assert "a hard table hit is a collision" in ranges["notes"] and "'the path goes low'" in ranges["notes"]
    assert all(isinstance(x, int) for rs in ranges["unsafe_ranges"].values() for r in rs for x in r)
    events = [json.loads(x) for x in open(os.path.join(out, "safety_events.jsonl"))]
    assert [(e["episode"], e["kind"]) for e in events] == [(0, "contact"), (0, "clear"), (2, "contact"), (2, "clear"),
                                                           (3, "contact"), (3, "clear")]
    lines = [json.loads(x) for x in open(os.path.join(out, "meta", "teleop", "episodes.jsonl"))]
    assert [l["episode"] for l in lines] == [0, 1, 2, 3] and [l["outcome"] for l in lines] == ["contact", "unknown", "unknown", "knock"]
    assert [l["role"] for l in lines] == ["source", "replay", None, None] and lines[2]["tableContact"]
    scenes = [json.loads(x) for x in open(os.path.join(out, "meta", "teleop", "scenes.jsonl"))]
    assert [s["episode"] for s in scenes] == [0, 1, 2, 3] and [o["id"] for o in scenes[1]["objects"]] == [3, 4, 5]
    contacts = [json.loads(x) for x in open(os.path.join(out, "meta", "teleop", "contacts.jsonl"))]
    assert [(c["episode"], c["kind"]) for c in contacts] == [(0, "object"), (2, "table"), (3, "object")]
    frames = pd.read_parquet(os.path.join(out, "meta", "teleop", "frames.parquet"))
    assert len(frames) == sum(len(l) for _, _, l in written)
    assert list(frames.columns[:4]) == ["episode_index", "frame_index", "sensed_ns", "t_s"]
    assert frames["unsafe"].tolist() == [v for _, _, l in written for v in l]
    assert frames["min_clearance_mm"].isna().all() and str(frames["min_clearance_id"].dtype) == "Int32"
    assert frames["in_contact_table"].sum() > 0 and (frames.loc[frames["in_contact_table"], "unsafe"] == 1).all()
    report = json.load(open(os.path.join(out, "meta", "teleop", "report.json")))
    assert report["episodes"] == 4 and report["outcomes"] == {"contact": 1, "knock": 1, "unknown": 2}
    assert report["contacts"] == {"object": 2, "table": 1} and report["replayGroups"] == 1 and report["scenarios"] == {"mixed": 4}
    assert report["options"]["caption"] == "neutral" and report["options"]["tableSafe"] is False and report["split"] is None
    assert any(m.startswith("[mix] jetrover_20261122_100000_abcdef: 4 episode(s)") for m in logged)
    assert any(m.startswith("episode 3: mixed") and m.endswith("knock") for m in logged)
    # Built table_safe: the rule before 2026-11-23, word for word its notes; the sidecars hold the same contacts.
    safe = str(tmp_path / "safe")
    safety.build(Writer(), safe, [read_take(p) for p in paths], log=lambda m: None, options=sidecars.Options(table_safe=True))
    ranges = json.load(open(os.path.join(safe, "safety_ranges.json")))
    assert set(ranges["unsafe_ranges"]) == {"0", "3"} and ranges["notes"] == safety.NOTES and "other than the table" in safety.NOTES
    assert [json.loads(x)["episode"] for x in open(os.path.join(safe, "safety_events.jsonl"))] == [0, 0, 3, 3]
    frames = pd.read_parquet(os.path.join(safe, "meta", "teleop", "frames.parquet"))
    assert frames["in_contact_table"].sum() > 0 and frames.loc[frames["in_contact_table"], "unsafe"].sum() == 0
    assert json.load(open(os.path.join(safe, "meta", "teleop", "report.json")))["options"]["tableSafe"] is True
    # The old captions on request, for comparing a rebuild with a dataset from before.
    w2 = Writer()
    safety.build(w2, str(tmp_path / "old"), [read_take(paths[0])], log=lambda m: None, options=sidecars.Options(caption="category"))
    assert [task for _, _, task in w2.episodes] == ["mixed"]


def test_a_split_build_writes_datasets_each_with_its_report_and_never_parts_a_group(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    paths = batch(scenarios)
    w, logged = Writer(), []
    out = str(tmp_path / "out" / "jetrover_20261122_100000_abcdef")
    roots = safety.build_split(w, out, [read_take(p) for p in paths], 2, log=logged.append)
    assert [os.path.basename(r) for r in roots] == ["jetrover_20261122_100000_abcdef_1of2", "jetrover_20261122_100000_abcdef_2of2"]
    takes_in = [[json.loads(x)["take"] for x in open(os.path.join(r, "meta", "teleop", "takes.jsonl"))] for r in roots]
    assert sorted(t for ts in takes_in for t in ts) == sorted(os.path.abspath(p) for p in paths)  # each take once
    assert any(os.path.abspath(paths[0]) in ts and os.path.abspath(paths[1]) in ts for ts in takes_in)  # the group together
    for k, r in enumerate(roots, start=1):
        report = json.load(open(os.path.join(r, "meta", "teleop", "report.json")))
        assert report["split"] == {"part": k, "of": 2} and report["episodes"] == len(takes_in[k - 1])
        lines = [json.loads(x) for x in open(os.path.join(r, "meta", "teleop", "episodes.jsonl"))]
        assert [l["episode"] for l in lines] == list(range(len(lines)))  # each dataset counts its own episodes
    assert sum(1 for m in logged if m.startswith("[mix] ")) == 2
    with pytest.raises(FileExistsError):
        safety.build_split(w, out, [read_take(p) for p in paths], 2, log=lambda m: None)


def test_safety_compare_reads_the_new_files_and_an_older_reference_without_them(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    paths = batch(scenarios)
    takes = [read_take(p) for p in paths]
    full, first, again = (str(tmp_path / n) for n in ("full", "first", "again"))
    safety.build(Writer(), full, takes, log=lambda m: None)
    safety.build(Writer(), first, takes[:2], log=lambda m: None)
    safety.build(Writer(), again, takes, log=lambda m: None)
    assert safety.compare_datasets(full, again) == []
    assert safety.compare_datasets(full, first) == []  # the first two episodes' lines, rows and ranges
    with open(os.path.join(again, "meta", "teleop", "episodes.jsonl"), "a") as f:
        f.write(json.dumps({"episode": 3, "outcome": "contact"}) + "\n")
    assert safety.compare_datasets(full, again) == [os.path.join("meta", "teleop", "episodes.jsonl") + ": differs from line 5 (5 lines, the reference's 4)"]
    # The unavoidable ranges, one set for each stop time, are compared set by set over the first episodes.
    unav = os.path.join(full, "meta", "teleop", "unavoidable_ranges.json")
    sets = json.load(open(unav))
    assert sorted(sets["unavoidable_ranges"]) == ["fast", "median", "slow"] and "0" in sets["unavoidable_ranges"]["median"]
    sets["unavoidable_ranges"]["median"]["0"] = [[0, 1]]
    json.dump(sets, open(unav, "w"))
    assert safety.compare_datasets(full, first) == [os.path.join("meta", "teleop", "unavoidable_ranges.json") + ": differs"]
    # A reference built before docs/adr/0038 has no safety_ranges.json or sidecars: they are left out, and said so.
    os.remove(os.path.join(full, "safety_ranges.json"))
    notes = []
    assert safety.compare_datasets(full, first, notes) == [] and "predates docs/adr/0038" in notes[0]


def test_a_rebuild_takes_its_references_options_even_from_a_report_that_named_them_otherwise(scenarios, tmp_path, capsys):
    # just isaac-safety-build-check rebuilds a reference with the options its report.json records (teleop_lerobot
    # safety-options). A report from before 2026-11-23 says tableUnsafe (false: the table left out, --table-safe now).
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    from teleop_lerobot.__main__ import main

    takes = [read_take(p) for p in batch(scenarios)]
    built = dict(table_safe=True, geometry_dir="/nowhere", stop=None)  # as the builds before 2026-11-23 by default
    ref, again, counted = (str(tmp_path / n) for n in ("ref", "again", "counted"))
    safety.build(Writer(), ref, takes, log=lambda m: None, options=sidecars.Options(**built))
    want = ["--caption", "neutral", "--geometry-dir=/nowhere", "--table-safe", "--no-unavoidable"]
    assert safety.rebuild_options(ref) == want
    path = os.path.join(ref, "meta", "teleop", "report.json")
    report = json.load(open(path))
    report["options"] = {"caption": "neutral", "tableUnsafe": False, "geometryDir": "/nowhere", "stopModel": None}
    report["split"] = {"part": 1, "of": 2}  # a split build's part: a rebuild of its takes is one dataset
    json.dump(report, open(path, "w"), indent=1)
    assert safety.rebuild_options(ref) == want
    assert main(["safety-options", ref]) == 0 and capsys.readouterr().out.strip() == " ".join(want)
    safety.build(Writer(), again, takes, log=lambda m: None, options=sidecars.Options(**built))
    assert safety.compare_datasets(ref, again) == []  # the notes word for word, the options by their new names
    safety.build(Writer(), counted, takes, log=lambda m: None, options=sidecars.Options(geometry_dir="/nowhere"))
    diffs = safety.compare_datasets(ref, counted)  # today's defaults: episode 2's table brush, unavoidable ranges, the options
    assert "manual_safety_labels.json: differs (episodes [2])" in diffs and "safety_ranges.json: differs" in diffs
    assert os.path.join("meta", "teleop", "report.json") + ": differs" in diffs
    assert os.path.join("meta", "teleop", "unavoidable_ranges.json") + ": not in the reference" in diffs
    os.remove(path)  # a dataset from before docs/adr/0038: category captions, and its twin reported no table contacts
    assert safety.rebuild_options(ref) == ["--caption", "category"]

    def options_of(options):
        (tmp_path / "opts" / "meta" / "teleop").mkdir(parents=True, exist_ok=True)
        (tmp_path / "opts" / "meta" / "teleop" / "report.json").write_text(json.dumps({"options": options}))
        return safety.rebuild_options(str(tmp_path / "opts"))
    plain = ["--caption", "neutral", "--geometry-dir="]
    assert options_of({"caption": "neutral", "tableSafe": False, "geometryDir": None,
                       "stopModel": {"stopTimes": dict(sidecars.STOP_TIMES_S)}}) == plain  # today's defaults
    assert options_of({"caption": "neutral", "tableSafe": False, "geometryDir": None,
                       "stopModel": {"stopTimes": {"given": 0.35}}}) == plain + ["--stop-time-s=0.35"]
    assert options_of({"caption": "neutral", "tableUnsafe": True, "geometryDir": None,  # before 2026-11-23, both given
                       "stopModel": {"stopTimeS": 0.25}}) == plain + ["--stop-time-s=0.25"]


def test_an_old_takes_planned_events_and_odd_numbers_reach_the_lines_as_json_can_hold_them(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    se = ["SE", 1100 * MS, 1, 2, "touch", 2, 0.5, 0.7, float("nan"), 1.1]
    nan_pose = {**STANDING, 3: (float("nan"), -0.02, -0.17, UPRIGHT)}
    path = take(scenarios, "collide_and_recover", world([se, wc(1800, "0:5:2:nan")], poses=nan_pose, sizes="2:nan;3:1"))
    e = episode(path)
    assert e.line["events"] == [{"index": 1, "drawn": 2, "kind": "touch", "object": 2, "approachS": 0.1, "finalS": 0.5,
                                 "contactS": 0.7, "endS": None, "clearS": 1.1}]
    assert [o["id"] for o in e.scene["objects"]] == [2, 4, 5] and e.scene["objects"][0]["sizeFactor"] == 1.0
    assert e.contacts[0]["peakForceN"] == 0.0 and e.contacts[0]["open"]
    root = str(tmp_path / "ds")
    safety.build(Writer(), root, [read_take(path)], log=lambda m: None)
    for name in ("episodes.jsonl", "scenes.jsonl", "contacts.jsonl"):
        text = open(os.path.join(root, "meta", "teleop", name)).read()
        assert "NaN" not in text and "Infinity" not in text
        [json.loads(x) for x in text.splitlines()]


def test_safety_report_says_what_a_split_build_holds_and_finds_a_parted_group(scenarios, tmp_path, capsys):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    from teleop_lerobot.__main__ import main

    paths = batch(scenarios)
    out = str(tmp_path / "out" / "v2check")
    roots = safety.build_split(Writer(), out, [read_take(p) for p in paths], 2, log=lambda m: None)
    assert main(["safety-report", *roots]) == 0
    text = capsys.readouterr().out
    assert "[report] no faults in 2 dataset(s)" in text and text.count("[mix]") == 0 and "clearance in 2 of 2" not in text
    assert "replay groups: 1; one look in 1, one layout seed in 1, every edited replay's layout hash unlike its source's in 1" in text
    # Built apart, a group's source and replay land in two datasets: a fault.
    a, b = str(tmp_path / "a"), str(tmp_path / "b")
    safety.build(Writer(), a, [read_take(paths[0])], log=lambda m: None)
    safety.build(Writer(), b, [read_take(paths[1])], log=lambda m: None)
    assert main(["safety-report", a, b]) == 1
    text = capsys.readouterr().out
    assert "FAULT: replay group 3.1 is in 2 datasets: a (episode 0); b (episode 0)" in text and "FAULT: replay group 3.1 has" not in text
    # Two parts with one folder name, in two folders, are told apart by their paths.
    x, y = str(tmp_path / "x" / "part"), str(tmp_path / "y" / "part")
    safety.build(Writer(), x, [read_take(paths[0])], log=lambda m: None)
    safety.build(Writer(), y, [read_take(paths[1])], log=lambda m: None)
    assert main(["safety-report", x, y, x + "/"]) == 1
    text = capsys.readouterr().out
    assert f"FAULT: replay group 3.1 is in 2 datasets: {x} (episode 0); {y} (episode 0)" in text
    assert f"FAULT: {x} is given twice" in text


def test_safety_report_checks_a_train_val_test_set_at_once_and_faults_a_layout_or_a_seed_in_two(scenarios, tmp_path, capsys):
    # A replay group's takes share their commands and a layout's takes their scene: each stays inside one split part, or the
    # parts leak. Every dataset of a train, val and test set can be checked together.
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    from teleop_lerobot.__main__ import main

    paths = batch(scenarios)  # a source and its replay (seed 639309532987986191), then seed 11 (ELSEWHERE) and 12 (THIRD)
    train, val, test, leaky = (str(tmp_path / n) for n in ("train", "val", "test", "leaky"))
    safety.build(Writer(), train, [read_take(p) for p in paths[:2]], log=lambda m: None)
    safety.build(Writer(), val, [read_take(paths[2])], log=lambda m: None)
    safety.build(Writer(), test, [read_take(paths[3])], log=lambda m: None)
    assert main(["safety-report", train, val, test]) == 0
    text = capsys.readouterr().out
    assert "[report] across 3 datasets: each of 1 replay group(s), 4 layout hash(es) and 3 layout seed(s) in one dataset" in text
    assert "[report] train: unavoidable ranges (twin stop-latency 2026-11-23, provisional until the real arm is measured): " \
           "fast 0.17 s: 2 frames in 1 episode(s); median 0.245 s: 3 frames in 1 episode(s); slow 0.31 s: 4 frames in 1 episode(s)" in text
    # Val's layout again (the same objects where they stood, another seed), and test's seed again (the same draw settled
    # elsewhere: another hash).
    same_layout = take(scenarios, "mixed", world(seed=13, poses=ELSEWHERE), name="20261122T100400Z-take005")
    same_seed = take(scenarios, "mixed", world(seed=12, poses={**STANDING, 2: (0.22, 0.12, -0.15, UPRIGHT)}),
                     name="20261122T100500Z-take006")
    safety.build(Writer(), leaky, [read_take(same_layout), read_take(same_seed)], log=lambda m: None)
    assert main(["safety-report", train, val, test, leaky]) == 1
    text = capsys.readouterr().out
    layout = json.loads(open(os.path.join(val, "meta", "teleop", "episodes.jsonl")).readline())["layoutHash"]
    assert f"FAULT: layout hash {layout[:16]} is in 2 datasets: leaky (episode 0); val (episode 0)" in text
    assert f"FAULT: layout seed 12 of scene {SCENE_SHA[:12]} is in 2 datasets: leaky (episode 1); test (episode 0)" in text
    assert "across 4 datasets: 0 of 1 replay group(s), 1 of 5 layout hash(es) and 1 of 4 layout seed(s) in more than one" in text
    assert text.count("FAULT") == 2
    # Unavoidable ranges are frames the labels leave safe: one over an unsafe frame is a fault.
    path = os.path.join(train, "meta", "teleop", "unavoidable_ranges.json")
    unav = json.load(open(path))
    unav["unavoidable_ranges"]["median"]["0"] = [[16, 16]]  # inside episode 0's contact, frames 15-17
    json.dump(unav, open(path, "w"))
    assert main(["safety-report", train]) == 1
    assert "FAULT: train: unavoidable ranges (median) overlap unsafe frames in episode(s) [0]" in capsys.readouterr().out
