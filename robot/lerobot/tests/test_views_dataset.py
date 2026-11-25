"""The twin's view cameras and every camera's downsampled depth and masks in a safety dataset, opt-in (``safety-build --views``
and ``--depth-masks WxH``; docs/adr/0038 "V, views", "In the safety datasets"). A synthetic take whose depth and masks are
the table ray-cast through each camera's recorded pose: without the options every file is as before, with them the
``CM`` cameras' files are unchanged and the views, their scene entries, visibility rows and depth/mask arrays are there and
agree with the records."""

import json
import math
import os

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from conftest import finish, jetrover_take, write_jlog  # noqa: E402
from teleop_lerobot import depth_masks as D  # noqa: E402
from teleop_lerobot import safety, sidecars  # noqa: E402
from teleop_lerobot import views as VW  # noqa: E402
from teleop_lerobot.jlog import read_jlog  # noqa: E402
from teleop_lerobot.takes import TakeError, read_take  # noqa: E402
from test_clearance import geometry  # noqa: E402,F401  (the fixture: the URDF's robot and a 5 cm test cube)

TABLE = {"id": "table", "center": [0.336, -0.014, -0.2162], "size": [0.472, 1.6, 0.04], "collides": True}
TOP = TABLE["center"][2] + TABLE["size"][2] / 2
SCENE = {"format": "teleop-scene", "version": 2, "name": "synthetic", "robotProfile": "jetrover", "surfaces": [TABLE],
         "objects": []}
TARGET = (0.3, 0.0, TOP)
SHA = "cd" * 32


def pose(position, target=TARGET):
    return tuple(position) + tuple(sidecars.look_at(position, target))


# The room cameras (64x48) and the views (64x36), each at its own pose; view1 moves along +x by 2 mm a frame.
CAMS = {"third-left": (64, 48, 40.0, pose((0.9, 0.5, 0.3))), "wrist": (64, 48, 40.0, pose((0.9, -0.5, 0.3))),
        "view0": (64, 36, 36.0, pose((0.3, -0.9, 0.4)))}


def view1_pose(n, off=0.0):
    return pose((0.3 + 0.002 * n + off, 0.9, 0.5))


def render(width, height, f, p):
    """Depth (uint16 mm to the image plane) and mask of the table top seen from ``p``: id 10, and 107 on a 12 cm square at
    (0.3, 0), 20 mm nearer, as an object standing there would be (roughly); 0 elsewhere."""
    u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    ray = np.stack([(u - width / 2) / f, -(v - height / 2) / f, -np.ones_like(u)], axis=-1)
    world = ray @ D._quat_matrix(p[3:7]).T
    t = (TOP - p[2]) / world[..., 2]
    hit = np.asarray(p[:3]) + world * t[..., None]
    c, s = TABLE["center"], TABLE["size"]
    on = (t > 0) & (abs(hit[..., 0] - c[0]) < s[0] / 2) & (abs(hit[..., 1] - c[1]) < s[1] / 2)
    depth = np.where(on, np.round(t * 1000.0), 0)
    mask = np.where(on, 10, 0).astype(np.uint16)
    box = on & (abs(hit[..., 0] - 0.3) < 0.06) & (abs(hit[..., 1]) < 0.06)
    mask[box] = 107
    depth[box] -= 20  # an object standing on the table: its surface nearer than the table's
    return depth.astype(np.uint16), mask


def jpeg(width, height, n, k):
    return cv2.imencode(".jpg", np.full((height, width, 3), ((n * 9) % 256, 40 * k, 200 - 30 * k), np.uint8))[1].tobytes()


def cm_row(name):
    w, h, f, p = CAMS[name]
    return ["CM", name, w, h, f, f, w / 2, h / 2, 0.01, 6.0, *p, "", "rgb;depth;mask", "base"]


def vw_row(name):
    if name == "view0":
        w, h, f, p = CAMS[name]
        mode, motion = "static", ""
    else:
        (w, h, f), p = (64, 36, 36.0), view1_pose(0)
        mode, motion = "moving", "20.0:30.0:5.0:0.1:0.02:0.1:0.2:0.3:0.4"
    return ["VW", 3, name, w, h, f, f, w / 2, h / 2, 0.05, 6.0, *p, "rgb;depth;mask", mode,
            "75.0:20.0:35.0:1.3:1.5:0.3:0.0:-0.19:2", 1.1, motion, ""]


def views_take(root, name="20261125T100000Z-take001", at_s=0.0, render_off=0.0, views=True):
    """A take with the two room cameras and, with ``views``, view0 (static) and view1 (moving): every camera's depth and
    mask the table rendered through its recorded pose (view1's ``render_off`` m off its ``VP`` pose)."""
    def payload(cam, stream, n):
        if cam == "view1":
            (w, h, f), p = (64, 36, 36.0), view1_pose(n, render_off)
        else:
            w, h, f, p = CAMS[cam]
        if stream == "rgb":
            return jpeg(w, h, n, 3)
        depth, mask = render(w, h, f, p)
        return cv2.imencode(".png", depth if stream == "depth" else mask)[1].tobytes()

    return jetrover_take(root, name=name, scenario="mixed", cameras=("third-left", "wrist"), scene=SCENE, at_s=at_s,
                         image=lambda cam, n: jpeg(64, 48, n, 0 if cam == "wrist" else 1), cm=cm_row,
                         isaac_extra=[["VS", 3, 7, SHA, 2], vw_row("view0"), vw_row("view1")] if views else [],
                         views=("view0", "view1") if views else (), payload=payload, view_pose=lambda v, n: view1_pose(n))


class Writer:
    """A LeRobot stand-in (as test_sidecars'): meta/info.json with the episode count, and each take's cameras."""

    def __init__(self):
        self.cameras = []

    def append(self, root, repo, t):
        self.cameras.append([(c.name, c.width, c.height, c.view) for c in t.cameras])
        os.makedirs(os.path.join(root, "meta"), exist_ok=True)
        with open(os.path.join(root, "meta", "info.json"), "w") as f:
            json.dump({"total_episodes": len(self.cameras)}, f)
        return len(self.cameras) - 1


ON = sidecars.Options(views=True, depth_masks=(16, 9))


def test_a_take_read_with_views_has_them_after_its_cm_cameras_and_the_same_frames(scenarios):
    path = views_take(scenarios)
    plain, viewed = read_take(path), read_take(path, views=True)
    assert [c.name for c in plain.cameras] == ["third-left", "wrist"]
    assert [(c.name, c.width, c.height, c.view) for c in viewed.cameras] == [
        ("third-left", 64, 48, False), ("wrist", 64, 48, False), ("view0", 64, 36, True), ("view1", 64, 36, True)]
    assert len(viewed.frames) == len(plain.frames) > 0
    for a, b in zip(plain.frames, viewed.frames):
        assert (a.state, a.action, a.timing) == (b.state, b.action, b.timing)
        assert {k: b.parts[k] for k in a.parts} == a.parts and set(b.parts) == set(a.parts) | {"view0", "view1"}
    f = viewed.frames[5]
    assert viewed.jpeg(f, viewed.cameras[3]) == jpeg(64, 36, f.timing["frame"], 3)
    with pytest.raises(TakeError, match="no view cameras.*--views build needs every take"):
        read_take(views_take(scenarios, name="20261125T100100Z-take002", views=False), views=True)


def test_the_view_records_read_as_session_format_defines_them(scenarios):
    path = views_take(scenarios)
    vs = VW.read(read_jlog(os.path.join(path, "isaac.jlog"), "isaac"), path)
    assert (vs.change, vs.seed, vs.sha256, vs.count) == (3, 7, SHA, 2)
    v0, v1 = vs.cameras
    assert (v0.mode, v0.motion, v0.pose) == ("static", None, CAMS["view0"][3])
    assert v1.mode == "moving" and v1.motion["periodS"] == 20.0 and v1.draw["target"] == [0.3, 0.0, -0.19]
    assert v1.draw["attempts"] == 2 and v0.gain == 1.1 and v0.broken == ()
    assert VW.pose_at(vs, v1, 12) == pytest.approx(view1_pose(12)) and VW.pose_at(vs, v0, 12) == v0.pose
    with pytest.raises(VW.ViewsError, match="mode static with motion"):
        VW.parse_vw([str(x) for x in vw_row("view0")[1:22]] + ["1:2:3:4:5:6:7:8:9", ""])


def test_without_the_options_nothing_changes_and_with_them_only_the_view_files_are_added(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    paths = [views_take(scenarios), views_take(scenarios, name="20261125T100100Z-take002", at_s=10)]
    off, on, again = (str(tmp_path / n / "jetrover_v") for n in ("off", "on", "again"))
    w_off, w_on = Writer(), Writer()
    safety.build(w_off, off, [read_take(p) for p in paths], log=lambda m: None)
    safety.build(Writer(), again, [read_take(p) for p in paths], log=lambda m: None,
                 options=sidecars.Options(views=False, depth_masks=None))
    logged = []
    safety.build(w_on, on, [read_take(p, views=True) for p in paths], log=logged.append, options=ON)
    assert safety.compare_datasets(off, again) == []
    files = lambda root: sorted(os.path.relpath(os.path.join(d, n), root) for d, _, ns in os.walk(root) for n in ns)
    assert not any("depth_masks" in f for f in files(off))
    report = json.load(open(os.path.join(off, "meta", "teleop", "report.json")))
    assert "views" not in report["options"] and "depthMasks" not in report["options"]  # reported exactly as before
    assert [c[2:] for c in w_off.cameras[0]] == [(48, False)] * 2 and len(w_on.cameras[0]) == 4
    # With them: the files a build without them writes, each the same but the two that list the views (and
    # visibility.parquet, which has no rows here: the take has no object poses; its view rows are tested below).
    diffs = safety.compare_datasets(on, off)
    assert sorted(d.split(":")[0] for d in diffs) == [os.path.join("meta", "teleop", n) for n in ("report.json", "scenes.jsonl")]
    assert set(files(on)) - set(files(off)) == {os.path.join("meta", "teleop", "depth_masks.json")} | {
        os.path.join("meta", "teleop", "depth_masks", f"episode_{e:06d}.npz") for e in (0, 1)}
    a = [json.loads(x) for x in open(os.path.join(off, "meta", "teleop", "scenes.jsonl"))]
    b = [json.loads(x) for x in open(os.path.join(on, "meta", "teleop", "scenes.jsonl"))]
    assert [{k: v for k, v in x.items() if k != "views"} for x in b] == a
    assert json.load(open(os.path.join(on, "meta", "teleop", "report.json")))["options"]["depthMasks"] == [16, 9]
    assert safety.rebuild_options(on)[-2:] == ["--views", "--depth-masks=16x9"] and "--views" not in safety.rebuild_options(off)
    assert any(m.startswith("[depth-masks] 2 episode(s) at 16x9, cameras third-left, front, view0, view1") for m in logged)
    # A rebuild with them writes the same bytes, the depth and masks too.
    twice = str(tmp_path / "twice" / "jetrover_v")
    safety.build(Writer(), twice, [read_take(p, views=True) for p in paths], log=lambda m: None, options=ON)
    assert safety.compare_datasets(on, twice) == []


def test_the_scene_line_gives_each_view_as_its_vw_and_a_moving_ones_pose_at_every_frame(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    path = views_take(scenarios)
    take = read_take(path, views=True)
    root = str(tmp_path / "jetrover_v")
    safety.build(Writer(), root, [take], log=lambda m: None, options=ON)
    line = json.loads(open(os.path.join(root, "meta", "teleop", "scenes.jsonl")).readline())
    assert [c["name"] for c in line["cameras"]] == ["third-left", "front"]
    v0, v1 = line["views"]
    for entry, row in ((v0, vw_row("view0")), (v1, vw_row("view1"))):
        assert [entry[k] for k in ("name", "width", "height", "fx", "fy", "cx", "cy", "near", "far")] == row[2:11]
        assert entry["pose"] == pytest.approx(list(row[11:18])) and entry["poseFrame"] == "base" and entry["mount"] is None
        assert (entry["mode"], entry["specSha256"], entry["viewsChange"], entry["viewsSeed"]) == (row[19], SHA, 3, 7)
        assert entry["streams"] == ["rgb", "depth", "mask"] and entry["gain"] == 1.1 and entry["broken"] == []
        assert entry["draw"] == {"hfovDeg": 75.0, "azimuthDeg": 20.0, "elevationDeg": 35.0, "distanceM": 1.3, "rollDeg": 1.5,
                                 "target": [0.3, 0.0, -0.19], "attempts": 2}
    assert v0["framePoses"] is None and v0["motion"] is None and v1["motion"]["azAmpDeg"] == 30.0
    assert len(v1["framePoses"]) == len(take.frames)
    for f, p in zip(take.frames, v1["framePoses"]):
        assert p == pytest.approx(list(view1_pose(f.timing["frame"])))


def test_the_depth_and_masks_are_the_sampled_source_pixels_ids_kept_and_filled_frames_repeat(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    path = views_take(scenarios)
    take = read_take(path, views=True)
    root = str(tmp_path / "jetrover_v")
    safety.build(Writer(), root, [take], log=lambda m: None, options=ON)
    desc = json.load(open(os.path.join(root, D.DESCRIPTOR)))
    assert desc["size"] == [16, 9] and set(desc["cameras"]) == {"third-left", "front", "view0", "view1"}
    assert desc["cameras"]["view0"] == {"source": [64, 36], "rows": [4 * i + 2 for i in range(9)],
                                        "cols": [4 * j + 2 for j in range(16)]}
    assert desc["cameras"]["front"]["rows"] == [math.floor((i + 0.5) * 48 / 9) for i in range(9)]
    data = np.load(D.episode_path(root, 0))
    n = len(take.frames)
    assert data["source_frame"].tolist() == [f.timing["frame"] for f in take.frames]
    assert data["filled"].tolist() == [f.timing["filled"] for f in take.frames]
    for name, recorded in (("third-left", "third-left"), ("front", "wrist"), ("view0", "view0"), ("view1", "view1")):
        cam = desc["cameras"][name]
        for stream in ("depth", "mask"):
            arr = data[f"{name}.{stream}"]
            assert arr.shape == (n, 9, 16) and arr.dtype == np.uint16
        for i in (0, n // 2, n - 1):
            k = int(data["source_frame"][i])
            if recorded == "view1":
                (w, h, f), p = (64, 36, 36.0), view1_pose(k)
            else:
                w, h, f, p = CAMS[recorded]
            depth, mask = render(w, h, f, p)
            assert (data[f"{name}.depth"][i] == depth[np.ix_(cam["rows"], cam["cols"])]).all()
            assert (data[f"{name}.mask"][i] == mask[np.ix_(cam["rows"], cam["cols"])]).all()
    ids = {int(x) for name in desc["cameras"] for x in np.unique(data[f"{name}.mask"])}
    assert ids <= {0, 10, 107} and 107 in ids  # never a blend of two ids
    # The filled metric: frames.parquet's filled are the arrays' and the take's, views or not.
    import pandas as pd

    frames = pd.read_parquet(os.path.join(root, "meta", "teleop", "frames.parquet"))
    assert frames["filled"].tolist() == data["filled"].tolist() == [f.timing["filled"] for f in read_take(path).frames]


def test_the_table_projected_through_each_recorded_pose_lands_on_its_id_and_a_wrong_pose_is_caught(scenarios, tmp_path):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    good = str(tmp_path / "good")
    safety.build(Writer(), good, [read_take(views_take(scenarios), views=True)], log=lambda m: None,
                 options=sidecars.Options(views=True, depth_masks=(32, 18)))
    result = D.projection_check(good)
    assert result["problems"] == [] and set(result["cameras"]) == {"third-left", "front", "view0", "view1"}
    for name, c in result["cameras"].items():
        assert c["table.onPlane"] > 50 and c["tableIdShare"] == 1.0 and c["seeThroughShare"] == 0.0, name
    # view1 rendered 3 cm off the pose its VP records: its depth no longer lies on the table's plane through that pose.
    bad = str(tmp_path / "bad")
    safety.build(Writer(), bad, [read_take(views_take(scenarios, name="20261125T100100Z-take002", render_off=0.03), views=True)],
                 log=lambda m: None, options=sidecars.Options(views=True, depth_masks=(32, 18)))
    result = D.projection_check(bad)
    assert result["problems"] and all(p.startswith("view1:") for p in result["problems"])


def test_a_build_refuses_takes_whose_cameras_differ_or_that_record_no_depth(scenarios, tmp_path):
    a = read_take(views_take(scenarios), views=True)
    b = read_take(views_take(scenarios, name="20261125T100100Z-take002", at_s=10), views=True)
    b.cameras = b.cameras[:3]
    with pytest.raises(ValueError, match="same cameras and sizes"):
        safety.build_streams(Writer(), str(tmp_path / "x"), [a, b], log=lambda m: None, options=ON)
    assert not os.path.exists(str(tmp_path / "x"))
    real = read_take(jetrover_take(scenarios, name="20261125T100200Z-take003", source="robot", at_s=20))
    with pytest.raises(ValueError, match="--depth-masks needs takes of the twin"):
        safety.build_streams(Writer(), str(tmp_path / "y"), [real], log=lambda m: None,
                             options=sidecars.Options(depth_masks=(16, 9)))
    assert not os.path.exists(str(tmp_path / "y"))


def test_the_npz_is_written_the_same_twice_and_numpy_reads_it(tmp_path):
    arrays = {"a.depth": np.arange(24, dtype=np.uint16).reshape(2, 3, 4), "filled": np.array([True, False])}
    p, q = str(tmp_path / "p.npz"), str(tmp_path / "q.npz")
    D.write_npz(p, arrays)
    os.utime(p, (0, 0))
    D.write_npz(q, arrays)
    assert open(p, "rb").read() == open(q, "rb").read()
    got = np.load(q)
    assert got.files == ["a.depth", "filled"] and (got["a.depth"] == arrays["a.depth"]).all()
    assert D.parse_size("160x90") == (160, 90)
    with pytest.raises(ValueError):
        D.parse_size("160")
    assert D.sample_indices(640, 160)[:3].tolist() == [2, 6, 10] and D.sample_indices(1280, 160)[:2].tolist() == [4, 12]


def _rewrite_with_views(take):
    """masks_take's take with two views added: view0 the ``top`` camera exactly (its streams the same sidecar records), view1
    moving, at ``top``'s pose but 1 m to the side from frame 60 on."""
    import shutil

    path = os.path.join(take, "isaac.jlog")
    with open(path, encoding="ascii") as f:
        lines = f.read().splitlines()
    header, body = lines[0].split("|"), [l.split("|") for l in lines[1:-1]]
    top = next(r for r in body if r[0] == "CM" and r[1] == "top")
    rows = []
    for r in body:
        if r[0] == "VF":
            parts = dict(p.split(":", 1) for p in r[5].split(";"))
            extra = [f"{v}.{st}:{parts['top.' + st]}" for v in ("view0", "view1") for st in ("rgb", "mask")]
            n = int(r[2])
            x = float(top[10]) + (1.0 if n >= 60 else 0.0)
            rows.append(["VP", r[2], r[3], f"view1:{x}:" + ":".join(top[11:17])])
            r = r[:5] + [r[5] + ";" + ";".join(extra)]
        rows.append(r)
        if r[0] == "CM" and r[1] == "side":
            rows.append(["VS", 0, 5, SHA, 2])
            for v, mode in (("view0", "static"), ("view1", "moving")):
                rows.append(["VW", 0, v] + top[2:10] + top[10:17] + ["rgb;mask", mode, "60:0:89:1:0:0.3:0:0:1", 1.0,
                                                                     "10:1:1:0:0:0:0:0:0" if mode == "moving" else "", ""])
    for v in ("view0", "view1"):
        for st in ("rgb", "mask"):
            shutil.copy(os.path.join(take, "isaac", f"top.{st}.frames"), os.path.join(take, "isaac", f"{v}.{st}.frames"))
    write_jlog(path, "isaac", header[5], rows)
    finish(take)
    return take


def test_visibility_has_rows_for_each_view_at_its_own_pose(scenarios, geometry):  # noqa: F811
    from test_visibility import frame_rows, masks_take
    from teleop_lerobot import visibility as V

    path = _rewrite_with_views(masks_take(scenarios))
    take = read_take(path, views=True)
    assert [c.name for c in take.cameras] == ["top", "wrist", "side", "view0", "view1"]
    rec = sidecars.read_records(path)
    vis = V.compute(take, geometry, rec, {"wrist": "front"}, view_records=VW.read(rec.frames, path))
    plain = V.compute(read_take(path), geometry, rec, {"wrist": "front"})
    n = len(take.frames)
    keep = [i for i, c in enumerate(vis.columns["camera"]) if c not in ("view0", "view1")]
    nan = lambda xs: [None if isinstance(x, float) and math.isnan(x) else x for x in xs]  # noqa: E731
    assert ({k: nan([v[i] for i in keep]) for k, v in vis.columns.items()}
            == {k: nan(v) for k, v in plain.columns.items()})  # the CM cameras' rows as before
    top, v0, v1 = frame_rows(vis, "top", 31), frame_rows(vis, "view0", 31), frame_rows(vis, "view1", 31)
    assert len(v0) == len(v1) == n
    same = ("frame_index", "visible_px", "expected_px", "visible_fraction", "in_frustum", "reason")
    assert [{k: r[k] for k in same} for r in v0] == [{k: r[k] for k in same} for r in top]  # view0 is top exactly
    numbers = [f.timing["frame"] for f in take.frames]
    for r in v1:  # at top's pose until frame 60, then 1 m aside: the box out of its frustum
        if numbers[r["frame_index"]] >= 60:
            assert r["expected_px"] == 0 and r["in_frustum"] is False
        else:
            assert r["expected_px"] == 100 and r["in_frustum"] is True


def test_with_lerobot_the_three_cameras_videos_are_the_same_bytes_and_the_views_read_back(scenarios, tmp_path):
    pytest.importorskip("lerobot")
    from lerobot.datasets import LeRobotDataset
    from teleop_lerobot.writer import SAFETY_CRF, SAFETY_PRESET, LeRobotWriter

    paths = [views_take(scenarios), views_take(scenarios, name="20261125T100100Z-take002", at_s=10)]
    off, on = str(tmp_path / "off" / "jetrover_w"), str(tmp_path / "on" / "jetrover_w")
    writer = lambda: LeRobotWriter(crf=SAFETY_CRF, preset=SAFETY_PRESET)  # noqa: E731
    safety.build(writer(), off, [read_take(p) for p in paths], log=lambda m: None)
    safety.build(writer(), on, [read_take(p, views=True) for p in paths], log=lambda m: None, options=ON)
    videos = lambda root: sorted(os.path.relpath(os.path.join(d, n), root) for d, _, ns in os.walk(root)  # noqa: E731
                                 for n in ns if n.endswith(".mp4"))
    assert videos(off) and set(videos(off)) < set(videos(on))
    for rel in videos(off):
        assert open(os.path.join(off, rel), "rb").read() == open(os.path.join(on, rel), "rb").read(), rel
    assert {r.split(os.sep)[1] for r in set(videos(on)) - set(videos(off))} == {"observation.images.view0",
                                                                                "observation.images.view1"}
    ds = LeRobotDataset("local/jetrover-isaac-safety_jetrover_w", root=on)
    assert ds.meta.features["observation.images.view1"]["shape"] == [36, 64, 3] or \
        tuple(ds.meta.features["observation.images.view1"]["shape"]) == (36, 64, 3)
    item = ds[3]
    assert tuple(item["observation.images.view0"].shape) == (3, 36, 64)
    assert tuple(item["observation.images.front"].shape) == (3, 48, 64)
    diffs = {d.split(":")[0] for d in safety.compare_datasets(on, off)}
    assert diffs <= {os.path.join("meta", n) for n in ("info.json", "stats.json")} | {
        os.path.join("meta", "teleop", n) for n in ("report.json", "scenes.jsonl", "visibility.parquet")} | {
        d for d in diffs if d.startswith(os.path.join("meta", "episodes"))}, diffs


REAL = os.environ.get("TELEOP_VIEWS_DATASET")  # a dataset built --views --depth-masks from real twin takes, on the GPU machine


@pytest.mark.skipif(not REAL, reason="set TELEOP_VIEWS_DATASET to a safety dataset built from real takes with both options")
def test_a_real_views_dataset_passes_the_projection_check():
    result = D.projection_check(REAL, sidecars.GEOMETRY_DIR)
    assert result["problems"] == [] and any(name.startswith("view") for name in result["cameras"])
