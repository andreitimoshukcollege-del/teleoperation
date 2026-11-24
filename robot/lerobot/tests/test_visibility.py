"""Visibility per frame, camera and object (docs/adr/0038 L): silhouettes with known pixel counts, near-plane clipping, and a
tiny take with masks end to end, through the build and safety-report."""

import json
import math
import os

import numpy as np
import pytest

from conftest import MS, Sidecar, finish, session_json, write_jlog
from teleop_lerobot import clearance as C
from teleop_lerobot import visibility as V
from teleop_lerobot.takes import read_take
from test_clearance import JOINTS, geometry  # noqa: F401  (the fixture: the URDF's robot and a 5 cm test cube)

cv2 = pytest.importorskip("cv2")


def view(name="top", width=64, height=48, f=100.0, near=0.01, rotation=np.eye(3), position=(0.0, 0.0, 0.0), mount=None):
    return V.View(name, name, width, height, f, f, width / 2, height / 2, near, np.asarray(rotation, float),
                  np.asarray(position, float), mount, True)


# --- rasterising -------------------------------------------------------------------------------------------------------------

def test_a_rectangle_covers_the_pixels_whose_centres_it_holds():
    rect = np.array([[10.0, 10.0], [20.0, 10.0], [20.0, 30.0], [10.0, 30.0]])
    assert V.silhouette([rect], 64, 48) == 10 * 20
    assert V.silhouette([rect[::-1]], 64, 48) == 200  # either winding
    assert V.silhouette([rect + 0.49], 64, 48) == 200  # centres 10.5 .. 19.5 still inside, 20.5 not yet
    assert V.silhouette([rect + 0.51], 64, 48) == 200  # 10.5 left behind, 20.5 taken in


def test_the_union_counts_an_overlap_once_and_the_image_clips():
    a = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    assert V.silhouette([a, a + [5.0, 0.0]], 64, 48) == 15 * 10
    count, canvas = V.silhouette([a, a + [5.0, 0.0]], 64, 48, canvas=True)
    assert count == canvas.sum() == 150 and canvas[:10, :15].all()
    assert V.silhouette([a - [5.0, 5.0]], 64, 48) == 25  # three quarters off the image
    assert V.silhouette([a + [100.0, 0.0]], 64, 48) == 0


def test_a_triangle_covers_about_its_area():
    tri = np.array([[3.0, 2.0], [60.0, 9.0], [21.0, 45.0]])
    area = 0.5 * abs(np.cross(tri[1] - tri[0], tri[2] - tri[0]))
    assert abs(V.silhouette([tri], 64, 48) - area) < 0.03 * area


# --- projecting -----------------------------------------------------------------------------------------------------------

def test_a_square_facing_the_camera_projects_to_its_pixels():
    # A 10 cm cube 1 m in front: its near face 0.95 m away is the silhouette, 10.5 px across: centres 27.5 .. 36.5.
    cube = C.box_piece([0.1, 0.1, 0.1]).vertices + [0.0, 0.0, -1.0]
    hull = V.project_hull(cube, view())
    assert V.silhouette([hull], 64, 48) == 100
    assert hull[:, 0].min() == pytest.approx(32 - 100 * 0.05 / 0.95)


def test_the_near_plane_clips_a_piece_reaching_behind_the_camera():
    v = view(near=0.1)
    long_box = C.box_piece([0.02, 0.02, 1.0]).vertices + [0.05, 0.0, 0.0]  # from 0.5 m in front to 0.5 m behind
    clipped = C.box_piece([0.02, 0.02, 0.4]).vertices + [0.05, 0.0, -0.3]  # its part 0.1 .. 0.5 m in front
    a = V.silhouette([V.project_hull(long_box, v)], 64, 48)
    assert a == V.silhouette([V.project_hull(clipped, v)], 64, 48) and a > 0
    assert V.project_hull(C.box_piece([0.02, 0.02, 0.02]).vertices + [0.0, 0.0, 0.5], v) is None  # wholly behind


# --- a tiny take with masks ---------------------------------------------------------------------------------------------

TOP = ["CM", "top", 64, 48, 100, 100, 32, 24, 0.01, 6, 0.3, 0.0, 1.0, 1, 0, 0, 0, "", "rgb;depth;mask", "base"]
# The wrist camera 0.5 m above wrist_link, looking down; at zero wrist_link is at (0.26, 0, 0.035), turned by nothing.
WRIST = ["CM", "wrist", 64, 48, 100, 100, 32, 24, 0.01, 6, 0.0, 0.0, 0.5, 1, 0, 0, 0, "wrist_link", "rgb;depth;mask", "mount"]
SIDE = ["CM", "side", 64, 48, 100, 100, 32, 24, 0.01, 6, 0.3, 0.0, 1.0, 1, 0, 0, 0, "", "rgb", "base"]


def masks_take(root, objects=None, wrist_mask=True, ids=True, wo=None):
    """A take whose arm stays at zero, seen from above by ``top`` (1 m over the box at x 0.3) and ``wrist``, and by ``side``
    with no mask. The box (id 31, 10 cm) stands on z 0; its mask id 131 covers its 100 pixels in ``top`` but for the 20 of a
    row the arm (7) covers in frames 30 on; ``wrist`` shows all 400 of its pixels."""
    take = os.path.join(root, "mixed", "20261124T100000Z-take001")
    os.makedirs(take)
    sid = session_json(take, "jetrover-session", 1, "isaac", "mixed")
    scene = {"format": "teleop-scene", "version": 2, "name": "tiny", "robotProfile": "jetrover",
             "surfaces": [{"id": "table", "center": [0.35, 0, -0.02], "size": [0.5, 1.0, 0.04]}],
             "objects": objects or [
                 {"id": 31, "name": "box", "shape": "box", "size": [0.1, 0.1, 0.1], "kind": "movable", "role": "obstacle"},
                 {"id": 33, "name": "parked", "shape": "box", "size": [0.05, 0.05, 0.05], "kind": "movable", "role": "obstacle"},
                 {"id": 34, "name": "cube", "shape": "mesh", "asset": "test/missing", "size": [0.05, 0.05, 0.05],
                  "kind": "movable", "role": "distractor"}]}
    with open(os.path.join(take, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(scene, f)
    rows = []
    for k in range(0, 41):
        t = (1000 + 50 * k) * MS
        rows.append(["JC", t, t, k, t, "A", "1:0.1:1"])
    for k in range(0, 116):
        t = (900 + 20 * k) * MS
        rows.append(["RO", t, ";".join(f"{m}:500:300" for m in (1, 2, 3, 4, 10, 5))])
        rows.append(["RI", t + 5 * MS, ";".join(f"{m}:1:500" for m in (1, 2, 3, 4, 5, 10))])
    rows.append(["WO", 100 * MS, 90 * MS, 1, wo or "31:0.3:0:0:1:0:0:0;33:17:-15:-4.97:1:0:0:0;34:0.5:0.3:0:1:0:0:0"])
    rows.sort(key=lambda r: int(r[1]))
    write_jlog(os.path.join(take, "robothost.jlog"), "robothost", sid, rows)
    cams = ["top", "wrist", "side"]
    sides = {f"{c}.rgb": Sidecar(os.path.join(take, "isaac", f"{c}.rgb.frames")) for c in cams}
    sides.update({f"{c}.mask": Sidecar(os.path.join(take, "isaac", f"{c}.mask.frames"))
                  for c in ("top", "wrist") if c == "top" or wrist_mask})
    frames = [TOP, WRIST, SIDE]
    if ids:
        frames.append(["ID", "7:arm;8:hand;10:table;131:object_31;133:object_33;134:object_34"])
    for n in range(0, 91):
        sensed = 500 * MS + round(n * 1e9 / 30)
        parts = []
        for stream, side in sides.items():
            if stream.endswith(".rgb"):
                payload = f"{stream}-{n}".encode()
            else:
                m = np.zeros((48, 64), np.uint16)
                if stream == "top.mask":
                    m[19:29, 27:37] = 131
                    if n >= 30:
                        m[19:21, 27:37] = 7
                else:
                    m[14:34, 30:50] = 131
                payload = cv2.imencode(".png", m)[1].tobytes()
            off, length = side.add(payload)
            parts.append(f"{stream}:{off}:{length}")
        frames.append(["VQ", n, sensed, ";".join(f"{j}:0" for j in JOINTS)])
        frames.append(["VF", sensed + 100 * MS, n, sensed, "", ";".join(parts)])
    for side in sides.values():
        side.close()
    write_jlog(os.path.join(take, "isaac.jlog"), "isaac", sid, frames)
    finish(take)
    return take


def frame_rows(vis, camera, object_id):
    c = vis.columns
    return [{k: c[k][i] for k in c} for i in range(len(c["frame_index"])) if c["camera"][i] == camera and c["object_id"][i] == object_id]


def test_a_tiny_take_end_to_end(scenarios, geometry):  # noqa: F811
    take = read_take(masks_take(scenarios))
    _, R, p = __import__("test_clearance").poses(geometry)
    assert np.allclose(R[4], np.eye(3)) and np.allclose(p[4], [0.26, 0, 0.035])  # wrist_link at zero, as the WRIST row assumes
    vis = V.compute(take, geometry, rename={"wrist": "front"})
    n = len(take.frames)
    assert set(vis.columns) == set(V.COLUMNS) - {"episode_index"}
    top = frame_rows(vis, "top", 31)
    assert len(top) == n and {r["expected_px"] for r in top} == {100} and all(r["in_frustum"] for r in top)
    late = [r for r in top if r["visible_px"] == 80]
    assert late and all(r["visible_fraction"] == pytest.approx(0.8) for r in late)
    assert {r["visible_px"] for r in top} == {80, 100} and all(r["reason"] is None for r in top)
    front = frame_rows(vis, "front", 31)  # the wrist, as the dataset names it: 0.485 m above the box top, 4 cm to its side
    assert {r["expected_px"] for r in front} == {400} and {r["visible_px"] for r in front} == {400}
    assert not frame_rows(vis, "top", 33)  # parked: not on the table
    side = frame_rows(vis, "side", 31)
    assert all(r["visible_px"] is None and r["expected_px"] == 100 and math.isnan(r["visible_fraction"])
               and r["reason"] == "the camera records no mask" for r in side)
    cube = frame_rows(vis, "top", 34)  # a mesh with no geometry in the cache: visible counted, expected unknown
    assert all(r["visible_px"] == 0 and r["expected_px"] is None and r["in_frustum"] is None
               and "no geometry" in r["reason"] and r["role"] == "distractor" for r in cube)


def test_what_cannot_be_had_is_null_with_a_reason(scenarios, geometry, tmp_path):  # noqa: F811
    take = read_take(masks_take(scenarios, wrist_mask=False, ids=False))
    vis = V.compute(take, geometry, rename={"wrist": "front"})
    front = frame_rows(vis, "front", 31)
    assert all(r["visible_px"] is None and r["expected_px"] == 400 and r["reason"] == "no wrist.mask.frames" for r in front)
    assert {r["visible_px"] for r in frame_rows(vis, "top", 31)} == {80, 100}  # no ID table: 100 + n all the same
    nothing = V.compute(take, None)
    assert all(r["expected_px"] is None and r["reason"] == "no geometry folder given" for r in frame_rows(nothing, "top", 31))
    assert all("no geometry folder" in r["reason"] for r in frame_rows(nothing, "wrist", 31))
    os.remove(os.path.join(take.path, "scene.json"))
    none = V.compute(take, geometry)
    assert none.columns["camera"] == [None] and none.columns["reason"] == ["no scene.json"]


# --- the build, the summary and the report --------------------------------------------------------------------------------

def test_the_summary_counts_cameras_per_obstacle_frame_and_contacts_seen_before():
    pd = pytest.importorskip("pandas")
    rows = []
    for frame in range(20):
        for cam, frac in (("front", 0.0), ("third-left", 0.9 if frame < 10 else 0.2), ("third-right", 0.6)):
            rows.append({"episode_index": 0, "frame_index": frame, "camera": cam, "object_id": 31, "role": "obstacle",
                         "visible_px": int(frac * 100), "expected_px": 100, "visible_fraction": frac, "in_frustum": True,
                         "reason": None})
        rows.append({"episode_index": 0, "frame_index": frame, "camera": "front", "object_id": 62, "role": "distractor",
                     "visible_px": 0, "expected_px": 0, "visible_fraction": float("nan"), "in_frustum": False, "reason": None})
    df = pd.DataFrame(rows)
    s = V.summary(df, [{"episode": 0, "object": 31, "kind": "object", "startFrame": 18},
                       {"episode": 0, "object": 62, "kind": "object", "startFrame": 5},
                       {"episode": 0, "object": 0, "kind": "table", "startFrame": 3},
                       {"episode": 0, "object": 31, "kind": "object", "startFrame": 0}], fps=15)
    assert s["obstacleFrames"] == 20  # the distractor is not an obstacle
    assert (s["seenInTwoOrMore"], s["seenInOne"], s["seenInNone"]) == (0.5, 0.5, 0.0)
    assert s["perCamera"]["front"] == {"seen": 0.0, "inFrustum": 1.0} and s["perCamera"]["third-left"]["seen"] == 0.5
    assert s["contacts"] == {"total": 3, "atStart": 1, "seenInSecondBefore": 1, "seenJustBefore": 1,
                             "shareSeenInSecondBefore": 0.5, "shareSeenJustBefore": 0.5}
    line = V.summary_line("ds", s)
    assert "2+ cameras 50.0%" in line and "1 of 2 (50.0%)" in line and "1 at the first frame left out" in line


def test_a_build_writes_visibility_and_safety_report_sums_it_up(scenarios, geometry, tmp_path, capsys):  # noqa: F811
    pd = pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    from teleop_lerobot import safety, sidecars
    from teleop_lerobot.__main__ import main
    from test_sidecars import Writer

    path = masks_take(scenarios)
    out = str(tmp_path / "ds")
    safety.build(Writer(), out, [read_take(path)], log=lambda m: None, options=sidecars.Options(geometry_dir=geometry))
    df = pd.read_parquet(os.path.join(out, "meta", "teleop", "visibility.parquet"))
    assert list(df.columns) == list(V.COLUMNS) and set(df["episode_index"]) == {0}
    assert str(df["visible_px"].dtype) == "Int32" and df.loc[df["camera"] == "side", "visible_px"].isna().all()
    report = json.load(open(os.path.join(out, "meta", "teleop", "report.json")))
    assert report["options"]["visibility"] is True
    assert main(["safety-report", out]) == 0
    lines = capsys.readouterr().out
    assert "ds: visibility of" in lines and "2+ cameras" in lines
    assert safety.rebuild_options(out)[-1] != "--no-visibility"

    off = str(tmp_path / "off")
    safety.build(Writer(), off, [read_take(path)], log=lambda m: None,
                 options=sidecars.Options(geometry_dir=geometry, visibility=False))
    assert not os.path.exists(os.path.join(off, "meta", "teleop", "visibility.parquet"))
    assert safety.rebuild_options(off)[-1] == "--no-visibility"
    assert main(["safety-report", off]) == 0 and "off: no visibility" in capsys.readouterr().out
