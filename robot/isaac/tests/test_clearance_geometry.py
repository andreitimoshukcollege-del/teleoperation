"""The collision-geometry export for the clearance (docs/adr/0038 decision 4): the arm's boxes from the URDF, matching them
against the importer's, the cache key, and the hull pieces. pxr and CoACD run only on the GPU machine, not here."""

import json
import math
import os

import pytest

from conftest import REPO
from teleop_isaac import clearance_geometry as cg

URDF = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.urdf")
LINKS = ["base_link", "turntable", "proximal_link", "distal_link", "wrist_link", "gripper_link", "left_finger", "right_finger"]


def test_the_urdf_gives_every_link_its_boxes_and_the_pinned_base():
    r = cg.urdf_robot(URDF)
    assert r["links"] == LINKS and r["fixed"] == ["base_link"]
    count = {lk: sum(1 for b in r["boxes"] if b["link"] == lk) for lk in LINKS}
    assert count == {"base_link": 18, "turntable": 2, "proximal_link": 3, "distal_link": 3, "wrist_link": 5, "gripper_link": 2,
                     "left_finger": 1, "right_finger": 1}
    pad = next(b for b in r["boxes"] if b["link"] == "right_finger")
    assert pad["center"] == pytest.approx([0.147565, -0.02266, 0.003051])
    assert pad["half"] == pytest.approx([0.029787 / 2, 0.011285 / 2, 0.007074 / 2])
    assert [j["name"] for j in r["joints"]] == ["base_yaw", "proximal", "distal", "wrist", "roll", "gripper", "gripper_mirror"]
    for b in r["boxes"]:  # every box's axes a right-handed orthonormal frame
        a = b["axes"]
        for i in range(3):
            assert sum(v * v for v in a[i]) == pytest.approx(1.0)
            for j in range(i + 1, 3):
                assert sum(x * y for x, y in zip(a[i], a[j])) == pytest.approx(0.0, abs=1e-12)
        cross = [a[0][1] * a[1][2] - a[0][2] * a[1][1], a[0][2] * a[1][0] - a[0][0] * a[1][2], a[0][0] * a[1][1] - a[0][1] * a[1][0]]
        assert cross == pytest.approx(a[2])


def test_a_turned_box_follows_urdf_roll_pitch_yaw():
    r = cg.urdf_robot(URDF)
    g = next(b for b in r["boxes"] if b["name"] == "gripper.0")
    m = cg._rpy_matrix(0.3729971, -1.5053658, 1.2401309)
    assert g["axes"] == pytest.approx([[m[0][k], m[1][k], m[2][k]] for k in range(3)])
    # The importer's matrix for it (seen in the converted USD, 2026-11-22): its rows are the box's sides times its axes.
    usd_rows = [[0.00074, 0.00216, 0.03493], [-0.06213, -0.00259, 0.00148], [0.0028, -0.06502, 0.00397]]
    for k in range(3):
        side = 2 * g["half"][k]
        assert [side * v for v in g["axes"][k]] == pytest.approx(usd_rows[k], abs=2e-5)


def test_the_same_boxes_match_whatever_their_axes_order_and_a_moved_one_does_not():
    boxes = cg.urdf_robot(URDF)["boxes"]
    flipped = []
    for b in boxes:  # the same box, its x and y swapped and z flipped: the same eight corners
        a = b["axes"]
        flipped.append({**b, "axes": [a[1], a[0], [-v for v in a[2]]], "half": [b["half"][1], b["half"][0], b["half"][2]]})
    assert cg.same_boxes(boxes, flipped) == []
    moved = [dict(b) for b in boxes]
    moved[-1] = {**moved[-1], "center": [moved[-1]["center"][0] + 0.002, *moved[-1]["center"][1:]]}
    problems = cg.same_boxes(boxes, moved)  # 2 mm off: not the importer's box
    assert len(problems) == 1 and problems[0].startswith("right_finger: a box at")
    assert cg.same_boxes(boxes + [boxes[-1]], boxes) == ["right_finger: 2 boxes against 1"]


def test_a_collision_that_is_not_a_box_is_refused(tmp_path):
    with open(URDF, encoding="utf-8") as f:
        text = f.read().replace('<geometry><box size="0.029787 0.011285 0.007074"/></geometry>',
                                '<geometry><cylinder radius="0.005" length="0.03"/></geometry>')
    path = tmp_path / "jetrover.urdf"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError, match="right_finger"):
        cg.urdf_robot(str(path))


def test_an_asset_is_exported_again_when_it_or_the_method_changes():
    entry = {"name": "025_mug", "usd": "025_mug/025_mug.usd", "files": {"025_mug.usd": "ab" * 32}, "rotation": [0.7, 0.7, 0, 0],
             "size": [0.117, 0.093, 0.081], "centre": [0.001, 0.0, 0.04]}
    key = cg.asset_key(entry, "coacd 1.0.7 ...")
    assert key == cg.asset_key(json.loads(json.dumps(entry)), "coacd 1.0.7 ...")  # stable, whatever the key order
    assert key != cg.asset_key({**entry, "files": {"025_mug.usd": "cd" * 32}}, "coacd 1.0.7 ...")
    assert key != cg.asset_key({**entry, "centre": [0.0, 0.0, 0.04]}, "coacd 1.0.7 ...")
    assert key != cg.asset_key(entry, "hull: one convex hull of each UsdGeom.Mesh")


def test_the_scene_names_its_mesh_assets_once_each():
    assets = cg.scene_assets(os.path.join(REPO, "core", "Scenes", "jetrover-bench.json"))
    assert assets == sorted(set(assets)) and "polyhaven/cardboard_box_01" in assets and "ycb/025_mug" in assets
    assert all(a.count("/") == 1 for a in assets)


def test_coacd_runs_at_physx_budget():
    assert (cg.MAX_HULLS, cg.HULL_VERTICES) == (32, 64)  # PhysxConvexDecompositionCollisionAPI's defaults, omni.physx 110.4
    assert cg.COACD["max_convex_hull"] == 32 and cg.COACD["max_ch_vertex"] == 64 and cg.COACD["decimate"] is True
    assert cg.COACD["seed"] == 0


def test_a_hull_piece_has_outward_faces():
    pytest.importorskip("scipy")
    import numpy as np

    rng = np.random.default_rng(1)
    pts = rng.normal(size=(200, 3))
    piece = cg.hull_piece(pts)
    v, f = np.array(piece["vertices"]), np.array(piece["faces"])
    centre = v.mean(axis=0)
    for a, b, c in f:
        n = np.cross(v[b] - v[a], v[c] - v[a])
        assert n @ (v[a] - centre) > 0
