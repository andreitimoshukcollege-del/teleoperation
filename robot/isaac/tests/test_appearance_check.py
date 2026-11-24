"""appearance_check.py's checks (docs/adr/0038 "A, appearance"), on a table ray-cast here: the twin check of 2026-11-23 found
CM right and the checker wrong, twice. The table's mask covers its sides, and a camera turned down by its jitter sees the front
one in a corner of its image, which the checker took for the top (4.8-21.3 mm off at p99 in 10 of 20 takes). And it measured
turns as twice the arc cosine of a dot product, which reads CM's 7-decimal quaternions 0.01-0.04 degrees off."""

import json
import math
import os

import numpy as np

from conftest import REPO
import appearance_check as ac
from teleop_isaac import appearance, kinematics
from teleop_isaac import scene as scene_module
from teleop_isaac.renderer import depth_mm, intrinsics

BENCH = os.path.join(REPO, "core", "Scenes", "jetrover-bench.json")
# Take 017's draw for third-left: tilted down 1.96 degrees, moved 14.8 mm down and 10.9 mm forward. It brought the table's front
# face into the image's bottom left corner.
TAKE_017 = appearance.CameraJitter(-0.0029, -0.0148, -0.0109, -1.96, 0.74, 0.77)


def bench():
    return scene_module.load(BENCH)


def camera(scene, cid):
    c = next(c for c in scene.extra["cameras"] if c["id"] == cid)
    return c, intrinsics(int(c["width"]), int(c["height"]), float(c["hfovDeg"]))


def cast(scene, cam, pose, width=640, height=360):
    """Depth (uint16 mm, distance to the image plane) and mask (10 on the table) of the table box seen from ``pose``, by casting
    every pixel's ray at its top and its four sides; and which pixels hit a side."""
    s = next(s for s in scene.surfaces if s.id == "table")
    lo = [c - z / 2 for c, z in zip(s.center, s.size)]
    hi = [c + z / 2 for c, z in zip(s.center, s.size)]
    uu, vv = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    rays = np.stack([(uu - cam["cx"]) / cam["fx"], -(vv - cam["cy"]) / cam["fy"], -np.ones_like(uu)]).reshape(3, -1)
    d = np.array(ac.quat_matrix(*pose[1])) @ rays  # a ray's parameter is its depth: its camera z is -1
    o = np.array(pose[0])
    best = np.full(d.shape[1], np.inf)
    side = np.zeros(d.shape[1], bool)
    for axis in range(3):
        for plane in (lo[axis], hi[axis]):
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (plane - o[axis]) / d[axis]
            p = o[:, None] + t * d
            inside = np.ones_like(t, bool)
            for other in range(3):
                if other != axis:
                    inside &= (p[other] >= lo[other] - 1e-9) & (p[other] <= hi[other] + 1e-9)
            hit = (t > 0) & inside & (t < best)
            best = np.where(hit, t, best)
            side = np.where(hit, axis != 2, side)
    found = np.isfinite(best)
    depth = depth_mm(np.where(found, best, np.inf)).reshape(height, width)
    mask = np.where(found, ac.TABLE_ID, 0).astype(np.uint16).reshape(height, width)
    return depth, mask, (side & found).reshape(height, width)


def test_the_tables_front_face_in_a_jittered_view_is_not_taken_for_its_top():
    scene = bench()
    c, cam = camera(scene, "third-left")
    pose = appearance.jittered(ac.scene_pose(c), TAKE_017)
    depth, mask, side = cast(scene, cam, pose)
    assert side.sum() > 300  # the front face is in view, as it was in take 017
    top, *footprint = ac.table_top(scene)
    err = np.abs(ac.table_heights(depth, mask, cam, pose, footprint, sample=0) - top)
    assert len(err) > 20000 and np.percentile(err, 99) < 0.6e-3  # through the pose rendered: flat, to the depth's 1 mm steps
    everything = np.abs(ac.table_heights(depth, mask, cam, pose, sample=0) - top)
    assert np.percentile(everything, 99) > 3e-3  # every table pixel, its face too: the false failure of the twin check


def test_the_table_check_still_fails_a_pose_that_is_not_the_one_rendered():
    scene = bench()
    c, cam = camera(scene, "third-left")
    pose = appearance.jittered(ac.scene_pose(c), TAKE_017)
    depth, mask, _ = cast(scene, cam, pose)
    top, *footprint = ac.table_top(scene)
    wrong = {
        "the scene file's pose (the jitter ignored)": ac.scene_pose(c),
        "turned 0.5 degrees more": appearance.jittered(pose, appearance.CameraJitter(0, 0, 0, 0.5, 0, 0)),
        "1 cm lower": appearance.jittered(pose, appearance.CameraJitter(0, -0.01, 0, 0, 0, 0)),
        "the jitter's turn in the room's frame": (pose[0], kinematics.q_mul(TAKE_017.rotation(), ac.scene_pose(c)[1])),
    }
    for what, p in wrong.items():
        err = np.abs(ac.table_heights(depth, mask, cam, p, footprint, sample=0) - top)
        assert np.percentile(err, 99) > 3e-3, what


def test_cm_written_to_seven_decimals_is_within_the_checks_bounds_of_the_pose_given():
    """The renderer records each camera's pose moved by its jitter, to 7 decimals; the checker compares it with the scene file's
    pose moved by CJ. Over many draws it is within 0.01 mm and 0.001 degrees: the arc cosine read up to 0.05 degrees here."""
    scene = bench()
    spec = appearance.parse(json.load(open(os.path.join(REPO, "robot", "isaac", "tests", "appearance-example.json")))["appearance"])
    worst = [0.0, 0.0]
    for c in scene.extra["cameras"]:
        for change in range(100):
            j = appearance.camera_jitter(spec.jitter_for(bool(c.get("mount"))), 9, change, c["id"])
            p, q = appearance.jittered(ac.scene_pose(c), j)
            cm = (tuple(round(v, 7) for v in p), tuple(round(v, 7) for v in q))  # the preamble's rounding
            mm, deg = ac.cm_offset(cm, c, j)
            worst = [max(worst[0], mm), max(worst[1], deg)]
    assert worst[0] < ac.CM_MM and worst[1] < ac.CM_DEG
    assert worst[1] > 0  # the rounding is there, and measured


def test_the_table_top_is_the_tables_footprint_inside_its_edges():
    scene = bench()
    s = next(s for s in scene.surfaces if s.id == "table")
    top, x0, x1, y0, y1 = ac.table_top(scene)
    assert top == s.top
    assert math.isclose(x1, s.center[0] + s.size[0] / 2 - ac.TOP_EDGE_M) and math.isclose(y0, s.center[1] - s.size[1] / 2 + ac.TOP_EDGE_M)
