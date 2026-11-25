"""views_check.py (docs/adr/0038 "V, views") on takes made here: each view's table depth ray-cast from the pose it records,
so the check passes; and fails where the records and the frames disagree, as a renderer bug would make them."""

import json
import math
import os
import shutil

import numpy as np
import pytest

from conftest import REPO
import views_check
from teleop_isaac import scene as scene_module
from teleop_isaac import views
from teleop_isaac.isaac_session import Sidecar, field
from test_appearance_check import cast

MIXED = os.path.join(REPO, "core", "Scenes", "jetrover-mixed.json")
RECON = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "views", "recon.json")
FRAMES = 12


def _seed_with_a_moving_view(spec, geo):
    for seed in range(1, 200):
        drawn = views.draw(spec, geo, seed, 3)
        if any(v.motion is not None for v in drawn) and any(v.motion is None for v in drawn):
            return seed, drawn
    raise AssertionError("no seed draws one static and one moving view")


def make_take(root, shown_offset=0, drop_vp=False, wrong_vw=False):
    """A take with the shipped spec's two views. ``shown_offset``: a moving view's frames rendered from the pose that many
    frames later than its VP says (a wrong frame association). ``drop_vp``: no VP records. ``wrong_vw``: VW's azimuth off."""
    cv2 = pytest.importorskip("cv2")
    take = os.path.join(root, "take")
    os.makedirs(os.path.join(take, "isaac"))
    shutil.copy(MIXED, os.path.join(take, "scene.json"))
    scene = scene_module.load(MIXED)
    spec = views.load(RECON)
    geo = views.geometry(spec, scene)
    seed, drawn = _seed_with_a_moving_view(spec, geo)
    # Frames from where the moving view moves fastest in its loop, so a frame's offset shows in its depth.
    mv = next(v for v in drawn if v.motion is not None)
    loop = int(mv.motion.period * 30)
    k0 = max(range(loop), key=lambda k: math.dist(views.pose_at_frame(mv, k, 30)[0], views.pose_at_frame(mv, k + 1, 30)[0]))
    with open(os.path.join(take, "isaac", "views.json"), "wb") as f:
        f.write(views.views_json(spec, "recon.json"))
    lines = [["BK", 1, 3, seed, ""], ["VS", *views.vs_fields(spec, seed, 3)]]
    for v in drawn:
        fields = list(views.vw_fields(spec, v, 3, 1.0))
        if wrong_vw and v.index == 0:
            d = fields[19][0]
            fields[19] = [[d[0], round(d[1] + 1.0, 4), *d[2:]]]
        lines.append(["VW", *fields])
    sidecars = {f"{v.id}.{st}": Sidecar(os.path.join(take, "isaac", f"{v.id}.{st}.frames")) for v in drawn for st in spec.record}
    for n in range(FRAMES):
        parts, poses = [], []
        for v in drawn:
            pose = views.pose_at_frame(v, k0 + n, 30)
            render = views.pose_at_frame(v, k0 + n + shown_offset, 30) if v.motion is not None else pose
            depth, mask, _ = cast(scene, v.intrinsics, render, v.width, v.height)
            blobs = {"rgb": cv2.imencode(".jpg", np.zeros((v.height, v.width, 3), np.uint8))[1].tobytes(),
                     "depth": cv2.imencode(".png", depth)[1].tobytes(), "mask": cv2.imencode(".png", mask)[1].tobytes()}
            for st in spec.record:
                off, length = sidecars[f"{v.id}.{st}"].append(blobs[st])
                parts.append(f"{v.id}.{st}:{off}:{length}")
            if v.motion is not None:
                poses.append((v.id, pose))
        if poses and not drop_vp:
            lines.append(["VP", n, 1000 + n, views.vp_items(poses)])
        lines.append(["VF", 2000 + n, n, 1000 + n, "", ";".join(parts)])
    for s in sidecars.values():
        s.close()
    with open(os.path.join(take, "isaac.jlog"), "w", encoding="ascii") as f:
        f.write("JSES|1|isaac|1000000000|CLOCK_MONOTONIC|test\n")
        for row in lines:
            f.write("|".join(field(x) for x in row) + "\n")
    return take


def test_a_take_whose_frames_were_rendered_from_the_poses_it_records_passes(tmp_path):
    take = make_take(str(tmp_path))
    r = views_check.check_take(take, [])
    assert r["problems"] == [], r
    assert set(r["views"]) == {"view0", "view1"}
    for v in r["views"].values():
        assert v["depth_p99_mm"] < 1.0 and v["table_share"] > 0.05
    moving = [v for v in r["views"].values() if v["mode"] == "moving"]
    assert moving and moving[0]["association"]["frames"] > 0 and len(moving[0]["association"]["mean_mm_by_offset"]) == 3
    assert views_check.main([take]) == 0


def test_a_moving_view_rendered_a_frame_off_its_vp_is_caught(tmp_path):
    take = make_take(str(tmp_path), shown_offset=1)
    r = views_check.check_take(take, None)
    assert any("frame association" in p or "mm off" in p for p in r["problems"]), r


def test_missing_vp_or_a_vw_that_is_not_the_draw_fails(tmp_path):
    a = views_check.check_take(make_take(str(tmp_path / "a"), drop_vp=True), None)
    assert any("VP" in p for p in a["problems"])
    b = views_check.check_take(make_take(str(tmp_path / "b"), wrong_vw=True), None)
    assert any("not its draw" in p for p in b["problems"])


def test_a_take_without_views_is_a_problem_not_a_pass(tmp_path):
    take = tmp_path / "plain"
    (take / "isaac").mkdir(parents=True)
    (take / "isaac.jlog").write_text("JSES|1|isaac|1000000000|CLOCK_MONOTONIC|x\nBK|1|0|1|\n", encoding="ascii")
    assert views_check.check_take(str(take), None)["problems"]
    assert views_check.main([str(take)]) == 1


def test_the_sheet_is_written(tmp_path):
    cv2 = pytest.importorskip("cv2")
    take = make_take(str(tmp_path))
    out = str(tmp_path / "sheet.jpg")
    assert views_check.main([take, "--sheet", out]) == 0
    img = cv2.imread(out)
    assert img is not None and img.shape[1] == 640
