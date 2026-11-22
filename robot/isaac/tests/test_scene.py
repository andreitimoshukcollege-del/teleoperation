"""The scene file as Isaac reads it, against Core's reader: the shared vectors, and Core's canonical parse of the shipped
scene (golden/scene-fr3-duo-bench.json, written by SceneDocumentTests)."""

import hashlib
import json
import math
import os

import pytest

from conftest import REPO, golden
from teleop_isaac import scene

SCENES = os.path.join(REPO, "core", "Scenes")


def test_every_vector_is_accepted_or_refused_as_core_does():
    with open(os.path.join(SCENES, "testdata", "vectors.json"), encoding="utf-8") as f:
        cases = json.load(f)["cases"]
    assert len(cases) >= 30
    for c in cases:
        if c["expect"] == "ok":
            assert scene.parse(c["scene"]).objects, c["name"]
        else:
            with pytest.raises(scene.SceneError) as e:
                scene.parse(c["scene"])
            assert str(e.value).startswith(c["expect"] + ":"), f"{c['name']}: expected '{c['expect']}:', got '{e.value}'"


def test_the_shipped_scene_parses_as_core_parses_it():
    g = golden("scene-fr3-duo-bench.json")
    path = os.path.join(SCENES, "fr3-duo-bench.json")
    with open(path, "rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == g["sha256"], "the scene changed: run `just gen-isaac-goldens`"
    s = scene.load(path)
    assert s.name == g["scene"]
    t = s.table
    assert [t.x_min, t.x_max, t.y_min, t.y_max, t.top] == pytest.approx(g["table"], abs=1e-12)
    assert [*s.region, s.clearance] == pytest.approx(g["region"], abs=1e-12)
    assert len(s.objects) == len(g["objects"])
    for o, e in zip(s.objects, g["objects"]):
        assert (o.id, o.name, o.shape) == (e["id"], e["name"], e["shape"])
        assert list(o.size) == pytest.approx(e["size"], abs=1e-12)
        assert list(o.nominal) == pytest.approx(e["nominal"], abs=1e-12)
        assert o.footprint_radius == pytest.approx(e["footprintRadius"], abs=1e-12)
        assert list(o.jitter) == pytest.approx(e["jitter"], abs=1e-12)
        assert (o.movable, o.mass) == (e["movable"], pytest.approx(e["mass"], abs=1e-12))


def test_the_isaac_only_sections_are_kept_for_the_renderer():
    s = scene.load(os.path.join(SCENES, "fr3-duo-bench.json"))
    cams = {c["id"]: c for c in s.extra["cameras"]}
    # docs/adr/0032 amendment 3: two third-person cameras and a camera on each wrist, no head camera.
    assert list(cams) == ["third-left", "third-right", "wrist-left", "wrist-right"]
    assert "mount" not in cams["third-left"]  # the render lag is measured with the first camera, which must stand still
    assert cams["wrist-left"]["mount"] == "left_fr3v2_hand" and cams["wrist-right"]["mount"] == "right_fr3v2_hand"
    # The headset's two displays take ports 6003 (right) and 6007 (left): the wrists, on their own sides.
    assert (cams["wrist-right"]["feedPort"], cams["wrist-left"]["feedPort"]) == (6003, 6007)
    assert {c["feedPort"] for c in cams.values()} == {6003, 6007, 6008, 6009}
    assert all(c["record"] == ["rgb", "depth", "mask"] and (c["width"], c["height"]) == (640, 360) for c in cams.values())
    assert s.extra["backdrops"]["change"] == "per-session"
    hidden = [x.id for x in s.surfaces if x.headset == "hide"]
    assert hidden == ["floor"] and next(x for x in s.surfaces if x.id == "floor").collides  # what a knocked-over object lands on


def test_the_rests_and_spots_are_cores():
    # docs/adr/0037 amendment 6: Core's canonical rests of the JetRover bench (golden/scene-rests-jetrover-bench.json, written by
    # SceneRestTests). Isaac must stand a layout's lying objects as RobotHost drew them, and accept its edge spots, or it
    # refuses them (OUTSIDE_REGION) or stands them up.
    g = golden("scene-rests-jetrover-bench.json")
    path = os.path.join(SCENES, "jetrover-bench.json")
    with open(path, "rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == g["sha256"], "the scene changed: run `just gen-isaac-goldens`"
    s = scene.load(path)
    assert [s.cluster_share, s.lying_share, s.edge_share] == pytest.approx(g["shares"], abs=1e-12)
    assert list(s.edge_regions) == pytest.approx(g["edgeRegions"], abs=1e-12)
    assert [o.name for o in s.objects] == [e["name"] for e in g["objects"]]
    for o, e in zip(s.objects, g["objects"]):
        for rest, (ex, ey, ez, radius, lie) in enumerate(e["rests"]):
            assert list(o.rest_extents(rest)) == pytest.approx([ex, ey, ez], abs=1e-12), (o.name, rest)
            assert o.footprint_radius_at(rest) == pytest.approx(radius, abs=1e-12), (o.name, rest)
            assert o.can_lie(rest) == bool(lie), (o.name, rest)
    for i, rest, x, y, yaw, *pose in g["poses"]:
        assert list(s.pose_at(int(i), x, y, yaw, 1.0, int(rest))) == pytest.approx(pose, abs=1e-12), (s.objects[int(i)].name, rest)
    for qw, qx, qy, qz, rest, yaw in g["restOf"]:
        got = scene.rest_of((qw, qx, qy, qz))
        assert got[0] == int(rest) and got[1] == pytest.approx(yaw, abs=1e-12), (qw, qx, qy, qz)
    assert sum(f for *_, f in g["fits"]) > 20 and sum(1 - f for *_, f in g["fits"]) > 20  # both answers are pinned
    for i, rest, x, y, fits in g["fits"]:
        assert s.fits_on_table(int(i), x, y, 1.0, int(rest)) == bool(fits), (s.objects[int(i)].name, rest, x, y)
    for a, ax, ay, ra, b, bx, by, rb, hit in g["overlaps"]:
        assert s.overlaps(int(a), ax, ay, int(b), bx, by, 1.0, 1.0, int(ra), int(rb)) == bool(hit), (a, b, ra, rb)


def test_a_rest_round_trips_through_its_pose_and_a_tilt_keeps_it():
    s = scene.load(os.path.join(SCENES, "jetrover-bench.json"))
    for i, o in enumerate(s.objects):
        for rest in range(scene.REST_COUNT):
            for yaw in (-3.0, -1.2, 0.0, 0.4, 2.9):
                p = s.pose_at(i, 0.25, -0.05, yaw, 1.0, rest)
                back = scene.rest_of(p)
                assert back[0] == rest and abs(math.remainder(back[1] - yaw, 2 * math.pi)) < 1e-9, (o.name, rest, yaw)
                assert abs(p[2] - (s.table.top + o.rest_extents(rest)[2] / 2)) < 1e-12  # its lowest face on the table
    # A settled object a little off its rest keeps it (25 degrees about a level axis), and its heading.
    tin = next(i for i, o in enumerate(s.objects) if o.name == "cleaner-tin")
    c, sn = math.cos(0.22), math.sin(0.22)
    tilt = (c, sn * 0.6, sn * 0.8, 0.0)
    for rest in range(scene.REST_COUNT):
        w, x, y, z = s.pose_at(tin, 0.3, 0.0, 1.1, 1.0, rest)[3:]
        a = tilt
        q = (a[0] * w - a[1] * x - a[2] * y - a[3] * z, a[0] * x + a[1] * w + a[2] * z - a[3] * y,
             a[0] * y - a[1] * z + a[2] * w + a[3] * x, a[0] * z + a[1] * y - a[2] * x + a[3] * w)
        back = scene.rest_of(q)
        assert back[0] == rest and abs(back[1] - 1.1) < 1e-9


def test_a_movable_object_may_stand_by_the_edge_a_fixed_one_not():
    s = scene.load(os.path.join(SCENES, "jetrover-bench.json"))
    can = next(i for i, o in enumerate(s.objects) if o.name == "tomato-soup-can")
    fixture = next(i for i, o in enumerate(s.objects) if not o.movable)
    x0, x1, y0, y1 = s.edge_regions[:4]
    # In the first rectangle, clear of the robot's keep-out lying too, and outside the fixtures' rectangles (x -0.15 to 0.06).
    x, y = -0.16, max(y0, s.robot_keep_out[3] + s.objects[can].footprint_radius_at(1) + 0.001)
    assert x0 <= x <= x1 and s.in_edge_regions(x, y) and s.fits_on_table(can, x, y) and s.fits_on_table(can, x, y, rest=1)
    assert not s.fits_on_table(fixture, x, y)
    assert not s.fits_on_table(can, x, y0 - 0.03)  # outside the rectangle and the region
    assert not s.fits_on_table(can, x0 + 0.005, y, rest=1)  # lying, its footprint would reach past the bench's back edge
