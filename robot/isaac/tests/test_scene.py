"""The scene file as Isaac reads it, against Core's reader: the shared vectors, and Core's canonical parse of the shipped
scene (golden/scene-fr3-duo-bench.json, written by SceneDocumentTests)."""

import hashlib
import json
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
