"""The YCB objects' rules, offline (docs/adr/0037 decision 4): what a download must be, how each object is stood up, and the
scene entry it becomes. The USD inspection (Isaac's pxr) is a stand-in here."""

import hashlib
import json
import os

import pytest

from teleop_isaac import scene as scene_module
from teleop_isaac import ycb


def test_only_usd_crates_and_png_or_jpeg_textures_under_materials_are_kept():
    assert ycb.usd_ok(b"PXR-USDC\x00\x08rest") and ycb.usd_ok(b"#usda 1.0\n")
    assert not ycb.usd_ok(b"<html>") and not ycb.usd_ok(b"PXR-USDC" + b"\x00" * ycb.MAX_BYTES)
    png = b"\x89PNG\r\n\x1a\n" + b"\x00" * 10
    assert ycb.texture_ok("Materials/Textures/025_mug_COLOR.png", png)
    assert not ycb.texture_ok("Materials/Textures/025_mug_COLOR.png", b"\xff\xd8\xff")  # a JPEG called .png
    assert not ycb.texture_ok("../../etc/passwd.png", png)
    assert not ycb.texture_ok("Materials/../../x.png", png)
    assert not ycb.texture_ok("Other/x.png", png)
    assert not ycb.texture_ok("Materials/x.exe", png)


def test_an_object_standing_on_its_y_is_turned_a_quarter_about_x():
    # The soup can's asset is 6.8 x 10.2 x 6.8 cm, tall along y: stood up it is 6.8 x 6.8 x 10.2, centre carried along.
    e = ycb.entry("005_tomato_soup_can", {}, (-0.034, -0.05, -0.034), (0.034, 0.052, 0.034))
    assert e["up"] == "y" and e["rotation"] == list(ycb.UPRIGHT["y"])
    assert e["size"] == pytest.approx([0.068, 0.068, 0.102])
    assert e["centre"] == pytest.approx([0.0, 0.0, 0.001])
    assert e["mass"] == 0.349 and e["material"] == "metal" and (e["staticFriction"], e["dynamicFriction"]) == ycb.FRICTION["metal"]
    # The quaternion is that rotation: (x, y, z) -> (x, -z, y).
    w, x, _, _ = ycb.UPRIGHT["y"]
    assert w * w + x * x == pytest.approx(1.0) and ycb.stand("y", (0, 1, 0)) == (0, 0, 1) and ycb.stand("y", (0, 0, 1)) == (0, -1, 0)


def test_an_object_standing_on_its_z_is_left_as_it_is():
    e = ycb.entry("061_foam_brick", {}, (-0.039, -0.0255, -0.0265), (0.039, 0.0255, 0.0265))
    assert e["up"] == "z" and e["rotation"] == [1.0, 0.0, 0.0, 0.0] and e["size"] == pytest.approx([0.078, 0.051, 0.053])


def test_every_object_has_a_mass_a_known_surface_and_an_up():
    for name, (mass, material, up) in ycb.OBJECTS.items():
        assert 0 < mass < scene_module.MAX_MASS and material in ycb.FRICTION and up in ycb.UPRIGHT, name


def test_a_scene_entry_is_a_movable_mesh_resting_on_the_table_that_the_scene_reader_takes(tmp_path):
    e = ycb.entry("025_mug", {}, (-0.0585, -0.0407, -0.0465), (0.0585, 0.0407, 0.0465))
    o = ycb.scene_object(e, 31, 0.3, 0.0, -0.1962, (0.8, 0.8, 0.8))
    assert o["shape"] == "mesh" and o["asset"] == "ycb/025_mug" and o["name"] == "mug" and o["kind"] == "movable"
    assert o["position"][2] == pytest.approx(-0.1962 + e["size"][2] / 2, abs=1e-4)
    from conftest import REPO
    doc = json.load(open(os.path.join(REPO, "core", "Scenes", "jetrover-bench.json")))
    doc["objects"] = [o]
    doc.pop("pool", None)
    s = scene_module.parse(doc)
    assert s.objects[0].asset == "ycb/025_mug" and s.objects[0].footprint_radius == pytest.approx(0.5 * (e["size"][0] ** 2 + e["size"][1] ** 2) ** 0.5)


def test_fetch_keeps_each_usd_and_its_textures_under_their_sha_and_resumes(tmp_path):
    usd = b"PXR-USDC" + b"\x01" * 32
    png = b"\x89PNG\r\n\x1a\n" + b"\x02" * 32
    asked = []

    def download(url):
        asked.append(url)
        return usd if url.endswith(".usd") else png

    state = {}

    def inspect(path):
        tex = os.path.join(os.path.dirname(path), "Materials", "Textures", "x_COLOR.png")
        missing = [] if os.path.exists(tex) else ["Materials/Textures/x_COLOR.png"]
        state[path] = state.get(path, 0) + 1
        return missing, (-0.01, -0.02, -0.03), (0.01, 0.02, 0.03)

    m = ycb.fetch(str(tmp_path), ["025_mug", "061_foam_brick"], download=download, inspect=inspect, say=lambda *_: None)
    assert [e["name"] for e in m["objects"]] == ["025_mug", "061_foam_brick"]
    mug = m["objects"][0]
    assert mug["files"] == {"025_mug.usd": hashlib.sha256(usd).hexdigest(), "Materials/Textures/x_COLOR.png": hashlib.sha256(png).hexdigest()}
    assert (tmp_path / "025_mug" / "Materials" / "Textures" / "x_COLOR.png").read_bytes() == png
    assert all(u.startswith(ycb.ASSETS) for u in asked)
    assert ycb.status(str(tmp_path))["bad"] == []
    asked.clear()
    ycb.fetch(str(tmp_path), ["025_mug"], download=download, inspect=inspect, say=lambda *_: None)
    assert asked == []  # kept already
    (tmp_path / "025_mug" / "025_mug.usd").write_bytes(b"tampered")
    assert ycb.status(str(tmp_path))["bad"] == [str(tmp_path / "025_mug" / "025_mug.usd")]


def test_fetch_refuses_a_name_it_does_not_know_and_a_texture_outside_materials(tmp_path):
    with pytest.raises(ValueError):
        ycb.fetch(str(tmp_path), ["../etc"], download=lambda u: b"", inspect=lambda p: ([], (0,) * 3, (0,) * 3), say=lambda *_: None)
    m = ycb.fetch(str(tmp_path), ["025_mug"], download=lambda u: b"PXR-USDC" + b"\x00" * 8,
                  inspect=lambda p: (["../secret.png"], (0,) * 3, (0.1,) * 3), say=lambda *_: None)
    assert m["objects"] == []


def test_the_bridge_reads_a_mesh_asset_from_its_librarys_manifest(tmp_path):
    from teleop_isaac.world import mesh_asset
    e = ycb.entry("025_mug", {}, (-0.0585, -0.0407, -0.0465), (0.0585, 0.0407, 0.0465))
    (tmp_path / "ycb").mkdir()
    (tmp_path / "ycb" / "manifest.json").write_text(json.dumps({"objects": [e]}))
    info = mesh_asset("ycb/025_mug", str(tmp_path))
    assert os.path.normpath(info["usd"]) == os.path.normpath(os.path.join(str(tmp_path), "ycb", "025_mug", "025_mug.usd"))
    assert info["rotation"] == tuple(ycb.UPRIGHT["y"]) and info["centre"] == tuple(e["centre"])
    with pytest.raises(KeyError):
        mesh_asset("ycb/011_banana", str(tmp_path))
    with pytest.raises(FileNotFoundError):
        mesh_asset("gso/thing", str(tmp_path))
