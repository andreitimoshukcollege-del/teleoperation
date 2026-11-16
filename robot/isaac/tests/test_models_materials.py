"""Real fixtures and materials, offline (docs/adr/0037 amendment 5): what a Poly Haven model or texture download must be,
the texture paths a model's USD is pointed at, the scene's materials section, and the seeded pick."""

import dataclasses
import hashlib
import json
import os

import pytest

from conftest import REPO
from teleop_isaac import materials, models
from teleop_isaac import scene as scene_module

PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 16
JPG = b"\xff\xd8\xff" + b"\x00" * 16
EXR = b"\x76\x2f\x31\x01" + b"\x00" * 16
USD = b"PXR-USDC" + b"\x00" * 16


def test_a_model_keeps_its_usd_and_textures_under_textures_only():
    assert models.file_ok("x.usd", USD) and not models.file_ok("x.usd", b"<html>")
    assert models.file_ok("textures/x_diff_1k.jpg", JPG) and models.file_ok("textures/x_nor_gl_1k.exr", EXR)
    assert models.file_ok("textures/x_alpha_1k.png", PNG)
    assert not models.file_ok("textures/x_diff_1k.jpg", PNG)      # not what its name says
    assert not models.file_ok("../x.jpg", JPG) and not models.file_ok("textures/../../x.jpg", JPG)
    assert not models.file_ok("other/x.jpg", JPG)
    assert models.url_ok("https://dl.polyhaven.org/file/ph-assets/Models/x.usd")
    assert not models.url_ok("http://dl.polyhaven.org/x.usd") and not models.url_ok("https://evil.example/x.usd")


def _polyhaven(files):
    """A stand-in for the API and the download host: ``files`` maps a relative path to its bytes."""
    usd_name = next(k for k in files if k.endswith(".usd"))
    asset = usd_name[:-4]
    spec = lambda rel: {"url": f"https://dl.polyhaven.org/file/{rel}", "size": len(files[rel]), "md5": hashlib.md5(files[rel]).hexdigest()}
    listing = {"usd": {"1k": {"usd": dict(spec(usd_name), include={rel: spec(rel) for rel in files if rel != usd_name})}}}

    def download(url, limit):
        if url.endswith(f"/files/{asset}"):
            return json.dumps(listing).encode()
        if url.endswith(f"/info/{asset}"):
            return json.dumps({"name": asset.title(), "authors": {"Someone": "All"}, "categories": ["containers"]}).encode()
        return files[url.split("/file/", 1)[1]]

    return download


def test_a_models_texture_paths_are_pointed_at_its_own_folder_and_a_missing_one_refuses_it(tmp_path):
    files = {"jug.usd": USD, "textures/jug_diff_1k.jpg": JPG, "textures/jug_rough_1k.jpg": JPG}
    seen = {}

    def localize(usd, have):
        seen["have"] = sorted(have)
        return []

    m = models.fetch(str(tmp_path), ["jug"], download=_polyhaven(files), inspect=lambda p: ("y", (-0.1, 0.0, -0.1), (0.1, 0.3, 0.1)),
                     localize=localize, say=lambda *_: None)
    e = m["objects"][0]
    assert seen["have"] == ["jug_diff_1k.jpg", "jug_rough_1k.jpg"]
    assert e["up"] == "y" and e["size"] == pytest.approx([0.2, 0.2, 0.3]) and e["license"] == models.LICENSE
    assert (tmp_path / "jug" / "textures" / "jug_diff_1k.jpg").read_bytes() == JPG
    assert models.status(str(tmp_path))["bad"] == []
    m = models.fetch(str(tmp_path / "b"), ["jug"], download=_polyhaven(files), inspect=lambda p: ("z", (0,) * 3, (1,) * 3),
                     localize=lambda usd, have: ["jug_metal_1k.exr"], say=lambda *_: None)
    assert m["objects"] == []  # it names a texture it did not include


def test_a_texture_keeps_three_maps_checked_and_its_real_size(tmp_path):
    tid = "oak_veneer_01"
    maps = {"Diffuse": ("jpg", JPG), "Rough": ("jpg", JPG), "nor_gl": ("png", PNG)}
    listing = {k: {"1k": {fmt: {"url": f"https://dl.polyhaven.org/file/{tid}_{k}.{fmt}", "size": len(b), "md5": hashlib.md5(b).hexdigest()}}}
               for k, (fmt, b) in maps.items()}

    def download(url, limit):
        if url.endswith(f"/files/{tid}"):
            return json.dumps(listing).encode()
        if url.endswith(f"/info/{tid}"):
            return json.dumps({"name": "Oak", "dimensions": [2000, 1000]}).encode()
        key = url.rsplit("_", 1)[1].split(".")[0]
        return maps["nor_gl" if key == "gl" else key][1]

    m = materials.fetch(str(tmp_path), [tid], download=download, say=lambda *_: None)
    e = m["textures"][0]
    assert e["class"] == "wood" and e["sizeM"] == 2.0 and sorted(e["files"]) == ["diffuse.jpg", "normal.png", "roughness.jpg"]
    assert materials.status(str(tmp_path))["bad"] == []
    with pytest.raises(ValueError):
        materials.fetch(str(tmp_path), ["not_listed"], download=download, say=lambda *_: None)


def test_an_objects_material_is_drawn_from_the_seed_the_change_and_its_id_alone_within_its_classes():
    tex = [{"id": f"w{i}", "class": "wood"} for i in range(5)] + [{"id": f"m{i}", "class": "metal"} for i in range(3)]
    a = materials.pick(tex, ["wood"], 7, 3, 31)
    assert a == materials.pick(tex, ["wood"], 7, 3, 31) and a["class"] == "wood"
    assert materials.pick(tex, ["metal"], 7, 3, 31)["class"] == "metal"
    assert len({materials.pick(tex, ["wood", "metal"], 7, k, 31)["id"] for k in range(40)}) >= 6
    assert materials.pick(tex, ["plastic"], 7, 3, 31) is None


def _bench():
    return scene_module.load(os.path.join(REPO, "core", "Scenes", "jetrover-bench.json"))


def test_the_bench_wears_wood_and_metal_and_a_grey_laminate_table():
    from teleop_isaac.renderer import materials_spec
    m = materials_spec(_bench())
    assert m["classes"] == {"box": ["wood"], "cylinder": ["metal", "wood"]}
    assert m["surfaces"] == {"table": "grey_oak_veneer_01", "bench": "grey_oak_veneer_01"}
    assert 0 < m["surfaceDesaturation"] <= 1 and max(m["surfaceTint"]) < 0.6  # grey, not the light oak's own colour


def test_a_materials_section_names_known_classes_surfaces_and_textures():
    from teleop_isaac.renderer import materials_spec
    s = _bench()
    for change, word in (({"box": ["plastic"]}, "classes"), ({"surfaces": {"ceiling": "oak_veneer_01"}}, "ceiling"),
                         ({"surfaces": {"table": "nope"}}, "nope"), ({"tileScale": 0}, "tileScale"),
                         ({"surfaceTint": [2, 0, 0]}, "surfaceTint")):
        t = dataclasses.replace(s, extra=dict(s.extra, materials=dict(s.extra["materials"], **change)))
        with pytest.raises(ValueError, match=word):
            materials_spec(t)
    assert materials_spec(dataclasses.replace(s, extra={k: v for k, v in s.extra.items() if k != "materials"})) is None


def test_the_benchs_fixtures_are_real_models_bigger_than_the_robot_on_the_tables_wings():
    s = _bench()
    fixed = [(i, o) for i, o in enumerate(s.objects) if not o.movable]
    assert len(fixed) == 8 and all(o.shape == "mesh" and o.asset.startswith("polyhaven/") for _, o in fixed)
    assert all(o.size[2] >= 0.28 for _, o in fixed)
    for i, o in fixed:
        assert s.fits_on_table(i, *o.position[:2]), o.name     # each in a wing at its nominal spot
        assert not s.fits_on_table(i, 0.3, 0.0), o.name         # never in the middle of the table
