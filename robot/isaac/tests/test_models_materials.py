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


def _with_materials(s, **change):
    return dataclasses.replace(s, extra=dict(s.extra, materials=dict(s.extra["materials"], **change)))


DARK = ("black-resin", "black-laminate")


def test_the_table_and_the_bench_draw_a_look_each_take_and_the_bench_is_furniture_dark_more_often():
    from teleop_isaac.renderer import materials_spec
    m = materials_spec(_bench())
    assert m["classes"] == {"box": ["wood"], "cylinder": ["metal", "wood"]}
    looks, table, bench = m["looks"], m["surfaces"]["table"], m["surfaces"]["bench"]
    assert len(table) == len(looks) == 14 and set(bench) < set(table)  # the table may also be covered: a mat, cloth, paper
    assert set(bench) == set(DARK) | {"grey-laminate", "wood-kitchen", "wood-table", "wood-plywood", "wood-oak"}
    assert all(looks[lid]["weight"] == (2.0 if lid in DARK else 1.0) for lid in table)
    resin = looks["black-resin"]  # the real lab bench: nearly black, satin
    assert resin["texture"] == "smooth_concrete_floor" and max(resin["tint"]) <= 0.06 and resin["desaturation"] == 1.0
    assert resin["roughness"] == 0.3 and 0 < resin["roughnessTexture"] < 0.5
    grey = looks["grey-laminate"]  # the look the table had before
    assert grey["texture"] == "grey_oak_veneer_01" and grey["desaturation"] == 0.85 and grey["tint"] == [0.42] * 3
    assert looks["wood-oak"]["roughness"] is None and looks["wood-oak"]["roughnessTexture"] == 1.0  # its texture's own
    assert {looks[lid]["texture"] for lid in table} >= set(materials.PROCEDURAL)
    assert all(materials.TEXTURES[looks[lid]["texture"]] in ("surface", "wood") for lid in table)


def test_a_materials_section_names_known_classes_surfaces_looks_and_textures():
    from teleop_isaac.renderer import materials_spec
    s = _bench()
    looks = s.extra["materials"]["looks"]

    def look(**change):
        return dict(looks, **{"black-resin": dict(looks["black-resin"], **change)})

    for change, word in (
            ({"box": ["plastic"]}, "classes"), ({"box": ["surface"]}, "classes"),  # a shape never draws a table top
            ({"surfaces": {"ceiling": "oak_veneer_01"}}, "ceiling"), ({"surfaces": {"table": "nope"}}, "nope"),
            ({"surfaces": {"table": ["black-resin", "no-such-look"]}}, "no-such-look"),
            ({"surfaces": {"table": []}}, "empty"), ({"surfaces": {"table": ["wood-oak", "wood-oak"]}}, "twice"),
            ({"looks": look(texture="marble_99")}, "marble_99"),
            ({"looks": look(weight=0)}, "weight"), ({"looks": look(weight=-1)}, "weight"), ({"looks": look(weight="2")}, "weight"),
            ({"looks": look(weight=True)}, "weight"), ({"looks": look(weight=float("nan"))}, "weight"),
            ({"looks": look(roughness=1.5)}, "roughness"), ({"looks": look(roughness=-0.1)}, "roughness"),
            ({"looks": look(roughness="satin")}, "roughness"), ({"looks": look(roughnessTexture=2)}, "roughnessTexture"),
            ({"looks": look(tint=[0.1, 0.1])}, "tint"), ({"looks": look(tint=[0.1, 0.1, 1.5])}, "tint"),
            ({"looks": look(desaturation=2)}, "desaturation"), ({"looks": look(shine=1)}, "shine"),
            ({"looks": dict(looks, plywood={"texture": "plywood"})}, "texture's id"),
            ({"looks": dict(looks, **{"Black Resin": {"texture": "plywood"}})}, "lowercase"),
            ({"objects": {"no-such-object": ["wood-oak"]}}, "no-such-object"),
            ({"objects": {"mug": ["wood-oak"]}}, "keeps its own look"),  # a scan wears its own textures
            ({"tileScale": 0}, "tileScale"), ({"surfaceTint": [2, 0, 0]}, "surfaceTint")):
        with pytest.raises(ValueError, match=word):
            materials_spec(_with_materials(s, **change))
    assert materials_spec(dataclasses.replace(s, extra={k: v for k, v in s.extra.items() if k != "materials"})) is None


def test_a_surface_may_still_name_one_texture_which_wears_the_sections_desaturation_and_tint():
    from teleop_isaac.renderer import materials_spec
    m = materials_spec(_with_materials(_bench(), surfaces={"table": "grey_oak_veneer_01"}, surfaceDesaturation=0.85,
                                       surfaceTint=[0.42, 0.42, 0.42]))
    assert m["surfaces"] == {"table": ["grey_oak_veneer_01"]}
    look = m["looks"]["grey_oak_veneer_01"]
    assert look["texture"] == "grey_oak_veneer_01" and look["desaturation"] == 0.85 and look["tint"] == [0.42] * 3
    assert {materials.surface_look(m["looks"], m["surfaces"]["table"], 7, k, "table") for k in range(20)} == {"grey_oak_veneer_01"}


def test_a_surfaces_look_is_drawn_from_the_seed_the_change_and_the_surface_alone_by_weight():
    import math
    from teleop_isaac.renderer import materials_spec
    m = materials_spec(_bench())
    looks, table, bench = m["looks"], m["surfaces"]["table"], m["surfaces"]["bench"]
    assert materials.surface_look(looks, table, 7, 3, "table") == materials.surface_look(looks, table, 7, 3, "table")
    n = 6000
    on_table = [materials.surface_look(looks, table, 7, k, "table") for k in range(n)]
    on_bench = [materials.surface_look(looks, bench, 7, k, "bench") for k in range(n)]
    for ids, seen in ((table, on_table), (bench, on_bench)):
        total = sum(looks[lid]["weight"] for lid in ids)
        for lid in ids:
            p = looks[lid]["weight"] / total
            assert abs(seen.count(lid) / n - p) < 4 * math.sqrt(p * (1 - p) / n), lid
    dark_bench, dark_table = (sum(seen.count(lid) for lid in DARK) / n for seen in (on_bench, on_table))
    assert dark_bench > 0.4 and dark_table < 0.3  # 4 in 9 against 4 in 16
    assert sum(a == b for a, b in zip(on_table, on_bench)) < 0.15 * n  # each surface its own draw
    assert [materials.surface_look(looks, table, 8, k, "table") for k in range(50)] != on_table[:50]
    assert materials.pick_look({"a": {"weight": 3.0}, "b": {}}, ["a", "b"], "k") in ("a", "b")
    with pytest.raises(ValueError):
        materials.pick_look(looks, [], "k")


def test_a_simple_shape_may_wear_looks_by_name_drawn_apart_from_the_surfaces():
    from teleop_isaac.renderer import materials_spec
    s = _bench()
    phone = dataclasses.replace(s.objects[0], name="phone", shape="box", asset="")
    t = _with_materials(dataclasses.replace(s, objects=(phone,) + s.objects[1:]), objects={"phone": ["black-laminate", "grey-laminate"]})
    m = materials_spec(t)
    assert m["objects"] == {"phone": ["black-laminate", "grey-laminate"]}
    drawn = [materials.object_look(m["looks"], m["objects"]["phone"], 7, k, "phone") for k in range(60)]
    assert set(drawn) == {"black-laminate", "grey-laminate"}
    assert drawn != [materials.surface_look(m["looks"], m["objects"]["phone"], 7, k, "phone") for k in range(60)]


def test_the_table_top_textures_are_never_a_shapes_and_four_of_them_are_made_here():
    surface = {t for t, c in materials.TEXTURES.items() if c == "surface"}
    assert surface == {"smooth_concrete_floor", "laminate_floor_02", "kitchen_wood", "wood_table_001", "plywood", "rough_linen",
                       "denim_fabric_03", "hessian_230"} | set(materials.PROCEDURAL)
    assert all(materials.ASSET_ID.match(t) for t in materials.TEXTURES)
    tex = [{"id": t, "class": c} for t, c in materials.TEXTURES.items()]
    assert all(materials.pick(tex, ["wood", "metal"], 7, k, 31)["class"] != "surface" for k in range(300))


def _png_pixels(data: bytes):
    """A PNG as materials.png writes it (8-bit, filter 0 on every row), decoded: an (h, w, channels) uint8 array."""
    import struct
    import zlib

    import numpy as np
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    at, idat, head = 8, b"", None
    while at < len(data):
        (n,) = struct.unpack(">I", data[at:at + 4])
        tag, body = data[at + 4:at + 8], data[at + 8:at + 8 + n]
        assert zlib.crc32(tag + body) & 0xFFFFFFFF == struct.unpack(">I", data[at + 8 + n:at + 12 + n])[0]
        if tag == b"IHDR":
            head = struct.unpack(">IIBBBBB", body)
        elif tag == b"IDAT":
            idat += body
        at += 12 + n
    w, h, depth, colour_type = head[:4]
    ch = {0: 1, 2: 3}[colour_type]
    raw = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(h, 1 + w * ch)
    assert depth == 8 and (raw[:, 0] == 0).all()
    return raw[:, 1:].reshape(h, w, ch)


def test_the_textures_made_here_need_no_download_tile_seamlessly_and_are_kept_like_the_rest(tmp_path):
    import numpy as np

    def offline(url, limit):
        raise AssertionError(f"made here, never fetched: {url}")

    m = materials.fetch(str(tmp_path), list(materials.PROCEDURAL), download=offline, say=lambda *_: None)
    assert [e["id"] for e in m["textures"]] == sorted(materials.PROCEDURAL)
    for e in m["textures"]:
        assert e["class"] == "surface" and e["generator"] == materials.GENERATOR and e["license"].startswith("CC0 1.0")
        assert e["sizeM"] == materials.PROCEDURAL[e["id"]][0] and sorted(e["files"]) == ["diffuse.png", "normal.png", "roughness.png"]
        for name, channels in (("diffuse.png", 3), ("roughness.png", 1), ("normal.png", 3)):
            data = (tmp_path / e["id"] / name).read_bytes()
            a = _png_pixels(data).astype(float)
            assert materials.map_ok(name, data) and a.shape == (materials.PROCEDURAL_PX, materials.PROCEDURAL_PX, channels)
            # Seamless: across the wrap, neighbouring columns and rows differ no more than neighbours inside do.
            inner_x, inner_y = np.abs(np.diff(a, axis=1)).mean(), np.abs(np.diff(a, axis=0)).mean()
            assert np.abs(a[:, 0] - a[:, -1]).mean() <= 2 * inner_x + 0.5, (e["id"], name)
            assert np.abs(a[0] - a[-1]).mean() <= 2 * inner_y + 0.5, (e["id"], name)
            if name == "normal.png":
                assert a[..., 2].mean() > 240  # a little relief, not a bumpy surface
            if name == "roughness.png":
                assert 0.45 * 255 < a.mean() < 0.95 * 255
    assert materials.status(str(tmp_path))["bad"] == []
    # The same bytes again; one made by another generator is made again, the rest left alone.
    assert materials.procedural_maps("teleop_kraft_paper")["diffuse.png"] == (tmp_path / "teleop_kraft_paper" / "diffuse.png").read_bytes()
    m["textures"][0]["generator"] = "an older one"
    materials.save_manifest(str(tmp_path), m)
    said = []
    materials.fetch(str(tmp_path), list(materials.PROCEDURAL), download=offline, say=said.append)
    assert len(said) == 1 and m["textures"][0]["id"] in said[0]


def test_the_cutting_mat_is_green_with_a_line_every_centimetre_and_a_bold_one_every_five():
    n = 600  # half a millimetre a pixel
    a = _png_pixels(materials.procedural_maps("teleop_cutting_mat", n)["diffuse.png"]).astype(float)
    assert a[..., 1].mean() > a[..., 0].mean() + 20 and a[..., 1].mean() > a[..., 2].mean()
    column = a.mean(axis=(0, 2))  # a vertical line brightens its column

    def at(u: float) -> float:  # the brighter of the two columns either side of u metres
        k = int(round(u / (0.30 / n)))
        return max(column[(k - 1) % n], column[k % n])

    lines, between = [at(k * 0.01) for k in range(30)], [at(k * 0.01 + 0.005) for k in range(30)]
    bold, thin = [at(k * 0.05) for k in range(6)], [at(k * 0.01) for k in range(30) if k % 5]
    assert min(lines) > max(between) + 10 and min(bold) > max(thin)


def test_wear_finds_each_map_by_its_stem_whatever_its_format():
    from teleop_isaac.renderer import texture_maps
    polyhaven = {"files": {"diffuse.jpg": "a", "roughness.jpg": "b", "normal.png": "c"}}
    made = {"files": {"diffuse.png": "a", "roughness.png": "b", "normal.png": "c"}}
    assert texture_maps(polyhaven) == [("diffuse_texture", "diffuse.jpg", False), ("reflectionroughness_texture", "roughness.jpg", True),
                                       ("normalmap_texture", "normal.png", True)]
    assert [name for _, name, _ in texture_maps(made)] == ["diffuse.png", "roughness.png", "normal.png"]
    assert texture_maps({"files": {"diffuse.png": "a"}})[1:] == [("reflectionroughness_texture", "", True), ("normalmap_texture", "", True)]


def test_the_benchs_fixtures_are_real_models_bigger_than_the_robot_beside_it_within_reach():
    import math
    s = _bench()
    fixed = [(i, o) for i, o in enumerate(s.objects) if not o.movable]
    assert len(fixed) == 9 and all(o.shape == "mesh" and o.asset.startswith("polyhaven/") for _, o in fixed)
    assert all(o.size[2] >= 0.28 for _, o in fixed)
    for i, o in fixed:
        assert s.fits_on_table(i, *o.position[:2]), o.name     # beside the robot at its nominal spot
        assert not s.fits_on_table(i, 0.3, 0.0), o.name         # never in the work area in front
        assert not s.fits_on_table(i, 0.0, 0.0), o.name         # never on the robot
        near = math.hypot(*o.position[:2]) - o.footprint_radius
        assert near <= 0.24, f"{o.name}: its near side {near:.2f} m from the base, beyond the wrist's reach"
