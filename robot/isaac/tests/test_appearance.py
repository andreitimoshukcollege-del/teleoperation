"""Each take's look (docs/adr/0038 decision 5, workstream A) without Isaac: the scene file's appearance section, its draws (the
scanned objects' colours, the light, every camera's jitter), the poses the jitter gives, and the records. With pxr (Isaac's
Python), also the materials a colour changes, on a stage built here; elsewhere those tests are skipped."""

import colorsys
import json
import math
import os

import pytest

from conftest import REPO, ROOT
from teleop_isaac import appearance, kinematics
from teleop_isaac import scene as scene_module
from teleop_isaac.isaac_session import field
from teleop_isaac.renderer import compose, look_at

EXAMPLE = os.path.join(ROOT, "tests", "appearance-example.json")  # the section docs/adr/0038 "A, appearance" shows
BENCH = os.path.join(REPO, "core", "Scenes", "jetrover-bench.json")


def example():
    with open(EXAMPLE, encoding="utf-8") as f:
        return json.load(f)["appearance"]


def bench(tmp_path, section=None):
    """The JetRover bench, with ``section`` as its appearance section when given."""
    with open(BENCH, encoding="utf-8") as f:
        doc = json.load(f)
    if section is not None:
        doc["appearance"] = section
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(doc))
    return scene_module.load(str(path))


def angle_deg(q1, q2):
    return math.degrees(2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(q1, q2))))))


# --- the section ---------------------------------------------------------------------------------------------------------

def test_the_shipped_scenes_have_no_appearance_section_so_nothing_changes(tmp_path):
    assert appearance.spec(bench(tmp_path)) is None
    assert appearance.spec(scene_module.load(os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json"))) is None


def test_the_example_section_reads_every_part(tmp_path):
    s = appearance.spec(bench(tmp_path, example()))
    assert s.objects == appearance.ObjectsSpec((0.85, 1.1), (0.0, 0.25), 0.05)
    assert s.lighting.kelvin == (5500.0, 7500.0) and s.lighting.exposure == (108.0, 128.0)
    assert s.lighting.key == appearance.KeySpec((0.5, 2.5), (0.0, 360.0), (35.0, 75.0), (1.0, 6.0), (3500.0, 6500.0))
    assert s.room == appearance.JitterSpec(0.015, 2.0) and s.mounted == appearance.JitterSpec(0.003, 1.5)
    assert s.jitter_for(True) is s.mounted and s.jitter_for(False) is s.room


def test_every_part_is_optional():
    s = appearance.parse({})
    assert (s.objects, s.lighting, s.room, s.mounted) == (None, None, None, None)
    s = appearance.parse({"cameras": {"mounted": {"rotationDeg": 1.0}}})
    assert s.room is None and s.mounted == appearance.JitterSpec(0.0, 1.0)
    s = appearance.parse({"lighting": {"key": {"intensity": [1, 1], "azimuthDeg": [0, 90], "elevationDeg": [45, 45]}}})
    assert s.lighting.kelvin is None and s.lighting.key.angle_deg == (0.53, 0.53) and s.lighting.key.kelvin is None


@pytest.mark.parametrize("section, words", [
    ({"colours": {}}, "no field colours"),
    ({"objects": {"brightness": [0.85, 1.1], "hue": 0.1}}, "no field hue"),
    ({"objects": {"brightness": [1.2, 0.9]}}, "low above its high"),
    ({"objects": {"brightness": [0.2, 1.0]}}, "outside"),
    ({"objects": {"desaturation": [0.0, 1.5]}}, "outside"),
    ({"objects": {"tint": 0.5}}, "outside"),
    ({"objects": {"brightness": 1.0}}, "[low, high]"),
    ({"objects": {"brightness": [1.0, True]}}, "a number"),
    ({"lighting": {"colourTemperatureK": [900, 6500]}}, "outside"),
    ({"lighting": {"exposure": [100, 300]}}, "outside"),
    ({"lighting": {"key": {"azimuthDeg": [0, 90], "elevationDeg": [30, 60]}}}, "intensity [low, high] is missing"),
    ({"lighting": {"key": {"intensity": [1, 2], "azimuthDeg": [0, 400], "elevationDeg": [30, 60]}}}, "more than 360"),
    ({"lighting": {"key": {"intensity": [1, 2], "azimuthDeg": [0, 90], "elevationDeg": [30, 90]}}}, "outside"),
    ({"cameras": {"room": {"translationM": 0.2}}}, "outside"),
    ({"cameras": {"wrist": {"translationM": 0.001}}}, "no field wrist"),
    ([], "an object"),
])
def test_a_section_refuses_unknown_fields_and_values_out_of_bounds(section, words):
    with pytest.raises(ValueError) as e:
        appearance.parse(section)
    assert words in str(e.value)


def test_a_scene_with_a_bad_section_is_refused_when_its_appearance_is_read(tmp_path):
    with pytest.raises(ValueError, match="appearance"):
        appearance.spec(bench(tmp_path, {"objects": {"tint": 0.9}}))


# --- the scanned objects' colours ------------------------------------------------------------------------------------

def test_a_tint_is_a_cast_of_its_hue_at_unit_luminance_and_bounded_by_its_strength():
    assert appearance.tint_rgb(123.0, 0.0) == (1.0, 1.0, 1.0)
    for hue in range(0, 360, 15):
        for k in (0.01, 0.05, 0.1, 0.3):
            t = appearance.tint_rgb(hue, k)
            assert abs(sum(w * c for w, c in zip(appearance.LUMA, t)) - 1.0) < 2e-4
            assert all(1 - k - 1e-4 <= c <= 1 / (1 - k) + 1e-4 for c in t)
            if k >= 0.05:  # the cast's own hue is the tint's
                h = colorsys.rgb_to_hsv(*t)[0] * 360.0
                assert min(abs(h - hue), 360 - abs(h - hue)) < 1.0
    red = appearance.tint_rgb(0.0, 0.05)
    assert red[0] > red[1] == red[2]


def test_an_objects_colour_is_drawn_from_the_seed_the_change_and_its_id_alone_within_the_bounds():
    s = appearance.parse(example()).objects
    draws = {}
    for seed in (1, 7):
        for change in range(40):
            for oid in (31, 32, 41, 60):
                c = appearance.object_colour(s, seed, change, oid)
                assert c == appearance.object_colour(s, seed, change, oid)
                assert 0.85 <= c.brightness <= 1.1 and 0.0 <= c.desaturation <= 0.25
                assert abs(sum(w * x for w, x in zip(appearance.LUMA, c.tint)) - 1.0) < 2e-4
                assert all(0.95 - 1e-4 <= x <= 1 / 0.95 + 1e-4 for x in c.tint)
                assert c.gain() == tuple(round(x * c.brightness, 5) for x in c.tint)
                draws[seed, change, oid] = c
    assert len(set(draws.values())) == len(draws)  # every object, change and seed its own colour
    b = sorted(c.brightness for c in draws.values())
    assert b[0] < 0.87 and b[-1] > 1.08  # the draws fill the range
    assert max(c.desaturation for c in draws.values()) > 0.23


# --- the light -------------------------------------------------------------------------------------------------------

def test_the_light_is_drawn_within_its_ranges_and_each_quantity_from_its_own_stream():
    s = appearance.parse(example()).lighting
    seen = []
    for change in range(200):
        lt = appearance.lighting(s, 3, change, 1000.0)
        assert lt == appearance.lighting(s, 3, change, 1000.0)
        assert 5500 <= lt.kelvin <= 7500 and lt.kelvin == int(lt.kelvin)
        assert 108 <= lt.exposure <= 128
        k = lt.key
        assert 0 <= k.azimuth_deg < 360 and 35 <= k.elevation_deg <= 75 and 1 <= k.angle_deg <= 6
        assert 500 <= k.intensity <= 2500 and 3500 <= k.kelvin <= 6500
        seen.append(lt)
    assert len({lt.key.azimuth_deg for lt in seen}) > 180  # 0.01 degree steps: a few may meet
    assert min(lt.key.elevation_deg for lt in seen) < 37 and max(lt.key.elevation_deg for lt in seen) > 73
    # Without the exposure range, the colour temperature and the key are drawn as before.
    alone = appearance.LightingSpec(s.kelvin, None, s.key)
    for change in range(20):
        a, b = appearance.lighting(s, 3, change, 1000.0), appearance.lighting(alone, 3, change, 1000.0)
        assert (a.kelvin, a.key) == (b.kelvin, b.key) and b.exposure is None


def test_the_key_shines_from_its_azimuth_and_elevation_toward_the_robot():
    for az, el in ((0, 45), (90, 30), (215, 70), (300, 10)):
        q = appearance.key_orientation(az, el)
        travels = kinematics.q_rotate(q, (0.0, 0.0, -1.0))  # a distant light shines along its -z
        source = (math.cos(math.radians(el)) * math.cos(math.radians(az)),
                  math.cos(math.radians(el)) * math.sin(math.radians(az)), math.sin(math.radians(el)))
        assert all(abs(t + s) < 1e-9 for t, s in zip(travels, source))


def test_the_keys_intensity_multiplies_the_rooms_mean_radiance(tmp_path):
    assert appearance.key_reference(bench(tmp_path)) == 1000.0  # jetrover-bench's meanRadiance
    fr3 = scene_module.load(os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json"))
    assert appearance.key_reference(fr3) == 300.0  # no environment: its dome's intensity


# --- the cameras -------------------------------------------------------------------------------------------------------

def test_a_cameras_jitter_is_bounded_deterministic_and_none_without_a_spec():
    s = appearance.JitterSpec(0.015, 2.0)
    js = [appearance.camera_jitter(s, 5, change, cid) for change in range(300) for cid in ("third-left", "wrist")]
    assert js[0] == appearance.camera_jitter(s, 5, 0, "third-left")
    assert all(abs(v) <= 0.015 for j in js for v in (j.dx, j.dy, j.dz))
    assert all(abs(v) <= 2.0 for j in js for v in (j.pitch_deg, j.yaw_deg, j.roll_deg))
    assert max(abs(j.dx) for j in js) > 0.014 and max(abs(j.roll_deg) for j in js) > 1.9
    assert appearance.camera_jitter(None, 5, 0, "wrist") == appearance.NO_JITTER


def test_a_jitter_moves_the_camera_along_and_about_its_own_axes():
    pose = ((0.88, 0.51, 0.32), look_at((0.88, 0.51, 0.32), (0.08, 0.0, 0.07)))
    same = appearance.jittered(pose, appearance.NO_JITTER)
    assert all(abs(a - b) < 1e-12 for a, b in zip(same[0], pose[0])) and angle_deg(same[1], pose[1]) < 1e-6
    # 1 cm along its own x (right in its image).
    p, q = appearance.jittered(pose, appearance.CameraJitter(0.01, 0, 0, 0, 0, 0))
    right = kinematics.q_rotate(pose[1], (1.0, 0.0, 0.0))
    assert all(abs(a - b - 0.01 * r) < 1e-12 for a, b, r in zip(p, pose[0], right))
    # A 2 degree tilt turns the view up by 2 degrees, about the camera's x.
    p, q = appearance.jittered(pose, appearance.CameraJitter(0, 0, 0, 2.0, 0, 0))
    before, after = kinematics.q_rotate(pose[1], (0, 0, -1.0)), kinematics.q_rotate(q, (0, 0, -1.0))
    assert abs(math.degrees(math.acos(sum(a * b for a, b in zip(before, after)))) - 2.0) < 1e-9
    assert sum(a * b for a, b in zip(after, kinematics.q_rotate(pose[1], (0, 1.0, 0)))) > 0  # toward its up: a tilt up
    assert abs(sum(a * b for a, b in zip(after, right))) < 1e-12  # and not sideways
    # At most the bounds: 2 degrees about each axis turns it by at most 2 * sqrt(3) degrees.
    s = appearance.JitterSpec(0.015, 2.0)
    for change in range(200):
        j = appearance.camera_jitter(s, 1, change, "third-right")
        p, q = appearance.jittered(pose, j)
        assert angle_deg(q, pose[1]) <= 2.0 * math.sqrt(3) + 0.01
        assert math.dist(p, pose[0]) <= 0.015 * math.sqrt(3) + 1e-12


def test_a_wrist_cameras_jitter_in_its_mount_frame_is_the_same_turn_and_shift_in_the_room():
    """CM records the wrist camera's pose in its link's frame. Jittered there and posed by the link, it is where the camera
    jittered in its own frame after posing would be: a reader's forward kinematics and the renderer agree."""
    local = ((0.1, 0.0, 0.05), look_at((0.1, 0.0, 0.05), (0.26, 0.0, -0.01)))
    link = ((0.21, -0.05, 0.12), kinematics.q_rpy(0.3, -0.7, 1.1))
    j = appearance.CameraJitter(0.002, -0.003, 0.001, 1.2, -0.8, 1.5)
    a = compose(link, appearance.jittered(local, j))
    b = appearance.jittered(compose(link, local), j)
    assert math.dist(a[0], b[0]) < 1e-12 and angle_deg(a[1], b[1]) < 1e-6


# --- the records -------------------------------------------------------------------------------------------------------

def test_the_records_hold_what_was_drawn_in_the_documented_fields():
    colour = appearance.ObjectColour(0.95, 0.1, (1.0123, 0.99, 0.98))
    assert field(appearance.oa_items({41: ("omnipbr", appearance.NEUTRAL), 31: ("mtlx", colour)})) == \
        "31:mtlx:0.95:0.1:1.0123:0.99:0.98;41:omnipbr:1.0:0.0:1.0:1.0:1.0"
    jit = appearance.CameraJitter(0.001, -0.002, 0.0, 1.5, -0.25, 0.0)
    assert field(appearance.cj_items([("third-left", jit), ("wrist", appearance.NO_JITTER)])) == \
        "third-left:0.001:-0.002:0.0:1.5:-0.25:0.0;wrist:0.0:0.0:0.0:0.0:0.0:0.0"
    lt = appearance.Lighting(6200, 118.4, appearance.KeyLight(210.5, 55.0, 1500.0, 5000, 2.5))
    assert [field(f) for f in appearance.lt_fields(lt)] == ["6200", "118.4", "210.5:55.0:1500.0:5000:2.5"]
    assert [field(f) for f in appearance.lt_fields(appearance.Lighting(None, None, None))] == ["", "", ""]
    no_kelvin = appearance.Lighting(None, None, appearance.KeyLight(10.0, 40.0, 900.0, None, 1.0))
    assert field(appearance.lt_fields(no_kelvin)[2]) == "10.0:40.0:900.0::1.0"


def test_render_contexts_follow_kits_setting_or_its_default():
    assert appearance.render_contexts(None) == ("mdl", "mtlx", "")
    assert appearance.render_contexts(["mdl", ""]) == ("mdl", "")
    assert appearance.render_contexts("mdl") == ("mdl", "mtlx", "")


# --- the materials, with pxr (Isaac's Python) ------------------------------------------------------------------------

def _stage():
    pxr = pytest.importorskip("pxr")
    from pxr import Usd

    return pxr, Usd.Stage.CreateInMemory()


def _omnipbr(stage, path):
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("Materials/Textures/x_COLOR.png"))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat, sh


def _polyhaven(stage, path):
    """A Poly Haven model's material as its USD has it: a UsdPreviewSurface reading a texture, and a MaterialX surface whose
    base colour comes from a node graph."""
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    tex = UsdShade.Shader.Define(stage, path + "/Image_Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./textures/x_diff_1k.jpg"))
    rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    pv = UsdShade.Shader.Define(stage, path + "/Principled_BSDF")
    pv.CreateIdAttr("UsdPreviewSurface")
    pv.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    mat.CreateSurfaceOutput().ConnectToSource(pv.ConnectableAPI(), "surface")
    graph = UsdShade.NodeGraph.Define(stage, path + "/NodeGraphs")
    image = UsdShade.Shader.Define(stage, path + "/NodeGraphs/Image_Texture_Color")
    image.CreateIdAttr("ND_image_color3")
    out = graph.CreateOutput("node_006_out", Sdf.ValueTypeNames.Color3f)
    out.ConnectToSource(image.CreateOutput("out", Sdf.ValueTypeNames.Color3f))
    surf = UsdShade.Shader.Define(stage, path + "/Principled_BSDF_mtlx1")
    surf.CreateIdAttr("ND_open_pbr_surface_surfaceshader")
    surf.CreateInput("base_color", Sdf.ValueTypeNames.Color3f).ConnectToSource(out)
    mat.CreateSurfaceOutput("mtlx").ConnectToSource(surf.CreateOutput("out", Sdf.ValueTypeNames.Token))
    return mat, pv, surf, tex, out


def test_an_omnipbr_scan_takes_its_colour_through_its_own_albedo_inputs():
    _, stage = _stage()
    _, sh = _omnipbr(stage, "/World/Obj/Shape/Looks/OmniPBR")
    targets = appearance.prepare_colour(stage, "/World/Obj/Shape")
    assert [t.method for t in targets] == ["omnipbr"]
    method, applied = appearance.apply_colour(targets, appearance.ObjectColour(0.9, 0.2, (1.02, 0.99, 0.97)))
    assert method == "omnipbr" and applied.desaturation == 0.2
    assert tuple(round(v, 5) for v in sh.GetInput("diffuse_tint").Get()) == (1.02, 0.99, 0.97)
    assert abs(sh.GetInput("albedo_desaturation").Get() - 0.2) < 1e-7 and abs(sh.GetInput("albedo_brightness").Get() - 0.9) < 1e-7


def test_a_materialx_scan_gets_two_nodes_in_front_of_its_base_colour_and_nothing_else_changes():
    _, stage = _stage()
    mat, pv, surf, tex, graph_out = _polyhaven(stage, "/World/Obj/Shape/_materials/m")
    targets = appearance.prepare_colour(stage, "/World/Obj/Shape")
    assert [t.method for t in targets] == ["mtlx"]  # Kit takes mtlx before the universal UsdPreviewSurface
    base = surf.GetInput("base_color").GetConnectedSources()[0][0]
    assert str(base.source.GetPath()).endswith(appearance.MTLX_TINT) and base.sourceName == "out"
    tint = stage.GetPrimAtPath(f"/World/Obj/Shape/_materials/m/{appearance.MTLX_TINT}")
    sat = stage.GetPrimAtPath(f"/World/Obj/Shape/_materials/m/{appearance.MTLX_SATURATE}")
    from pxr import UsdShade
    tint, sat = UsdShade.Shader(tint), UsdShade.Shader(sat)
    assert tint.GetIdAttr().Get() == "ND_multiply_color3" and sat.GetIdAttr().Get() == "ND_saturate_color3"
    src = sat.GetInput("in").GetConnectedSources()[0][0]
    assert src.source.GetPath() == graph_out.GetAttr().GetPrim().GetPath() and src.sourceName == "node_006_out"
    assert tuple(tint.GetInput("in2").Get()) == (1.0, 1.0, 1.0) and sat.GetInput("amount").Get() == 1.0  # neutral until drawn
    method, applied = appearance.apply_colour(targets, appearance.ObjectColour(1.05, 0.1, (1.0, 1.0, 1.0)))
    assert method == "mtlx" and applied.desaturation == 0.1
    assert abs(sat.GetInput("amount").Get() - 0.9) < 1e-7
    assert tuple(round(v, 5) for v in tint.GetInput("in2").Get()) == (1.05, 1.05, 1.05)
    assert not tex.GetInput("scale")  # the universal network the renderer never draws is left alone
    assert pv.GetInput("diffuseColor").GetConnectedSources()[0][0].source.GetPath() == tex.GetPath()


def test_a_preview_only_scan_takes_its_colour_through_its_texture_scale_without_desaturation():
    _, stage = _stage()
    _, pv, surf, tex, _ = _polyhaven(stage, "/World/Obj/Shape/_materials/m")
    targets = appearance.prepare_colour(stage, "/World/Obj/Shape", contexts=("mdl", ""))  # a renderer without MaterialX
    assert [t.method for t in targets] == ["preview"]
    method, applied = appearance.apply_colour(targets, appearance.ObjectColour(0.9, 0.2, (1.0, 1.0, 1.0)))
    assert method == "preview" and applied.desaturation == 0.0
    assert tuple(round(v, 5) for v in tex.GetInput("scale").Get()) == (0.9, 0.9, 0.9, 1.0)
    assert not surf.GetInput("base_color").GetConnectedSources()[0][0].source.GetPath().name.startswith("Teleop")


def test_another_mdl_material_and_an_empty_shape_are_left_alone():
    _, stage = _stage()
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, "/World/Obj/Shape/Looks/Glass")
    sh = UsdShade.Shader.Define(stage, "/World/Obj/Shape/Looks/Glass/Shader")
    sh.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    assert appearance.prepare_colour(stage, "/World/Obj/Shape") == []
    assert appearance.prepare_colour(stage, "/World/Nothing") == []
    assert appearance.apply_colour([], appearance.NEUTRAL) == ("", appearance.NEUTRAL)
