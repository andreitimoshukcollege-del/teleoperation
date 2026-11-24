"""Every drawn object carries its own mask id (docs/adr/0038 "A, masks"): the labels' parsing, the drawn pixels with no id,
the parked shelf out of every camera's sight (so the renderer never hides and redraws an object), the renderer's check at
each take's start, and, with pxr (Isaac's Python; skipped elsewhere), the labels on each object's own shapes through a
resize, a box turned into a mesh and a relabel."""

import glob
import math
import os
import re

import pytest

from conftest import REPO, ROOT
from teleop_isaac import scene as scene_module
from teleop_isaac.renderer import (OBJECT_BASE, SIGHT_MOUNTED_M, SIGHT_STILL_M, MaskCheck, look_at, intrinsics, mask_label_id,
                                   mask_table, objects_in_view, out_of_sight, project_centre, stable_mask, unlabelled_px)
from teleop_isaac.world import RESIZE_SCALES, IsaacWorld

SCENES = sorted(p for p in glob.glob(os.path.join(REPO, "core", "Scenes", "*.json")) if not p.endswith(".shapes.json"))


def test_a_label_isaac_joined_still_names_its_object():
    table = mask_table([37, 80])
    assert mask_label_id({"class": "object_37"}, table) == OBJECT_BASE + 37
    assert mask_label_id("object_80", table) == OBJECT_BASE + 80
    # A label inherited and its own joined, with or without a space; a scanned asset's own label beside the object's.
    assert mask_label_id({"class": "object_37,object_37"}, table) == OBJECT_BASE + 37
    assert mask_label_id({"class": "object_37, mug"}, table) == OBJECT_BASE + 37
    assert mask_label_id({"class": "clutter,object_80"}, table) == OBJECT_BASE + 80
    assert mask_label_id({"class": ["object_80"]}, table) == OBJECT_BASE + 80
    # Two objects in one label, nothing the table names, Isaac's reserved labels: no id.
    assert mask_label_id({"class": "object_37,object_80"}, table) == 0
    assert mask_label_id({"class": "mug"}, table) == 0
    assert mask_label_id({"class": "UNLABELLED"}, table) == 0
    assert mask_label_id({"class": "BACKGROUND"}, table) == 0
    assert mask_label_id({}, table) == 0 and mask_label_id(None, table) == 0


def test_a_joined_label_reaches_the_stable_mask():
    import numpy as np
    table = mask_table([37])
    seg = np.array([[0, 4, 5]], np.uint32)
    info = {"0": {"class": "BACKGROUND"}, "4": {"class": "object_37,object_37"}, "5": {"class": "arm"}}
    assert stable_mask(seg, info, table).tolist() == [[0, OBJECT_BASE + 37, 7]]


def test_the_pixels_drawn_with_no_id_are_counted_and_the_background_is_not():
    import numpy as np
    table = mask_table([37])
    seg = np.array([[0, 1, 1, 2], [3, 9, 4, 4]], np.uint32)
    info = {"0": {"class": "BACKGROUND"}, "1": {"class": "UNLABELLED"}, "2": {"class": "object_37"}, "3": {"class": "mug"},
            "4": "table"}
    # UNLABELLED (2), a label the table does not name (1) and an id with no entry at all (1).
    assert unlabelled_px(seg, info, table) == 4
    assert unlabelled_px(np.zeros((2, 2), np.uint32), info, table) == 0
    signed = np.array([[-1, 2]], np.int32)
    assert unlabelled_px(signed, info, table) == 1
    huge = np.array([[70000, 2, 0]], np.uint32)
    assert unlabelled_px(huge, info, table) == 1


def _sight(cameras):
    """The renderer's ``sight`` for a scene's cameras: a still camera where the file puts it, one on the arm near the base
    (it moves within the arm's reach, which SIGHT_MOUNTED_M covers)."""
    return [(tuple(c["position"]) if "position" in c else (0.0, 0.0, 0.5),
             float(c.get("far", 10.0)) + (SIGHT_MOUNTED_M if "mount" in c else SIGHT_STILL_M)) for c in cameras]


@pytest.mark.parametrize("path", SCENES, ids=lambda p: os.path.basename(p))
def test_every_parked_object_is_out_of_every_cameras_sight(path):
    """So the renderer leaves a parked object drawn on the shelf and never hides it: hiding an object and drawing it again
    when it was unparked lost its mask label for whole takes on the twins (2026-11-23/24)."""
    s = scene_module.load(path)
    cameras = s.extra.get("cameras") or []
    if not cameras:
        pytest.skip("no cameras")
    sight = _sight(cameras)
    world = IsaacWorld(s, "/robot")
    for i in range(len(s.objects)):
        for k, f in enumerate(RESIZE_SCALES):
            assert out_of_sight(s.parked_pose(i, f)[:3], sight), (s.objects[i].name, f)
            assert out_of_sight(world.parked(i, k)[:3], sight), (s.objects[i].name, k)
    # And what is on the table is in sight.
    assert not out_of_sight((0.3, 0.0, s.table.top + 0.05), sight)


def _camera(position, target, width=640, height=360, hfov=69.0, far=6.0):
    spec = {"id": "cam", "width": width, "height": height, "near": 0.02, "far": far, **intrinsics(width, height, hfov)}
    return spec, (*position, *look_at(position, target))


def test_an_objects_centre_projects_into_the_camera_that_looks_at_it():
    spec, pose = _camera((0.88, 0.5, 0.32), (0.3, 0.0, -0.18))
    u, v = project_centre((0.3, 0.0, -0.18), pose, spec)
    assert abs(u - 320.0) < 1e-6 and abs(v - 180.0) < 1e-6
    assert project_centre((2.0, 1.2, 0.5), pose, spec) is None  # behind the camera
    assert project_centre((-8.0, -5.0, -3.5), pose, spec) is None  # past the far plane
    assert project_centre((0.3, 3.0, -0.18), pose, spec) is None  # out of the image


def test_an_object_counts_as_in_view_unless_something_nearer_covers_its_centre():
    import numpy as np
    spec, pose = _camera((0.88, 0.5, 0.32), (0.3, 0.0, -0.18))
    spec2 = dict(spec, id="cam2")
    centre = (0.3, 0.0, -0.18)
    dist_mm = 1000.0 * math.dist(centre, pose[:3])
    poses = {37: (*centre, 1.0, 0.0, 0.0, 0.0), 80: (5.0, 5.0, 5.0, 1.0, 0.0, 0.0, 0.0)}
    assert objects_in_view(poses, [(spec, pose, None)]) == {37: ["cam"]}
    free = np.zeros((360, 640), np.uint16)  # nothing hit: nothing in front
    near = np.full((360, 640), int(dist_mm - 300), np.uint16)  # something 30 cm nearer
    own = np.full((360, 640), int(dist_mm - 30), np.uint16)  # its own near side, 3 cm in front of its centre
    assert objects_in_view(poses, [(spec, pose, free), (spec2, pose, near)], {37: 0.04}) == {37: ["cam"]}
    assert objects_in_view(poses, [(spec, pose, near)], {37: 0.04}) == {}
    assert objects_in_view(poses, [(spec, pose, own)], {37: 0.04}) == {37: ["cam"]}


def test_the_check_names_an_object_in_view_that_no_mask_holds():
    import numpy as np
    check = MaskCheck(frames=3)
    assert not check.active
    check.begin("take")
    labelled = np.zeros((4, 4), np.uint16)
    labelled[0, 0] = OBJECT_BASE + 62
    for _ in range(3):
        check.add("third-left", labelled, 5)
        check.add("wrist", np.zeros((4, 4), np.uint16), 1)
        done = check.frame_done()
    assert done and not check.active
    missing, unlabelled = check.result({62: ["third-left"], 80: ["third-left", "third-right"]})
    assert missing == [80] and unlabelled == {"third-left": 5, "wrist": 1}
    assert check.why == "take"


def test_a_check_skips_the_render_lag_and_a_new_reason_restarts_it():
    import numpy as np
    check = MaskCheck(frames=2)
    check.begin("unpark", skip=2)
    seen = np.full((2, 2), OBJECT_BASE + 5, np.uint16)
    check.add("c", seen, 0)  # still in the lag: not counted
    assert not check.frame_done() and check.counts == {} and check.skip == 1
    check.begin("resize")  # while one runs: the window starts over, both reasons kept, the lag still skipped
    assert check.why == "unpark+resize" and check.skip == 1 and check.left == 2
    check.add("c", seen, 0)
    assert not check.frame_done() and check.counts == {}
    check.add("c", seen, 0)
    assert not check.frame_done() and check.counts == {5: 4}
    check.add("c", seen, 0)
    assert check.frame_done() and not check.active
    assert check.result({5: ["c"]})[0] == []
    check.begin("take", fresh=True)
    assert check.why == "take" and check.skip == 0 and check.left == 2


def test_the_renderer_labels_each_objects_shapes_and_hides_no_object_it_need_not():
    """The renderer's wiring of the above: objects labelled on their shapes (label_object), a parked object hidden only when
    a camera might see it (out_of_sight), shown again it is relabelled, the lag measured with the objects moved away rather
    than hidden, and the check run at each take's start and recorded (ML)."""
    with open(os.path.join(ROOT, "teleop_isaac", "renderer.py"), encoding="utf-8") as f:
        src = f.read()
    run = src[src.index("def run("):]
    assert "label_object(stage, path, f\"object_{o.id}\")" in run
    assert not re.search(r'label\(stage\.GetPrimAtPath\(path\), f"object_', run)
    assert "shown = not parked or out_of_sight(pose[:3], sight)" in run
    assert "relabel(o.id, \"shown again\")" in run
    assert "objects_root.MakeInvisible()" not in run and "objects_away.Set(" in run
    assert "take_started(session.directory)" in run and 'session_record("ML"' in run


# --- with pxr (Isaac's Python) -----------------------------------------------------------------------------------------

def _pxr():
    pytest.importorskip("pxr")
    try:
        from pxr import UsdSemantics  # noqa: F401
    except ImportError:
        pytest.skip("this pxr has no UsdSemantics")


def _labels(stage, path):
    from pxr import UsdSemantics
    prim = stage.GetPrimAtPath(path)
    if not prim.HasAPI(UsdSemantics.LabelsAPI, "class"):
        return None
    v = UsdSemantics.LabelsAPI(prim, "class").GetLabelsAttr().Get()
    return None if v is None else list(v)


def _shapes_scene():
    """fr3-duo-bench: three boxes and a cylinder, which IsaacWorld builds with no asset files."""
    return scene_module.load(os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json"))


def test_each_objects_own_shapes_carry_its_label_through_a_box_turned_mesh_a_resize_and_a_relabel(monkeypatch):
    _pxr()
    import sys
    import types
    import pxr
    from pxr import Usd, UsdGeom
    if not hasattr(pxr, "PhysxSchema"):  # Isaac's Python without Kit: the renderer's copy (physics=False) never uses it
        stub = types.ModuleType("pxr.PhysxSchema")
        monkeypatch.setitem(sys.modules, "pxr.PhysxSchema", stub)
        monkeypatch.setattr(pxr, "PhysxSchema", stub, raising=False)
    from teleop_isaac.renderer import clear_labels, label_object, labels_lost, metric_box, set_labels

    s = _shapes_scene()
    stage = Usd.Stage.CreateInMemory()
    world = IsaacWorld(s, "/robot")
    world.build(stage, physics=False)
    stage.SetEditTarget(stage.GetSessionLayer())
    paths = {}
    for i, (o, path) in enumerate(zip(s.objects, world.object_paths)):
        if o.shape == "box" and i % 2 == 0:
            metric_box(stage, f"{path}/Shape", o.size)  # a textured box: a mesh before it is labelled
        paths[o.id] = label_object(stage, path, f"object_{o.id}")
        assert paths[o.id] == [f"{path}/Shape"], paths[o.id]  # its shape, the prim that draws
        assert _labels(stage, path) is None  # not its Xform: labels never nest
        assert _labels(stage, f"{path}/Shape") == [f"object_{o.id}"]
    for i, (o, path) in enumerate(zip(s.objects, world.object_paths)):
        if o.shape == "box" and i % 2 == 1:
            metric_box(stage, f"{path}/Shape", o.size)  # a box turned into a mesh after it was labelled keeps the label
        world.set_scale(i, 1.25)  # a resize
        img = UsdGeom.Imageable(stage.GetPrimAtPath(path))
        img.MakeInvisible()
        img.MakeVisible()
        assert labels_lost(stage, paths[o.id], f"object_{o.id}") == [], o.name
    o, path = s.objects[0], world.object_paths[0]
    clear_labels(stage, paths[o.id])  # the relabel's first half
    assert _labels(stage, f"{path}/Shape") is None and labels_lost(stage, paths[o.id], f"object_{o.id}") == paths[o.id]
    set_labels(stage, paths[o.id], [f"object_{o.id}"])
    assert labels_lost(stage, paths[o.id], f"object_{o.id}") == []


def test_a_scanned_assets_meshes_each_carry_the_label_and_an_old_label_on_the_xform_goes():
    _pxr()
    from pxr import Usd, UsdGeom, UsdSemantics
    from teleop_isaac.renderer import label_object

    asset = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(asset, "/Root")
    UsdGeom.Mesh.Define(asset, "/Root/handle")
    UsdGeom.Mesh.Define(asset, "/Root/Body/shell")
    UsdGeom.Scope.Define(asset, "/Root/Looks")
    asset.SetDefaultPrim(asset.GetPrimAtPath("/Root"))
    stage = Usd.Stage.CreateInMemory()
    obj = UsdGeom.Xform.Define(stage, "/World/Scene/Objects/o63_screwdrivers")
    shape = UsdGeom.Xform.Define(stage, "/World/Scene/Objects/o63_screwdrivers/Shape")
    shape.GetPrim().GetReferences().AddReference(asset.GetRootLayer().identifier)
    UsdSemantics.LabelsAPI.Apply(obj.GetPrim(), "class").CreateLabelsAttr().Set(["object_63"])  # the old way
    paths = label_object(stage, str(obj.GetPath()), "object_63")
    assert paths == ["/World/Scene/Objects/o63_screwdrivers/Shape/handle", "/World/Scene/Objects/o63_screwdrivers/Shape/Body/shell"]
    assert all(_labels(stage, p) == ["object_63"] for p in paths)
    assert _labels(stage, str(obj.GetPath())) is None

    # An instanced asset: its meshes cannot be edited, so the instance carries the label.
    inst = UsdGeom.Xform.Define(stage, "/World/Scene/Objects/o64/Shape")
    inst.GetPrim().GetReferences().AddReference(asset.GetRootLayer().identifier)
    inst.GetPrim().SetInstanceable(True)
    assert label_object(stage, "/World/Scene/Objects/o64", "object_64") == ["/World/Scene/Objects/o64/Shape"]
    assert _labels(stage, "/World/Scene/Objects/o64/Shape") == ["object_64"]
