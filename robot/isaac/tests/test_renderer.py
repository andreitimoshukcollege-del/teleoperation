"""The renderer's parts that need no Isaac: motors to URDF joints, the lag estimate, the cameras' geometry, the stamps."""

import math

import pytest

from conftest import profile_path
from teleop_isaac import camwire, kinematics, profile
from teleop_isaac.renderer import (MotorMap, Stamps, backdrop_quads, estimate_lag, intrinsics, look_at, mount_camera_orientation,
                                   compose, mounted_camera_local, relative_pose, screen_corners)


def rotate(q, v):
    return kinematics.q_rotate(q, v)


def test_motors_become_urdf_joint_values_and_hands_become_finger_travel():
    p = profile.load(profile_path())
    m = MotorMap(p)
    arm = p.arms[0]
    g = arm.gripper
    values = m.values([(arm.joints[3].motor_id, -2.0), (g.motor_id, 0.5)])
    assert values[arm.joints[3].name] == -2.0
    second = g.name[:-1] + "2"
    assert values[g.name] == values[second] == 0.5 * g.finger_travel
    assert m.values([(250, 1.0)]) == {}


def test_the_lag_is_the_most_common_sample_and_says_how_many_agree():
    assert estimate_lag([2, 2, 2, 1]) == (2, 0.75)
    assert estimate_lag([]) == (None, 0.0)


def test_adr_0032s_intrinsics():
    k = intrinsics(640, 360, 69.0)
    assert abs(k["fx"] - 465.6) < 0.1 and k["cx"] == 320 and k["cy"] == 180


def test_a_look_at_camera_looks_at_its_target():
    pos, target = (1.15, 0.75, 0.60), (0.45, 0.0, 0.05)
    q = look_at(pos, target)
    forward = rotate(q, (0.0, 0.0, -1.0))  # a USD camera looks along its -z
    d = [t - p for t, p in zip(target, pos)]
    n = math.sqrt(sum(c * c for c in d))
    assert all(abs(f - c / n) < 1e-9 for f, c in zip(forward, d))
    assert rotate(q, (0.0, 1.0, 0.0))[2] > 0  # up is up


def test_a_mounted_camera_looks_along_its_mounts_x():
    q = mount_camera_orientation()
    assert all(abs(a - b) < 1e-9 for a, b in zip(rotate(q, (0.0, 0.0, -1.0)), (1.0, 0.0, 0.0)))
    assert all(abs(a - b) < 1e-9 for a, b in zip(rotate(q, (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0)))


def test_an_image_is_stamped_with_the_state_posed_lag_frames_before():
    s = Stamps()
    assert s.shown(2) == camwire.UNSET
    for t in (10, 20, 30, 40):
        s.posed(t)
    assert s.shown(0) == 40 and s.shown(2) == 20 and s.shown(4) == camwire.UNSET


def test_masks_label_each_links_visual_by_side_and_part_and_nothing_that_nests():
    from teleop_isaac.renderer import mask_class
    assert mask_class("left_fr3v2_link3_visual") == "left_arm"
    assert mask_class("right_fr3v2_hand_visual") == "right_hand"
    assert mask_class("left_fr3v2_leftfinger_visual") == "left_hand"
    assert mask_class("franka_head_1") == "head"
    assert mask_class("fr3_duo_mount_mounting_point_visual") == "mount"
    assert mask_class("left_fr3v2_link3") is None  # a link nests the next one: never labelled itself


def test_a_frames_segmentation_becomes_stable_ids_and_depth_millimetres():
    import numpy as np
    from teleop_isaac.renderer import OBJECT_BASE, depth_mm, mask_table, stable_mask
    table = mask_table([1, 4])
    seg = np.array([[0, 7], [9, 3]], np.uint32)
    info = {"7": {"class": "left_hand"}, "9": {"class": "object_4"}, "3": {"class": "something_else"}, "0": {"class": "BACKGROUND"}}
    assert stable_mask(seg, info, table).tolist() == [[0, 2], [OBJECT_BASE + 4, 0]]
    assert depth_mm(np.array([0.5, float("inf"), float("nan"), 70.0, 0.0])).tolist() == [500, 0, 0, 0, 0]


def test_a_screen_reads_left_to_right_from_the_side_it_faces():
    # The P4 screen beyond the table, facing the robot: as it was built before screens had a normal.
    c = screen_corners((1.30, 0.0, -0.10), (2.40, 1.00), (-1, 0))
    assert [tuple(round(v, 9) for v in p) for p in c] == [(1.30, 1.2, -0.6), (1.30, -1.2, -0.6), (1.30, -1.2, 0.4), (1.30, 1.2, 0.4)]
    for normal in ((1, 0), (0, 1), (0, -1), (-1, 0)):
        p0, p1, _, p3 = screen_corners((0.0, 0.0, 0.0), (2.0, 1.0), normal)
        u = [b - a for a, b in zip(p0, p1)]
        v = [b - a for a, b in zip(p0, p3)]
        # u x v points the way the screen faces: the image is not mirrored from that side.
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        assert n[0] * normal[0] + n[1] * normal[1] > 0 and abs(n[2]) < 1e-12


def test_the_shipped_backdrop_is_the_table_mat_a_floor_mat_and_four_screens_around_the_robot():
    from conftest import REPO
    import os
    from teleop_isaac import scene as scene_module
    s = scene_module.load(os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json"))
    quads = {qid: (cls, corners) for qid, cls, corners in backdrop_quads(s)}
    assert set(quads) == {"mat", "floor", "far", "back", "left", "right"}
    assert quads["mat"][0] == "mat" and quads["floor"][0] == "floor"
    mat = quads["mat"][1]
    assert min(p[0] for p in mat) == s.table.x_min and max(p[0] for p in mat) == s.table.x_max
    floor = quads["floor"][1]
    assert (min(p[0] for p in floor), max(p[0] for p in floor)) == (-1.0, 1.4)
    # The screens close the scene in on every side the third-person cameras look past.
    xs = [p[0] for qid in ("far", "back", "left", "right") for p in quads[qid][1]]
    ys = [p[1] for qid in ("far", "back", "left", "right") for p in quads[qid][1]]
    assert min(xs) <= -1.0 and max(xs) >= 1.4 and min(ys) <= -1.8 and max(ys) >= 1.8


def test_a_wrist_camera_looks_past_its_fingertips():
    # docs/adr/0032 amendment 3: D405-style, on the hand's back, looking along the fingers with the fingertips in view.
    from conftest import REPO
    import os
    from teleop_isaac import scene as scene_module
    s = scene_module.load(os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json"))
    cams = {c["id"]: c for c in s.extra["cameras"]}
    for side in ("left", "right"):
        c = cams[f"wrist-{side}"]
        assert c["mount"] == f"{side}_fr3v2_hand"
        p, q = mounted_camera_local(c)
        forward = rotate(q, (0.0, 0.0, -1.0))
        assert forward[2] > 0.85  # along the fingers (the hand's +z), tilted toward them
        k = intrinsics(c["width"], c["height"], c["hfovDeg"])
        tip = (0.0, 0.0, 0.11)  # between the fingertips, just short of the tool point
        xc = rotate((q[0], -q[1], -q[2], -q[3]), [t - a for t, a in zip(tip, p)])
        assert xc[2] < 0
        u, v = k["cx"] + k["fx"] * xc[0] / -xc[2], k["cy"] - k["fy"] * xc[1] / -xc[2]
        assert 0 <= u < c["width"] and c["height"] / 2 < v < c["height"]  # in view, in the lower half
    # A camera without a look-at point keeps the URDF head mount's convention.
    assert mounted_camera_local({"mount": "head_camera_mounting_point"}) == ((0.0, 0.0, 0.0), mount_camera_orientation())


def test_a_pose_relative_to_its_parent_composes_back():
    parent = ((0.3, -0.2, 0.5), kinematics.q_axis((0.2, 1.0, 0.3), 0.8))
    local = ((0.06, 0.0, 0.04), kinematics.q_axis((1.0, 0.0, 0.2), -0.4))
    world_p = [a + b for a, b in zip(parent[0], rotate(parent[1], local[0]))]
    world_q = kinematics.q_mul(parent[1], local[1])
    assert all(abs(a - b) < 1e-12 for a, b in zip(compose(parent, local)[0], world_p))
    p, q = relative_pose(parent, compose(parent, local))
    assert all(abs(a - b) < 1e-12 for a, b in zip(p, local[0]))
    assert all(abs(a - b) < 1e-12 for a, b in zip(q, local[1]))


def test_the_jetrovers_links_have_their_own_mask_classes_and_the_fr3s_keep_theirs():
    from teleop_isaac.renderer import MASK_IDS, mask_class

    assert mask_class("proximal_link_visual") == "arm" and mask_class("gripper_link_visual") == "hand"
    # As Isaac's importer names the JetRover's boxes (seen in its converted USD, 2026-11-08).
    assert mask_class("tn__proximal0_YE") == "arm" and mask_class("tn__proximal2_1_XH") == "arm"
    assert mask_class("tn__gripper1_nC") == "hand" and mask_class("tn__turntable0_oF") == "arm"
    assert mask_class("tn__Body12_ab") == "mount"
    assert mask_class("left_finger_visual") == "hand"
    assert mask_class("base_link_visual") == "mount"
    # The CAD's meshes, one visual per link and colour (docs/adr/0034 amendment 6).
    assert mask_class("wrist_link_2_visual") == "arm" and mask_class("left_finger_0_visual") == "hand"
    assert mask_class("gripper_link_0_visual") == "hand" and mask_class("base_link_1_visual") == "mount"
    assert mask_class("left_fr3v2_link3_visual") == "left_arm" and mask_class("right_fr3v2_hand_visual") == "right_hand"
    assert (MASK_IDS["arm"], MASK_IDS["hand"]) == (7, 8) and MASK_IDS["left_arm"] == 1


def test_the_sim_motor_map_turns_the_grippers_opening_into_finger_travel():
    from teleop_isaac import servo
    from teleop_isaac.renderer import SimMotorMap

    m = SimMotorMap(servo.SimModel.load())
    v = m.values([(2, 1.0), (10, 0.5)])
    assert v["proximal"] == 1.0
    g = servo.SimModel.load().motors[10]
    assert v["gripper"] == v["gripper_mirror"] == pytest.approx(g.finger_table[10])
    assert m.values([(10, 0.0)])["gripper"] == pytest.approx(-g.finger_open)  # open: past the CAD's pose


def test_the_render_settings_default_to_frames_that_show_only_their_own_pose():
    # Isaac's DLSS Performance smears a moving camera's frames (2026-11-14), and Isaac 6.2 turns anything but 3 or 4 back
    # into it (2026-11-15): DLAA, two samples a pixel, one render a pose.
    from teleop_isaac.renderer import parse_args

    a = parse_args(["--scene", "s.json"])
    assert (a.rtx_renderer, a.anti_aliasing, a.samples_per_pixel, a.subframes, a.motion_blur) == ("RealTimePathTracing", 4, 2, 1, False)
    b = parse_args(["--scene", "s.json", "--anti-aliasing", "3", "--dlss-mode", "2", "--subframes", "2", "--motion-blur"])
    assert (b.anti_aliasing, b.dlss_mode, b.subframes, b.motion_blur) == (3, 2, 2, True)
    for bad in (["--subframes", "0"], ["--samples-per-pixel", "0"], ["--anti-aliasing", "0"]):
        with pytest.raises(SystemExit):
            parse_args(["--scene", "s.json"] + bad)


def test_the_feeds_move_up_by_the_twins_port_offset():
    # A second twin on the machine (docs/adr/0037 amendment 7) serves its camera feeds 100 ports up; the first keeps the scene's.
    from teleop_isaac.renderer import parse_args

    assert parse_args(["--scene", "s.json"]).feed_port_offset == 0
    assert parse_args(["--scene", "s.json", "--feed-port-offset", "100"]).feed_port_offset == 100
    for bad in (["--feed-port-offset", "-1"], ["--feed-port-offset", "60000"]):
        with pytest.raises(SystemExit):
            parse_args(["--scene", "s.json"] + bad)


def test_the_render_settings_are_the_carb_settings_run_sets_and_checks():
    # Set once the stage has loaded (opening it puts Isaac's back) and checked again before streaming.
    from teleop_isaac.renderer import parse_args, render_settings

    assert render_settings(parse_args(["--scene", "s.json"])) == {
        "/rtx/rendermode": "RealTimePathTracing", "/rtx/post/aa/op": 4, "/rtx/post/dlss/execMode": 0, "/rtx/rtpt/spp": 2,
        "/rtx/post/motionblur/enabled": False}
    assert render_settings(parse_args(["--scene", "s.json", "--rtx-renderer", "PathTracing", "--motion-blur"])) == {
        "/rtx/rendermode": "PathTracing"}


# --- the room around the table (docs/adr/0037 decision 2) -------------------------------------------------------------

def _jetrover_bench():
    from conftest import REPO
    import os
    from teleop_isaac import scene as scene_module
    return scene_module.load(os.path.join(REPO, "core", "Scenes", "jetrover-bench.json"))


def test_the_jetrover_bench_is_a_grey_table_in_an_hdri_room_with_no_photo_surfaces():
    from teleop_isaac.renderer import environment_spec
    s = _jetrover_bench()
    assert backdrop_quads(s) == []  # no mat on the table, no screens around it
    env = environment_spec(s)
    assert env["dome"] == "dome" and env["meanRadiance"] > 0 and env["yawDeg"] == [0.0, 360.0]
    assert env["hideSurfaces"] == ["floor"]  # the room's own floor shows; the bridge's floor still catches what falls
    assert [x["kind"] for x in s.extra["lights"]] == ["dome"]  # the room is the light: no fixed distant light
    grey = {x.id: x.colour for x in s.surfaces}
    assert grey["table"] == grey["bench"] and len(set(grey["table"])) == 1


def test_an_environment_must_name_a_dome_light_a_brightness_and_real_surfaces():
    import dataclasses
    from teleop_isaac.renderer import environment_spec
    s = _jetrover_bench()
    for change, word in (({"kind": "box"}, "kind"), ({"dome": "sun"}, "dome"), ({"meanRadiance": 0}, "meanRadiance"),
                         ({"yawDeg": [10, 5]}, "yawDeg"), ({"rotateXYZDeg": [0, 0]}, "rotateXYZDeg"),
                         ({"hideSurfaces": ["ceiling"]}, "ceiling"), ({"change": "per-frame"}, "change"),
                         ({"targetBrightness": 300}, "targetBrightness")):
        t = dataclasses.replace(s, extra=dict(s.extra, environment=dict(s.extra["environment"], **change)))
        with pytest.raises(ValueError, match=word):
            environment_spec(t)
    t = dataclasses.replace(s, extra={k: v for k, v in s.extra.items() if k != "environment"})
    assert environment_spec(t) is None  # a scene with no room (fr3-duo-bench): backdrops as before


def test_a_room_has_settled_once_the_brightness_holds_still_past_the_render_lag():
    from teleop_isaac.renderer import settled
    assert not settled([198.1, 140.9, 141.4, 141.6], at_least=5)                # too soon
    assert settled([198.1, 140.9, 141.45, 141.61, 141.71, 141.77], at_least=5)  # measured on the 5090, 2026-11-15
    assert not settled([100.0, 100.0, 100.0, 100.0, 130.0], at_least=3)        # the new room has only just shown
    assert settled([100.0, 130.0, 130.1, 130.1, 130.2], at_least=3)


def test_the_meter_corrects_the_dome_toward_the_target_within_its_range():
    from teleop_isaac.renderer import METER_RANGE, metered_intensity
    # 1.5x too bright: the dome drops by 1.5^2.2, as an image's brightness goes with radiance^(1/2.2).
    assert metered_intensity(1000.0, 1000.0, 177.0, 118.0) == pytest.approx(1000.0 / 1.5 ** 2.2)
    assert metered_intensity(1000.0, 1000.0, 118.0 / 1.5, 118.0) == pytest.approx(1000.0 * 1.5 ** 2.2)
    assert metered_intensity(1000.0, 1000.0, 118.0, 118.0) == pytest.approx(1000.0)
    # Never past METER_RANGE of the luminance-normalised intensity, however dark the image (a black frame included).
    assert metered_intensity(3000.0, 1000.0, 1.0, 118.0) == pytest.approx(1000.0 * METER_RANGE[1])
    assert metered_intensity(300.0, 1000.0, 255.0, 10.0) == pytest.approx(1000.0 * METER_RANGE[0])
    assert metered_intensity(1000.0, 1000.0, 0.0, 118.0) == pytest.approx(1000.0 * METER_RANGE[1])


def test_a_cameras_own_exposure_is_a_gain_in_linear_light_within_its_range():
    import numpy as np
    from teleop_isaac.renderer import CAMERA_GAIN_RANGE, camera_gain, exposure_lut
    assert camera_gain(118.0, 118.0) == pytest.approx(1.0)
    assert camera_gain(59.0, 118.0) == pytest.approx(CAMERA_GAIN_RANGE[1])   # 2^2.2 > 4: as far as it goes
    assert camera_gain(177.0, 118.0) == pytest.approx((118 / 177) ** 2.2)
    assert camera_gain(0.0, 118.0) == pytest.approx(CAMERA_GAIN_RANGE[1])    # a black frame never divides by zero
    assert exposure_lut(1.0) is None
    up, down = exposure_lut(2.0), exposure_lut(0.5)
    assert up.dtype == np.uint8 and len(up) == 256
    assert (np.diff(up.astype(int)) >= 0).all() and (np.diff(down.astype(int)) >= 0).all()  # never reorders tones
    assert up[0] == 0 and up[255] == 255 and down[0] == 0
    # Double the light: mid-grey 118 (linear 0.181) becomes linear 0.362, sRGB ~161; half: linear 0.090, sRGB ~85.
    assert abs(int(up[118]) - 161) <= 1 and abs(int(down[118]) - 85) <= 1
    img = np.array([[[118, 0, 255]]], np.uint8)
    assert up[img].tolist() == [[[up[118], 0, 255]]]


# --- frame pacing (docs/adr/0038 "A, appearance", item 28) ------------------------------------------------------------

def _masks_label_by_label(seg, id_to_labels, table):
    """The renderer's masks as they were made before 2026-11-23: one comparison of the frame per label."""
    import numpy as np

    out = np.zeros(seg.shape, np.uint16)
    for k, v in id_to_labels.items():
        label = v.get("class") if isinstance(v, dict) else v
        if label in table:
            out[seg == int(k)] = table[label]
    return out


def test_masks_made_by_one_table_lookup_are_the_masks_made_label_by_label():
    import numpy as np
    from teleop_isaac.renderer import mask_table, stable_mask
    rng = np.random.default_rng(7)
    table = mask_table([31, 32, 41, 60, 65])
    labels = ["arm", "hand", "table", "bench", "clutter", "object_31", "object_32", "object_41", "object_60", "object_65",
              "BACKGROUND", "UNLABELLED", "lag_cube", "object_99"]
    for trial in range(40):
        keys = rng.choice(np.arange(0, 200), size=len(labels), replace=False)
        info = {str(int(k)): ({"class": lab} if trial % 2 else lab) for k, lab in zip(keys, labels)}
        seg = rng.choice(np.concatenate([keys, [201, 4000, 0]]), size=(36, 64)).astype(np.uint32)
        got = stable_mask(seg, info, table)
        assert got.dtype == np.uint16 and (got == _masks_label_by_label(seg, info, table)).all()
    signed = np.array([[-3, 5, 7], [9, 12, 99999]], np.int32)  # ids the table does not name, below 0 too, are 0
    info = {"5": {"class": "hand"}, "9": {"class": "object_41"}, "12": {"class": "BACKGROUND"}}
    assert stable_mask(signed, info, table).tolist() == [[0, 8, 0], [141, 0, 0]]
    huge = np.array([[70000, 3]], np.uint32)  # an id past the lookup table's reach is labelled the old way
    assert stable_mask(huge, {"70000": {"class": "table"}, "3": {"class": "arm"}}, table).tolist() == [[10, 7]]
    assert stable_mask(huge, {}, table).tolist() == [[0, 0]]


def test_a_cameras_exposure_table_gives_the_same_bytes_through_opencv():
    import numpy as np
    from teleop_isaac.renderer import apply_lut, exposure_lut
    rgb = np.random.default_rng(3).integers(0, 256, size=(36, 64, 3), dtype=np.uint8)
    for gain in (0.3, 0.8, 1.7, 3.9):
        lut = exposure_lut(gain)
        out = apply_lut(rgb, lut)
        assert out.dtype == np.uint8 and out.shape == rgb.shape and (out == lut[rgb]).all()


def test_the_garbage_collectors_pauses_and_the_loops_stretches_are_reported_per_window():
    from teleop_isaac.renderer import GcWatch, LoopTimes
    now = [0.0]
    w = GcWatch(clock=lambda: now[0])
    for gen, ms in ((0, 0.2), (0, 0.3), (2, 65.0), (1, 1.0)):
        w("start", {"generation": gen})
        now[0] += ms / 1e3
        w("stop", {"generation": gen, "collected": 0})
    assert w.report() == "gc 2/1/1 (gen 0/1/2), 66.5 ms, longest 65.0 ms (gen 2)"
    assert w.report() == "gc 0/0/0 (gen 0/1/2), 0.0 ms, longest 0.0 ms (gen -1)"  # a new window
    w("stop", {"generation": 0})  # a stop without its start (registered mid-collection) is not counted
    assert w.counts == [0, 0, 0]
    t = LoopTimes()
    for ms in (30.0, 10.0, 20.0):
        t.add("render", ms)
    t.add("read", 2.5)
    assert t.report() == "ms median/longest: render 20.0/30.0, read 2.5/2.5"
    assert t.report() == "ms: none"


def test_the_heap_is_frozen_unless_asked_not_to():
    from teleop_isaac.renderer import parse_args
    assert parse_args(["--scene", "x.json"]).gc_freeze is True
    assert parse_args(["--scene", "x.json", "--no-gc-freeze"]).gc_freeze is False
