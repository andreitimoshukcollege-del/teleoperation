"""The renderer's parts that need no Isaac: motors to URDF joints, the lag estimate, the cameras' geometry, the stamps."""

import math

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
