"""The renderer's forward kinematics, from the URDF, against Core's (golden/fr3-duo-fk-v1.json, written by Fr3FkGoldenTests)."""

import math
import os

import pytest

from conftest import REPO, golden
from teleop_isaac import kinematics as k

URDF = os.path.join(REPO, "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf")


def test_tool_poses_match_cores_kinematics_at_every_golden_joint_set():
    joints = k.read_urdf(URDF)
    g = golden("fr3-duo-fk-v1.json")
    assert len(g["sets"]) >= 10
    for s in g["sets"]:
        for arm, expected in s["tools"].items():
            p, q = k.link_pose(joints, s["joints"], f"{arm}_fr3v2_hand_tcp")
            if q[0] * expected[3] + q[1] * expected[4] + q[2] * expected[5] + q[3] * expected[6] < 0:
                q = tuple(-c for c in q)  # q and -q are one rotation
            assert list(p) == pytest.approx(expected[:3], abs=1e-9)
            assert list(q) == pytest.approx(expected[3:], abs=1e-9)


def test_the_head_camera_mount_is_where_adr_0032_says():
    p, q = k.link_pose(k.read_urdf(URDF), {}, "head_camera_mounting_point")
    assert list(p) == pytest.approx([0.042463788936189344, -0.014113, 0.39052590079539492], abs=1e-12)
    pitch = 2 * math.atan2(q[2], q[0])
    assert pitch == pytest.approx(0.7156, abs=1e-12) and abs(q[1]) < 1e-12 and abs(q[3]) < 1e-12


def test_local_motion_composes_to_the_chain():
    joints = k.read_urdf(URDF)
    j = joints["left_fr3v2_joint4"]
    rest_p, rest_q = j.xyz, k.q_rpy(*j.rpy)
    p, q = k.local_motion(j, rest_p, rest_q, -1.2)
    parent_p, parent_q = k.link_pose(joints, {"left_fr3v2_joint4": -1.2}, j.parent)
    child_p, child_q = k.link_pose(joints, {"left_fr3v2_joint4": -1.2}, j.child)
    d = k.q_rotate(parent_q, p)
    assert [parent_p[i] + d[i] for i in range(3)] == pytest.approx(list(child_p), abs=1e-12)
    assert list(k.q_mul(parent_q, q)) == pytest.approx(list(child_q), abs=1e-12)
