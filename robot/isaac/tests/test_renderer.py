"""The renderer's parts that need no Isaac: motors to URDF joints, the lag estimate, the cameras' geometry, the stamps."""

import math

from conftest import profile_path
from teleop_isaac import camwire, kinematics, profile
from teleop_isaac.renderer import MotorMap, Stamps, estimate_lag, intrinsics, look_at, mount_camera_orientation


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
