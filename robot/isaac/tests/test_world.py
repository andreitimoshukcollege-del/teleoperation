"""The bridge's placement rules (teleop_isaac/world.py) without Isaac: the arm check is a stand-in."""

import math
import os

from conftest import REPO
from teleop_isaac import scene, wire
from teleop_isaac.world import WorldModel, is_valid, yaw_of

SCENE = os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json")


def no_arm(index, pose, margin):
    return False


def at(x, y, yaw=0.0, z=0.3):
    return (x, y, z, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def model():
    return WorldModel(scene.load(SCENE))


def test_objects_start_at_their_nominal_poses():
    m = model()
    for o, p in zip(m.scene.objects, m.poses):
        assert p == o.nominal
    assert m.revision == 0


def test_core_seed_1_layout_is_applied_whole():
    # core/Scenes/testdata/layout-seed1.txt: x, y, yaw per object in scene order, as RobotHost would place them.
    m = model()
    with open(os.path.join(REPO, "core", "Scenes", "testdata", "layout-seed1.txt"), encoding="utf-8") as f:
        rows = [tuple(float(v) for v in line.split()) for line in f if line.strip()]
    for o, (x, y, yaw) in zip(m.scene.objects, rows):
        outcome, i, upright = m.check(wire.CAUSE_LAYOUT, o.id, at(x, y, yaw), no_arm)
        assert outcome == wire.APPLIED, o.name
        m.apply(i, upright)
    assert m.revision == len(rows)
    for o, p, (x, y, yaw) in zip(m.scene.objects, m.poses, rows):
        assert abs(p[2] - (m.scene.table.top + o.size[2] / 2)) < 1e-12  # stood on the table, whatever z was asked
        assert abs(yaw_of(p) - yaw) < 1e-9


def test_each_rule_refuses_in_order():
    m = model()
    post = m.scene.objects[0]
    assert m.check(wire.CAUSE_LAYOUT, 200, at(0.5, 0.3), no_arm)[0] == wire.UNKNOWN_OBJECT
    assert m.check(wire.CAUSE_LAYOUT, post.id, (0.5, 0.3, 0.15, 2.0, 0.0, 0.0, 0.0), no_arm)[0] == wire.INVALID_POSE
    assert m.check(wire.CAUSE_LAYOUT, post.id, (float("nan"), 0.3, 0.15, 1.0, 0.0, 0.0, 0.0), no_arm)[0] == wire.INVALID_POSE
    assert m.check(wire.CAUSE_LAYOUT, post.id, at(1.5, 0.3), no_arm)[0] == wire.OUTSIDE_REGION
    other = m.poses[1]
    assert m.check(wire.CAUSE_LAYOUT, post.id, at(other[0], other[1]), no_arm)[0] == wire.OVERLAPS_OBJECT
    assert m.check(wire.CAUSE_LAYOUT, post.id, at(0.5, 0.3), lambda i, p, mg: True)[0] == wire.OVERLAPS_ARM
    assert m.revision == 0


def test_the_arm_is_checked_at_the_upright_pose_with_the_scene_clearance():
    m = model()
    asked = []

    def arm(index, pose, margin):
        asked.append((index, pose, margin))
        return False

    outcome, i, upright = m.check(wire.CAUSE_LAYOUT, m.scene.objects[0].id, at(0.5, 0.3, 0.2, z=9.0), arm)
    assert outcome == wire.APPLIED
    assert asked == [(i, upright, m.scene.clearance)]


def test_the_headset_may_not_move_an_arm_touched_object_or_a_fixed_one():
    m = model()
    post = m.scene.objects[0]
    touching = lambda i, p, margin: margin < m.scene.clearance  # touching now, clear at the new pose
    assert m.check(wire.CAUSE_HEADSET, post.id, at(0.5, 0.3), touching)[0] == wire.ARM_TOUCHING
    assert m.check(wire.CAUSE_LAYOUT, post.id, at(0.5, 0.3), touching)[0] == wire.APPLIED  # layouts are RobotHost's

    doc = scene.load(SCENE)
    fixed = WorldModel(scene.Scene(doc.name, doc.robot_profile, doc.surfaces, doc.table, doc.bench,
                                   tuple(o if k else o.__class__(**{**o.__dict__, "operator_movable": False})
                                         for k, o in enumerate(doc.objects)),
                                   doc.region, doc.clearance))
    assert fixed.check(wire.CAUSE_HEADSET, post.id, at(0.5, 0.3), no_arm)[0] == wire.NOT_MOVABLE


def test_an_object_may_be_placed_where_it_already_is():
    m = model()
    o = m.scene.objects[2]
    outcome, _, _ = m.check(wire.CAUSE_LAYOUT, o.id, o.nominal, no_arm)
    assert outcome == wire.APPLIED  # its own footprint is not another object's


def test_world_messages_carry_every_object_and_count_snapshots():
    m = model()
    seq = iter(range(1, 100))
    first = m.world_messages(lambda: next(seq), 123456789)
    assert len(first) == 1
    sensed, snapshot, revision, part, parts, objects = wire.decode_world(first[0])
    assert (sensed, snapshot, revision, part, parts) == (123456789, 1, 0, 0, 1)
    assert [o.id for o in objects] == [o.id for o in m.scene.objects]
    assert all(o.flags == 0 for o in objects)  # every shipped object is movable: a dynamic body, not kinematic
    assert wire.decode_world(m.world_messages(lambda: next(seq), 2)[0])[1] == 2


def test_more_than_16_objects_go_in_parts():
    m = model()
    many = [wire.ObjectPose(k + 1, wire.FLAG_KINEMATIC, at(0.5, 0.0)) for k in range(20)]
    messages = m.world_messages(lambda: 1, 5, many)
    decoded = [wire.decode_world(d) for d in messages]
    assert [(d[3], d[4], len(d[5])) for d in decoded] == [(0, 2, 16), (1, 2, 4)]
    assert decoded[0][1] == decoded[1][1]  # one snapshot


def test_validity_matches_core():
    assert is_valid(at(0.1, 0.2, 1.0))
    assert not is_valid((0, 0, 0, 1.002, 0, 0, 0.1))
    assert not is_valid((0, 0, float("inf"), 1, 0, 0, 0))


def test_prim_names_are_usd_names_and_stay_apart():
    from teleop_isaac.world import prim_name
    import re
    names = [prim_name(f"o{o.id}", o.name) for o in model().scene.objects]
    assert all(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", n) for n in names)
    assert len(set(names)) == len(names)
    assert prim_name("o1", "post-left") != prim_name("o2", "post.left")


def test_a_movable_object_is_where_physics_has_it_and_a_fixed_one_where_it_was_placed():
    # docs/adr/0031 amendment 4: a knocked-over object is where PhysX has it, for the placement rules; not a placement.
    m = model()
    can = m.scene.objects[3]
    assert can.movable
    knocked = (0.45, 0.05, 0.04, math.cos(0.7), math.sin(0.7), 0.0, 0.0)  # lying on its side
    m.sense(3, knocked)
    assert m.poses[3] == knocked and m.revision == 0
    post = m.scene.objects[0]
    # The post may not go where the can lies now, and the can stands up again wherever it is placed.
    assert m.check(wire.CAUSE_LAYOUT, post.id, at(0.45, 0.05), no_arm)[0] == wire.OVERLAPS_OBJECT
    outcome, i, upright = m.check(wire.CAUSE_LAYOUT, can.id, at(0.40, 0.0), no_arm)
    assert outcome == wire.APPLIED and upright[2] == m.scene.table.top + can.size[2] / 2 and upright[4:6] == (0.0, 0.0)

    doc = scene.load(SCENE)
    fixed = WorldModel(scene.Scene(doc.name, doc.robot_profile, doc.surfaces, doc.table, doc.bench,
                                   tuple(o.__class__(**{**o.__dict__, "movable": False, "mass": 0.0}) for o in doc.objects),
                                   doc.region, doc.clearance))
    fixed.sense(3, knocked)
    assert fixed.poses[3] == tuple(doc.objects[3].nominal)
    assert all(o.flags == wire.FLAG_KINEMATIC for o in fixed.objects())
