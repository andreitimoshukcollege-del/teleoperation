"""The bridge's placement rules (teleop_isaac/world.py) without Isaac: the arm check is a stand-in."""

import math
import os
import struct

import pytest

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


# --- resizing (docs/adr/0034 amendment 6) -------------------------------------------------------------------------------

def test_a_resize_stands_the_object_up_where_it_is_at_its_new_size_and_reports_every_size():
    m = model()
    # An object with room to grow by a fifth where it stands now.
    i = next(k for k, ob in enumerate(m.scene.objects) if ob.operator_movable and m.check_resize(ob.id, 1.25, no_arm)[0] == wire.APPLIED)
    o = m.scene.objects[i]
    seen = []

    def arm(index, pose, margin):
        seen.append(m.scales[index])
        return False

    outcome, index, upright = m.check_resize(o.id, 1.25, arm)
    assert (outcome, index) == (wire.APPLIED, i)
    assert seen == [1.0, 1.25]  # touching now at its size, then in the way at the new one
    assert m.scales[i] == 1.0  # nothing changes until it is applied
    assert abs(upright[2] - (m.scene.table.top + 1.25 * o.size[2] / 2)) < 1e-12
    assert upright[:2] == m.poses[i][:2]
    m.apply_resize(i, 1.25, upright)
    assert (m.scales[i], m.sizes_revision, m.revision) == (1.25, 1, 1)
    revision, sizes = wire.decode_sizes(m.sizes_message(7))
    assert revision == 1 and abs(dict(sizes)[o.id] - 1.25) < 1e-6 and len(sizes) == len(m.scene.objects)


def test_a_resize_is_refused_by_the_same_rules_at_the_new_size():
    m = model()
    o = m.scene.objects[0]
    assert m.check_resize(200, 1.5, no_arm)[0] == wire.UNKNOWN_OBJECT
    assert m.check_resize(o.id, 0.2, no_arm)[0] == wire.INVALID_POSE
    assert m.check_resize(o.id, 1.5, no_arm)[0] == wire.INVALID_POSE  # not a step: 1.25 to a whole power only
    assert m.check_resize(o.id, float("nan"), no_arm)[0] == wire.INVALID_POSE
    assert m.check_resize(o.id, 1.5625, lambda i, p, mg: True)[0] == wire.ARM_TOUCHING
    assert m.check_resize(o.id, 1.5625, lambda i, p, mg: mg >= m.scene.clearance)[0] == wire.OVERLAPS_ARM
    # Another object just clear of it now: tripled, it would reach that one, and its size is unchanged.
    i, k = m.index_of(o.id), (m.index_of(o.id) + 1) % len(m.scene.objects)
    p = m.poses[i]
    gap = m.scene.objects[i].footprint_radius + m.scene.objects[k].footprint_radius + m.scene.clearance + 0.002
    m.poses[k] = (p[0], p[1] + gap, *m.poses[k][2:])
    assert m.check_resize(o.id, 1.25 ** 8, no_arm)[0] in (wire.OVERLAPS_OBJECT, wire.OUTSIDE_REGION)
    assert m.scales[i] == 1.0


def test_placements_use_each_objects_current_size():
    m = model()
    o = m.scene.objects[2]
    i = m.index_of(o.id)
    _, _, upright = m.check_resize(o.id, 1.5625, no_arm)
    if upright is None:
        return  # this scene has no room to double it; the rule is covered by the refusals above
    m.apply_resize(i, 1.5625, upright)
    outcome, _, placed = m.check(wire.CAUSE_HEADSET, o.id, m.poses[i], no_arm)
    assert outcome == wire.APPLIED and abs(placed[2] - (m.scene.table.top + 1.5625 * o.size[2] / 2)) < 1e-12


def test_the_resize_steps_are_cores():
    from teleop_isaac.scene import RESIZE_SCALES, is_valid_scale

    assert RESIZE_SCALES[0] == 1.25 ** -3 and RESIZE_SCALES[-1] == 1.25 ** 8 and 1.0 in RESIZE_SCALES
    assert all(is_valid_scale(s) for s in RESIZE_SCALES) and not is_valid_scale(1.1) and not is_valid_scale(1.25 ** 9)


def test_a_resize_that_would_reach_past_an_edge_moves_the_object_in_until_it_fits():
    m = model()
    i = next(k for k, ob in enumerate(m.scene.objects) if ob.operator_movable)
    o = m.scene.objects[i]
    # Put it hard against the region's far edge, others well away, then double it: applied, moved in by the growth.
    x0, x1, y0, y1 = m.scene.region
    r = o.footprint_radius + 0.0011
    m.poses = [((x1 - r) if k == i else 9.0 + k, 0.0 if k == i else 9.0, *p[2:]) for k, p in enumerate(m.poses)]
    outcome, _, upright = m.check_resize(o.id, 1.25 ** 3, no_arm)
    assert outcome == wire.APPLIED
    assert abs(upright[0] - (x1 - o.footprint_radius * 1.25 ** 3 - 0.001)) < 1e-9 and upright[1] == 0.0
    assert m.check_resize(o.id, 1.25 ** 40, no_arm)[0] == wire.INVALID_POSE


def test_a_resize_that_would_reach_another_object_finds_the_nearest_spot_where_it_fits():
    m = model()
    i = next(k for k, ob in enumerate(m.scene.objects) if ob.operator_movable)
    o, k = m.scene.objects[i], (i + 1) % len(m.scene.objects)
    p = m.poses[i]
    gap = o.footprint_radius + m.scene.objects[k].footprint_radius + m.scene.clearance + 0.002
    m.poses[k] = (p[0], p[1] + gap, *m.poses[k][2:])  # a neighbour just clear of it now
    outcome, _, upright = m.check_resize(o.id, 1.5625, no_arm)
    assert outcome == wire.APPLIED
    assert not m.scene.overlaps(i, upright[0], upright[1], k, m.poses[k][0], m.poses[k][1], 1.5625, 1.0)
    assert (upright[0] - p[0]) ** 2 + (upright[1] - p[1]) ** 2 < 0.05 ** 2  # moved only as far as it had to


# --- a pool's objects, parked and placed (docs/adr/0037 decision 3) ---------------------------------------------------

POOL = os.path.join(REPO, "core", "Scenes", "jetrover-bench.json")


def pool_model():
    return WorldModel(scene.load(POOL))


def test_a_pools_objects_start_parked_on_the_shelf_and_say_so():
    m = pool_model()
    assert m.scene.is_pool and all(m.parked)
    for i, p in enumerate(m.poses):
        assert p == m.scene.parked_pose(i)
    assert all(o.flags & wire.FLAG_PARKED for o in m.objects())


def test_park_is_applied_whatever_pose_comes_and_any_other_placement_brings_the_object_back():
    m = pool_model()
    i = next(k for k, o in enumerate(m.scene.objects) if o.movable)
    oid = m.scene.objects[i].id
    outcome, index, upright = m.check(wire.CAUSE_LAYOUT, oid, at(0.28, 0.0, z=-0.15), no_arm)
    assert outcome == wire.APPLIED
    m.apply(index, upright)
    assert not m.parked[i] and not (m.objects()[i].flags & wire.FLAG_PARKED)
    outcome, index, upright = m.check(wire.CAUSE_PARK, oid, (float("nan"),) * 7, no_arm)  # the pose is the shelf's, not this
    assert (outcome, index, upright) == (wire.APPLIED, i, None)
    m.park(index, m.scene.parked_pose(i))
    assert m.parked[i] and m.objects()[i].flags & wire.FLAG_PARKED
    assert m.check(wire.CAUSE_PARK, 250, at(0, 0), no_arm)[0] == wire.UNKNOWN_OBJECT


def test_a_parked_object_is_never_in_the_way_and_the_headset_cannot_move_it():
    m = pool_model()
    a, b = [k for k, o in enumerate(m.scene.objects) if o.movable][:2]
    # Both parked: nothing stands on the table, so a placement right where a parked object's nominal pose is succeeds.
    pa = m.scene.objects[a].position
    assert m.check(wire.CAUSE_LAYOUT, m.scene.objects[b].id, at(pa[0], pa[1], z=-0.15), no_arm)[0] == wire.APPLIED
    assert m.check(wire.CAUSE_HEADSET, m.scene.objects[a].id, at(0.28, 0.1, z=-0.15), no_arm)[0] == wire.NOT_MOVABLE


def test_a_parked_object_changes_size_on_the_shelf_whatever_is_on_the_table():
    # docs/adr/0037 amendment 2: a layout resizes on the shelf before it places (RobotHost refuses the headset's resize of a
    # parked object). Nothing on the table can be in its way there, nor an arm.
    m = pool_model()
    a = next(k for k, o in enumerate(m.scene.objects) if o.movable and o.operator_movable)
    outcome, index, pose = m.check_resize(m.scene.objects[a].id, 2.44140625, lambda i, p, margin: True)
    assert (outcome, index) == (wire.APPLIED, a) and pose == m.scene.parked_pose(a, 2.44140625)
    m.apply_resize(index, 2.44140625, pose)
    assert m.parked[a] and m.scales[a] == 2.44140625 and m.objects()[a].flags & wire.FLAG_PARKED


def test_park_crosses_the_wire_and_resize_never_does():
    d = wire.encode_place(1, 2, 3, wire.CAUSE_PARK, 4, at(0, 0))
    assert wire.decode_place(d)[2] == wire.CAUSE_PARK
    assert wire.decode_place(wire.encode_place(1, 2, 3, 4, 4, at(0, 0))) is None  # 4 is RobotHost's resize record only


# --- rests (docs/adr/0037 amendment 6) ---------------------------------------------------------------------------------

def named(m, name):
    return next(i for i, o in enumerate(m.scene.objects) if o.name == name)


def test_a_lying_pose_is_placed_lying_on_its_rest_at_its_height():
    # RobotHost lays some of a layout's objects on a side; the bridge stands each as drawn, not upright.
    m = pool_model()
    s, tin = m.scene, named(m, "cleaner-tin")
    want = s.pose_at(tin, 0.26, 0.05, 0.9, 1.0, 2)
    outcome, i, placed = m.check(wire.CAUSE_LAYOUT, s.objects[tin].id, (*want[:2], 0.3, *want[3:]), no_arm)
    assert (outcome, i) == (wire.APPLIED, tin)
    assert placed == pytest.approx(want, abs=1e-12)
    assert abs(placed[2] - (s.table.top + s.objects[tin].size[0] / 2)) < 1e-12  # its x side up: 8.8 cm tall, whatever z came
    # As the wire carries it (float32): the same rest and heading.
    f32 = struct.unpack("<7f", struct.pack("<7f", *want))
    outcome, _, again = m.check(wire.CAUSE_LAYOUT, s.objects[tin].id, f32, no_arm)
    assert outcome == wire.APPLIED and again[:3] == pytest.approx(want[:3], abs=1e-6)
    assert scene.rest_of(again)[0] == 2 and scene.rest_of(again)[1] == pytest.approx(0.9, abs=1e-6)


def test_an_upright_or_slightly_tilted_pose_stands_up_and_a_steep_one_lies_on_its_nearest_side():
    m = pool_model()
    s, tin = m.scene, named(m, "cleaner-tin")
    oid = s.objects[tin].id
    outcome, _, placed = m.check(wire.CAUSE_LAYOUT, oid, at(0.26, 0.05, 0.9, z=0.3), no_arm)
    assert outcome == wire.APPLIED and placed == pytest.approx(s.pose_at(tin, 0.26, 0.05, 0.9), abs=1e-12)  # as before
    for degrees, rest in ((30, 0), (60, 1), (-60, 4), (150, 3)):
        a = math.radians(degrees)
        outcome, _, placed = m.check(wire.CAUSE_LAYOUT, oid, (0.26, 0.05, 0.3, math.cos(a / 2), math.sin(a / 2), 0.0, 0.0), no_arm)
        assert outcome == wire.APPLIED and scene.rest_of(placed)[0] == rest, degrees
        assert abs(placed[2] - (s.table.top + s.objects[tin].rest_extents(rest)[2] / 2)) < 1e-12


def test_a_lying_object_keeps_the_others_off_its_footprint_there():
    m = pool_model()
    s, tin, can = m.scene, named(m, "cleaner-tin"), named(m, "tomato-soup-can")
    outcome, _, placed = m.check(wire.CAUSE_LAYOUT, s.objects[tin].id, s.pose_at(tin, 0.25, 0.0, 0.0, 1.0, 1), no_arm)
    assert outcome == wire.APPLIED
    m.apply(tin, placed)
    standing = s.objects[tin].footprint_radius + s.objects[can].footprint_radius + s.clearance
    lying = s.objects[tin].footprint_radius_at(1) + s.objects[can].footprint_radius + s.clearance
    d = (standing + lying) / 2  # clear of the tin standing, not of the tin lying
    assert m.check(wire.CAUSE_LAYOUT, s.objects[can].id, at(0.25, d, z=0.0), no_arm)[0] == wire.OVERLAPS_OBJECT
    m.apply(tin, s.pose_at(tin, 0.25, 0.0, 0.0))  # stood up again: its round base
    assert m.check(wire.CAUSE_LAYOUT, s.objects[can].id, at(0.25, d, z=0.0), no_arm)[0] == wire.APPLIED


def test_a_movable_object_may_be_placed_by_the_edge_and_a_fixed_one_may_not():
    # The edge regions lie outside the placement region (the bench's back edge, behind the arm): a layout's edge object stands
    # there, and the bridge must not refuse it as OUTSIDE_REGION.
    m = pool_model()
    s, can = m.scene, named(m, "tomato-soup-can")
    y = max(s.edge_regions[2], s.robot_keep_out[3] + s.objects[can].footprint_radius_at(1) + 0.001)
    for rest in (0, 1):
        outcome, _, placed = m.check(wire.CAUSE_LAYOUT, s.objects[can].id, s.pose_at(can, -0.16, y, 0.3, 1.0, rest), no_arm)
        assert outcome == wire.APPLIED and scene.rest_of(placed)[0] == rest
    fixture = next(i for i, o in enumerate(s.objects) if not o.movable)
    assert m.check(wire.CAUSE_LAYOUT, s.objects[fixture].id, at(-0.16, y, z=0.0), no_arm)[0] == wire.OUTSIDE_REGION


def test_clutter_is_placed_anywhere_in_the_region_flat_by_the_rules_every_movable_object_has():
    # docs/adr/0037 "Raised clutter": RobotHost stands a layout's clutter anywhere in the placement region, flat as the file has
    # it, out of the arm's reach too (the cameras' foreground). The bridge places it by the rules of every movable object.
    m = pool_model()
    s, pen, tape = m.scene, named(m, "pen"), named(m, "tape-roll")
    o = s.objects[pen]
    assert o.clutter and o.movable and not o.operator_movable and s.clutter == (1, 3)
    x, y = s.region[1] - o.footprint_radius - 0.001, s.region[3] - o.footprint_radius - 0.001  # the region's far corner
    assert math.hypot(x, y) > 0.30  # beyond the reach band the pool's other movable objects stand in
    outcome, i, placed = m.check(wire.CAUSE_LAYOUT, o.id, s.pose_at(pen, x, y, 0.4), no_arm)
    assert (outcome, i) == (wire.APPLIED, pen) and scene.rest_of(placed) == (scene.REST_UPRIGHT, pytest.approx(0.4, abs=1e-12))
    assert abs(placed[2] - (s.table.top + o.size[2] / 2)) < 1e-12  # flat on the table, 4.5 mm up
    m.apply(pen, placed)
    assert m.flags(pen) == 0  # a dynamic body on the table: the arm can push it about
    # Another object keeps clear of its footprint; off the region, or on the robot's keep-out, it is refused.
    assert m.check(wire.CAUSE_LAYOUT, s.objects[tape].id, s.pose_at(tape, x, y, 0.0), no_arm)[0] == wire.OVERLAPS_OBJECT
    assert m.check(wire.CAUSE_LAYOUT, o.id, s.pose_at(pen, s.region[1] + 0.05, 0.0, 0.0), no_arm)[0] == wire.OUTSIDE_REGION
    assert m.check(wire.CAUSE_LAYOUT, o.id, s.pose_at(pen, 0.2, 0.0, 0.0), no_arm)[0] == wire.OUTSIDE_REGION  # in the region, on the robot
    # It never lies on a side, and the headset leaves it alone.
    assert not any(o.can_lie(r) for r in range(scene.REST_COUNT))
    assert m.check(wire.CAUSE_HEADSET, o.id, s.pose_at(pen, 0.3, 0.0, 0.0), no_arm)[0] == wire.NOT_MOVABLE


def test_clutter_is_its_own_rectangle_at_its_heading_where_its_circle_would_not_fit():
    # docs/adr/0037 "Raised clutter": the screwdrivers' circle is 29 cm across; turned along y, the pair is 10 cm deep and stands by
    # the region's far edge, as RobotHost draws it. Across, it would reach past the edge. Rules 5 and 6 take the same rectangle.
    m = pool_model()
    s, sd, pen = m.scene, named(m, "screwdrivers"), named(m, "pen")
    o = s.objects[sd]
    x = s.region[1] - o.size[0] / 2 - 0.001
    assert not s.fits_on_table(sd, x, 0.0)  # its circle, which holds it at any heading
    outcome, _, placed = m.check(wire.CAUSE_LAYOUT, o.id, s.pose_at(sd, x, 0.0, 0.0), no_arm)
    assert outcome == wire.APPLIED
    assert m.check(wire.CAUSE_LAYOUT, o.id, s.pose_at(sd, x, 0.0, math.pi / 2), no_arm)[0] == wire.OUTSIDE_REGION
    m.apply(sd, placed)
    # A pen lying along it in front of it (nearer the robot) keeps the 1 cm clearance from its side at 1.05 cm; at 0.95 cm, not.
    gap = o.size[0] / 2 + s.objects[pen].size[1] / 2
    assert m.check(wire.CAUSE_LAYOUT, s.objects[pen].id, s.pose_at(pen, x - gap - 0.0105, 0.0, math.pi / 2), no_arm)[0] == wire.APPLIED
    assert m.check(wire.CAUSE_LAYOUT, s.objects[pen].id, s.pose_at(pen, x - gap - 0.0095, 0.0, math.pi / 2), no_arm)[0] == wire.OVERLAPS_OBJECT


def test_a_resize_stands_the_object_up_clear_of_a_lying_neighbour_as_it_lies():
    m = model()  # the FR3 bench: every object may be resized
    s, post, can = m.scene, 0, 3
    # The post knocked over 20 cm from the can: lying, its footprint reaches 15 cm (standing, 4 cm), so the can grown by a
    # quarter no longer fits where it stands and moves to the nearest spot clear of it.
    m.sense(post, s.pose_at(post, 0.40, 0.20, 0.0, 1.0, 1))
    assert not s.overlaps(can, 0.40, 0.0, post, 0.40, 0.20, 1.25, 1.0) and s.overlaps(can, 0.40, 0.0, post, 0.40, 0.20, 1.25, 1.0, 0, 1)
    outcome, _, upright = m.check_resize(s.objects[can].id, 1.25, no_arm)
    assert outcome == wire.APPLIED and scene.rest_of(upright)[0] == 0
    assert not s.overlaps(can, upright[0], upright[1], post, 0.40, 0.20, 1.25, 1.0, 0, 1)
    assert (upright[0] - 0.40) ** 2 + upright[1] ** 2 < 0.06 ** 2  # moved only as far as it had to
