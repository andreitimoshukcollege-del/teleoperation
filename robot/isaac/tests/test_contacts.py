"""The arm's contacts with the scene's objects, from PhysX's contact events (docs/adr/0036 amendment 1)."""

import os

import pytest

from teleop_isaac import wire
from teleop_isaac.contacts import (CONTACT_OFFSET_M, FOUND, LOST, PERSIST, REST_OFFSET_M, TOUCH_SEPARATION_M, UNKNOWN_LINK,
                                   ContactEvent, ContactTracker, urdf_links)

ROBOT = "/World/jetrover"
LINKS = ["base_link", "turntable", "proximal_link", "distal_link", "wrist_link", "gripper_link", "left_finger", "right_finger"]
# Object 2 can be resized: a second body, parked, stands for the same object.
OBJECTS = {"/World/Scene/Objects/o1_cube": 1, "/World/Scene/Objects/o2_post": 2, "/World/Scene/Objects/o2_post_s1": 2}
TABLE = "/World/Scene/Surfaces/s1_table"


def ev(body0, body1, kind, impulse=0.001, shape0=None, shape1=None, separation=float("inf")):
    return ContactEvent(body0, body1, shape0 or body0 + "/collisions", shape1 or body1 + "/Shape", kind, impulse, separation)


def tracker():
    return ContactTracker(ROBOT, LINKS, OBJECTS, dt=0.001)


def test_a_touch_starts_lasts_and_ends_once_each():
    t = tracker()
    assert t.update([ev(f"{ROBOT}/gripper_link", "/World/Scene/Objects/o1_cube", FOUND, 0.002)]) is True
    assert t.contacts() == [(0, 5, 1, pytest.approx(2.0))]  # 0.002 N s over a 1 ms step
    assert t.update([ev(f"{ROBOT}/gripper_link", "/World/Scene/Objects/o1_cube", PERSIST, 0.004)]) is False
    assert t.contacts()[0][3] == pytest.approx(4.0)  # the force follows, the set does not change
    assert t.update([]) is False
    assert t.update([ev(f"{ROBOT}/gripper_link", "/World/Scene/Objects/o1_cube", LOST, 0.0)]) is True
    assert t.contacts() == [] and t.touching() == ()


def test_either_order_and_nested_link_prims():
    t = tracker()
    # The object first, and a robot body nested below its link's prim, as the URDF importer may make them.
    t.update([ev("/World/Scene/Objects/o2_post", f"{ROBOT}/left_finger/visuals_xform", FOUND, 0.001)])
    assert t.touching() == ((6, 2),)


def test_only_the_arm_against_objects_counts():
    t = tracker()
    changed = t.update([
        ev("/World/Scene/Objects/o1_cube", TABLE, FOUND),  # an object on the table
        ev("/World/Scene/Objects/o1_cube", "/World/Scene/Objects/o2_post", FOUND),  # two objects
        ev(f"{ROBOT}/base_link", TABLE, FOUND),  # the robot on a surface
    ])
    assert changed is False and t.contacts() == []


def test_several_shape_pairs_are_one_contact_until_the_last_parts():
    t = tracker()
    a = ev(f"{ROBOT}/distal_link", "/World/Scene/Objects/o1_cube", FOUND, 0.001, shape0="s/a")
    b = ev(f"{ROBOT}/distal_link", "/World/Scene/Objects/o1_cube", FOUND, 0.003, shape0="s/b")
    assert t.update([a, b]) is True
    assert t.contacts() == [(0, 3, 1, pytest.approx(4.0))]  # one contact, the forces summed
    assert t.update([ev(f"{ROBOT}/distal_link", "/World/Scene/Objects/o1_cube", LOST, shape0="s/a")]) is False
    assert t.touching() == ((3, 1),)
    assert t.update([ev(f"{ROBOT}/distal_link", "/World/Scene/Objects/o1_cube", LOST, shape0="s/b")]) is True


def test_a_resized_body_is_its_object_and_forget_drops_it():
    t = tracker()
    t.update([ev(f"{ROBOT}/wrist_link", "/World/Scene/Objects/o2_post_s1", FOUND),
              ev(f"{ROBOT}/wrist_link", "/World/Scene/Objects/o1_cube", FOUND)])
    assert t.touching() == ((4, 1), (4, 2))
    t.forget(2)
    assert t.touching() == ((4, 1),)
    assert t.update([]) is True  # forgetting changed the set: the next step says so
    t.clear()
    assert t.contacts() == []


def test_a_body_that_names_no_link_is_still_a_contact():
    t = tracker()
    t.update([ev(f"{ROBOT}/something_else", "/World/Scene/Objects/o1_cube", FOUND)])
    assert t.touching() == ((UNKNOWN_LINK, 1),)


def test_the_contacts_go_on_the_wire_and_back():
    t = tracker()
    t.update([ev(f"{ROBOT}/gripper_link", "/World/Scene/Objects/o1_cube", FOUND, 0.0015)])
    sensed, contacts = wire.decode_contacts(wire.encode_contacts(7, 123456789, t.contacts()))
    assert sensed == 123456789 and [(a, l, o) for a, l, o, _ in contacts] == [(0, 5, 1)]
    assert contacts[0][3] == pytest.approx(1.5)
    assert wire.decode_contacts(wire.encode_contacts(8, 5, [])) == (5, [])  # contact over: an empty message


def test_the_jetrover_urdf_links_are_in_file_order():
    here = os.path.dirname(os.path.abspath(__file__))
    urdf = os.path.join(here, "..", "..", "jetrover", "urdf", "jetrover.urdf")
    assert urdf_links(urdf) == LINKS


def test_dt_must_be_positive():
    with pytest.raises(ValueError):
        ContactTracker(ROBOT, LINKS, OBJECTS, dt=0.0)


GRIPPER, CUBE = f"{ROBOT}/gripper_link", "/World/Scene/Objects/o1_cube"


def test_a_pair_inside_the_offsets_is_near_until_it_touches():
    # docs/adr/0036 amendment 4: PhysX finds the pair 6 mm apart, with no push; it closes in and touches at 0.4 mm, pushes,
    # backs off to 3 mm (still reported) and is lost.
    t = tracker()
    assert t.update([ev(GRIPPER, CUBE, FOUND, 0.0, separation=0.006)]) is False
    assert t.touching() == () and t.contacts() == [] and t.near_events == 1
    assert t.update([ev(GRIPPER, CUBE, PERSIST, 0.0, separation=0.0015)]) is False
    assert t.update([ev(GRIPPER, CUBE, PERSIST, 0.0, separation=0.0004)]) is True  # within 1 mm, no push yet: a touch
    assert t.contacts() == [(0, 5, 1, 0.0)]
    assert t.update([ev(GRIPPER, CUBE, PERSIST, 0.0008, separation=-0.0002)]) is False
    assert t.contacts() == [(0, 5, 1, pytest.approx(0.8))]
    assert t.update([ev(GRIPPER, CUBE, PERSIST, 0.0, separation=0.003)]) is True  # backed off: near again, not a contact
    assert t.touching() == ()
    assert t.update([ev(GRIPPER, CUBE, LOST, 0.0)]) is False
    assert t.near_events == 3
    assert (t.min_separation, t.max_separation) == (pytest.approx(-0.0002), pytest.approx(0.006))


def test_an_impulse_is_a_touch_whatever_the_separation():
    # A fast link the solver stops before it would pass in during the step: its points start more than 1 mm apart.
    t = tracker()
    assert t.update([ev(GRIPPER, CUBE, FOUND, 0.0005, separation=0.0018)]) is True
    assert t.touching() == ((5, 1),)


def test_the_touch_threshold_is_exactly_one_millimetre():
    t = tracker()
    t.update([ev(GRIPPER, CUBE, FOUND, 0.0, separation=TOUCH_SEPARATION_M)])
    assert t.touching() == ((5, 1),)
    t.update([ev(GRIPPER, CUBE, PERSIST, 0.0, separation=TOUCH_SEPARATION_M + 1e-5)])
    assert t.touching() == ()


def test_the_offsets_can_report_a_touch_and_see_the_fastest_step_coming():
    assert REST_OFFSET_M == 0.0  # bodies at rest touch at separation 0: the threshold measures a gap
    pair = 2 * CONTACT_OFFSET_M  # PhysX: a pair's contacts within the two shapes' contact offsets summed
    assert pair > TOUCH_SEPARATION_M
    fastest_step = 8.0 * 0.3 * 0.001  # 8 rad/s, 0.3 m out, one 1 ms step
    assert pair >= 2 * fastest_step
