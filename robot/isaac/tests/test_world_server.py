"""The world half of the arm-server wire both bridges serve (teleop_isaac/world_server.py), with a stand-in for Isaac's
world: the placement rules are WorldModel's own, the arm is never in the way, and PhysX reports the model's poses."""

import os

from conftest import REPO
from teleop_isaac import scene, wire
from teleop_isaac.world import WorldModel
from teleop_isaac.world_server import WorldServer

SCENE = os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json")


class StandInWorld:
    def __init__(self):
        self.model = WorldModel(scene.load(SCENE))

    def place(self, cause, object_id, pose):
        outcome, i, upright = self.model.check(cause, object_id, pose, lambda index, p, margin: False)
        if outcome == wire.APPLIED:
            self.model.apply(i, upright)
            return outcome, upright
        return outcome, tuple(pose)

    def resize(self, object_id, scale):
        outcome, i, pose = self.model.check_resize(object_id, scale, lambda index, p, margin: False)
        if outcome == wire.APPLIED:
            self.model.apply_resize(i, scale, pose)
            return outcome, pose
        return outcome, (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

    def read(self):
        return self.model.objects()


def server(world, sent, tapped, world_every=20):
    seq = [0]

    def next_sequence():
        seq[0] += 1
        return seq[0]

    ids = [o.id for o in world.model.scene.objects] if world is not None else []
    return WorldServer(world, b"\x07" * 32, ids, lambda m, c: sent.append((m, c)), next_sequence, tapped.append,
                       steps_per_second=1000, world_every=world_every, tag="test", clock_ns=lambda: 123)


def kinds(messages):
    return [wire.kind_of(m) for m, _ in messages]


def test_after_a_welcome_the_scene_and_then_the_sizes():
    sent, tapped = [], []
    w = StandInWorld()
    s = server(w, sent, tapped)
    s.publish(1, "host", tapping=False)  # sizes at start
    info = s.world_info()
    assert wire.decode_world_info(info) == (b"\x07" * 32, tuple(o.id for o in w.model.scene.objects))
    s.publish(2, "host", tapping=False)  # and again after each welcome
    assert kinds(sent) == [wire.SIZES, wire.SIZES]


def test_a_place_is_applied_and_answered_with_the_new_revision():
    sent, tapped = [], []
    w = StandInWorld()
    s = server(w, sent, tapped)
    o = next(o for o in w.model.scene.objects if o.movable)
    x, y, z, qw, qx, qy, qz = o.nominal
    place = wire.encode_place(1, 0, 17, wire.CAUSE_HEADSET, o.id, (x + 0.01, y, z, qw, qx, qy, qz))
    assert s.handle(wire.PLACE, place, "host")
    request, object_id, outcome, revision, pose = wire.decode_placed(sent[-1][0])
    assert (request, object_id, outcome, revision) == (17, o.id, wire.APPLIED, 1)
    assert abs(pose[0] - (x + 0.01)) < 1e-6 and sent[-1][1] == "host"


def test_an_applied_resize_sends_the_sizes_at_the_next_publish():
    sent, tapped = [], []
    w = StandInWorld()
    s = server(w, sent, tapped, world_every=1000)
    s.publish(1, "host", tapping=False)
    o = next(o for o in w.model.scene.objects if o.movable)
    assert s.handle(wire.RESIZE, wire.encode_resize(1, 0, 18, o.id, 1.25), "host")
    assert wire.decode_placed(sent[-1][0])[2] == wire.APPLIED
    s.publish(2, "host", tapping=False)
    assert kinds(sent) == [wire.SIZES, wire.PLACED, wire.SIZES]
    assert dict(wire.decode_sizes(sent[-1][0])[1])[o.id] == 1.25


def test_poses_every_world_every_steps_to_the_client_and_the_tap():
    sent, tapped = [], []
    s = server(StandInWorld(), sent, tapped, world_every=20)
    for step in range(1, 41):
        s.publish(step, None, tapping=True)  # no client yet: the renderer's tap still gets them
    assert sent == []
    assert [wire.kind_of(m) for m in tapped].count(wire.WORLD) >= 2
    assert wire.kind_of(tapped[0]) == wire.SIZES


def test_without_a_world_nothing_is_published_and_every_edit_is_refused():
    sent, tapped = [], []
    s = server(None, sent, tapped)
    assert s.world_info() is None
    s.publish(20, "host", tapping=True)
    assert sent == [] and tapped == []
    assert s.handle(wire.PLACE, wire.encode_place(1, 0, 3, wire.CAUSE_HEADSET, 2, (0.5, 0, 0.1, 1, 0, 0, 0)), "host")
    assert s.handle(wire.RESIZE, wire.encode_resize(2, 0, 4, 2, 1.5), "host")
    assert [wire.decode_placed(m)[2] for m, _ in sent] == [wire.UNKNOWN_OBJECT, wire.UNKNOWN_OBJECT]


def test_other_kinds_are_not_the_worlds():
    sent, tapped = [], []
    s = server(StandInWorld(), sent, tapped)
    assert not s.handle(wire.TARGETS, b"", "host")
    assert sent == []
