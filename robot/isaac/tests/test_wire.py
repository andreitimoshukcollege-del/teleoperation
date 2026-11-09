"""The bridge's wire against RobotHost's, byte for byte (golden/arm-server-wire-v1.json, written by ArmServerWireTests)."""

import struct

from conftest import golden
from teleop_isaac import wire


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def test_every_golden_message_encodes_and_decodes_exactly():
    messages = golden("arm-server-wire-v1.json")["messages"]
    assert [m["kind"] for m in messages] == ["hello", "welcome", "targets", "enable", "state", "health",
                                             "world_info", "world", "place", "placed", "contacts", "resize", "sizes"]
    for m in messages:
        raw = bytes.fromhex(m["hex"])
        assert wire.sequence_of(raw) == m["sequence"], m["kind"]
        if m["kind"] == "hello":
            assert wire.encode_hello(m["sequence"], m["hostTicks"], m["ticksPerSecond"], m["profile"]) == raw
            assert wire.decode_hello(raw) == wire.Hello(m["hostTicks"], m["ticksPerSecond"], m["profile"])
        elif m["kind"] == "welcome":
            assert wire.encode_welcome(m["sequence"], m["server"], m["serverNowTicks"], m["motors"], m["description"]) == raw
            assert wire.decode_welcome(raw) == (m["server"], m["serverNowTicks"], tuple(m["motors"]), m["description"])
        elif m["kind"] == "targets":
            targets = [(t[0], t[1]) for t in m["targets"]]
            assert wire.encode_targets(m["sequence"], m["hostTicks"], targets) == raw
            got = wire.decode_targets(raw)
            assert got.host_ticks == m["hostTicks"] and list(got.targets) == [(i, f32(v)) for i, v in targets]
        elif m["kind"] == "enable":
            assert wire.encode_enable(m["sequence"], m["hostTicks"], m["enabled"]) == raw
            assert wire.decode_enable(raw) == wire.Enable(m["hostTicks"], m["enabled"])
        elif m["kind"] == "state":
            motors = [tuple(x) for x in m["motors"]]
            assert wire.encode_state(m["sequence"], m["sensedTicks"], m["serverTicks"], m["flags"], motors) == raw
            sensed, server, flags, got = wire.decode_state(raw)
            assert (sensed, server, flags) == (m["sensedTicks"], m["serverTicks"], m["flags"])
            assert [tuple(g) for g in got] == [(i, f32(q), f32(v), f32(c)) for i, q, v, c in motors]
        elif m["kind"] == "health":
            assert wire.encode_health(m["sequence"], m["realTimeFactor"], m["lagMilliseconds"], m["lateSteps"], m["steps"]) == raw
            assert wire.decode_health(raw) == (m["realTimeFactor"], m["lagMilliseconds"], m["lateSteps"], m["steps"])
        elif m["kind"] == "world_info":
            sha = bytes.fromhex(m["sceneSha256"])
            assert wire.encode_world_info(m["sequence"], sha, m["ids"]) == raw
            assert wire.decode_world_info(raw) == (sha, tuple(m["ids"]))
        elif m["kind"] == "world":
            objects = [wire.ObjectPose(o[0], o[1], tuple(o[2:])) for o in m["objects"]]
            assert wire.encode_world(m["sequence"], m["sensedTicks"], m["snapshot"], m["revision"], m["part"], m["parts"], objects) == raw
            rounded = [wire.ObjectPose(o.id, o.flags, tuple(f32(v) for v in o.pose)) for o in objects]
            assert wire.decode_world(raw) == (m["sensedTicks"], m["snapshot"], m["revision"], m["part"], m["parts"], rounded)
        elif m["kind"] == "place":
            assert wire.encode_place(m["sequence"], m["hostTicks"], m["request"], m["cause"], m["id"], m["pose"]) == raw
            assert wire.decode_place(raw) == (m["hostTicks"], m["request"], m["cause"], m["id"], tuple(f32(v) for v in m["pose"]))
        elif m["kind"] == "placed":
            assert wire.encode_placed(m["sequence"], m["request"], m["id"], m["outcome"], m["revision"], m["pose"]) == raw
            assert wire.decode_placed(raw) == (m["request"], m["id"], m["outcome"], m["revision"], tuple(f32(v) for v in m["pose"]))
        elif m["kind"] == "resize":
            assert wire.encode_resize(m["sequence"], m["hostTicks"], m["request"], m["id"], m["scale"]) == raw
            assert wire.decode_resize(raw) == (m["hostTicks"], m["request"], m["id"], f32(m["scale"]))
        elif m["kind"] == "sizes":
            assert wire.encode_sizes(m["sequence"], m["revision"], [tuple(x) for x in m["sizes"]]) == raw
            assert wire.decode_sizes(raw) == (m["revision"], [(i, f32(f)) for i, f in m["sizes"]])
        elif m["kind"] == "contacts":
            assert wire.encode_contacts(m["sequence"], m["sensedTicks"], [tuple(c) for c in m["contacts"]]) == raw
            assert wire.decode_contacts(raw) == (m["sensedTicks"], [(a, l, o, f32(f)) for a, l, o, f in m["contacts"]])


def test_foreign_and_malformed_datagrams_are_refused():
    good = wire.encode_enable(1, 2, True)
    assert wire.kind_of(b"XS" + good[2:]) == 0
    assert wire.kind_of(good[:2] + bytes([2]) + good[3:]) == 0
    assert wire.decode_enable(good[:-1]) is None
    assert wire.decode_enable(good[:-1] + bytes([2])) is None
    state = wire.encode_state(1, 2, 3, 0, [(1, 0.0, 0.0, 0.0)])
    assert wire.decode_state(state[:-1]) is None
    assert wire.decode_targets(wire.encode_targets(1, 2, [(1, 0.5)])[:-2]) is None
