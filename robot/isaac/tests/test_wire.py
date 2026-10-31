"""The bridge's wire against RobotHost's, byte for byte (golden/arm-server-wire-v1.json, written by ArmServerWireTests)."""

import struct

from conftest import golden
from teleop_isaac import wire


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def test_every_golden_message_encodes_and_decodes_exactly():
    messages = golden("arm-server-wire-v1.json")["messages"]
    assert [m["kind"] for m in messages] == ["hello", "welcome", "targets", "enable", "state", "health"]
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


def test_foreign_and_malformed_datagrams_are_refused():
    good = wire.encode_enable(1, 2, True)
    assert wire.kind_of(b"XS" + good[2:]) == 0
    assert wire.kind_of(good[:2] + bytes([2]) + good[3:]) == 0
    assert wire.decode_enable(good[:-1]) is None
    assert wire.decode_enable(good[:-1] + bytes([2])) is None
    state = wire.encode_state(1, 2, 3, 0, [(1, 0.0, 0.0, 0.0)])
    assert wire.decode_state(state[:-1]) is None
    assert wire.decode_targets(wire.encode_targets(1, 2, [(1, 0.5)])[:-2]) is None
