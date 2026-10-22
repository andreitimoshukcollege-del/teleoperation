from __future__ import annotations

import struct

from armpred.baselines.relay_replay import RelayReplayPredictor
from armpred.baselines.types import JointPredictorConfig, JointSample

from .conftest import load


def _f(bits: int) -> float:
    return struct.unpack("<f", struct.pack("<i", bits))[0]


def _bits(value: float) -> int:
    return struct.unpack("<i", struct.pack("<f", value))[0]


def _config(c: dict) -> JointPredictorConfig:
    return JointPredictorConfig(
        max_joints=c["maxJoints"],
        command_history_capacity=c["commandHistoryCapacity"],
        max_joint_speed_rad_per_sec=_f(c["maxJointSpeedRadPerSec"][1]),
        instant_motor_ids=tuple(c["instantMotorIds"]),
        max_horizon_ticks=c["maxHorizonTicks"],
        ticks_per_second=c["ticksPerSecond"],
        scaled_motor_ids=tuple(c["scaledMotorIds"]),
        speed_scales=tuple(_f(b) for b in c["speedScales"]),
        relay_min_move_seconds=_f(c["relayMinMoveSeconds"][1]),
        relay_deadband_rad=_f(c["relayDeadbandRad"][1]),
        relay_move_speed_rad_per_sec=_f(c["relayMoveSpeedRadPerSec"][1]),
        sensed_age_motor_ids=tuple(c["sensedAgeMotorIds"]),
        sensed_age_seconds=tuple(_f(b) for b in c["sensedAgeSeconds"]),
    )


def _samples(items) -> list:
    return [JointSample(m, _f(bits), bool(sensed)) for m, bits, sensed in items]


def test_every_prediction_matches_core_bit_for_bit():
    cases = load("relay-replay-v1.json")["cases"]
    assert len(cases) == 3
    for case in cases:
        config = _config(case["config"])
        # The config's floats must round-trip exactly, or every later comparison is meaningless.
        assert _bits(config.relay_deadband_rad) == case["config"]["relayDeadbandRad"][1]
        p = RelayReplayPredictor(config)
        predictions = 0
        for i, call in enumerate(case["calls"]):
            op = call["op"]
            if op == "reset":
                p.reset()
            elif op == "command":
                p.observe_command(call["t"], _samples(call["targets"]), _f(call["speed"][1]))
            elif op == "state":
                p.observe_state(call["t"], _samples(call["joints"]))
            elif op == "predict":
                ok, out = p.try_predict(call["t"])
                where = f"{case['name']} call {i}"
                assert ok == call["ok"], where
                got = [[s.motor_id, _bits(s.value), int(s.sensed)] for s in out] if ok else []
                assert got == call["out"], where
                d = p.diagnostics
                assert (d.horizon_ticks, d.accepted, d.rejected) == (call["horizon"], call["accepted"], call["rejected"]), where
                predictions += 1
            else:
                raise AssertionError(f"unknown op {op}")
        assert predictions > 500
        gate = case["gate"]
        assert (p.written, p.deadband_drops, p.coalesced) == (gate["written"], gate["deadbandDrops"], gate["coalesced"]), case["name"]
