"""The trainer's copies of the trajectory shaper and franka-replay (docs/adr/0027) against Core's
golden files (TrainerGoldenBuilder): every step and every prediction, bit for bit."""

from __future__ import annotations

import struct

from armpred.baselines.franka_replay import FrankaReplayPredictor, ShapedState, TrajectoryShaperConfig, step
from armpred.baselines.types import JointPredictorConfig, JointSample

from .conftest import load


def _f(bits: int) -> float:
    return struct.unpack("<f", struct.pack("<i", bits))[0]


def _bits(value: float) -> int:
    return struct.unpack("<i", struct.pack("<f", value))[0]


def _samples(items) -> list:
    return [JointSample(m, _f(bits), bool(sensed)) for m, bits, sensed in items]


def test_every_shaper_step_matches_core_exactly():
    golden = load("joint-shaper-v1.json")
    shaper = TrajectoryShaperConfig.from_golden(golden["shaper"])
    assert len(golden["cases"]) == len(shaper.joints) == 5
    for case in golden["cases"]:
        limits = shaper.joints[shaper.index_of(case["motorId"])]
        s = ShapedState.at_rest(case["start"])
        for i, row in enumerate(case["steps"]):
            step(s, row[0], limits, shaper)
            where = f"motor {case['motorId']} step {i}"
            assert s.q == row[1], f"{where}: q {s.q!r} != {row[1]!r}"
            if limits.is_finger:
                assert int(s.moving) == row[2] and s.to == row[3], where
            else:
                assert s.v == row[2] and s.a == row[3], f"{where}: v/a {s.v!r}/{s.a!r} != {row[2]!r}/{row[3]!r}"


def test_every_franka_replay_prediction_matches_core_bit_for_bit():
    golden = load("franka-replay-v1.json")
    shaper = TrajectoryShaperConfig.from_golden(golden["shaper"])
    assert len(golden["cases"]) == 2
    for case in golden["cases"]:
        c = case["config"]
        config = JointPredictorConfig(max_joints=c["maxJoints"], command_history_capacity=c["commandHistoryCapacity"],
                                      max_joint_speed_rad_per_sec=2.0, instant_motor_ids=(8,),
                                      max_horizon_ticks=c["maxHorizonTicks"], ticks_per_second=c["ticksPerSecond"])
        p = FrankaReplayPredictor(config, shaper)
        predictions = 0
        for i, call in enumerate(case["calls"]):
            op = call["op"]
            if op == "reset":
                p.reset()
            elif op == "command":
                p.observe_command(call["t"], _samples(call["targets"]))
            elif op == "state":
                p.observe_state(call["t"], _samples(call["joints"]))
            else:
                ok, out = p.try_predict(call["t"])
                where = f"{case['name']} call {i}"
                assert ok == call["ok"], where
                got = [[s.motor_id, _bits(s.value), int(s.sensed)] for s in out]
                assert got == call["out"], f"{where}: {got} != {call['out']}"
                d = p.diagnostics
                assert (d.horizon_ticks, d.accepted, d.rejected) == (call["horizon"], call["accepted"], call["rejected"]), where
                predictions += 1
        assert predictions > 500
