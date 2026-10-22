"""relay-replay-servo's copy against Core (trainer/contract/golden/relay-replay-servo.json): the same
call sequences as relay-replay's golden file, under neutral, default and custom servo parameters."""

from __future__ import annotations

import math
import struct

from armpred.baselines.relay_replay_servo import RelayReplayServoPredictor
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
        servo_max_speed_rad_per_sec=_f(c["servoMaxSpeedRadPerSec"][1]),
        servo_lag_seconds=_f(c["servoLagSeconds"][1]),
        servo_pipeline_delay_seconds=_f(c["servoPipelineDelaySeconds"][1]),
        servo_first_move_from_measured=bool(c["servoFirstMoveFromMeasured"]),
        servo_step_seconds=_f(c["servoStepSeconds"][1]),
    )


def _samples(items) -> list:
    return [JointSample(m, _f(bits), bool(sensed)) for m, bits, sensed in items]


def _ulps(a_bits: int, b_bits: int) -> int:
    # float32 bit patterns as ordered integers (sign-magnitude to two's-complement order).
    order = lambda b: b if b >= 0 else -(b & 0x7FFFFFFF)  # noqa: E731
    return abs(order(a_bits) - order(b_bits))


def test_every_prediction_matches_core():
    cases = load("relay-replay-servo.json")["cases"]
    assert [c["name"].split("/")[0] for c in cases].count("custom") == 3
    worst = 0
    for case in cases:
        config = _config(case["config"])
        assert _bits(config.servo_step_seconds) == case["config"]["servoStepSeconds"][1]
        assert math.isinf(config.servo_max_speed_rad_per_sec) == (case["config"]["servoMaxSpeedRadPerSec"][0] == "Infinity")
        p = RelayReplayServoPredictor(config)
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
                if ok:
                    assert [s.motor_id for s in out] == [x[0] for x in call["out"]], where
                    assert [int(s.sensed) for s in out] == [x[2] for x in call["out"]], where
                    for s, x in zip(out, call["out"]):
                        # Math.Exp and math.exp are each platform's C library; allow one float32 ULP.
                        d = _ulps(_bits(s.value), x[1])
                        assert d <= 1, f"{where}: motor {s.motor_id} {s.value!r} vs {_f(x[1])!r}"
                        worst = max(worst, d)
                d = p.diagnostics
                assert (d.horizon_ticks, d.accepted, d.rejected) == (call["horizon"], call["accepted"], call["rejected"]), where
                predictions += 1
            else:
                raise AssertionError(f"unknown op {op}")
        assert predictions > 500
        gate = case["gate"]
        assert (p.written, p.deadband_drops, p.coalesced) == (gate["written"], gate["deadbandDrops"], gate["coalesced"]), case["name"]
    assert worst <= 1
