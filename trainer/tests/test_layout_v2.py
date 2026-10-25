"""Layout v2 (ADR 0021 decision 10): the commanded path covers every command since the last reading at
any delay, the network block carries a forecast and nothing without one, and shards of another
layout are refused."""

from __future__ import annotations

import json

import numpy as np
import pytest

from armpred.baselines.types import JointSample, headset_joint_config
from armpred.data.build import load_shards
from armpred.features import layout as L
from armpred.net import forecast as F

NS = 1_000_000_000


def test_widths():
    # v3: six joint rows (motors 1-5, then the gripper), each with a six-wide identity and a reference flag.
    assert L.PREDICTED_MOTORS == (1, 2, 3, 4, 5, 10)
    assert L.JOINT_FEATURES == 2 + 80 * 2 + 16 * 3 + 8 * 4 + 6 + 2 + 1 == 251
    assert L.NETWORK_FEATURES == 2 * 9 * 5 + 2 + 2 + 6 + 1 == 101
    assert L.ARM_FEATURES == 13 + 101 and L.arm_features(True) == L.ARM_FEATURES + 1
    assert L.layout_id(False) == "v3" and L.layout_id(True) == "v3-aware"


def test_the_gripper_row_is_in_servo_radians_and_needs_no_reading():
    state = L.FeatureState(headset_joint_config(NS))
    for k in range(48):
        targets = [JointSample(m, 0.1, False) for m in L.ARM_MOTORS] + [JointSample(L.GRIPPER_MOTOR, 0.5, False)]
        state.observe_command(int(k / 48.0 * NS), targets, 0.0)
    # The arm read, the gripper not: as in a recording from before it was read. The window is still valid.
    state.observe_state(int(0.9 * NS), [JointSample(m, 0.1, True) for m in L.ARM_MOTORS] + [JointSample(L.GRIPPER_MOTOR, 0.5, False)])
    w = state.window(int(1.0 * NS))
    g = L.PREDICTED_MOTORS.index(L.GRIPPER_MOTOR)
    assert w.valid and w.joints.shape == (6, L.JOINT_FEATURES)
    assert w.joints[g, -1] == 0.0 and w.joints[0, -1] == 1.0  # the reference flag
    assert w.newest[g] == w.prior[g]  # no reading: referenced to the prior
    # Once the gripper is read, its row is in servo radians (fraction / GRIPPER_FRACTION_PER_RAD).
    state.observe_state(int(0.95 * NS), [JointSample(m, 0.1, True) for m in L.ARM_MOTORS] + [JointSample(L.GRIPPER_MOTOR, 0.45, True)])
    w = state.window(int(1.0 * NS))
    assert w.joints[g, -1] == 1.0
    assert abs(w.newest[g] - 0.45 / L.GRIPPER_FRACTION_PER_RAD) < 1e-9
    assert L.to_wire(L.GRIPPER_MOTOR, L.to_model(L.GRIPPER_MOTOR, 0.45)) == 0.45 and L.to_model(1, 0.3) == 0.3


def _state_with_commands(uplink_s: float, send_period_s: float = 1 / 48.0):
    """Commands every 1/48 s for 2 s, each taking effect ``uplink_s`` after it is sent; one reading."""
    state = L.FeatureState(headset_joint_config(NS))
    t_send = np.arange(0.0, 2.0, send_period_s)
    for k, ts in enumerate(t_send):
        targets = [JointSample(m, 0.01 * k + 0.001 * m, False) for m in L.ARM_MOTORS]
        state.observe_command(int((ts + uplink_s) * NS), targets, 0.0)
    state.observe_state(int(1.6 * NS), [JointSample(m, 0.0, True) for m in L.ARM_MOTORS])
    return state, t_send


def test_the_commanded_path_covers_what_v1_missed_at_high_uplink_delay():
    # At 300 ms of uplink delay, v1's 24 newest commands reached back only ~0.5 s from the newest send,
    # leaving the commands that took effect in [T-0.5, T-0.26) in neither its list nor its bins.
    uplink = 0.3
    state, t_send = _state_with_commands(uplink)
    target = int(2.0 * NS)
    w = state.window(target)
    row = w.joints[0]
    path = row[2:2 + 2 * L.N_PATH].reshape(L.N_PATH, 2)
    grid = target + int(L.PATH_AHEAD_S * NS) - np.round(np.arange(L.N_PATH) * L.PATH_STEP_S * NS).astype(np.int64)
    eff = ((t_send + uplink) * NS).astype(np.int64)
    for i, g in enumerate(grid):
        k = np.searchsorted(eff, g, side="right") - 1
        if k < 0:
            assert path[i, 1] == 0.0
            continue
        assert path[i, 1] == 1.0
        expected = np.arcsinh((0.01 * k + 0.001 * 1 - 0.0) / L.ANGLE_SCALE_RAD)
        assert path[i, 0] == pytest.approx(expected, abs=1e-5)
    gap = (grid >= target - int(0.5 * NS)) & (grid < target - int(0.26 * NS))
    assert gap.sum() > 5 and (path[gap, 1] == 1.0).all()


def test_the_network_block_is_the_forecast_or_nothing():
    state, _ = _state_with_commands(0.05)
    target = int(2.0 * NS)
    lo, hi = L.BASE_ARM_FEATURES, L.BASE_ARM_FEATURES + L.NETWORK_FEATURES
    assert not state.window(target).arm[lo:hi].any()
    stream = F.ForecastStream.empty("oracle", np.array([int(1.9 * NS)]))
    stream.q_ms[0] = 120.0
    stream.loss[0] = 0.02
    stream.floor_ms[0] = 100.0
    stream.valid_q[0] = stream.valid_loss[0] = stream.valid_outage[0] = True
    state.observe_forecast(stream, 0)
    block = state.window(target).arm[lo:hi]
    assert block[0] == pytest.approx(np.arcsinh(1.2), abs=1e-6) and block[4] == pytest.approx(0.02)
    assert block[-1] == pytest.approx(0.1 / L.TIME_SCALE_S)  # its age
    assert (block[-7:-1] == 1.0).all()  # the six validity flags
    state.clear_forecast()
    assert not state.window(target).arm[lo:hi].any()


def test_a_shard_built_for_another_layout_is_refused(tmp_path):
    path = tmp_path / "train.npz"
    np.savez(path, view=np.zeros(0, np.int32))
    (tmp_path / "train.json").write_text(json.dumps({"views": [], "samples": 0, "aware": False}))
    with pytest.raises(ValueError, match="layout v1"):
        load_shards([str(path)], layout="v3")
