"""The FR3 Duo's trainer pieces (docs/adr/0028): the simulator writes session v2 that the reader and the checker
take, deterministically; views and layout v4 samples come out of it; the v4 export's NumPy forward pass is the graph's."""

from __future__ import annotations

import os

import numpy as np
import pytest

from armpred.contract.chain_profile import load_chain_profile, start_pose
from armpred.contract.check import check_session_dir
from armpred.contract.session import load_session
from armpred.data.dataset import SampleConfig, view_samples
from armpred.data.schedule import parse
from armpred.data.views import synthesize
from armpred.features import layout_v4 as L4
from armpred.sim.fr3 import FR3_PROFILE, sample_fr3_params, simulate_fr3_session, write_fr3_session


@pytest.fixture(scope="module")
def session_dir(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("fr3") / "s")
    p = sample_fr3_params(5, duration_s=20.0, payload_grams=1000.0, payload_changes_per_min=6.0, speed=1.0)
    write_fr3_session(simulate_fr3_session(p), d)
    return d


def test_the_profile_is_two_seven_joint_arms_with_hands():
    p = load_chain_profile(FR3_PROFILE)
    assert p.motors == [1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18]
    assert p.hand_motors == [8, 18]
    # The two mounts tilt the arms apart: gravity points differently in each base.
    g0, g1 = p.arms[0].gravity_in_base(), p.arms[1].gravity_in_base()
    assert abs(np.linalg.norm(g0) - 1) < 1e-12 and not np.allclose(g0, g1)
    assert start_pose(p)[:7] == pytest.approx(list(np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])))


def test_a_simulated_session_is_a_checked_v2_session(session_dir):
    s = load_session(session_dir)
    assert s.version == 2 and s.meta["format"] == "teleop-session" and s.source == "sim"
    assert s.commands.motor.shape[1] == 16 and len(s.commands) > 800
    assert s.reads.value is not None and set(np.unique(s.reads.motor)) == set(load_chain_profile(FR3_PROFILE).motors)
    report = check_session_dir(session_dir, allow_read_gaps=True)
    assert report.ok, report.lines()
    # The truth is spaced by the arm's own 4 ms steps.
    t = s.reads.t_req[s.reads.motor == 1]
    assert set(np.diff(t)) == {4_000_000}


def test_the_simulator_is_deterministic(tmp_path):
    p = sample_fr3_params(9, duration_s=5.0)
    a, b = simulate_fr3_session(p), simulate_fr3_session(p)
    assert a.robothost == b.robothost and a.operator == b.operator


def test_an_unknown_payload_pulls_the_arm_off_its_target():
    from dataclasses import replace

    base = sample_fr3_params(3, duration_s=8.0, payload_grams=0.0, speed=0.3)
    heavy = replace(base, payload_grams=3000.0)
    q0 = simulate_fr3_session(base).tracked
    q1 = simulate_fr3_session(heavy).tracked
    # Same commands and seed: only the payload's bias differs, by a fraction of a degree to a few degrees.
    gap = np.abs(q1 - q0)[[1, 3, 9, 11]]
    assert (gap > 1e-4).all() and (gap < 0.1).all(), gap


def test_layout_v4_samples_from_a_view(session_dir):
    view = synthesize(load_session(session_dir), None, parse("50ms-5j@0"), seed=2)
    s = view_samples(view, np.random.default_rng(1), SampleConfig(queries_per_s=5.0), layout=L4)
    assert len(s) > 20
    assert s.joints.shape[1] == L4.JOINT_FEATURES == 253 and s.arm.shape[1] == L4.ARM_FEATURES == 125
    assert s.valid.mean() > 0.9
    # The identity flags: one per row, the hand's at HAND_FLAG_INDEX.
    ident = s.joints[:, L4.HAND_FLAG_INDEX - 7:L4.HAND_FLAG_INDEX + 1]
    assert np.all(ident.sum(axis=1) == 1)
    hand = np.isin(np.array(L4.predicted_motors())[s.joint], [8, 18])
    assert np.all(s.joints[hand, L4.HAND_FLAG_INDEX] == 1) and np.all(s.joints[~hand, L4.HAND_FLAG_INDEX] == 0)


def test_the_v4_export_forward_pass_is_the_graphs(tmp_path, session_dir):
    pytest.importorskip("onnxruntime")
    torch = pytest.importorskip("torch")
    from armpred.export_v4 import arm_of_row, export_onnx_v4, mlp_forward_v4, mlp_layers_from_onnx_v4, write_weights_v2
    from armpred.models.net import build

    torch.manual_seed(0)
    model = build("mlp", layout="v4")
    for p in model.parameters():
        torch.nn.init.normal_(p, std=0.05)
    rows = list(arm_of_row())
    x = np.random.default_rng(0).normal(0, 1, size=(8, 16 * L4.JOINT_FEATURES + 2 * L4.ARM_FEATURES)).astype(np.float32)
    x[:, [r * L4.JOINT_FEATURES + L4.HAND_FLAG_INDEX for r in range(16)]] = [1.0 if r in (7, 15) else 0.0 for r in range(16)]
    info = export_onnx_v4(model, str(tmp_path / "m.onnx"), {}, x)
    assert info["parity_max_abs"] < 1e-5 and info["input_length"] == 4298
    layers = mlp_layers_from_onnx_v4(info["path"])
    with torch.no_grad():
        from armpred.models.net import FlatExportV4

        flat = FlatExportV4(model, L4.JOINT_FEATURES, L4.ARM_FEATURES, tuple(rows), 2)
        want = np.concatenate([flat(torch.from_numpy(r[None, :])).numpy() for r in x])
    assert np.abs(mlp_forward_v4(layers, x, rows, 2) - want).max() < 1e-5
    w = write_weights_v2(info["path"], str(tmp_path / "m.weights"), "test", x)
    with open(w["path"], "rb") as f:
        head = f.read(28)
    assert head[:4] == b"TLMW" and int.from_bytes(head[4:8], "little") == 2
