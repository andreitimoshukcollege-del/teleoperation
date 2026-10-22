"""The real-robot path end to end, on simulated sessions standing in for recordings: check, pack
into a bundle, verify on unpacking, fine-tune under a pre-registered split, export."""

from __future__ import annotations

import json
import os

import pytest

from armpred.contract.bundle import BundleError, read_bundle, write_bundle
from armpred.contract.check import check_session_dir
from armpred.sim.params import sample_params
from armpred.sim.robot import simulate_session, write_session

PROFILE = {
    "name": "jetrover", "hasRotatingBase": True, "baseHeight": 0.035, "proximalLinkLength": 0.13,
    "distalLinkLength": 0.13, "wristJointCount": 1, "hasGripper": True, "gripperCanRotate": True,
    "joints": [{"motorId": 1, "role": "BaseYaw"}, {"motorId": 2, "role": "Proximal", "zeroOffsetRadians": 0.12217305},
               {"motorId": 3, "role": "Distal"}, {"motorId": 4, "role": "Wrist", "wristIndex": 0},
               {"motorId": 10, "role": "GripperMain"}, {"motorId": 5, "role": "GripperRotate"}],
}


@pytest.fixture(scope="module")
def bundle(tmp_path_factory):
    root = tmp_path_factory.mktemp("bundle")
    sessions = []
    for seed in (31, 32, 33):
        d = str(root / "sessions" / f"s{seed}")
        write_session(simulate_session(sample_params(seed, duration_s=20.0, slider=2.5, payload_grams=0.0,
                                                     hand_events_per_min=0.0)), d)
        report = check_session_dir(d, allow_read_gaps=True)
        with open(os.path.join(d, "check.json"), "w", encoding="utf-8") as f:
            json.dump({"ok": report.ok, "failures": report.failures, "read_gaps": report.read_gaps,
                       "stats": report.stats}, f, default=str)
        sessions.append(d)
    profile = str(root / "jetrover.json")
    with open(profile, "w", encoding="utf-8") as f:
        json.dump(PROFILE, f)
    path = str(root / "bundle.tar")
    index = write_bundle(sessions, path, profile, "test-sha")
    return root, path, [e["id"] for e in index["sessions"]]


def test_a_session_that_failed_its_check_is_never_packed(bundle, tmp_path):
    root, _, _ = bundle
    d = str(root / "sessions" / "s31")
    with open(os.path.join(d, "check.json"), "r", encoding="utf-8") as f:
        good = f.read()
    try:
        with open(os.path.join(d, "check.json"), "w", encoding="utf-8") as f:
            json.dump({"ok": False, "failures": ["drops"]}, f)
        with pytest.raises(BundleError, match="check failed"):
            write_bundle([d], str(tmp_path / "x.tar"), str(root / "jetrover.json"), "sha")
    finally:
        with open(os.path.join(d, "check.json"), "w", encoding="utf-8") as f:
            f.write(good)


def test_a_damaged_bundle_is_refused(bundle, tmp_path):
    _, path, ids = bundle
    into = str(tmp_path / "unpacked")
    index = read_bundle(path, into)
    assert [e["id"] for e in index["sessions"]] == ids
    victim = os.path.join(into, "sessions", ids[0], "driver.jlog")
    with open(victim, "ab") as f:
        f.write(b"x")
    with pytest.raises(BundleError, match="does not match"):
        read_bundle(path, into)


def test_finetune_needs_every_session_placed_and_then_trains_and_exports(bundle, tmp_path):
    pytest.importorskip("torch")
    pytest.importorskip("onnxruntime")
    from armpred.cli import main

    _, path, ids = bundle
    config = tmp_path / "ft.toml"
    config.write_text('seed = 1\nviews_per_session = 1\nval_views = 1\n[sample]\nqueries_per_s = 20.0\n'
                      '[train]\narch = "mlp"\nsteps = 20\nbatch = 256\nwarmup = 5\nval_every = 10\ndevice = "cpu"\n')
    partial = tmp_path / "partial.toml"
    partial.write_text(f'[splits]\ntrain = ["{ids[0]}"]\nval = ["{ids[1]}"]\ntest = []\n')
    with pytest.raises(SystemExit, match="does not place"):
        main(["finetune", "--data", path, "--split", str(partial), "--config", str(config),
              "--out", str(tmp_path / "r0"), "--workers", "1"])
    split = tmp_path / "split.toml"
    split.write_text(f'[splits]\ntrain = ["{ids[0]}", "{ids[1]}"]\nval = ["{ids[2]}"]\ntest = []\n')
    out = tmp_path / "r1"
    assert main(["finetune", "--data", path, "--split", str(split), "--config", str(config), "--out", str(out),
                 "--workers", "1"]) == 0
    card = json.loads((out / "model-card.json").read_text())
    assert card["source"] == "real" and card["parity"]["maxAbsRad"] <= 1e-5
    assert (out / "model.onnx").exists() and "bundle" in card["data"]
