"""The learned predictor's golden fixture for the repo's C# copy (docs/adr/0025): one real view's predictor
calls, and at every display tick the prediction, with the full window and the model's output at a spread of
them. The C# window (core/Teleop.Core/Prediction/LearnedJointWindow.cs) and backend (Inference/MlpBackend.cs)
must reproduce them. This is the reverse of contract/golden/: here the trainer is the source.

Written by ``armpred golden-learned`` into ``core/testdata/learned/``, never by hand."""

from __future__ import annotations

import json
import os
from typing import Dict, List

import numpy as np

from .baselines.types import headset_joint_config
from .features import layout as L

FORMAT = "teleop-learned-golden"
VERSION = 1


def _f(x) -> float:
    """A float32 value as JSON: its exact double, so the C# cast back to float is exact."""
    return float(np.float32(x))


def write_golden(run_dir: str, session_dir: str, profile_path: str, out_dir: str, schedule: str = "lan@0", seed: int = 7,
                 start_s: float = 60.0, seconds: float = 8.0, windows: int = 16, log=print) -> Dict[str, object]:
    from .contract.profile import load_profile
    from .contract.session import load_session
    from .data.dataset import CALL_COMMAND, CALL_STATE, call_stream, command_samples, state_samples
    from .data.schedule import parse
    from .data.views import synthesize
    from .evaluate import ModelPredictor
    from .export import write_weights

    with open(os.path.join(run_dir, "model-card.json"), "r", encoding="utf-8") as f:
        card = json.load(f)
    if card["model"]["layout"] != L.LAYOUT_ID or card["training"]["config"]["aware"]:
        raise SystemExit("golden-learned is for layout v3, speed-blind models")
    os.makedirs(out_dir, exist_ok=True)
    name = os.path.basename(os.path.normpath(run_dir))
    onnx_path = os.path.join(run_dir, "model.onnx")
    weights_name = f"{name}.weights"
    ages = card["prior"]["sensedAgeMs"]

    view = synthesize(load_session(session_dir), load_profile(profile_path), parse(schedule), seed)
    tps = int(view.tps)
    prior_config = headset_joint_config(tps, sensed_ages_ms=tuple(ages))
    model = ModelPredictor(onnx_path, prior_config, aware=False)
    model.reset()
    calls = call_stream(view)
    t_first = int(view.st_t[0])
    t0 = t_first + int(start_s * tps)
    t1 = t0 + int(seconds * tps)
    frame = tps / 90.0
    query_ticks = [t0 + int(round(k * frame)) for k in range(int(seconds * 90))]
    window_at = set(np.linspace(0, len(query_ticks) - 1, windows).round().astype(int).tolist())

    events: List[dict] = []
    golden_windows: List[dict] = []
    check: List[np.ndarray] = []  # every valid window: what the weights are checked against
    qi = 0
    ci = next(i for i, c in enumerate(calls) if c[0] >= t0)
    while qi < len(query_ticks):
        if ci < len(calls) and calls[ci][0] <= query_ticks[qi]:
            t, what, i = calls[ci]
            ci += 1
            if what == CALL_COMMAND:
                targets = command_samples(view, i)
                eff, speed = int(view.cmd_eff[i]), float(view.cmd_speed[i])
                model.observe_command(eff, targets, speed)
                events.append({"k": "c", "t": eff, "s": _f(speed), "m": [s.motor_id for s in targets],
                               "v": [_f(s.value) for s in targets]})
            elif what == CALL_STATE:
                joints = state_samples(view, i)
                sensed = int(view.st_sensed[i])
                model.observe_state(sensed, joints)
                events.append({"k": "s", "t": sensed, "m": [s.motor_id for s in joints], "v": [_f(s.value) for s in joints],
                               "f": [1 if s.sensed else 0 for s in joints]})
            continue
        target = query_ticks[qi]
        p, fallback = model.predict(target)
        w = model.state.window(target)
        if w.valid:
            check.append(np.concatenate([w.joints.reshape(-1), w.arm]).astype(np.float32))
        q = {"k": "q", "t": target, "fb": bool(fallback),
             "p": None if p is None else [None if not np.isfinite(v) else float(v) for v in p]}
        if qi in window_at:
            x = np.concatenate([w.joints.reshape(-1), w.arm]).astype(np.float32)
            y = model.session.run(["y"], {"x": x.reshape(1, -1)})[0].reshape(-1)
            q["w"] = len(golden_windows)
            golden_windows.append({"valid": bool(w.valid), "x": [_f(v) for v in x], "y": [_f(v) for v in y],
                                   "prior": [float(v) for v in w.prior]})
        events.append(q)
        qi += 1

    if not check:
        raise SystemExit("no valid window in the golden's span: start it later")
    weights = write_weights(onnx_path, os.path.join(out_dir, weights_name), f"{name} {card['model']['sha256'][:12]}",
                            check_inputs=np.array(check))
    c = prior_config
    golden = {
        "format": FORMAT, "version": VERSION,
        "source": {"run": name, "session": os.path.basename(os.path.normpath(session_dir)), "schedule": schedule, "seed": seed,
                   "startSeconds": start_s},
        "model": {"weights": weights_name, "weightsSha256": weights["sha256"], "onnxSha256": weights["onnxSha256"],
                  "modelId": weights["modelId"]},
        "config": {"ticksPerSecond": tps, "maxJoints": c.max_joints, "commandHistoryCapacity": c.command_history_capacity,
                   "maxJointSpeedRadPerSec": c.max_joint_speed_rad_per_sec, "maxHorizonTicks": int(c.max_horizon_ticks),
                   "scaledMotorIds": list(c.scaled_motor_ids), "speedScales": [_f(s) for s in c.speed_scales],
                   "sensedAgeMotorIds": list(c.sensed_age_motor_ids), "sensedAgeSeconds": [_f(s) for s in c.sensed_age_seconds]},
        "predictedMotors": list(L.PREDICTED_MOTORS),
        "gripperFractionPerRadian": L.GRIPPER_FRACTION_PER_RAD,
        "events": events,
        "windows": golden_windows,
    }
    path = os.path.join(out_dir, "golden-v3.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(golden, f, separators=(",", ":"))
    n_q = sum(1 for e in events if e["k"] == "q")
    n_fb = sum(1 for e in events if e["k"] == "q" and e["fb"])
    log(f"golden: {len(events)} calls ({n_q} predictions, {n_fb} fallbacks), {len(golden_windows)} windows -> {path}; "
        f"weights {weights_name} (parity {weights['parityMaxAbs']:.2e})")
    return {"path": path, "weights": weights}


def copy_weights(run_dir: str, out_path: str, golden_path: str) -> Dict[str, object]:
    """Only the weights, for shipping (e.g. into Bridge's Resources), checked on a golden's real windows."""
    from .export import write_weights

    with open(os.path.join(run_dir, "model-card.json"), "r", encoding="utf-8") as f:
        card = json.load(f)
    with open(golden_path, "r", encoding="utf-8") as f:
        windows = [w["x"] for w in json.load(f)["windows"] if w["valid"]]
    name = os.path.basename(os.path.normpath(run_dir))
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    return write_weights(os.path.join(run_dir, "model.onnx"), out_path, f"{name} {card['model']['sha256'][:12]}",
                         check_inputs=np.array(windows, dtype=np.float32))

