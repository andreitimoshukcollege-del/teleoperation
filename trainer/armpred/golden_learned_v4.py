"""Layout v4's golden fixture for the repo's C# copy (docs/adr/0028): one simulated FR3 view's predictor calls, and
at every display tick the prediction, with the full window and the model's output at a spread of them. The C# window
(core/Teleop.Core/Prediction/LearnedJointWindowV4.cs) and backend (Inference/MlpBackend.cs, weights format 2) must
reproduce them. As with ``golden_learned.py`` (layout v3), the trainer is the source.

Written by ``armpred golden-learned-v4`` into ``core/testdata/learned/``, never by hand. The view is simulated:
this pins arithmetic, not how good the model is."""

from __future__ import annotations

import json
import os
from typing import Dict, List

import numpy as np

from .features import layout_v4 as L4

FORMAT = "teleop-learned-golden"
VERSION = 2


def _f(x) -> float:
    return float(np.float32(x))


def write_golden_v4(run_dir: str, session_dir: str, out_dir: str, schedule: str = "50ms-5j@0", seed: int = 7, start_s: float = 30.0,
                    seconds: float = 6.0, windows: int = 16, log=print) -> Dict[str, object]:
    import onnxruntime as ort

    from .baselines.franka_replay import shaper_for_profile
    from .contract.session import load_session
    from .data.dataset import CALL_COMMAND, CALL_STATE, call_stream, command_samples, state_samples
    from .data.schedule import parse
    from .data.views import synthesize
    from .export_v4 import arm_of_row, flat_inputs_v4, write_weights_v2

    with open(os.path.join(run_dir, "model-card.json"), "r", encoding="utf-8") as f:
        card = json.load(f)
    if card["model"]["layout"] != L4.LAYOUT_ID:
        raise SystemExit("golden-learned-v4 is for layout v4 models")
    profile = L4.default_profile()
    name = os.path.basename(os.path.normpath(run_dir))
    onnx_path = os.path.join(run_dir, "model.onnx")
    weights_name = f"{name}.weights"
    os.makedirs(out_dir, exist_ok=True)

    view = synthesize(load_session(session_dir), None, parse(schedule), seed)
    tps = int(view.tps)
    config = L4.default_prior_config(tps, profile)
    state = L4.FeatureState(config, profile=profile)
    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    motors = [m for m, _, _ in L4._rows(profile)]

    calls = call_stream(view)
    t0 = int(view.st_t[0]) + int(start_s * tps)
    frame = tps / 90.0
    query_ticks = [t0 + int(round(k * frame)) for k in range(int(seconds * 90))]
    window_at = set(np.linspace(0, len(query_ticks) - 1, windows).round().astype(int).tolist())
    events: List[dict] = []
    golden_windows: List[dict] = []
    check: List[np.ndarray] = []
    qi = 0
    ci = next(i for i, c in enumerate(calls) if c[0] >= t0)
    while qi < len(query_ticks):
        if ci < len(calls) and calls[ci][0] <= query_ticks[qi]:
            _, what, i = calls[ci]
            ci += 1
            if what == CALL_COMMAND:
                targets = command_samples(view, i)
                eff = int(view.cmd_eff[i])
                state.observe_command(eff, targets)
                events.append({"k": "c", "t": eff, "s": 0.0, "m": [s.motor_id for s in targets], "v": [_f(s.value) for s in targets]})
            elif what == CALL_STATE:
                joints = state_samples(view, i)
                sensed = int(view.st_sensed[i])
                state.observe_state(sensed, joints)
                events.append({"k": "s", "t": sensed, "m": [s.motor_id for s in joints], "v": [_f(s.value) for s in joints],
                               "f": [1 if s.sensed else 0 for s in joints]})
            continue
        target = query_ticks[qi]
        w = state.window(target)
        ok, prior_out = state.prior.try_predict(target)
        x = flat_inputs_v4(w.joints[None], w.arms[None])[0]
        fallback = not w.valid
        p = None
        if ok:
            by = {s.motor_id: s.value for s in prior_out}
            if w.valid:
                y = session.run(["y"], {"x": x.reshape(1, -1)})[0].reshape(-1)
                check.append(x)
                by.update({m: L4.to_wire(m, w.prior[r] + float(y[2 * r]), profile) for r, m in enumerate(motors)})
            p = [float(np.float32(by[m])) if m in by else None for m in motors]
        q = {"k": "q", "t": target, "fb": bool(fallback), "p": p}
        if qi in window_at:
            y = session.run(["y"], {"x": x.reshape(1, -1)})[0].reshape(-1)
            q["w"] = len(golden_windows)
            golden_windows.append({"valid": bool(w.valid), "x": [_f(v) for v in x], "y": [_f(v) for v in y], "prior": [float(v) for v in w.prior]})
        events.append(q)
        qi += 1
    if not check:
        raise SystemExit("no valid window in the golden's span: start it later")
    weights = write_weights_v2(onnx_path, os.path.join(out_dir, weights_name), f"{name} {card['model']['sha256'][:12]}", np.array(check))
    shaper = shaper_for_profile(profile)
    golden = {
        "format": FORMAT, "version": VERSION, "layout": L4.LAYOUT_ID,
        "source": {"run": name, "session": os.path.basename(os.path.normpath(session_dir)), "schedule": schedule, "seed": seed,
                   "startSeconds": start_s, "simulated": True},
        "model": {"weights": weights_name, "weightsSha256": weights["sha256"], "onnxSha256": weights["onnxSha256"], "modelId": weights["modelId"]},
        "config": {"ticksPerSecond": tps, "maxJoints": config.max_joints, "commandHistoryCapacity": config.command_history_capacity,
                   "maxHorizonTicks": int(config.max_horizon_ticks), "instantMotorIds": list(config.instant_motor_ids)},
        "shaper": {"stepSeconds": shaper.step_seconds, "velocityScale": shaper.velocity_scale, "accelerationScale": shaper.acceleration_scale,
                   "jerkScale": shaper.jerk_scale, "brakingScale": shaper.braking_scale, "settleSeconds": shaper.settle_seconds,
                   "positionMargin": shaper.position_margin,
                   "motors": [{"motorId": j.motor_id, "isFinger": j.is_finger, "lower": j.lower, "upper": j.upper, "softLower": j.soft_lower,
                               "softUpper": j.soft_upper, "velocity": j.velocity, "acceleration": j.acceleration, "jerk": j.jerk,
                               "positionBasedDeceleration": j.position_based_deceleration,
                               "positionBasedVelocityOffset": j.position_based_velocity_offset, "fingerSpeed": j.finger_speed,
                               "fingerDeadband": j.finger_deadband} for j in shaper.joints]},
        "chain": {"motors": motors, "armOfRow": list(arm_of_row(profile)), "identity": [i for _, _, i in L4._rows(profile)],
                  "gravity": [float(g) for arm in profile.arms for g in arm.gravity_in_base()]},
        "events": events,
        "windows": golden_windows,
    }
    path = os.path.join(out_dir, "golden-v4.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(golden, f, separators=(",", ":"))
    n_q = sum(1 for e in events if e["k"] == "q")
    log(f"golden v4: {len(events)} calls ({n_q} predictions), {len(golden_windows)} windows -> {path}; weights {weights_name}")
    return {"path": path, "weights": weights}
