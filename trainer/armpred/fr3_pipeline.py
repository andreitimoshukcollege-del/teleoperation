"""``armpred fr3-sim-pretrain``: the FR3 Duo's simulated pretraining, end to end, into one ``runs/<name>/``
(docs/adr/0028). It runs the JetRover's ``sim-pretrain`` steps with the FR3's pieces:

1. simulate the session matrix (``sim/fr3.py``, session v2);
2. build views and layout v4 shards;
3. train the shared per-joint MLP on top of ``franka-replay``;
4. export the ``.onnx`` and ``model.weights`` format 2;
5. evaluate every predictor on the test sessions under the named schedules.

The report's title says SIMULATED, and it is never a gate verdict (ADR 0028 decision 5).
"""

from __future__ import annotations

import json
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, replace
from typing import Dict, List, Tuple

import numpy as np

from .contract.check import check_session_dir
from .data.schedule import random_uplink_schedule
from .pipeline import TEST_SCHEDULES, TRACES, build_splits, test_uplink_schedule, trainer_version
from .sim.fr3 import FR3_PROFILE, HELD_OUT_SPEED, Fr3SimRanges, sample_fr3_params, simulate_fr3_session, write_fr3_session


@dataclass(frozen=True)
class Fr3MatrixConfig:
    """Every speed x load cell, like the JetRover's matrix; a speed and a payload are held out of training."""

    speeds: Tuple[object, ...] = (0.3, 0.6, 1.0, 1.6, "varied")
    loads: Tuple[str, ...] = ("none", "payload-light", "payload-heavy", "payload-changing")
    sessions_per_cell: int = 3
    test_sessions_per_cell: int = 1
    minutes: float = 5.0
    uplink_schedule_share: float = 0.5
    test_uplink_schedule_share: float = 0.5
    val_share: float = 0.15
    light_payload_g: float = 500.0
    heavy_payload_g: float = 2000.0
    changing_payload_g: float = 1000.0
    held_out_payload_g: float = 1500.0


def _load_kw(load: str, m: Fr3MatrixConfig) -> Dict[str, float]:
    if load == "none":
        return {"payload_grams": 0.0, "payload_changes_per_min": 0.0}
    if load == "payload-light":
        return {"payload_grams": m.light_payload_g, "payload_changes_per_min": 0.0}
    if load == "payload-heavy":
        return {"payload_grams": m.heavy_payload_g, "payload_changes_per_min": 0.0}
    if load == "payload-held-out":
        return {"payload_grams": m.held_out_payload_g, "payload_changes_per_min": 2.0}
    return {"payload_grams": m.changing_payload_g, "payload_changes_per_min": 2.0}


def plan_fr3_sessions(m: Fr3MatrixConfig, seed: int) -> Dict[str, List[dict]]:
    rng = np.random.default_rng([seed, 0xF3])
    out: Dict[str, List[dict]] = {"train": [], "val": [], "test": []}
    next_seed = seed * 100_000
    for speed in m.speeds:
        for load in m.loads:
            for k in range(m.sessions_per_cell + m.test_sessions_per_cell):
                next_seed += 1
                split = "test" if k >= m.sessions_per_cell else ("val" if rng.random() < m.val_share else "train")
                r = rng.random()
                uplink = ("schedule:test" if r < m.test_uplink_schedule_share else "lan") if split == "test" else \
                    ("schedule:random" if r < m.uplink_schedule_share else "lan")
                out[split].append({"seed": next_seed, "speed": speed, "load": load, "uplink": uplink})
    for speed, load in ((HELD_OUT_SPEED, "none"), (HELD_OUT_SPEED, "payload-held-out"), (1.0, "payload-held-out")):
        next_seed += 1
        out["test"].append({"seed": next_seed, "speed": speed, "load": load, "uplink": "lan"})
    return out


def _uplink_kw(spec: dict, duration_s: float) -> Dict[str, str]:
    if spec["uplink"] == "schedule:random":
        rng = np.random.default_rng(np.random.PCG64([spec["seed"], 0x0091]))
        return {"uplink_schedule": random_uplink_schedule(duration_s, rng).to_json()}
    if spec["uplink"] == "schedule:test":
        return {"uplink_schedule": test_uplink_schedule(duration_s).to_json()}
    return {}


def _simulate_one(args) -> str:
    spec, m, ranges, directory = args
    params = sample_fr3_params(spec["seed"], ranges, duration_s=m.minutes * 60.0, speed=spec["speed"],
                               **_uplink_kw(spec, m.minutes * 60.0), **_load_kw(spec["load"], m))
    write_fr3_session(simulate_fr3_session(params), directory)
    report = check_session_dir(directory, allow_read_gaps=True)
    if not report.ok:
        raise RuntimeError("\n".join(report.lines()))
    return directory


def simulate_fr3_matrix(m: Fr3MatrixConfig, ranges: Fr3SimRanges, seed: int, out_dir: str, workers: int = 0, log=print) -> Dict[str, List[str]]:
    plan = plan_fr3_sessions(m, seed)
    jobs, dirs = [], {k: [] for k in plan}
    for split, specs in plan.items():
        for spec in specs:
            d = os.path.join(out_dir, "sessions", split, f"fr3sim-{spec['seed']:010d}")
            dirs[split].append(d)
            if not (os.path.exists(os.path.join(d, "session.json")) and check_session_dir(d, allow_read_gaps=True).ok):
                jobs.append((spec, m, ranges, d))
    log(f"simulating {len(jobs)} FR3 sessions ({len(dirs['train'])} train / {len(dirs['val'])} val / {len(dirs['test'])} test), "
        f"{m.minutes:g} min each")
    t0 = time.time()
    if jobs:
        with ProcessPoolExecutor(max_workers=workers or os.cpu_count() or 1) as pool:
            for k, _ in enumerate(pool.map(_simulate_one, jobs, chunksize=1)):
                if (k + 1) % max(1, len(jobs) // 10) == 0:
                    log(f"  {k + 1}/{len(jobs)} sessions ({time.time() - t0:.0f}s)")
    os.makedirs(os.path.join(out_dir, "sessions"), exist_ok=True)
    with open(os.path.join(out_dir, "sessions", "plan.json"), "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=2)
    return dirs


def _evaluate_one(args):
    from .contract.session import load_session
    from .data.schedule import parse
    from .data.views import synthesize
    from .evaluate_fr3 import LearnedV4, baselines, evaluate_view
    from .train import load_best

    session_dir, schedule, seed, out, cfg = args
    session = load_session(session_dir)
    view = synthesize(session, None, parse(schedule), seed=seed, traces_dir=TRACES)
    predictors = baselines(view.tps)
    if out is not None:
        predictors["learned"] = LearnedV4(load_best(out, cfg), view.tps)
    return evaluate_view(view, predictors)


def evaluate_fr3_run(out: str, test_dirs: List[str], cfg, workers: int, log=print, schedules=TEST_SCHEDULES, with_model: bool = True) -> dict:
    from .evaluate_fr3 import sim_gate_rows, summarize

    jobs = [(d, s, 1000 + i, out if with_model else None, cfg) for i, d in enumerate(test_dirs) for s in schedules]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=workers or os.cpu_count() or 1) as pool:
        results = list(pool.map(_evaluate_one, jobs, chunksize=1))
    summary = summarize(results)
    summary["gateDiagnostics"] = sim_gate_rows(summary)
    with open(os.path.join(out, "eval-fr3.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    log(f"evaluated {len(results)} views in {time.time() - t0:.0f}s")
    return summary


def write_report(out: str, summary: dict, train_summary: dict, card: dict) -> str:
    lines = ["# SIMULATED: FR3 Duo learned joint predictor (layout v4), simulated pretraining", "",
             "Every number below is from the trainer's FR3 simulator (`armpred/sim/fr3.py`). Its arm tracking is invented, "
             "and `franka-replay` is exact up to it, so none of this is the robot's and none of it is a gate verdict "
             "(docs/adr/0028 decision 5). A result exists only once `Teleop.Eval` has scored a model on real sessions into `results/`.", "",
             f"Model: `{card['model']['sha256'][:12]}`, {card['model']['inputLength']} in, {card['model']['outputLength']} out; "
             f"trained {train_summary['config']['steps']} steps on {train_summary['train_samples']} samples, best val p95 "
             f"{train_summary['best']['p95']:.3f} deg at step {train_summary['best']['step']} (prior {train_summary['val_prior']['p95']:.3f}, "
             f"hold-last {train_summary['val_hold_last']['p95']:.3f}).", "",
             "## Arm joints, joint_prediction_error_deg / joint_correction_deg (p50 / p95 / p99 / max)", "",
             "| schedule | predictor | err p50 | err p95 | err p99 | err max | cor p50 | cor p95 | cor max |", "|---|---|---|---|---|---|---|---|---|"]
    for sched, per in summary["bySchedule"].items():
        for name, m in per.items():
            e, c = m["joint_prediction_error_deg"], m["joint_correction_deg"]
            lines.append(f"| `{sched}` | `{name}` | {e['p50']:.3f} | {e['p95']:.3f} | {e['p99']:.3f} | {e['max']:.3f} | "
                         f"{c['p50']:.3f} | {c['p95']:.3f} | {c['max']:.3f} |")
    lines += ["", "## Hands, error in opening fraction (p95), never in the gate", "",
              "| schedule | " + " | ".join(sorted(next(iter(summary["bySchedule"].values())).keys())) + " |",
              "|---|" + "---|" * len(next(iter(summary["bySchedule"].values())))]
    for sched, per in summary["bySchedule"].items():
        lines.append(f"| `{sched}` | " + " | ".join(f"{per[n]['hand_error_fraction']['p95']:.3f}" for n in sorted(per)) + " |")
    lines += ["", "## The FR3 gate's F1-F3, read on simulated views (a diagnostic, not a verdict)", ""]
    lines += [f"- {row}" for row in summary.get("gateDiagnostics", [])]
    if summary.get("learnedInferenceUsMean") is not None:
        lines += ["", f"Learned predictor: {summary['learnedInferenceUsMean']:.0f} us per window in Python (window and torch; "
                      f"not the C# backend's F6 figure), fallback share {summary['learnedFallbackShare']:.4f}."]
    path = os.path.join(out, "report.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return path


def run_fr3_sim_pretrain(config: dict, out: str, workers: int, log, steps: int = 0, skip_eval: bool = False) -> int:
    from .cli import _forecast_cfg, _sha
    from .data.build import BuildConfig, load_shards
    from .data.dataset import SampleConfig
    from .data.schedule import RandomScheduleConfig
    from .export_v4 import export_onnx_v4, flat_inputs_v4, write_weights_v2
    from .features import layout_v4 as L4
    from .train import TrainConfig, load_best, train
    from . import __version__

    seed = int(config.get("seed", 1))
    matrix = Fr3MatrixConfig(**{k: tuple(v) if isinstance(v, list) else v for k, v in config.get("matrix", {}).items()})
    ranges = Fr3SimRanges(**{k: tuple(v) for k, v in config.get("ranges", {}).items()})
    build_cfg = BuildConfig(sample=SampleConfig(**config.get("sample", {})),
                            schedules=RandomScheduleConfig(**{k: tuple(v) if isinstance(v, list) else v for k, v in config.get("schedules", {}).items()}),
                            traces_dir=TRACES, workers=workers, forecast=_forecast_cfg(config), layout="v4")
    log(f"armpred {__version__} fr3-sim-pretrain -> {out} (trainer {trainer_version()})")
    dirs = simulate_fr3_matrix(matrix, ranges, seed, out, workers=workers, log=log)
    paths = build_splits(dirs, out, int(config.get("views_per_session", 4)), int(config.get("val_views", 1)), build_cfg, seed, log=log)
    with open(os.path.join(out, "prepared.json"), "w", encoding="utf-8") as f:
        json.dump({"dirs": dirs, "paths": paths, "aware": False, "layout": "v4", "forecast": asdict(build_cfg.forecast)}, f, indent=2)
    cfg = TrainConfig(**{**config.get("train", {}), "layout": "v4"})
    if steps:
        cfg = replace(cfg, steps=steps, val_every=max(1, min(cfg.val_every, steps)))
    train_ds, val_ds = load_shards([paths["train"]], layout="v4"), load_shards([paths["val"]], layout="v4")
    log(f"training mlp (layout v4) on {len(train_ds)} samples ({len(val_ds)} val), {cfg.steps} steps, batch {cfg.batch}")
    summary = train(train_ds, val_ds, cfg, out, log=log)
    log(f"best val p95 {summary['best']['p95']:.3f} deg at step {summary['best']['step']} "
        f"(prior {summary['val_prior']['p95']:.3f}, hold-last {summary['val_hold_last']['p95']:.3f})")

    # Export, with parity checked on real windows: rebuild whole windows from the first validation view.
    model = load_best(out, cfg)
    parity = _real_windows(dirs["val"][0] if dirs["val"] else dirs["train"][0], seed)
    info = export_onnx_v4(model, os.path.join(out, "model.onnx"), {"teleop.source": "sim", "teleop.robot_profile": "fr3-duo"}, parity)
    weights = write_weights_v2(info["path"], os.path.join(out, "model.weights"), f"{os.path.basename(out.rstrip('/'))} {info['sha256'][:12]}", parity)
    with open(FR3_PROFILE, "rb") as f:
        import hashlib

        profile_sha = hashlib.sha256(f.read()).hexdigest()
    rows = [a for _, a, _ in L4._rows(L4.default_profile())]
    card = {
        "format": "teleop-model-card", "version": 3,
        "model": {"path": "model.onnx", "sha256": info["sha256"], "opset": info["opset"], "ops": info["ops"],
                  "inputLength": info["input_length"], "outputLength": 2 * len(rows), "layout": "v4",
                  "joints": list(L4.default_profile().motors), "armOfRow": rows,
                  "handModelPerFraction": L4.HAND_MODEL_PER_FRACTION,
                  "caps": {"armRad": L4.MAX_CORRECTION_RAD, "hand": L4.HAND_MAX_CORRECTION}},
        "weights": weights,
        "robotProfile": {"name": "fr3-duo", "sha256": profile_sha},
        "prior": {"predictor": "franka-replay", "config": "fr3-duo shaper defaults"},
        "parity": {"maxAbsRad": info["parity_max_abs"], "rows": info["parity_rows"]},
        "trainer": trainer_version(),
        "data": {k: _sha(v) for k, v in paths.items()}, "source": "sim",
        "notes": "SIMULATED: trained on the trainer's FR3 simulator only; never the robot's (docs/adr/0028 decision 5)",
    }
    with open(os.path.join(out, "model-card.json"), "w", encoding="utf-8") as f:
        json.dump(card, f, indent=2)
    log(f"exported {info['path']} ({info['sha256'][:12]}), weights v2 {weights['sha256'][:12]}, parity {info['parity_max_abs']:.2e}")
    if skip_eval:
        return 0
    result = evaluate_fr3_run(out, dirs["test"], cfg, workers, log=log)
    report = write_report(out, result, summary, card)
    log(f"report: {report}")
    for row in result["gateDiagnostics"]:
        log("  " + row)
    return 0


def _real_windows(session_dir: str, seed: int, count: int = 64) -> np.ndarray:
    """``count`` whole v4 windows (every row and both arm blocks) from one session under the LAN, for parity checks."""
    from .contract.session import load_session
    from .data.dataset import CALL_COMMAND, CALL_STATE, call_stream, command_samples, state_samples
    from .data.schedule import parse
    from .data.views import synthesize
    from .export_v4 import flat_inputs_v4
    from .features import layout_v4 as L4

    view = synthesize(load_session(session_dir), None, parse("lan@0"), seed=seed, traces_dir=TRACES)
    state = L4.FeatureState(L4.default_prior_config(view.tps))
    calls = call_stream(view, None)
    targets = np.linspace(int(view.st_t[0]) + 3 * view.tps, int(view.st_t[-1]) - view.tps, count).astype(np.int64)
    joints, arms, ci = [], [], 0
    for t in targets:
        while ci < len(calls) and calls[ci][0] <= t:
            _, what, i = calls[ci]
            if what == CALL_COMMAND:
                state.observe_command(int(view.cmd_eff[i]), command_samples(view, i))
            elif what == CALL_STATE:
                state.observe_state(int(view.st_sensed[i]), state_samples(view, i))
            ci += 1
        w = state.window(int(t))
        joints.append(w.joints)
        arms.append(w.arms)
    return flat_inputs_v4(np.array(joints), np.array(arms))
