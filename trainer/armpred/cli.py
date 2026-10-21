"""``armpred``: the trainer's one entry point (trainer/README.md has the workflow)."""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time
from dataclasses import asdict, replace
from typing import List, Optional

import numpy as np

from . import MODEL_FORMAT_VERSION, SESSION_FORMAT_VERSION, VIEW_FORMAT_VERSION, __version__


def _log_to(path: str):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    f = open(path, "a", encoding="utf-8")

    def log(message: str) -> None:
        line = f"[{time.strftime('%H:%M:%S')}] {message}"
        print(line, flush=True)
        f.write(line + "\n")
        f.flush()

    return log


def cmd_check_session(args) -> int:
    from .contract.check import check_session_dir

    failed = 0
    for directory in args.sessions:
        report = check_session_dir(directory, allow_read_gaps=args.allow_read_gaps)
        print("\n".join(report.lines()))
        if args.json:
            with open(os.path.join(directory, "check.json"), "w", encoding="utf-8") as f:
                json.dump({"ok": report.ok, "failures": report.failures, "warnings": report.warnings,
                           "read_gaps": report.read_gaps, "stats": report.stats}, f, indent=2, default=str)
        failed += 0 if report.ok else 1
    return 1 if failed else 0


def cmd_simulate(args) -> int:
    from .sim.params import sample_params
    from .sim.robot import simulate_session, write_session

    for k in range(args.sessions):
        seed = args.seed + k
        params = sample_params(seed, duration_s=args.minutes * 60.0)
        d = os.path.join(args.out, f"sim-{seed:010d}")
        write_session(simulate_session(params), d)
        print(d)
    return 0


def _train_cfg(section: dict):
    from .train import TrainConfig

    return TrainConfig(**section) if section else TrainConfig()


def _prior_ages(config: dict) -> Optional[tuple]:
    """``[prior] sensed_ages_ms = [...]`` or ``[prior] file = "<sysid out>/prior.json"``; None is the
    headset's defaults."""
    prior = config.get("prior", {})
    if "sensed_ages_ms" in prior:
        return tuple(float(x) for x in prior["sensed_ages_ms"])
    if "file" in prior:
        with open(prior["file"], "r", encoding="utf-8") as f:
            return tuple(float(x) for x in json.load(f)["sensedAgeMs"])
    return None


def _prepare(config: dict, data_dir: str, workers: int, log) -> dict:
    """Simulate the session matrix and build the train/val shards into ``data_dir``, once: every
    training run of a grid then reads the same shards (and the same test sessions)."""
    from .data.build import BuildConfig
    from .data.dataset import SampleConfig
    from .data.schedule import RandomScheduleConfig
    from .pipeline import TRACES, MatrixConfig, build_splits, simulate_matrix
    from .sim.params import SimRanges

    seed = int(config.get("seed", 1))
    matrix = MatrixConfig(**{k: tuple(v) if isinstance(v, list) else v for k, v in config.get("matrix", {}).items()})
    ranges = SimRanges(**{k: tuple(v) for k, v in config.get("ranges", {}).items()})
    build_cfg = BuildConfig(sample=SampleConfig(**config.get("sample", {})),
                            schedules=RandomScheduleConfig(**{k: tuple(v) if isinstance(v, list) else v
                                                             for k, v in config.get("schedules", {}).items()}),
                            aware=bool(config.get("train", {}).get("aware", False)), traces_dir=TRACES, workers=workers,
                            sensed_ages_ms=_prior_ages(config))
    dirs = simulate_matrix(matrix, ranges, seed, data_dir, workers=workers, log=log)
    paths = build_splits(dirs, data_dir, int(config.get("views_per_session", 8)), int(config.get("val_views", 2)),
                         build_cfg, seed, log=log)
    with open(os.path.join(data_dir, "prepared.json"), "w", encoding="utf-8") as f:
        json.dump({"dirs": dirs, "paths": paths, "aware": build_cfg.aware, "sensedAgesMs": build_cfg.sensed_ages_ms}, f, indent=2)
    return {"dirs": dirs, "paths": paths}


def _copy_config(path: str, out: str) -> None:
    os.makedirs(out, exist_ok=True)
    with open(path, "rb") as src, open(os.path.join(out, "config.toml"), "wb") as dst:
        dst.write(src.read())


def _apply_sets(config: dict, sets: List[str]) -> dict:
    """``--set train.arch=conv``: a TOML value (number, bool, string) at a dotted key."""
    import tomllib

    for item in sets or []:
        key, _, raw = item.partition("=")
        try:
            value = tomllib.loads(f"v = {raw}")["v"]
        except tomllib.TOMLDecodeError:
            value = raw
        node = config
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return config


def _train_run(config: dict, data_dir: str, out: str, workers: int, log, steps: int = 0, skip_eval: bool = False) -> int:
    from .data.build import load_shards
    from .pipeline import trainer_version
    from .train import train

    with open(os.path.join(data_dir, "prepared.json"), "r", encoding="utf-8") as f:
        prepared = json.load(f)
    cfg = _train_cfg(config.get("train", {}))
    if steps:
        cfg = replace(cfg, steps=steps, val_every=max(1, min(cfg.val_every, steps)))
    if cfg.aware != prepared["aware"]:
        raise SystemExit(f"the shards in {data_dir} were built with aware={prepared['aware']}, the run asks aware={cfg.aware}")
    train_ds, val_ds = load_shards([prepared["paths"]["train"]]), load_shards([prepared["paths"]["val"]])
    log(f"armpred {__version__} train-run -> {out} (trainer {trainer_version()}); data {data_dir}")
    log(f"training {cfg.arch} on {len(train_ds)} samples ({len(val_ds)} val), {cfg.steps} steps, batch {cfg.batch}")
    summary = train(train_ds, val_ds, cfg, out, log=log)
    log(f"best val p95 {summary['best']['p95']:.3f} deg at step {summary['best']['step']} "
        f"(prior {summary['val_prior']['p95']:.3f}, hold-last {summary['val_hold_last']['p95']:.3f})")
    ages = prepared.get("sensedAgesMs")
    _export_run(out, cfg, val_ds, data_hashes={k: _sha(v) for k, v in prepared["paths"].items()}, log=log, source="sim",
                sensed_ages_ms=ages)
    if not skip_eval:
        _evaluate_run(out, prepared["dirs"]["test"], cfg, log=log, workers=workers, simulated=True, sensed_ages_ms=ages)
    return 0


def cmd_prepare(args) -> int:
    from .pipeline import load_config

    config = _apply_sets(load_config(args.config), args.set)
    _copy_config(args.config, args.out)
    _prepare(config, args.out, args.workers, _log_to(os.path.join(args.out, "log.txt")))
    return 0


def cmd_train_run(args) -> int:
    from .pipeline import load_config

    config = _apply_sets(load_config(args.config), args.set)
    _copy_config(args.config, args.out)
    with open(os.path.join(args.out, "overrides.json"), "w", encoding="utf-8") as f:
        json.dump(args.set or [], f)
    return _train_run(config, args.data, args.out, args.workers, _log_to(os.path.join(args.out, "log.txt")),
                      steps=args.steps, skip_eval=args.skip_eval)


def cmd_sim_pretrain(args) -> int:
    from .pipeline import load_config

    config = _apply_sets(load_config(args.config), args.set)
    _copy_config(args.config, args.out)
    log = _log_to(os.path.join(args.out, "log.txt"))
    _prepare(config, args.out, args.workers, log)
    return _train_run(config, args.out, args.out, args.workers, log, steps=args.steps, skip_eval=args.skip_eval)


def grid_tasks(grid: dict) -> List[List[str]]:
    """Every combination of a grid file's axes, as ``--set`` lists, in a fixed order."""
    import itertools

    axes = grid.get("axes", {})
    keys = sorted(axes)
    out = []
    for values in itertools.product(*(axes[k] for k in keys)):
        out.append([f"{k}={json.dumps(v) if isinstance(v, str) else str(v).lower() if isinstance(v, bool) else v}"
                    for k, v in zip(keys, values)])
    return out


def cmd_grid(args) -> int:
    from .pipeline import load_config

    grid = load_config(args.grid)
    tasks = grid_tasks(grid)
    if args.list:
        for i, t in enumerate(tasks):
            print(i, " ".join(t))
        return 0
    if not (0 <= args.index < len(tasks)):
        print(f"index {args.index} outside the grid's {len(tasks)} tasks", file=sys.stderr)
        return 2
    sets = tasks[args.index]
    name = "__".join(s.replace("train.", "").replace("=", "-").replace('"', "") for s in sets)
    out = os.path.join(args.out, f"{args.index:03d}__{name}")
    config = _apply_sets(load_config(args.config), sets)
    _copy_config(args.config, out)
    with open(os.path.join(out, "overrides.json"), "w", encoding="utf-8") as f:
        json.dump(sets, f)
    return _train_run(config, args.data, out, args.workers, _log_to(os.path.join(out, "log.txt")), steps=args.steps)


def _sha(path: str) -> str:
    from .export import sha256

    return sha256(path)


def _export_run(out: str, cfg, val_ds, data_hashes: dict, log, source: str, sensed_ages_ms=None) -> dict:
    from .export import export_onnx, flat_inputs
    from .features import layout as L
    from .pipeline import trainer_version
    from .train import load_best

    model = load_best(out, cfg)
    # Parity on real windows: rebuild full five-joint windows from validation rows is not possible
    # from per-joint rows, so tile one row per joint; every value is a real feature value.
    rows = val_ds.samples.joints[: 64 * 5].reshape(-1, 5, L.JOINT_FEATURES) if len(val_ds) >= 320 else None
    arms = val_ds.samples.arm[: rows.shape[0]] if rows is not None else None
    parity = flat_inputs(rows, arms) if rows is not None else None
    version = trainer_version()
    info = export_onnx(model, cfg.aware, os.path.join(out, "model.onnx"), {
        "teleop.arch": cfg.arch,
        "teleop.trainer_sha": str(version.get("sha")),
        "teleop.trainer_dirty": str(version.get("dirty")),
        "teleop.data_sha256": json.dumps(data_hashes, sort_keys=True),
        "teleop.source": source,
    }, parity_inputs=parity)
    summary = json.load(open(os.path.join(out, "train-summary.json"), "r", encoding="utf-8"))
    card = {
        "format": "jetrover-model-card", "version": MODEL_FORMAT_VERSION,
        "model": {"path": "model.onnx", "sha256": info["sha256"], "opset": info["opset"], "ops": info["ops"],
                  "inputLength": info["input_length"], "outputLength": 10, "layout": info["metadata"]["teleop.layout_id"]},
        "prior": {"predictor": "relay-replay", "speed": "aware" if cfg.aware else "blind",
                  "config": "headset" if not sensed_ages_ms else "headset+fitted-ages",
                  "sensedAgeMotorIds": [1, 2, 3, 4, 5],
                  "sensedAgeMs": list(sensed_ages_ms) if sensed_ages_ms else [45.0, 35.0, 25.0, 15.0, 5.0]},
        "parity": {"maxAbsRad": info["parity_max_abs"], "rows": info["parity_rows"], "toleranceRad": 1e-5},
        "trainer": {"version": __version__, **version},
        "formats": {"session": SESSION_FORMAT_VERSION, "views": VIEW_FORMAT_VERSION, "model": MODEL_FORMAT_VERSION},
        "data": data_hashes,
        "source": source,
        "training": {k: summary[k] for k in ("config", "best", "val_prior", "val_hold_last", "train_samples", "val_samples", "device", "torch")},
        "notResult": "A model is not a result until Teleop.Eval has scored it (docs/adr/0021 decision 8).",
    }
    with open(os.path.join(out, "model-card.json"), "w", encoding="utf-8") as f:
        json.dump(card, f, indent=2)
    log(f"exported model.onnx (sha256 {info['sha256'][:12]}..., parity {info['parity_max_abs']:.2e} rad, ops {info['ops']})")
    return card


def _evaluate_one(args):
    from .contract.profile import JETROVER
    from .contract.session import load_session
    from .data.schedule import parse
    from .data.views import synthesize
    from .evaluate import default_predictors, evaluate_view
    from .pipeline import TRACES

    from .contract.profile import load_profile

    session_dir, schedule, seed, onnx_path, aware, profile_path, ages = args
    session = load_session(session_dir)
    profile = load_profile(profile_path) if profile_path else JETROVER
    view = synthesize(session, profile, parse(schedule), seed=seed, traces_dir=TRACES)
    return evaluate_view(view, default_predictors(onnx_path, view.tps, aware=aware, sensed_ages_ms=ages), seed=seed)


def _evaluate_run(out: str, test_dirs: List[str], cfg, log, workers: int = 0, simulated: bool = True,
                  profile_path: Optional[str] = None, sensed_ages_ms=None) -> dict:
    from concurrent.futures import ProcessPoolExecutor

    from .evaluate import bootstrap_p95_difference, summarize
    from .pipeline import TEST_SCHEDULES
    from .report import write_report

    jobs = [(d, s, 7 + i, os.path.join(out, "model.onnx"), cfg.aware, profile_path,
             tuple(sensed_ages_ms) if sensed_ages_ms else None)
            for i, d in enumerate(test_dirs) for s in TEST_SCHEDULES]
    log(f"evaluating {len(jobs)} views ({len(test_dirs)} test sessions x {len(TEST_SCHEDULES)} schedules)")
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=workers or os.cpu_count() or 1) as pool:
        results = list(pool.map(_evaluate_one, jobs, chunksize=1))
    summary = summarize(results)
    # G6's "two replays bit-identical": the first view, evaluated again from scratch.
    again = _evaluate_one(jobs[0])
    summary["deterministic"] = all(
        np.array_equal(results[0].errors[n], again.errors[n]) and np.array_equal(results[0].corrections[n], again.corrections[n])
        for n in results[0].errors)
    rows = [b for b in ("relay-replay", "relay-replay-blind", "relay-replay-adaptive", "relay-replay-fitted")
            if b in results[0].errors]
    summary["bootstrap"] = {s: {b: bootstrap_p95_difference(results, "seq-model", b, schedule=s) for b in rows}
                            for s in sorted({r.schedule for r in results})}
    summary["simulated"] = simulated
    os.makedirs(os.path.join(out, "eval"), exist_ok=True)
    with open(os.path.join(out, "eval", "report.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, default=float)
    card_path = os.path.join(out, "model-card.json")
    card = json.load(open(card_path, "r", encoding="utf-8")) if os.path.exists(card_path) else None
    write_report(summary, os.path.join(out, "eval", "report.md"), simulated=simulated, card=card)
    log(f"evaluation done ({time.time() - t0:.0f}s): {os.path.join(out, 'eval', 'report.md')}")
    return summary


def cmd_evaluate(args) -> int:
    from .train import TrainConfig

    cfg = TrainConfig(aware=args.aware)
    log = _log_to(os.path.join(args.run, "log.txt"))
    dirs = sorted(glob.glob(os.path.join(args.sessions, "*", "session.json")))
    _evaluate_run(args.run, [os.path.dirname(d) for d in dirs], cfg, log=log, workers=args.workers,
                  simulated=not args.real, profile_path=args.profile)
    return 0


def cmd_make_views(args) -> int:
    from .contract.profile import JETROVER, load_profile
    from .contract.session import load_session
    from .data.schedule import parse
    from .data.views import save_view, synthesize
    from .pipeline import TRACES

    profile = load_profile(args.profile) if args.profile else JETROVER
    written = 0
    for directory in args.sessions:
        session = load_session(directory)
        for spec in args.schedule:
            schedule = parse(spec)
            if not schedule.only_named:
                print(f"refusing {spec!r}: official views use named profiles only", file=sys.stderr)
                return 2
            for seed in args.seeds:
                view = synthesize(session, profile, schedule, seed=seed, traces_dir=TRACES)
                name = f"{session.id}__{spec.replace(';', '+').replace('@', '-at-')}__seed{seed}.npz"
                save_view(view, os.path.join(args.out, name))
                written += 1
    print(f"wrote {written} views to {args.out}")
    return 0


def _unpack_bundle(path: str, into: str, log) -> dict:
    """A data bundle (contract/bundle.py), unpacked once and verified file by file."""
    from .contract.bundle import BundleError, read_bundle

    try:
        index = read_bundle(path, into)
    except BundleError as e:
        raise SystemExit(str(e))
    log(f"bundle {path}: {len(index['sessions'])} session(s), verified")
    return index


def cmd_pack_dataset(args) -> int:
    """What ``just pack-dataset`` runs: checked sessions into one bundle (contract/bundle.py)."""
    from .contract.bundle import BundleError, write_bundle

    try:
        index = write_bundle(args.sessions, args.out, args.profile, args.repo_sha)
    except BundleError as e:
        print(f"refused: {e}", file=sys.stderr)
        return 1
    print(f"{args.out}: {len(index['sessions'])} session(s)")
    return 0


def _split(path: str, ids: List[str]) -> dict:
    """The pre-registered split (configs/split-*.toml): every session in exactly one of train, val,
    test. A session it does not name is an error: a split decided after looking is not a split."""
    import tomllib

    with open(path, "rb") as f:
        splits = tomllib.load(f).get("splits", {})
    out = {k: list(splits.get(k, [])) for k in ("train", "val", "test")}
    named = [i for k in out for i in out[k]]
    if len(named) != len(set(named)):
        raise SystemExit(f"{path}: a session is in more than one split")
    missing = sorted(set(ids) - set(named))
    if missing:
        raise SystemExit(f"{path} does not place these sessions: {missing}")
    return out


def cmd_finetune(args) -> int:
    import torch

    from .data.build import BuildConfig, ViewPlan, build, load_shards
    from .data.dataset import SampleConfig
    from .data.schedule import RandomScheduleConfig
    from .pipeline import TRACES, load_config, trainer_version
    from .train import train

    config = _apply_sets(load_config(args.config), args.set)
    out = args.out
    _copy_config(args.config, out)
    log = _log_to(os.path.join(out, "log.txt"))
    bundle_dir = os.path.join(out, "bundle")
    index = _unpack_bundle(args.data, bundle_dir, log)
    ids = [e["id"] for e in index["sessions"]]
    split = _split(args.split, ids)
    profile_path = os.path.join(bundle_dir, "robot-profile.json")
    cfg = _train_cfg(config.get("train", {}))
    init_state = None
    if args.init:
        init_cfg = _apply_sets(load_config(os.path.join(args.init, "config.toml")), json.load(open(os.path.join(args.init, "overrides.json")))
                               if os.path.exists(os.path.join(args.init, "overrides.json")) else [])
        init_train = _train_cfg(init_cfg.get("train", {}))
        if (init_train.arch, init_train.aware) != (cfg.arch, cfg.aware):
            raise SystemExit(f"--init {args.init} is a {init_train.arch} (aware={init_train.aware}); the config asks {cfg.arch} (aware={cfg.aware})")
        init_state = torch.load(os.path.join(args.init, "checkpoints", "best.pt"), map_location="cpu")
        log(f"starting from {args.init}")
    build_cfg = BuildConfig(sample=SampleConfig(**config.get("sample", {})),
                            schedules=RandomScheduleConfig(**{k: tuple(v) if isinstance(v, list) else v
                                                             for k, v in config.get("schedules", {}).items()}),
                            aware=cfg.aware, traces_dir=TRACES, workers=args.workers, sensed_ages_ms=_prior_ages(config))
    seed = int(config.get("seed", 1))
    paths = {}
    for name, n_views in (("train", int(config.get("views_per_session", 16))), ("val", int(config.get("val_views", 4)))):
        plans = [ViewPlan(os.path.join(bundle_dir, "sessions", sid), seed=seed * 1_000_003 + i * 131 + v, profile_path=profile_path)
                 for i, sid in enumerate(split[name]) for v in range(n_views)]
        if not plans:
            raise SystemExit(f"the split has no {name} sessions")
        paths[name] = os.path.join(out, "data", f"{name}.npz")
        if not os.path.exists(paths[name]):
            info = build(plans, build_cfg, paths[name])
            log(f"{name}: {info['samples']} samples from {len(plans)} views")
    train_ds, val_ds = load_shards([paths["train"]]), load_shards([paths["val"]])
    log(f"armpred {__version__} finetune -> {out} (trainer {trainer_version()}), {len(train_ds)} samples")
    summary = train(train_ds, val_ds, cfg, out, log=log, init_state=init_state)
    log(f"best val p95 {summary['best']['p95']:.3f} deg (prior {summary['val_prior']['p95']:.3f}, hold-last {summary['val_hold_last']['p95']:.3f})")
    data_hashes = {k: _sha(v) for k, v in paths.items()}
    data_hashes["bundle"] = _sha(args.data)
    ages = build_cfg.sensed_ages_ms
    _export_run(out, cfg, val_ds, data_hashes=data_hashes, log=log, source="sim+real" if args.init else "real",
                sensed_ages_ms=ages)
    if split["test"]:
        _evaluate_run(out, [os.path.join(bundle_dir, "sessions", sid) for sid in split["test"]], cfg, log=log,
                      workers=args.workers, simulated=False, profile_path=profile_path, sensed_ages_ms=ages)
    else:
        log("no test sessions in the split: record them after freezing this model, then `armpred evaluate --real`")
    return 0


def cmd_sysid(args) -> int:
    from . import sysid
    from .contract.profile import JETROVER, load_profile

    dirs = list(args.sessions or [])
    profile = load_profile(args.profile) if args.profile else JETROVER
    if args.data:
        index = _unpack_bundle(args.data, os.path.join(args.out, "bundle"), print)
        dirs += [os.path.join(args.out, "bundle", "sessions", e["id"]) for e in index["sessions"]]
        profile = load_profile(os.path.join(args.out, "bundle", "robot-profile.json"))
    if not dirs:
        print("no sessions: pass session directories or --data <bundle>", file=sys.stderr)
        return 2
    result = sysid.identify(dirs, profile, servo_seconds=args.servo_seconds)
    js, tm = sysid.write(result, args.out)
    print(f"reading ages (ms): " + ", ".join(f"motor {m} {a * 1000:.1f}" for m, a in result["readingAgesS"].items()))
    print(f"bus: read {result['bus']['read_ms']:.2f} +- {result['bus']['read_jitter_ms']:.2f} ms, "
          f"misses {result['bus']['read_miss_probability']:.2%}, silences {result['bus']['board_silence_per_hour']:.2f}/h")
    if "servoSpeedNote" in result:
        print("note:", result["servoSpeedNote"])
    print(f"wrote {js} and {tm}")
    return 0


def cmd_not_yet(args) -> int:
    # Root CLAUDE.md invariant 10: an unimplemented step exits non-zero, never a stub that passes.
    print(f"'{args.command}' is NOT IMPLEMENTED yet: it needs real recordings (docs/adr/0021, step D1). "
          "Do not treat this as a passing step.", file=sys.stderr)
    return 70


def cmd_selftest(args) -> int:
    import platform

    ok = True
    print(f"armpred {__version__}, Python {platform.python_version()}, numpy {np.__version__}")
    try:
        import torch

        print(f"torch {torch.__version__}, CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            name = torch.cuda.get_device_name(0)
            cap = torch.cuda.get_device_capability(0)
            print(f"GPU 0: {name}, compute capability {cap[0]}.{cap[1]}, built for {torch.cuda.get_arch_list()}")
            if f"sm_{cap[0]}{cap[1]}" not in torch.cuda.get_arch_list():
                print(f"FAIL: this torch build has no kernels for sm_{cap[0]}{cap[1]} (a 5090 needs a CUDA 12.8+ build)")
                ok = False
            x = torch.randn(4096, 190, device="cuda")
            lin = torch.nn.Linear(190, 2).cuda()
            lin(x).sum().backward()
            torch.cuda.synchronize()
            print("tiny GPU training step: ok")
    except ImportError:
        print("torch is not installed: check-session and make-views work; training does not")
    try:
        import pytest

        code = pytest.main(["-q", "-p", "no:cacheprovider", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tests")])
        ok = ok and code == 0
    except ImportError:
        print("pytest is not installed: tests not run")
        ok = False
    print("selftest:", "OK" if ok else "FAILED")
    return 0 if ok else 1


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="armpred", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("selftest", help="check the environment (torch, CUDA arch) and run the tests")
    p.set_defaults(func=cmd_selftest)

    p = sub.add_parser("check-session", help="is a session fit to train or test on? (just recording-check)")
    p.add_argument("sessions", nargs="+")
    p.add_argument("--allow-read-gaps", action="store_true")
    p.add_argument("--json", action="store_true", help="also write check.json into each session")
    p.set_defaults(func=cmd_check_session)

    p = sub.add_parser("simulate", help="write simulated sessions (session v1)")
    p.add_argument("--out", required=True)
    p.add_argument("--sessions", type=int, default=1)
    p.add_argument("--minutes", type=float, default=10.0)
    p.add_argument("--seed", type=int, default=1)
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("sim-pretrain", help="prepare + train-run in one directory (a sim check)")
    p.add_argument("--config", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--set", action="append", help="override a config value, e.g. train.arch=conv")
    p.add_argument("--workers", type=int, default=0)
    p.add_argument("--steps", type=int, default=0, help="override train.steps (smoke runs)")
    p.add_argument("--skip-eval", action="store_true")
    p.set_defaults(func=cmd_sim_pretrain)

    p = sub.add_parser("prepare", help="simulate the session matrix and build the shards, once")
    p.add_argument("--config", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--set", action="append")
    p.add_argument("--workers", type=int, default=0)
    p.set_defaults(func=cmd_prepare)

    p = sub.add_parser("train-run", help="train, export and evaluate on prepared data")
    p.add_argument("--config", required=True)
    p.add_argument("--data", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--set", action="append")
    p.add_argument("--workers", type=int, default=0)
    p.add_argument("--steps", type=int, default=0)
    p.add_argument("--skip-eval", action="store_true")
    p.set_defaults(func=cmd_train_run)

    p = sub.add_parser("grid", help="one task of a grid (a Slurm array index), on prepared data")
    p.add_argument("--config", required=True)
    p.add_argument("--grid", required=True)
    p.add_argument("--data", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--list", action="store_true", help="print the tasks and exit")
    p.add_argument("--workers", type=int, default=0)
    p.add_argument("--steps", type=int, default=0)
    p.set_defaults(func=cmd_grid)

    p = sub.add_parser("evaluate", help="the trainer gate's numbers for a run, on a directory of sessions")
    p.add_argument("--run", required=True)
    p.add_argument("--sessions", required=True)
    p.add_argument("--aware", action="store_true")
    p.add_argument("--real", action="store_true", help="the sessions are recordings, not simulations")
    p.add_argument("--profile", help="the robot profile the sessions were recorded with (a bundle's robot-profile.json)")
    p.add_argument("--workers", type=int, default=0)
    p.set_defaults(func=cmd_evaluate)

    p = sub.add_parser("make-views", help="official views (named profiles only) for Teleop.Eval to score")
    p.add_argument("sessions", nargs="+")
    p.add_argument("--schedule", action="append", required=True)
    p.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    p.add_argument("--profile")
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_make_views)

    p = sub.add_parser("pack-dataset", help="pack checked sessions into one data bundle (just pack-dataset)")
    p.add_argument("sessions", nargs="+")
    p.add_argument("--out", required=True)
    p.add_argument("--profile", required=True, help="the robot profile they were recorded with")
    p.add_argument("--repo-sha", default="unknown")
    p.set_defaults(func=cmd_pack_dataset)

    p = sub.add_parser("finetune", help="train on a real data bundle (just pack-dataset), from a pretrained run")
    p.add_argument("--data", required=True, help="the bundle .tar")
    p.add_argument("--split", required=True, help="the pre-registered split, e.g. configs/split-v1.toml")
    p.add_argument("--config", required=True)
    p.add_argument("--init", help="a run directory to start from (its checkpoints/best.pt)")
    p.add_argument("--out", required=True)
    p.add_argument("--set", action="append")
    p.add_argument("--workers", type=int, default=0)
    p.set_defaults(func=cmd_finetune)

    p = sub.add_parser("sysid", help="fit the simulator's ranges and the reading ages to recordings")
    p.add_argument("sessions", nargs="*")
    p.add_argument("--data", help="a data bundle instead of (or as well as) session directories")
    p.add_argument("--profile", help="the robot profile (a bundle carries its own)")
    p.add_argument("--out", required=True)
    p.add_argument("--servo-seconds", type=float, default=300.0, help="how much of each session the servo fit uses")
    p.set_defaults(func=cmd_sysid)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
