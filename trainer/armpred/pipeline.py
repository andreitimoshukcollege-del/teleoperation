"""The end-to-end runs: ``sim-pretrain`` and ``finetune``, each into one ``runs/<name>/`` directory.

A run directory holds everything needed to say what it was and to reproduce it: the config, the
sessions (simulated) or the bundle hash (real), the shards, the checkpoints, the exported model and
its card, and the evaluation report. A simulated run's report says so in its title.
"""

from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import time
import tomllib
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import MODEL_FORMAT_VERSION, SESSION_FORMAT_VERSION, VIEW_FORMAT_VERSION, __version__
from .contract.check import check_session_dir
from .data.build import BuildConfig, ViewPlan, build, load_shards
from .data.dataset import SampleConfig
from .data.schedule import RandomScheduleConfig, Schedule, Segment, random_uplink_schedule
from .sim.params import SimRanges, sample_params
from .sim.robot import simulate_session, write_session

HERE = os.path.dirname(os.path.abspath(__file__))
TRAINER_ROOT = os.path.dirname(HERE)
TRACES = os.path.join(TRAINER_ROOT, "contract", "traces")

# Official scoring schedules: named profiles only, so the C# side can rebuild them exactly.
TEST_SCHEDULES = (
    "lan@0",
    "50ms-5j@0",
    "150ms-20j-0.5loss@0",
    "300ms-60j-2loss-bursty@0",
    "synthetic-burst@0",
    "lan@0;150ms-20j-0.5loss@120;lan@300",
)
# The forecast-quality sweep runs on these (each level re-runs the model, so not on every schedule).
FORECAST_SWEEP_SCHEDULES = ("lan@0", "150ms-20j-0.5loss@0")
# A test session's physical uplink, when it has one: named training profiles, switching every 45 s.
TEST_UPLINK_CYCLE = ("lan", "150ms-20j-0.5loss", "lan", "50ms-5j")
TEST_UPLINK_DWELL_S = 45.0


def test_uplink_schedule(duration_s: float) -> Schedule:
    n = int(duration_s // TEST_UPLINK_DWELL_S) + 1
    return Schedule(segments=tuple(Segment(k * TEST_UPLINK_DWELL_S, TEST_UPLINK_CYCLE[k % len(TEST_UPLINK_CYCLE)], True)
                                   for k in range(n)))


def trainer_version() -> Dict[str, object]:
    """The VERSION file a packed trainer carries, or git when run from the repo."""
    path = os.path.join(TRAINER_ROOT, "VERSION")
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=TRAINER_ROOT, capture_output=True, text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "."], cwd=TRAINER_ROOT, capture_output=True,
                                    text=True, check=True).stdout.strip())
        return {"sha": sha, "dirty": dirty, "source": "git"}
    except (OSError, subprocess.CalledProcessError):
        return {"sha": "unknown", "dirty": True, "source": "none"}


@dataclass(frozen=True)
class MatrixConfig:
    """The simulated session matrix: every slider x load cell, like the real protocol."""

    sliders: Tuple[object, ...] = (1.0, 2.5, 3.8, 6.0, "varied")
    loads: Tuple[str, ...] = ("none", "payload-light", "payload-heavy", "hand")
    sessions_per_cell: int = 4
    uplink_impaired_share: float = 0.0  # one Core profile for the whole session
    uplink_profiles: Tuple[str, ...] = ("50ms-5j", "150ms-20j-0.5loss")
    # ADR 0021 decision 13: a physical uplink that changes mid-session. Training sessions get random
    # schedules (trainer-only segments allowed); test sessions the named TEST_UPLINK_CYCLE.
    uplink_schedule_share: float = 0.5
    test_uplink_schedule_share: float = 0.5
    minutes: float = 10.0
    # Held out of training entirely (ADR 0021): a slider value and a payload in between.
    held_out_slider: float = 5.0
    held_out_payload_g: float = 150.0
    light_payload_g: float = 100.0
    heavy_payload_g: float = 250.0
    val_share: float = 0.15
    test_sessions_per_cell: int = 1


def _load_kw(load: str, m: MatrixConfig) -> Dict[str, float]:
    if load == "none":
        return {"payload_grams": 0.0, "hand_events_per_min": 0.0}
    if load == "payload-light":
        return {"payload_grams": m.light_payload_g, "hand_events_per_min": 0.0}
    if load == "payload-heavy":
        return {"payload_grams": m.heavy_payload_g, "hand_events_per_min": 0.0}
    if load == "payload-held-out":
        return {"payload_grams": m.held_out_payload_g, "hand_events_per_min": 0.0}
    return {"payload_grams": 0.0, "hand_events_per_min": 4.0}


def plan_sim_sessions(m: MatrixConfig, seed: int) -> Dict[str, List[dict]]:
    """Which sessions to simulate, per split. Every split gets every cell; test adds the held-out ones."""
    rng = np.random.default_rng(seed)
    out: Dict[str, List[dict]] = {"train": [], "val": [], "test": []}
    next_seed = seed * 100_000
    for slider in m.sliders:
        for load in m.loads:
            for k in range(m.sessions_per_cell + m.test_sessions_per_cell):
                next_seed += 1
                split = "test" if k >= m.sessions_per_cell else ("val" if rng.random() < m.val_share else "train")
                r = rng.random()
                if split == "test":
                    uplink = "schedule:test" if r < m.test_uplink_schedule_share else "lan"
                elif r < m.uplink_schedule_share:
                    uplink = "schedule:random"
                elif r < m.uplink_schedule_share + m.uplink_impaired_share:
                    uplink = str(rng.choice(m.uplink_profiles))
                else:
                    uplink = "lan"
                out[split].append({"seed": next_seed, "slider": slider, "load": load, "uplink": uplink})
    for load in ("none", "payload-held-out", "hand"):
        next_seed += 1
        out["test"].append({"seed": next_seed, "slider": m.held_out_slider, "load": load, "uplink": "lan"})
    next_seed += 1
    out["test"].append({"seed": next_seed, "slider": 2.5, "load": "payload-held-out", "uplink": "lan"})
    return out


def _uplink_kw(spec: dict, duration_s: float) -> Dict[str, str]:
    uplink = spec["uplink"]
    if uplink == "schedule:random":
        rng = np.random.default_rng(np.random.PCG64([spec["seed"], 0x0091]))
        return {"uplink_profile": "lan", "uplink_schedule": random_uplink_schedule(duration_s, rng).to_json()}
    if uplink == "schedule:test":
        return {"uplink_profile": "lan", "uplink_schedule": test_uplink_schedule(duration_s).to_json()}
    return {"uplink_profile": uplink}


def _simulate_one(args) -> str:
    spec, m, ranges, directory = args
    params = sample_params(spec["seed"], ranges, duration_s=m.minutes * 60.0, slider=spec["slider"],
                           **_uplink_kw(spec, m.minutes * 60.0), **_load_kw(spec["load"], m))
    write_session(simulate_session(params), directory)
    report = check_session_dir(directory, allow_read_gaps=True)
    if not report.ok:
        raise RuntimeError("\n".join(report.lines()))
    return directory


def simulate_matrix(m: MatrixConfig, ranges: SimRanges, seed: int, out_dir: str, workers: int = 0, log=print) -> Dict[str, List[str]]:
    plan = plan_sim_sessions(m, seed)
    jobs, dirs = [], {k: [] for k in plan}
    for split, specs in plan.items():
        for spec in specs:
            d = os.path.join(out_dir, "sessions", split, f"sim-{spec['seed']:010d}")
            dirs[split].append(d)
            # Reuse a session only if it still passes the check (a failed write is never reused).
            if not (os.path.exists(os.path.join(d, "session.json")) and check_session_dir(d, allow_read_gaps=True).ok):
                jobs.append((spec, m, ranges, d))
    log(f"simulating {len(jobs)} sessions ({sum(len(v) for v in dirs.values())} planned: "
        f"{len(dirs['train'])} train / {len(dirs['val'])} val / {len(dirs['test'])} test), {m.minutes:g} min each")
    t0 = time.time()
    if jobs:
        with ProcessPoolExecutor(max_workers=workers or os.cpu_count() or 1) as pool:
            for k, _ in enumerate(pool.map(_simulate_one, jobs, chunksize=1)):
                if (k + 1) % max(1, len(jobs) // 10) == 0:
                    log(f"  {k + 1}/{len(jobs)} sessions ({time.time() - t0:.0f}s)")
    with open(os.path.join(out_dir, "sessions", "plan.json"), "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=2)
    return dirs


def build_splits(dirs: Dict[str, List[str]], out_dir: str, views_per_session: int, val_views: int, cfg: BuildConfig,
                 seed: int, log=print) -> Dict[str, str]:
    paths = {}
    for split in ("train", "val"):
        n_views = views_per_session if split == "train" else val_views
        plans = [ViewPlan(d, seed=seed * 1_000_003 + i * 131 + v) for i, d in enumerate(dirs[split]) for v in range(n_views)]
        path = os.path.join(out_dir, "data", f"{split}.npz")
        if not os.path.exists(path):
            t0 = time.time()
            index = build(plans, cfg, path)
            log(f"{split}: {index['samples']} samples from {len(plans)} views ({time.time() - t0:.0f}s)")
        paths[split] = path
    return paths


def load_config(path: str) -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)
