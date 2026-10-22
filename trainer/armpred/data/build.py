"""Sessions -> views -> sample shards, in parallel. What ``sim-pretrain`` and ``finetune`` train on.

Every sample is tagged with its session's conditions so training can weight each speed x load cell
equally, and so evaluation can stratify. The tags come from ``session.json``, never from the model's
inputs.
"""

from __future__ import annotations

import json
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..baselines.types import headset_joint_config
from ..contract.profile import JETROVER, RobotProfile, load_profile
from ..contract.session import load_session
from ..features import layout as L
from ..net import forecast as FC
from .dataset import SampleConfig, Samples, view_samples
from .schedule import RandomScheduleConfig, Schedule, parse, random_schedule
from .views import synthesize


@dataclass(frozen=True)
class ViewPlan:
    session_dir: str
    seed: int
    schedule: Optional[str] = None  # a named schedule; None draws a random training schedule
    profile_path: Optional[str] = None


@dataclass(frozen=True)
class ForecastConfig:
    """Which network forecast level each view's samples are built at (ADR 0021 decision 12): one
    ``level`` for every view, or a draw per view from ``mixture``."""

    level: Optional[str] = None
    mixture: Tuple[Tuple[str, float], ...] = tuple(sorted(FC.TRAINING_MIXTURE.items()))

    def pick(self, seed: int) -> str:
        if self.level is not None:
            return self.level
        # Its own stream, so the schedule and the queries do not depend on the level.
        return FC.draw_level(np.random.default_rng(np.random.PCG64([int(seed) & 0xFFFFFFFF, 0x1E7E1])), dict(self.mixture))


@dataclass(frozen=True)
class BuildConfig:
    sample: SampleConfig = SampleConfig()
    schedules: RandomScheduleConfig = RandomScheduleConfig()
    aware: bool = False
    traces_dir: Optional[str] = None
    workers: int = 0  # 0: os.cpu_count()
    # The prior's per-motor reading ages (ms, motors 1-5): None is the headset's 45/35/25/15/5;
    # `armpred sysid` fits them from recordings (its prior.json).
    sensed_ages_ms: Optional[Tuple[float, ...]] = None
    forecast: ForecastConfig = ForecastConfig()


def condition_tags(meta: dict) -> Dict[str, float]:
    c = meta.get("conditions", {})
    slider = c.get("speedSliderRadPerSec", -1)
    load = c.get("load", {})
    return {
        "slider": float(slider) if isinstance(slider, (int, float)) else -1.0,  # -1: varied
        "payload_g": float(load.get("payloadGrams", 0.0) or 0.0),
        "hand": 1.0 if load.get("kind") == "hand" else 0.0,
        "uplink_impaired": 0.0 if c.get("network", {}).get("uplinkProfile", "lan") in ("lan", "none") else 1.0,
        "added_delay_ms": float(c.get("network", {}).get("addedDelayMs", 0) or 0),
    }


def _one(plan: ViewPlan, cfg: BuildConfig) -> Tuple[Samples, Dict[str, object]]:
    session = load_session(plan.session_dir)
    profile = load_profile(plan.profile_path) if plan.profile_path else JETROVER
    rng = np.random.default_rng(np.random.PCG64(plan.seed))
    duration = (int(session.reads.t_resp.max()) - int(session.reads.t_req.min())) / session.ticks_per_second
    schedule = parse(plan.schedule) if plan.schedule else random_schedule(duration, rng, cfg.schedules)
    view = synthesize(session, profile, schedule, seed=plan.seed, traces_dir=cfg.traces_dir)
    prior = headset_joint_config(view.tps, sensed_ages_ms=cfg.sensed_ages_ms) if cfg.sensed_ages_ms else None
    level = cfg.forecast.pick(plan.seed)
    samples = view_samples(view, rng, cfg.sample, prior_config=prior, aware=cfg.aware, level=level,
                           forecast_seed=plan.seed ^ 0x5F0CA57)
    info = {"session": session.id, "schedule": schedule.describe(), "namedOnly": schedule.only_named,
            "seed": plan.seed, "samples": len(samples), "forecastLevel": level, **condition_tags(session.meta)}
    return samples, info


def _one_star(args):
    return _one(*args)


def build(plans: Sequence[ViewPlan], cfg: BuildConfig, out_path: str) -> Dict[str, object]:
    """Builds every view's samples and writes one shard (``.npz``) plus its index (``.json``)."""
    workers = cfg.workers or os.cpu_count() or 1
    results: List[Tuple[Samples, Dict[str, object]]] = []
    if workers == 1 or len(plans) == 1:
        results = [_one(p, cfg) for p in plans]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            results = list(pool.map(_one_star, [(p, cfg) for p in plans], chunksize=1))
    parts, infos, view_index = [], [], []
    pair_offset = 0
    for k, (s, info) in enumerate(results):
        # Pair ids are per view; shift each view's past the previous ones (counted before the shift).
        n_pairs = int(s.pair.max()) + 1 if (s.pair >= 0).any() else 0
        s.pair = np.where(s.pair >= 0, s.pair.astype(np.int64) + pair_offset, -1).astype(np.int32)
        pair_offset += n_pairs
        parts.append(s)
        infos.append(info)
        view_index.append(np.full(len(s), k, dtype=np.int32))
    samples = Samples.concat(parts) if parts else None
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    tags = {key: np.concatenate([np.full(len(p), info[key], dtype=np.float32) for p, info in zip(parts, infos)])
            for key in ("slider", "payload_g", "hand", "uplink_impaired", "added_delay_ms")} if parts else {}
    arrays = {name: getattr(samples, name) for name in Samples.__dataclass_fields__} if samples else {}
    np.savez(out_path, view=np.concatenate(view_index) if view_index else np.zeros(0, np.int32), **arrays,
             **{f"tag_{k}": v for k, v in tags.items()})
    index = {"views": infos, "samples": int(sum(len(p) for p in parts)), "aware": cfg.aware,
             "layout": L.layout_id(cfg.aware), "sample": asdict(cfg.sample), "forecast": asdict(cfg.forecast)}
    with open(os.path.splitext(out_path)[0] + ".json", "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2)
    return index


@dataclass
class Dataset:
    samples: Samples
    view: np.ndarray
    tags: Dict[str, np.ndarray]
    index: List[dict] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.samples)


def load_shards(paths: Sequence[str], layout: Optional[str] = None) -> Dataset:
    """Shards into one dataset. With ``layout``, a shard built for another layout is refused: its
    rows would be read as columns they are not (a v1 shard has no layout id at all)."""
    parts, views, tags, index = [], [], {}, []
    view_offset = 0
    for path in paths:
        with open(os.path.splitext(path)[0] + ".json", "r", encoding="utf-8") as f:
            built = json.load(f).get("layout")
        if layout is not None and built != layout:
            raise ValueError(f"{path} was built for layout {built or 'v1'}, not {layout}: prepare it again")
        with np.load(path) as z:
            parts.append(Samples(**{name: z[name] for name in Samples.__dataclass_fields__}))
            views.append(z["view"] + view_offset)
            for key in z.files:
                if key.startswith("tag_"):
                    tags.setdefault(key[4:], []).append(z[key])
        with open(os.path.splitext(path)[0] + ".json", "r", encoding="utf-8") as f:
            info = json.load(f)
        index += info["views"]
        view_offset += len(info["views"])
    return Dataset(samples=Samples.concat(parts), view=np.concatenate(views),
                   tags={k: np.concatenate(v) for k, v in tags.items()}, index=index)
