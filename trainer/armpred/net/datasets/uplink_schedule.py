"""Uplink schedules of the tiers' traces for Isaac sessions (docs/adr/0035 N4): what RobotHost's robot-side uplink
impairment replays, so the arm lives through good, poor and very-bad networks while the operator drives.

A schedule is RobotHost's control-file grammar (``schedule <seed> <profile>@<seconds>;...``, ``UplinkSchedule``) with
``trace:<dataset>/<id>`` segments (``UplinkProfiles``: a ``.trace`` v2 replayed by time). Segments cycle through the
asked tiers in order; for each, a dataset that has traces of that tier on the **train** side of ``split.json`` is drawn
uniformly (so RIPE Atlas's many good traces do not crowd out the rest), then one of its traces. Held-out traces are never
used: a session recorded over a test trace would leak it into training.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence

import numpy as np

from . import index as I
from . import split as SP

TIERS = ("good", "poor", "very-bad")


def train_traces(root: str, datasets: Sequence[str]) -> Dict[str, Dict[str, List[str]]]:
    """tier -> dataset -> trace ids on the train side of the split."""
    import json

    with open(os.path.join(root, SP.FILE), encoding="utf-8") as f:
        side = json.load(f)["traces"]
    out: Dict[str, Dict[str, List[str]]] = {t: {} for t in TIERS}
    for name in datasets:
        doc = I.load(root, name)
        if doc is None:
            continue
        for e in doc["traces"]:
            if side.get(f"{name}/{e['id']}") == "train" and e["tier"] in out:
                out[e["tier"]].setdefault(name, []).append(e["id"])
    for tier in out:
        for name in out[tier]:
            out[tier][name].sort()
    return out


def make(root: str, datasets: Sequence[str], seed: int, segment_s: float = 60.0, segments: int = 15,
         tiers: Sequence[str] = TIERS) -> str:
    if segments < 1 or not segment_s > 0:
        raise ValueError("at least one segment, of positive length")
    bad = [t for t in tiers if t not in TIERS]
    if bad or not tiers:
        raise ValueError(f"tiers are {TIERS}, not {list(tiers)}")
    pool = train_traces(root, datasets)
    rng = np.random.Generator(np.random.PCG64(seed))
    parts = []
    for k in range(segments):
        tier = tiers[k % len(tiers)]
        names = sorted(pool[tier])
        if not names:
            raise ValueError(f"no train-side trace of tier {tier} in {list(datasets)}: run the split first")
        name = names[int(rng.integers(len(names)))]
        ids = pool[tier][name]
        parts.append(f"trace:{name}/{ids[int(rng.integers(len(ids)))]}@{k * segment_s:g}")
    return f"schedule {seed} " + ";".join(parts)
