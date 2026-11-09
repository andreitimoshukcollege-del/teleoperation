"""The tier split (docs/adr/0035 decision 2): some traces of every source and tier are held out of training entirely, by
group, so a test trace never shares a session, a unit, a probe or a test case with a training one.

A group is held out when the hash of (salt, dataset, group) falls under ``fraction``: adding data never moves a group
already assigned. A (dataset, tier) cell with at least two groups and none under the threshold holds out its lowest-hash
group, so every tier of every source has a test side. ``split.json`` records the salt, the fraction, each trace's side and
the counts.
"""

from __future__ import annotations

import hashlib
import json
import os
from collections import defaultdict
from typing import Dict, List

from . import fetching
from . import index as I

SALT = "adr-0035-n2"
FRACTION = 0.2
FILE = "split.json"


def _h(dataset: str, group: str, salt: str) -> float:
    d = hashlib.sha256(f"{salt}\x00{dataset}\x00{group}".encode()).digest()
    return int.from_bytes(d[:8], "big") / 2.0 ** 64


def make(root: str, datasets: List[str], salt: str = SALT, fraction: float = FRACTION) -> Dict:
    cells: Dict = defaultdict(lambda: defaultdict(list))  # (dataset, tier) -> group -> [trace ids]
    for name in datasets:
        doc = I.load(root, name)
        if doc is None:
            continue
        for e in doc["traces"]:
            cells[(name, e["tier"])][e["group"]].append(e["id"])
    side: Dict[str, str] = {}
    counts: Dict = defaultdict(lambda: defaultdict(lambda: {"train": 0, "test": 0, "groupsTest": 0, "groups": 0}))
    for (name, tier), groups in sorted(cells.items()):
        ranked = sorted(groups, key=lambda g: _h(name, g, salt))
        test = {g for g in ranked if _h(name, g, salt) < fraction}
        if not test and len(ranked) >= 2:
            test = {ranked[0]}
        for g in ranked:
            s = "test" if g in test else "train"
            c = counts[name][tier]
            c["groups"] += 1
            c["groupsTest"] += s == "test"
            for tid in groups[g]:
                side[f"{name}/{tid}"] = s
                c[s] += 1
    doc = {"format": "teleop-network-split", "version": 1, "salt": salt, "fraction": fraction, "madeUtc": fetching.now_utc(),
           "counts": {k: dict(v) for k, v in counts.items()}, "traces": dict(sorted(side.items()))}
    p = os.path.join(root, FILE)
    os.makedirs(root, exist_ok=True)
    with open(p + ".tmp", "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
        f.write("\n")
    os.replace(p + ".tmp", p)
    return doc
