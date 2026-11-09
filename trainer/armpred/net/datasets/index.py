"""A converted dataset's traces and their index (docs/adr/0035 amendment 3).

``<root>/<dataset>/traces/<id>.trace`` (version 2) and ``<root>/<dataset>/index.json``: per trace its id, the group
it belongs to (what must stay on one side of the split: a G.1050 test case, a Puffer session, an FCC unit, a RIPE Atlas
probe, a Mahimahi trace, one of our recordings), its sha256, its assessment (tiering.py: the G.1050 profile and the
tier, the Y.1541 class, delay, loss, bursts, reordering) and what the converter assumed to make it.
"""

from __future__ import annotations

import json
import os
from typing import Dict, Iterable, List, Optional, Tuple

from . import fetching
from . import trace as T
from .tiering import assess

FORMAT = "teleop-network-dataset-index"
VERSION = 1


def safe_id(s: str) -> str:
    out = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in s).strip("._")
    if not out:
        raise ValueError(f"no id left of {s!r}")
    return out[:120]


def emit(root: str, name: str, items: Iterable[Tuple[str, str, T.Trace, Dict]], about: Dict,
         tier_override: Optional[Dict[str, str]] = None) -> Dict:
    """Write every (id, group, trace, info) and the index. ``tier_override``: id -> tier, where the source itself says
    the tier (G.1050's own test cases); the measured one is kept beside it."""
    d = fetching.dataset_dir(root, name)
    tdir = os.path.join(d, "traces")
    os.makedirs(tdir, exist_ok=True)
    entries: List[Dict] = []
    seen = set()
    for tid, group, tr, info in items:
        tid = safe_id(tid)
        if tid in seen:
            raise ValueError(f"{name}: two traces called {tid}")
        seen.add(tid)
        path = os.path.join(tdir, tid + ".trace")
        T.write(path, tr)
        a = assess(tr).to_json()
        entry = {"id": tid, "group": group, "file": os.path.relpath(path, d), "sha256": fetching.sha256_of(path),
                 "measuredTier": a["tier"], "tier": a["tier"], "assessment": a, "info": info}
        if tier_override and tid in tier_override:
            entry["tier"] = tier_override[tid]
            entry["tierFrom"] = "source"
        else:
            entry["tierFrom"] = "measured"
        entries.append(entry)
    for old in os.listdir(tdir):
        if old.endswith(".trace") and old[:-6] not in seen:
            os.remove(os.path.join(tdir, old))  # a trace this conversion no longer makes
    doc = {"format": FORMAT, "version": VERSION, "dataset": name, "convertedUtc": fetching.now_utc(), **about,
           "traces": entries}
    p = os.path.join(d, "index.json")
    with open(p + ".tmp", "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
        f.write("\n")
    os.replace(p + ".tmp", p)
    return doc


def load(root: str, name: str) -> Optional[Dict]:
    p = os.path.join(fetching.dataset_dir(root, name), "index.json")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        doc = json.load(f)
    if doc.get("format") != FORMAT or doc.get("version") != VERSION:
        raise ValueError(f"{p}: not a {FORMAT} v{VERSION}")
    return doc
