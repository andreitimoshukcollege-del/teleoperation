"""RIPE Atlas (docs/adr/0035 decision 2): pings from probes on every kind of connection (data centres, home fibre, cable,
DSL, Wi-Fi, LTE, 5G, satellite), to the k-root server, every 240 s.

**Source.** The public REST API (no key for reading public results): probes by tag from ``/api/v2/probes/`` (``tags``
comma-separated means all of them; user tags are self-reported, ``system-`` tags are RIPE's), then the built-in
measurement 1001 (IPv4 ping to k-root, 3 packets every 240 s) from ``/api/v2/measurements/1001/results/`` for a fixed
window. A result: ``timestamp``, ``prb_id``, ``sent``, ``rcvd``, ``result`` (``{"rtt": ms}``, or ``{"x": "*"}`` lost, or
``{"error": ...}``). Terms (RIPE Atlas Service Terms and Conditions v3.5, clause 4.5): research use is allowed and its
results may be published; commercial use needs permission; nothing says raw data may be republished, so none is.

**Conversion.** Per probe, each ``BLOCK_S`` of results (about 180 packets) becomes a *summary* trace of 60 s
(synth.py): one-way floor, mean and spread half the block's minimum, mean and standard deviation of round trip, loss the
block's lost packets over sent. A probe is one group.
"""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import os
import statistics
import time
import urllib.parse
from collections import defaultdict
from typing import Dict, Iterator, List, Optional, Tuple

from . import fetching, synth
from . import index as I
from .trace import Trace

NAME = "ripe-atlas"
API = "https://atlas.ripe.net/api/v2/"
MEASUREMENT = 1001  # built-in IPv4 ping to k.root-servers.net (193.0.14.129), 3 packets / 240 s
SOURCE = fetching.Source(
    name=NAME,
    title="RIPE Atlas built-in ping 1001 (k-root), probes chosen by connection tag",
    hosts=["atlas.ripe.net"],
    licence="RIPE Atlas Service Terms and Conditions v3.5 (effective 2025-11-04): research use allowed (4.5), commercial use needs permission; raw data not republished.",
    terms_url="https://www.ripe.net/about-us/legal/ripe-atlas-service-terms-and-conditions/",
    citation="RIPE NCC, RIPE Atlas, https://atlas.ripe.net (measurement 1001).",
    notes="Clause 4.5 invites notifying atlas@ripe.net about research results.",
)
TAG_SETS: Tuple[Tuple[str, ...], ...] = (
    ("datacentre",), ("home", "fibre"), ("home", "cable"), ("home", "dsl"), ("home", "wi-fi"),
    ("lte",), ("5g",), ("starlink",), ("satellite",),
)
START = "2026-11-01T00:00:00Z"
DAYS = 2
PROBES_PER_SET = 15
BATCH = 15
BLOCK_S = 4 * 3600
TRACE_S = 60.0
MIN_PACKETS = 30
MAX_RESPONSE = 100_000_000


def _h(s: str) -> str:
    return hashlib.sha256(s.encode()).hexdigest()


def _unix(iso: str) -> int:
    return int(dt.datetime.strptime(iso, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=dt.timezone.utc).timestamp())


def fetch(root: str, start: str = START, days: int = DAYS) -> fetching.Manifest:
    d = fetching.dataset_dir(root, NAME)
    dl = os.path.join(d, "download")
    old = fetching.load_manifest(root, NAME) or {}
    known = {f["path"]: f["sha256"] for f in old.get("files", [])}
    t0 = _unix(start)
    t1 = t0 + days * 86400
    files = []
    for tags in TAG_SETS:
        label = "-".join(tags)
        q = urllib.parse.urlencode({"tags": ",".join(tags), "status": 1, "page_size": 500})
        got = fetching.download(f"{API}probes/?{q}", dl, f"probes-{label}.json", SOURCE, MAX_RESPONSE, known)
        files.append(got)
        with open(os.path.join(d, got.path), encoding="utf-8") as f:
            probes = json.load(f).get("results", [])
        ids = sorted((p["id"] for p in probes if isinstance(p.get("id"), int)), key=lambda i: _h(f"probe/{i}"))[:PROBES_PER_SET]
        for k in range(0, len(ids), BATCH):
            q = urllib.parse.urlencode({"start": t0, "stop": t1, "probe_ids": ",".join(map(str, ids[k:k + BATCH])), "format": "json"})
            files.append(fetching.download(f"{API}measurements/{MEASUREMENT}/results/?{q}", dl, f"results-{label}-{k // BATCH}.json",
                                           SOURCE, MAX_RESPONSE, known, timeout_s=300.0))
            time.sleep(1.0)  # be gentle: RIPE publishes no number, and answers 429 when pressed
    m = fetching.Manifest(SOURCE, files, kept=f"probe lists per tag set, and measurement {MEASUREMENT}'s results for up to {PROBES_PER_SET} probes each, {start} + {days} days",
                          extra={"start": start, "days": days, "tagSets": [list(t) for t in TAG_SETS]})
    fetching.save_manifest(root, m)
    return m


def blocks(results: List[Dict], block_s: int = BLOCK_S) -> Iterator[Tuple[int, List[float], int, int]]:
    """Per block start: (start, round trips ms, sent, lost)."""
    by: Dict[int, Tuple[List[float], List[int]]] = defaultdict(lambda: ([], [0, 0]))
    for r in results:
        ts = r.get("timestamp")
        if not isinstance(ts, int) or r.get("type") != "ping":
            continue
        rtts, counts = by[ts - ts % block_s]
        for p in r.get("result", []) or []:
            if not isinstance(p, dict):
                continue
            if isinstance(p.get("rtt"), (int, float)) and p["rtt"] > 0:
                rtts.append(float(p["rtt"]))
                counts[0] += 1
            elif p.get("x") == "*":
                counts[0] += 1
                counts[1] += 1
    for start in sorted(by):
        rtts, (sent, lost) = by[start]
        yield start, rtts, sent, lost


def items(root: str) -> Iterator[Tuple[str, str, Trace, Dict]]:
    d = fetching.dataset_dir(root, NAME)
    dl = os.path.join(d, "download")
    probes: Dict[int, Dict] = {}
    tagset_of: Dict[int, str] = {}
    per_probe: Dict[int, List[Dict]] = defaultdict(list)
    seen = set()  # a probe in two tag sets was fetched twice
    for tags in TAG_SETS:
        label = "-".join(tags)
        p = os.path.join(dl, f"probes-{label}.json")
        if not os.path.exists(p):
            continue
        with open(p, encoding="utf-8") as f:
            for pr in json.load(f).get("results", []):
                if isinstance(pr.get("id"), int):
                    probes.setdefault(pr["id"], pr)
                    tagset_of.setdefault(pr["id"], label)
        k = 0
        while os.path.exists(os.path.join(dl, f"results-{label}-{k}.json")):
            with open(os.path.join(dl, f"results-{label}-{k}.json"), encoding="utf-8") as f:
                for r in json.load(f):
                    if isinstance(r, dict) and isinstance(r.get("prb_id"), int) and (r["prb_id"], r.get("timestamp")) not in seen:
                        seen.add((r["prb_id"], r.get("timestamp")))
                        per_probe[r["prb_id"]].append(r)
            k += 1
    for pid in sorted(per_probe):
        pr = probes.get(pid, {})
        tags = [t.get("slug") for t in pr.get("tags", []) if isinstance(t, dict)]
        for start, rtts, sent, lost in blocks(per_probe[pid]):
            if sent < MIN_PACKETS or len(rtts) < 2:
                continue
            floor, mean, sd = min(rtts) / 2.0, statistics.fmean(rtts) / 2.0, statistics.pstdev(rtts) / 2.0
            loss = lost / sent
            seed = int(_h(f"{pid}/{start}")[:16], 16)
            yield f"probe{pid}-{start}", f"probe{pid}", synth.summary_trace(seed, TRACE_S, floor, mean, sd, loss), {
                "probe": pid, "tagSet": tagset_of.get(pid), "tags": tags, "country": pr.get("country_code"), "asn": pr.get("asn_v4"),
                "blockStartUnix": start, "blockSeconds": BLOCK_S, "packets": sent, "loss": loss, "kind": "summary",
                "rtt_ms": {"min": 2 * floor, "avg": 2 * mean, "std": 2 * sd}, "seed": seed, "sendIntervalMs": synth.INTERVAL_MS,
                "assumption": "one-way = RTT / 2 of a 4-hour block of 3-packet pings; shifted gamma per datagram, i.i.d.; Bernoulli loss; FIFO"}


def convert(root: str) -> Dict:
    return I.emit(root, NAME, items(root), {"source": SOURCE.title, "measurement": MEASUREMENT})
