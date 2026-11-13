"""Puffer (Stanford; docs/adr/0035 decision 2, poor): real home connections to a live TV service, one smoothed round
trip (Linux ``tcpi_rtt``) per video chunk sent, about every 2 s per stream.

**Source.** ``storage.googleapis.com/puffer-data-release/<day>/video_sent_<day>.csv`` (a day runs 11:00Z to 11:00Z). The
columns used, by name from the header: ``time (ns GMT)``, ``session_id``, ``index`` (a stream is the pair),
``min_rtt`` and ``rtt`` (microseconds). **No licence for the data is stated** on the data-description, terms or FAQ
pages (checked 2026-11-09); the page asks that the data be downloaded from their server only once, so the file is
fetched once and kept, and nothing of it leaves the GPU machine.

**Conversion.** A stream lasting at least ``MIN_SECONDS`` becomes a *level* trace (synth.py): one-way delay half the
smoothed round trip, interpolated between chunks, datagrams every 20 ms, at most ``MAX_SECONDS``. ``tcpi_rtt`` is smoothed
and a video chunk's own traffic is in it, so the trace carries the connection's slow changes, not packet-scale jitter;
the index says so. Up to ``MAX_STREAMS`` streams, chosen by hash; a session's streams are one group.
"""

from __future__ import annotations

import csv
import hashlib
import io
import os
from collections import defaultdict
from typing import Dict, Iterator, List, Tuple

from . import fetching, synth
from . import index as I
from .trace import Trace

NAME = "puffer"
DAY = "2026-11-07T11_2026-11-08T11"
URL = "https://storage.googleapis.com/puffer-data-release/{day}/video_sent_{day}.csv"
SOURCE = fetching.Source(
    name=NAME,
    title="Puffer video_sent (Stanford), one day",
    hosts=["storage.googleapis.com"],
    licence="None stated (data-description, terms and FAQ pages checked 2026-11-09). Research use only; not redistributed.",
    terms_url="https://puffer.stanford.edu/data-description/",
    citation="F. Y. Yan et al., Learning in situ: a randomized experiment in video streaming, NSDI 2020; puffer.stanford.edu.",
    notes="The data-description page: 'We would also be grateful if you could download the data from our server only once.'",
)
MAX_FILE = 2_000_000_000
MIN_SECONDS = 120.0
MAX_SECONDS = 600.0
MAX_STREAMS = 400
COLUMNS = ("time (ns GMT)", "session_id", "index", "min_rtt", "rtt")


def fetch(root: str, day: str = DAY) -> fetching.Manifest:
    if not (len(day) == 27 and day[10] == "T" and day.count("_") == 1):
        raise ValueError(f"not a Puffer day: {day!r}")
    d = fetching.dataset_dir(root, NAME)
    old = fetching.load_manifest(root, NAME) or {}
    known = {f["path"]: f["sha256"] for f in old.get("files", [])}
    got = fetching.download(URL.format(day=day), os.path.join(d, "download"), f"video_sent_{day}.csv", SOURCE, MAX_FILE, known,
                            timeout_s=120.0)
    m = fetching.Manifest(SOURCE, [got], kept=f"video_sent for {day}; the conversion keeps time, stream, rtt and min_rtt", extra={"day": day})
    fetching.save_manifest(root, m)
    return m


def _h(s: str) -> str:
    return hashlib.sha256(s.encode()).hexdigest()


def streams(lines: io.TextIOBase) -> Dict[Tuple[str, str], List[Tuple[float, float, float]]]:
    """(session hash, index) -> [(time s, rtt ms, min_rtt ms)] from a video_sent CSV. Rows that do not parse are skipped."""
    reader = csv.reader(lines)
    head = next(reader)
    try:
        col = [head.index(c) for c in COLUMNS]
    except ValueError as e:
        raise ValueError(f"video_sent header lacks a column: {e}; header {head}")
    out: Dict[Tuple[str, str], List[Tuple[float, float, float]]] = defaultdict(list)
    for row in reader:
        try:
            t, sess, idx, mn, rtt = (row[i] for i in col)
            out[(_h(sess)[:16], idx)].append((int(t) / 1e9, int(rtt) / 1000.0, int(mn) / 1000.0))
        except (ValueError, IndexError):
            continue
    return out


def items(root: str) -> Iterator[Tuple[str, str, Trace, Dict]]:
    m = fetching.load_manifest(root, NAME)
    day = (m or {}).get("day", DAY)
    p = os.path.join(fetching.dataset_dir(root, NAME), "download", f"video_sent_{day}.csv")
    with open(p, encoding="utf-8", newline="") as f:
        all_streams = streams(f)
    long_enough = [(k, v) for k, v in all_streams.items() if len(v) >= 2 and max(r[0] for r in v) - min(r[0] for r in v) >= MIN_SECONDS]
    long_enough.sort(key=lambda kv: _h(f"{kv[0][0]}/{kv[0][1]}"))
    for (sess, idx), rows in long_enough[:MAX_STREAMS]:
        rows.sort()
        tr = synth.level_trace([r[0] for r in rows], [r[1] / 2.0 for r in rows], max_seconds=MAX_SECONDS)
        yield f"{sess}-{idx}", sess, tr, {
            "day": day, "session": sess, "stream": idx, "chunks": len(rows), "sendIntervalMs": synth.INTERVAL_MS, "kind": "level",
            "minRttMs": min(r[2] for r in rows),
            "assumption": "one-way delay = tcpi_rtt / 2 (smoothed, includes the video's own queueing), interpolated between ~2 s chunks; no packet-scale jitter; no loss"}


def convert(root: str) -> Dict:
    return I.emit(root, NAME, items(root), {"source": SOURCE.title})
