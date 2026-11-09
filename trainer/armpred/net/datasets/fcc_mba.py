"""The FCC's Measuring Broadband America (docs/adr/0035 decision 2, poor): US home broadband (DSL, cable, fibre) measured
by whiteboxes, idle and under load.

**Source.** ``data.fcc.gov/download/measuring-broadband-america/2023/data-raw-2023-jul.tar.gz`` (about 1.7 GB, the last
monthly raw release: the SamKnows panel ended 2023-08-01), and ``unit-profile-sept2022.xlsx`` (each unit's ISP and
technology). Used from the tarball, by their headers (the 2023 technical appendix and the data dictionary disagree on a
column or two, so nothing is read by position):

- ``curr_udplatency.csv``: per unit and hour, about 2000 UDP packets to a test server: ``rtt_min``, ``rtt_avg``,
  ``rtt_std`` (microseconds, over the fastest 99%), ``successes``, ``failures`` (lost: no reply in 3 s);
- ``curr_dlping.csv``, ``curr_ulping.csv``: latency under load: a packet every 100 ms for the 10 s of a download or
  upload speed test, same columns.

fcc.gov's terms page could not be fetched (HTTP 403, 2026-11-09). Federal works are generally public domain; the data
were collected by a contractor, so the terms are recorded as unverified. Nothing is redistributed.

**Conversion.** A row becomes a *summary* trace (synth.py): one-way floor, mean and spread half the round trip's, loss
``failures / (successes + failures)``; 60 s for an idle hour, 10 s (the test's own length) under load. Per unit, up to
``ROWS_PER_UNIT`` rows of each file, chosen by hash; up to ``MAX_UNITS`` units. A unit is one group.
"""

from __future__ import annotations

import csv
import hashlib
import heapq
import itertools
import io
import os
import re
import tarfile
import zipfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Dict, Iterator, List, Optional, Tuple

from . import fetching, synth
from . import index as I
from .trace import Trace

NAME = "fcc-mba"
BASE = "https://data.fcc.gov/download/measuring-broadband-america/2023/"
ARCHIVE = "data-raw-2023-jul.tar.gz"
PROFILE = "unit-profile-sept2022.xlsx"
SOURCE = fetching.Source(
    name=NAME,
    title="FCC Measuring Broadband America, raw data July 2023",
    hosts=["data.fcc.gov"],
    licence="Unverified: fcc.gov's terms page returned HTTP 403 (2026-11-09); US federal works are generally public domain (17 U.S.C. 105), collected by contractor SamKnows. Not redistributed.",
    terms_url="https://www.fcc.gov/general/measuring-broadband-america",
    citation="FCC Office of Engineering and Technology, Measuring Broadband America, raw data releases; Technical Appendix to the Thirteenth MBA Report (2023).",
)
MAX_ARCHIVE = 4_000_000_000
MAX_PROFILE = 10_000_000
FILES = {"curr_udplatency.csv": ("idle", 60.0), "curr_dlping.csv": ("download-load", 10.0), "curr_ulping.csv": ("upload-load", 10.0)}
NEEDED = ("unit_id", "rtt_min", "rtt_avg", "rtt_std", "successes", "failures")
MAX_UNITS = 300
ROWS_PER_UNIT = 4


def fetch(root: str) -> fetching.Manifest:
    d = fetching.dataset_dir(root, NAME)
    old = fetching.load_manifest(root, NAME) or {}
    known = {f["path"]: f["sha256"] for f in old.get("files", [])}
    dl = os.path.join(d, "download")
    files = [fetching.download(BASE + PROFILE, dl, PROFILE, SOURCE, MAX_PROFILE, known),
             fetching.download(BASE + ARCHIVE, dl, ARCHIVE, SOURCE, MAX_ARCHIVE, known, timeout_s=120.0)]
    m = fetching.Manifest(SOURCE, files, kept="the July 2023 archive and the unit profile; the conversion reads curr_udplatency, curr_dlping and curr_ulping")
    fetching.save_manifest(root, m)
    return m


def _h(s: str) -> str:
    return hashlib.sha256(s.encode()).hexdigest()


def xlsx_rows(data: bytes) -> List[List[str]]:
    """The first sheet of an .xlsx, as rows of strings (standard library only)."""
    ns = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        shared: List[str] = []
        if "xl/sharedStrings.xml" in z.namelist():
            for si in ET.fromstring(z.read("xl/sharedStrings.xml")).findall("m:si", ns):
                shared.append("".join(t.text or "" for t in si.iter(f"{{{ns['m']}}}t")))
        sheets = sorted(n for n in z.namelist() if re.match(r"^xl/worksheets/sheet\d+\.xml$", n))
        root = ET.fromstring(z.read(sheets[0]))
    rows = []
    for r in root.iter(f"{{{ns['m']}}}row"):
        cells: Dict[int, str] = {}
        for c in r.findall("m:c", ns):
            ref = c.get("r", "")
            col = 0
            for ch in re.match(r"[A-Z]*", ref).group(0):
                col = col * 26 + ord(ch) - 64
            v = c.find("m:v", ns)
            text = "" if v is None else (v.text or "")
            if c.get("t") == "s" and text:
                text = shared[int(text)]
            elif c.get("t") == "inlineStr":
                text = "".join(t.text or "" for t in c.iter(f"{{{ns['m']}}}t"))
            cells[col - 1] = text
        if cells:
            rows.append([cells.get(i, "") for i in range(max(cells) + 1)])
    return rows


def unit_profiles(data: bytes) -> Dict[str, Dict[str, str]]:
    rows = xlsx_rows(data)
    head = [h.strip() for h in rows[0]]
    out = {}
    for r in rows[1:]:
        rec = dict(zip(head, r))
        uid = rec.get("Unit ID", "").split(".")[0]
        if uid:
            out[uid] = {"isp": rec.get("ISP", ""), "technology": rec.get("Technology", ""), "state": rec.get("State", "")}
    return out


def rows_of(text: io.TextIOBase) -> Iterator[Dict[str, str]]:
    reader = csv.DictReader(text)
    missing = [c for c in NEEDED if c not in (reader.fieldnames or [])]
    if missing:
        raise ValueError(f"header lacks {missing}: {reader.fieldnames}")
    yield from reader


def summary_of(row: Dict[str, str]) -> Optional[Tuple[float, float, float, float]]:
    """(floor, mean, sd) one-way in ms and the loss ratio, or None for a row that does not make sense."""
    try:
        mn, avg, sd = (float(row[k]) / 1000.0 for k in ("rtt_min", "rtt_avg", "rtt_std"))
        ok, lost = int(float(row["successes"])), int(float(row["failures"]))
    except (ValueError, KeyError):
        return None
    if ok + lost <= 0 or ok <= 0 or not (0 < mn <= avg) or sd < 0 or avg > 10_000:
        return None
    return mn / 2.0, avg / 2.0, sd / 2.0, lost / (ok + lost)


def items(root: str) -> Iterator[Tuple[str, str, Trace, Dict]]:
    dl = os.path.join(fetching.dataset_dir(root, NAME), "download")
    with open(os.path.join(dl, PROFILE), "rb") as f:
        profiles = unit_profiles(f.read())
    keep = ROWS_PER_UNIT * 3  # spare rows for ones that do not make sense
    tiebreak = itertools.count()  # duplicate rows must never compare their dicts
    picked: Dict[str, Dict[str, list]] = defaultdict(lambda: defaultdict(list))  # unit -> file -> heap of (-hash, row)
    with tarfile.open(os.path.join(dl, ARCHIVE), "r:gz") as tar:
        for member in tar:
            base = os.path.basename(member.name)
            if not member.isfile() or base not in FILES:
                continue
            f = tar.extractfile(member)
            if f is None:
                continue
            for row in rows_of(io.TextIOWrapper(f, encoding="utf-8", errors="replace", newline="")):
                uid = row["unit_id"].strip()
                if not uid:
                    continue
                key = _h(f"{uid}/{base}/{row.get('dtime', '')}/{row.get('target', '')}")
                heap = picked[uid][base]
                item = (_neg(key), key, next(tiebreak), {k: row.get(k, "") for k in NEEDED + ("dtime", "target")})
                if len(heap) < keep:
                    heapq.heappush(heap, item)
                elif item[0] > heap[0][0]:
                    heapq.heapreplace(heap, item)
    units = sorted(picked, key=lambda u: _h(f"unit/{u}"))[:MAX_UNITS]
    for uid in units:
        for base, heap in sorted(picked[uid].items()):
            kind, seconds = FILES[base]
            used = 0
            for _, key, _, row in sorted(heap, key=lambda x: (x[1], x[2])):
                if used >= ROWS_PER_UNIT:
                    break
                s = summary_of(row)
                if s is None:
                    continue
                floor, mean, sd, loss = s
                seed = int(key[:16], 16)
                used += 1
                yield f"unit{uid}-{kind}-{used}", f"unit{uid}", synth.summary_trace(seed, seconds, floor, mean, sd, loss), {
                    "unit": uid, **profiles.get(uid, {}), "test": base, "kind": "summary", "load": kind, "dtime": row.get("dtime", ""),
                    "target": row.get("target", ""), "rtt_ms": {"min": 2 * floor, "avg": 2 * mean, "std": 2 * sd}, "loss": loss,
                    "seed": seed, "sendIntervalMs": synth.INTERVAL_MS,
                    "assumption": "one-way = RTT / 2 (min, avg, std of the fastest 99%); shifted gamma per datagram, i.i.d.; Bernoulli loss; FIFO"}


def _neg(hexkey: str) -> int:
    """Heap order: the heap's top is the largest hash kept, so a smaller one replaces it."""
    return -int(hexkey[:16], 16)


def convert(root: str) -> Dict:
    return I.emit(root, NAME, items(root), {"source": SOURCE.title, "url": BASE + ARCHIVE})
