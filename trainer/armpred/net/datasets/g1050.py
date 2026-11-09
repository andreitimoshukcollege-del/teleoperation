"""ITU-T G.1050 (07/2016)'s standard test cases (docs/adr/0035 amendment 2): per-packet delay and loss from the
Recommendation's own simulations, for each of its three service test profiles, which are the tiers.

**Source.** The Recommendation's electronic attachment (Annex H), freely available from the ITU:
``T-REC-G.1050-201607-I!!SOFT-ZST-E``, a zip of about 780 MB holding an ISO 9660 image. Its ``./D`` (DSL) and ``./G``
(GPON) folders hold the simulator's output per scenario and test case (Annex F, Table F.1):

- ``$TECH/$SC/$TC/$FLOW.out``: the managed flows' "delay-loss schedule" (clause 6.6, Table 3): a header, then one row per
  packet, ``time delay(ms) drop``. The VoIP flows (``VoIP1_Fwd``, ``VoIP1_Rev``: G.729, one packet every 20 ms) are the
  ones kept: the same rate and size class as the joint stream. Profiles A and B (``w``, ``p``) carry managed flows;
  profile C (``u``) has none ("There are no managed streams in the Unmanaged service test profile C").
- ``$TECH/$SC/$TC/bw-{up,dwn}.csv``: the iPerf flow's delay per time bin (clause 6.6, Table 4: ``t``, bandwidth, packets,
  ``mean`` delay, ``m2`` the mean squared delay, "used to introduce jitter for network emulation"). Kept for profile C,
  where a packet crossing the saturated, unmanaged link sees that queue.

**Conversion.** A VoIP ``.out`` becomes a trace as it is: each row a datagram, ``drop`` 1 a loss, the send interval the
rows' median spacing. A ``bw`` file is a *statistical* trace: datagrams every 20 ms, each bin's delays drawn from a normal
of the bin's mean and standard deviation (``sqrt(m2 - mean^2)``, the use Table 4 gives ``m2``), clipped at zero,
delivered first in first out (synth.py), seeded by the file's path; the index says so. The tier is the test case's own profile (``tierFrom: source``); the measured one is kept beside it.
"""

from __future__ import annotations

import gzip
import hashlib
import io
import os
import re
import zipfile
from typing import Dict, Iterator, List, Optional, Tuple

import numpy as np

from . import fetching, iso9660, synth
from . import index as I
from .standards import TIER_OF_PROFILE
from .trace import LOST, Trace, from_ms

NAME = "g1050-2016"
URL = "https://www.itu.int/rec/dologin_pub.asp?lang=e&id=T-REC-G.1050-201607-I!!SOFT-ZST-E&type=items"
SOURCE = fetching.Source(
    name=NAME,
    title="ITU-T G.1050 (07/2016) electronic attachment: simulator output of the standard test cases",
    hosts=["www.itu.int"],
    licence="ITU publication, freely available ('Freely available items'); copyright ITU. Used for research; not redistributed.",
    terms_url="https://www.itu.int/en/ITU-T/publications/Pages/default.aspx",
    citation="ITU-T Recommendation G.1050 (07/2016), Network model for evaluating multimedia transmission performance over Internet Protocol, Annex H.",
)
ZIP_NAME = "g1050-2016-attachment.zip"
MAX_ZIP = 1_200_000_000
MAX_ISO = 4_000_000_000
MAX_MEMBER = 400_000_000
CASE = re.compile(r"^(?P<tech>[DG])(?P<profile>[wpu])(?P<severity>\d+)t?$")
KEEP_FLOWS = ("VoIP1_Fwd", "VoIP1_Rev")
STAT_INTERVAL_MS = 20.0


def fetch(root: str) -> fetching.Manifest:
    d = fetching.dataset_dir(root, NAME)
    old = fetching.load_manifest(root, NAME) or {}
    known = {f["path"]: f["sha256"] for f in old.get("files", [])}
    got = fetching.download(URL, os.path.join(d, "download"), ZIP_NAME, SOURCE, MAX_ZIP, known, timeout_s=120.0)
    m = fetching.Manifest(SOURCE, [got], kept="the whole attachment zip; the conversion reads the VoIP .out and bw-*.csv files out of its ISO image")
    fetching.save_manifest(root, m)
    return m


def _iso_path(root: str) -> str:
    """The ISO image, taken out of the zip once (zip members are read, never extracted by their own names)."""
    d = fetching.dataset_dir(root, NAME)
    iso = os.path.join(d, "download", "g1050-2016-attachment.iso")
    if os.path.exists(iso):
        return iso
    with zipfile.ZipFile(os.path.join(d, "download", ZIP_NAME)) as z:
        isos = [i for i in z.infolist() if i.filename.lower().endswith(".iso")]
        if len(isos) != 1:
            raise ValueError(f"expected one .iso in the attachment, found {[i.filename for i in z.infolist()][:20]}")
        if isos[0].file_size > MAX_ISO:
            raise ValueError(f"{isos[0].filename}: {isos[0].file_size} bytes")
        with z.open(isos[0]) as src, open(iso + ".part", "wb") as dst:
            n = 0
            for block in iter(lambda: src.read(1 << 20), b""):
                n += len(block)
                if n > MAX_ISO:
                    raise ValueError("the ISO image is larger than its zip entry said")
                dst.write(block)
    os.replace(iso + ".part", iso)
    return iso


def listing(root: str) -> List[str]:
    """Every path in the attachment (for ``status``: what is in it)."""
    with open(_iso_path(root), "rb") as f:
        return [e.path + ("/" if e.is_dir else "") for e in iso9660.walk(f)]


def _text(data: bytes, path: str) -> str:
    if path.lower().endswith(".gz"):
        data = gzip.decompress(data)
    return data.decode("latin-1")


def parse_out(text: str) -> Tuple[List[float], List[Optional[float]]]:
    """A ``.out`` delay-loss schedule: (times s, delays ms, None where dropped). Header lines are skipped."""
    times, delays = [], []
    for line in text.splitlines():
        parts = re.split(r"[,\s]+", line.strip())
        if len(parts) < 3:
            continue
        try:
            t, d, drop = float(parts[0]), float(parts[1]), int(float(parts[2]))
        except ValueError:
            continue  # a header line
        times.append(t)
        delays.append(None if drop else d)
    return times, delays


def parse_bins(text: str) -> List[Tuple[float, float, float]]:
    """A ``bw`` file (clause 6.6 Table 4): (bin start s, mean delay ms, standard deviation ms) per bin with packets."""
    out = []
    for line in text.splitlines():
        parts = re.split(r"[,\s]+", line.strip())
        if len(parts) < 5:
            continue
        try:
            t, _bw, packets, mean, m2 = (float(x) for x in parts[:5])
        except ValueError:
            continue
        if packets <= 0:
            continue
        out.append((t, mean, float(np.sqrt(max(0.0, m2 - mean * mean)))))
    return out


def statistical_trace(bins: List[Tuple[float, float, float]], seed: int) -> Trace:
    rng = synth.generator(seed)
    if len(bins) < 2:
        raise ValueError("too few bins")
    width = float(np.median(np.diff([b[0] for b in bins])))
    n = max(1, int(round(width * 1000.0 / STAT_INTERVAL_MS)))
    delays = np.concatenate([np.maximum(0.0, rng.normal(mean, sd, n)) for _, mean, sd in bins])
    return synth.to_trace(delays, np.zeros(len(delays), dtype=bool), STAT_INTERVAL_MS)


def _seed(s: str) -> int:
    return int.from_bytes(hashlib.sha256(s.encode()).digest()[:8], "big")


def items(root: str) -> Iterator[Tuple[str, str, Trace, Dict]]:
    with open(_iso_path(root), "rb") as f:
        entries = iso9660.walk(f)
        for e in entries:
            if e.is_dir:
                continue
            parts = e.path.split("/")
            if len(parts) < 4:
                continue
            tech, scen, case, fname = parts[-4], parts[-3], parts[-2], parts[-1]
            m = CASE.match(case)
            if tech not in ("D", "G") or not m:
                continue
            base = fname[:-3] if fname.lower().endswith(".gz") else fname
            group = f"{m['tech']}{m['profile']}{m['severity']}"  # a test case, whatever the scenario
            info = {"testCase": case, "scenario": scen, "access": {"D": "DSL", "G": "GPON"}[tech],
                    "profile": m["profile"], "severity": int(m["severity"]), "source": e.path}
            if m["profile"] in "wp" and base.endswith(".out") and base[:-4] in KEEP_FLOWS:
                times, delays = parse_out(_text(iso9660.read(f, e, MAX_MEMBER), e.path))
                if len(times) < 100:
                    continue
                interval = float(np.median(np.diff(times))) * 1000.0
                info.update(flow=base[:-4], kind="per-packet", sendIntervalMs=round(interval, 3))
                yield f"{case}-{scen}-{base[:-4]}", group, from_ms(delays, interval), info
            elif m["profile"] == "u" and re.match(r"^bw-(up|dwn)\.csv$", base):
                bins = parse_bins(_text(iso9660.read(f, e, MAX_MEMBER), e.path))
                if len(bins) < 10:
                    continue
                seed = _seed(e.path)
                info.update(flow=base[:-4], kind="statistical", seed=seed, sendIntervalMs=STAT_INTERVAL_MS,
                            assumption="iPerf per-bin mean and sqrt(m2 - mean^2) as a normal per 20 ms datagram, clipped at 0, FIFO; no loss (TCP)")
                yield f"{case}-{scen}-{base[:-4]}", group, statistical_trace(bins, seed), info


def convert(root: str) -> Dict:
    made = list(items(root))
    override = {I.safe_id(tid): TIER_OF_PROFILE[info["profile"]] for tid, _, _, info in made}
    return I.emit(root, NAME, made, {"source": SOURCE.title, "url": URL}, tier_override=override)
