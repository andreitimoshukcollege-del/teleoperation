"""ITU-T G.1050 (07/2016)'s standard test cases (docs/adr/0035 amendment 2): per-packet delay and loss from the
Recommendation's own simulations, for each of its three service test profiles, which are the tiers.

**Source.** The Recommendation's electronic attachment (Annex H), freely available from the ITU:
``T-REC-G.1050-201607-I!!SOFT-ZST-E``, a zip of about 780 MB. (Annex H describes an ISO 9660 image; the zip as published
holds the folders directly.) ``Software/COPYRIGHT.txt``: the simulator output, the test cases and the interfering
traffic "are free from copyright restriction". ``Software/{D,G}/out.{bidi,dwn,up}/<case>-t/`` hold the simulator's
output per access technology (DSL, GPON), scenario and test case (Annex F, Table F.1):

- ``$FLOW.out.gz``: the managed flows' "delay-loss schedule" (clause 6.6, Table 3): a header (``Source:,"TIA TR30.3:
  TIA-921C"`` ... ``Time, Delay, Drop``), then one CSV row per packet, ``time,delay(ms),drop``. The VoIP flows (``VoIP1_Fwd``, ``VoIP1_Rev``: G.729, one packet every 20 ms) are the
  ones kept: the same rate and size class as the joint stream. Profiles A and B (``w``, ``p``) carry managed flows;
  profile C (``u``) has none ("There are no managed streams in the Unmanaged service test profile C").
- ``bw-{up,dwn}.csv``: the iPerf flow's delay per time bin (clause 6.6, Table 4: ``t``, bandwidth, packets,
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

from . import fetching, synth
from . import index as I
from .standards import TIER_OF_PROFILE
from .trace import LOST, Trace, from_ms

NAME = "g1050-2016"
URL = "https://www.itu.int/rec/dologin_pub.asp?lang=e&id=T-REC-G.1050-201607-I!!SOFT-ZST-E&type=items"
SOURCE = fetching.Source(
    name=NAME,
    title="ITU-T G.1050 (07/2016) electronic attachment: simulator output of the standard test cases",
    hosts=["www.itu.int"],
    licence="ITU publication, freely available; the attachment's COPYRIGHT.txt: the simulator output, test cases and interfering traffic 'are free from copyright restriction'. Not redistributed.",
    terms_url="https://www.itu.int/en/ITU-T/publications/Pages/default.aspx",
    citation="ITU-T Recommendation G.1050 (07/2016), Network model for evaluating multimedia transmission performance over Internet Protocol, Annex H.",
)
ZIP_NAME = "g1050-2016-attachment.zip"
MAX_ZIP = 1_200_000_000
MAX_MEMBER = 400_000_000
CASE = re.compile(r"^(?P<tech>[DG])(?P<profile>[wpu])(?P<severity>\d+)(-?t)?$")
MEMBER = re.compile(r"^Software/(?P<tech>[DG])/out\.(?P<scen>bidi|dwn|up)/(?P<case>[^/]+)/(?P<file>[^/]+)$")
KEEP_FLOWS = ("VoIP1_Fwd", "VoIP1_Rev")
STAT_INTERVAL_MS = 20.0


def fetch(root: str) -> fetching.Manifest:
    d = fetching.dataset_dir(root, NAME)
    old = fetching.load_manifest(root, NAME) or {}
    known = {f["path"]: f["sha256"] for f in old.get("files", [])}
    got = fetching.download(URL, os.path.join(d, "download"), ZIP_NAME, SOURCE, MAX_ZIP, known, timeout_s=120.0)
    m = fetching.Manifest(SOURCE, [got], kept="the whole attachment zip; the conversion reads its VoIP .out.gz and bw-*.csv files in place")
    fetching.save_manifest(root, m)
    return m


def _zip(root: str) -> zipfile.ZipFile:
    return zipfile.ZipFile(os.path.join(fetching.dataset_dir(root, NAME), "download", ZIP_NAME))


def listing(root: str) -> List[str]:
    """Every path in the attachment (for looking at what is in it)."""
    with _zip(root) as z:
        return z.namelist()


def _read(z: zipfile.ZipFile, info: zipfile.ZipInfo) -> bytes:
    """A member's bytes, read in place (never extracted under its own name), size-checked."""
    if info.file_size > MAX_MEMBER:
        raise ValueError(f"{info.filename}: {info.file_size} bytes")
    with z.open(info) as f:
        data = f.read(MAX_MEMBER + 1)
    if len(data) > MAX_MEMBER:
        raise ValueError(f"{info.filename}: larger than its entry said")
    return data


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
    with _zip(root) as z:
        for info in sorted(z.infolist(), key=lambda i: i.filename):
            mm = MEMBER.match(info.filename)
            if not mm:
                continue
            m = CASE.match(mm["case"])
            if not m or m["tech"] != mm["tech"]:
                continue
            fname, scen, case = mm["file"], mm["scen"], m["tech"] + m["profile"] + m["severity"]
            base = fname[:-3] if fname.lower().endswith(".gz") else fname
            info_ = {"testCase": case, "scenario": scen, "access": {"D": "DSL", "G": "GPON"}[m["tech"]],
                     "profile": m["profile"], "severity": int(m["severity"]), "source": info.filename}
            if m["profile"] in "wp" and base.endswith(".out") and base[:-4] in KEEP_FLOWS:
                times, delays = parse_out(_text(_read(z, info), fname))
                if len(times) < 100:
                    continue
                interval = float(np.median(np.diff(times))) * 1000.0
                info_.update(flow=base[:-4], kind="per-packet", sendIntervalMs=round(interval, 3))
                yield f"{case}-{scen}-{base[:-4]}", case, from_ms(delays, interval), info_
            elif m["profile"] == "u" and re.match(r"^bw-(up|dwn)\.csv$", base):
                bins = parse_bins(_text(_read(z, info), fname))
                if len(bins) < 10:
                    continue
                seed = _seed(info.filename)
                info_.update(flow=base[:-4], kind="statistical", seed=seed, sendIntervalMs=STAT_INTERVAL_MS,
                             assumption="iPerf per-bin mean and sqrt(m2 - mean^2) as a normal per 20 ms datagram, clipped at 0, FIFO; no loss (TCP)")
                yield f"{case}-{scen}-{base[:-4]}", case, statistical_trace(bins, seed), info_


def convert(root: str) -> Dict:
    made = list(items(root))
    override = {I.safe_id(tid): TIER_OF_PROFILE[info["profile"]] for tid, _, _, info in made}
    return I.emit(root, NAME, made, {"source": SOURCE.title, "url": URL}, tier_override=override)
