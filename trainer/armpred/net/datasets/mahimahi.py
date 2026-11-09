"""Mahimahi's cellular traces (docs/adr/0035 decision 2, very bad): the delivery opportunities of real LTE, UMTS and
EVDO links, measured with Saturator (Winstein, Sivaraman and Balakrishnan, NSDI 2013), replayed through a queue
carrying the joint stream.

**Source.** ``github.com/ravinet/mahimahi``'s ``traces/`` (GPL-3.0, the repository's licence; no separate licence for
the traces): 8 links, a ``.down`` and an ``.up`` each. A line is the millisecond at which one 1500-byte packet could
leave the bottleneck; a repeated millisecond is several (``traces/README``).

**Conversion.** Datagrams of ``DATAGRAM_BYTES`` every 20 ms enter a first-in-first-out queue in front of the link; each
opportunity carries up to 1500 bytes of what is queued (a datagram may span opportunities). A datagram's delay is when
its last byte leaves minus when it was sent. A datagram not delivered within ``MAX_AGE_MS`` is lost and what is left of it
never uses the link: the 3 s after which the FCC's MBA counts a UDP packet lost (Technical Appendix, 2023), where an
unbounded queue would turn a driving trace's dead zones into delays of a minute. The trace has no propagation delay of its
own: the link's capacity and its outages are what it adds, and the training mix adds a base delay. One pass, no loop;
datagrams still queued when the trace ends are left out. A link's two directions are one group in the split.
"""

from __future__ import annotations

import os
from typing import Dict, Iterator, List, Tuple

import numpy as np

from . import fetching, synth
from . import index as I
from .trace import Trace

NAME = "mahimahi"
BASE = "https://raw.githubusercontent.com/ravinet/mahimahi/master/traces/"
LINKS = ("ATT-LTE-driving-2016", "ATT-LTE-driving", "TMobile-LTE-driving", "TMobile-LTE-short", "TMobile-UMTS-driving",
         "Verizon-EVDO-driving", "Verizon-LTE-driving", "Verizon-LTE-short")
SOURCE = fetching.Source(
    name=NAME,
    title="Mahimahi cellular link traces (Saturator, NSDI 2013)",
    hosts=["raw.githubusercontent.com"],
    licence="GPL-3.0 (the mahimahi repository's COPYING); no separate licence for the trace files. Not redistributed.",
    terms_url="https://github.com/ravinet/mahimahi/blob/master/COPYING",
    citation="K. Winstein, A. Sivaraman, H. Balakrishnan, Stochastic Forecasts Achieve High Throughput and Low Delay over Cellular Networks, NSDI 2013; R. Netravali et al., Mahimahi, USENIX ATC 2015.",
)
MAX_FILE = 20_000_000
PACKET_BYTES = 1500
DATAGRAM_BYTES = 284  # the joint datagram's maximum (JointDatagram.MaxBytes 256) plus IPv4 and UDP headers
MAX_AGE_MS = 3000.0  # FCC MBA: a UDP packet not answered within three seconds is lost


def fetch(root: str) -> fetching.Manifest:
    d = fetching.dataset_dir(root, NAME)
    old = fetching.load_manifest(root, NAME) or {}
    known = {f["path"]: f["sha256"] for f in old.get("files", [])}
    files = [fetching.download(BASE + f"{link}.{way}", os.path.join(d, "download"), f"{link}.{way}", SOURCE, MAX_FILE, known)
             for link in LINKS for way in ("down", "up")]
    m = fetching.Manifest(SOURCE, files, kept="all 16 trace files")
    fetching.save_manifest(root, m)
    return m


def parse(text: str) -> np.ndarray:
    """Opportunity times (ms), in order. Anything that is not a non-negative integer line is refused."""
    out: List[int] = []
    last = -1
    for n, line in enumerate(text.splitlines(), start=1):
        line = line.strip()
        if not line:
            continue
        if not line.isdigit():
            raise ValueError(f"line {n}: not a millisecond count: {line[:40]!r}")
        v = int(line)
        if v < last:
            raise ValueError(f"line {n}: time goes backwards")
        out.append(v)
        last = v
    if not out:
        raise ValueError("no opportunities")
    return np.asarray(out, dtype=np.int64)


def replay(opportunities_ms: np.ndarray, interval_ms: float = synth.INTERVAL_MS, datagram_bytes: int = DATAGRAM_BYTES,
           max_age_ms: float = MAX_AGE_MS) -> Trace:
    """The joint stream through the link, one pass: a datagram every ``interval_ms`` while the trace lasts."""
    end = float(opportunities_ms[-1])
    sends = np.arange(0.0, end, interval_ms)
    departs = np.full(len(sends), np.nan)
    lost = np.zeros(len(sends), dtype=bool)
    head = 0  # first datagram not yet fully sent
    left = datagram_bytes  # bytes of the head datagram still queued
    for t in opportunities_ms:
        budget = PACKET_BYTES
        while head < len(sends) and sends[head] <= t:
            if t - sends[head] > max_age_ms:
                lost[head] = True  # not delivered within the limit: dropped, and what is left of it never uses the link
                head += 1
                left = datagram_bytes
                continue
            if budget == 0:
                break
            take = min(budget, left)
            budget -= take
            left -= take
            if left == 0:
                departs[head] = t
                head += 1
                left = datagram_bytes
    last = head  # first in, first out: everything before the head was sent or dropped; the rest was still queued at the end
    delays = np.where(lost[:last], 0.0, departs[:last] - sends[:last])
    return synth.to_trace(delays, lost[:last], interval_ms)


def items(root: str) -> Iterator[Tuple[str, str, Trace, Dict]]:
    d = os.path.join(fetching.dataset_dir(root, NAME), "download")
    for link in LINKS:
        for way in ("down", "up"):
            p = os.path.join(d, f"{link}.{way}")
            with open(p, encoding="ascii") as f:
                ops = parse(f.read())
            yield f"{link}-{way}", link, replay(ops), {
                "link": link, "direction": way, "seconds": round(float(ops[-1]) / 1000.0, 3), "datagramBytes": DATAGRAM_BYTES,
                "sendIntervalMs": synth.INTERVAL_MS, "kind": "replayed",
                "maxAgeMs": MAX_AGE_MS,
                "assumption": "FIFO queue, a datagram not delivered within 3 s lost (FCC MBA timeout); no propagation delay of its own; the tail still queued at the end left out"}


def convert(root: str) -> Dict:
    return I.emit(root, NAME, items(root), {"source": SOURCE.title, "url": BASE})
