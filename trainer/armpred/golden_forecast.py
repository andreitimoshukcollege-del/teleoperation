"""The causal network forecasters' golden fixture for the repo's C# copy (docs/adr/0035 N3): what the headset observes
over 30 s of a bursty link, the ``persistence`` and ``rtt-half`` forecasts issued every 100 ms from it
(``net/forecast.py``), the network block ``features/layout.py`` encodes from some of them, and the effective times
``forecast.effective_times`` gives a command stream. ``core/Teleop.Core/Forecast/`` must reproduce them. The trainer is
the source, as for ``golden_learned.py``.

The observations are made up here (seeded), not simulated through a view: the forecasters see only arrival stamps,
ClockSync's send estimates, sequence numbers and round trips, so the fixture exercises exactly those, including what a
view rarely has: frames before ClockSync syncs, a 450 ms stall released as a burst, lost frames, a frame overtaken by a
later one, a round trip whose echoing frame had no send estimate, a negative round trip.

Written by ``armpred golden-forecast`` into ``core/testdata/learned/``, never by hand."""

from __future__ import annotations

import json
import os
from typing import Dict, List

import numpy as np

from .features import layout as L
from .net import forecast as FC

FORMAT = "teleop-forecast-golden"
VERSION = 1
TPS = 1_000_000_000  # the headset's clock (Android's Stopwatch: ns)
T0 = 5_000_000_000_000


def _f(x) -> float:
    """A float32 value as JSON: its exact double, so the C# cast back to float is exact."""
    return float(np.float32(x))


def observations(seed: int = 11, seconds: float = 30.0) -> FC.Observations:
    rng = np.random.Generator(np.random.PCG64(seed))
    ms = TPS // 1000
    n = int(seconds * 50)
    sync_from = T0 + int(1.5 * TPS)
    send = T0 + np.arange(n, dtype=np.int64) * 20 * ms
    delay = (12.0 + rng.gamma(2.0, 2.0, n)) * ms
    arrival = send + delay.astype(np.int64)
    stall = (send >= T0 + 10 * TPS) & (send < T0 + int(10.45 * TPS))
    release = T0 + int(10.45 * TPS) + 12 * ms
    arrival[stall] = release + np.arange(int(stall.sum())) * (ms // 10)  # held, then let go together
    keep = np.ones(n, dtype=bool)
    keep[[700, 701, 1100]] = False  # lost
    swap = 900  # overtaken: 900 arrives just after 901
    arrival[swap] = arrival[swap + 1] + ms // 2
    offset_err = (0.3 * np.sin(np.arange(n) / 200.0) * ms).astype(np.int64)  # ClockSync's estimate wanders
    send_est = np.where(arrival >= sync_from, send + offset_err, -1)
    idx = np.nonzero(keep)[0]
    order = idx[np.argsort(arrival[idx], kind="stable")]
    rtt_t, rtt, down, unc, minrtt, synced = [], [], [], [], [], []
    smooth = None
    lo = None
    for k in range(int(seconds)):
        t = T0 + k * TPS + 7 * ms
        r = int((28.0 + rng.gamma(2.0, 3.0)) * ms)
        if k == 17:
            r = -2 * ms  # a negative round trip: kept for the forecasters, rejected by ClockSync
        if k == 6:
            r = 480 * ms  # the round trip echoed during the stall
        d = int((13.0 + rng.gamma(2.0, 2.0)) * ms) if t >= sync_from + TPS else -1
        if r > 0:
            lo = r if lo is None else min(lo, r)
            smooth = r // 2 if smooth is None else smooth + int(round(0.1 * (r // 2 - smooth)))
        rtt_t.append(t)
        rtt.append(r)
        down.append(d)
        unc.append(smooth or 0)
        minrtt.append(lo or 0)
        synced.append(k >= 2)
    a = lambda v: np.asarray(v, dtype=np.int64)  # noqa: E731
    return FC.Observations(
        tps=TPS, frame_arrival=arrival[order], frame_send_est=send_est[order], frame_index=order.astype(np.int64),
        rtt_t=a(rtt_t), rtt_ticks=a(rtt), rtt_down_ticks=a(down), sync_t=a(rtt_t), sync_uncertainty=a(unc),
        sync_min_rtt=a(minrtt), sync_synced=np.asarray(synced, dtype=bool))


def _stream_json(s: FC.ForecastStream) -> Dict[str, object]:
    return {
        "validQ": s.valid_q.astype(int).tolist(), "validLoss": s.valid_loss.astype(int).tolist(),
        "validOutage": s.valid_outage.astype(int).tolist(),
        "floorMs": [[_f(x) for x in row] for row in s.floor_ms], "outage": [[_f(x) for x in row] for row in s.outage],
        "qMs": [[_f(x) for x in s.q_ms[i].reshape(-1)] for i in range(len(s))],
        "loss": [[_f(x) for x in s.loss[i].reshape(-1)] for i in range(len(s))],
    }


def build(seed: int = 11) -> Dict[str, object]:
    obs = observations(seed)
    t_issue = FC.issue_times(T0, T0 + 30 * TPS, TPS)
    streams = {level: FC.make_forecasts(level, t_issue, [], obs, seed) for level in FC.CAUSAL}
    pers = streams["persistence"]
    blocks = [i for i in range(0, len(t_issue), 7)]
    rng = np.random.Generator(np.random.PCG64(seed + 1))
    cmd_send = np.sort(T0 + (rng.random(1000) * 30 * TPS).astype(np.int64))
    cmd_eff = cmd_send + 15 * (TPS // 1000)  # RTT/2, as the headset would pass without a forecast
    return {
        "format": FORMAT, "version": VERSION, "tps": TPS, "seed": seed,
        "about": "Causal forecasters (persistence, rtt-half) of trainer/armpred/net/forecast.py, the network block of "
                 "features/layout.py and forecast.effective_times, for core/Teleop.Core/Forecast/ (docs/adr/0035 N3). "
                 "Written by armpred golden-forecast; never edit by hand.",
        "horizonsMs": list(FC.HORIZONS_MS), "quantiles": list(FC.QUANTILES),
        "frames": [[int(t), int(s), int(i)] for t, s, i in zip(obs.frame_arrival, obs.frame_send_est, obs.frame_index)],
        "roundTrips": [[int(t), int(r), int(d), int(u), int(m), int(bool(y))] for t, r, d, u, m, y in
                       zip(obs.rtt_t, obs.rtt_ticks, obs.rtt_down_ticks, obs.sync_uncertainty, obs.sync_min_rtt, obs.sync_synced)],
        "issues": [int(t) for t in t_issue],
        "streams": {level: _stream_json(s) for level, s in streams.items()},
        "blocks": {"level": "persistence", "index": blocks,
                   "values": [[_f(x) for x in L.encode_forecast(pers, i)] for i in blocks]},
        "commands": {"send": [int(x) for x in cmd_send], "effNoForecast": [int(x) for x in cmd_eff],
                     "effPersistence": [int(x) for x in FC.effective_times(cmd_send, cmd_eff, pers, TPS)],
                     "effRttHalf": [int(x) for x in FC.effective_times(cmd_send, cmd_eff, streams["rtt-half"], TPS)]},
    }


def write(out_dir: str, seed: int = 11, log=print) -> str:
    doc = build(seed)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "forecast-golden-v1.json")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    s = doc["streams"]["persistence"]
    n = len(doc["issues"])
    log(f"{path}: {len(doc['frames'])} frames, {len(doc['roundTrips'])} round trips, {n} issues "
        f"(persistence valid down {sum(v[1] for v in s['validQ'])}/{n}, up {sum(v[0] for v in s['validQ'])}/{n})")
    return path
