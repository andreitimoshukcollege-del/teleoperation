"""Statistical traces: datagram delays drawn from what an RTT-only source measured (docs/adr/0035 decision 2).

Puffer, the FCC's MBA and RIPE Atlas do not time single datagrams at 50 Hz; they report a round trip every 2 s (Puffer's
smoothed ``tcpi_rtt``), or an hour's or a day's summary (minimum, mean, standard deviation, loss). A trace from them keeps
what they measured and states what was assumed, in its index entry:

- **one-way delay is half the round trip**, its floor half the minimum round trip and its spread half the round trip's
  standard deviation (the two directions alike);
- **the spread is a shifted gamma** above the floor with the measured mean and standard deviation (delay above a floor
  is skewed right), drawn independently per datagram: a summary says nothing of how delay is correlated in time;
- **loss is independent** (Bernoulli) at the measured rate;
- **delivery is first in, first out**: a datagram never arrives before the one sent before it (one queue), so the draws
  never invent reordering.

Every draw is from ``numpy.random.Generator(PCG64(seed))`` with the seed in the index.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .trace import LOST, Trace

TPS = 10_000_000
INTERVAL_MS = 20.0  # the joint stream's cadence (the headset's first trace: 20.3 ms)


def generator(seed: int) -> np.random.Generator:
    return np.random.Generator(np.random.PCG64(seed))


def shifted_gamma(rng: np.random.Generator, floor: float, mean: float, sd: float, n: int) -> np.ndarray:
    """``n`` draws at least ``floor`` with mean ``mean`` and standard deviation ``sd`` (constant when there is no spread)."""
    excess = mean - floor
    if excess <= 1e-9 or sd <= 1e-9:
        return np.full(n, max(mean, floor), dtype=np.float64)
    shape = (excess / sd) ** 2
    scale = sd * sd / excess
    return floor + rng.gamma(shape, scale, n)


def fifo(delays_ms: np.ndarray, lost: np.ndarray, interval_ms: float) -> np.ndarray:
    """Raise delays so no datagram arrives before the one sent before it."""
    out = delays_ms.copy()
    last_arrival = -np.inf
    for i in range(len(out)):
        if lost[i]:
            continue
        a = i * interval_ms + out[i]
        if a < last_arrival:
            out[i] = last_arrival - i * interval_ms
            a = last_arrival
        last_arrival = a
    return out


def to_trace(delays_ms: np.ndarray, lost: np.ndarray, interval_ms: float = INTERVAL_MS) -> Trace:
    d = np.round(fifo(delays_ms, lost, interval_ms) * TPS / 1000.0).astype(np.int64)
    d[lost] = LOST
    return Trace(TPS, int(round(interval_ms * TPS / 1000.0)), d)


def summary_trace(seed: int, seconds: float, floor_ms: float, mean_ms: float, sd_ms: float, loss: float,
                  interval_ms: float = INTERVAL_MS) -> Trace:
    """A trace of ``seconds`` from one-way summary statistics."""
    rng = generator(seed)
    n = max(1, int(round(seconds * 1000.0 / interval_ms)))
    delays = shifted_gamma(rng, max(0.0, floor_ms), max(0.0, mean_ms), max(0.0, sd_ms), n)
    lost = rng.random(n) < min(1.0, max(0.0, loss))
    return to_trace(delays, lost, interval_ms)


def level_trace(times_s: Sequence[float], levels_ms: Sequence[float], interval_ms: float = INTERVAL_MS,
                max_seconds: Optional[float] = None) -> Trace:
    """A trace that follows a measured delay level (interpolated linearly between samples), with no spread of its own."""
    t = np.asarray(times_s, dtype=np.float64)
    v = np.asarray(levels_ms, dtype=np.float64)
    order = np.argsort(t, kind="stable")
    t, v = t[order] - t[order][0], v[order]
    end = t[-1] if max_seconds is None else min(t[-1], max_seconds)
    sends = np.arange(0.0, end, interval_ms / 1000.0)
    delays = np.interp(sends, t, v)
    return to_trace(delays, np.zeros(len(delays), dtype=bool), interval_ms)
