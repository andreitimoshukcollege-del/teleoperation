"""Relating the sender's clock to the robot's, offline, from a whole session.

A ``JC`` record says when the robot read a command (robot clock) and when the headset stamped it
(the headset's clock). The fastest commands crossed the link in about the minimum one-way delay, so
the lower envelope of ``robot_arrival - stamp`` is the offset between the clocks plus that minimum,
and its slope over the session is the clocks' drift. Windowed minima and a line through them give
both; the truth's timing is only as good as this, so a simulated session (whose mapping is known)
checks it in the tests.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

NS = 1_000_000_000


@dataclass(frozen=True)
class ClockMap:
    """robot_ns = intercept + slope * sender_ticks_in_ns (minus the assumed minimum one-way delay)."""

    slope: float
    intercept: float
    min_one_way_ns: float

    def sender_to_robot(self, sender_ns: np.ndarray) -> np.ndarray:
        return self.intercept + self.slope * np.asarray(sender_ns, dtype=np.float64) - self.min_one_way_ns


def fit_sender_clock(robot_arrival_ns: np.ndarray, sender_ticks: np.ndarray, sender_tps: int,
                     window_s: float = 20.0, min_one_way_ns: float = 500_000.0) -> ClockMap:
    sender_ns = np.asarray(sender_ticks, dtype=np.float64) * (NS / sender_tps)
    arrival = np.asarray(robot_arrival_ns, dtype=np.float64)
    if sender_ns.size < 2:
        raise ValueError("need at least two commands to relate the clocks")
    d = arrival - sender_ns
    origin = sender_ns[0]
    bins = np.floor((sender_ns - origin) / (window_s * NS)).astype(np.int64)
    xs, ys = [], []
    for b in np.unique(bins):
        sel = bins == b
        k = int(np.argmin(d[sel]))
        xs.append(sender_ns[sel][k])
        ys.append(d[sel][k])
    xs, ys = np.array(xs), np.array(ys)
    if xs.size >= 2:
        # A line through the window minima: d(x) = a + b x, so arrival = x (1 + b) + a.
        b, a = np.polyfit(xs - origin, ys, 1)
    else:
        a, b = float(ys[0]), 0.0
    slope = 1.0 + b
    intercept = a - b * origin
    return ClockMap(slope=slope, intercept=intercept, min_one_way_ns=min_one_way_ns)
