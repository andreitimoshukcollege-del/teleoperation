"""Copy of ``Time/ClockSync.cs``.

Two places where C# arithmetic is not Python's, reproduced deliberately:
- ``long / 2`` truncates toward zero (``trunc_div``), where Python's ``//`` floors;
- the EWMA is ``smoothed += (long)Math.Round(alpha * (sample - smoothed))``, in double with the
  config's ``float`` alpha widened to double (0.2f is 0.20000000298...), and ``Math.Round`` rounds
  half to even, as Python's ``round`` does.

Until 2026-10-31 Core blended the two ``long`` values in single precision, which froze the offset on
a float32 grid of milliseconds (docs/adr/0008 amendment). Views built with a trainer from before then
carry that error, as the headsets of the time did.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def trunc_div(a: int, b: int) -> int:
    """C# integer division: truncates toward zero."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def round_half_even(x: float) -> int:
    """C# ``Math.Round(double)``: banker's rounding, like Python's ``round``."""
    return int(round(x))


@dataclass(frozen=True)
class ClockSyncConfig:
    history_capacity: int
    smoothing_alpha: float  # C# float
    max_acceptable_rtt_ticks: int
    outlier_rtt_multiple: float
    min_samples_before_trusted: int


def headset_config(operator_tps: int) -> ClockSyncConfig:
    """``JetRoverArmConfig``'s ClockSync defaults (unity/Teleop.Bridge/Runtime/JetRoverArmConfig.cs)."""
    return ClockSyncConfig(
        history_capacity=32,
        smoothing_alpha=0.2,
        max_acceptable_rtt_ticks=int(2000.0 / 1000.0 * operator_tps),
        outlier_rtt_multiple=3.0,
        min_samples_before_trusted=3,
    )


class ClockSync:
    def __init__(self, config: ClockSyncConfig):
        self.config = config
        self._history = [0] * config.history_capacity
        self.reset()

    def reset(self) -> None:
        self._count = 0
        self._next = 0
        self.offset_ticks = 0
        self.uncertainty_ticks = 0
        self.last_rtt_ticks = 0
        self.accepted = 0
        self.rejected = 0

    @property
    def is_synced(self) -> bool:
        return self.accepted >= self.config.min_samples_before_trusted

    @property
    def min_rtt_ticks(self) -> int:
        return min(self._history[: self._count]) if self._count else 0

    def add_round_trip(self, operator_send: int, operator_tps: int, robot_recv: int, robot_send: int,
                       robot_tps: int, operator_recv: int) -> bool:
        rate_ratio = operator_tps / robot_tps
        robot_recv_scaled = round_half_even(robot_recv * rate_ratio)
        robot_send_scaled = round_half_even(robot_send * rate_ratio)
        rtt = (operator_recv - operator_send) - (robot_send_scaled - robot_recv_scaled)
        if rtt < 0:
            self.rejected += 1
            return False

        self._history[self._next] = rtt
        self._next = (self._next + 1) % len(self._history)
        if self._count < len(self._history):
            self._count += 1
        min_rtt = min(self._history[: self._count])

        if rtt > self.config.max_acceptable_rtt_ticks:
            self.rejected += 1
            return False
        if self.accepted > 0 and rtt > int(self.config.outlier_rtt_multiple * min_rtt):
            self.rejected += 1
            return False

        offset_sample = trunc_div((operator_send - robot_recv_scaled) + (operator_recv - robot_send_scaled), 2)
        uncertainty_sample = trunc_div(rtt, 2)
        if self.accepted == 0:
            self.offset_ticks = offset_sample
            self.uncertainty_ticks = uncertainty_sample
        else:
            alpha = float(np.float32(self.config.smoothing_alpha))  # C#: double alpha = <float>
            # A long times a double: the difference is far below 2^53, so it widens exactly, as in C#.
            self.offset_ticks += round_half_even(alpha * (offset_sample - self.offset_ticks))
            self.uncertainty_ticks += round_half_even(alpha * (uncertainty_sample - self.uncertainty_ticks))
        self.last_rtt_ticks = rtt
        self.accepted += 1
        return True

    def to_operator_ticks(self, robot_ticks: int, robot_tps: int, operator_tps: int) -> int:
        rate_ratio = operator_tps / robot_tps
        return round_half_even(robot_ticks * rate_ratio) + self.offset_ticks
