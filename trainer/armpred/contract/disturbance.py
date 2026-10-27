"""Finding sessions the headset's network disturbance altered, from the robot's side alone.

Sessions recorded before the headset reported its disturbance (HD records, 2026-10-27) say nothing about
it, but the console's LAG / JITTER / LOSS act on the uplink before the real send, so they leave marks in
RobotHost's ``JC`` records (``armpred check-disturbance``, ``just recordings-disturbed``):

- **reordering**: a command arriving after one with a higher sequence number. Jitter does that; a LAN
  almost never does;
- **loss**: sequence numbers that never arrived;
- **added delay**: each command's arrival minus its capture stamp (two clocks, so only the changes
  mean anything), against the session's own fastest arrivals: a lag switched on part-way shows as a
  step. A lag on for a whole session shows only against neighbouring sessions of the same headset and
  robot boots (``neighbour_ms``), which share the clock offset.

The headset's ADDED DELAY slider acts on its own view and leaves no mark here. numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .session import Session

WINDOW_S = 2.0
# Thresholds: a LAN over Wi-Fi reorders and loses next to nothing, and its delay rarely holds 30 ms above
# its best for two seconds.
REORDER_MAX = 0.005
LOSS_MAX = 0.01
STEP_MS = 30.0
STEP_SHARE_MAX = 0.10  # a LAN's Wi-Fi spikes held 30+ ms in up to 5% of 2 s windows (2026-10-22 test-1)
NEIGHBOUR_MS = 40.0


@dataclass
class UplinkReport:
    session: str
    start: int  # robot clock, ns: the first command's arrival
    commands: int
    reorder: float  # share of commands that arrived after a newer one
    loss: float  # share of sequence numbers never received
    baseline_ms: float  # 1st percentile of arrival minus capture (clock offset plus the best delay)
    excess_p50_ms: float
    excess_p95_ms: float
    step_share: float  # share of 2 s windows whose median delay was STEP_MS above the baseline
    neighbour_ms: Optional[float] = None  # this baseline above its neighbours' (constant lag), when known
    robot_side: bool = False  # RobotHost's own uplink impairment was on (UP records): intended, reported apart

    @property
    def reasons(self) -> List[str]:
        out = []
        if self.reorder > REORDER_MAX:
            out.append(f"{self.reorder:.1%} reordered")
        if self.loss > LOSS_MAX:
            out.append(f"{self.loss:.1%} lost")
        if self.step_share > STEP_SHARE_MAX:
            out.append(f"delay {STEP_MS:.0f}+ ms above its best in {self.step_share:.0%} of the time")
        if self.neighbour_ms is not None and self.neighbour_ms > NEIGHBOUR_MS:
            out.append(f"delay {self.neighbour_ms:.0f} ms above the neighbouring sessions all along")
        return out

    @property
    def disturbed(self) -> bool:
        """Altered by the headset: marks of a disturbance in a session the robot did not impair itself."""
        return bool(self.reasons) and not self.robot_side


def uplink_report(session: Session) -> Optional[UplinkReport]:
    """None for a session with too few commands to say."""
    c = session.commands
    if len(c) < 50:
        return None
    op_tps = 1_000_000_000
    if len(session.keepalives):
        op_tps = int(session.keepalives[0][2])
    t = c.t.astype(np.int64)
    seq = c.seq.astype(np.int64)
    running = np.maximum.accumulate(seq)
    reorder = float(np.mean(seq[1:] < running[:-1]))
    span = int(seq.max() - seq.min() + 1)
    loss = max(0.0, 1.0 - len(np.unique(seq)) / span) if span > 0 else 0.0
    d_ms = (t - c.capture.astype(np.float64) * (1e9 / op_tps)) / 1e6
    baseline = float(np.percentile(d_ms, 1))
    excess = d_ms - baseline
    window = int(WINDOW_S * 1e9)
    bins = (t - t[0]) // window
    medians = np.array([np.median(excess[bins == b]) for b in np.unique(bins)])
    return UplinkReport(
        session=str(session.meta.get("id", "")), start=int(t[0]), commands=len(c), reorder=reorder, loss=loss,
        baseline_ms=baseline, excess_p50_ms=float(np.percentile(excess, 50)), excess_p95_ms=float(np.percentile(excess, 95)),
        step_share=float(np.mean(medians > STEP_MS)) if len(medians) else 0.0,
        robot_side=bool(session.uplink_changes) or (session.meta.get("conditions", {}).get("network", {}) or {}).get(
            "uplinkProfile", "lan") not in ("lan", "none"),
    )


def compare_neighbours(reports: Sequence[UplinkReport], same_offset_ms: float = 2000.0) -> None:
    """Sets ``neighbour_ms``: each baseline against the lowest of the time-adjacent sessions that share
    its clock offset (within ``same_offset_ms``: the same headset and robot boots). Clock drift between
    sessions minutes apart is a few ms; a lag on all session long is tens to hundreds."""
    ordered = sorted(reports, key=lambda r: r.start)
    for i, r in enumerate(ordered):
        near = [o for o in ordered[max(0, i - 2):i] + ordered[i + 1:i + 3]
                if abs(o.baseline_ms - r.baseline_ms) < same_offset_ms and abs(o.start - r.start) < 2 * 3600 * 10**9]
        if near:
            r.neighbour_ms = r.baseline_ms - min(o.baseline_ms for o in near)
