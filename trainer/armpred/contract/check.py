"""``armpred check-session`` (``just recording-check``): is a session fit to train or test on?

A failure means the session must not be packed (``just pack-dataset`` refuses it). Every check
reports what it measured, so a failure says where to look, not just that something is wrong.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from .jlog import JlogError
from .session import Session, load_session

# A RobotHost feedback value must come from a driver read at most this long before it (ns). The
# relay forwards at 20 Hz and calls a reading stale after 0.2 s (robot/ros relay_node.py), but the
# driver publishes a cycle's readings only when the cycle ends, and one missed read holds a cycle for
# its 1 s timeout; 1.5 s covers that. Clocks that were not shared miss by hours, not seconds.
CAUSALITY_WINDOW_NS = 1_500_000_000
# The share of feedback values allowed to have no matching read before the clocks are called
# inconsistent. Not zero: the relay keeps a value until it is stale, and a read can land between
# RobotHost's poll and the relay's forward.
CAUSALITY_MAX_UNMATCHED = 0.01
# The longest a measured motor may go without a successful read (ns).
MAX_READ_GAP_NS = 500_000_000


@dataclass
class CheckReport:
    session_id: str
    failures: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    stats: Dict[str, object] = field(default_factory=dict)
    read_gaps: List[Dict[str, float]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.failures

    def lines(self) -> List[str]:
        out = [f"session {self.session_id}: {'OK' if self.ok else 'FAILED'}"]
        out += [f"  FAIL {f}" for f in self.failures]
        out += [f"  warn {w}" for w in self.warnings]
        out += [f"  {k}: {v}" for k, v in self.stats.items()]
        return out


def check_session_dir(directory: str, allow_read_gaps: bool = False) -> CheckReport:
    try:
        session = load_session(directory)
    except (JlogError, OSError, ValueError, KeyError) as e:
        report = CheckReport(session_id=directory)
        report.failures.append(f"unreadable: {e}")
        return report
    return check_session(session, allow_read_gaps=allow_read_gaps)


def check_session(session: Session, allow_read_gaps: bool = False) -> CheckReport:
    report = CheckReport(session_id=session.id)
    tps = session.ticks_per_second

    for name, log in session.files.items():
        if log.dropped:
            report.failures.append(f"{name}.jlog dropped {log.dropped} records (writer ring full)")

    if session.source == "robot":
        for name, build in (session.meta.get("builds") or {}).items():
            if build.get("dirty"):
                report.failures.append(f"{name} was built from a dirty tree ({build.get('sha')})")
            if not build.get("sha"):
                report.failures.append(f"{name} build sha is missing")

    if len(session.commands) == 0:
        report.warnings.append("no joint commands")
    if len(session.frames) == 0:
        report.failures.append("no joint-state frames")
    if len(session.reads) == 0:
        report.failures.append("no driver reads")
        return report

    _check_ordering(session, report)
    _check_causality(session, report)
    _check_read_gaps(session, report, allow_read_gaps)

    if len(session.subscriber_changes) > 1:
        report.failures.append(
            f"the joint-state stream changed subscriber {len(session.subscriber_changes) - 1} time(s) "
            "(something else sent on the joint port, e.g. joint-probe)")
    if len(session.sender_changes) > 1:
        report.failures.append(f"the joint-command sender changed {len(session.sender_changes) - 1} time(s)")

    duration = (int(session.reads.t_resp.max()) - int(session.reads.t_req.min())) / tps
    report.stats["duration_s"] = round(duration, 1)
    report.stats["commands"] = len(session.commands)
    report.stats["frames"] = len(session.frames)
    if len(session.commands):
        outcomes = {chr(o): int(n) for o, n in zip(*np.unique(session.commands.outcome, return_counts=True))}
        report.stats["command_outcomes"] = outcomes
        if outcomes.get("R", 0):
            report.warnings.append(f"{outcomes['R']} joint command(s) refused as colliding (ADR 0020)")
        if outcomes.get("P", 0):
            report.warnings.append(
                f"{outcomes['P']} joint command(s) ignored while the arm was parking or relaxed (ADR 0023 decision 5)")
    per_motor = {}
    for m in np.unique(session.reads.motor):
        sel = session.reads.motor == m
        ok = int(session.reads.ok[sel].sum())
        per_motor[int(m)] = f"{ok}/{int(sel.sum())} ok, {ok / duration:.1f} Hz" if duration > 0 else f"{ok}/{int(sel.sum())} ok"
    report.stats["reads"] = per_motor
    return report


def _check_ordering(session: Session, report: CheckReport) -> None:
    for name, t in (
        # JC is written when the plant takes the command, so it is in tDeliver order; its socket
        # time t can go back when a robot-side uplink impairment reorders.
        ("JC", session.commands.t_deliver),
        ("FR", session.frames.t),
        ("RD", session.reads.t_req),
        ("WR", session.writes.t),
    ):
        if len(t) > 1 and np.any(np.diff(t) < 0):
            report.failures.append(f"{name} records go back in time")
    bad = session.reads.t_resp < session.reads.t_req
    if np.any(bad):
        report.failures.append(f"{int(bad.sum())} reads end before they start")


def _check_causality(session: Session, report: CheckReport) -> None:
    """Every valid feedback value RobotHost took must be a value the driver read shortly before.

    This is what proves the two logs share a clock: the match is on time *and* value, so two
    clocks hours apart, or a log of another session, fail it."""
    rin = session.relay_in
    valid = rin.extra > 0.5
    if not np.any(valid):
        report.warnings.append("no valid relay feedback in robothost.jlog; clock causality not checked")
        return
    reads = session.reads
    unmatched = 0
    checked = 0
    for m in np.unique(rin.motor[valid]):
        sel_reads = (reads.motor == m) & reads.ok
        r_t = reads.t_resp[sel_reads]
        r_p = reads.pulse[sel_reads]
        order = np.argsort(r_t, kind="stable")
        r_t, r_p = r_t[order], r_p[order]
        sel = valid & (rin.motor == m)
        f_t, f_p = rin.t[sel], rin.pulse[sel]
        checked += len(f_t)
        # Any read of this value in the window counts: the relay keeps a value until it is stale,
        # and an unchanged position is read many times over.
        lo = np.searchsorted(r_t, f_t - CAUSALITY_WINDOW_NS, side="left")
        hi = np.searchsorted(r_t, f_t, side="right")
        for k in range(len(f_t)):
            window = r_p[lo[k]:hi[k]]
            if window.size == 0 or not np.any(window == int(round(f_p[k]))):
                unmatched += 1
    share = unmatched / checked if checked else 0.0
    report.stats["clock_causality"] = f"{checked - unmatched}/{checked} feedback values match a read <= 1.5 s earlier"
    if share > CAUSALITY_MAX_UNMATCHED:
        report.failures.append(
            f"clock causality: {unmatched}/{checked} ({share:.1%}) feedback values have no matching driver read "
            "in the 1.5 s before them -- the two logs do not share a clock, or one is from another session")


def _check_read_gaps(session: Session, report: CheckReport, allow: bool) -> None:
    reads = session.reads
    tps = session.ticks_per_second
    worst = {}
    for m in np.unique(reads.motor):
        sel = (reads.motor == m) & reads.ok
        t = np.sort(reads.t_mid[sel])
        if t.size < 2:
            report.failures.append(f"motor {int(m)}: fewer than two successful reads")
            continue
        gaps = np.diff(t)
        worst[int(m)] = float(gaps.max()) / tps * 1000.0
        for i in np.nonzero(gaps > MAX_READ_GAP_NS)[0]:
            report.read_gaps.append({
                "motor": int(m),
                "from_s": float(t[i] - t[0]) / tps,
                "length_ms": float(gaps[i]) / tps * 1000.0,
            })
    report.stats["longest_read_gap_ms"] = {m: round(v, 1) for m, v in worst.items()}
    if report.read_gaps:
        message = (f"{len(report.read_gaps)} read gap(s) over {MAX_READ_GAP_NS // 1_000_000} ms "
                   f"(longest {max(g['length_ms'] for g in report.read_gaps):.0f} ms)")
        if allow:
            report.warnings.append(message + "; allowed, recorded for the index so training skips them")
        else:
            report.failures.append(message + " -- the board may have gone silent; pass --allow-read-gaps to keep it annotated")
