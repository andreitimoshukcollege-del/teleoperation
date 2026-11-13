"""One session + one schedule + one seed -> the headset's predictor call sequence, and the truth.

The headset side is a copy of ``JetRoverJointStateLink`` (unity/Teleop.Bridge/Runtime):
- it processes arrived frames on each display tick (about 90 Hz);
- ``ClockSync.AddRoundTrip`` runs only when a frame's echoed operator stamp changes, with the frame's
  *true* arrival;
- a frame's sensed time is ``sensed != 0 ? sensed : robotSend``, converted only once synced; frames
  decoded before sync never reach the predictor;
- a frame is held back ``AddedDelayMs`` before ``ObserveState``;
- a command's effective time is its send time plus ``OffsetUncertaintyTicks`` (RTT/2) when synced,
  and its speed is the largest speed among its targets.

The robot side is the recording itself: when the robot read each command, what it sent in each
frame, and when the driver read each servo. Only the network between them is synthesised.

**Every command the headset sent reaches the predictor** (ADR 0021 decision 13), including the ones
the uplink lost, because the headset's predictor saw them too. A session with ``operator.jlog`` (every
simulated one) says exactly which; for a recorded one without it, a gap in the command sequence
numbers is filled by interpolating its neighbours, and marked.

A view also keeps what the network forecasters need (``armpred.net.forecast``): each frame's send
time as the headset estimated it, every keepalive round trip, each command's true uplink delay, and
in ``meta`` enough to rebuild both directions' true conditions (``view_truths``).
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..contract.profile import RobotProfile
from ..contract.session import MAX_JOINTS, Session
from ..net.clocksync import ClockSync, headset_config, round_half_even
from ..net.forecast import ForecastStream, LinkTruth, Observations
from .clockmap import ClockMap, fit_sender_clock
from .schedule import Schedule, Segment, build_downlink

NS = 1_000_000_000
HEADSET_TICK_NS = NS / 90.0
# 3 (2026-10-25): the gripper's truth (motor 10) is its 0..1 fraction, as its frames and commands carry
# it, not the arm's angle formula. Older views are read with their gripper truth dropped.
VIEW_VERSION = 3
GRIPPER_TRUTH_MOTOR = 10  # the JetRover's gripper (robot profile role GripperMain)
PASSTHROUGH_PROFILES = ("lan", "none")
# A recorded session's LAN under the robot-side impairment (the clock fit assumes 0.5 ms minimum).
RECORDED_LAN_BASE_MS, RECORDED_LAN_JITTER_MS = 1.0, 0.5
# Gaps in the command sequence longer than this are a restart, not a loss.
MAX_RECONSTRUCTED_GAP = 25


@dataclass
class View:
    meta: dict
    # ObserveCommand calls, in call order
    cmd_t: np.ndarray  # int64 operator ticks: when the call was made (the send)
    cmd_eff: np.ndarray  # int64 effective ticks passed to the predictor
    cmd_speed: np.ndarray  # float32
    cmd_count: np.ndarray  # int16
    cmd_motor: np.ndarray  # int16 [n, MAX_JOINTS]
    cmd_value: np.ndarray  # float32 [n, MAX_JOINTS]
    # ObserveState calls, in call order
    st_t: np.ndarray  # int64 operator ticks: the display tick that released it
    st_sensed: np.ndarray  # int64 sensed ticks passed to the predictor
    st_count: np.ndarray
    st_motor: np.ndarray
    st_value: np.ndarray
    st_flags: np.ndarray  # uint8, bit 0 = sensed
    # The truth: every successful driver read, on the operator's clock
    tr_t: np.ndarray  # int64
    tr_motor: np.ndarray  # int16
    tr_value: np.ndarray  # float64 rad
    # Diagnostics
    frame_send: np.ndarray  # int64 operator ticks
    frame_delivered: np.ndarray  # bool
    frame_arrival: np.ndarray  # int64
    frame_segment: np.ndarray  # int16
    sync_t: np.ndarray  # int64: when each round trip was offered to ClockSync
    sync_offset: np.ndarray
    sync_uncertainty: np.ndarray
    sync_accepted: np.ndarray  # bool
    events_t: np.ndarray  # int64 operator ticks
    events_kind: List[str] = field(default_factory=list)
    events_detail: List[str] = field(default_factory=list)
    # Version 2 (ADR 0021 decision 13); a version-1 file loads with the defaults.
    cmd_lost: Optional[np.ndarray] = None  # bool: never reached the robot (lost on the uplink)
    cmd_reconstructed: Optional[np.ndarray] = None  # bool: not in the recording; filled from a sequence gap
    cmd_true_uplink: Optional[np.ndarray] = None  # int64 ticks from the send to the plant; -1 if lost
    frame_send_est: Optional[np.ndarray] = None  # int64: the send time as the headset estimated it; -1 before sync
    sync_rtt: Optional[np.ndarray] = None  # int64: every round trip's RTT, accepted or not
    sync_frame: Optional[np.ndarray] = None  # int64: the frame that echoed it
    sync_min_rtt: Optional[np.ndarray] = None  # int64: ClockSync's window minimum after it
    sync_synced: Optional[np.ndarray] = None  # bool: ClockSync trusted after it
    forecasts: Optional[ForecastStream] = None  # saved with a view only when one was attached

    @property
    def tps(self) -> int:
        return int(self.meta["operatorTicksPerSecond"])

    def __post_init__(self) -> None:
        n, s = len(self.cmd_t), len(self.sync_t)
        if self.cmd_lost is None:
            self.cmd_lost = np.zeros(n, dtype=bool)
        if self.cmd_reconstructed is None:
            self.cmd_reconstructed = np.zeros(n, dtype=bool)
        if self.cmd_true_uplink is None:
            self.cmd_true_uplink = np.full(n, -1, dtype=np.int64)
        if self.frame_send_est is None:
            self.frame_send_est = np.full(len(self.frame_send), -1, dtype=np.int64)
        if self.sync_rtt is None:
            self.sync_rtt = np.full(s, -1, dtype=np.int64)
        if self.sync_frame is None:
            self.sync_frame = np.full(s, -1, dtype=np.int64)
        if self.sync_min_rtt is None:
            self.sync_min_rtt = np.zeros(s, dtype=np.int64)
        if self.sync_synced is None:
            self.sync_synced = np.cumsum(self.sync_accepted.astype(np.int64)) >= 3


def _session_start(session: Session) -> int:
    candidates = [int(session.reads.t_req.min())]
    if len(session.commands):
        candidates.append(int(session.commands.t.min()))
    if len(session.frames):
        candidates.append(int(session.frames.t.min()))
    return min(candidates)


def sender_clock(session: Session) -> Tuple[ClockMap, int]:
    """The original headset's clock against the robot's: exact for a simulated session (its latents
    say), fitted from the lower envelope of the command arrivals for a recorded one."""
    sender_tps = int(session.keepalives[0, 2]) if len(session.keepalives) else 1_000_000_000
    if session.latents is not None and "operator_uptime_ns" in session.latents.params:
        lp = session.latents.params
        uptime = int(lp["operator_uptime_ns"])
        drift = float(lp["operator_clock_drift_ppm"]) * 1e-6
        start = int(lp["robot_boot_ns"])
        # The simulator's clock: op = uptime + (t - start) * (1 + drift).
        slope = 1.0 / (1.0 + drift)
        return ClockMap(slope=slope, intercept=start - uptime * slope, min_one_way_ns=0.0), sender_tps
    if len(session.commands) < 2:
        raise ValueError(f"{session.id}: too few commands to relate the clocks")
    return fit_sender_clock(session.commands.t, session.commands.capture, sender_tps), sender_tps


@dataclass
class _Commands:
    """Every command the headset sent: what it carried, and what became of it at the robot."""

    capture: np.ndarray  # int64, sender ticks
    count: np.ndarray
    motor: np.ndarray
    angle: np.ndarray
    speed: np.ndarray
    t_deliver: np.ndarray  # int64 robot ticks; -1 if lost
    lost: np.ndarray
    reconstructed: np.ndarray


def _commands(session: Session) -> _Commands:
    c = session.commands
    oc = session.operator_commands
    if oc is not None and len(oc):
        # operator.jlog lists every send; the robot's JC lines say which arrived (matched by sequence).
        by_seq = {int(s): i for i, s in enumerate(c.seq)}
        hit = np.array([by_seq.get(int(s), -1) for s in oc.seq], dtype=np.int64)
        t_deliver = np.where(hit >= 0, c.t_deliver[np.maximum(hit, 0)], -1).astype(np.int64)
        return _Commands(oc.capture.astype(np.int64), oc.count, oc.motor, oc.angle, oc.speed, t_deliver,
                         hit < 0, np.zeros(len(oc), dtype=bool))
    # No operator log: the recording's commands, plus one per short gap in their sequence numbers.
    order = np.argsort(c.seq, kind="stable")
    rows: List[Tuple] = []
    for a, b in zip(order[:-1], order[1:]):
        rows.append((int(a), None))
        gap = int(c.seq[b] - c.seq[a])
        if 1 < gap <= MAX_RECONSTRUCTED_GAP and c.capture[b] > c.capture[a]:
            for k in range(1, gap):
                rows.append((int(a), (int(b), k / gap)))
    if len(order):
        rows.append((int(order[-1]), None))
    n = len(rows)
    width = c.motor.shape[1] if c.motor.ndim == 2 else MAX_JOINTS
    out = _Commands(np.zeros(n, np.int64), np.zeros(n, np.int16), np.zeros((n, width), np.int16),
                    np.zeros((n, width), np.float32), np.zeros((n, width), np.float32),
                    np.full(n, -1, np.int64), np.zeros(n, bool), np.zeros(n, bool))
    for r, (a, fill) in enumerate(rows):
        out.count[r], out.motor[r], out.speed[r] = c.count[a], c.motor[a], c.speed[a]
        if fill is None:
            out.capture[r], out.angle[r], out.t_deliver[r] = c.capture[a], c.angle[a], c.t_deliver[a]
        else:
            b, u = fill
            out.capture[r] = int(round(c.capture[a] + (c.capture[b] - c.capture[a]) * u))
            out.angle[r] = (c.angle[a] + (c.angle[b] - c.angle[a]) * u).astype(np.float32)
            out.lost[r] = out.reconstructed[r] = True
    return out


def _uplink_spec(session: Session, t0: int) -> Dict[str, object]:
    """What the robot side did to the uplink, as a schedule on its own origin (see ``view_truths``)."""
    lp = session.latents.params if session.latents is not None else {}
    if "uplink_schedule" in lp or "uplink_profile" in lp:
        if lp.get("uplink_schedule"):
            schedule = Schedule.from_json(lp["uplink_schedule"])
        else:
            schedule = Schedule(segments=(Segment(0.0, lp.get("uplink_profile", "lan"), True),))
        return {"schedule": schedule.to_json(), "originRobotNs": int(lp["robot_boot_ns"]),
                "lanBaseMs": float(lp.get("lan_one_way_ms", 2.0)), "lanJitterMs": float(lp.get("lan_jitter_ms", 1.0))}
    segments = [Segment(0.0, "none", True)]
    for t, name, _ in session.uplink_changes:
        start = max(0.0, (int(t) - t0) / NS)
        if segments and abs(segments[-1].start_s - start) < 1e-9:
            segments[-1] = Segment(start, name, True)
        else:
            segments.append(Segment(start, name, True))
    return {"schedule": Schedule(segments=tuple(segments)).to_json(), "originRobotNs": int(t0),
            "lanBaseMs": RECORDED_LAN_BASE_MS, "lanJitterMs": RECORDED_LAN_JITTER_MS}


def synthesize(session: Session, profile: RobotProfile, schedule: Schedule, seed: int,
               traces_dir: Optional[str] = None, operator_tps: int = 1_000_000_000) -> View:
    rng = np.random.default_rng(np.random.PCG64(seed))
    t0 = _session_start(session)
    end = int(max(session.reads.t_resp.max(), session.frames.t.max() if len(session.frames) else 0))
    uptime = int(rng.uniform(0.1, 72.0) * 3600 * NS)
    drift = float(rng.uniform(-50.0, 50.0)) * 1e-6
    ratio = operator_tps / NS

    def op(t_robot) -> np.ndarray:
        """The virtual headset's clock at robot time(s): its own uptime, running ppm fast or slow."""
        t = np.asarray(t_robot, dtype=np.float64)
        return (uptime * ratio + np.round((t - t0) * (1.0 + drift) * ratio)).astype(np.int64)

    cmap, sender_tps = sender_clock(session)
    rel_s = lambda t: (np.asarray(t, dtype=np.float64) - t0) / NS  # noqa: E731

    # -- commands: every one the headset sent, when (the uplink shift moves sends earlier) -----------
    c = _commands(session)
    robot_send = cmap.sender_to_robot(c.capture.astype(np.float64) * (NS / sender_tps))
    shift_ns = schedule.uplink.ms_at(rel_s(robot_send)) * 1e6
    virtual_send = op(robot_send - shift_ns)
    order = np.argsort(virtual_send, kind="stable")
    cmd_send = virtual_send[order]
    # JetRoverJointStateLink.OnJointCommandSent: the largest speed among the targets, from 0.
    cmd_speed = np.array([max([float(c.speed[i, k]) for k in range(int(c.count[i]))] + [0.0]) for i in order],
                         dtype=np.float32)
    delivered_cmd = c.t_deliver[order] >= 0
    cmd_true_uplink = np.where(delivered_cmd, op(np.maximum(c.t_deliver[order], 0)) - cmd_send, -1).astype(np.int64)

    # -- frames: each through the downlink of the segment it was sent in --------------------------
    f = session.frames
    frame_send = op(f.robot_send)
    delivered, arrival, segment = build_downlink(schedule, frame_send, rel_s(f.robot_send), seed, operator_tps, traces_dir)
    arrival = np.where(delivered, np.maximum(arrival, frame_send), arrival)
    # The echoed keepalive stamps were the original headset's; move them to the virtual one.
    echo_robot = cmap.sender_to_robot(f.echo_operator.astype(np.float64) * (NS / sender_tps))
    echo_virtual = op(echo_robot - schedule.uplink.ms_at(rel_s(echo_robot)) * 1e6)
    deliver_order = np.nonzero(delivered)[0]
    deliver_order = deliver_order[np.lexsort((deliver_order, arrival[deliver_order]))]

    # -- the headset: display ticks, ClockSync, the hold-back, the predictor calls ----------------
    sync = ClockSync(headset_config(operator_tps))
    robot_tps = session.ticks_per_second
    rate = operator_tps / robot_tps
    tick_jitter = rng.normal(0.0, 0.4e6 * ratio, size=int((end - t0) / NS * 90) + 200)
    added = int(schedule.added_delay_ms / 1000.0 * operator_tps)
    st_rows: List[Tuple[int, int, int]] = []  # (observe tick, sensed ticks, frame index)
    cmd_eff = np.zeros(len(cmd_send), dtype=np.int64)
    sync_rows: List[Tuple] = []
    frame_send_est = np.full(len(frame_send), -1, dtype=np.int64)
    queue: List[Tuple[int, int, int]] = []  # (release, sensed operator ticks or -1, frame index)
    last_echo = None
    fi = 0  # next frame in delivery order
    ci = 0  # next command
    tick = int(op(t0))
    k = 0
    stop = int(op(end))
    while tick <= stop or ci < len(cmd_send):
        # Commands sent since the last tick: the ClockSync state they saw is the last tick's.
        while ci < len(cmd_send) and cmd_send[ci] <= tick:
            uplink = sync.uncertainty_ticks if sync.is_synced else 0
            cmd_eff[ci] = int(cmd_send[ci]) + uplink
            ci += 1
        # Frames that have arrived: ClockSync with the true arrival, then into the hold-back queue.
        while fi < len(deliver_order) and arrival[deliver_order[fi]] <= tick:
            j = int(deliver_order[fi])
            fi += 1
            echo = int(echo_virtual[j])
            if echo != last_echo:
                last_echo = echo
                # The RTT ClockSync computes, kept whether or not it is accepted (the forecasters' input).
                rtt = (int(arrival[j]) - echo) - (round_half_even(int(f.robot_send[j]) * rate)
                                                  - round_half_even(int(f.echo_robot_recv[j]) * rate))
                accepted = sync.add_round_trip(echo, operator_tps, int(f.echo_robot_recv[j]), int(f.robot_send[j]),
                                               robot_tps, int(arrival[j]))
                sync_rows.append((int(arrival[j]), sync.offset_ticks, sync.uncertainty_ticks, accepted, rtt, j,
                                  sync.min_rtt_ticks, sync.is_synced))
            if sync.is_synced:
                frame_send_est[j] = sync.to_operator_ticks(int(f.robot_send[j]), robot_tps, operator_tps)
            robot_time = int(f.sensed[j]) if f.sensed[j] != 0 else int(f.robot_send[j])
            sensed_op = sync.to_operator_ticks(robot_time, robot_tps, operator_tps) if sync.is_synced else -1
            queue.append((int(arrival[j]) + added, sensed_op, j))
        # Released frames become the measurement, and reach the predictor if they were synced.
        if queue:
            keep = []
            for release, sensed_op, j in queue:
                if release <= tick:
                    if sensed_op != -1:
                        st_rows.append((tick, sensed_op, j))
                else:
                    keep.append((release, sensed_op, j))
            queue = keep
        if tick > stop and ci >= len(cmd_send):
            break
        tick = int(tick + HEADSET_TICK_NS * ratio + (tick_jitter[k] if k < len(tick_jitter) else 0.0))
        k += 1

    # -- assemble -------------------------------------------------------------------------------
    st_idx = np.array([r[2] for r in st_rows], dtype=np.int64)
    ok = session.reads.ok
    tr_motor = session.reads.motor[ok]
    if session.reads.value is not None:
        # Session v2: the truth is the measured value itself, in the profile's joint coordinates.
        tr_value = session.reads.value[ok].astype(np.float64)
    else:
        tr_value = np.zeros(int(ok.sum()), dtype=np.float64)
        pulses = session.reads.pulse[ok]
        for m in np.unique(tr_motor):
            sel = tr_motor == m
            tr_value[sel] = profile.pulse_to_value(int(m), pulses[sel])
    events = _events(session, schedule, t0, op)
    meta = {
        "format": "jetrover-view",
        "version": VIEW_VERSION,
        "session": session.id,
        "source": session.source,
        "schedule": schedule.describe(),
        "namedOnly": schedule.only_named,
        "seed": int(seed),
        "operatorTicksPerSecond": int(operator_tps),
        "robotTicksPerSecond": int(robot_tps),
        "operatorUptimeNs": uptime,
        "operatorDriftPpm": drift * 1e6,
        "conditions": session.meta.get("conditions", {}),
        "t0RobotNs": int(t0),
        "downlinkSchedule": schedule.to_json(),
        "uplink": _uplink_spec(session, t0),
        "commandsLost": int(c.lost.sum()),
        "commandsReconstructed": int(c.reconstructed.sum()),
    }
    col = lambda i, dtype: np.array([r[i] for r in sync_rows], dtype=dtype)  # noqa: E731
    return View(
        meta=meta,
        cmd_t=cmd_send.astype(np.int64),
        cmd_eff=cmd_eff,
        cmd_speed=cmd_speed,
        cmd_count=c.count[order].astype(np.int16),
        cmd_motor=c.motor[order].astype(np.int16),
        cmd_value=c.angle[order].astype(np.float32),
        st_t=np.array([r[0] for r in st_rows], dtype=np.int64),
        st_sensed=np.array([r[1] for r in st_rows], dtype=np.int64),
        st_count=f.count[st_idx].astype(np.int16) if len(st_idx) else np.zeros(0, np.int16),
        st_motor=f.motor[st_idx].astype(np.int16) if len(st_idx) else np.zeros((0, f.motor.shape[1]), np.int16),
        st_value=f.value[st_idx].astype(np.float32) if len(st_idx) else np.zeros((0, f.motor.shape[1]), np.float32),
        st_flags=f.flags[st_idx].astype(np.uint8) if len(st_idx) else np.zeros((0, f.motor.shape[1]), np.uint8),
        tr_t=op(session.reads.t_mid[ok]),
        tr_motor=tr_motor.astype(np.int16),
        tr_value=tr_value,
        frame_send=frame_send,
        frame_delivered=delivered,
        frame_arrival=arrival,
        frame_segment=segment.astype(np.int16),
        sync_t=col(0, np.int64),
        sync_offset=col(1, np.int64),
        sync_uncertainty=col(2, np.int64),
        sync_accepted=col(3, bool),
        events_t=np.array([e[0] for e in events], dtype=np.int64),
        events_kind=[e[1] for e in events],
        events_detail=[e[2] for e in events],
        cmd_lost=c.lost[order],
        cmd_reconstructed=c.reconstructed[order],
        cmd_true_uplink=cmd_true_uplink,
        frame_send_est=frame_send_est,
        sync_rtt=col(4, np.int64),
        sync_frame=col(5, np.int64),
        sync_min_rtt=col(6, np.int64),
        sync_synced=col(7, bool),
    )


def _events(session: Session, schedule: Schedule, t0: int, op) -> List[Tuple[int, str, str]]:
    """Everything an event-windowed metric may align on, on the virtual headset's clock: network
    switches (this view's), uplink changes (the robot side's ``UP`` records, and this view's slow
    shift), and the session's own events (simulated latents, marks)."""
    out: List[Tuple[int, str, str]] = []
    for s in schedule.segments:
        out.append((int(op(t0 + int(s.start_s * NS))), "network", s.name))
    for t_s, d_ms in schedule.uplink.knots[1:]:
        out.append((int(op(t0 + int(t_s * NS))), "uplink", f"shift:{d_ms:.1f}ms"))
    for t, name, _ in session.uplink_changes:
        out.append((int(op(int(t))), "uplink", name))
    if session.latents is not None:
        for t, kind, detail in session.latents.events:
            if kind != "uplink":  # the UP records already say it
                out.append((int(op(t)), kind, detail))
    for t, text in session.marks:
        out.append((int(op(t)), "mark", text))
    out.sort(key=lambda e: e[0])
    return out


# -- the true network and what the headset observed, for the forecasters -----------------------------

def _clock(view: View):
    """The view's virtual headset clock and its inverse, from ``meta``."""
    tps = view.tps
    ratio = tps / NS
    uptime = int(view.meta["operatorUptimeNs"])
    drift = float(view.meta["operatorDriftPpm"]) * 1e-6
    t0 = int(view.meta["t0RobotNs"])

    def op(t_robot: float) -> int:
        return int(uptime * ratio + np.round((float(t_robot) - t0) * (1.0 + drift) * ratio))

    def robot(t_op: float) -> float:
        return t0 + (float(t_op) - uptime * ratio) / ((1.0 + drift) * ratio)

    return op, robot, t0


def view_truths(view: View) -> Tuple[LinkTruth, LinkTruth]:
    """(uplink, downlink): what each direction really did, by send time on the view's clock."""
    if "downlinkSchedule" not in view.meta:
        raise ValueError(f"{view.meta.get('session')}: a version-1 view has no network truth")
    op, robot, t0 = _clock(view)
    tps = view.tps
    down_schedule = Schedule.from_json(view.meta["downlinkSchedule"])
    realized = (view.frame_send, (view.frame_arrival - view.frame_send) / tps * 1e3, view.frame_delivered)
    down = LinkTruth(down_schedule, tps, to_sched_s=lambda t: (robot(t) - t0) / NS,
                     to_op=lambda x: op(t0 + x * NS), realized=realized)
    spec = view.meta["uplink"]
    up_schedule = Schedule.from_json(spec["schedule"])
    origin = int(spec["originRobotNs"])
    lan_base, lan_jitter = float(spec["lanBaseMs"]), float(spec["lanJitterMs"])
    # A passthrough segment is the LAN alone; every other segment rides on top of it.
    from ..net.profiles import NetworkProfile, ms_to_ticks

    segments = []
    for s in up_schedule.segments:
        if s.named and s.name in PASSTHROUGH_PROFILES:
            segments.append(Segment(s.start_s, s.name, False, profile=NetworkProfile(0, ms_to_ticks(lan_jitter, tps), 0.0, 0.0)))
        else:
            segments.append(s)
    up_schedule = Schedule(segments=tuple(segments))
    shift = Schedule.from_json(view.meta["downlinkSchedule"]).uplink

    def shift_ms(t_op: int) -> float:
        return float(shift.ms_at(np.array([(robot(t_op) - t0) / NS]))[0])

    def up_to_op(x: float) -> int:
        # The virtual send whose command reaches the robot at schedule second x: the shift moved it
        # earlier, and the shift changes slowly enough that one correction finds it.
        at = origin + x * NS
        return op(at - shift_ms(op(at)) * 1e6)

    # The robot side impairs by when a command reaches it, which is its virtual send plus the shift.
    up = LinkTruth(up_schedule, tps, to_sched_s=lambda t: (robot(t) + shift_ms(t) * 1e6 - origin) / NS,
                   to_op=up_to_op, offset_ms=lambda t: lan_base + shift_ms(t))
    return up, down


def view_observations(view: View) -> Observations:
    """What the headset measured: the frames' arrivals and estimated sends, and the round trips."""
    tps = view.tps
    delivered = np.nonzero(view.frame_delivered)[0]
    down_ms = np.full(len(view.sync_t), -1, dtype=np.int64)
    for i, j in enumerate(view.sync_frame):
        if j >= 0 and view.frame_send_est[j] >= 0:
            down_ms[i] = int(view.frame_arrival[j] - view.frame_send_est[j])
    return Observations(
        tps=tps,
        frame_arrival=view.frame_arrival[delivered],
        frame_send_est=view.frame_send_est[delivered],
        frame_index=delivered.astype(np.int64),
        rtt_t=view.sync_t,
        rtt_ticks=view.sync_rtt,
        rtt_down_ticks=down_ms,
        sync_t=view.sync_t,
        sync_uncertainty=view.sync_uncertainty,
        sync_min_rtt=view.sync_min_rtt,
        sync_synced=view.sync_synced,
    )


# -- views on disk (contract/VIEW_FORMAT.md) ---------------------------------------------------------

_ARRAYS_V1 = ("cmd_t", "cmd_eff", "cmd_speed", "cmd_count", "cmd_motor", "cmd_value", "st_t", "st_sensed", "st_count",
              "st_motor", "st_value", "st_flags", "tr_t", "tr_motor", "tr_value", "frame_send", "frame_delivered",
              "frame_arrival", "frame_segment", "sync_t", "sync_offset", "sync_uncertainty", "sync_accepted", "events_t")
_ARRAYS_V2 = ("cmd_lost", "cmd_reconstructed", "cmd_true_uplink", "frame_send_est", "sync_rtt", "sync_frame",
              "sync_min_rtt", "sync_synced")
_FORECAST_ARRAYS = ("t", "q_ms", "loss", "floor_ms", "outage", "valid_q", "valid_loss", "valid_outage")


def save_view(view: View, path: str) -> None:
    arrays = {name: getattr(view, name) for name in _ARRAYS_V1 + _ARRAYS_V2}
    meta = dict(view.meta)
    if view.forecasts is not None:
        meta["forecastLevel"] = view.forecasts.level
        for name in _FORECAST_ARRAYS:
            arrays[f"fc_{name}"] = getattr(view.forecasts, name)
    arrays["meta_json"] = np.frombuffer(json.dumps(meta, sort_keys=True).encode("utf-8"), dtype=np.uint8)
    arrays["events_kind"] = np.array(view.events_kind, dtype="U32")
    arrays["events_detail"] = np.array(view.events_detail, dtype="U64")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez(path, **arrays)


def load_view(path: str) -> View:
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(bytes(z["meta_json"]).decode("utf-8"))
        version = int(meta.get("version", 0))
        if meta.get("format") != "jetrover-view" or version not in (1, 2, 3):
            raise ValueError(f"{path}: not a version-1, -2 or -3 view")
        kw = {name: z[name] for name in _ARRAYS_V1}
        if version < 3 and "tr_motor" in kw:
            # Before v3 a gripper read went through the arm's angle formula: not a value anything predicts.
            keep = kw["tr_motor"] != GRIPPER_TRUTH_MOTOR
            for name in [n for n in kw if n.startswith("tr_")]:
                kw[name] = kw[name][keep]
        if version >= 2:
            kw.update({name: z[name] for name in _ARRAYS_V2})
        forecasts = None
        if "forecastLevel" in meta:
            forecasts = ForecastStream(level=str(meta.pop("forecastLevel")),
                                       **{name: z[f"fc_{name}"] for name in _FORECAST_ARRAYS})
        return View(meta=meta, events_kind=[str(x) for x in z["events_kind"]],
                    events_detail=[str(x) for x in z["events_detail"]], forecasts=forecasts, **kw)
