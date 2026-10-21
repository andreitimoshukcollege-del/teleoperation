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
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..contract.profile import RobotProfile
from ..contract.session import MAX_JOINTS, Session
from ..net.clocksync import ClockSync, headset_config
from .clockmap import ClockMap, fit_sender_clock
from .schedule import Schedule, build_downlink

NS = 1_000_000_000
HEADSET_TICK_NS = NS / 90.0


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

    @property
    def tps(self) -> int:
        return int(self.meta["operatorTicksPerSecond"])


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

    # -- commands: when the virtual headset sent each one (the uplink shift moves sends earlier) ----
    c = session.commands
    robot_send = cmap.sender_to_robot(c.capture.astype(np.float64) * (NS / sender_tps))
    shift_ns = schedule.uplink.ms_at(rel_s(robot_send)) * 1e6
    virtual_send = op(robot_send - shift_ns)
    order = np.argsort(virtual_send, kind="stable")
    cmd_send = virtual_send[order]
    # JetRoverJointStateLink.OnJointCommandSent: the largest speed among the targets, from 0.
    cmd_speed = np.array([max([float(c.speed[i, k]) for k in range(int(c.count[i]))] + [0.0]) for i in order],
                         dtype=np.float32)

    # -- frames: each through the downlink of the segment it was sent in --------------------------
    f = session.frames
    frame_send = op(f.robot_send)
    delivered, arrival, segment = build_downlink(schedule, frame_send, rel_s(f.robot_send), seed, operator_tps, traces_dir)
    # The echoed keepalive stamps were the original headset's; move them to the virtual one.
    echo_robot = cmap.sender_to_robot(f.echo_operator.astype(np.float64) * (NS / sender_tps))
    echo_virtual = op(echo_robot - schedule.uplink.ms_at(rel_s(echo_robot)) * 1e6)
    deliver_order = np.nonzero(delivered)[0]
    deliver_order = deliver_order[np.lexsort((deliver_order, arrival[deliver_order]))]

    # -- the headset: display ticks, ClockSync, the hold-back, the predictor calls ----------------
    sync = ClockSync(headset_config(operator_tps))
    tick_jitter = rng.normal(0.0, 0.4e6 * ratio, size=int((end - t0) / NS * 90) + 200)
    added = int(schedule.added_delay_ms / 1000.0 * operator_tps)
    st_rows: List[Tuple[int, int, int]] = []  # (observe tick, sensed ticks, frame index)
    cmd_eff = np.zeros(len(cmd_send), dtype=np.int64)
    sync_rows: List[Tuple[int, int, int, bool]] = []
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
                accepted = sync.add_round_trip(echo, operator_tps, int(f.echo_robot_recv[j]), int(f.robot_send[j]),
                                               session.ticks_per_second, int(arrival[j]))
                sync_rows.append((int(arrival[j]), sync.offset_ticks, sync.uncertainty_ticks, accepted))
            robot_time = int(f.sensed[j]) if f.sensed[j] != 0 else int(f.robot_send[j])
            sensed_op = sync.to_operator_ticks(robot_time, session.ticks_per_second, operator_tps) if sync.is_synced else -1
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
    tr_value = np.zeros(int(ok.sum()), dtype=np.float64)
    pulses = session.reads.pulse[ok]
    for m in np.unique(tr_motor):
        sel = tr_motor == m
        tr_value[sel] = profile.pulse_to_angle(int(m), pulses[sel])
    events = _events(session, schedule, t0, op)
    meta = {
        "format": "jetrover-view",
        "version": 1,
        "session": session.id,
        "source": session.source,
        "schedule": schedule.describe(),
        "namedOnly": schedule.only_named,
        "seed": int(seed),
        "operatorTicksPerSecond": int(operator_tps),
        "robotTicksPerSecond": int(session.ticks_per_second),
        "operatorUptimeNs": uptime,
        "operatorDriftPpm": drift * 1e6,
        "conditions": session.meta.get("conditions", {}),
    }
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
        st_motor=f.motor[st_idx].astype(np.int16) if len(st_idx) else np.zeros((0, MAX_JOINTS), np.int16),
        st_value=f.value[st_idx].astype(np.float32) if len(st_idx) else np.zeros((0, MAX_JOINTS), np.float32),
        st_flags=f.flags[st_idx].astype(np.uint8) if len(st_idx) else np.zeros((0, MAX_JOINTS), np.uint8),
        tr_t=op(session.reads.t_mid[ok]),
        tr_motor=tr_motor.astype(np.int16),
        tr_value=tr_value,
        frame_send=frame_send,
        frame_delivered=delivered,
        frame_arrival=arrival,
        frame_segment=segment.astype(np.int16),
        sync_t=np.array([r[0] for r in sync_rows], dtype=np.int64),
        sync_offset=np.array([r[1] for r in sync_rows], dtype=np.int64),
        sync_uncertainty=np.array([r[2] for r in sync_rows], dtype=np.int64),
        sync_accepted=np.array([r[3] for r in sync_rows], dtype=bool),
        events_t=np.array([e[0] for e in events], dtype=np.int64),
        events_kind=[e[1] for e in events],
        events_detail=[e[2] for e in events],
    )


def _events(session: Session, schedule: Schedule, t0: int, op) -> List[Tuple[int, str, str]]:
    """Everything an event-windowed metric may align on, on the virtual headset's clock: network
    switches (this view's), uplink knots, and the session's own events (simulated latents, marks)."""
    out: List[Tuple[int, str, str]] = []
    for s in schedule.segments:
        out.append((int(op(t0 + int(s.start_s * NS))), "network", s.name))
    for t_s, d_ms in schedule.uplink.knots[1:]:
        out.append((int(op(t0 + int(t_s * NS))), "uplink", f"{d_ms:.1f}ms"))
    if session.latents is not None:
        for t, kind, detail in session.latents.events:
            out.append((int(op(t)), kind, detail))
    for t, text in session.marks:
        out.append((int(op(t)), "mark", text))
    out.sort(key=lambda e: e[0])
    return out


# -- views v1 on disk (contract/VIEW_FORMAT.md) ---------------------------------------------------

_ARRAYS = ("cmd_t", "cmd_eff", "cmd_speed", "cmd_count", "cmd_motor", "cmd_value", "st_t", "st_sensed", "st_count",
           "st_motor", "st_value", "st_flags", "tr_t", "tr_motor", "tr_value", "frame_send", "frame_delivered",
           "frame_arrival", "frame_segment", "sync_t", "sync_offset", "sync_uncertainty", "sync_accepted", "events_t")


def save_view(view: View, path: str) -> None:
    import json

    arrays = {name: getattr(view, name) for name in _ARRAYS}
    arrays["meta_json"] = np.frombuffer(json.dumps(view.meta, sort_keys=True).encode("utf-8"), dtype=np.uint8)
    arrays["events_kind"] = np.array(view.events_kind, dtype="U32")
    arrays["events_detail"] = np.array(view.events_detail, dtype="U64")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez(path, **arrays)


def load_view(path: str) -> View:
    import json

    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(bytes(z["meta_json"]).decode("utf-8"))
        if meta.get("format") != "jetrover-view" or int(meta.get("version", 0)) != 1:
            raise ValueError(f"{path}: not a version-1 view")
        kw = {name: z[name] for name in _ARRAYS}
        return View(meta=meta, events_kind=[str(x) for x in z["events_kind"]],
                    events_detail=[str(x) for x in z["events_detail"]], **kw)
