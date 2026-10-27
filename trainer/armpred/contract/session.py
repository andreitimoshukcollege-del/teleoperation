"""Loads a session directory (contract/SESSION_FORMAT.md) into typed numpy arrays.

Every array is in the file's own time order. Times stay in the writer's integer ticks (Jetson
``CLOCK_MONOTONIC`` ns for everything the robot writes); nothing here converts clocks, so what the
headset would have seen is decided in one place, ``armpred.data``.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .jlog import Jlog, JlogError, read_jlog, split_items

MAX_JOINTS = 13  # JointStateCodec.MaxJointsPerMessage / JointCommandCodec.MaxJointsPerMessage

OUTCOME_ACCEPTED, OUTCOME_STALE, OUTCOME_REFUSED = ord("A"), ord("S"), ord("R")


@dataclass
class Commands:
    """``JC``: joint commands as RobotHost received them."""

    t: np.ndarray  # int64, socket arrival
    t_deliver: np.ndarray  # int64, reached the plant
    seq: np.ndarray  # int64
    capture: np.ndarray  # int64, sender's clock
    outcome: np.ndarray  # uint8: A / S / R
    count: np.ndarray  # int16
    motor: np.ndarray  # int16 [n, MAX_JOINTS], 0 past count
    angle: np.ndarray  # float32 [n, MAX_JOINTS], rad (gripper: 0..1 fraction)
    speed: np.ndarray  # float32 [n, MAX_JOINTS], rad/s

    def __len__(self) -> int:
        return int(self.t.shape[0])


@dataclass
class OperatorCommands:
    """``OC`` (``operator.jlog``): every joint command the headset sent, on its own clock, whether or
    not it reached the robot."""

    t: np.ndarray  # int64, operator ticks
    seq: np.ndarray  # int64
    capture: np.ndarray  # int64, the capture stamp it carried (operator ticks)
    count: np.ndarray  # int16
    motor: np.ndarray  # int16 [n, MAX_JOINTS]
    angle: np.ndarray  # float32 [n, MAX_JOINTS]
    speed: np.ndarray  # float32 [n, MAX_JOINTS]
    ticks_per_second: int = 1_000_000_000

    def __len__(self) -> int:
        return int(self.t.shape[0])


@dataclass
class Frames:
    """``FR``: joint-state frames as RobotHost sent them."""

    t: np.ndarray
    seq: np.ndarray
    robot_send: np.ndarray
    sensed: np.ndarray
    echo_operator: np.ndarray
    echo_robot_recv: np.ndarray
    count: np.ndarray
    motor: np.ndarray  # int16 [n, MAX_JOINTS]
    flags: np.ndarray  # uint8 [n, MAX_JOINTS], bit 0 = sensed
    value: np.ndarray  # float32 [n, MAX_JOINTS]

    def __len__(self) -> int:
        return int(self.t.shape[0])


@dataclass
class Reads:
    """``RD``: every position read the driver made. ``pulse`` is -1 for a miss."""

    t_req: np.ndarray
    t_resp: np.ndarray
    motor: np.ndarray  # int16
    pulse: np.ndarray  # int32

    @property
    def ok(self) -> np.ndarray:
        return self.pulse >= 0

    @property
    def t_mid(self) -> np.ndarray:
        """The truth's time convention: the middle of the bus transaction (integer ns)."""
        return self.t_req + (self.t_resp - self.t_req) // 2

    def __len__(self) -> int:
        return int(self.t_req.shape[0])


@dataclass
class ServoRows:
    """A list record flattened to one row per servo: ``RO``, ``RI``, ``CI``, ``WR``, ``GP``, ``GD``."""

    t: np.ndarray
    motor: np.ndarray
    pulse: np.ndarray  # float64 (RO sends floats; the rest are integers)
    extra: np.ndarray  # pps (RO, CI), valid (RI), durationMs (WR), 0 otherwise

    def __len__(self) -> int:
        return int(self.t.shape[0])


@dataclass
class Session:
    path: str
    meta: dict
    ticks_per_second: int
    commands: Commands
    frames: Frames
    reads: Reads
    keepalives: np.ndarray  # int64 [n, 3]: t, opTicks, opTps
    relay_out: ServoRows
    relay_in: ServoRows
    gate_in: ServoRows
    gate_pending: ServoRows
    gate_dropped: ServoRows
    writes: ServoRows
    cycles: np.ndarray  # int64 [n, 2]
    battery: np.ndarray  # int64 [n, 2]: t, mV
    subscriber_changes: List[Tuple[int, str]] = field(default_factory=list)
    sender_changes: List[Tuple[int, str]] = field(default_factory=list)
    uplink_changes: List[Tuple[int, str, int]] = field(default_factory=list)
    rounds: List["CourseLabel"] = field(default_factory=list)
    marks: List[Tuple[int, str]] = field(default_factory=list)
    latents: Optional["Latents"] = None
    operator_commands: Optional[OperatorCommands] = None
    files: Dict[str, Jlog] = field(default_factory=dict)

    @property
    def id(self) -> str:
        return str(self.meta.get("id", os.path.basename(self.path.rstrip("/\\"))))

    @property
    def source(self) -> str:
        return str(self.meta.get("source", "robot"))


@dataclass
class Latents:
    params: Dict[str, str]
    joints: np.ndarray  # float64 [n, 6]: t, motor, speedScale, lagSeconds, loadNm, stalled
    events: List[Tuple[int, str, str]]


def _i64(rows: List[List[str]], index: int) -> np.ndarray:
    return np.array([int(r[index]) for r in rows], dtype=np.int64)


def _joint_list(rows: List[List[str]], index: int, parts: int) -> Tuple[np.ndarray, List[np.ndarray]]:
    """A ``motor:a:b;...`` field into (count, [motor, a, b] as [n, MAX_JOINTS] arrays)."""
    n = len(rows)
    count = np.zeros(n, dtype=np.int16)
    columns = [np.zeros((n, MAX_JOINTS), dtype=np.float64) for _ in range(parts)]
    for i, r in enumerate(rows):
        items = split_items(r[index])
        if len(items) > MAX_JOINTS:
            raise JlogError(f"record has {len(items)} joints, more than {MAX_JOINTS}")
        count[i] = len(items)
        for k, item in enumerate(items):
            if len(item) != parts:
                raise JlogError(f"joint item {item!r} does not have {parts} parts")
            for p in range(parts):
                columns[p][i, k] = float(item[p])
    return count, columns


def _servo_rows(rows: List[List[str]], index: int, parts: int, extra_part: Optional[int], extra_field: Optional[int] = None) -> ServoRows:
    """Flattens a list record (time at field 0, the list at ``index``) to one row per servo."""
    ts, motors, pulses, extras = [], [], [], []
    for r in rows:
        t = int(r[0])
        field_extra = float(r[extra_field]) if extra_field is not None else 0.0
        for item in split_items(r[index]):
            if len(item) != parts:
                raise JlogError(f"servo item {item!r} does not have {parts} parts")
            ts.append(t)
            motors.append(int(item[0]))
            if parts == 3 and extra_part == 1:  # RI: motor:valid:pulse
                pulses.append(float(item[2]))
                extras.append(float(item[1]))
            else:
                pulses.append(float(item[1]))
                extras.append(float(item[extra_part]) if extra_part is not None else field_extra)
    return ServoRows(
        t=np.array(ts, dtype=np.int64),
        motor=np.array(motors, dtype=np.int16),
        pulse=np.array(pulses, dtype=np.float64),
        extra=np.array(extras, dtype=np.float64),
    )


def _single_servo_rows(rows: List[List[str]]) -> ServoRows:
    """``GP``/``GD``: ``t|motor|pulse``."""
    return ServoRows(
        t=_i64(rows, 0),
        motor=np.array([int(r[1]) for r in rows], dtype=np.int16),
        pulse=np.array([float(r[2]) for r in rows], dtype=np.float64),
        extra=np.zeros(len(rows), dtype=np.float64),
    )


def parse_commands(rows: List[List[str]]) -> Commands:
    count, (motor, angle, speed) = _joint_list(rows, 5, 3)
    return Commands(
        t=_i64(rows, 0),
        t_deliver=_i64(rows, 1),
        seq=_i64(rows, 2),
        capture=_i64(rows, 3),
        outcome=np.array([ord(r[4]) for r in rows], dtype=np.uint8),
        count=count,
        motor=motor.astype(np.int16),
        angle=angle.astype(np.float32),
        speed=speed.astype(np.float32),
    )


def parse_operator_commands(rows: List[List[str]], ticks_per_second: int) -> OperatorCommands:
    count, (motor, angle, speed) = _joint_list(rows, 3, 3)
    return OperatorCommands(
        t=_i64(rows, 0),
        seq=_i64(rows, 1),
        capture=_i64(rows, 2),
        count=count,
        motor=motor.astype(np.int16),
        angle=angle.astype(np.float32),
        speed=speed.astype(np.float32),
        ticks_per_second=int(ticks_per_second),
    )


def parse_frames(rows: List[List[str]]) -> Frames:
    count, (motor, flags, value) = _joint_list(rows, 6, 3)
    return Frames(
        t=_i64(rows, 0),
        seq=_i64(rows, 1),
        robot_send=_i64(rows, 2),
        sensed=_i64(rows, 3),
        echo_operator=_i64(rows, 4),
        echo_robot_recv=_i64(rows, 5),
        count=count,
        motor=motor.astype(np.int16),
        flags=flags.astype(np.uint8),
        value=value.astype(np.float32),
    )


def parse_reads(rows: List[List[str]]) -> Reads:
    return Reads(
        t_req=_i64(rows, 0),
        t_resp=_i64(rows, 1),
        motor=np.array([int(r[2]) for r in rows], dtype=np.int16),
        pulse=np.array([int(r[3]) if r[3] != "" else -1 for r in rows], dtype=np.int32),
    )


def _pairs(rows: List[List[str]], parts: int) -> np.ndarray:
    if not rows:
        return np.zeros((0, parts), dtype=np.int64)
    return np.array([[int(x) for x in r[:parts]] for r in rows], dtype=np.int64)


@dataclass(frozen=True)
class CourseLabel:
    """One ``TK`` record: the headset's training course changed label (docs/adr/0024). A round runs
    from its ``active`` record to its ``done``; ``suspended`` stretches inside it are not driving."""

    t: int
    course: int
    round: int
    task: str
    phase: str
    purpose: str
    level: Optional[float]  # rad/s; None for a varied round
    score: float
    disturbed: bool


def parse_course_label(r: List[str]) -> CourseLabel:
    if len(r) < 9:
        raise JlogError(f"TK record with {len(r)} fields, want 9")
    return CourseLabel(
        t=int(r[0]), course=int(r[1]), round=int(r[2]), task=r[3], phase=r[4], purpose=r[5],
        level=None if r[6] == "varied" else float(r[6]), score=float(r[7]), disturbed=r[8] == "1",
    )


def load_session(directory: str) -> Session:
    """Reads and validates every file of a session. Raises ``JlogError`` on anything malformed."""
    meta_path = os.path.join(directory, "session.json")
    if not os.path.exists(meta_path):
        raise JlogError(f"{directory}: no session.json")
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
    if meta.get("format") != "jetrover-session" or int(meta.get("version", 0)) != 1:
        raise JlogError(f"{meta_path}: not a version-1 jetrover session")

    host = read_jlog(os.path.join(directory, "robothost.jlog"), "robothost")
    driver = read_jlog(os.path.join(directory, "driver.jlog"), "driver")
    if host.header.ticks_per_second != driver.header.ticks_per_second:
        raise JlogError(f"{directory}: robothost and driver tick rates differ")
    if host.header.clock != driver.header.clock:
        raise JlogError(f"{directory}: robothost and driver clocks differ ({host.header.clock} vs {driver.header.clock})")
    files: Dict[str, Jlog] = {"robothost": host, "driver": driver}

    marks: List[Tuple[int, str]] = []
    marks_path = os.path.join(directory, "marks.jlog")
    if os.path.exists(marks_path):
        marks_log = read_jlog(marks_path, "marks")
        files["marks"] = marks_log
        marks = [(int(r[0]), r[1]) for r in marks_log.tag("MK")]

    operator_commands = None
    operator_path = os.path.join(directory, "operator.jlog")
    if os.path.exists(operator_path):
        operator_log = read_jlog(operator_path, "operator")
        files["operator"] = operator_log
        operator_commands = parse_operator_commands(operator_log.tag("OC"), operator_log.header.ticks_per_second)

    latents = None
    latents_path = os.path.join(directory, "latents.jlog")
    if os.path.exists(latents_path):
        latents_log = read_jlog(latents_path, "latents")
        files["latents"] = latents_log
        latents = Latents(
            params={r[0]: r[1] for r in latents_log.tag("LP")},
            joints=np.array([[float(x) for x in r[:6]] for r in latents_log.tag("LJ")], dtype=np.float64).reshape(-1, 6),
            events=[(int(r[0]), r[1], r[2] if len(r) > 2 else "") for r in latents_log.tag("EV")],
        )

    return Session(
        path=directory,
        meta=meta,
        ticks_per_second=host.header.ticks_per_second,
        commands=parse_commands(host.tag("JC")),
        frames=parse_frames(host.tag("FR")),
        reads=parse_reads(driver.tag("RD")),
        keepalives=_pairs(host.tag("KA"), 3),
        relay_out=_servo_rows(host.tag("RO"), 1, 3, 2),
        relay_in=_servo_rows(host.tag("RI"), 1, 3, 1),
        gate_in=_servo_rows(driver.tag("CI"), 1, 3, 2),
        gate_pending=_single_servo_rows(driver.tag("GP")),
        gate_dropped=_single_servo_rows(driver.tag("GD")),
        writes=_servo_rows(driver.tag("WR"), 2, 2, None, extra_field=1),
        cycles=_pairs(driver.tag("CY"), 2),
        battery=_pairs(driver.tag("BV"), 2),
        subscriber_changes=[(int(r[0]), r[1]) for r in host.tag("SS")],
        sender_changes=[(int(r[0]), r[1]) for r in host.tag("JS")],
        uplink_changes=[(int(r[0]), r[1], int(r[2]) if r[2] != "" else 0) for r in host.tag("UP")],
        rounds=[parse_course_label(r) for r in host.tag("TK")],
        marks=marks,
        latents=latents,
        operator_commands=operator_commands,
        files=files,
    )
