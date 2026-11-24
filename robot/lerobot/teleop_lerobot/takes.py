"""A headset take, read into the frames of one LeRobot episode (docs/adr/0036 decisions 2-4). No LeRobot here: numpy is
not needed either, so this runs and is tested anywhere.

The take is a session (trainer/contract/SESSION_FORMAT.md):

- its joints come from ``robothost.jlog``: ``RO``, ``RI`` and ``JC`` for the JetRover (version 1), and ``QS``, ``AO``
  and ``JC`` for the FR3 Duo (version 2);
- the action is the newest command sent, in the units the plant sends its arm: the newest ``RO`` (JetRover, relay pulses)
  or ``AO`` (FR3 Duo), the target the arm is heading for. A command RobotHost clamped counts (its ``RO`` is the clamped
  point), one it refused does not (docs/adr/0038 L, "Sent commands");
- its frames come from ``isaac.jlog`` (Isaac's renderer) or ``camera.jlog`` (the real robot's CameraHost), with ``VF``
  records pointing into TFRM sidecars.

Every one of those is stamped on the same CLOCK_MONOTONIC. A frame's ``stateSensed`` is the time it shows: the physics
state Isaac posed, or the start of the camera's exposure. Its joints are read at that time.
"""

from __future__ import annotations

import bisect
import hashlib
import json
import math
import os
import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .jlog import JlogError, read_jlog, items

# The lab's schema (SINRG-Lab/world-models data_collection): the JetRover's six servos as raw pulses, the gripper last.
JETROVER_MOTORS = (1, 2, 3, 4, 5, 10)
JETROVER_SENSED = (1, 2, 3, 4, 5)  # read back (RI); the gripper's slot is its last command, as the lab's recorder has it
JETROVER_NAMES = ["servo_" + str(m) for m in JETROVER_MOTORS]
FR3_NAMES = ([f"left_joint{i}" for i in range(1, 8)] + ["left_gripper"]
             + [f"right_joint{i}" for i in range(1, 8)] + ["right_gripper"])

# How RobotHost turns a JetRover joint command into relay pulses (GenericArmPlantConfig.Default, GenericArmPlant.ApplyJointTargets):
# an arm joint's angle plus its profile's zeroOffsetRadians at 1000 pulses per 240 degrees from 500, within its range; the
# gripper's 0..1 fraction from its open to its closed pulse. commands.parquet gives a command RobotHost did not pass on as given
# (refused or clamped) in pulses with it, once the take's own accepted commands have shown it is RobotHost's (check_conversion,
# within PULSE_TOLERANCE: the plant computes in float32).
PULSE_PER_RADIAN = 1000.0 / math.radians(240.0)
ZERO_PULSE = 500.0
PULSE_RANGE = (0.0, 1000.0)
GRIPPER_OPEN_PULSE, GRIPPER_CLOSED_PULSE = 116.0, 645.0
PULSE_TOLERANCE = 0.01

SOURCES = {"robot": "real", "isaac": "isaac"}  # session.json source -> the dataset folder's; nothing else has frames
FRAMES = {"real": ("camera", "camera"), "isaac": ("isaac", "isaac")}  # source -> (frames log writer, sidecar folder)


class TakeError(ValueError):
    """A take that cannot become an episode, and why."""


@dataclass
class Camera:
    name: str
    width: int
    height: int
    sidecar: str  # path of its rgb sidecar


@dataclass
class Frame:
    """One episode frame: the recorded frame it shows and the joints at that frame's time."""

    parts: Dict[str, Tuple[int, int]]  # camera -> (offset, length) of its JPEG in the camera's sidecar
    state: List[float]
    action: List[float]
    timing: Dict[str, object]


@dataclass
class Take:
    path: str
    session_id: str
    robot: str      # "jetrover" | "fr3_duo"
    source: str     # "real" | "isaac"
    scenario: str
    task: str
    names: List[str]
    fps: int
    cameras: List[Camera]
    frames: List[Frame] = field(default_factory=list)

    @property
    def dataset_group(self) -> str:
        return f"{self.robot}-{self.source}"

    def jpeg(self, frame: Frame, camera: Camera) -> bytes:
        offset, length = frame.parts[camera.name]
        return read_payload(camera.sidecar, offset, length)


def read_payload(path: str, offset: int, length: int) -> bytes:
    """One record of a TFRM sidecar, by the payload offset a ``VF`` names. Its length prefix must agree."""
    with open(path, "rb") as f:
        if offset < 8 + 4:
            raise TakeError(f"{path}: offset {offset} is inside the header")
        f.seek(offset - 4)
        (prefix,) = struct.unpack("<I", f.read(4))
        if prefix != length:
            raise TakeError(f"{path}: the record at {offset} is {prefix} bytes, VF says {length}")
        data = f.read(length)
    if len(data) != length:
        raise TakeError(f"{path}: the record at {offset} is cut off")
    return data


def verify_checksums(take: str) -> None:
    """``sha256sum -c files.sha256`` in Python: every listed file, inside the take, as listed."""
    listing = os.path.join(take, "files.sha256")
    if not os.path.exists(listing):
        raise TakeError(f"{take}: not complete (no files.sha256)")
    root = os.path.realpath(take)
    with open(listing, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if not line:
                continue
            digest, name = line[:64], line[66:] if line[64:66] in ("  ", " *") else None
            if name is None:
                raise TakeError(f"{listing}: malformed line {line!r}")
            path = os.path.realpath(os.path.join(take, name))
            if not path.startswith(root + os.sep):
                raise TakeError(f"{listing}: {name!r} is outside the take")
            h = hashlib.sha256()
            with open(path, "rb") as g:
                for block in iter(lambda: g.read(1 << 20), b""):
                    h.update(block)
            if h.hexdigest() != digest:
                raise TakeError(f"{take}: {name} does not match files.sha256")


def checksum_id(take: str) -> str:
    """The sha256 of the take's files.sha256: names the take's content in a dataset's takes.jsonl."""
    with open(os.path.join(take, "files.sha256"), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


class _Track:
    """One motor's values over time, newest-at-or-before lookups."""

    def __init__(self):
        self.t: List[int] = []
        self.v: List[float] = []

    def add(self, t: int, v: float) -> None:
        self.t.append(t)
        self.v.append(v)

    def finish(self) -> None:
        order = sorted(range(len(self.t)), key=lambda i: self.t[i])  # stable: same-time values keep their order
        self.t = [self.t[i] for i in order]
        self.v = [self.v[i] for i in order]

    def at(self, t: int) -> Optional[Tuple[int, float]]:
        i = bisect.bisect_right(self.t, t) - 1
        return (self.t[i], self.v[i]) if i >= 0 else None


def _tracks(rows, motor_part: int, value_part: int, valid_part: Optional[int] = None, items_field: int = 1,
            time_field: int = 0, scale: Optional[Dict[int, float]] = None) -> Dict[int, _Track]:
    out: Dict[int, _Track] = {}
    for r in rows:
        t = int(r[time_field])
        for it in items(r[items_field]):
            if valid_part is not None and it[valid_part] != "1":
                continue
            m = int(it[motor_part])
            v = float(it[value_part]) * (scale.get(m, 1.0) if scale else 1.0)
            out.setdefault(m, _Track()).add(t, v)
    for tr in out.values():
        tr.finish()
    return out


def _event_tracks(events: List[Tuple[int, Dict[int, float]]]) -> Dict[int, _Track]:
    out: Dict[int, _Track] = {}
    for t, values in events:
        for m, v in values.items():
            out.setdefault(m, _Track()).add(t, v)
    for tr in out.values():
        tr.finish()
    return out


def _jetrover_pulses(path: str):
    """The plant's own turning of a ``JC``'s targets into relay pulses, for this take's robot profile (``robot-profile.json``,
    the one RobotHost converted with). A motor the profile lacks, a motor named a second time and a value that is not finite
    are left out, as the plant leaves them out."""
    try:
        with open(os.path.join(path, "robot-profile.json"), encoding="utf-8") as f:
            profile = json.load(f)
        joints = {}
        for joint in profile["joints"]:
            j = {str(k).lower(): v for k, v in joint.items()}  # RobotHost reads the profile's names in any case
            lo, hi = PULSE_RANGE
            if j.get("minangleradians") is not None:
                lo = ZERO_PULSE + float(j["minangleradians"]) * PULSE_PER_RADIAN
            if j.get("maxangleradians") is not None:
                hi = ZERO_PULSE + float(j["maxangleradians"]) * PULSE_PER_RADIAN
            joints[int(j["motorid"])] = (str(j["role"]).lower() == "grippermain", float(j.get("zerooffsetradians") or 0.0), lo, hi)
    except (OSError, ValueError, KeyError, TypeError) as e:
        raise TakeError(f"{path}: a command RobotHost did not pass on as given needs robot-profile.json to be put in pulses ({e})")

    def pulses(targets: str) -> Dict[int, float]:
        out: Dict[int, float] = {}
        for it in items(targets):
            m, angle = int(it[0]), float(it[1])
            if m not in joints or m in out or not math.isfinite(angle):
                continue
            gripper, offset, lo, hi = joints[m]
            if gripper:
                out[m] = GRIPPER_OPEN_PULSE + min(1.0, max(0.0, angle)) * (GRIPPER_CLOSED_PULSE - GRIPPER_OPEN_PULSE)
            else:
                out[m] = min(hi, max(lo, ZERO_PULSE + (angle + offset) * PULSE_PER_RADIAN))
        return out

    return pulses


def check_conversion(path: str, host, convert, out_tag: str = "RO", tolerance: float = PULSE_TOLERANCE) -> int:
    """Checks ``convert`` (:func:`_jetrover_pulses`) against the take's own accepted commands before a command RobotHost did
    not pass on as given (refused, or clamped) is turned into relay units with it: each accepted command's own record (the
    first ``out_tag`` written once it reached the plant, before the next command did) must be it, within ``tolerance``.
    Returns how many were checked; raises ``TakeError`` when one differs or none has a record of its own."""
    sent = _out_events(host, out_tag)
    times = [t for t, _ in sent]
    commands = host.tag("JC")
    arrivals = sorted(int(r[1]) for r in commands)
    checked = 0
    for r in commands:
        if r[4] != "A":
            continue
        arrived = int(r[1])
        i = bisect.bisect_left(times, arrived)
        later = bisect.bisect_right(arrivals, arrived)
        if i == len(times) or (later < len(arrivals) and times[i] >= arrivals[later]):
            continue  # no record of its own before the next command reached the plant
        for m, v in convert(r[5]).items():
            got = sent[i][1].get(m)
            if got is not None and abs(got - v) > tolerance:
                raise TakeError(f"{path}: RobotHost sent motor {m} {got} ({out_tag}) for the command that reached it at {arrived}, "
                                f"not {v:.6g} as takes.py converts it: its conversion has changed, so the commands it did not "
                                f"pass on as given cannot be put in relay units")
        checked += 1
    if not checked:
        raise TakeError(f"{path}: no accepted command has an {out_tag} of its own to check the conversion against")
    return checked


def _out_events(host, out_tag: str) -> List[Tuple[int, Dict[int, float]]]:
    """Every ``out_tag`` record as ``(t, {motor: value})`` in time order."""
    return sorted(((int(r[0]), {int(it[0]): float(it[1]) for it in items(r[1])}) for r in host.tag(out_tag)), key=lambda e: e[0])


def _jetrover_actions(path: str, host) -> List[Tuple[int, Dict[int, float]]]:
    """The JetRover's action events: every ``RO``, the targets RobotHost sent the relay, at its time. That is the command it
    passed on (``JC`` ``A``), the farthest passing point toward one it clamped (``C``, since RobotHost's clamping host check,
    ``HK``), and its own park targets; a command it refused (``R``: nothing passed, the arm holds) or ignored while parking
    (``P``) leaves the previous one in force. So the action is the target the arm is heading for (docs/adr/0038 L, "Sent
    commands"; for a few hours on 2026-11-24, 735ab2f and 1dff8ed, a refused command was the action too)."""
    return _out_events(host, "RO")


def _fr3_actions(path: str, host, profile: dict) -> List[Tuple[int, Dict[int, float]]]:
    """The FR3 Duo's action events, as the plant sends its arm backend (radians, a hand's 0..1 fraction): every ``AO``. A
    refused command leaves the previous one in force, as on the JetRover."""
    return _out_events(host, "AO")


def _fr3_order(profile: dict) -> Tuple[List[int], Dict[int, float]]:
    """The FR3 Duo's motors in the lab's order (each arm's seven joints, then its gripper) and each gripper's travel."""
    motors, travel = [], {}
    for arm in profile["arms"]:
        motors += [j["motorId"] for j in arm["joints"]]
        g = arm.get("gripper")
        if not arm.get("hasGripper") or not g:
            raise TakeError("an FR3 Duo arm without a gripper is not in the lab's schema")
        motors.append(g["motorId"])
        travel[g["motorId"]] = float(g["fingerTravel"])
    if len(motors) != 16:
        raise TakeError(f"the FR3 Duo profile has {len(motors)} motors, the lab's schema 16")
    return motors, travel


def read_take(path: str, fps: int = 15) -> Take:
    """Reads a complete take into its episode frames. Raises ``TakeError`` (or ``JlogError``) when it cannot be one."""
    with open(os.path.join(path, "session.json"), encoding="utf-8") as f:
        meta = json.load(f)
    scenario = meta.get("scenario") or {}
    if not scenario.get("name"):
        raise TakeError(f"{path}: not a take (session.json has no scenario)")
    source = SOURCES.get(str(meta.get("source", "robot")))
    if source is None:
        raise TakeError(f"{path}: source {meta.get('source')!r} records no frames")
    kind = (meta.get("format"), int(meta.get("version", 0)))
    host = read_jlog(os.path.join(path, "robothost.jlog"), "robothost")
    tps = host.ticks_per_second

    if kind == ("jetrover-session", 1):
        robot, names, motors = "jetrover", JETROVER_NAMES, list(JETROVER_MOTORS)
        relay = _tracks(host.tag("RO"), 0, 1)
        sent = relay  # the action: the newest relay target (_jetrover_actions)
        read = _tracks(host.tag("RI"), 0, 2, valid_part=1)

        def joints(t: int):
            state, ages = [], []
            for m in JETROVER_SENSED:
                got = read.get(m) and read[m].at(t)
                if not got:
                    return None
                ages.append(t - got[0])
                state.append(got[1])
            # The gripper's slot: its newest relay target, the gripper as RobotHost last told it (the lab's recorder has its
            # last command), as the action has it.
            gripper = relay.get(motors[-1]) and relay[motors[-1]].at(t)
            if not gripper:
                return None
            action, sent_at = [], []
            for m in motors:
                got = sent.get(m) and sent[m].at(t)
                if not got:
                    return None
                sent_at.append(t - got[0])
                action.append(got[1])
            return state + [gripper[1]], action, max(ages), max(sent_at)
    elif kind == ("teleop-session", 2):
        with open(os.path.join(path, "robot-profile.json"), encoding="utf-8") as f:
            profile = json.load(f)
        if profile.get("name") != "fr3-duo":
            raise TakeError(f"{path}: robot {profile.get('name')!r} has no schema here")
        robot, names = "fr3_duo", FR3_NAMES
        motors, travel = _fr3_order(profile)
        measured: Dict[int, Dict[int, float]] = {}
        for r in host.tag("QS"):
            measured[int(r[1])] = {int(it[0]): float(it[1]) * travel.get(int(it[0]), 1.0) for it in items(r[2])}
        sent = _event_tracks([(t, {m: v * travel.get(m, 1.0) for m, v in values.items()})
                              for t, values in _fr3_actions(path, host, profile)])

        def joints(t: int):
            m_now = measured.get(t)
            if m_now is None or any(m not in m_now for m in motors):
                return None
            action, sent_at = [], []
            for m in motors:
                got = sent.get(m) and sent[m].at(t)
                if not got:
                    return None
                sent_at.append(t - got[0])
                action.append(got[1])
            return [m_now[m] for m in motors], action, 0, max(sent_at)
    else:
        raise TakeError(f"{path}: session {kind} has no schema here")

    accepted = [int(r[1]) for r in host.tag("JC") if r[4] in ("A", "C")]
    if not accepted:
        raise TakeError(f"{path}: no operator command was accepted")
    # The episode spans the operator's commands RobotHost judged (A accepted, C clamped, R refused; since 2026-11-24, until
    # then the accepted ones): a take whose last commands RobotHost refused keeps its last frames, their action the last one
    # sent, and a replay spans what its source does whatever RobotHost refused in either.
    driven = [int(r[1]) for r in host.tag("JC") if r[4] in ("A", "C", "R")]
    first, last = min(driven), max(driven)

    writer, folder = FRAMES[source]
    frames_log = read_jlog(os.path.join(path, writer + ".jlog"), writer)
    if frames_log.ticks_per_second != tps or frames_log.clock != host.clock:
        raise TakeError(f"{path}: {writer}.jlog is on another clock than robothost.jlog")
    cameras: List[Camera] = []
    for r in frames_log.tag("CM"):
        streams = r[17].split(";") if len(r) > 17 else ["rgb"]  # CM: ... mount (16), streams (17), poseFrame
        if "rgb" in streams:
            cameras.append(Camera(r[0], int(r[1]), int(r[2]), os.path.join(path, folder, r[0] + ".rgb.frames")))
    if not cameras:
        raise TakeError(f"{path}: {writer}.jlog has no camera recording rgb")

    candidates = []  # (t, frame, parts, state, action, stateAge, actionAge)
    seen = set()
    for r in frames_log.tag("VF"):
        t, flags = int(r[2]), r[3]
        if "B" in flags or t < first or t > last or t in seen:
            continue
        parts = {}
        for stream, offset, length in (p.rsplit(":", 2) for p in r[4].split(";") if p):
            cam, _, kind_ = stream.rpartition(".")
            if kind_ == "rgb":
                parts[cam] = (int(offset), int(length))
        if any(c.name not in parts for c in cameras):
            continue
        j = joints(t)
        if j is None:
            continue
        seen.add(t)
        candidates.append((t, int(r[1]), parts) + tuple(j))
    if not candidates:
        raise TakeError(f"{path}: no frame between the first and last operator command has its joints")
    candidates.sort(key=lambda c: c[0])

    take = Take(path, str(meta.get("id", os.path.basename(path))), robot, source, scenario["name"],
                scenario.get("description") or scenario["name"], names, fps, cameras)
    # Each grid point takes the recorded frame nearest it, if one is within half a step; else the last frame again,
    # flagged as filled. (Until 2026-11-15 it took the newest frame at most 1 ms past the point: the twin's frames wobble
    # by ~3 ms, so at 30 fps written as 15 that picked frames 1, 2 or 3 apart, a 5 Hz judder in 80 of 85 episodes.)
    step, t0 = tps / fps, candidates[0][0]
    half = step / 2
    i, k, previous = 0, 0, None
    while True:
        grid = t0 + k * step
        if grid > candidates[-1][0] + half:
            break
        while i + 1 < len(candidates) and abs(candidates[i + 1][0] - grid) < abs(candidates[i][0] - grid):  # a tie: the earlier
            i += 1
        chosen = i if (abs(candidates[i][0] - grid) <= half or previous is None) else previous
        t, number, parts, state, action, state_age, action_age = candidates[chosen]
        take.frames.append(Frame(parts, [float(x) for x in state], [float(x) for x in action], {
            "grid": int(round(grid)), "sensed": t, "frame": number, "filled": chosen == previous,
            "stateAge": int(state_age), "actionAge": int(action_age)}))
        previous = chosen
        k += 1
    return take


def raw_events(take: "Take"):
    """The take's commands and readings as the lab recorder's ``raw_ros_events.jsonl`` lines (SINRG-Lab/world-models,
    ``jetrover_record_bridge.py``): a ``ready``, then ``action`` and ``state`` events in time order. ``action`` carries
    every motor in the dataset's order, the commands the action column is made of (every ``RO`` or ``AO``: the commands sent,
    a clamped one's clamped point included, a refused one not); ``state`` the sensed ones (for the JetRover, motors 1 to 5, and only
    when all of them read, as the lab's bridge emits them). Its times are the session's one clock (CLOCK_MONOTONIC):
    ``jetson_mono_ns`` and ``pc_received_mono_ns`` are both the time RobotHost logged it. ``jetson_wall_ns`` is
    null: a take keeps no wall clock per record."""
    host = read_jlog(os.path.join(take.path, "robothost.jlog"), "robothost")
    events = []
    if take.robot == "jetrover":
        motors, sensed = list(JETROVER_MOTORS), list(JETROVER_SENSED)
        for t, got in _jetrover_actions(take.path, host):  # the action column's: every RO
            if all(m in got for m in motors):
                events.append((t, "action", motors, [got[m] for m in motors]))
        for r in host.tag("RI"):
            got = {int(it[0]): float(it[2]) for it in items(r[1]) if it[1] == "1"}
            if all(m in got for m in sensed):
                events.append((int(r[0]), "state", sensed, [got[m] for m in sensed]))
    else:
        with open(os.path.join(take.path, "robot-profile.json"), encoding="utf-8") as f:
            profile = json.load(f)
        motors, travel = _fr3_order(profile)
        for t, values in _fr3_actions(take.path, host, profile):  # the action column's: every AO
            got = {m: v * travel.get(m, 1.0) for m, v in values.items()}
            ids = [m for m in motors if m in got]
            events.append((t, "action", ids, [got[m] for m in ids]))
        for r in host.tag("QS"):
            got = {int(it[0]): float(it[1]) * travel.get(int(it[0]), 1.0) for it in items(r[2])}
            if all(m in got for m in motors):
                events.append((int(r[1]), "state", motors, [got[m] for m in motors]))
    events.sort(key=lambda e: e[0])
    first = events[0][0] if events else 0
    out = [{"type": "ready", "jetson_wall_ns": None, "jetson_mono_ns": first, "servo_ids": list(motors), "pc_received_mono_ns": first}]
    for t, kind, ids, values in events:
        out.append({"type": kind, "jetson_wall_ns": None, "jetson_mono_ns": t, "servo_ids": list(ids), "values": values,
                    "pc_received_mono_ns": t})
    return out


__all__ = ["Camera", "Frame", "JlogError", "Take", "TakeError", "check_conversion", "checksum_id", "read_payload", "read_take",
           "verify_checksums", "raw_events"]
