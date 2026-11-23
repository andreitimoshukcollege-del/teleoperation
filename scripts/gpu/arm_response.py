#!/usr/bin/env python3
"""How the JetRover's arm follows its commands, real and twin (docs/adr/0038 workstream R, items 26, 29, 32 and 17).

Reads recorded sessions and takes (``robothost.jlog``: ``JC``, ``RO``, ``RI``, ``TQ``) and the safety datasets (LeRobot v3,
``observation.state`` and ``action`` in raw servo units), and reports, per group of inputs:

- ``tracking``: the world-models measure, on 15 fps frames: the share of frames with ``max|state - action|`` over motors 1-5
  above 20 units (4.8 degrees), each joint's error, how long the stretches above it last, and the tool point's error (the
  "first row" of world-models' ADR 0002: from the measured arm to the command, 0.15 m along ``gripper_link``'s x);
- ``motion``: the commanded motion per 15 fps frame, per joint in servo units and at the tool point in cm (also per axis in
  DROID's end-effector frame, world-models' ``ee_motion``), and the measured motion beside it;
- ``bins``: each joint's error against how fast its command moves: the same arm should show the same error at the same
  speed, whoever drives it;
- ``lag``: the delay that best lines each joint's reading (``RI``) up with its command (``RO``), on a 10 ms grid;
- ``kinematics``: the commanded joint speed and acceleration at the command rate (``JC``, smoothed as
  ``~/analysis/motion/metrics.py`` smooths), and the tool point's speed: what speed caps would have to match;
- ``steps``: the response to a command step (``RO`` jumps and holds): delay, rise time, peak speed, overshoot;
- ``stops``: what the arm does after its command stops moving mid-move and holds for a while: how long it keeps moving,
  how far (per joint, and at the tool point), and how far past the held command it goes.

A frame grid for a session is 15 fps over each stretch of accepted commands (``JC`` outcome ``A``, no gap over 0.3 s, torque
on); its state is the newest valid reading (``RI``) and its action the newest relay target (``RO``), as the dataset builder
(``robot/lerobot/teleop_lerobot/takes.py``) takes them at each recorded frame. A dataset's frames are its own.

Standard library only, except a LeRobot dataset's parquet (pyarrow). Run it from a checkout (it reads the URDF and the FK):

    python3 scripts/gpu/arm_response.py <path>... [--label NAME] [--json OUT] [--only tracking,motion,...]

A path is a session or take folder (has ``robothost.jlog``), a LeRobot dataset (has ``meta/info.json``; ``--follow-takes``
also reads the takes its ``meta/teleop/takes.jsonl`` names, for the measures that need ``JC``/``RO``/``RI`` at their own
rate), or a folder holding either, searched two levels deep. Several ``--group LABEL=path,path`` compare groups.
"""

from __future__ import annotations

import argparse
import bisect
import glob
import json
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

# The checkout whose URDF and FK it reads: TELEOP_REPO (for `ssh host 'TELEOP_REPO=~/repos/isaac python3 - ...' < this`),
# else the one this file is in.
REPO = os.path.expanduser(os.environ.get("TELEOP_REPO") or
                          os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")))
sys.path.insert(0, os.path.join(REPO, "robot", "isaac"))
from teleop_isaac import kinematics as kin  # noqa: E402

URDF = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.urdf")
SIM = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.sim.json")
ZERO_PULSE = 500.0
PULSE_PER_RAD = 1000.0 / (240.0 * math.pi / 180.0)  # 238.73: GenericArmPlantConfig.Default
DEG_PER_UNIT = 0.24
ARM = (1, 2, 3, 4, 5)          # the motors read back and scored (the gripper's state is its command in the datasets)
PITCH = (1, 2, 3, 4)           # yaw and the three pitch joints: what moves the tool point most
JOINT_OF = {1: "base_yaw", 2: "proximal", 3: "distal", 4: "wrist", 5: "roll"}
THRESHOLD = 20.0               # world-models' motion_threshold, raw units
FPS = 15
TOOL_M = 0.15                  # world-models' TCP_OFFSET_M along gripper_link's x
GAP_S = 0.30                   # no accepted command for longer: a new stretch (metrics.py's GAP_S)


# --- reading ---------------------------------------------------------------------------------------------------------------

class Session:
    """What a session or take recorded, per motor, on RobotHost's clock (``tps`` ticks a second)."""

    def __init__(self, path: str):
        self.path = path
        self.name = os.path.basename(os.path.normpath(path))
        self.tps = 1_000_000_000
        self.ro: Dict[int, Tuple[List[int], List[float]]] = {}
        self.pps: List[Tuple[int, float]] = []
        self.ri: Dict[int, Tuple[List[int], List[float]]] = {}
        self.jc: List[Tuple[int, str, Dict[int, float], float]] = []  # (t, outcome, motor -> angle rad, speed rad/s)
        self.tq: List[Tuple[int, int]] = []
        with open(os.path.join(path, "robothost.jlog"), encoding="utf-8", errors="replace") as f:
            for line in f:
                tag = line[:3]
                if tag == "RI|":
                    p = line.rstrip("\n").split("|")
                    t = int(p[1])
                    for e in p[2].split(";"):
                        m, valid, pulse = e.split(":")
                        if valid == "1":
                            tr = self.ri.setdefault(int(m), ([], []))
                            tr[0].append(t)
                            tr[1].append(float(pulse))
                elif tag == "RO|":
                    p = line.rstrip("\n").split("|")
                    t = int(p[1])
                    pps = 0.0
                    for e in p[2].split(";"):
                        m, pulse, pps_s = e.split(":")
                        tr = self.ro.setdefault(int(m), ([], []))
                        tr[0].append(t)
                        tr[1].append(float(pulse))
                        pps = float(pps_s)
                    self.pps.append((t, pps))
                elif tag == "JC|":
                    p = line.rstrip("\n").split("|")
                    angles, speed = {}, 0.0
                    for e in p[6].split(";"):
                        q = e.split(":")
                        if len(q) == 3:
                            angles[int(q[0])] = float(q[1])
                            speed = max(speed, float(q[2]))
                    self.jc.append((int(p[1]), p[5], angles, speed))
                elif tag == "TQ|":
                    p = line.rstrip("\n").split("|")
                    self.tq.append((int(p[1]), int(p[2])))
                elif line.startswith("JSES|"):
                    p = line.rstrip("\n").split("|")
                    if len(p) > 3 and p[3].isdigit():
                        self.tps = int(p[3])
        for d in (self.ro, self.ri):
            for m, (ts, vs) in d.items():
                order = sorted(range(len(ts)), key=lambda i: ts[i])
                d[m] = ([ts[i] for i in order], [vs[i] for i in order])
        self.jc.sort(key=lambda r: r[0])
        self.tq.sort()
        self.offsets = zero_offsets(path)

    def at(self, track: Dict[int, Tuple[List[int], List[float]]], m: int, t: int) -> Optional[float]:
        tr = track.get(m)
        if not tr:
            return None
        i = bisect.bisect_right(tr[0], t) - 1
        return tr[1][i] if i >= 0 else None

    def torque_on(self, t: int) -> bool:
        i = bisect.bisect_right(self.tq, (t, 9)) - 1
        return i < 0 or self.tq[i][1] == 1

    def stretches(self) -> List[Tuple[int, int]]:
        """Spans of accepted commands with no gap over GAP_S and torque on throughout."""
        gap = int(GAP_S * self.tps)
        out: List[Tuple[int, int]] = []
        start = last = None
        for t, outcome, _, _ in self.jc:
            ok = outcome == "A" and self.torque_on(t)
            if ok and last is not None and t - last <= gap:
                last = t
                continue
            if start is not None and last - start > self.tps:  # stretches under a second are dropped
                out.append((start, last))
            start, last = (t, t) if ok else (None, None)
        if start is not None and last - start > self.tps:
            out.append((start, last))
        return out


def zero_offsets(path: str) -> Dict[int, float]:
    """Each motor's zero offset as the session was recorded (its robot-profile.json), else the twin's sidecar."""
    prof = os.path.join(path, "robot-profile.json")
    if os.path.exists(prof):
        with open(prof, encoding="utf-8") as f:
            doc = json.load(f)
        out = {int(j["motorId"]): float(j.get("zeroOffsetRadians", 0.0)) for j in doc.get("joints", [])}
        return {m: out.get(m, 0.0) for m in ARM}
    return sim_offsets()


def sim_offsets() -> Dict[int, float]:
    with open(SIM, encoding="utf-8") as f:
        doc = json.load(f)
    return {int(m["motorId"]): float(m["zeroOffsetRadians"]) for m in doc["motors"] if int(m["motorId"]) in ARM}


class Episode:
    """15 fps frames: ``state`` and ``action`` are lists of 5-lists (motors 1-5, raw units); ``t`` seconds."""

    def __init__(self, name: str, t: List[float], state: List[List[float]], action: List[List[float]], offsets: Dict[int, float]):
        self.name, self.t, self.state, self.action, self.offsets = name, t, state, action, offsets


def session_episodes(s: Session) -> List[Episode]:
    out = []
    step = s.tps / FPS
    for k, (a, b) in enumerate(s.stretches()):
        ts, st, ac = [], [], []
        g = float(a)
        while g <= b:
            t = int(g)
            state = [s.at(s.ri, m, t) for m in ARM]
            action = [s.at(s.ro, m, t) for m in ARM]
            if None not in state and None not in action:
                ts.append((t - a) / s.tps)
                st.append(state)
                ac.append(action)
            g += step
        if len(ts) >= FPS:
            out.append(Episode(f"{s.name}#{k}", ts, st, ac, s.offsets))
    return out


def dataset_episodes(path: str) -> List[Episode]:
    import pyarrow.parquet as pq  # only a dataset needs it

    offsets = sim_offsets()
    by_ep: Dict[int, List[Tuple[int, List[float], List[float]]]] = {}
    for f in sorted(glob.glob(os.path.join(path, "data", "chunk-*", "file-*.parquet"))):
        tab = pq.read_table(f, columns=["episode_index", "frame_index", "observation.state", "action"]).to_pydict()
        for e, i, s, a in zip(tab["episode_index"], tab["frame_index"], tab["observation.state"], tab["action"]):
            by_ep.setdefault(int(e), []).append((int(i), [float(x) for x in s[:5]], [float(x) for x in a[:5]]))
    out = []
    name = os.path.basename(os.path.normpath(path))
    for e in sorted(by_ep):
        rows = sorted(by_ep[e], key=lambda r: r[0])
        out.append(Episode(f"{name}/ep{e}", [r[0] / FPS for r in rows], [r[1] for r in rows], [r[2] for r in rows], offsets))
    return out


def dataset_takes(path: str) -> List[str]:
    out = []
    ledger = os.path.join(path, "meta", "teleop", "takes.jsonl")
    if os.path.exists(ledger):
        with open(ledger, encoding="utf-8") as f:
            for line in f:
                if line.strip():
                    take = json.loads(line).get("take")
                    if take and os.path.exists(os.path.join(take, "robothost.jlog")):
                        out.append(take)
    return out


def expand(paths: Sequence[str], follow_takes: bool) -> Tuple[List[str], List[str]]:
    """(sessions/takes, datasets) under the paths."""
    sessions, datasets = [], []

    def visit(p: str, depth: int) -> None:
        if os.path.exists(os.path.join(p, "robothost.jlog")):
            sessions.append(p)
        elif os.path.exists(os.path.join(p, "meta", "info.json")):
            datasets.append(p)
            if follow_takes:
                sessions.extend(dataset_takes(p))
        elif depth < 2 and os.path.isdir(p):
            for q in sorted(os.listdir(p)):
                if not q.startswith((".", "_")):
                    visit(os.path.join(p, q), depth + 1)

    for p in paths:
        visit(os.path.abspath(p), 0)
    return sessions, datasets


# --- forward kinematics ----------------------------------------------------------------------------------------------------

_JOINTS = None


def tool(pulses: Sequence[float], offsets: Dict[int, float]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """The tool point (0.15 m along gripper_link's x) and gripper_link's orientation, base frame, for motors 1-5's pulses."""
    global _JOINTS
    if _JOINTS is None:
        _JOINTS = kin.read_urdf(URDF)
    values = {JOINT_OF[m]: (p - ZERO_PULSE) / PULSE_PER_RAD - offsets.get(m, 0.0) for m, p in zip(ARM, pulses)}
    p, q = kin.link_pose(_JOINTS, values, "gripper_link")
    d = kin.q_rotate(q, (TOOL_M, 0.0, 0.0))
    return (p[0] + d[0], p[1] + d[1], p[2] + d[2]), q


def dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def ee_delta(p0, q0, p1) -> Tuple[float, float, float]:
    """The tool point's move from frame k to k+1 in frame k's DROID-style end-effector axes (x = -y_g, y = -z_g, z = x_g)."""
    w, x, y, z = q0
    inv = (w, -x, -y, -z)
    d = kin.q_rotate(inv, (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]))  # in gripper_link's axes
    return (-d[1], -d[2], d[0])


# --- statistics ------------------------------------------------------------------------------------------------------------

def pct(v: Sequence[float], q: float) -> Optional[float]:
    if not v:
        return None
    s = sorted(v)
    x = q * (len(s) - 1)
    i = int(x)
    j = min(i + 1, len(s) - 1)
    return s[i] + (x - i) * (s[j] - s[i])


def summary(v: Sequence[float], qs=(0.5, 0.9, 0.95, 0.99), nd: int = 2) -> Dict:
    out = {"p" + format(q * 100, "g"): (round(pct(v, q), nd) if v else None) for q in qs}
    out["n"] = len(v)
    return out


# --- the measures ----------------------------------------------------------------------------------------------------------

def tracking(eps: List[Episode]) -> Dict:
    over_total = n = 0
    runs: List[int] = []
    per = {m: [] for m in ARM}
    maxerr: List[float] = []
    tool_err: List[float] = []
    first_row: List[float] = []
    for e in eps:
        run = 0
        for k, (s, a) in enumerate(zip(e.state, e.action)):
            errs = [abs(x - y) for x, y in zip(s, a)]
            for m, v in zip(ARM, errs):
                per[m].append(v)
            me = max(errs)
            maxerr.append(me)
            n += 1
            if me > THRESHOLD:
                over_total += 1
                run += 1
            elif run:
                runs.append(run)
                run = 0
            te = dist(tool(s, e.offsets)[0], tool(a, e.offsets)[0]) * 100.0
            tool_err.append(te)
            if k == 0:
                first_row.append(te)
        if run:
            runs.append(run)
    over_runs = sum(runs)
    long_share = (sum(r for r in runs if r > 6) / over_runs) if over_runs else None
    return {
        "frames": n, "episodes": len(eps),
        "share_over_20": round(over_total / n, 4) if n else None,
        "max_joint_error_deg": summary([x * DEG_PER_UNIT for x in maxerr]),
        "joint_error_units": {str(m): summary(per[m]) for m in ARM},
        "runs_over_20_frames": dict(summary([float(r) for r in runs], (0.5, 0.9, 0.99), 1), max=max(runs) if runs else None),
        "share_of_over_frames_in_runs_over_6": round(long_share, 3) if long_share is not None else None,
        "tool_error_cm": summary(tool_err),
        "episode_first_frame_tool_error_cm": summary(first_row, (0.5, 0.9)),
    }


def motion(eps: List[Episode]) -> Dict:
    cmd = {m: [] for m in ARM}
    meas = {m: [] for m in ARM}
    cmd_tool, meas_tool, cmd_tool_moving = [], [], []
    axes = ([], [], [])
    moving = n = 0
    for e in eps:
        prev_a = prev_s = None
        for s, a in zip(e.state, e.action):
            pa, qa = tool(a, e.offsets)
            ps, _ = tool(s, e.offsets)
            if prev_a is not None:
                da = [abs(x - y) for x, y in zip(a, prev_a[0])]
                for m, v in zip(ARM, da):
                    cmd[m].append(v)
                for m, v in zip(ARM, (abs(x - y) for x, y in zip(s, prev_s[0]))):
                    meas[m].append(v)
                d = dist(pa, prev_a[1]) * 100.0
                cmd_tool.append(d)
                meas_tool.append(dist(ps, prev_s[1]) * 100.0)
                for i, v in enumerate(ee_delta(prev_a[1], prev_a[2], pa)):
                    axes[i].append(v * 100.0)
                n += 1
                if max(da) >= 2.0:
                    moving += 1
                    cmd_tool_moving.append(d)
            prev_a, prev_s = (a, pa, qa), (s, ps)
    return {
        "frame_pairs": n,
        "share_command_moving": round(moving / n, 3) if n else None,
        "command_units_per_frame": {str(m): summary(cmd[m]) for m in ARM},
        "measured_units_per_frame": {str(m): summary(meas[m]) for m in ARM},
        "command_tool_cm_per_frame": summary(cmd_tool),
        "command_tool_cm_per_frame_moving": summary(cmd_tool_moving),
        "measured_tool_cm_per_frame": summary(meas_tool),
        "command_ee_axis_cm_p1_p99": {k: [round(pct(v, 0.01), 2), round(pct(v, 0.99), 2)] if v else None
                                      for k, v in zip(("x", "y", "z"), axes)},
    }


BINS = (0, 25, 50, 100, 200, 400, 800)  # commanded joint speed, units/s


def bins(eps: List[Episode]) -> Dict:
    """Each pitch/yaw joint's |error| by how fast its command moved over the last 0.2 s (3 frames)."""
    acc: Dict[int, List[float]] = {i: [] for i in range(len(BINS))}
    for e in eps:
        for k in range(3, len(e.t)):
            dt = e.t[k] - e.t[k - 3]
            if dt <= 0:
                continue
            for j, m in enumerate(ARM):
                if m not in PITCH:
                    continue
                v = abs(e.action[k][j] - e.action[k - 3][j]) / dt
                b = bisect.bisect_right(BINS, v) - 1
                acc[b].append(abs(e.state[k][j] - e.action[k][j]))
    total = sum(len(v) for v in acc.values())
    out = {}
    for i, lo in enumerate(BINS):
        hi = BINS[i + 1] if i + 1 < len(BINS) else None
        v = acc[i]
        out[f"{lo}-{hi if hi else 'up'}"] = {"share": round(len(v) / total, 4) if total else None,
                                              "err_p50": round(pct(v, 0.5), 1) if v else None,
                                              "err_p90": round(pct(v, 0.9), 1) if v else None,
                                              "share_err_over_20": round(sum(1 for x in v if x > THRESHOLD) / len(v), 3) if v else None}
    return out


def grid(s: Session, track, m: int, a: int, b: int, step: int) -> List[Optional[float]]:
    tr = track.get(m)
    if not tr:
        return []
    ts, vs = tr
    out, i = [], bisect.bisect_right(ts, a) - 1
    t = a
    while t <= b:
        while i + 1 < len(ts) and ts[i + 1] <= t:
            i += 1
        out.append(vs[i] if i >= 0 else None)
        t += step
    return out


def lag(sessions: List[Session], max_ms: int = 500) -> Dict:
    """Per joint, the delay d minimising the median |RI(t) - RO(t - d)| over moments the command moved (3+ units in 0.1 s)."""
    dt_ms = 10
    res = {}
    for m in PITCH:
        cost = {d: [] for d in range(0, max_ms + 1, dt_ms)}
        for s in sessions:
            step = s.tps * dt_ms // 1000
            for a, b in s.stretches():
                ro = grid(s, s.ro, m, a, b, step)
                ri = grid(s, s.ri, m, a, b, step)
                if not ro or len(ro) != len(ri):
                    continue
                lagn = max_ms // dt_ms
                for k in range(lagn + 10, len(ro)):
                    if ri[k] is None or ro[k] is None or ro[k - 10] is None or abs(ro[k] - ro[k - 10]) < 3:
                        continue
                    for d in cost:
                        c = ro[k - d // dt_ms]
                        if c is not None:
                            cost[d].append(abs(ri[k] - c))
        med = {d: pct(v, 0.5) for d, v in cost.items() if v}
        if not med:
            continue
        best = min(med, key=lambda d: (med[d], d))
        res[str(m)] = {"best_ms": best, "median_err_at_best": round(med[best], 1), "median_err_at_0": round(med[0], 1),
                       "n": len(cost[best])}
    return res


def smooth(v: List[float], sigma_n: float) -> List[float]:
    r = int(3 * sigma_n + 0.5)
    w = [math.exp(-0.5 * (i / sigma_n) ** 2) for i in range(-r, r + 1)]
    out = []
    n = len(v)
    for k in range(n):
        acc = ws = 0.0
        for i in range(-r, r + 1):
            j = k + i
            if 0 <= j < n:
                acc += w[i + r] * v[j]
                ws += w[i + r]
        out.append(acc / ws)
    return out


def kinematics(sessions: List[Session], fs: float = 100.0, sigma_s: float = 0.025) -> Dict:
    """Commanded joint speed and acceleration (rad/s, rad/s^2) and the tool point's speed (m/s), from accepted JC."""
    speed = {m: [] for m in ARM}
    speed_moving = {m: [] for m in ARM}
    accel = {m: [] for m in ARM}
    tool_speed: List[float] = []
    reversals = 0
    seconds = 0.0
    for s in sessions:
        for a, b in s.stretches():
            rows = [(t, ang) for t, o, ang, _ in s.jc if a <= t <= b and o == "A" and all(m in ang for m in ARM)]
            if len(rows) < 10:
                continue
            ts = [(t - a) / s.tps for t, _ in rows]
            n = int(ts[-1] * fs)
            if n < 20:
                continue
            seconds += ts[-1]
            series = {}
            for m in ARM:
                vs = [ang[m] for _, ang in rows]
                out, i = [], 0
                for k in range(n):
                    t = k / fs
                    while i + 1 < len(ts) and ts[i + 1] <= t:
                        i += 1
                    if i + 1 < len(ts) and ts[i + 1] > ts[i]:
                        u = (t - ts[i]) / (ts[i + 1] - ts[i])
                        out.append(vs[i] + u * (vs[i + 1] - vs[i]))
                    else:
                        out.append(vs[i])
                series[m] = smooth(out, sigma_s * fs)
            vel = {m: [(series[m][k + 1] - series[m][k - 1]) * fs / 2 for k in range(1, n - 1)] for m in ARM}
            for m in ARM:
                v = vel[m]
                speed[m].extend(abs(x) for x in v)
                speed_moving[m].extend(abs(x) for x in v if abs(x) > 0.05)
                accel[m].extend(abs(v[k + 1] - v[k - 1]) * fs / 2 for k in range(1, len(v) - 1))
                if m in PITCH:
                    sign, peak = 0, 0.0
                    for x in v:  # a reversal: the joint turns back after moving 3 degrees one way
                        if abs(x) < 0.05:
                            continue
                        sg = 1 if x > 0 else -1
                        if sign and sg != sign and peak >= math.radians(3):
                            reversals += 1
                            peak = 0.0
                        if sg != sign:
                            sign, peak = sg, 0.0
                        peak += abs(x) / fs
            offs = s.offsets
            prev = None
            for k in range(0, n, 2):
                pulses = [ZERO_PULSE + (series[m][k] + offs.get(m, 0.0)) * PULSE_PER_RAD for m in ARM]
                p = tool(pulses, offs)[0]
                if prev is not None:
                    tool_speed.append(dist(p, prev) * fs / 2)
                prev = p
    qs = (0.5, 0.9, 0.95, 0.99, 0.999)
    return {
        "seconds": round(seconds, 1),
        "joint_speed_rad_s": {str(m): summary(speed[m], qs) for m in ARM},
        "joint_speed_rad_s_moving": {str(m): summary(speed_moving[m], qs) for m in ARM},
        "joint_accel_rad_s2": {str(m): summary(accel[m], qs, 1) for m in ARM},
        "tool_speed_m_s": summary(tool_speed, qs, 3),
        "reversals_per_min_joints_1_4": round(reversals / seconds * 60, 1) if seconds else None,
    }


def steps(sessions: List[Session]) -> Dict:
    """Responses to RO steps: a joint's target jumps 40+ units after holding 0.5 s and then holds 1.2 s."""
    rows = []
    for s in sessions:
        tps = s.tps
        for m in PITCH:
            if m not in s.ro or m not in s.ri:
                continue
            ts, vs = s.ro[m]
            rts, rvs = s.ri[m]
            for i in range(1, len(ts)):
                jump = vs[i] - vs[i - 1]
                if abs(jump) < 40:
                    continue
                t0 = ts[i]
                j0 = bisect.bisect_left(ts, t0 - tps // 2)
                if any(abs(vs[k] - vs[i - 1]) > 1 for k in range(j0, i)):
                    continue
                j1 = bisect.bisect_right(ts, t0 + int(1.2 * tps))
                if any(abs(vs[k] - vs[i]) > 1 for k in range(i, j1)) or (ts[-1] - t0) < 1.2 * tps:
                    continue
                if not s.torque_on(t0):
                    continue
                target = vs[i]
                k0 = bisect.bisect_left(rts, t0 - tps // 5)
                k1 = bisect.bisect_left(rts, t0)
                before = [rvs[k] for k in range(k0, k1)]
                if not before:
                    continue
                base = pct(before, 0.5)
                amp = target - base
                if abs(amp) < 30:
                    continue
                sign = 1 if amp > 0 else -1
                after = [(rts[k], rvs[k]) for k in range(k1, bisect.bisect_right(rts, t0 + int(1.2 * tps)))]

                def cross(frac: float) -> Optional[float]:
                    for t, v in after:
                        if (v - base) * sign >= frac * abs(amp):
                            return (t - t0) / tps
                    return None

                t10, t50, t90 = cross(0.1), cross(0.5), cross(0.9)
                peak = 0.0
                for (ta, va), (tb, vb) in zip(after, after[2:]):
                    if tb > ta:
                        peak = max(peak, (vb - va) * sign * tps / (tb - ta))
                over = max(((v - target) * sign for _, v in after), default=0.0)
                tail = [v for t, v in after if t >= t0 + int(1.0 * tps)]
                pps = next((p for t, p in reversed(s.pps) if t <= t0), 0.0)
                rows.append({"session": s.name, "motor": m, "amp": round(amp, 1), "pps": round(pps, 1),
                             "t10_ms": None if t10 is None else round(t10 * 1000), "t50_ms": None if t50 is None else round(t50 * 1000),
                             "t90_ms": None if t90 is None else round(t90 * 1000),
                             "rise_10_90_ms": None if t10 is None or t90 is None else round((t90 - t10) * 1000),
                             "peak_units_s": round(peak), "overshoot_units": round(max(0.0, over), 1),
                             "error_at_1s_units": round(abs(pct(tail, 0.5) - target), 1) if tail else None})
    by_pps: Dict[float, List[Dict]] = {}
    for r in rows:
        by_pps.setdefault(round(r["pps"]), []).append(r)
    out = {}
    for pps, rs in sorted(by_pps.items()):
        def med(key):
            v = [r[key] for r in rs if r[key] is not None]
            return round(pct(v, 0.5), 1) if v else None
        out[str(pps)] = {"n": len(rs), "amp_abs_p50": med("amp") if False else round(pct([abs(r["amp"]) for r in rs], 0.5), 1),
                         "t10_ms_p50": med("t10_ms"), "t50_ms_p50": med("t50_ms"), "t90_ms_p50": med("t90_ms"),
                         "rise_10_90_ms_p50": med("rise_10_90_ms"), "peak_units_s_p50": med("peak_units_s"),
                         "peak_over_pps_p50": round(pct([r["peak_units_s"] / r["pps"] for r in rs if r["pps"] > 0], 0.5), 2) if rs else None,
                         "overshoot_units_p50": med("overshoot_units"), "error_at_1s_units_p50": med("error_at_1s_units")}
    return {"by_pps": out, "n": len(rows)}


def stops(sessions: List[Session], hold_s: float = 0.6, min_speed: float = 60.0) -> Dict:
    """Moments every joint's relay target stops moving (no RO change over 1 unit for ``hold_s``) after at least one moved
    ``min_speed`` units/s over the last 0.15 s. Per moving joint: how long its reading keeps changing (until it stays
    within 2 units of where it ends), how far it goes after the stop, and how far past the held target; the tool point's
    travel after the stop."""
    events = []
    for s in sessions:
        tps = s.tps
        motors = [m for m in ARM if m in s.ro and m in s.ri]
        if len(motors) < len(ARM):
            continue
        ts = s.ro[1][0]
        vals = {m: s.ro[m][1] for m in motors}
        if any(len(vals[m]) != len(ts) for m in motors):
            continue
        last_stop = -10 ** 18
        for i in range(1, len(ts) - 1):
            t0 = ts[i]
            if t0 - last_stop < tps:  # one event per second at most
                continue
            # the target moved into i, and holds from i on
            if max(abs(vals[m][i] - vals[m][i - 1]) for m in motors) < 1.0:
                continue
            j1 = bisect.bisect_right(ts, t0 + int(hold_s * tps))
            if j1 >= len(ts) or any(abs(vals[m][k] - vals[m][i]) > 1.0 for m in motors for k in range(i, j1)):
                continue
            if not s.torque_on(t0) or not s.torque_on(t0 + int(hold_s * tps)):
                continue
            j0 = bisect.bisect_left(ts, t0 - int(0.15 * tps))
            if ts[i] - ts[j0] <= 0:
                continue
            speed = {m: abs(vals[m][i] - vals[m][j0]) * tps / (ts[i] - ts[j0]) for m in motors}
            if max(speed[m] for m in PITCH) < min_speed:
                continue
            end = t0 + int(hold_s * tps)
            state0 = [s.at(s.ri, m, t0) for m in ARM]
            if None in state0:
                continue
            final = []
            per = {}
            for m in ARM:
                rts, rvs = s.ri[m]
                k0, k1 = bisect.bisect_left(rts, t0), bisect.bisect_right(rts, end)
                window = [(rts[k], rvs[k]) for k in range(k0, k1)]
                tail = [v for t, v in window if t >= end - int(0.15 * tps)]
                if len(tail) < 2:
                    break
                fin = pct(tail, 0.5)
                final.append(fin)
                settle = t0
                for t, v in window:
                    if abs(v - fin) > 2.0:
                        settle = t
                sign = 1 if vals[m][i] >= vals[m][j0] else -1
                past = max(((v - vals[m][i]) * sign for _, v in window), default=0.0)
                if speed[m] >= min_speed:
                    per[m] = {"speed_units_s": round(speed[m]), "stop_ms": round((settle - t0) * 1000 / tps),
                              "travel_units": round(abs(fin - state0[ARM.index(m)]), 1),
                              "error_at_stop_units": round(abs(vals[m][i] - state0[ARM.index(m)]), 1),
                              "past_target_units": round(max(0.0, past), 1)}
            if len(final) < len(ARM) or not per:
                continue
            p0 = tool(state0, s.offsets)[0]
            pf = tool(final, s.offsets)[0]
            pc = tool([vals[m][i] for m in ARM], s.offsets)[0]
            pcb = tool([vals[m][j0] for m in ARM], s.offsets)[0]
            tool_speed = dist(pc, pcb) * tps / (ts[i] - ts[j0])
            events.append({"session": s.name, "t_s": round((t0 - ts[0]) / tps, 2), "joints": per,
                           "tool_speed_m_s": round(tool_speed, 3), "tool_travel_cm": round(dist(p0, pf) * 100, 2),
                           "tool_error_at_stop_cm": round(dist(p0, pc) * 100, 2), "stop_ms": max(j["stop_ms"] for j in per.values())})
            last_stop = t0
    joints = {}
    for m in PITCH:
        rs = [e["joints"][m] for e in events if m in e["joints"]]
        if rs:
            joints[str(m)] = {"n": len(rs), "speed_units_s": summary([r["speed_units_s"] for r in rs], (0.5, 0.9), 0),
                              "stop_ms": summary([r["stop_ms"] for r in rs], (0.5, 0.9), 0),
                              "travel_deg": summary([r["travel_units"] * DEG_PER_UNIT for r in rs], (0.5, 0.9)),
                              "past_target_deg": summary([r["past_target_units"] * DEG_PER_UNIT for r in rs], (0.5, 0.9))}
    by_speed = {}
    for lo, hi in ((0.0, 0.1), (0.1, 0.2), (0.2, 0.4), (0.4, 10.0)):
        rs = [e for e in events if lo <= e["tool_speed_m_s"] < hi]
        if rs:
            by_speed[f"{lo}-{hi}"] = {"n": len(rs), "stop_ms": summary([e["stop_ms"] for e in rs], (0.5, 0.9), 0),
                                      "tool_travel_cm": summary([e["tool_travel_cm"] for e in rs], (0.5, 0.9)),
                                      "tool_error_at_stop_cm": summary([e["tool_error_at_stop_cm"] for e in rs], (0.5, 0.9))}
    return {"n": len(events), "joints": joints, "tool_by_command_speed_m_s": by_speed,
            "stop_ms": summary([e["stop_ms"] for e in events], (0.5, 0.9), 0),
            "tool_travel_cm": summary([e["tool_travel_cm"] for e in events], (0.5, 0.9)),
            "events": events if len(events) <= 40 else events[:40]}


MEASURES = ("tracking", "motion", "bins", "lag", "kinematics", "steps", "stops")


def report(label: str, paths: Sequence[str], only: Sequence[str], follow_takes: bool) -> Dict:
    sess_paths, datasets = expand(paths, follow_takes)
    sessions = []
    for p in sess_paths:
        try:
            sessions.append(Session(p))
        except (OSError, ValueError, IndexError) as e:
            print(f"[arm_response] skipped {p}: {e}", file=sys.stderr)
    eps: List[Episode] = []
    for d in datasets:
        eps.extend(dataset_episodes(d))
    if not datasets:  # a dataset's own frames, else the sessions' 15 fps grid
        for s in sessions:
            eps.extend(session_episodes(s))
    out: Dict = {"label": label, "sessions": len(sessions), "datasets": [os.path.basename(d) for d in datasets],
                 "speed_slider_pps": summary([p for s in sessions for _, p in s.pps], (0.5,), 0)}
    for name in only:
        if name in ("tracking", "motion", "bins"):
            out[name] = globals()[name](eps) if eps else None
        else:
            out[name] = globals()[name](sessions) if sessions else None
        print(f"[arm_response] {label}: {name} done", file=sys.stderr)
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--label", default="input")
    ap.add_argument("--group", action="append", default=[], help="LABEL=path[,path...] (repeatable)")
    ap.add_argument("--only", default=",".join(MEASURES))
    ap.add_argument("--follow-takes", action="store_true", help="also read the takes a dataset's ledger names")
    ap.add_argument("--json", help="write the report here")
    a = ap.parse_args(argv)
    only = [x for x in a.only.split(",") if x]
    bad = [x for x in only if x not in MEASURES]
    if bad:
        ap.error(f"unknown measure(s) {bad}; choose from {MEASURES}")
    groups = [(a.label, a.paths)] if a.paths else []
    for g in a.group:
        label, _, rest = g.partition("=")
        groups.append((label, [p for p in rest.split(",") if p]))
    if not groups:
        ap.error("no input")
    doc = {"threshold_units": THRESHOLD, "fps": FPS, "tool_offset_m": TOOL_M, "groups": [report(l, p, only, a.follow_takes) for l, p in groups]}
    text = json.dumps(doc, indent=1)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            f.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
