"""The simulated JetRover against the real one (docs/adr/0034 decision 6, phase J3; docs/metrics.md section 10), without Isaac.

- :func:`extract_session` reads a recorded real session's ``robothost.jlog`` down to what a replay needs: the relay targets
  (``RO``), the relay feedback (``RI``) and the torque switches (``TQ``), on RobotHost's clock.
- :class:`Replayer` replays them into any simulated servo set (:class:`ServoSim`) at their recorded times, from the session's
  first valid reading, through the driver's write gate, single thread and feedback cycles (:class:`servo.DriverEmulator`,
  docs/adr/0034 amendment 13; ``gate=False`` replays as before it), and scores the simulated readings
  against the recorded ones: ``sim_tracking_error_deg`` at every valid reading of motors 1-5, ``sim_settle_error_deg`` at
  those at least 0.5 s after the last command that changed that motor's target. The gripper is never scored.
- :func:`fit` is a coordinate search over the simulator's parameters, minimising the RMS tracking error of motors 1-4 on
  the fit sessions (:meth:`Score.objective`); the held-out sessions are scored after, never fitted.

The Isaac adapter is ``jetrover_bridge.py --replay``; the plain-Python one in the tests is a servo that follows its ramp.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, replace
from typing import Callable, Dict, List, Optional, Protocol, Sequence, Tuple

from . import servo

SCORED = (1, 2, 3, 4, 5)   # the motors that report a position; 10, the gripper, never does
POOLED = (1, 2, 3, 4)      # the pooled figure: the arm's pitch joints and the yaw
SETTLE_SECONDS = 0.5       # a reading counts toward settling this long after its motor's last target change


# --- the sessions -------------------------------------------------------------------------------------------------------

def extract_cycles(directory: str, tps: int) -> List:
    """The driver's read cycles from a session's ``driver.jlog``: ``[[start, end, [[t, motor], ...]], ...]`` in RobotHost's
    ticks (both logs are CLOCK_MONOTONIC), each with the reads that answered, at their request times. Empty without the log."""
    path = os.path.join(directory, "driver.jlog")
    if not os.path.exists(path):
        return []
    dtps, windows, reads = 1_000_000_000, [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            tag, _, rest = line.rstrip("\n").partition("|")
            p = rest.split("|")
            if tag == "JSES" and len(p) >= 3 and p[2].isdigit():
                dtps = int(p[2])
            elif tag == "CY" and len(p) >= 2 and p[0].isdigit() and p[1].isdigit():
                windows.append((int(p[0]), int(p[1])))
            elif tag == "RD" and len(p) >= 4 and p[0].isdigit() and p[3] != "":
                reads.append((int(p[0]), int(p[2])))
    windows.sort()
    reads.sort()
    scale = tps / dtps
    out, ri = [], 0
    for a, b in windows:
        while ri < len(reads) and reads[ri][0] < a:
            ri += 1
        mine = []
        while ri < len(reads) and reads[ri][0] <= b:
            mine.append([int(reads[ri][0] * scale), reads[ri][1]])
            ri += 1
        out.append([int(a * scale), int(b * scale), mine])
    return out


def extract_session(directory: str) -> Dict:
    """A session's replay: ``{"name", "tps", "events": [[t, kind, payload], ...], "cycles": [...]}`` with ``events`` sorted by
    ``t`` (RobotHost's ticks), ``kind`` ``"RO"`` (``[[motor, pulse, pps], ...]``), ``"RI"`` (``[[motor, valid, pulse], ...]``)
    or ``"TQ"`` (``1``/``0``), and ``cycles`` the driver's own read cycles (:func:`extract_cycles`; empty without its log)."""
    tps = 1_000_000_000
    events = []
    with open(os.path.join(directory, "robothost.jlog"), encoding="utf-8") as f:
        for line in f:
            tag, _, rest = line.rstrip("\n").partition("|")
            if tag == "JSES":
                parts = rest.split("|")
                if len(parts) >= 3 and parts[2].isdigit():
                    tps = int(parts[2])
            elif tag in ("RO", "RI", "TQ"):
                t, _, body = rest.partition("|")
                if not t.lstrip("-").isdigit():
                    continue
                if tag == "TQ":
                    events.append([int(t), "TQ", int(body.split("|")[0] == "1")])
                    continue
                entries = []
                for e in body.split(";"):
                    p = e.split(":")
                    if len(p) != 3:
                        continue
                    if tag == "RO":
                        entries.append([int(p[0]), float(p[1]), float(p[2])])
                    else:
                        entries.append([int(p[0]), int(p[1]), float(p[2])])
                events.append([int(t), tag, entries])
    events.sort(key=lambda e: e[0])
    return {"name": os.path.basename(os.path.normpath(directory)), "tps": tps, "events": events,
            "cycles": extract_cycles(directory, tps), "zeroOffsets": recorded_zero_offsets(directory)}


def recorded_zero_offsets(directory: str) -> Dict[str, float]:
    """Each motor's zero offset in the profile the session was recorded with (its ``robot-profile.json``), by motor id; empty
    without one. A pulse means the angle that profile says: the JetRover's shoulder servo was replaced on 2026-11-11 and its
    offset moved 44 degrees, so sessions before and after it must not share one conversion."""
    path = os.path.join(directory, "robot-profile.json")
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    return {str(int(j["motorId"])): float(j.get("zeroOffsetRadians", 0.0)) for j in doc.get("joints", [])}


# --- the simulator's side ------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Params:
    """What the fit tunes. ``delay_ms`` holds each command back before the servo starts its ramp (the bus and the driver);
    ``speed_scale`` multiplies each command's pulses per second (a servo slower or faster than asked)."""
    stiffness: float = 60.0
    damping: float = 1.5
    effort_scale: float = 1.0
    delay_ms: float = 0.0
    speed_scale: float = 1.0

    def to_dict(self) -> Dict[str, float]:
        return {"stiffness": self.stiffness, "damping": self.damping, "effortScale": self.effort_scale,
                "delayMs": self.delay_ms, "speedScale": self.speed_scale}


class ServoSim(Protocol):
    """A simulated arm, driven a physics step at a time."""

    dt: float

    def reset(self, pulses: Dict[int, float], params: Params) -> None:
        """Every motor at ``pulses``, at rest, torque on, with ``params`` (the sim applies ``speed_scale`` to each command)."""
        ...

    def command(self, motor: int, pulse: float, pps: float) -> None:
        """Without the driver's gate: the servo ramps to ``pulse`` at ``pps`` times ``speed_scale``."""
        ...

    def write(self, motor: int, pulse: float, seconds: float) -> None:
        """A board move packet's servo: to ``pulse`` over ``seconds`` divided by ``speed_scale``."""
        ...

    def torque(self, on: bool) -> None: ...

    def step(self) -> None: ...

    def read(self, motor: int) -> float:
        """The motor's position now, in pulses."""
        ...


@dataclass
class Score:
    tracking: Dict[int, List[float]] = field(default_factory=dict)  # motor -> errors (deg) at every valid reading
    settle: Dict[int, List[float]] = field(default_factory=dict)    # motor -> errors (deg) at settled readings

    def add(self, other: "Score") -> None:
        for mine, theirs in ((self.tracking, other.tracking), (self.settle, other.settle)):
            for m, v in theirs.items():
                mine.setdefault(m, []).extend(v)

    def summary(self) -> Dict:
        def pct(v: Sequence[float], q: float) -> Optional[float]:
            if not v:
                return None
            s = sorted(v)
            return round(s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))], 3)

        out: Dict = {}
        for name, d in (("sim_tracking_error_deg", self.tracking), ("sim_settle_error_deg", self.settle)):
            per = {str(m): {"p50": pct(d.get(m, []), 0.5), "p95": pct(d.get(m, []), 0.95), "n": len(d.get(m, []))} for m in SCORED}
            pooled = [x for m in POOLED for x in d.get(m, [])]
            out[name] = {"motors": per, "pooled1to4": {"p50": pct(pooled, 0.5), "p95": pct(pooled, 0.95), "n": len(pooled)}}
        return out

    def objective(self) -> float:
        """What the fit minimises: the RMS of motors 1-4's tracking errors, plus half that of their settle errors. Not a p95:
        most readings are of joints standing still, so a pooled p95 is blind to the moving ones the fit is about. The
        reported figures are still section 10's p50 and p95."""
        def rms(d: Dict[int, List[float]]) -> Optional[float]:
            v = [x for m in POOLED for x in d.get(m, [])]
            return math.sqrt(sum(x * x for x in v) / len(v)) if v else None

        t, r = rms(self.tracking), rms(self.settle)
        return (t if t is not None else 1e9) + 0.5 * (r if r is not None else 0.0)


class Replayer:
    """Replays one session into a :class:`ServoSim` and scores it (docs/metrics.md section 10)."""

    def __init__(self, model: servo.SimModel, gate: bool = True, cycles: str = "recorded"):
        """``gate``: the driver's write gate and single thread (amendment 13). ``cycles``: ``"recorded"`` reads at the session's
        own driver cycles where it has them (:class:`servo.RecordedFeedback`), ``"emulated"`` runs the driver's cycles from
        the first reading (:class:`servo.FeedbackEmulator`, as before amendment 13)."""
        if cycles not in ("recorded", "emulated"):
            raise ValueError(f"cycles must be 'recorded' or 'emulated', not {cycles!r}")
        self.model = model
        self.gate = gate
        self.cycles = cycles

    def deg(self, pulses: float) -> float:
        return abs(pulses) / self.model.pulse_per_radian * 180.0 / math.pi

    def run(self, session: Dict, sim: ServoSim, params: Params, seconds: float = 0.0) -> Score:
        """Replays with the session's own zero offsets (the profile it was recorded with) and puts the model's back after."""
        before = self.model.set_zero_offsets({int(m): o for m, o in (session.get("zeroOffsets") or {}).items()})
        try:
            return self._run(session, sim, params, seconds)
        finally:
            self.model.set_zero_offsets(before)

    def _run(self, session: Dict, sim: ServoSim, params: Params, seconds: float) -> Score:
        events = session["events"]
        tps = float(session["tps"])
        first = next((i for i, e in enumerate(events) if e[1] == "RI" and all(v for m, v, _ in e[2] if m in SCORED)), None)
        score = Score()
        if first is None:
            return score
        t0 = events[first][0]
        start = {m: p for m, v, p in events[first][2] if v}
        # The gripper starts at its first commanded pulse (it never reports), else open.
        g = next((m for m, mo in self.model.motors.items() if mo.is_gripper), None)
        if g is not None:
            start[g] = next((p for e in events if e[1] == "RO" for m, p, _ in e[2] if m == g), self.model.motors[g].open_pulse)
        for m in self.model.motors:
            start.setdefault(m, self.model.zero_pulse)
        sim.reset(start, params)
        recorded = session.get("cycles") or []
        if self.cycles == "recorded" and recorded:
            fb = servo.RecordedFeedback(self.model, [((a - t0) / tps, (b - t0) / tps, [((t - t0) / tps, m) for t, m in reads])
                                                     for a, b, reads in recorded if b >= t0])
        else:
            fb = servo.FeedbackEmulator(self.model)
        drv = servo.DriverEmulator(self.model, fb, servo_move=sim.write, servo_command=sim.command, servo_torque=sim.torque,
                                   gate=self.gate)
        delay = params.delay_ms / 1000.0
        # Commands and torque switches before the first reading are already in the state it shows.
        todo = [e for e in events[first + 1:] if e[1] in ("RO", "TQ")]
        readings = [e for e in events[first:] if e[1] == "RI"]
        end = (readings[-1][0] - t0) / tps if readings else 0.0
        if seconds > 0:
            end = min(end, seconds)
        last_target: Dict[int, float] = {m: start[m] for m in SCORED}
        changed_at: Dict[int, float] = {m: -1e9 for m in SCORED}
        ti = ri = step = 0
        now = 0.0
        eps = 1e-9  # recorded times that fall on a physics step count at that step, however the floats round
        while now <= end:
            while ti < len(todo) and (todo[ti][0] - t0) / tps + (delay if todo[ti][1] == "RO" else 0.0) <= now + eps:
                kind, payload = todo[ti][1], todo[ti][2]
                if kind == "TQ":
                    drv.torque(now, bool(payload))
                else:
                    sent = (todo[ti][0] - t0) / tps
                    drv.command(now, payload)  # the sim applies params.speed_scale, as the live bridge does
                    for m, pulse, pps in payload:
                        if m in last_target and abs(pulse - last_target[m]) > 0.5:
                            last_target[m] = pulse
                            changed_at[m] = sent
                ti += 1
            mid = drv.due_read(now)
            while mid is not None:
                drv.record(mid, sim.read(mid), now)
                mid = drv.due_read(now)
            while ri < len(readings) and (readings[ri][0] - t0) / tps <= now + eps:
                at = (readings[ri][0] - t0) / tps
                for m, valid, pulse in readings[ri][2]:
                    if not valid or m not in SCORED or m not in fb.readings:
                        continue
                    err = self.deg(fb.readings[m][0] - pulse)
                    score.tracking.setdefault(m, []).append(err)
                    if at - changed_at[m] >= SETTLE_SECONDS:
                        score.settle.setdefault(m, []).append(err)
                ri += 1
            sim.step()
            step += 1
            now = step * sim.dt  # counted, not summed: a sum of steps drifts off the recorded times
        return score


# --- the fit ------------------------------------------------------------------------------------------------------------

# Each parameter's search: (how it moves, its first step, its bounds). "log" multiplies, "lin" adds.
SEARCH = {
    "stiffness": ("log", 2.0, (5.0, 2000.0)),
    "damping": ("log", 2.0, (0.05, 50.0)),
    "effort_scale": ("log", 1.5, (0.2, 5.0)),
    "delay_ms": ("lin", 20.0, (0.0, 120.0)),
    "speed_scale": ("log", 1.25, (0.4, 2.5)),
}


def fit(evaluate: Callable[[Params], float], start: Params = Params(), rounds: int = 3,
        log: Callable[[str], None] = lambda s: None) -> Tuple[Params, float, List[Dict]]:
    """Coordinate search: each parameter in turn tried one step either way, kept when better, the steps halved each round.
    Deterministic. Returns the best parameters, their objective and every evaluation made."""
    history: List[Dict] = []
    cache: Dict[Tuple, float] = {}

    def score(p: Params) -> float:
        key = tuple(round(v, 6) for v in p.to_dict().values())
        if key not in cache:
            cache[key] = evaluate(p)
            history.append({"params": p.to_dict(), "objective": round(cache[key], 4)})
            log(f"[fit] {p.to_dict()} -> {cache[key]:.3f}")
        return cache[key]

    best, best_score = start, score(start)
    steps = {k: v[1] for k, v in SEARCH.items()}
    for _ in range(rounds):
        for name, (kind, _, (lo, hi)) in SEARCH.items():
            improved = True
            while improved:
                improved = False
                cur = getattr(best, name)
                for direction in (1, -1):
                    if kind == "log":
                        nxt = cur * steps[name] ** direction
                    else:
                        nxt = cur + steps[name] * direction
                    nxt = min(max(nxt, lo), hi)
                    if abs(nxt - cur) < 1e-9:
                        continue
                    trial = replace(best, **{name: nxt})
                    s = score(trial)
                    if s < best_score - 1e-6:
                        best, best_score, improved = trial, s, True
                        break
        for name, (kind, _, _) in SEARCH.items():
            steps[name] = math.sqrt(steps[name]) if kind == "log" else steps[name] / 2
    return best, best_score, history


def write_result(path: str, doc: Dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
