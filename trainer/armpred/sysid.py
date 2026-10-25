"""``armpred sysid``: what the recordings say about the robot, as the simulator's ranges and a fitted
prior (ADR 0021 step D1). Everything here is measured from the session logs alone.

- **The bus:** read duration and its spread, how often a read misses, how often the board falls
  silent (every motor without a reading for over 2 s).
- **RobotHost's loop** (the frame cadence) and **the battery** (its start and its drift).
- **Reading ages:** how much older than its frame's sensed stamp each motor's value is. The fitted
  ``relay-replay`` takes these (``SensedAgeSeconds``) instead of the headset's guessed 45..5 ms.
- **The servos:** a lag and a speed limit per motor, fitted by running the simulator's servo model
  (a timed linear setpoint from where the servo is, followed with a first-order lag under a speed
  limit; no load) from every recorded write, over a grid of (lag, speed), against every recorded
  read. Sessions with a payload or a hand bias this, so only ``load: none`` sessions are used.
- **The pipeline delay:** how long a command takes from RobotHost's socket to the driver's gate (half
  a RobotHost loop waiting to be read, then the relay's poll and the driver's queue): what
  ``relay-replay-servo`` adds to every effective time (ADR 0021 decision 15).

``prior.json`` carries the fitted reading ages and, under ``relayReplayServo``, the servo lag, speed
limit (only when identified) and pipeline delay, for ``relay-replay-fitted`` and
``relay-replay-servo-fitted``.

The output is ``sysid.json`` (every estimate, with how much data it rests on) and ``sysid.toml``: a
``[ranges]`` table that drops into a ``sim-pretrain`` config, each range the fitted spread widened by
``WIDEN``, so the next pretraining draws around the robot instead of around guesses.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .contract.profile import JETROVER, RobotProfile
from .contract.session import Session, load_session

ARM = (1, 2, 3, 4, 5)
NS = 1_000_000_000
WIDEN = 0.15  # each fitted range is widened by this fraction either side
# Up to 0.2 s: the first robot fit (2026-10-25, the pilot) put most servos at 0.05 s, the old grid's top, so the
# lag was pinned there rather than fitted. A fit at either end of the grid is reported (servoLagNote).
LAGS_S = np.array([0.002, 0.004, 0.006, 0.008, 0.010, 0.013, 0.016, 0.020, 0.025, 0.030, 0.040, 0.050,
                   0.060, 0.075, 0.090, 0.110, 0.130, 0.160, 0.200])
SPEEDS_RAD_S = np.arange(2.0, 12.01, 0.5)
# A speed counts as fitted only if it beats the fastest candidate clearly; a nearly flat error says
# the limit never bound. And a range narrows only with enough fitted speeds behind it.
SPEED_IDENTIFIED = 0.95
MIN_SPEED_FITS = 3
STEP_NS = 1_000_000


def bus_timing(sessions: Sequence[Session]) -> Dict[str, float]:
    durations, reads, misses, silences, hours = [], 0, 0, 0, 0.0
    for s in sessions:
        r = s.reads
        ok = r.ok
        durations.append((r.t_resp[ok] - r.t_req[ok]) / 1e6)
        reads += len(r)
        misses += int((~ok).sum())
        span = (int(r.t_resp.max()) - int(r.t_req.min())) / NS
        hours += span / 3600.0
        # A silence: no motor read successfully for over 2 s.
        t = np.sort(r.t_mid[ok])
        silences += int((np.diff(t) > 2 * NS).sum()) if t.size > 1 else 0
    d = np.concatenate(durations) if durations else np.zeros(0)
    return {
        "read_ms": float(np.median(d)) if d.size else float("nan"),
        "read_jitter_ms": float(np.std(d)) if d.size else float("nan"),
        "read_miss_probability": misses / reads if reads else float("nan"),
        "board_silence_per_hour": silences / hours if hours else float("nan"),
        "reads": reads,
        "hours": hours,
    }


def robothost_loop_ms(sessions: Sequence[Session]) -> float:
    """The streamer sends once at least 20 ms have passed, checked once per loop, so a frame interval
    is a whole number of loops just over 20 ms; the median interval over that count is the loop."""
    intervals = np.concatenate([np.diff(s.frames.t) / 1e6 for s in sessions if len(s.frames) > 1])
    if intervals.size == 0:
        return float("nan")
    med = float(np.median(intervals))
    loops = max(1, round(med / 5.2))
    return med / loops


def pipeline_delay_s(sessions: Sequence[Session], loop_ms: float) -> float:
    """From RobotHost's socket to the driver's gate: half a RobotHost loop (a datagram waits for the
    loop to read it), then each command array's time from RobotHost's relay send (``RO``) to the
    driver's gate (``CI``), the median over every array."""
    deltas = []
    for s in sessions:
        ro = np.unique(s.relay_out.t)
        ci = np.unique(s.gate_in.t)
        if ro.size == 0 or ci.size == 0:
            continue
        k = np.searchsorted(ro, ci, side="right") - 1
        ok = k >= 0
        d = (ci[ok] - ro[k[ok]]) / NS
        deltas.append(d[d < 0.5])  # anything slower is a gap in the stream, not the pipeline
    d = np.concatenate(deltas) if deltas else np.zeros(0)
    if d.size == 0 or not np.isfinite(loop_ms):
        return float("nan")
    return float(np.median(d)) + 0.5 * loop_ms / 1000.0


def battery(sessions: Sequence[Session]) -> Dict[str, float]:
    starts, drops = [], []
    for s in sessions:
        b = s.battery
        if len(b) >= 2:
            minutes = (b[:, 0] - b[0, 0]) / NS / 60.0
            slope = np.polyfit(minutes, b[:, 1].astype(float), 1)[0] if minutes[-1] > 0 else 0.0
            starts.append(float(b[0, 1]))
            drops.append(float(-slope))
    return {"start_mv": float(np.median(starts)) if starts else float("nan"),
            "drop_mv_per_min": float(np.median(drops)) if drops else float("nan")}


def reading_ages_s(sessions: Sequence[Session], profile: RobotProfile = JETROVER) -> Dict[int, float]:
    """Per motor: frame sensed stamp minus the read midpoint of the value it carried.

    The value is matched by pulse, not by time: the relay forwards what the driver last *published*
    (at the end of a read cycle), so the newest read before the stamp is often one nobody has seen
    yet. Only frames where the joint moved count: a still joint's pulse matches every read, and the
    newest of them would make the age look young."""
    ages: Dict[int, List[float]] = {m: [] for m in ARM}
    for s in sessions:
        r = s.reads
        f = s.frames
        fresh = np.concatenate([[True], np.diff(f.sensed) != 0]) & (f.sensed > 0)
        idx = np.nonzero(fresh)[0]
        for m in ARM:
            sel = (r.motor == m) & r.ok
            t_mid, t_resp, pulse = r.t_mid[sel], r.t_resp[sel], r.pulse[sel]
            order = np.argsort(t_resp)
            t_mid, t_resp, pulse = t_mid[order], t_resp[order], pulse[order]
            col = None
            last_pulse = None
            for i in idx:
                if col is None:
                    hits = np.nonzero(f.motor[i] == m)[0]
                    if hits.size == 0:
                        break
                    col = int(hits[0])
                if not (f.flags[i, col] & 1):
                    continue
                p = int(round(float(profile.angle_to_pulse(m, float(f.value[i, col])))))
                moved = last_pulse is not None and p != last_pulse
                last_pulse = p
                if not moved:
                    continue
                k = np.searchsorted(t_resp, f.sensed[i], side="right") - 1
                lo = k
                while lo >= 0 and pulse[lo] != p and f.sensed[i] - t_resp[lo] < 1.5 * NS:
                    lo -= 1
                if lo >= 0 and pulse[lo] == p:
                    ages[m].append((int(f.sensed[i]) - int(t_mid[lo])) / NS)
    return {m: float(np.median(v)) for m, v in ages.items() if v}


@dataclass
class ServoFit:
    lag_s: float
    speed_rad_s: float
    rms_rad: float
    reads: int


def fit_servo(session: Session, motor: int, profile: RobotProfile = JETROVER, max_seconds: Optional[float] = 300.0) -> Optional[ServoFit]:
    """Runs the no-load servo model over the session for every (lag, speed) on the grid at once, from
    the recorded writes, and scores it at the recorded reads."""
    w = session.writes
    sel_w = w.motor == motor
    wt, wp, wd = w.t[sel_w], w.pulse[sel_w], w.extra[sel_w]
    r = session.reads
    sel_r = (r.motor == motor) & r.ok
    rt, rp = r.t_mid[sel_r], r.pulse[sel_r].astype(float)
    if wt.size < 10 or rt.size < 50:
        return None
    start = int(min(wt[0], rt[0]))
    end = int(max(wt[-1], rt[-1]))
    if max_seconds is not None:
        end = min(end, start + int(max_seconds * NS))
    ppr = profile.pulse.pulses_per_radian
    lag = LAGS_S[:, None] * np.ones((1, SPEEDS_RAD_S.size))
    vmax = (SPEEDS_RAD_S[None, :] * ppr) * np.ones((LAGS_S.size, 1))
    p = np.full(lag.shape, rp[0])
    move_from = p.copy()
    move_to = p.copy()
    move_start = start
    move_end = start
    wi = 0
    ri = int(np.searchsorted(rt, start))
    err = np.zeros(lag.shape)
    n = 0
    dt = STEP_NS / NS
    t = start
    while t <= end:
        while wi < wt.size and wt[wi] <= t:
            move_from = p.copy()
            move_to = np.full(lag.shape, wp[wi])
            move_start = int(wt[wi])
            move_end = move_start + int(wd[wi] * 1e6)
            wi += 1
        if t >= move_end:
            sp = move_to
        else:
            u = (t - move_start) / max(1, move_end - move_start)
            sp = move_from + (move_to - move_from) * u
        v = np.clip((sp - p) / lag, -vmax, vmax)
        p = p + v * dt
        while ri < rt.size and rt[ri] <= t:
            err += (p - rp[ri]) ** 2
            n += 1
            ri += 1
        t += STEP_NS
    if n == 0:
        return None
    k = np.unravel_index(int(np.argmin(err)), err.shape)
    # The servo's own limit only shows when a move asks for more than it can give; when the relay
    # timed every move slower (a slow slider), the error is flat along speed and no speed is fitted.
    row = err[k[0]]
    identified = bool(row[k[1]] < SPEED_IDENTIFIED * row[-1])
    return ServoFit(lag_s=float(LAGS_S[k[0]]), speed_rad_s=float(SPEEDS_RAD_S[k[1]]) if identified else float("nan"),
                    rms_rad=float(math.sqrt(err[k] / n)) / ppr, reads=n)


def _range(values: Sequence[float], lo_floor: float = 0.0) -> Tuple[float, float]:
    v = np.asarray([x for x in values if np.isfinite(x)], dtype=float)
    lo, hi = float(np.percentile(v, 10)), float(np.percentile(v, 90))
    span = max(hi - lo, 0.1 * abs(float(np.median(v))) + 1e-9)
    return max(lo_floor, lo - WIDEN * span - WIDEN * abs(lo)), hi + WIDEN * span + WIDEN * abs(hi)


def identify(session_dirs: Sequence[str], profile: RobotProfile = JETROVER, servo_seconds: float = 300.0) -> Dict[str, object]:
    sessions = [load_session(d) for d in session_dirs]
    unloaded = [s for s in sessions if s.meta.get("conditions", {}).get("load", {}).get("kind", "none") == "none"]
    out: Dict[str, object] = {"sessions": [s.id for s in sessions], "servoSessions": [s.id for s in unloaded]}
    out["bus"] = bus_timing(sessions)
    out["robothostLoopMs"] = robothost_loop_ms(sessions)
    out["pipelineDelayS"] = pipeline_delay_s(sessions, out["robothostLoopMs"])
    out["battery"] = battery(sessions)
    out["readingAgesS"] = {str(m): a for m, a in reading_ages_s(sessions, profile).items()}
    fits: Dict[int, List[ServoFit]] = {m: [] for m in ARM}
    for s in unloaded:
        for m in ARM:
            f = fit_servo(s, m, profile, max_seconds=servo_seconds)
            if f is not None:
                fits[m].append(f)
    out["servo"] = {str(m): [f.__dict__ for f in v] for m, v in fits.items()}
    lags = [f.lag_s for v in fits.values() for f in v]
    speeds = [f.speed_rad_s for v in fits.values() for f in v if np.isfinite(f.speed_rad_s)]
    bus = out["bus"]
    ranges: Dict[str, Tuple[float, float]] = {}
    if lags:
        ranges["servo_lag_s"] = _range(lags, 0.001)
        pinned = sum(1 for x in lags if x in (float(LAGS_S[0]), float(LAGS_S[-1])))
        if pinned:
            out["servoLagNote"] = (f"{pinned} of {len(lags)} servo fits are at the end of the lag grid "
                                   f"({LAGS_S[0]:g} or {LAGS_S[-1]:g} s), so the lag may lie beyond it: widen LAGS_S")
    if len(speeds) >= MIN_SPEED_FITS:
        ranges["servo_speed_rad_s"] = _range(speeds, 0.5)
    else:
        out["servoSpeedNote"] = ("no session moved faster than the servo allows, so its speed limit is not identified "
                                 "and the simulator keeps its default range; record some fast-slider sessions")
    # relay-replay-servo's parameters (one servo model for every joint, as the C# class has).
    servo: Dict[str, float] = {}
    if lags:
        servo["servoLagSeconds"] = float(np.median(lags))
    if len(speeds) >= MIN_SPEED_FITS:
        servo["servoMaxSpeedRadPerSec"] = float(np.median(speeds))
    if np.isfinite(out["pipelineDelayS"]):
        servo["servoPipelineDelaySeconds"] = out["pipelineDelayS"]
    out["relayReplayServo"] = servo
    if np.isfinite(bus["read_ms"]):
        ranges["read_ms"] = (max(1.0, bus["read_ms"] - 2 * bus["read_jitter_ms"]), bus["read_ms"] + 2 * bus["read_jitter_ms"])
        ranges["read_jitter_ms"] = (0.5 * bus["read_jitter_ms"], 1.5 * bus["read_jitter_ms"] + 0.1)
        ranges["read_miss_probability"] = (0.0, max(1e-5, 2.0 * bus["read_miss_probability"]))
        ranges["board_silence_per_hour"] = (0.0, max(0.05, 2.0 * bus["board_silence_per_hour"]))
    if np.isfinite(out["robothostLoopMs"]):
        ranges["robothost_loop_ms"] = (0.98 * out["robothostLoopMs"], 1.02 * out["robothostLoopMs"])
    bat = out["battery"]
    if np.isfinite(bat["start_mv"]):
        ranges["battery_start_mv"] = (bat["start_mv"] - 300.0, min(12600.0, bat["start_mv"] + 300.0))
        ranges["battery_drop_mv_per_min"] = (max(0.0, 0.5 * bat["drop_mv_per_min"]), 1.5 * bat["drop_mv_per_min"] + 1.0)
    out["ranges"] = {k: [round(a, 6), round(b, 6)] for k, (a, b) in ranges.items()}
    return out


def write(result: Dict[str, object], out_dir: str) -> Tuple[str, str]:
    os.makedirs(out_dir, exist_ok=True)
    js = os.path.join(out_dir, "sysid.json")
    with open(js, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    ages = result.get("readingAgesS", {})
    if all(str(m) in ages for m in ARM):
        with open(os.path.join(out_dir, "prior.json"), "w", encoding="utf-8") as f:
            json.dump({"sensedAgeMotorIds": list(ARM), "sensedAgeMs": [round(ages[str(m)] * 1000.0, 2) for m in ARM],
                       "relayReplayServo": result.get("relayReplayServo", {}), "from": result["sessions"]}, f, indent=2)
    tm = os.path.join(out_dir, "sysid.toml")
    with open(tm, "w", encoding="utf-8", newline="\n") as f:
        f.write("# From `armpred sysid` on " + ", ".join(result["sessions"]) + "\n")
        f.write("# Paste into a sim-pretrain config to draw the simulator around the robot.\n[ranges]\n")
        for k, (a, b) in result["ranges"].items():
            f.write(f"{k} = [{a}, {b}]\n")
    return js, tm
