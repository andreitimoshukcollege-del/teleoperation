"""The simulated FR3 Duo, headset to arm and back (docs/adr/0028 decision 3), writing session v2.

The JetRover's simulator (``robot.py``) models a relay, a driver and hobby servos. An FR3 has none of those:
its 48 Hz targets are shaped by the repo's own ``JointTrajectoryShaper`` and tracked by Franka's joint
controller. In the order the real code does it:

- **The headset** renders at about 90 Hz. On a frame it sends a joint command if 1/48 s has passed (every
  motor, both arms and both hands, with no speed) and a keepalive every 0.5 s. Every command also goes to
  ``operator.jlog``, lost or not.
- **The uplink** is the JetRover simulator's: a LAN link, then for a live-impaired session the robot side's
  impairment (one Core profile, or a schedule).
- **RobotHost** (``SerialChainHost``) runs a loop every 1-6 ms. Keepalives feed the streamer's echo. Commands
  go through ``SerialChainPlant``: the staleness check, then the soft-limit and 0..1 clamps. The loop
  advances the arm to now and sends a frame every 20 ms while subscribed.
- **The arm** (``Fr3SimBackend``): the shaper at 1 ms, an exact copy pinned by golden files (``baselines/
  franka_replay.py``). Behind it, Franka's joint impedance tracking, the guessed part:
  - a second-order lag per joint (natural frequency and damping drawn per session);
  - a payload bias: Franka compensates the gravity of the load it was told about, so an unknown payload
    pulls joint j off its target by tau_j / K_j. tau_j is the payload's weight at the TCP, taken about
    joint j's axis through the profile's forward kinematics, and K_j is the impedance stiffness;
  - each reading carries a little seeded noise.
- **The truth** is the tracked position without noise, recorded every 4 ms as ``QS``, as RobotHost does.

The arm's tracking dynamics are invented. ``franka-replay`` is exact up to them and the network, so a learned
model's simulated win shows only that it learned what is written here (ADR 0028 decision 5).
"""

from __future__ import annotations

import heapq
import json
import math
import os
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..baselines.franka_replay import ShapedState, TrajectoryShaperConfig, shaper_for_profile, step
from ..contract.chain_profile import SerialChainProfile, load_chain_profile, start_pose
from ..contract.jlog import JlogWriter, join_items
from ..data.schedule import Schedule, ScheduledLink, parse
from ..net.impairments import EmulatedLink, FixedDelay, UniformJitter
from ..net.profiles import ms_to_ticks

NS = 1_000_000_000
MS = 1_000_000
G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))
FR3_PROFILE = os.path.join(os.path.dirname(os.path.dirname(HERE)), "contract", "profiles", "fr3-duo.json")

HEADSET_FRAME_NS = int(NS / 90)
COMMAND_INTERVAL_NS = int(NS / 48.0)
KEEPALIVE_NS = NS // 2
FRAME_INTERVAL_NS = int(NS / 50.0)
SUBSCRIPTION_TIMEOUT_NS = 3 * NS
STEP_NS = MS
TRUTH_EVERY_STEPS = 4
PAYLOAD_EVERY_STEPS = 10  # the payload's torque changes with the pose slowly: recomputed every 10 ms
PASSTHROUGH_PROFILES = ("lan", "none")

# Franka's default joint impedance stiffness (libfranka's examples), N m / rad, joints 1-7.
FR3_STIFFNESS = (600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0)
# The operator's speed regimes: the fastest a joint is asked to move on a reach (rad/s). "varied" changes it.
TRAIN_SPEEDS = (0.3, 0.6, 1.0, 1.6)
HELD_OUT_SPEED = 1.3


@dataclass(frozen=True)
class Fr3SimRanges:
    track_hz: Tuple[float, float] = (8.0, 30.0)
    track_damping: Tuple[float, float] = (0.7, 1.2)
    stiffness_scale: Tuple[float, float] = (0.5, 2.0)
    noise_rad: Tuple[float, float] = (0.0, 2e-5)
    robothost_loop_ms: Tuple[float, float] = (1.0, 6.0)
    lan_one_way_ms: Tuple[float, float] = (0.8, 4.0)
    lan_jitter_ms: Tuple[float, float] = (0.2, 2.0)
    operator_clock_drift_ppm: Tuple[float, float] = (-50.0, 50.0)
    operator_uptime_h: Tuple[float, float] = (0.1, 72.0)


@dataclass(frozen=True)
class Fr3SimParams:
    """One simulated FR3 session's draw; every field ends up in ``latents.jlog`` as an ``LP`` record."""

    seed: int
    duration_s: float = 600.0
    speed: object = 1.0  # the operator's reach speed (rad/s), or "varied"
    amplitude_rad: float = 0.6
    payload_grams: float = 0.0
    payload_changes_per_min: float = 0.0  # 0: the payload is held the whole session
    uplink_profile: str = "lan"
    uplink_schedule: str = ""
    uplink_seed: int = 0
    lan_one_way_ms: float = 2.0
    lan_jitter_ms: float = 1.0
    track_hz: Tuple[float, ...] = (20.0,) * 14
    track_damping: Tuple[float, ...] = (1.0,) * 14
    stiffness_nm_per_rad: Tuple[float, ...] = FR3_STIFFNESS * 2
    noise_rad: float = 0.0
    robothost_loop_ms: float = 2.0
    robot_boot_ns: int = 3_600 * NS
    operator_clock_drift_ppm: float = 0.0
    operator_uptime_ns: int = 3_600 * NS
    operator_tps: int = NS

    def as_latent_params(self) -> Dict[str, str]:
        return {k: ",".join(str(x) for x in v) if isinstance(v, (tuple, list)) else str(v) for k, v in asdict(self).items()}


def sample_fr3_params(seed: int, ranges: Fr3SimRanges = Fr3SimRanges(), duration_s: float = 600.0, speed: Optional[object] = None,
                      payload_grams: Optional[float] = None, payload_changes_per_min: Optional[float] = None,
                      uplink_profile: str = "lan", uplink_schedule: str = "") -> Fr3SimParams:
    rng = np.random.default_rng(np.random.PCG64([seed, 0xF3]))
    if speed is None:
        speed = "varied" if rng.random() < 0.25 else float(rng.choice(TRAIN_SPEEDS))
    if payload_grams is None:
        payload_grams = float(rng.choice([0.0, 0.0, 500.0, 1000.0, 2000.0]))
    if payload_changes_per_min is None:
        payload_changes_per_min = float(rng.choice([0.0, 1.0, 2.0])) if payload_grams > 0 else 0.0
    u = lambda r: float(rng.uniform(*r))  # noqa: E731
    # One controller for every joint, a small per-joint spread: the joints are the same design.
    hz, damping, scale = u(ranges.track_hz), u(ranges.track_damping), u(ranges.stiffness_scale)
    return Fr3SimParams(
        seed=seed, duration_s=duration_s, speed=speed, amplitude_rad=float(rng.uniform(0.3, 0.9)),
        payload_grams=payload_grams, payload_changes_per_min=payload_changes_per_min,
        uplink_profile=uplink_profile, uplink_schedule=uplink_schedule, uplink_seed=int(rng.integers(1, 2**31)),
        lan_one_way_ms=u(ranges.lan_one_way_ms), lan_jitter_ms=u(ranges.lan_jitter_ms),
        track_hz=tuple(hz * float(rng.uniform(0.85, 1.15)) for _ in range(14)),
        track_damping=tuple(damping * float(rng.uniform(0.9, 1.1)) for _ in range(14)),
        stiffness_nm_per_rad=tuple(k * scale * float(rng.uniform(0.85, 1.15)) for k in FR3_STIFFNESS * 2),
        noise_rad=u(ranges.noise_rad), robothost_loop_ms=u(ranges.robothost_loop_ms),
        robot_boot_ns=int(rng.uniform(0.2, 48.0) * 3600e9),
        operator_clock_drift_ppm=u(ranges.operator_clock_drift_ppm),
        operator_uptime_ns=int(u(ranges.operator_uptime_h) * 3600e9),
    )


@dataclass
class Fr3Trajectory:
    """The operator's targets at ``RATE_HZ`` from t=0: ``q`` [n, motors] in profile order, and its events."""

    q: np.ndarray
    events: List[Tuple[float, str, str]]

    RATE_HZ = 250

    def at(self, t_s: float) -> np.ndarray:
        return self.q[min(max(int(t_s * self.RATE_HZ), 0), len(self.q) - 1)]


def _min_jerk(tau: np.ndarray) -> np.ndarray:
    tau = np.clip(tau, 0.0, 1.0)
    return tau**3 * (10.0 - 15.0 * tau + 6.0 * tau**2)


def operator_trajectory(p: Fr3SimParams, profile: SerialChainProfile, rng: np.random.Generator) -> Fr3Trajectory:
    """What the headset's IK would ask of each joint: per arm, min-jerk reaches between poses within
    ``amplitude_rad`` of the ready pose (inside the soft limits less 0.15 rad), holds, and slow traces; each hand
    opens or closes every few seconds. A reach takes the time its largest joint needs at the session's speed."""
    rate = Fr3Trajectory.RATE_HZ
    n = int(p.duration_s * rate) + 1
    motors = profile.motors
    home = np.array(start_pose(profile))
    q = np.tile(home, (n, 1)).astype(np.float64)
    events: List[Tuple[float, str, str]] = []
    col = 0
    for arm in profile.arms:
        cols = list(range(col, col + len(arm.joints)))
        lo = np.array([max(j.soft_lower + 0.15, home[c] - p.amplitude_rad) for j, c in zip(arm.joints, cols)])
        hi = np.array([min(j.soft_upper - 0.15, home[c] + p.amplitude_rad) for j, c in zip(arm.joints, cols)])
        t, pose = 0, home[cols].copy()
        speed = float(p.speed) if p.speed != "varied" else float(rng.choice(TRAIN_SPEEDS))
        while t < n:
            if p.speed == "varied" and rng.random() < 0.2:
                speed = float(rng.uniform(0.25, 1.8))
                events.append((t / rate, "slider", f"{speed:.3f}"))
            r = rng.random()
            if r < 0.15:  # hold
                length = int(rng.uniform(0.3, 2.0) * rate)
                q[t:t + length, cols] = pose
            elif r < 0.3:  # a slow trace: a sinusoid on a few joints about the current pose
                length = int(rng.uniform(2.0, 6.0) * rate)
                tt = np.arange(min(length, n - t)) / rate
                trace = np.tile(pose, (len(tt), 1))
                for k in rng.choice(len(cols), size=int(rng.integers(1, 4)), replace=False):
                    amp = float(rng.uniform(0.05, 0.25))
                    w = 2 * math.pi * float(rng.uniform(0.1, 0.5))
                    trace[:, k] = np.clip(pose[k] + amp * np.sin(w * tt), lo[k], hi[k])
                q[t:t + len(tt), cols] = trace
                pose = trace[-1].copy() if len(tt) else pose
            else:  # a min-jerk reach
                goal = lo + (hi - lo) * rng.random(len(cols))
                distance = float(np.max(np.abs(goal - pose)))
                # A min-jerk reach peaks at 1.875x its mean speed.
                length = max(int(rate * 1.875 * distance / max(speed, 1e-3) * float(rng.uniform(1.0, 1.5))), int(0.2 * rate))
                tau = _min_jerk(np.arange(min(length, n - t)) / length)
                q[t:t + len(tau), cols] = pose + np.outer(tau, goal - pose)
                pose = goal if len(tau) == length else q[t + len(tau) - 1, cols].copy()
            t += length
        col += len(arm.joints)
        if arm.gripper is not None:
            k, t, state = col, 0, 1.0
            while t < n:
                length = int(rng.uniform(3.0, 15.0) * rate)
                q[t:t + length, k] = state
                t += length
                state = 0.0 if state > 0.5 else float(rng.choice([1.0, 0.6]))
            col += 1
    assert col == len(motors)
    return Fr3Trajectory(q.astype(np.float32), events)


class _Fr3Sim:
    def __init__(self, p: Fr3SimParams, profile: SerialChainProfile):
        self.p = p
        self.profile = profile
        self.rng = np.random.default_rng(np.random.PCG64(p.seed))
        self.motors = profile.motors
        self.hand = [m in profile.hand_motors for m in self.motors]
        self.shaper: TrajectoryShaperConfig = shaper_for_profile(profile)
        assert [j.motor_id for j in self.shaper.joints] == self.motors
        self.start = p.robot_boot_ns
        self.end = self.start + int(p.duration_s * NS)
        self.traj = operator_trajectory(p, profile, self.rng)
        self.robothost: List[Tuple] = []
        self.operator: List[Tuple] = []
        self.latents: List[Tuple] = []
        self._events: List[Tuple[int, int, str, object]] = []
        self._seq = 0
        lan = [FixedDelay(ms_to_ticks(p.lan_one_way_ms, NS)), UniformJitter(ms_to_ticks(p.lan_jitter_ms, NS))]
        self.lan = EmulatedLink(lan, p.uplink_seed)
        self.uplink_schedule: Optional[Schedule] = None
        if p.uplink_schedule:
            self.uplink_schedule = Schedule.from_json(p.uplink_schedule)
        elif p.uplink_profile not in PASSTHROUGH_PROFILES:
            self.uplink_schedule = parse(f"{p.uplink_profile}@0")
        self.uplink_link = ScheduledLink(self.uplink_schedule, p.uplink_seed + 1, NS, None) if self.uplink_schedule is not None else None
        self.inbox: List[Tuple[int, int, str, object]] = []
        # The headset
        self.op_seq = 0
        self.last_cmd_op: Optional[int] = None
        self.last_keepalive_op: Optional[int] = None
        # RobotHost
        self.last_capture: Optional[int] = None
        self.subscribed_at: Optional[int] = None
        self.echo_op = self.echo_recv = 0
        self.last_frame_ns: Optional[int] = None
        self.frame_seq = 0
        self.saw_sender = False
        self.lo = [0.0 if h else j.soft_lower for h, j in zip(self.hand, self._limits())]
        self.hi = [1.0 if h else j.soft_upper for h, j in zip(self.hand, self._limits())]
        # The arm
        home = start_pose(profile)
        self.shaped = [ShapedState.at_rest(v) for v in home]
        self.target = list(home)
        self.tracked = np.array(home, dtype=np.float64)
        self.tracked_v = np.zeros(len(home))
        self.bias = np.zeros(len(home))
        self.steps = 0
        self.step_t = self.start
        self.arm_cols = [[i for i, m in enumerate(self.motors) if profile.arm_of(m) == a and not self.hand[i]] for a in range(len(profile.arms))]
        self.payload_kg = p.payload_grams / 1000.0
        self.payload_on = self.payload_kg > 0
        self.payload_changes = self._draw_payload_changes()

    def _limits(self):
        out = []
        for arm in self.profile.arms:
            out += list(arm.joints)
            if arm.gripper is not None:
                out.append(None)
        return [j if j is not None else _Hand() for j in out]

    def _draw_payload_changes(self) -> List[int]:
        rate = self.p.payload_changes_per_min
        if rate <= 0 or self.payload_kg <= 0:
            return []
        out, t = [], self.start + int(self.rng.exponential(60.0 / rate) * NS)
        while t < self.end:
            out.append(t)
            t += int(max(5.0, self.rng.exponential(60.0 / rate)) * NS)
        return out

    def _push(self, t: int, kind: str, payload: object = None) -> None:
        self._seq += 1
        heapq.heappush(self._events, (int(t), self._seq, kind, payload))

    def op_time(self, t_robot: int) -> int:
        return self.p.operator_uptime_ns + int((t_robot - self.start) * (1.0 + self.p.operator_clock_drift_ppm * 1e-6))

    # -- the arm ---------------------------------------------------------------------------------

    def _payload_bias(self) -> None:
        """Each arm joint's offset under the payload it was not told about: tau_j / K_j (zero without one)."""
        self.bias[:] = 0.0
        if not self.payload_on:
            return
        force = np.array([0.0, 0.0, -self.payload_kg * G])
        for a, arm in enumerate(self.profile.arms):
            cols = self.arm_cols[a]
            origins, axes, tcp = arm.frames(self.tracked[cols])
            for k, c in enumerate(cols):
                torque = float(axes[k] @ np.cross(tcp - origins[k], force))
                self.bias[c] = torque / self.p.stiffness_nm_per_rad[a * 7 + k if k < 7 else k]

    def advance(self, t: int) -> None:
        dt = STEP_NS / NS
        omega = [2 * math.pi * h for h in self.p.track_hz]
        joint_index = {c: n for n, c in enumerate(c for cols in self.arm_cols for c in cols)}
        while self.step_t + STEP_NS <= t:
            if self.steps % PAYLOAD_EVERY_STEPS == 0:
                self._payload_bias()
            for i in range(len(self.motors)):
                step(self.shaped[i], self.target[i], self.shaper.joints[i], self.shaper)
                if self.hand[i]:
                    self.tracked[i] = self.shaped[i].q
                    continue
                n = joint_index[i]
                w, z = omega[n], self.p.track_damping[n]
                reference = self.shaped[i].q + self.bias[i]
                accel = w * w * (reference - self.tracked[i]) - 2 * z * w * self.tracked_v[i]
                self.tracked_v[i] += accel * dt
                self.tracked[i] += self.tracked_v[i] * dt
            self.step_t += STEP_NS
            self.steps += 1
            if self.steps % TRUTH_EVERY_STEPS == 0:
                self._truth = (self.step_t, self.tracked.copy())
                self._pending_truth.append(self._truth)

    # -- the headset -----------------------------------------------------------------------------

    def headset_frame(self, t: int) -> None:
        op_now = self.op_time(t)
        if self.last_keepalive_op is None or op_now - self.last_keepalive_op >= KEEPALIVE_NS:
            self.last_keepalive_op = op_now
            self._send_uplink(t, "KA", (op_now,))
        if self.last_cmd_op is None or op_now - self.last_cmd_op >= COMMAND_INTERVAL_NS:
            self.last_cmd_op = op_now
            q = self.traj.at((t - self.start) / NS)
            targets = [(m, float(np.float32(q[i])), 0.0) for i, m in enumerate(self.motors)]
            self._send_uplink(t, "JC", (self.op_seq, op_now, targets))
            self.operator.append(("OC", op_now, self.op_seq, op_now, join_items(targets)))
            self.op_seq += 1
        self._push(t + HEADSET_FRAME_NS + int(self.rng.normal(0, 0.4 * MS)), "headset")

    def _send_uplink(self, t: int, kind: str, payload: tuple) -> None:
        ok, at_socket = self.lan.transmit_one(t)
        if not ok:
            return
        deliver = at_socket
        if self.uplink_link is not None:
            ok, deliver, k = self.uplink_link.transmit_one(at_socket, (at_socket - self.start) / NS)
            if not ok:
                return
            segment = self.uplink_schedule.segments[k]
            if segment.named and segment.name in PASSTHROUGH_PROFILES:
                deliver = at_socket
            deliver = max(deliver, at_socket)
        self._seq += 1
        heapq.heappush(self.inbox, (deliver, self._seq, kind, (at_socket, payload)))

    # -- RobotHost -------------------------------------------------------------------------------

    def robothost_loop(self, t: int) -> None:
        self._pending_truth: List[Tuple[int, np.ndarray]] = []
        while self.inbox and self.inbox[0][0] <= t:
            deliver, _, kind, (at_socket, payload) = heapq.heappop(self.inbox)
            t_socket = t if self.uplink_link is None else min(t, at_socket + int(self.rng.uniform(0.0, self.p.robothost_loop_ms) * MS))
            if kind == "KA":
                (op_ticks,) = payload
                self.robothost.append(("KA", t, op_ticks, self.p.operator_tps))
                if self.subscribed_at is None:
                    self.robothost.append(("SS", t, "operator:6002"))
                self.subscribed_at = t
                self.echo_op, self.echo_recv = op_ticks, t
                continue
            seq, capture, targets = payload
            if not self.saw_sender:
                self.robothost.append(("JS", t, "operator:6002"))
                self.saw_sender = True
            # The backend is advanced to the command's arrival before its targets take over (Fr3SimBackend.Send).
            self.advance(t)
            if self.last_capture is not None and capture <= self.last_capture:
                outcome = "S"
            else:
                outcome = "A"
                self.last_capture = capture
            self.robothost.append(("JC", t_socket, t, seq, capture, outcome, join_items(targets)))
            if outcome == "A":
                sent = []
                for i, (m, value, _) in enumerate(targets):
                    k = self.motors.index(m)
                    v = float(np.float32(min(max(value, self.lo[k]), self.hi[k])))
                    self.target[k] = v
                    sent.append((m, v))
                self.robothost.append(("AO", t, join_items(sent)))
        self.advance(t)
        for sensed, values in self._pending_truth:
            self.robothost.append(("QS", t, sensed, join_items((m, float(v)) for m, v in zip(self.motors, values))))
        if (self.subscribed_at is not None and t - self.subscribed_at <= SUBSCRIPTION_TIMEOUT_NS
                and (self.last_frame_ns is None or t - self.last_frame_ns >= FRAME_INTERVAL_NS)):
            noise = self.rng.normal(0.0, self.p.noise_rad, len(self.motors)) if self.p.noise_rad > 0 else np.zeros(len(self.motors))
            joints = [(m, 1, float(np.float32(self.tracked[i] + (0.0 if self.hand[i] else noise[i])))) for i, m in enumerate(self.motors)]
            self.robothost.append(("FR", t, self.frame_seq, t, self.step_t, self.echo_op, self.echo_recv, join_items(joints)))
            self.frame_seq += 1
            self.last_frame_ns = t
        loop = int(self.p.robothost_loop_ms * MS + self.rng.normal(0, 0.15 * MS))
        self._push(t + max(MS // 2, loop), "robothost")

    def run(self) -> "_Fr3Sim":
        self._pending_truth = []
        self._push(self.start + int(self.rng.uniform(0, HEADSET_FRAME_NS)), "headset")
        self._push(self.start + int(self.rng.uniform(0, 5 * MS)), "robothost")
        for ev_t, kind, detail in self.traj.events:
            self.latents.append(("EV", self.start + int(ev_t * NS), kind, detail))
        if self.payload_on:
            self.latents.append(("EV", self.start, "payload", repr(self.p.payload_grams)))
        for at in self.payload_changes:
            self._push(at, "payload_change")
        if self.uplink_schedule is not None:
            for seg in self.uplink_schedule.segments:
                at = self.start + int(seg.start_s * NS)
                if at < self.end:
                    name = "none" if seg.named and seg.name in PASSTHROUGH_PROFILES else seg.name
                    self._push(at, "uplink_change", name)
                    self.latents.append(("EV", at, "uplink", name))
        while self._events:
            t, _, kind, payload = heapq.heappop(self._events)
            if t >= self.end:
                break
            if kind == "headset":
                self.headset_frame(t)
            elif kind == "robothost":
                self.robothost_loop(t)
            elif kind == "payload_change":
                self.payload_on = not self.payload_on
                self.latents.append(("EV", t, "payload", repr(self.p.payload_grams if self.payload_on else 0.0)))
            elif kind == "uplink_change":
                self.robothost.append(("UP", t, payload, self.p.uplink_seed + 1))
        self.latents.sort(key=lambda r: r[1])
        return self


class _Hand:
    soft_lower, soft_upper = 0.0, 1.0


def simulate_fr3_session(params: Fr3SimParams, profile: Optional[SerialChainProfile] = None) -> _Fr3Sim:
    """One session in memory; deterministic for a given ``params`` (its seed included)."""
    return _Fr3Sim(params, profile or load_chain_profile(FR3_PROFILE)).run()


def write_fr3_session(sim: _Fr3Sim, directory: str, profile_path: str = FR3_PROFILE) -> str:
    """Writes a simulated FR3 session as session v2 (contract/SESSION_FORMAT.md), with ``source: sim``."""
    p = sim.p
    session_id = f"fr3sim-{p.seed:010d}"
    os.makedirs(directory, exist_ok=True)
    with JlogWriter(os.path.join(directory, "robothost.jlog"), "robothost", NS, "CLOCK_MONOTONIC", session_id) as w:
        for rec in sim.robothost:
            w.record(*rec)
    with JlogWriter(os.path.join(directory, "operator.jlog"), "operator", p.operator_tps, "OPERATOR", session_id) as w:
        for rec in sim.operator:
            w.record(*rec)
    with JlogWriter(os.path.join(directory, "latents.jlog"), "latents", NS, "CLOCK_MONOTONIC", session_id) as w:
        for key, value in p.as_latent_params().items():
            w.record("LP", key, value)
        for rec in sim.latents:
            w.record(*rec)
    with open(profile_path, "rb") as f:
        profile_bytes = f.read()
    with open(os.path.join(directory, "robot-profile.json"), "wb") as f:
        f.write(profile_bytes)
    import hashlib

    meta = {
        "format": "teleop-session", "version": 2, "id": session_id, "source": "sim", "label": f"fr3 sim seed {p.seed}",
        "conditions": {
            "speedSliderRadPerSec": p.speed,
            "load": {"kind": "payload" if p.payload_grams else "none", "payloadGrams": p.payload_grams},
            "network": {"uplinkProfile": "schedule" if p.uplink_schedule else p.uplink_profile, "uplinkSeed": p.uplink_seed,
                        "addedDelayMs": 0},
            "motion": "sim", "operator": "sim",
        },
        "builds": {}, "robotProfile": json.loads(profile_bytes)["name"],
        "robotProfileSha256": hashlib.sha256(profile_bytes).hexdigest(),
        "notes": "simulated FR3 Duo (armpred.sim.fr3); its arm's tracking is invented, never the robot's numbers",
    }
    with open(os.path.join(directory, "session.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(meta, f, indent=2)
        f.write("\n")
    return directory
