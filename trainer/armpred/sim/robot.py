"""The simulated pipeline, headset to servo and back, as a discrete-event simulation.

What happens, in the order the real code does it (file and function named for each):

- **The headset** (Bridge ``JetRoverOperatorBridge``/``JetRoverJointStateLink``) renders at about
  90 Hz; on a frame it sends a joint command if 1/48 s has passed since the last, and a keepalive
  every 0.5 s. Commands carry the slider speed for every joint.
- **The uplink** delivers both through a LAN link and, when the session is "live impaired", the
  robot side's impairment (``UplinkImpairedReceiver``): one Core profile, or a schedule that changes
  mid-session (ADR 0021 decision 13), each change logged as ``UP``. ``JC`` gets ``t`` and
  ``tDeliver``. Every command the headset sends is also written to ``operator.jlog``, lost or not.
- **RobotHost** (``Program.cs``) runs a loop every ~5 ms: it reads what has arrived (keepalives feed
  the streamer's echo; commands go through ``GenericArmPlant.CommandJointAngles``: the staleness
  check, pulse conversion, the step clamp, ``RequestedPulsesPerSecond``), polls relay feedback, and
  sends a ``JointStateFrame`` every 20 ms while subscribed.
- **The relay** (``relay_node.py``) publishes the newest command datagram every 20 ms and sends the
  newest reading per motor every 50 ms, valid if under 0.2 s old.
- **The driver** (``robot_controller_node.py``, ``servo_controller.py``) is single-threaded: a
  feedback cycle reads motors 1-5 in turn (flushing pending targets between reads, waiting 3 ms
  after a write), and queued commands (depth 3) run between cycles through the write gate.
- **The servos** (the guessed part, ``SimParams``): a timed linear setpoint from the servo's
  position to the target; the position follows it with a lag, a speed limit scaled by battery and
  load, and droop under load (payload gravity, hand resistance).
- **Reads** sample the position somewhere inside the bus transaction, quantised to whole pulses;
  some miss and cost the 1 s timeout; a board silence makes every read miss for a while.
"""

from __future__ import annotations

import heapq
import json
import math
import os
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from ..contract.jlog import JlogWriter, join_items
from ..contract.profile import JETROVER, RobotProfile
from ..data.schedule import Schedule, ScheduledLink, parse
from ..net.impairments import EmulatedLink, FixedDelay, UniformJitter
from ..net.profiles import ms_to_ticks
from . import operator as op_motion
from .params import ARM_MOTORS, GRIPPER_MOTOR, SimParams

NS = 1_000_000_000
MS = 1_000_000
PHYSICS_STEP_NS = 1 * MS
G = 9.81

# Masses and lever arms for payload gravity (guesses; sysid's job is the servo response, not these).
LINK1_KG, LINK2_KG, WRIST_KG = 0.08, 0.12, 0.15
WRIST_TO_COM_M, WRIST_TO_PAYLOAD_M = 0.05, 0.10

# Robot-side uplink profiles that impair nothing (UplinkImpairedReceiver passes them through).
PASSTHROUGH_PROFILES = ("lan", "none")

# Real timing constants, from the code that has them.
HEADSET_FRAME_NS = int(NS / 90)
COMMAND_INTERVAL_NS = int(NS / 48.0)
KEEPALIVE_NS = NS // 2
FRAME_INTERVAL_NS = int(NS / 50.0)  # JointStateStreamer: (long)(ticksPerSecond / rateHz)
SUBSCRIPTION_TIMEOUT_NS = 3 * NS
RELAY_POLL_NS = 20 * MS
RELAY_FEEDBACK_NS = 50 * MS
RELAY_STALE_NS = 200 * MS
DRIVER_TIMER_NS = 20 * MS
DRIVER_QUEUE_DEPTH = 3
WRITE_SETTLE_NS = 3 * MS
READ_TIMEOUT_NS = 1 * NS
MIN_MOVE_S = 0.05
MIN_MEANINGFUL_PULSE_DELTA = 5
STEP_SIZE_PULSES, MAX_DIRECTION = 50.0, 20.0


@dataclass
class _Servo:
    pulse: float  # position, in pulse units (float)
    move_from: float = 0.0
    move_to: float = 0.0
    move_start: int = 0
    move_end: int = 0

    def setpoint(self, t: int) -> float:
        if t >= self.move_end:
            return self.move_to
        if t <= self.move_start:
            return self.move_from
        u = (t - self.move_start) / (self.move_end - self.move_start)
        return self.move_from + (self.move_to - self.move_from) * u


@dataclass
class _HandEvent:
    start: int
    end: int
    ramp: int
    motors: Tuple[int, ...]
    torque_nm: float

    def torque(self, t: int) -> float:
        if t < self.start or t > self.end:
            return 0.0
        ramp_in = min(1.0, (t - self.start) / self.ramp)
        ramp_out = min(1.0, (self.end - t) / self.ramp)
        return self.torque_nm * min(ramp_in, ramp_out)


@dataclass
class SimOutput:
    params: SimParams
    robothost: List[Tuple] = field(default_factory=list)
    driver: List[Tuple] = field(default_factory=list)
    operator: List[Tuple] = field(default_factory=list)
    latents: List[Tuple] = field(default_factory=list)
    start_ns: int = 0
    end_ns: int = 0


class _Sim:
    def __init__(self, params: SimParams, profile: RobotProfile):
        self.p = params
        self.profile = profile
        self.rng = np.random.default_rng(np.random.PCG64(params.seed))
        self.out = SimOutput(params=params)
        self.ppr = profile.pulse.pulses_per_radian
        self.start = params.robot_boot_ns
        self.end = self.start + int(params.duration_s * NS)
        self.out.start_ns, self.out.end_ns = self.start, self.end
        self.traj = op_motion.generate(params, self.rng)
        self._events: List[Tuple[int, int, str, object]] = []
        self._seq = 0
        # Profile order, as RobotHost reads and sends joints.
        self.joint_order = [j.motor_id for j in profile.joints]
        # Uplink: the LAN, then (live-impaired sessions) the robot side's impairment, a schedule (one
        # Core profile is a one-segment schedule).
        lan = [FixedDelay(ms_to_ticks(params.lan_one_way_ms, NS)), UniformJitter(ms_to_ticks(params.lan_jitter_ms, NS))]
        self.lan = EmulatedLink(lan, params.uplink_seed)
        self.uplink_schedule: Optional[Schedule] = None
        if params.uplink_schedule:
            self.uplink_schedule = Schedule.from_json(params.uplink_schedule)
        elif params.uplink_profile not in PASSTHROUGH_PROFILES:
            self.uplink_schedule = parse(f"{params.uplink_profile}@0")
        self.uplink_link = (ScheduledLink(self.uplink_schedule, params.uplink_seed + 1, NS, None)
                            if self.uplink_schedule is not None else None)
        # Headset state
        self.op_seq = 0
        self.last_cmd_op: Optional[int] = None
        self.last_keepalive_op = None
        # RobotHost state
        self.inbox: List[Tuple[int, int, str, object]] = []  # (deliver, seq, kind, payload)
        self.target_pulse = {m: float(self._initial_pulse(m)) for m in self.joint_order}
        self.sensed_pulse: Dict[int, float] = {}
        self.last_feedback_ns = 0
        self.last_accepted_capture = None
        self.subscribed_at = None
        self.echo_op = 0
        self.echo_recv = 0
        self.last_frame_ns = None
        self.frame_seq = 0
        self.saw_sender = False
        self.relay_socket: Optional[Tuple[int, list]] = None  # newest RO datagram (t, targets)
        self.relay_socket_seen = None
        self.rh_feedback_inbox: List[Tuple[int, list]] = []
        # Relay state
        self.relay_latest: Dict[int, Tuple[int, int]] = {}  # motor -> (t, pulse)
        # Driver state
        self.driver_queue: Deque[Tuple[int, list]] = deque(maxlen=DRIVER_QUEUE_DEPTH)
        self.driver_busy_until = self.start
        self.next_timer = self.start + int(self.rng.uniform(0, DRIVER_TIMER_NS))
        self.last_sent_pulse: Dict[int, int] = {}
        self.next_allowed: Dict[int, int] = {}
        self.pending: Dict[int, Tuple[int, float]] = {}
        self.last_write_ns = None
        # Servos
        self.servos = {m: _Servo(pulse=float(self._initial_pulse(m))) for m in list(ARM_MOTORS) + [GRIPPER_MOTOR]}
        for s in self.servos.values():
            s.move_from = s.move_to = s.pulse
        self.physics_t = self.start
        self.hand_events = self._draw_hand_events()
        self.silences = self._draw_silences()
        self.speed_scale = {m: 1.0 for m in ARM_MOTORS}
        self.load_nm = {m: 0.0 for m in ARM_MOTORS}
        self.next_latent_ns = self.start

    # -- helpers -------------------------------------------------------------------------------

    def _initial_pulse(self, motor: int) -> float:
        if motor == GRIPPER_MOTOR:
            return self.profile.pulse.gripper_open_pulse
        return float(self.profile.angle_to_pulse(motor, self.traj.arm[0][ARM_MOTORS.index(motor)]))

    def _push(self, t: int, kind: str, payload: object = None) -> None:
        self._seq += 1
        heapq.heappush(self._events, (int(t), self._seq, kind, payload))

    def op_time(self, t_robot: int) -> int:
        """The headset's clock (ns) at robot time ``t_robot``: its own uptime, running ppm fast or slow.
        Both clocks count from their own boot, so the offset between them is hours to days."""
        elapsed = t_robot - self.start
        return self.p.operator_uptime_ns + int(elapsed * (1.0 + self.p.operator_clock_drift_ppm * 1e-6))

    def battery_mv(self, t: int) -> float:
        minutes = (t - self.start) / NS / 60.0
        return self.p.battery_start_mv - self.p.battery_drop_mv_per_min * minutes

    def _draw_hand_events(self) -> List[_HandEvent]:
        out: List[_HandEvent] = []
        rate = self.p.hand_events_per_min
        if rate <= 0:
            return out
        t = self.start + int(self.rng.exponential(60.0 / rate) * NS)
        while t < self.end:
            length = int(self.rng.uniform(0.5, 3.0) * NS)
            k = int(self.rng.integers(1, 3))
            motors = tuple(int(m) for m in self.rng.choice([1, 2, 3, 4], size=k, replace=False))
            stall = float(np.mean([self.p.stall_torque_nm[ARM_MOTORS.index(m)] for m in motors]))
            # Light, as the protocol requires; now and then enough to stall (at most 2 s).
            frac = float(self.rng.uniform(0.15, 0.6)) if self.rng.random() > 0.15 else float(self.rng.uniform(0.9, 1.2))
            if frac > 0.9:
                length = min(length, 2 * NS)
            out.append(_HandEvent(t, t + length, int(self.rng.uniform(0.1, 0.3) * NS), motors, frac * stall))
            t += length + int(self.rng.exponential(60.0 / rate) * NS)
        return out

    def _draw_silences(self) -> List[Tuple[int, int]]:
        out = []
        rate = self.p.board_silence_per_hour
        if rate <= 0:
            return out
        t = self.start + int(self.rng.exponential(3600.0 / rate) * NS)
        while t < self.end:
            length = int(self.rng.uniform(2.0, 12.0) * NS)
            out.append((t, t + length))
            t += length + int(self.rng.exponential(3600.0 / rate) * NS)
        return out

    def _silent(self, t: int) -> bool:
        return any(a <= t < b for a, b in self.silences)

    # -- servo physics -------------------------------------------------------------------------

    def _gravity_nm(self) -> Dict[int, float]:
        """Generalised gravity torque on the proximal, distal and wrist pitch joints (N m), from
        the planar kinematics of ArmKinematics.Forward plus the wrist, gripper and payload."""
        q = {m: float(self.profile.pulse_to_angle(m, self.servos[m].pulse)) for m in (2, 3, 4)}
        l1, l2 = self.profile.proximal_link_length, self.profile.distal_link_length
        a1, a2, a3 = q[2], q[2] + q[3], q[2] + q[3] + q[4]
        x_elbow = l1 * math.cos(a1)
        x_wrist = x_elbow + l2 * math.cos(a2)
        masses = [
            (LINK1_KG, 0.5 * l1 * math.cos(a1)),
            (LINK2_KG, x_elbow + 0.5 * l2 * math.cos(a2)),
            (WRIST_KG, x_wrist + WRIST_TO_COM_M * math.cos(a3)),
            (self.p.payload_grams / 1000.0, x_wrist + WRIST_TO_PAYLOAD_M * math.cos(a3)),
        ]
        pivots = {2: (0.0, 0), 3: (x_elbow, 1), 4: (x_wrist, 2)}
        out = {}
        for m, (x_joint, first) in pivots.items():
            out[m] = -G * sum(mass * (x - x_joint) for mass, x in masses[first:])
        return out

    def _advance_physics(self, t: int) -> None:
        dt_s = PHYSICS_STEP_NS / NS
        while self.physics_t + PHYSICS_STEP_NS <= t:
            now = self.physics_t + PHYSICS_STEP_NS
            gravity = self._gravity_nm()
            battery = max(0.3, self.battery_mv(now) / 12000.0)
            for m in ARM_MOTORS:
                k = ARM_MOTORS.index(m)
                s = self.servos[m]
                error = s.setpoint(now) - s.pulse
                hand = sum(e.torque(now) for e in self.hand_events if m in e.motors)
                # Hand resistance opposes the way the servo is trying to go; gravity has its own sign.
                load = gravity.get(m, 0.0) - math.copysign(hand, error) if abs(error) > 0.25 else gravity.get(m, 0.0)
                stall = self.p.stall_torque_nm[k]
                free = max(0.0, 1.0 - abs(load) / stall)
                v_lim = self.p.servo_speed_rad_s[k] * battery * free * self.ppr  # pulses/s
                droop = load / self.p.servo_stiffness_nm_per_rad[k] * self.ppr  # pulses
                v = (error + droop) / self.p.servo_lag_s[k]
                v = max(-v_lim, min(v_lim, v))
                s.pulse += v * dt_s
                self.speed_scale[m] = battery * free
                self.load_nm[m] = load
            g = self.servos[GRIPPER_MOTOR]
            g.pulse += (g.setpoint(now) - g.pulse) * min(1.0, dt_s / 0.04)
            self.physics_t = now
            if now >= self.next_latent_ns:
                for m in ARM_MOTORS:
                    k = ARM_MOTORS.index(m)
                    stalled = 1 if self.speed_scale[m] <= 0.0 else 0
                    self.out.latents.append(("LJ", now, m, round(self.speed_scale[m], 5),
                                             self.p.servo_lag_s[k], round(self.load_nm[m], 5), stalled))
                self.next_latent_ns += 100 * MS

    # -- the headset ---------------------------------------------------------------------------

    def headset_frame(self, t: int) -> None:
        op_now = self.op_time(t)
        sim_s = (t - self.start) / NS
        if self.last_keepalive_op is None or op_now - self.last_keepalive_op >= KEEPALIVE_NS:
            self.last_keepalive_op = op_now
            self._send_uplink(t, "KA", (op_now,))
        if self.last_cmd_op is None or op_now - self.last_cmd_op >= COMMAND_INTERVAL_NS:
            self.last_cmd_op = op_now
            arm, gripper, slider = self.traj.at(sim_s)
            targets = []
            for m in self.joint_order:
                if m == GRIPPER_MOTOR:
                    targets.append((m, float(np.float32(gripper)), float(np.float32(slider))))
                else:
                    targets.append((m, float(np.float32(arm[ARM_MOTORS.index(m)])), float(np.float32(slider))))
            self._send_uplink(t, "JC", (self.op_seq, op_now, targets))
            self.out.operator.append(("OC", op_now, self.op_seq, op_now, join_items(targets)))
            self.op_seq += 1
        jitter = int(self.rng.normal(0, 0.4 * MS))
        self._push(t + HEADSET_FRAME_NS + jitter, "headset")

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

    # -- RobotHost -----------------------------------------------------------------------------

    def robothost_loop(self, t: int) -> None:
        while self.inbox and self.inbox[0][0] <= t:
            deliver, _, kind, (at_socket, payload) = heapq.heappop(self.inbox)
            # UdpTransport stamps arrival when the loop reads the socket, so without a robot-side
            # impairment both of JC's times are this iteration. With one, the socket was read on an
            # earlier iteration (about when it arrived) and the emulator released it now.
            if self.uplink_link is None:
                t_socket = t
            else:
                t_socket = min(t, at_socket + int(self.rng.uniform(0.0, self.p.robothost_loop_ms) * MS))
            if kind == "KA":
                (op_ticks,) = payload
                self.out.robothost.append(("KA", t, op_ticks, self.p.operator_tps))
                if self.subscribed_at is None:
                    self.out.robothost.append(("SS", t, "operator:6002"))
                self.subscribed_at = t
                self.echo_op, self.echo_recv = op_ticks, t
            else:
                seq, capture, targets = payload
                if not self.saw_sender:
                    self.out.robothost.append(("JS", t, "operator:6002"))
                    self.saw_sender = True
                outcome = self._command(t, capture, targets)
                self.out.robothost.append(("JC", t_socket, t, seq, capture, outcome, join_items(targets)))
        # Step: take relay feedback.
        while self.rh_feedback_inbox and self.rh_feedback_inbox[0][0] <= t:
            _, entries = self.rh_feedback_inbox.pop(0)
            self.out.robothost.append(("RI", t, join_items(entries)))
            any_valid = False
            for motor, valid, pulse in entries:
                if valid:
                    self.sensed_pulse[motor] = float(pulse)
                    any_valid = True
            if any_valid:
                self.last_feedback_ns = t
        # The joint-state frame, every 20 ms while subscribed.
        if (self.subscribed_at is not None and t - self.subscribed_at <= SUBSCRIPTION_TIMEOUT_NS
                and (self.last_frame_ns is None or t - self.last_frame_ns >= FRAME_INTERVAL_NS)):
            joints = []
            for m in self.joint_order:
                j = self.profile.joint(m)
                if m in self.sensed_pulse:
                    value = (self.sensed_pulse[m] - self.profile.pulse.zero_pulse) / self.ppr - j.zero_offset_radians
                    flags = 1
                elif m == GRIPPER_MOTOR:
                    span = self.profile.pulse.gripper_closed_pulse - self.profile.pulse.gripper_open_pulse
                    value, flags = (self.target_pulse[m] - self.profile.pulse.gripper_open_pulse) / span, 0
                else:
                    value, flags = (self.target_pulse[m] - self.profile.pulse.zero_pulse) / self.ppr - j.zero_offset_radians, 0
                joints.append((m, flags, float(np.float32(value))))
            self.out.robothost.append(("FR", t, self.frame_seq, t, self.last_feedback_ns, self.echo_op, self.echo_recv,
                                       join_items(joints)))
            self.frame_seq += 1
            self.last_frame_ns = t
        loop = int(self.p.robothost_loop_ms * MS + self.rng.normal(0, 0.15 * MS))
        self._push(t + max(MS, loop), "robothost")

    def _command(self, t: int, capture: int, targets: list) -> str:
        if self.last_accepted_capture is not None and capture <= self.last_accepted_capture:
            return "S"
        self.last_accepted_capture = capture
        requested = max((s for m, _, s in targets if m != GRIPPER_MOTOR), default=0.0)
        pps = min(requested * self.ppr, self.profile.pulse.default_pulses_per_second) if requested > 0 else self.profile.pulse.default_pulses_per_second
        outgoing = []
        for m, angle, _ in targets:
            if m == GRIPPER_MOTOR:
                lo = min(self.profile.pulse.gripper_open_pulse, self.profile.pulse.gripper_closed_pulse)
                hi = max(self.profile.pulse.gripper_open_pulse, self.profile.pulse.gripper_closed_pulse)
                frac = min(max(angle, 0.0), 1.0)
                pulse = self.profile.pulse.gripper_open_pulse + frac * (self.profile.pulse.gripper_closed_pulse - self.profile.pulse.gripper_open_pulse)
                self.target_pulse[m] = min(max(pulse, lo), hi)
            else:
                raw = float(self.profile.angle_to_pulse(m, angle))
                clamped = min(max(raw, self.profile.pulse.min_pulse), self.profile.pulse.max_pulse)
                direction = max(-MAX_DIRECTION, min(MAX_DIRECTION, (clamped - self.target_pulse[m]) / STEP_SIZE_PULSES))
                self.target_pulse[m] += direction * STEP_SIZE_PULSES
            outgoing.append((m, float(np.float32(self.target_pulse[m])), float(np.float32(pps))))
        self.out.robothost.append(("RO", t, join_items(outgoing)))
        self.relay_socket = (t, outgoing)
        return "A"

    # -- the relay -----------------------------------------------------------------------------

    def relay_poll(self, t: int) -> None:
        if self.relay_socket is not None and self.relay_socket is not self.relay_socket_seen:
            self.relay_socket_seen = self.relay_socket
            arrive = t + int(self.rng.uniform(0.3, 1.5) * MS)
            self._push(arrive, "driver_cmd", list(self.relay_socket[1]))
        self._push(t + RELAY_POLL_NS, "relay_poll")

    def relay_feedback(self, t: int) -> None:
        entries = []
        for motor in sorted(self.relay_latest):
            at, pulse = self.relay_latest[motor]
            entries.append((motor, 1 if t - at <= RELAY_STALE_NS else 0, float(pulse)))
        if entries:
            self.rh_feedback_inbox.append((t + int(0.2 * MS), entries))
        self._push(t + RELAY_FEEDBACK_NS, "relay_feedback")

    # -- the driver ----------------------------------------------------------------------------

    def driver_cmd(self, t: int, targets: list) -> None:
        self.driver_queue.append((t, targets))
        self.driver_wake(t)

    def driver_wake(self, t: int) -> None:
        """One turn of the single-threaded executor. The commands that queued while it was busy run
        first, as a burst (ADR 0018 §7: "commands are handled in bursts between feedback polls"),
        then the feedback timer's cycle if it is due. A cycle is computed eagerly from its start:
        nothing but the driver touches the servos, so nothing that happens meanwhile changes it."""
        if t < self.driver_busy_until:
            return  # a wake is already scheduled at busy_until
        if self.driver_queue:
            t = self._drain_commands(t)
        if t >= self.next_timer:
            end = self._feedback_cycle(t)
            # rcl: the timer keeps its own period, skipping the periods it missed.
            while self.next_timer <= t:
                self.next_timer += DRIVER_TIMER_NS
            self.driver_busy_until = end
            self._push(end, "driver_wake")
            return
        self.driver_busy_until = t
        self._push(self.next_timer, "driver_wake")

    def _drain_commands(self, t: int) -> int:
        while self.driver_queue:
            _, targets = self.driver_queue.popleft()
            t = self._set_positions(t, targets) + int(self.rng.uniform(0.4, 1.5) * MS)
        return t

    def _set_positions(self, t: int, targets: list) -> int:
        rows, moves = [], []
        for motor, pulse, pps in targets:
            next_pos = int(pulse)  # int() truncates toward zero, like Python's int on a float
            next_pos = 0 if next_pos < 0 else 1000 if next_pos > 1000 else next_pos
            rows.append((motor, next_pos, pps))
            if t < self.next_allowed.get(motor, 0):
                self.pending[motor] = (next_pos, pps)
                self.out.driver.append(("GP", t, motor, next_pos))
                continue
            self.pending.pop(motor, None)
            self._plan(t, motor, next_pos, pps, moves)
        self.out.driver.append(("CI", t, join_items(rows)))
        self._send(t, moves)
        return t

    def _flush_pending(self, t: int) -> None:
        if not self.pending:
            return
        moves = []
        for motor in list(self.pending):
            if t < self.next_allowed.get(motor, 0):
                continue
            next_pos, pps = self.pending.pop(motor)
            self._plan(t, motor, next_pos, pps, moves)
        self._send(t, moves)

    def _plan(self, t: int, motor: int, next_pos: int, pps: float, moves: list) -> None:
        last = self.last_sent_pulse.get(motor)
        if last is not None and abs(next_pos - last) < MIN_MEANINGFUL_PULSE_DELTA:
            self.out.driver.append(("GD", t, motor, next_pos))
            return
        delta = abs(next_pos - last) if last is not None else 0
        duration = max(MIN_MOVE_S, delta / pps)
        self.last_sent_pulse[motor] = next_pos
        self.next_allowed[motor] = t + int(duration * NS)
        moves.append((duration, motor, next_pos))

    def _send(self, t: int, moves: list) -> None:
        if not moves:
            return
        by_duration: Dict[float, list] = {}
        for duration, motor, pos in moves:
            by_duration.setdefault(duration, []).append((motor, pos))
        self._advance_physics(t)
        for duration, positions in by_duration.items():
            ms = int(duration * 1000)
            self.out.driver.append(("WR", t, ms, join_items(positions)))
            for motor, pos in positions:
                s = self.servos.get(motor)
                if s is None:
                    continue
                s.move_from = s.pulse
                s.move_to = float(pos)
                s.move_start = t
                s.move_end = t + ms * MS
        self.last_write_ns = t

    def _feedback_cycle(self, t: int) -> int:
        start = t
        published = []
        for motor in ARM_MOTORS:
            self._flush_pending(t)
            if self.last_write_ns is not None and t < self.last_write_ns + WRITE_SETTLE_NS:
                t = self.last_write_ns + WRITE_SETTLE_NS
            t_req = t
            if self._silent(t_req) or self.rng.random() < self.p.read_miss_probability:
                t = t_req + READ_TIMEOUT_NS
                self.out.driver.append(("RD", t_req, t, motor, None))
                continue
            duration = max(1.5 * MS, self.rng.normal(self.p.read_ms, self.p.read_jitter_ms) * MS)
            t_sample = t_req + int(duration * self.rng.uniform(0.3, 0.7))
            self._advance_physics(t_sample)
            exact = self.servos[motor].pulse
            pulse = int(round(exact + (self.rng.integers(-1, 2) if self.rng.random() < 0.05 else 0)))
            pulse = max(0, min(1000, pulse))
            t = t_req + int(duration)
            self.out.driver.append(("RD", t_req, t, motor, pulse))
            self.out.latents.append(("TR", t_sample, motor, round(exact, 4)))
            published.append((motor, pulse))
        self.out.driver.append(("CY", start, t))
        # Published when the cycle ends, and only then seen by the relay: the cycle was computed
        # ahead of time, so it must not leak its readings into the relay before they exist.
        self._push(t + int(self.rng.uniform(0.3, 1.2) * MS), "relay_update", published)
        return t + int(0.3 * MS)

    # -- the session ---------------------------------------------------------------------------

    def run(self) -> SimOutput:
        self._push(self.start + int(self.rng.uniform(0, HEADSET_FRAME_NS)), "headset")
        self._push(self.start + int(self.rng.uniform(0, 5 * MS)), "robothost")
        self._push(self.start + int(self.rng.uniform(0, RELAY_POLL_NS)), "relay_poll")
        self._push(self.start + int(self.rng.uniform(0, RELAY_FEEDBACK_NS)), "relay_feedback")
        self._push(self.next_timer, "driver_wake")
        self._push(self.start, "battery")
        for ev_t, kind, detail in self.traj.events:
            self.out.latents.append(("EV", self.start + int(ev_t * NS), kind, detail))
        for e in self.hand_events:
            self.out.latents.append(("EV", e.start, "hand-on", ",".join(str(m) for m in e.motors) + f";{e.torque_nm:.4f}"))
            self.out.latents.append(("EV", e.end, "hand-off", ",".join(str(m) for m in e.motors)))
        for a, b in self.silences:
            self.out.latents.append(("EV", a, "board-silent", ""))
            self.out.latents.append(("EV", b, "board-back", ""))
        if self.p.payload_grams:
            self.out.latents.append(("EV", self.start, "payload", repr(self.p.payload_grams)))
        if self.uplink_schedule is not None:
            for seg in self.uplink_schedule.segments:
                at = self.start + int(seg.start_s * NS)
                if at < self.end:
                    name = "none" if seg.named and seg.name in PASSTHROUGH_PROFILES else seg.name
                    self._push(at, "uplink_change", name)
                    self.out.latents.append(("EV", at, "uplink", name))
        while self._events:
            t, _, kind, payload = heapq.heappop(self._events)
            if t >= self.end:
                break
            if kind == "headset":
                self.headset_frame(t)
            elif kind == "robothost":
                self.robothost_loop(t)
            elif kind == "relay_poll":
                self.relay_poll(t)
            elif kind == "relay_feedback":
                self.relay_feedback(t)
            elif kind == "driver_cmd":
                self.driver_cmd(t, payload)
            elif kind == "driver_wake":
                self.driver_wake(t)
            elif kind == "relay_update":
                for motor, pulse in payload:
                    self.relay_latest[motor] = (t, pulse)
            elif kind == "uplink_change":
                self.out.robothost.append(("UP", t, payload, self.p.uplink_seed + 1))
            elif kind == "battery":
                self.out.driver.append(("BV", t, int(self.battery_mv(t))))
                self._push(t + 5 * NS, "battery")
        self.out.latents.sort(key=lambda r: r[1] if isinstance(r[1], int) else 0)
        return self.out


def simulate_session(params: SimParams, profile: RobotProfile = JETROVER) -> SimOutput:
    """Runs one session in memory. Deterministic for a given ``params`` (its seed included)."""
    return _Sim(params, profile).run()


def write_session(out: SimOutput, directory: str, profile_json: Optional[dict] = None) -> str:
    """Writes a simulated session as session v1 (contract/SESSION_FORMAT.md)."""
    p = out.params
    session_id = f"sim-{p.seed:010d}"
    os.makedirs(directory, exist_ok=True)
    with JlogWriter(os.path.join(directory, "robothost.jlog"), "robothost", NS, "CLOCK_MONOTONIC", session_id) as w:
        for rec in out.robothost:
            w.record(*rec)
    driver = sorted(out.driver, key=lambda r: r[1])
    with JlogWriter(os.path.join(directory, "driver.jlog"), "driver", NS, "CLOCK_MONOTONIC", session_id) as w:
        for rec in driver:
            w.record(*rec)
    with JlogWriter(os.path.join(directory, "operator.jlog"), "operator", p.operator_tps, "OPERATOR", session_id) as w:
        for rec in out.operator:
            w.record(*rec)
    with JlogWriter(os.path.join(directory, "latents.jlog"), "latents", NS, "CLOCK_MONOTONIC", session_id) as w:
        for key, value in p.as_latent_params().items():
            w.record("LP", key, value)
        for rec in out.latents:
            w.record(*rec)
    load = {"kind": "payload" if p.payload_grams else ("hand" if p.hand_events_per_min else "none"),
            "payloadGrams": p.payload_grams}
    meta = {
        "format": "jetrover-session",
        "version": 1,
        "id": session_id,
        "source": "sim",
        "label": f"sim seed {p.seed}",
        "conditions": {
            "speedSliderRadPerSec": p.slider,
            "load": load,
            "network": {"uplinkProfile": "schedule" if p.uplink_schedule else p.uplink_profile,
                        "uplinkSeed": p.uplink_seed, "addedDelayMs": 0},
            "motion": "sim",
            "operator": "sim",
        },
        "builds": {},
        "robotProfileSha256": "",
        "notes": "simulated; never the robot's numbers",
    }
    with open(os.path.join(directory, "session.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(meta, f, indent=2)
        f.write("\n")
    if profile_json is not None:
        with open(os.path.join(directory, "robot-profile.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(profile_json, f, indent=2)
            f.write("\n")
    return directory
