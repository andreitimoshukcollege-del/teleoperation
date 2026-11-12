"""The JetRover's servos and driver as Isaac stands in for them (docs/adr/0034 decisions 1, 3 and 4), without Isaac.

- :class:`SimModel` reads ``robot/jetrover/urdf/jetrover.sim.json`` (written by RobotHost's ``JetRoverSimModel``): each
  motor's joint, and the pulse conversion ``GenericArmPlant`` uses, so a pulse means the same angle on both sides.
- :class:`TimedSetpoint` is what a Hiwonder bus servo does with a move: it ramps its own setpoint linearly from where it is to
  the target over ``|delta| / pulses_per_second`` (the driver's ``durationMs``), and a new move starts from wherever the
  ramp had got to. The PhysX drive then follows the ramp, as the servo's motor follows its setpoint, within its torque.
- :class:`DriverEmulator` is the driver node between the relay and the servos (docs/adr/0034 amendment 13): its write gate,
  the real driver's own module (``write_gate.py``, imported by :func:`write_gate`), and its single thread, so a command that
  arrives during a read cycle waits for the cycle's end, and the gate's waiting targets go out between reads.
- :class:`FeedbackEmulator` is what the driver and relay report: a cycle every 20 ms reading motors 1 to 5 back to back in
  whole pulses, published when it ends; the newest reading of every motor sent every 20 ms, valid while under 0.2 s old;
  a motor that never reports (the gripper) always invalid.

The relay protocol itself is the ROS relay's own module (``robot/ros/src/teleop_relay``), imported by :func:`relay_protocol`,
so there is one Python implementation of it.
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
DEFAULT_SIM = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.sim.json")


def relay_protocol():
    """The ROS relay's protocol module (relay protocol v5), the one Python copy of it."""
    path = os.path.join(REPO, "robot", "ros", "src", "teleop_relay")
    if path not in sys.path:
        sys.path.insert(0, path)
    from teleop_relay import relay_protocol as rp  # noqa: E402

    return rp


def write_gate():
    """The servo driver's write gate (``servo_controller/write_gate.py``), the one Python copy of it (amendment 13)."""
    path = os.path.join(REPO, "robot", "ros", "src", "jetrover_arm_control")
    if path not in sys.path:
        sys.path.insert(0, path)
    from jetrover_arm_control.servo_controller import write_gate as wg  # noqa: E402

    return wg


@dataclass(frozen=True)
class Motor:
    motor_id: int
    joint: str
    zero_offset: float
    feedback: bool
    stall_torque: float
    open_pulse: float = 0.0
    closed_pulse: float = 0.0
    finger_travel: float = 0.0
    fingers: Tuple[str, ...] = ()
    finger_open: float = 0.0  # metres each finger opens past the CAD's pose at the open pulse (docs/adr/0034 amendment 8)
    finger_table: Tuple[float, ...] = ()  # each finger's travel at 0, 0.05, ..., 1 of the closing: the twin's four-bar (amendment 10)

    @property
    def is_gripper(self) -> bool:
        return bool(self.fingers)


class SimModel:
    def __init__(self, doc: Dict):
        if doc.get("format") != "jetrover-sim" or doc.get("version") != 1:
            raise ValueError("not a jetrover-sim version 1 sidecar")
        self.zero_pulse = float(doc["zeroPulse"])
        self.pulse_per_radian = float(doc["pulsePerRadian"])
        self.min_pulse = float(doc["minPulse"])
        self.max_pulse = float(doc["maxPulse"])
        self.default_pps = float(doc["defaultPulsesPerSecond"])
        self.feedback_seconds = float(doc["feedbackPeriodSeconds"])
        self.driver_cycle_seconds = float(doc.get("driverCycleSeconds", 0.02))
        self.read_seconds = float(doc.get("readSeconds", 0.0029))
        self.urdf = doc["urdf"]
        self.motors: Dict[int, Motor] = {}
        for m in doc["motors"]:
            self.motors[int(m["motorId"])] = Motor(int(m["motorId"]), m["joint"], float(m["zeroOffsetRadians"]), bool(m["feedback"]),
                                                   float(m["stallTorqueNm"]), float(m.get("openPulse", 0.0)), float(m.get("closedPulse", 0.0)),
                                                   float(m.get("fingerTravelMeters", 0.0)), tuple(m.get("fingers", ())),
                                                   float(m.get("fingerOpenMeters", 0.0)), tuple(float(v) for v in m.get("fingerTravelByFraction", ())))

    @staticmethod
    def load(path: str = DEFAULT_SIM) -> "SimModel":
        with open(path, encoding="utf-8") as f:
            return SimModel(json.load(f))

    def value(self, motor_id: int, pulse: float) -> float:
        """The joint's value for a pulse: radians, as ``GenericArmPlant`` converts (less the mounting offset); for the gripper,
        each finger's closing travel in metres from the CAD's pose, from its open (``-finger_open``) to its closed pulse
        (``finger_travel``)."""
        m = self.motors[motor_id]
        if m.is_gripper:
            return self.finger_value(motor_id, self.gripper_fraction(motor_id, pulse))
        return (pulse - self.zero_pulse) / self.pulse_per_radian - m.zero_offset

    def pulse(self, motor_id: int, value: float) -> float:
        """The pulse a joint at ``value`` reads (the inverse of :meth:`value`)."""
        m = self.motors[motor_id]
        if m.is_gripper:
            return m.open_pulse + self.finger_fraction(motor_id, value) * (m.closed_pulse - m.open_pulse)
        return self.zero_pulse + (value + m.zero_offset) * self.pulse_per_radian

    def gripper_fraction(self, motor_id: int, pulse: float) -> float:
        """How closed the gripper is at ``pulse``: 0 at the open pulse, 1 at the closed one."""
        m = self.motors[motor_id]
        span = m.closed_pulse - m.open_pulse
        return min(max((pulse - m.open_pulse) / span if span else 0.0, 0.0), 1.0)

    def finger_value(self, motor_id: int, fraction: float) -> float:
        """Each finger's travel for a gripper ``fraction`` (0 open, 1 closed): the twin's table when the sidecar has one,
        else linear from ``-finger_open`` to ``finger_travel``."""
        m = self.motors[motor_id]
        f = min(max(fraction, 0.0), 1.0)
        t = m.finger_table
        if len(t) > 1:
            x = f * (len(t) - 1)
            i = min(int(x), len(t) - 2)
            return t[i] + (x - i) * (t[i + 1] - t[i])
        return -m.finger_open + f * (m.finger_open + m.finger_travel)

    def finger_fraction(self, motor_id: int, travel: float) -> float:
        """The gripper's closing (0..1) for each finger's ``travel``: :meth:`finger_value` inverted."""
        m = self.motors[motor_id]
        t = m.finger_table
        if len(t) > 1:
            if travel <= t[0]:
                return 0.0
            for i in range(len(t) - 1):
                if travel <= t[i + 1]:
                    span = t[i + 1] - t[i]
                    return (i + ((travel - t[i]) / span if span else 0.0)) / (len(t) - 1)
            return 1.0
        span = m.finger_open + m.finger_travel
        return min(max((travel + m.finger_open) / span if span else 0.0, 0.0), 1.0)

    def clamp(self, pulse: float) -> float:
        return min(max(pulse, self.min_pulse), self.max_pulse)


class TimedSetpoint:
    """One servo's own setpoint: a linear ramp to each move's target over ``|delta| / pulses_per_second``."""

    def __init__(self, pulse: float):
        self.pulse = float(pulse)
        self.target = float(pulse)
        self._rate = 0.0  # pulses per second, signed

    def move(self, target: float, pulses_per_second: float) -> None:
        self.target = float(target)
        delta = self.target - self.pulse
        speed = abs(pulses_per_second) if pulses_per_second and pulses_per_second > 0 else 0.0
        if speed <= 0.0 or delta == 0.0:
            self.pulse = self.target if speed <= 0.0 else self.pulse  # no speed given: the servo jumps its setpoint
            self._rate = 0.0
            return
        self._rate = speed if delta > 0 else -speed

    def move_over(self, target: float, seconds: float) -> None:
        """A board move packet: from wherever the ramp stands to ``target`` in ``seconds`` (the packet's duration)."""
        self.target = float(target)
        delta = self.target - self.pulse
        if delta == 0.0 or seconds <= 0.0:
            self.pulse = self.target
            self._rate = 0.0
            return
        self._rate = delta / seconds

    def hold(self, pulse: float) -> None:
        """Where the servo is now becomes its setpoint and target (torque coming back on holds the arm where it lies)."""
        self.pulse = self.target = float(pulse)
        self._rate = 0.0

    def step(self, seconds: float) -> float:
        if self._rate != 0.0:
            nxt = self.pulse + self._rate * seconds
            if (self._rate > 0 and nxt >= self.target) or (self._rate < 0 and nxt <= self.target):
                nxt, self._rate = self.target, 0.0
            self.pulse = nxt
        return self.pulse


class FeedbackEmulator:
    """The driver's feedback cycles and the relay's periodic feedback datagram, as the real ones run (since 2026-10-25):

    - every ``cycle_seconds`` (the driver's ``pollRate``, 20 ms) a cycle reads motors 1 to 5 back to back, ``read_seconds``
      each (2.9 ms, the trainer's fitted bus read), in whole pulses;
    - a cycle's readings are published when it ends, and only then seen by the relay;
    - the relay sends the newest reading of every motor every ``feedback_seconds`` (its ``_FEEDBACK_SEND_PERIOD_SECONDS``,
      20 ms), valid while under ``valid_seconds`` old (0.2 s); a motor that never reports (the gripper) always invalid.
    """

    def __init__(self, model: SimModel, read_seconds: Optional[float] = None, cycle_seconds: Optional[float] = None, valid_seconds: float = 0.2):
        self.model = model
        self.read_seconds = model.read_seconds if read_seconds is None else read_seconds
        self.cycle_seconds = model.driver_cycle_seconds if cycle_seconds is None else cycle_seconds
        self.valid_seconds = valid_seconds
        self.order: List[int] = sorted(m.motor_id for m in model.motors.values() if m.feedback)
        self.readings: Dict[int, Tuple[float, float]] = {}  # motor -> (pulse, time read), as the relay has them
        self._pending: List[Tuple[int, float, float]] = []
        self._cycle_start = 0.0
        self._next_read = 0.0
        self._turn = 0
        self._busy_until = 0.0
        self._next_send = 0.0

    def busy(self, now: float) -> bool:
        """Whether a read cycle is running at ``now``: from its first read until its last one has ended. The real driver's
        single thread runs nothing else then (:class:`DriverEmulator`)."""
        return self._turn != 0 or now < self._busy_until

    def due_read(self, now: float) -> Optional[int]:
        """The motor to read now, or None: a cycle's reads back to back, a cycle every ``cycle_seconds``."""
        if not self.order or now < self._next_read:
            return None
        if self._turn == 0:
            self._cycle_start = now
        m = self.order[self._turn]
        self._turn += 1
        if self._turn < len(self.order):
            self._next_read = now + self.read_seconds
        else:
            self._turn = 0
            self._busy_until = now + self.read_seconds
            self._next_read = max(self._cycle_start + self.cycle_seconds, now + self.read_seconds)
        return m

    def record(self, motor_id: int, pulse: float, now: float) -> None:
        self._pending.append((motor_id, float(round(pulse)), now))  # the bus reports whole pulses
        if self._turn == 0:  # the cycle's last read: it publishes, read_seconds after the read began
            for mid, p, t in self._pending:
                self.readings[mid] = (p, t)
            self._pending.clear()

    def due_feedback(self, now: float) -> Optional[List[Tuple[int, bool, float]]]:
        """The feedback datagram's entries when one is due, else None. Every motor of the model, in motor order."""
        if now < self._next_send:
            return None
        self._next_send = now + self.model.feedback_seconds
        out = []
        for mid in sorted(self.model.motors):
            r = self.readings.get(mid)
            valid = self.model.motors[mid].feedback and r is not None and now - r[1] < self.valid_seconds
            out.append((mid, valid, r[0] if valid else 0.0))
        return out


class RecordedFeedback(FeedbackEmulator):
    """Feedback cycles at the times a real session's driver ran them (its ``driver.jlog``: ``CY`` windows and the ``RD``
    reads that answered), for replaying that session (docs/adr/0034 amendment 13). With the driver's single thread in front
    of the gate, a few milliseconds of cycle phase decide which targets are written, so a replay that runs cycles of its own
    would gate a different stream of moves than the robot did.

    ``cycles``: ``[(start, end, [(t, motor), ...]), ...]`` in the replay's seconds, sorted. A cycle's readings are published
    at its last read, as the driver publishes them; the relay's feedback datagram is :class:`FeedbackEmulator`'s."""

    def __init__(self, model: SimModel, cycles, valid_seconds: float = 0.2):
        super().__init__(model, valid_seconds=valid_seconds)
        self.cycles = [(float(a), float(b), [(float(t), int(m)) for t, m in reads]) for a, b, reads in cycles]
        self._ci = 0       # the cycle whose reads are next
        self._ri = 0       # the next read within it
        self._bi = 0       # the first cycle not yet ended, for busy()
        self._publish_next = False

    EPS = 1e-9  # a recorded time that falls on the caller's step counts at that step

    def busy(self, now: float) -> bool:
        while self._bi < len(self.cycles) and self.cycles[self._bi][1] <= now + self.EPS:
            self._bi += 1
        return self._bi < len(self.cycles) and self.cycles[self._bi][0] <= now + self.EPS

    def due_read(self, now: float) -> Optional[int]:
        while self._ci < len(self.cycles):
            reads = self.cycles[self._ci][2]
            if self._ri < len(reads):
                t, m = reads[self._ri]
                if t > now + self.EPS:
                    return None
                self._ri += 1
                self._publish_next = self._ri == len(reads)
                return m
            self._ci, self._ri = self._ci + 1, 0
        return None

    def record(self, motor_id: int, pulse: float, now: float) -> None:
        self._pending.append((motor_id, float(round(pulse)), now))
        if self._publish_next:
            for mid, p, t in self._pending:
                self.readings[mid] = (p, t)
            self._pending.clear()
            self._publish_next = False


class DriverEmulator:
    """The JetRover's servo driver node as the real one runs it (``robot_controller_node.py`` and ``ServoController``,
    docs/adr/0034 amendment 13), in front of any simulated servos.

    - **One thread.** Command arrays and torque switches that arrive while a read cycle runs wait for its end, in order,
      the command subscription keeping its newest :attr:`COMMAND_DEPTH` and the torque one its newest :attr:`TORQUE_DEPTH`,
      as rclpy's single-threaded executor and the node's queue depths do.
    - **The write gate** is the driver's own (:func:`write_gate`): a servo takes a new move only after its last one's
      duration, the newest waiting target wins and is written between reads, and a target under 5 pulses from the last one
      written is dropped. Each board packet starts its servos' moves over its duration, in whole milliseconds as the packet
      encodes it (``servo_move(motor, pulse, seconds)``).
    - **A torque switch** drops the waiting targets and times the next move from the last reading, every time the relay
      publishes one, as the driver does; ``servo_torque(on)`` is called for each.
    - ``gate=False`` is the bridge before amendment 13: every target goes straight to ``servo_command(motor, pulse, pps)``
      as it arrives.

    ``log(tag, *fields)``, when given, receives the driver's own records (``CI``, ``GP``, ``GD``, ``WR``, ``TQ``,
    ``trainer/contract/SESSION_FORMAT.md``) stamped by ``clock_ns()``. Times passed in are the caller's seconds.
    """

    COMMAND_DEPTH = 3
    TORQUE_DEPTH = 1

    def __init__(self, model: SimModel, feedback: "FeedbackEmulator", servo_move=None, servo_command=None, servo_torque=None,
                 gate: bool = True, log=None, clock_ns=None):
        self.model = model
        self.feedback = feedback
        self.gated = gate
        self._move = servo_move
        self._command = servo_command
        self._torque = servo_torque
        self._log = log
        self._clock_ns = clock_ns
        self._wg = write_gate() if gate else None
        self.gate = self._wg.WriteGate() if gate else None
        self._queue: List[Tuple[str, object]] = []
        self.last_read: Dict[int, int] = {}
        self.packets = 0

    def _record(self, tag: str, *fields) -> None:
        if self._log is not None:
            self._log(tag, self._clock_ns(), *fields)

    def command(self, now: float, targets: Sequence[Tuple[int, float, float]]) -> None:
        """A relay command array arrived: ``(motor, pulse, pulses per second)`` per servo."""
        if not self.gated:
            items = [(int(m), self.model.clamp(p), float(pps)) for m, p, pps in targets]
            self._record("CI", [(m, int(p), pps) for m, p, pps in items])
            for m, p, pps in items:
                self._command(m, p, pps)
            return
        self._enqueue("command", list(targets), self.COMMAND_DEPTH)
        self.tick(now)

    def torque(self, now: float, on: bool) -> None:
        if not self.gated:
            self._record("TQ", 1 if on else 0)
            self._torque(bool(on))
            return
        self._enqueue("torque", bool(on), self.TORQUE_DEPTH)
        self.tick(now)

    def _enqueue(self, kind: str, payload, depth: int) -> None:
        self._queue.append((kind, payload))
        mine = [i for i, (k, _) in enumerate(self._queue) if k == kind]
        if len(mine) > depth:
            del self._queue[mine[0]]  # a full subscription queue drops its oldest message

    def tick(self, now: float) -> None:
        """Runs whatever waited for the thread, once no read cycle is running."""
        if not self.gated or not self._queue or self.feedback.busy(now):
            return
        queue, self._queue = self._queue, []
        for kind, payload in queue:
            if kind == "command":
                resolved = [(m, int(self.model.clamp(p)), pps) for m, p, pps in self._wg.resolve(payload)]
                self._record("CI", [(m, p, float(pps)) for m, p, pps in resolved])
                moves, events = self.gate.offer(resolved, now)
                self._events(events)
                self._write(moves)
            else:
                self.gate.torque_switched(self.last_read)
                self._torque(payload)
                self._record("TQ", 1 if payload else 0)

    def due_read(self, now: float) -> Optional[int]:
        """The motor to read now, or None. Before each read the gate's waiting targets whose cooldown is over go out."""
        self.tick(now)
        mid = self.feedback.due_read(now)
        if mid is not None and self.gated:
            moves, events = self.gate.flush(now)
            self._events(events)
            self._write(moves)
        return mid

    def record(self, motor_id: int, pulse: float, now: float) -> None:
        self.feedback.record(motor_id, pulse, now)
        self.last_read[motor_id] = int(round(pulse))

    def _events(self, events) -> None:
        for tag, motor, pulse in events:
            self._record(tag, motor, pulse)

    def _write(self, moves) -> None:
        for duration, positions in self._wg.packets(moves):
            ms = int(duration * 1000)
            self._record("WR", ms, [(m, p) for m, p in positions])
            self.packets += 1
            for m, p in positions:
                self._move(m, float(p), ms / 1000.0)


def home_pulses(model: SimModel, home: Sequence[float]) -> Dict[int, float]:
    """Pulses for the collision model's home pose (yaw, proximal, distal, wrist, roll), gripper open."""
    joints = ("base_yaw", "proximal", "distal", "wrist", "roll")
    out = {}
    for mid, m in model.motors.items():
        if m.is_gripper:
            out[mid] = m.open_pulse
        else:
            out[mid] = model.pulse(mid, home[joints.index(m.joint)])
    return out
