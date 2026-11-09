"""The JetRover's servos and driver as Isaac stands in for them (docs/adr/0034 decisions 1, 3 and 4), without Isaac.

- :class:`SimModel` reads ``robot/jetrover/urdf/jetrover.sim.json`` (written by RobotHost's ``JetRoverSimModel``): each
  motor's joint, and the pulse conversion ``GenericArmPlant`` uses, so a pulse means the same angle on both sides.
- :class:`TimedSetpoint` is what a Hiwonder bus servo does with a move: it ramps its own setpoint linearly from where it is to
  the target over ``|delta| / pulses_per_second`` (the driver's ``durationMs``), and a new move starts from wherever the
  ramp had got to. The PhysX drive then follows the ramp, as the servo's motor follows its setpoint, within its torque.
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
                                                   float(m.get("fingerOpenMeters", 0.0)))

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
            return -m.finger_open + self.gripper_fraction(motor_id, pulse) * (m.finger_open + m.finger_travel)
        return (pulse - self.zero_pulse) / self.pulse_per_radian - m.zero_offset

    def pulse(self, motor_id: int, value: float) -> float:
        """The pulse a joint at ``value`` reads (the inverse of :meth:`value`)."""
        m = self.motors[motor_id]
        if m.is_gripper:
            span = m.finger_open + m.finger_travel
            f = (value + m.finger_open) / span if span else 0.0
            return m.open_pulse + min(max(f, 0.0), 1.0) * (m.closed_pulse - m.open_pulse)
        return self.zero_pulse + (value + m.zero_offset) * self.pulse_per_radian

    def gripper_fraction(self, motor_id: int, pulse: float) -> float:
        """How closed the gripper is at ``pulse``: 0 at the open pulse, 1 at the closed one."""
        m = self.motors[motor_id]
        span = m.closed_pulse - m.open_pulse
        return min(max((pulse - m.open_pulse) / span if span else 0.0, 0.0), 1.0)

    def finger_value(self, motor_id: int, fraction: float) -> float:
        """Each finger's travel for a gripper ``fraction`` (0 open, 1 closed)."""
        m = self.motors[motor_id]
        return -m.finger_open + min(max(fraction, 0.0), 1.0) * (m.finger_open + m.finger_travel)

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
        self._next_send = 0.0

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
