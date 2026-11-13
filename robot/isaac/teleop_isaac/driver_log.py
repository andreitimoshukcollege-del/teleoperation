"""The JetRover driver's half of an Isaac session (docs/adr/0035 N4): ``driver.jlog``, as the real ROS driver writes it
(``robot/ros/.../session_log.py``; trainer/contract/SESSION_FORMAT.md), from the bridge that stands in for that driver.

The trainer's truth for the JetRover is the driver's servo reads (``RD``); without them ``armpred check-session``
refuses a session, and an Isaac session could never be training data. The bridge already emulates the driver's feedback
cycle (``servo.FeedbackEmulator``: motors read back to back, ``read_seconds`` each, every ``cycle_seconds``), so the log is
what that emulation did:

- ``RD tReq tResp motor pulse``: each emulated read. The bridge samples the joint at the step it takes the reading, so
  that step is the read's end (``tResp``), and ``tReq`` is ``read_seconds`` before it: the midpoint the trainer scores at
  is half a read (1.45 ms) before the sample. Every value RobotHost then takes as feedback (``RI``) is one of these, which
  is what the trainer's clock-causality check asks;
- ``CY t tEnd``: each feedback cycle, from its first read's ``tReq`` to its last read's ``tResp``;
- the driver's own records, from :class:`servo.DriverEmulator` (docs/adr/0034 amendment 13), as the real driver writes
  them: ``CI t targets`` when a command array reaches the gate (after the single thread has let it through),
  ``GP t motor pulse`` and ``GD t motor pulse`` for the gate's decisions, ``WR t durationMs positions`` per board packet
  with its commanded duration, and ``TQ t on`` per torque switch.

With ``--driver-gate off`` (the bridge before amendment 13) there is no gate: ``CI`` per command array as it arrives,
``WR`` one per servo moved, with the ramp's duration at the servo's own speed (:meth:`DriverLog.moved`), and no ``GP``,
``GD`` or ``TQ``. No battery (``BV``) either way. Everything is ``time.monotonic_ns()``, CLOCK_MONOTONIC, RobotHost's clock, and the
log follows RobotHost's control file like ``isaac.jlog`` (``isaac_session.IsaacSession`` with ``writer="driver"``), never
blocking the physics loop.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple


class DriverLog:
    """``session``: anything with ``record(tag, *fields)`` (an ``IsaacSession`` opened with ``writer="driver"``)."""

    def __init__(self, session, read_seconds: float, cycle_motors: Sequence[int]):
        self.session = session
        self.read_ns = int(round(read_seconds * 1e9))
        self.cycle_motors = list(cycle_motors)
        self._cycle_start: Optional[int] = None
        self._seen: List[int] = []

    def driver(self, tag: str, t_ns: int, *fields) -> None:
        """One of the driver's own records (``CI``, ``GP``, ``GD``, ``WR``, ``TQ``), from :class:`servo.DriverEmulator`."""
        self.session.record(tag, t_ns, *fields)

    def moved(self, t_ns: int, motor: int, target: float, from_pulse: float, servo_pps: float) -> None:
        """The bridge started a servo's ramp from ``from_pulse`` to ``target`` at ``servo_pps``."""
        duration_ms = int(abs(target - from_pulse) / servo_pps * 1000.0) if servo_pps > 0 else 0
        self.session.record("WR", t_ns, duration_ms, [(int(motor), int(round(target)))])

    def read(self, t_ns: int, motor: int, pulse: float) -> None:
        """The bridge took a motor's reading at ``t_ns`` (the read's end)."""
        t_req = t_ns - self.read_ns
        self.session.record("RD", t_req, t_ns, int(motor), int(round(pulse)))
        if not self._seen:
            self._cycle_start = t_req
        self._seen.append(int(motor))
        if len(self._seen) >= len(self.cycle_motors):
            self.session.record("CY", self._cycle_start, t_ns)
            self._seen = []
            self._cycle_start = None
