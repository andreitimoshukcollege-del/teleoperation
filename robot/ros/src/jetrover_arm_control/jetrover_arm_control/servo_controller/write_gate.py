"""The servo driver's write gate, as plain Python: no serial port, no ROS, no clock of its own.

`ServoController` drives the real servos through it, and the Isaac JetRover's bridge (teleoperation's
robot/isaac/teleop_isaac/servo.py, docs/adr/0034 amendment 13) imports this same module, so the simulated driver
gates its servos exactly as the real one does and the two cannot drift. Keep it importable without anything beyond
the standard library.

What it decides, per servo (see `ServoController`'s class doc for the history):

- a target within `MIN_MEANINGFUL_PULSE_DELTA` of the last pulse sent is dropped (`GD`);
- a target that arrives while the servo is still cooling down from its last move is kept as its pending target, the
  newest winning (`GP`), and written by `flush` once the cooldown ends;
- otherwise it becomes a move whose duration is the distance from the last pulse sent at the given speed, never under
  `MIN_MOVE_DURATION_SECONDS` (a servo's first move has no distance to measure, so it takes the minimum), and the
  servo cools down for that duration.

Times are whatever monotonic seconds the caller passes; the gate never reads a clock.
"""

# The write-gate threshold, in pulse units. A bus-traffic optimization only (skip a physical
# write that wouldn't move the servo perceptibly) -- see docs/adr/0010-absolute-joint-targets-over-local-relay.md
# for why this stopped being a correctness requirement once setPos began receiving already-resolved
# absolute pulse targets instead of relative deltas.
MIN_MEANINGFUL_PULSE_DELTA = 5

# Minimum move duration regardless of distance, seconds -- avoids an implausibly fast/jerky
# instantaneous move for a near-zero delta. The move *speed* itself (pulses/second) is a per-call
# parameter (docs/adr/0011-generic-robot-arm-profiles.md), supplied by GenericArmPlant over the wire.
MIN_MOVE_DURATION_SECONDS = 0.05

# The board's pulse range; targets are clamped into it.
PULSE_MIN = 0
PULSE_MAX = 1000


def resolve(targets):
    """(servoID, pulse, pulses_per_second) as the driver receives them -> the same with the pulse truncated to an int
    and clamped to the board's range, as `ServoController.setPositions` writes them to its `CI` record."""
    resolved = []
    for servoID, direction, pulses_per_second in targets:
        nextPos = int(direction)
        nextPos = PULSE_MIN if nextPos < PULSE_MIN else PULSE_MAX if nextPos > PULSE_MAX else nextPos
        resolved.append((servoID, nextPos, pulses_per_second))
    return resolved


def packets(moves):
    """Moves [(duration, servoID, pulse)] -> board packets [(duration, [[servoID, pulse], ...])], one per distinct
    duration (a packet carries one duration for all its servos), in the order the durations first appear."""
    byDuration = {}
    for duration, servoID, nextPos in moves:
        byDuration.setdefault(duration, []).append([servoID, nextPos])
    return list(byDuration.items())


class WriteGate:
    """Per-servo write gating. `offer` and `flush` return (moves, events): moves are (duration, servoID, pulse) to
    write now, events are ("GP" | "GD", servoID, pulse) in the order they were decided, for the session log."""

    def __init__(self):
        # The last pulse actually sent per servoID (only for the gate and the next duration, not a position belief),
        # and the monotonic time after which that servo's next real write is due.
        self.lastSentPulse = {}
        self.nextAllowedWriteAt = {}
        # The newest target that arrived during a servo's cooldown: servoID -> (pulse, pulses_per_second).
        self.pending = {}

    def offer(self, resolved, now):
        """Resolved targets (see `resolve`) arriving at `now`."""
        moves, events = [], []
        for servoID, nextPos, pulses_per_second in resolved:
            if now < self.nextAllowedWriteAt.get(servoID, 0.0):
                # Still cooling down from this servo's last real move: keep it as the pending target, newest wins.
                self.pending[servoID] = (nextPos, pulses_per_second)
                events.append(("GP", servoID, nextPos))
                continue
            self.pending.pop(servoID, None)
            self._plan(servoID, nextPos, pulses_per_second, now, moves, events)
        return moves, events

    def flush(self, now):
        """Every pending target whose servo's cooldown has expired by `now`."""
        moves, events = [], []
        if not self.pending:
            return moves, events
        for servoID in list(self.pending):
            if now < self.nextAllowedWriteAt.get(servoID, 0.0):
                continue
            nextPos, pulses_per_second = self.pending.pop(servoID)
            self._plan(servoID, nextPos, pulses_per_second, now, moves, events)
        return moves, events

    def clear_pending(self):
        self.pending.clear()

    def torque_switched(self, lastReadPulse):
        """A torque switch: pending targets are dropped (one written after a torque-off could stiffen a servo again),
        the last pulse sent is forgotten and the last one read takes its place (a limp arm may have moved), and no
        servo is cooling down."""
        self.pending.clear()
        self.lastSentPulse = dict(lastReadPulse)
        self.nextAllowedWriteAt.clear()

    def _plan(self, servoID, nextPos, pulses_per_second, now, moves, events):
        lastSent = self.lastSentPulse.get(servoID)
        if lastSent is not None and abs(nextPos - lastSent) < MIN_MEANINGFUL_PULSE_DELTA:
            events.append(("GD", servoID, nextPos))
            return
        pulseDelta = abs(nextPos - lastSent) if lastSent is not None else 0
        duration = max(MIN_MOVE_DURATION_SECONDS, pulseDelta / pulses_per_second)
        # The board's firmware paces the physical move over `duration`; the next write for this servo waits for it.
        self.lastSentPulse[servoID] = nextPos
        self.nextAllowedWriteAt[servoID] = now + duration
        moves.append((duration, servoID, nextPos))
