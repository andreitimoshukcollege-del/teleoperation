"""Copy of ``Prediction/RelayReplayServoPredictor.cs`` (relay-replay-servo), operation for operation.

relay-replay's gate and replay as a servo's setpoint c(t), with every command reaching the gate a
constant pipeline delay late and a servo's first move sized from its sensed measurement; then a
first-order lag and a speed cap, stepped forward from the anchor in double precision. Read the C#
class doc for the model. ``tests/golden/test_relay_replay_servo.py`` pins this copy to the C#.

As in ``relay_replay.py``: C# ``float`` arithmetic rounds with ``f32``; ``double`` is a Python float;
``Math.Round`` is Python's ``round``. ``math.exp`` stands in for ``Math.Exp``: both call the
platform's C library, which can differ in the last place between platforms, so the golden test
allows one float32 ULP on the outputs.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from .relay_replay import RelayReplayPredictor
from .types import JointPredictorConfig, JointSample, f32


class RelayReplayServoPredictor(RelayReplayPredictor):
    def __init__(self, config: JointPredictorConfig):
        tps = float(config.ticks_per_second)
        self._pipeline_ticks = int(round(float(config.servo_pipeline_delay_seconds) * tps))
        self._first_move_from_measured = bool(config.servo_first_move_from_measured)
        self._tps = tps
        self._step_ticks = max(1, int(round(float(config.servo_step_seconds) * tps)))
        self._step_seconds = self._step_ticks / tps
        self._lag_seconds = float(config.servo_lag_seconds)
        self._has_lag = self._lag_seconds > 0.0
        self._step_decay = math.exp(-self._step_seconds / self._lag_seconds) if self._has_lag else 0.0
        self._servo_speed = float(config.servo_max_speed_rad_per_sec)
        self._capped = not math.isinf(self._servo_speed)
        super().__init__(config)

    # -- the two input changes ------------------------------------------------------------------

    def observe_command(self, effective_ticks: int, targets: Sequence[JointSample], speed_rad_per_sec: float) -> None:
        # Judged by the effective time as given; every command reaches the gate the same delay late.
        if effective_ticks < self._last_command_ticks:
            self._rejected += 1
            return
        self._last_command_ticks = effective_ticks
        at = effective_ticks + self._pipeline_ticks
        speed_rad_per_sec = f32(speed_rad_per_sec)
        speed = speed_rad_per_sec if (speed_rad_per_sec > 0.0 and speed_rad_per_sec < self._max_move_speed) else self._max_move_speed
        for t in targets:
            g = self._gate_slot(t.motor_id)
            if g < 0:
                continue
            target = f32(t.value)
            if self._gate_instant[g]:
                self._append_move(g, at, at, target)
                continue
            if self._has_pending[g] and self._cooldown_end[g] <= at:
                self._has_pending[g] = False
                self._try_write(g, self._cooldown_end[g], self._pending_target[g], self._pending_speed[g])
            if at >= self._cooldown_end[g]:
                self._try_write(g, at, target, speed)
            else:
                if self._has_pending[g]:
                    self.coalesced += 1
                self._has_pending[g] = True
                self._pending_target[g] = target
                self._pending_speed[g] = speed
        self._accepted += 1

    def _passes(self, g: int, target: float) -> Tuple[bool, float]:
        if self._has_last_sent[g]:
            last = self._last_sent[g]
        else:
            j = self._sensed_index_of(self._gate_ids[g]) if self._first_move_from_measured else -1
            if j < 0:
                return True, 0.0
            last = self._measured[j]
        distance = abs(f32(target - last))
        return (not (distance < f32(self.config.relay_deadband_rad * self._gate_scale[g]))), distance

    def _sensed_index_of(self, motor_id: int) -> int:
        for j in range(self._joint_count):
            if self._ids[j] == motor_id and self._sensed[j]:
                return j
        return -1

    # -- the setpoint and the servo -------------------------------------------------------------

    def _predict_joint(self, j: int, end: int) -> float:
        g = self._gate_index(self._ids[j])
        if g < 0 or self._gate_instant[g]:
            return super()._predict_joint(j, end)
        segments = self._build_setpoint(j, g, end)
        return self._respond(j, g, segments, end)

    def _build_setpoint(self, j: int, g: int, end: int) -> List[List]:
        """[start, end, from, to] segments of c(t) up to ``end``, as relay-replay's replay visits them."""
        anchor = self._anchor[j]
        measured = self._measured[j]
        has_active, active_end, active_target = False, 0, 0.0
        segs: List[List] = []
        past = False
        size = self._move_size[g]
        oldest = (self._move_head[g] - size + self._capacity) % self._capacity
        base = g * self._capacity
        for k in range(size):
            slot = base + (oldest + k) % self._capacity
            start = self._move_start[slot]
            if start <= anchor:
                has_active, active_end, active_target = True, self._move_end[slot], self._move_target[slot]
                continue
            if not segs:
                segs.append(self._anchor_segment(anchor, measured, has_active, active_end, active_target))
            if start > end:
                past = True
                break
            segs.append([start, self._move_end[slot], _at(segs[-1], start), self._move_target[slot]])
        if not past and self._has_pending[g] and self._cooldown_end[g] <= end:
            at = self._cooldown_end[g]
            passes, distance = self._passes(g, self._pending_target[g])
            if passes:
                stop = at + self._move_ticks(g, distance, self._pending_speed[g])
                if at <= anchor:
                    has_active, active_end, active_target = True, stop, self._pending_target[g]
                else:
                    if not segs:
                        segs.append(self._anchor_segment(anchor, measured, has_active, active_end, active_target))
                    segs.append([at, stop, _at(segs[-1], at), self._pending_target[g]])
        if not segs:
            segs.append(self._anchor_segment(anchor, measured, has_active, active_end, active_target))
        return segs

    @staticmethod
    def _anchor_segment(anchor: int, measured: float, has_active: bool, active_end: int, active_target: float) -> List:
        if has_active and active_end > anchor:
            return [anchor, active_end, measured, active_target]
        return [anchor, anchor, measured, measured]

    def _respond(self, j: int, g: int, segs: List[List], end: int) -> float:
        t = self._anchor[j]
        p = float(self._measured[j])
        speed = self._servo_speed * float(self._gate_scale[g])
        step_limit = speed * self._step_seconds
        s = 0
        n = len(segs)
        for _ in range((end - t) // self._step_ticks):
            t += self._step_ticks
            while s + 1 < n and segs[s + 1][0] <= t:
                s += 1
            p = self._step(p, _at(segs[s], t), self._step_decay, step_limit)
        if end > t:
            dt = (end - t) / self._tps
            while s + 1 < n and segs[s + 1][0] <= end:
                s += 1
            decay = math.exp(-dt / self._lag_seconds) if self._has_lag else 0.0
            p = self._step(p, _at(segs[s], end), decay, speed * dt)
        return f32(p)

    def _step(self, p: float, c: float, decay: float, limit: float) -> float:
        lagged = c + (p - c) * decay if self._has_lag else c
        if self._capped:
            step = lagged - p
            if step > limit:
                return p + limit
            if step < -limit:
                return p - limit
        return lagged


def _at(seg: List, t: int) -> float:
    """A segment's setpoint at ``t``, in relay-replay's float arithmetic (JointReplay.At)."""
    start, stop, frm, to = seg
    if t >= stop:
        return to
    if t <= start:
        return frm
    u = float(t - start) / float(stop - start)
    return f32(frm + f32(f32(to - frm) * u))
