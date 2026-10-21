"""Copy of ``Prediction/RelayReplayPredictor.cs`` (relay-replay), operation for operation.

Read the C# class doc for what the model is; this file only reproduces it. Where C# computes in
``float`` the copy rounds with ``f32``; where it computes in ``double`` (move durations, the
replay's interpolation fraction) the copy uses Python floats; ``Math.Round`` is banker's rounding,
like Python's ``round``. ``tests/golden/test_relay_replay.py`` pins it to the C# original.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from .types import LONG_MIN, JointPredictorConfig, JointSample, PredictorDiagnostics, f32


class _JointReplay:
    __slots__ = ("anchor", "measured", "end", "anchored", "has_active", "active_end", "active_target",
                 "seg_start", "seg_end", "seg_from", "seg_to")

    def __init__(self, anchor: int, measured: float, end: int):
        self.anchor = anchor
        self.measured = measured
        self.end = end
        self.anchored = False
        self.has_active = False
        self.active_end = 0
        self.active_target = 0.0
        self.seg_start = anchor
        self.seg_end = anchor
        self.seg_from = measured
        self.seg_to = measured

    def visit(self, start: int, stop: int, target: float) -> bool:
        if start <= self.anchor:
            self.has_active = True
            self.active_end = stop
            self.active_target = target
            return True
        if not self.anchored:
            self._anchor_now()
        if start > self.end:
            return False
        frm = self._at(start)
        self.seg_start = start
        self.seg_end = stop
        self.seg_from = frm
        self.seg_to = target
        return True

    def finish(self) -> float:
        if not self.anchored:
            self._anchor_now()
        return self._at(self.end)

    def _anchor_now(self) -> None:
        self.anchored = True
        if self.has_active and self.active_end > self.anchor:
            self.seg_start = self.anchor
            self.seg_end = self.active_end
            self.seg_from = self.measured
            self.seg_to = self.active_target
        else:
            self.seg_start = self.anchor
            self.seg_end = self.anchor
            self.seg_from = self.measured
            self.seg_to = self.measured

    def _at(self, t: int) -> float:
        if t >= self.seg_end:
            return self.seg_to
        if t <= self.seg_start:
            return self.seg_from
        u = float(t - self.seg_start) / float(self.seg_end - self.seg_start)
        # C#: _segFrom + (float)((_segTo - _segFrom) * u): a float difference, a double product,
        # cast to float, then a float sum.
        return f32(self.seg_from + f32(f32(self.seg_to - self.seg_from) * u))


class RelayReplayPredictor:
    def __init__(self, config: JointPredictorConfig):
        self.config = config
        self._max_joints = config.max_joints
        self._capacity = config.command_history_capacity
        self._min_move_ticks = int(round(config.relay_min_move_seconds * float(config.ticks_per_second)))
        self._max_move_speed = config.relay_move_speed_rad_per_sec
        m, cap = self._max_joints, self._capacity
        self._ids = [0] * m
        self._measured = [0.0] * m
        self._anchor = [0] * m
        self._sensed = [False] * m
        self._gate_ids = [0] * m
        self._gate_instant = [False] * m
        self._gate_scale = [1.0] * m
        self._last_sent = [0.0] * m
        self._has_last_sent = [False] * m
        self._cooldown_end = [LONG_MIN] * m
        self._has_pending = [False] * m
        self._pending_target = [0.0] * m
        self._pending_speed = [0.0] * m
        self._move_start = [0] * (m * cap)
        self._move_end = [0] * (m * cap)
        self._move_target = [0.0] * (m * cap)
        self._move_head = [0] * m
        self._move_size = [0] * m
        self.reset()

    # -- public surface (ICommandAwarePredictor) ------------------------------------------------

    def reset(self) -> None:
        self._joint_count = 0
        self._has_state = False
        self._state_ticks = 0
        self._gate_count = 0
        self._last_command_ticks = LONG_MIN
        self._accepted = 0
        self._rejected = 0
        self._last_horizon = 0
        self.written = 0
        self.deadband_drops = 0
        self.coalesced = 0

    @property
    def diagnostics(self) -> PredictorDiagnostics:
        return PredictorDiagnostics(self._last_horizon, self._state_ticks if self._has_state else 0,
                                    self._accepted, self._rejected)

    @property
    def has_state(self) -> bool:
        return self._has_state

    @property
    def state_ticks(self) -> int:
        return self._state_ticks

    def observe_command(self, effective_ticks: int, targets: Sequence[JointSample], speed_rad_per_sec: float) -> None:
        if effective_ticks < self._last_command_ticks:
            self._rejected += 1
            return
        self._last_command_ticks = effective_ticks
        speed_rad_per_sec = f32(speed_rad_per_sec)
        speed = speed_rad_per_sec if (speed_rad_per_sec > 0.0 and speed_rad_per_sec < self._max_move_speed) else self._max_move_speed
        for t in targets:
            g = self._gate_slot(t.motor_id)
            if g < 0:
                continue
            target = f32(t.value)
            if self._gate_instant[g]:
                self._append_move(g, effective_ticks, effective_ticks, target)
                continue
            if self._has_pending[g] and self._cooldown_end[g] <= effective_ticks:
                self._has_pending[g] = False
                self._try_write(g, self._cooldown_end[g], self._pending_target[g], self._pending_speed[g])
            if effective_ticks >= self._cooldown_end[g]:
                self._try_write(g, effective_ticks, target, speed)
            else:
                if self._has_pending[g]:
                    self.coalesced += 1
                self._has_pending[g] = True
                self._pending_target[g] = target
                self._pending_speed[g] = speed
        self._accepted += 1

    def observe_state(self, sensed_ticks: int, joints: Sequence[JointSample]) -> None:
        if self._has_state and sensed_ticks < self._state_ticks:
            self._rejected += 1
            return
        carry = self._has_state
        carried: List[JointSample] = []
        if carry:
            horizon = self._last_horizon
            _, carried = self._predict_all(sensed_ticks)
            self._last_horizon = horizon
        n = min(len(joints), self._max_joints)
        scratch: List[Tuple[int, float, int, bool]] = []
        for i in range(n):
            mid = joints[i].motor_id
            instant = self._is_instant(mid)
            sensed = joints[i].sensed
            value = f32(joints[i].value)
            anchor = sensed_ticks
            if sensed and not instant:
                anchor -= self._sensed_age_ticks(mid)
            elif carry and not instant:
                previous = self._index_of(mid)
                if previous >= 0:
                    value = carried[previous].value
            scratch.append((mid, value, anchor, sensed))
        for i, (mid, value, anchor, sensed) in enumerate(scratch):
            self._ids[i] = mid
            self._measured[i] = value
            self._anchor[i] = anchor
            self._sensed[i] = sensed
        self._joint_count = n
        self._state_ticks = sensed_ticks
        self._has_state = True
        self._accepted += 1

    def try_predict(self, target_ticks: int) -> Tuple[bool, List[JointSample]]:
        """(False, []) until a state has been observed; otherwise one sample per measured joint, in
        the order last observed. (C#'s output span is sized by the caller; the copy returns all.)"""
        return self._predict_all(target_ticks)

    # -- internals -----------------------------------------------------------------------------

    def _predict_all(self, target_ticks: int) -> Tuple[bool, List[JointSample]]:
        if not self._has_state:
            return False, []
        end = target_ticks
        if end < self._state_ticks:
            end = self._state_ticks
        if end > self._state_ticks + self.config.max_horizon_ticks:
            end = self._state_ticks + self.config.max_horizon_ticks
        self._last_horizon = end - self._state_ticks
        out = [JointSample(self._ids[j], self._predict_joint(j, end), self._sensed[j]) for j in range(self._joint_count)]
        return True, out

    def _predict_joint(self, j: int, end: int) -> float:
        g = self._gate_index(self._ids[j])
        if g < 0:
            return self._measured[j]
        size = self._move_size[g]
        oldest = (self._move_head[g] - size + self._capacity) % self._capacity
        base = g * self._capacity
        if self._gate_instant[g]:
            value = self._measured[j]
            for k in range(size):
                slot = base + (oldest + k) % self._capacity
                if self._move_start[slot] > end:
                    break
                value = self._move_target[slot]
            return value
        replay = _JointReplay(self._anchor[j], self._measured[j], end)
        for k in range(size):
            slot = base + (oldest + k) % self._capacity
            if not replay.visit(self._move_start[slot], self._move_end[slot], self._move_target[slot]):
                return replay.finish()
        if self._has_pending[g] and self._cooldown_end[g] <= end:
            at = self._cooldown_end[g]
            passes, distance = self._passes(g, self._pending_target[g])
            if passes:
                replay.visit(at, at + self._move_ticks(g, distance, self._pending_speed[g]), self._pending_target[g])
        return replay.finish()

    def _try_write(self, g: int, at: int, target: float, speed: float) -> None:
        passes, distance = self._passes(g, target)
        if not passes:
            self.deadband_drops += 1
            return
        stop = at + self._move_ticks(g, distance, speed)
        self._append_move(g, at, stop, target)
        self._last_sent[g] = target
        self._has_last_sent[g] = True
        self._cooldown_end[g] = stop
        self.written += 1

    def _passes(self, g: int, target: float) -> Tuple[bool, float]:
        if not self._has_last_sent[g]:
            return True, 0.0
        distance = abs(f32(target - self._last_sent[g]))
        return (not (distance < f32(self.config.relay_deadband_rad * self._gate_scale[g]))), distance

    def _move_ticks(self, g: int, distance: float, speed: float) -> int:
        # C#: (long)Math.Round(distance / ((double)speed * _gateScale[g]) * TicksPerSecond)
        ticks = int(round(distance / (float(speed) * float(self._gate_scale[g])) * float(self.config.ticks_per_second)))
        return ticks if ticks > self._min_move_ticks else self._min_move_ticks

    def _append_move(self, g: int, start: int, stop: int, target: float) -> None:
        slot = g * self._capacity + self._move_head[g]
        self._move_start[slot] = start
        self._move_end[slot] = stop
        self._move_target[slot] = target
        self._move_head[g] = (self._move_head[g] + 1) % self._capacity
        if self._move_size[g] < self._capacity:
            self._move_size[g] += 1

    def _gate_slot(self, motor_id: int) -> int:
        g = self._gate_index(motor_id)
        if g >= 0 or self._gate_count == self._max_joints:
            return g
        g = self._gate_count
        self._gate_count += 1
        self._gate_ids[g] = motor_id
        self._gate_instant[g] = self._is_instant(motor_id)
        self._gate_scale[g] = self._speed_scale(motor_id)
        self._has_last_sent[g] = False
        self._last_sent[g] = 0.0
        self._cooldown_end[g] = LONG_MIN
        self._has_pending[g] = False
        self._move_head[g] = 0
        self._move_size[g] = 0
        return g

    def _gate_index(self, motor_id: int) -> int:
        for g in range(self._gate_count):
            if self._gate_ids[g] == motor_id:
                return g
        return -1

    def _index_of(self, motor_id: int) -> int:
        for j in range(self._joint_count):
            if self._ids[j] == motor_id:
                return j
        return -1

    def _speed_scale(self, motor_id: int) -> float:
        for mid, scale in zip(self.config.scaled_motor_ids, self.config.speed_scales):
            if mid == motor_id:
                return scale
        return 1.0

    def _sensed_age_ticks(self, motor_id: int) -> int:
        for mid, age in zip(self.config.sensed_age_motor_ids, self.config.sensed_age_seconds):
            if mid == motor_id:
                return int(round(float(age) * float(self.config.ticks_per_second)))
        return 0

    def _is_instant(self, motor_id: int) -> bool:
        return motor_id in self.config.instant_motor_ids

    # -- for features ---------------------------------------------------------------------------

    def gate_state(self, motor_id: int) -> Optional[Tuple[int, float, bool]]:
        """(cooldown end, last written target, pending) for a gated joint, or None."""
        g = self._gate_index(motor_id)
        if g < 0:
            return None
        return self._cooldown_end[g], self._last_sent[g], self._has_pending[g]
