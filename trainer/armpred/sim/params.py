"""Every hidden quantity the simulator draws, and the ranges it draws them from.

Ranges are deliberately wide until the pilot recordings exist; ``sysid`` narrows them by writing a
``SimRanges`` override into a config. Nothing here is a model input: these become ``latents``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

ARM_MOTORS: Tuple[int, ...] = (1, 2, 3, 4, 5)
GRIPPER_MOTOR = 10
# What the driver reads each cycle, in order: robot/ros robot_controller_node.py FEEDBACK_MOTOR_IDS. The
# gripper was off the list until 2026-10-25 (it did not answer); it answers now.
READ_MOTORS: Tuple[int, ...] = ARM_MOTORS + (GRIPPER_MOTOR,)

# The headset's speed slider values the protocol trains on; 5.0 is held out (ADR 0021 protocol).
TRAIN_SLIDER_VALUES = (1.0, 2.5, 3.8, 6.0)


@dataclass(frozen=True)
class SimRanges:
    """Uniform ranges [lo, hi] for every drawn parameter. ``sysid`` replaces these."""

    servo_speed_rad_s: Tuple[float, float] = (4.0, 8.0)  # no-load, at a full battery
    servo_lag_s: Tuple[float, float] = (0.004, 0.035)
    servo_stiffness_nm_per_rad: Tuple[float, float] = (8.0, 30.0)  # position-loop stiffness (droop)
    stall_torque_nm: Tuple[float, float] = (2.5, 3.6)  # HTD-35H: about 3.4 N m at 12 V
    # Measured on three recordings (2026-10-22, 2026-10-25): a read took 2.9 ms at the median, 2.7 at
    # p10, 3.8 at p90. The 4-11 ms guessed before any recording made the simulated driver cycle at
    # 20 Hz against the robot's 44-50.
    read_ms: Tuple[float, float] = (2.6, 3.4)
    read_jitter_ms: Tuple[float, float] = (0.2, 0.8)
    read_miss_probability: Tuple[float, float] = (0.0, 0.0005)  # each miss blinds every motor for the 1 s timeout
    board_silence_per_hour: Tuple[float, float] = (0.0, 1.0)
    battery_start_mv: Tuple[float, float] = (11400.0, 12600.0)
    battery_drop_mv_per_min: Tuple[float, float] = (2.0, 25.0)
    robothost_loop_ms: Tuple[float, float] = (5.05, 5.6)  # Thread.Sleep(5) on the Jetson
    lan_one_way_ms: Tuple[float, float] = (0.8, 4.0)
    lan_jitter_ms: Tuple[float, float] = (0.2, 2.0)
    operator_clock_drift_ppm: Tuple[float, float] = (-50.0, 50.0)
    operator_uptime_h: Tuple[float, float] = (0.1, 72.0)  # the headset's monotonic clock at session start


@dataclass(frozen=True)
class SimParams:
    """One session's draw. Every field ends up in ``latents.jlog`` as an ``LP`` record."""

    seed: int
    duration_s: float = 600.0
    # The headset
    slider: object = 2.5  # rad/s, or "varied"
    slider_values: Tuple[float, ...] = TRAIN_SLIDER_VALUES
    motion_mix: Tuple[float, float, float, float, float] = (0.40, 0.30, 0.15, 0.10, 0.05)  # free, marks, trace, reversals, hold
    operator_clock_drift_ppm: float = 0.0
    operator_uptime_ns: int = 3_600 * 1_000_000_000
    operator_tps: int = 1_000_000_000
    # Load
    payload_grams: float = 0.0
    hand_events_per_min: float = 0.0  # 0: no hand resistance in this session
    # The network the robot actually saw (ADR 0021 decisions 4 and 13: the uplink is physical here).
    # ``uplink_schedule`` (a schedule's JSON, seconds from the session start) changes mid-session and
    # wins over ``uplink_profile``, one Core profile for the whole session.
    uplink_profile: str = "lan"
    uplink_schedule: str = ""
    uplink_seed: int = 0
    lan_one_way_ms: float = 2.0
    lan_jitter_ms: float = 1.0
    # The servos, per arm motor (motors 1-5); the gripper follows its command after a fixed lag
    servo_speed_rad_s: Tuple[float, ...] = (6.0,) * 5
    servo_lag_s: Tuple[float, ...] = (0.015,) * 5
    servo_stiffness_nm_per_rad: Tuple[float, ...] = (15.0,) * 5
    stall_torque_nm: Tuple[float, ...] = (3.4,) * 5
    # The board
    read_ms: float = 2.9
    read_jitter_ms: float = 0.4
    read_miss_probability: float = 0.001
    board_silence_per_hour: float = 0.0
    battery_start_mv: float = 12300.0
    battery_drop_mv_per_min: float = 8.0
    robothost_loop_ms: float = 5.2
    robot_boot_ns: int = 3_600 * 1_000_000_000

    def as_latent_params(self) -> Dict[str, str]:
        out = {}
        for k, v in asdict(self).items():
            out[k] = ",".join(str(x) for x in v) if isinstance(v, (tuple, list)) else str(v)
        return out


def _u(rng: np.random.Generator, lo_hi: Tuple[float, float]) -> float:
    lo, hi = lo_hi
    return float(rng.uniform(lo, hi))


def sample_params(seed: int, ranges: SimRanges = SimRanges(), duration_s: float = 600.0,
                  slider: Optional[object] = None, payload_grams: Optional[float] = None,
                  hand_events_per_min: Optional[float] = None, uplink_profile: Optional[str] = None,
                  slider_values: Tuple[float, ...] = TRAIN_SLIDER_VALUES, uplink_schedule: str = "") -> SimParams:
    """One session's parameters. Anything passed explicitly is fixed (to build a session matrix
    like the real protocol's); everything else is drawn from ``ranges``."""
    rng = np.random.default_rng(np.random.PCG64(seed))
    if slider is None:
        slider = "varied" if rng.random() < 0.25 else float(rng.choice(slider_values))
    if payload_grams is None:
        payload_grams = float(rng.choice([0.0, 0.0, 50.0, 100.0, 200.0, 250.0]))
    if hand_events_per_min is None:
        hand_events_per_min = float(rng.choice([0.0, 0.0, 0.0, 2.0, 4.0]))
    if uplink_profile is None:
        uplink_profile = "lan"
    per_joint = lambda r: tuple(_u(rng, r) for _ in range(5))  # noqa: E731
    # One servo model, small per-joint spread: the five are the same part.
    base_speed = _u(rng, ranges.servo_speed_rad_s)
    base_lag = _u(rng, ranges.servo_lag_s)
    return SimParams(
        seed=seed,
        duration_s=duration_s,
        slider=slider,
        slider_values=slider_values,
        operator_clock_drift_ppm=_u(rng, ranges.operator_clock_drift_ppm),
        operator_uptime_ns=int(_u(rng, ranges.operator_uptime_h) * 3600e9),
        payload_grams=payload_grams,
        hand_events_per_min=hand_events_per_min,
        uplink_profile=uplink_profile,
        uplink_schedule=uplink_schedule,
        uplink_seed=int(rng.integers(1, 2**31)),
        lan_one_way_ms=_u(rng, ranges.lan_one_way_ms),
        lan_jitter_ms=_u(rng, ranges.lan_jitter_ms),
        servo_speed_rad_s=tuple(base_speed * float(rng.uniform(0.9, 1.1)) for _ in range(5)),
        servo_lag_s=tuple(base_lag * float(rng.uniform(0.8, 1.25)) for _ in range(5)),
        servo_stiffness_nm_per_rad=per_joint(ranges.servo_stiffness_nm_per_rad),
        stall_torque_nm=per_joint(ranges.stall_torque_nm),
        read_ms=_u(rng, ranges.read_ms),
        read_jitter_ms=_u(rng, ranges.read_jitter_ms),
        read_miss_probability=_u(rng, ranges.read_miss_probability),
        board_silence_per_hour=_u(rng, ranges.board_silence_per_hour),
        battery_start_mv=_u(rng, ranges.battery_start_mv),
        battery_drop_mv_per_min=_u(rng, ranges.battery_drop_mv_per_min),
        robothost_loop_ms=_u(rng, ranges.robothost_loop_ms),
        robot_boot_ns=int(rng.uniform(0.2, 48.0) * 3600e9),
    )


def with_overrides(params: SimParams, **kw) -> SimParams:
    return replace(params, **kw)
