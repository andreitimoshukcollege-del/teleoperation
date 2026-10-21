"""The simulated operator: what the headset's IK would have asked of each joint, over time.

The real targets come from a hand through IK; here they come from the protocol's motion mix
(ADR 0021: free motion, reaching to marks, slow tracing, fast reversals, holding), as min-jerk
reaches and sinusoids in joint space. Recorded human command streams replace this once they exist
(``joint-drive`` replays them), which is the realistic version of this file.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .params import ARM_MOTORS, SimParams

RATE_HZ = 1000  # trajectory samples per second

# Rough usable ranges per arm motor (rad), kept inside the servos' range and away from the body.
# Guesses: the collision model (ADR 0020) decides the real envelope, and joint-drive's is confirmed
# by a person before it moves anything.
JOINT_RANGES = {
    1: (-1.2, 1.2),   # base yaw
    2: (-0.2, 1.3),   # proximal (shoulder) pitch
    3: (-1.4, 0.6),   # distal (elbow) pitch
    4: (-1.3, 1.3),   # wrist pitch
    5: (-1.5, 1.5),   # wrist roll
}


def min_jerk(tau: np.ndarray) -> np.ndarray:
    tau = np.clip(tau, 0.0, 1.0)
    return tau**3 * (10.0 - 15.0 * tau + 6.0 * tau**2)


@dataclass
class OperatorTrajectory:
    """Targets at RATE_HZ from t=0: ``arm`` [n, 5] rad, ``gripper`` [n] 0..1, ``slider`` [n] rad/s."""

    arm: np.ndarray
    gripper: np.ndarray
    slider: np.ndarray
    events: List[Tuple[float, str, str]]  # (t seconds, kind, detail)

    def at(self, t_s: float) -> Tuple[np.ndarray, float, float]:
        i = min(max(int(t_s * RATE_HZ), 0), len(self.gripper) - 1)
        return self.arm[i], float(self.gripper[i]), float(self.slider[i])


def _random_pose(rng: np.random.Generator, spread: float = 0.75) -> np.ndarray:
    pose = np.empty(5)
    for k, m in enumerate(ARM_MOTORS):
        lo, hi = JOINT_RANGES[m]
        mid, half = (lo + hi) / 2, (hi - lo) / 2
        pose[k] = mid + half * spread * rng.uniform(-1.0, 1.0)
    return pose


def _reach(out: np.ndarray, start: int, n: int, a: np.ndarray, b: np.ndarray) -> None:
    s = min_jerk(np.arange(n) / max(n - 1, 1))[:, None]
    out[start:start + n] = a + (b - a) * s


def generate(params: SimParams, rng: np.random.Generator) -> OperatorTrajectory:
    n = int(params.duration_s * RATE_HZ) + RATE_HZ
    arm = np.zeros((n, 5))
    events: List[Tuple[float, str, str]] = []
    marks = [_random_pose(rng, 0.6) for _ in range(int(rng.integers(4, 7)))]
    pose = _random_pose(rng, 0.3)
    i = 0
    styles = ("free", "marks", "trace", "reversals", "hold")
    while i < n:
        style = styles[int(rng.choice(5, p=np.asarray(params.motion_mix) / sum(params.motion_mix)))]
        seg_end = min(n, i + int(rng.uniform(5.0, 40.0) * RATE_HZ))
        events.append((i / RATE_HZ, "motion", style))
        if style in ("free", "marks"):
            while i < seg_end:
                goal = _random_pose(rng) if style == "free" else marks[int(rng.integers(len(marks)))]
                if style == "free" and rng.random() < 0.5:
                    goal = pose + (goal - pose) * rng.uniform(0.2, 0.6)  # many reaches are short
                steps = min(seg_end - i, int(rng.uniform(0.35, 2.5) * RATE_HZ))
                _reach(arm, i, steps, pose, goal)
                pose = arm[i + steps - 1].copy()
                i += steps
                hold = min(seg_end - i, int(rng.uniform(0.0, 1.5 if style == "free" else 2.0) * RATE_HZ))
                arm[i:i + hold] = pose
                i += hold
        elif style == "trace":
            t = np.arange(seg_end - i) / RATE_HZ
            center = pose.copy()
            wave = np.zeros((len(t), 5))
            for k in range(5):
                for _ in range(2):
                    f = rng.uniform(0.08, 0.4)
                    wave[:, k] += rng.uniform(0.05, 0.3) * np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi))
            wave -= wave[0]
            arm[i:seg_end] = center + wave
            pose = arm[seg_end - 1].copy()
            i = seg_end
        elif style == "reversals":
            t = np.arange(seg_end - i) / RATE_HZ
            wave = np.zeros((len(t), 5))
            for k in rng.choice(5, size=int(rng.integers(1, 4)), replace=False):
                f = rng.uniform(0.7, 2.0)
                wave[:, k] = rng.uniform(0.04, 0.25) * np.sin(2 * np.pi * f * t)
            arm[i:seg_end] = pose + wave
            pose = arm[seg_end - 1].copy()
            i = seg_end
        else:  # hold, with a small tremor
            t = np.arange(seg_end - i) / RATE_HZ
            arm[i:seg_end] = pose + 0.002 * np.sin(2 * np.pi * 8.0 * t)[:, None] * rng.uniform(-1, 1, size=5)
            i = seg_end
    for k, m in enumerate(ARM_MOTORS):
        lo, hi = JOINT_RANGES[m]
        np.clip(arm[:, k], lo, hi, out=arm[:, k])

    gripper = np.zeros(n)
    state, t_next = 0.0, rng.uniform(2.0, 15.0)
    while t_next * RATE_HZ < n:
        j = int(t_next * RATE_HZ)
        new = 1.0 - state
        steps = min(n - j, int(0.3 * RATE_HZ))
        gripper[j:j + steps] = state + (new - state) * min_jerk(np.arange(steps) / max(steps - 1, 1))
        gripper[j + steps:] = new
        state = new
        t_next += rng.uniform(4.0, 20.0)

    slider = np.zeros(n)
    if params.slider == "varied":
        t0 = 0
        while t0 < n:
            value = float(rng.choice(params.slider_values))
            t1 = min(n, t0 + int(rng.uniform(30.0, 90.0) * RATE_HZ))
            slider[t0:t1] = value
            events.append((t0 / RATE_HZ, "slider", repr(value)))
            t0 = t1
    else:
        slider[:] = float(params.slider)
        events.append((0.0, "slider", repr(float(params.slider))))
    return OperatorTrajectory(arm=arm, gripper=gripper, slider=slider, events=events)
