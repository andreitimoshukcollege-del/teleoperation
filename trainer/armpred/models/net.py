"""Architecture A (an MLP) and B (small convolutions over the window's sequences), and the export
wrapper that makes either one the flat graph ``IInferenceBackend`` expects."""

from __future__ import annotations

import math
from typing import Tuple

import torch
from torch import nn

from ..features import layout as L

MAX_CORRECTION = L.MAX_CORRECTION_RAD
GRIPPER_MAX_CORRECTION = L.GRIPPER_MAX_CORRECTION_RAD
LOG_SCALE_MIN, LOG_SCALE_MAX = math.log(1e-4), math.log(0.5)
# Where the gripper's identity flag sits in a joint row (the identity block's last entry, layout v3).
_GRIPPER_FLAG = 2 + L.N_PATH * 2 + L.N_READINGS * 3 + L.N_INNOVATIONS * 4 + L.PREDICTED_MOTORS.index(L.GRIPPER_MOTOR)


def _head(raw: torch.Tensor, joint_row: torch.Tensor, flag_index: int = _GRIPPER_FLAG,
          cap_rad: float = MAX_CORRECTION, flag_cap: float = GRIPPER_MAX_CORRECTION) -> torch.Tensor:
    """[..., 2] raw -> [..., 2]: the correction bounded to +-``cap_rad`` (+-``flag_cap`` on the row whose
    identity flag at ``flag_index`` is set: v3's gripper, v4's hand), the log-scale clamped. The defaults are v3's."""
    flag = joint_row[..., flag_index]
    cap = cap_rad + (flag_cap - cap_rad) * flag
    corr = cap * torch.tanh(raw[..., 0])
    log_b = torch.clamp(raw[..., 1], LOG_SCALE_MIN, LOG_SCALE_MAX)
    return torch.stack([corr, log_b], dim=-1)


class JointMLP(nn.Module):
    """A: [joint row | arm row] -> 128 -> 128 -> 64 -> 2, ReLU. About 70k parameters (layout v2)."""

    name = "mlp"

    def __init__(self, joint_features: int = L.JOINT_FEATURES, arm_features: int = L.ARM_FEATURES,
                 hidden: Tuple[int, ...] = (128, 128, 64), flag_index: int = _GRIPPER_FLAG,
                 cap_rad: float = MAX_CORRECTION, flag_cap: float = GRIPPER_MAX_CORRECTION):
        super().__init__()
        self.flag_index, self.cap_rad, self.flag_cap = flag_index, cap_rad, flag_cap
        layers = []
        width = joint_features + arm_features
        for h in hidden:
            layers += [nn.Linear(width, h), nn.ReLU()]
            width = h
        layers.append(nn.Linear(width, 2))
        self.net = nn.Sequential(*layers)
        # Start near "trust the prior": a small final layer, so the correction begins near zero.
        nn.init.normal_(self.net[-1].weight, std=1e-3)
        nn.init.zeros_(self.net[-1].bias)

    def forward(self, joint_row: torch.Tensor, arm_row: torch.Tensor) -> torch.Tensor:
        return _head(self.net(torch.cat([joint_row, arm_row], dim=-1)), joint_row, self.flag_index, self.cap_rad, self.flag_cap)


def _slices():
    """Where each part of a joint row sits (features/layout.py's table)."""
    k = 0
    out = {}
    for name, width in (("prior", 2), ("path", L.N_PATH * 2), ("readings", L.N_READINGS * 3),
                        ("innovations", L.N_INNOVATIONS * 4), ("identity", len(L.PREDICTED_MOTORS)), ("angle", 2),
                        ("reference", 1)):
        out[name] = (k, k + width)
        k += width
    assert k == L.JOINT_FEATURES
    return out


class _Stream(nn.Module):
    """One sequence (newest first, the last channel a mask): embed, two depthwise-separable
    convolutions (kernel 5), masked attention pooling."""

    def __init__(self, channels: int, width: int = 16):
        super().__init__()
        self.embed = nn.Linear(channels - 1, width)
        self.depth1 = nn.Conv1d(width, width, 5, padding=2, groups=width)
        self.point1 = nn.Conv1d(width, width, 1)
        self.depth2 = nn.Conv1d(width, width, 5, padding=2, groups=width)
        self.point2 = nn.Conv1d(width, width, 1)
        self.score = nn.Linear(width, 1)

    def forward(self, seq: torch.Tensor) -> torch.Tensor:
        mask = seq[..., -1:]
        h = torch.relu(self.embed(seq[..., :-1])) * mask
        h = h.transpose(1, 2)
        h = torch.relu(self.point1(self.depth1(h)))
        h = torch.relu(self.point2(self.depth2(h))).transpose(1, 2)
        score = self.score(h).squeeze(-1) + (mask.squeeze(-1) - 1.0) * 1e4
        weight = torch.softmax(score, dim=-1).unsqueeze(-1) * mask
        return (weight * h).sum(dim=1)


class JointConv(nn.Module):
    """B: each sequence (the commanded path, readings, innovations) through ``_Stream``, then the
    pooled streams, the joint's scalars and the arm row (network block included) -> 64 -> 2."""

    name = "conv"

    def __init__(self, arm_features: int = L.ARM_FEATURES, width: int = 16):
        super().__init__()
        self.s = _slices()
        self.path = _Stream(2, width)
        self.readings = _Stream(3, width)
        self.innovations = _Stream(4, width)
        scalars = 2 + len(L.PREDICTED_MOTORS) + 2 + 1 + arm_features
        self.head = nn.Sequential(nn.Linear(3 * width + scalars, 64), nn.ReLU(), nn.Linear(64, 2))
        nn.init.normal_(self.head[-1].weight, std=1e-3)
        nn.init.zeros_(self.head[-1].bias)

    def _part(self, row: torch.Tensor, name: str, channels: int) -> torch.Tensor:
        a, b = self.s[name]
        return row[..., a:b].reshape(row.shape[0], -1, channels)

    def forward(self, joint_row: torch.Tensor, arm_row: torch.Tensor) -> torch.Tensor:
        # The path's points sit on a fixed grid, so (target, mask) needs no time channel.
        pooled = [
            self.path(self._part(joint_row, "path", 2)),
            self.readings(self._part(joint_row, "readings", 3)),
            self.innovations(self._part(joint_row, "innovations", 4)),
        ]
        a, b = self.s["prior"]
        i0, i1 = self.s["identity"]
        g0, g1 = self.s["angle"]
        r0, r1 = self.s["reference"]
        scalars = torch.cat([joint_row[..., a:b], joint_row[..., i0:i1], joint_row[..., g0:g1], joint_row[..., r0:r1], arm_row], dim=-1)
        return _head(self.head(torch.cat(pooled + [scalars], dim=-1)), joint_row)


def build(name: str, aware: bool = False, layout: str = "v3") -> nn.Module:
    if layout == "v4":
        from ..features import layout_v4 as L4

        if name != "mlp":
            raise ValueError("layout v4 runs architecture A (mlp) only, the one the repo's backend computes")
        return JointMLP(joint_features=L4.JOINT_FEATURES, arm_features=L4.ARM_FEATURES, flag_index=L4.HAND_FLAG_INDEX,
                        cap_rad=L4.MAX_CORRECTION_RAD, flag_cap=L4.HAND_MAX_CORRECTION)
    arm = L.arm_features(aware)
    if name == "mlp":
        return JointMLP(arm_features=arm)
    if name == "conv":
        return JointConv(arm_features=arm)
    raise ValueError(f"unknown architecture {name!r}")


class FlatExportV4(nn.Module):
    """Layout v4's graph (docs/adr/0028 decision 2): ``x`` [1, rows * JOINT_FEATURES + arms * ARM_FEATURES]
    (every motor's row in profile order, then each arm's block) and ``y`` [1, rows * 2]. Row r is paired with arm
    ``arm_of_row[r]``'s block: one shared MLP for every joint of every arm."""

    def __init__(self, model: nn.Module, joint_features: int, arm_features: int, arm_of_row: Tuple[int, ...], arms: int):
        super().__init__()
        self.model = model
        self.jf, self.af = joint_features, arm_features
        self.arm_of_row = tuple(int(a) for a in arm_of_row)
        self.arms = arms

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        n = len(self.arm_of_row)
        joints = x[:, : n * self.jf].reshape(n, self.jf)
        blocks = x[:, n * self.jf:].reshape(self.arms, self.af)
        # Static shapes: one row of the arm blocks per joint row, picked by a fixed slice each.
        arm = torch.cat([blocks[a:a + 1] for a in self.arm_of_row], dim=0)
        return self.model(joints, arm).reshape(1, n * 2)


class FlatExport(nn.Module):
    """The graph the repo runs: one input ``x`` [1, 6 * JOINT_FEATURES + ARM_FEATURES(+1)] (the six
    joint rows, motors 1-5 then the gripper (10), then the arm row) and one output ``y`` [1, 12] (per
    joint: the correction in model units -- rad, the gripper's servo rad -- then the log-scale).
    ``IInferenceBackend`` reads and writes one flat span."""

    def __init__(self, model: nn.Module, arm_features: int):
        super().__init__()
        self.model = model
        self.arm_features = arm_features

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        n = len(L.PREDICTED_MOTORS)
        joints = x[:, : n * L.JOINT_FEATURES].reshape(n, L.JOINT_FEATURES)
        arm = x[:, n * L.JOINT_FEATURES:]
        # Static shapes, so a plain Concat rather than expand (which exports shape arithmetic).
        arm = torch.cat([arm] * n, dim=0)
        return self.model(joints, arm).reshape(1, n * 2)
