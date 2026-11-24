"""How far each command the robot side received, and poses near it, put the arm from the scene (docs/adr/0038 L, "Distances
at commands and sampled poses"): the labels world-models' robot-side safety check learns obstacle distance from. Two
sidecars of a safety dataset, both computed offline from the take's own records and the twin's collision geometry:

- ``meta/teleop/commands.parquet``, one row per action event of ``raw_ros_events.jsonl`` (every ``RO``, and every command
  RobotHost refused as the plant would have sent it: what the dataset's ``action`` is made of), at the commands' own rate:
  when RobotHost received it, whether it was sent or refused, its six servo values exactly as the action has them, and the signed distance from each arm link
  to the nearest obstacle and to the nearest surface **at the commanded pose**, and the smallest **along the way** there: at
  :data:`PATH_STEPS` poses from the arm's measured state when the command arrived (``RI``) to the commanded pose, each joint
  ramping at the event's own rate as the twin's servos do (``teleop_isaac.servo.TimedSetpoint``);
- ``meta/teleop/pose_samples.parquet``, for every frame, :data:`SAMPLES` made-up poses that are never executed, with the same
  distance columns, in that frame's scene: past the current command along its direction, reachable in
  :data:`REACH_SECONDS` at the mixed script's speed caps, anywhere within the joint limits, and searched to land
  :data:`NEAR_MM` from an obstacle.

**The distance** is :mod:`teleop_lerobot.clearance`'s, exactly: the link boxes as Isaac imports them, every obstacle's convex
pieces, GJK apart and the separating axis's penetration depth overlapping (negative), the surfaces by the links' box corners,
capped at its :data:`~teleop_lerobot.clearance.CUTOFF_MM`. Object poses are ``WO`` interpolated to the time (a command's
arrival, a frame's ``stateSensed``) as clearance interpolates them. What is new here is only the speed: many poses at one
time are culled together (bounding spheres per link and per piece, the boxes' own axes, an upper bound from each piece's
nearest vertex) and the pairs left are run through one GJK vectorised over all of them, nearest first per link.

Pure numpy.
"""

from __future__ import annotations

import bisect
import hashlib
import json
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import clearance as C
from .jlog import items, read_jlog
from .takes import (JETROVER_MOTORS, PULSE_PER_RADIAN, ZERO_PULSE, Take, TakeError, _jetrover_actions, _jetrover_pulses,
                    _tracks)

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
SIM_SIDECAR = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.sim.json")  # motor -> joint, the gripper's fingers
URDF = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.urdf")  # the joint limits
SERVO_NAMES = tuple(f"servo_{m}" for m in JETROVER_MOTORS)  # the dataset's action order: servo_1..5, servo_10
PATH_STEPS = 8  # poses along the way to a command, the last its commanded pose
# The pose samples of a frame: (kind, count). beyond: past the current command along its direction; reachable: within
# REACH_SECONDS at the caps; jump: anywhere within the joint limits (an accident); near: searched to land NEAR_MM from an
# obstacle (near misses and light hits).
SAMPLES: Tuple[Tuple[str, int], ...] = (("beyond", 8), ("reachable", 16), ("jump", 4), ("near", 4))
BEYOND = (1.5, 3.0)  # how far past the measured pose a beyond sample goes, in the command's own offsets
REACH_SECONDS = 0.5
# MixedOptions.DefaultCaps (core/Teleop.Eval/JointDrive/MixedTake.cs): yaw, shoulder, elbow, wrist pitch, roll, rad/s, the 99th
# percentile of 9 human headset sessions on the real JetRover. The gripper has no cap there: it moves at its commands' speed.
CAPS_RAD_S = (3.8, 2.6, 3.6, 2.8, 4.0)
GRIPPER_PPS = 2.5 * PULSE_PER_RADIAN  # a gripper pulse rate when no command gives one (the speed slider's default 2.5 rad/s)
NEAR_MM = (0.0, 50.0)
NEAR_TOLERANCE_MM = 2.5  # a near sample's search stops this close to its drawn target distance
NEAR_STEPS = 9  # bisection steps at most
GRID_POSES = 2048  # random poses whose tool points the near search picks a pose inside an object from
REACH_M = 0.45  # an obstacle is a near sample's target only when its bounding sphere comes this close to the base
SEED_MASK = (1 << 53) - 1  # the seed fits a double, for readers in other languages
METHOD = ("teleop_lerobot.clearance's signed distance (link boxes to obstacles' convex pieces: GJK apart, the separating "
          "axis's penetration depth overlapping; surfaces by the links' box corners), vectorised over poses; object poses WO "
          "interpolated to the command's arrival or the frame's stateSensed. The min_ and path_min_ columns are over the links a "
          "joint moves: the base never moves, so an object a layout put near it would set every row's minimum; its own distance "
          "is in the per-link columns")
PATH_METHOD = (f"{PATH_STEPS} poses from the arm's measured state when the command arrived (RI, the newest valid reading of "
               "each servo at or before received_ns) to the commanded pose, at 1/8..8/8 of the time the slowest servo needs: "
               "each servo ramps linearly in pulses at the event's rate (an RO's pulsesPerSecond, a refused command's speedRadPerSec x "
               "pulsePerRadian) and stops at its "
               "target, as the twin's servo setpoints do (teleop_isaac.servo.TimedSetpoint). The setpoint ramps from where "
               "the last ramp stood, not from the reading, and the PhysX drive trails it; neither is modelled. The start "
               "itself is not one of the poses")

SAMPLE_PATH_METHOD = (f"{PATH_STEPS} poses from the frame's measured state (its observation.state: each servo's newest valid RI "
                      "at or before sensed_ns, the gripper's newest relay target) to the sample, at 1/8..8/8 of the time the "
                      "slowest servo needs, as commands.parquet's way: each servo ramps linearly in pulses at the rate of the "
                      "command in force at the frame (the frame's action, the newest accepted row of commands.parquet at or "
                      "before sensed_ns: its RO's pulsesPerSecond; the mixed script's caps for a servo it gives none) and stops "
                      "at its target (teleop_isaac.servo.TimedSetpoint). The start itself is not one of the poses")


class Unavailable(Exception):
    """The take lacks what the command distances need."""


# --- the arm: pulses to joint values -----------------------------------------------------------------------------------------

class Arm:
    """The JetRover's six servos as the twin turns their pulses into URDF joint values: an arm joint's angle is
    ``(pulse - 500) / pulsePerRadian - zeroOffset`` (the take's ``robot-profile.json``, as RobotHost converted), the gripper's
    pulse a closing fraction from its open to its closed pulse and each finger's travel by the twin's table
    (``jetrover.sim.json``); limits in pulses are the URDF's joint limits within 0..1000."""

    def __init__(self, take_path: str, sim_path: str = SIM_SIDECAR, urdf_path: str = URDF):
        try:
            with open(sim_path, encoding="utf-8") as f:
                sim = json.load(f)
            with open(os.path.join(take_path, "robot-profile.json"), encoding="utf-8") as f:
                profile = json.load(f)
            limits = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
                      for j in ET.parse(urdf_path).getroot().findall("joint") if j.find("limit") is not None}
        except (OSError, ValueError, ET.ParseError, AttributeError, TypeError) as e:
            raise Unavailable(f"the servo conversion needs jetrover.sim.json, the URDF and the take's robot-profile.json: {e}") from e
        self.zero, self.ppr = float(sim["zeroPulse"]), float(sim["pulsePerRadian"])
        if abs(self.zero - ZERO_PULSE) > 1e-9 or abs(self.ppr - PULSE_PER_RADIAN) > 1e-3:
            raise Unavailable("jetrover.sim.json's pulse conversion is not RobotHost's")
        motors = {int(m["motorId"]): m for m in sim["motors"]}
        offsets = {}
        for j in profile.get("joints", []):
            low = {str(k).lower(): v for k, v in j.items()}
            offsets[int(low["motorid"])] = float(low.get("zerooffsetradians") or 0.0)
        self.joints: List[Tuple[str, ...]] = []
        self.offsets = np.zeros(len(JETROVER_MOTORS))
        lo, hi = np.zeros(len(JETROVER_MOTORS)), np.zeros(len(JETROVER_MOTORS))
        for i, mid in enumerate(JETROVER_MOTORS):
            m = motors.get(mid)
            if m is None:
                raise Unavailable(f"jetrover.sim.json has no motor {mid}")
            if m.get("fingers"):
                self.gripper = i
                self.open, self.closed = float(m["openPulse"]), float(m["closedPulse"])
                self.table = np.array(m.get("fingerTravelByFraction") or [], float)
                self.finger_open, self.finger_travel = float(m.get("fingerOpenMeters", 0.0)), float(m.get("fingerTravelMeters", 0.0))
                self.joints.append(tuple(m["fingers"]))
                lo[i], hi[i] = min(self.open, self.closed), max(self.open, self.closed)
            else:
                self.joints.append((m["joint"],))
                self.offsets[i] = offsets.get(mid, float(m.get("zeroOffsetRadians", 0.0)))
                a, b = limits.get(m["joint"], (-math.inf, math.inf))
                lo[i] = max(float(sim["minPulse"]), self.zero + (a + self.offsets[i]) * self.ppr)
                hi[i] = min(float(sim["maxPulse"]), self.zero + (b + self.offsets[i]) * self.ppr)
        self.lo, self.hi = lo, hi

    def finger(self, pulse: np.ndarray) -> np.ndarray:
        span = self.closed - self.open
        f = np.clip((pulse - self.open) / span if span else 0.0 * pulse, 0.0, 1.0)
        if len(self.table) > 1:
            return np.interp(f * (len(self.table) - 1), np.arange(len(self.table)), self.table)
        return -self.finger_open + f * (self.finger_open + self.finger_travel)

    def values(self, pulses: np.ndarray) -> Dict[str, np.ndarray]:
        """URDF joint values (radians, a finger in metres) for poses in pulses, (n, 6) in the action's order."""
        p = np.asarray(pulses, float)
        out: Dict[str, np.ndarray] = {}
        for i, names in enumerate(self.joints):
            v = self.finger(p[:, i]) if i == self.gripper else (p[:, i] - self.zero) / self.ppr - self.offsets[i]
            for name in names:
                out[name] = v
        return out


# --- many poses at once ------------------------------------------------------------------------------------------------------

@dataclass
class Result:
    """Per pose, per link: signed distance (m) to the nearest obstacle (at most the cutoff), its id (NO_OBSTACLE beyond the
    cutoff), and to the nearest surface (NaN for a body PhysX never collides with one)."""

    obstacle: np.ndarray
    obstacle_id: np.ndarray
    surface: np.ndarray
    link: Optional[np.ndarray] = None  # min_only: the nearest link's index (-1 when nothing is within the cutoff)
    pose: Optional[np.ndarray] = None  # groups: the pose of the group the nearest is at (-1 when nothing is within the cutoff)


class Engine:
    """Distances from many arm poses to a take's scene at many times, exactly as :func:`clearance.compute` measures one frame,
    batched: scenes are registered per time (:meth:`scene`) and :meth:`distances` takes each pose's scene."""

    def __init__(self, world: C.TakeWorld, cutoff_mm: float = C.CUTOFF_MM):
        self.world, self.robot = world, world.robot
        self.cutoff = cutoff_mm / 1000.0
        r = self.robot
        self.L, self.B = len(r.links), len(r.box_link)
        self.link_boxes = C._box_index(r)
        self.fixed = np.array([name in r.fixed for name in r.links])
        self._bodies: Dict[Tuple, int] = {}
        self.body_c: List[np.ndarray] = []
        self.body_r: List[float] = []
        self.body_id: List[int] = []
        self.body_pieces: List[Tuple[int, int]] = []
        self.body_sphere: List[Optional[Tuple[np.ndarray, float]]] = []
        self.pieces: List[C.Piece] = []
        self._scenes: Dict[Tuple, int] = {}
        self.scenes: List[np.ndarray] = []
        self._tables = None
        surf = [s for _, s in world.surfaces]
        self.surf_c = np.array([s.center for s in surf]).reshape(-1, 3)
        self.surf_h = np.array([(s.vertices.max(0) - s.vertices.min(0)) / 2 for s in surf]).reshape(-1, 3)

    def scene(self, t: float) -> int:
        """The scene at time ``t`` (every obstacle not parked, at its interpolated pose and size): an index for :meth:`distances`."""
        entries = []
        for body, pose, factor, placed in self.world.obstacles_at(t):
            key = (body.id, pose, factor)
            e = self._bodies.get(key)
            if e is None:
                e = len(self.body_id)
                self._bodies[key] = e
                self.body_id.append(body.id)
                if body.sphere > 0:
                    c, rad = np.asarray(pose[:3], float), body.sphere * factor
                    self.body_sphere.append((c, rad))
                    self.body_c.append(c)
                    self.body_r.append(rad)
                    self.body_pieces.append((len(self.pieces), 0))
                else:
                    self.body_sphere.append(None)
                    v = np.concatenate([p.vertices for p in placed]) if placed else np.zeros((1, 3))
                    c = (v.min(0) + v.max(0)) / 2
                    self.body_c.append(c)
                    self.body_r.append(float(np.linalg.norm(v - c, axis=1).max()) if placed else 0.0)
                    self.body_pieces.append((len(self.pieces), len(placed)))
                    self.pieces.extend(placed)
                self._tables = None
            entries.append(e)
        key = tuple(entries)
        s = self._scenes.get(key)
        if s is None:
            s = len(self.scenes)
            self._scenes[key] = s
            self.scenes.append(np.array(entries, int))
        return s

    def _finish(self):
        if self._tables is not None:
            return self._tables
        vmax = max([len(p.vertices) for p in self.pieces] + [1])
        P = len(self.pieces)
        verts = np.zeros((max(P, 1), vmax, 3))
        for i, p in enumerate(self.pieces):
            n = len(p.vertices)
            verts[i, :n] = p.vertices
            verts[i, n:] = p.vertices[0]  # padded with a vertex of its own: supports and bounds are unchanged
        pc = np.array([p.center for p in self.pieces]).reshape(-1, 3)
        pr = np.array([p.radius for p in self.pieces], float)
        start = np.array([s for s, _ in self.body_pieces], int)
        count = np.array([n for _, n in self.body_pieces], int)
        self._tables = (verts, pc, pr, np.array(self.body_c).reshape(-1, 3), np.array(self.body_r, float),
                        np.array(self.body_id, int), start, count)
        return self._tables

    def boxes(self, values: Dict[str, np.ndarray], n: int) -> Tuple[np.ndarray, np.ndarray]:
        R, p = self.robot.link_poses(values, n)
        Rl, pl = R[:, self.robot.box_link], p[:, self.robot.box_link]
        centres = np.einsum("nbij,bj->nbi", Rl, self.robot.box_center) + pl
        axes = np.einsum("bkj,nbij->nbki", self.robot.box_axes, Rl)
        return centres, axes

    def distances(self, values: Dict[str, np.ndarray], scene_of: Sequence[int], min_only: bool = False,
                  groups: Optional[np.ndarray] = None) -> Result:
        """Every link's distances for poses ``values`` (joint name -> (n,)), pose i in scene ``scene_of[i]``. ``min_only``:
        only the nearest link's (``Result.obstacle`` (n, 1), its link in ``Result.link``), every link's surface distance
        as ever. ``groups`` (min_only; (n,) of 0..G-1): only the nearest over each group's poses (``Result.obstacle`` (G, 1),
        its link and the pose it is at in ``Result.pose``), one threshold for the whole group, so that a pair no nearer than
        the group's best so far is never measured: the smallest on a way of several poses for little more than one."""
        scene_of = np.asarray(scene_of, int)
        n = len(scene_of)
        L, cutoff = self.L, self.cutoff
        if groups is not None:
            groups = np.asarray(groups, int)
            min_only = True
        per = 1 if min_only else L  # keys per pose: one, or one a link
        keys = (int(groups.max()) + 1 if n else 0) if groups is not None else n * per
        best = np.full(keys, cutoff)
        best_id = np.full(keys, C.NO_OBSTACLE, int)
        best_link = np.full(keys, -1, int)
        best_pose = np.full(keys, -1, int)
        surface = np.full((n, L), np.nan)
        self._per, self._best_link, self._best_pose, self._grouped = per, best_link, best_pose, groups is not None
        key_of = (lambda tp, tl: groups[tp]) if groups is not None else (lambda tp, tl: tp * per + (0 if min_only else tl))
        if n == 0:
            return self._result(best, best_id, surface, n)
        centres, axes = self.boxes(values, n)
        half = self.robot.box_half
        # Surfaces: each link's nearest box corner (clearance.surface_distance), all poses at once.
        if len(self.surf_c):
            signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
            corners = centres[:, :, None, :] + np.einsum("ck,bk,nbkj->nbcj", signs, half, axes)  # (n, B, 8, 3)
            per_box = None
            for c, h in zip(self.surf_c, self.surf_h):  # clearance.point_box_distance for an axis-aligned box
                q = np.abs(corners - c) - h
                o = np.maximum(q, 0)
                inner = np.minimum(np.maximum(np.maximum(q[..., 0], q[..., 1]), q[..., 2]), 0)
                d = (np.sqrt(np.einsum("nbci,nbci->nbc", o, o)) + inner).min(axis=2)
                per_box = d if per_box is None else np.minimum(per_box, d)  # (n, B)
            for li, idx in enumerate(self.link_boxes):
                if len(idx) and not self.fixed[li]:
                    surface[:, li] = np.minimum(per_box[:, idx].min(axis=1), cutoff)
        verts, pc, pr, bc, br, bid, bstart, bcount = self._finish()
        # Each link's bounding sphere, per pose.
        link_c = np.zeros((n, L, 3))
        link_r = np.zeros((n, L))
        for li, idx in enumerate(self.link_boxes):
            if len(idx):
                c = centres[:, idx].mean(axis=1)
                link_c[:, li] = c
                link_r[:, li] = (np.linalg.norm(centres[:, idx] - c[:, None], axis=2) + np.linalg.norm(half[idx], axis=1)).max(axis=1)
        # (pose, link, body) within the cutoff by bounding spheres, scene by scene.
        tp, tl, tb = [], [], []
        order = np.argsort(scene_of, kind="stable")
        bounds = np.flatnonzero(np.diff(scene_of[order])) + 1
        for group in np.split(order, bounds):
            bodies = self.scenes[scene_of[group[0]]]
            if not len(bodies):
                continue
            lb = np.linalg.norm(link_c[group][:, :, None, :] - bc[bodies][None, None], axis=3) - link_r[group][:, :, None] - br[bodies]
            i, l, e = np.nonzero(lb < cutoff)
            if min_only:  # the nearest moving link: the base never moves (its own distance is in the per-link columns)
                keep = ~self.fixed[l]
                i, l, e = i[keep], l[keep], e[keep]
            tp.append(group[i]); tl.append(l); tb.append(bodies[e])
        if not tp:
            return self._result(best, best_id, surface, n)
        tp, tl, tb = np.concatenate(tp), np.concatenate(tl), np.concatenate(tb)
        box_count = np.array([len(x) for x in self.link_boxes], int)
        box_start = np.concatenate([[0], np.cumsum(box_count)[:-1]])
        box_order = np.concatenate([x for x in self.link_boxes if len(x)] or [np.zeros(0, int)])
        # Spheres: exact, straight in.
        is_sphere = bcount[tb] == 0
        for k in np.flatnonzero(is_sphere):
            sph = self.body_sphere[tb[k]]
            if sph is None:
                continue
            idx = self.link_boxes[tl[k]]
            local = np.einsum("bij,bj->bi", axes[tp[k], idx], sph[0] - centres[tp[k], idx])
            q = np.abs(local) - half[idx]
            d = float((np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)).min()) - sph[1]
            key = key_of(tp[k], tl[k])
            if d < best[key]:
                best[key], best_id[key], best_link[key], best_pose[key] = d, bid[tb[k]], tl[k], tp[k]
        keep = ~is_sphere
        tp, tl, tb = tp[keep], tl[keep], tb[keep]
        # Expand to pieces, cull by the pieces' spheres; then to the link's boxes.
        cnt = bcount[tb]
        rep = np.repeat(np.arange(len(tb)), cnt)
        piece = bstart[tb][rep] + (np.arange(len(rep)) - np.repeat(np.cumsum(cnt) - cnt, cnt))
        tp, tl = tp[rep], tl[rep]
        near = np.linalg.norm(link_c[tp, tl] - pc[piece], axis=1) - link_r[tp, tl] - pr[piece] < cutoff
        tp, tl, piece = tp[near], tl[near], piece[near]
        cnt = box_count[tl]
        rep = np.repeat(np.arange(len(tl)), cnt)
        box = box_order[box_start[tl][rep] + (np.arange(len(rep)) - np.repeat(np.cumsum(cnt) - cnt, cnt))]
        tp, tl, piece = tp[rep], tl[rep], piece[rep]
        if not len(tp):
            return self._result(best, best_id, surface, n)
        # Bounds per (pose, box, piece). First the cheap lower bound, the box against the piece's bounding sphere; each link's
        # most promising pair then gives an upper bound (below), a threshold the others must beat. For what is left: the
        # lower bound also the gap along the box's own axes, the upper bound the signed distance from the box to the piece's
        # nearest vertex (a vertex inside the box at depth d: the pair overlaps by d at least).
        key = key_of(tp, tl)
        cl = np.einsum("kij,kj->ki", axes[tp, box], pc[piece] - centres[tp, box])
        qc = np.abs(cl) - half[box]
        oc = np.maximum(qc, 0)
        lb = (np.sqrt(np.einsum("ki,ki->k", oc, oc)) + np.minimum(np.maximum(np.maximum(qc[:, 0], qc[:, 1]), qc[:, 2]), 0)
              - pr[piece])
        alive = lb < best[key]
        key, lb, tp, box, piece = key[alive], lb[alive], tp[alive], box[alive], piece[alive]
        srt = np.lexsort((lb, key))
        key, lb, tp, box, piece = key[srt], lb[srt], tp[srt], box[srt], piece[srt]
        first = np.ones(len(key), bool)
        first[1:] = key[1:] != key[:-1]
        f = np.flatnonzero(first)
        if groups is not None:  # each pose's most promising pair: the group's threshold the least of its poses' bounds
            by_pose = np.lexsort((lb, tp))
            lead = np.ones(len(by_pose), bool)
            lead[1:] = tp[by_pose][1:] != tp[by_pose][:-1]
            f = by_pose[lead]
        _, ub_f = self._bounds(centres, axes, half, verts, tp[f], box[f], piece[f])
        thr = best.copy()
        np.minimum.at(thr, key[f], ub_f)
        alive = lb <= thr[key]
        key, lb, tp, box, piece = key[alive], lb[alive], tp[alive], box[alive], piece[alive]
        gap, ub = self._bounds(centres, axes, half, verts, tp, box, piece)
        lb = np.maximum(lb, gap)
        np.minimum.at(thr, key, ub)
        alive = lb <= thr[key]  # a pair whose bounds meet is exact already: it may set the threshold itself
        key, lb, ub, tp, box, piece = key[alive], lb[alive], ub[alive], tp[alive], box[alive], piece[alive]
        srt = np.lexsort((lb, key))
        key, lb, ub, tp, box, piece = key[srt], lb[srt], ub[srt], tp[srt], box[srt], piece[srt]
        # Nearest first: each round takes every link's lowest bound still under its best, exactly.
        while len(key):
            first = np.ones(len(key), bool)
            first[1:] = key[1:] != key[:-1]
            take = first & (lb < best[key])
            sel = np.flatnonzero(take)
            if len(sel):
                d = ub[sel].copy()  # bounds that meet (within 1 nm): the pair's distance, no GJK needed
                open_ = np.flatnonzero(ub[sel] - lb[sel] > 1e-9)
                if len(open_):
                    so = sel[open_]
                    d[open_] = self._exact(centres[tp[so], box[so]], axes[tp[so], box[so]], half[box[so]], piece[so], verts, pc)
                k = key[sel]
                better = d < best[k]
                best[k[better]] = d[better]
                best_id[k[better]] = bid[self._owner(piece[sel][better])]
                best_link[k[better]] = self.robot.box_link[box[sel][better]]
                best_pose[k[better]] = tp[sel][better]
            rest = ~first
            rest &= lb < best[key]
            key, lb, ub, tp, box, piece = key[rest], lb[rest], ub[rest], tp[rest], box[rest], piece[rest]
        return self._result(best, best_id, surface, n)

    @staticmethod
    def _bounds(centres, axes, half, verts, tp, box, piece) -> Tuple[np.ndarray, np.ndarray]:
        """Per (pose, box, piece): the gap along the box's own axes (a lower bound) and the signed distance from the box to the
        piece's nearest vertex (an upper bound)."""
        K = len(tp)
        gap, ub = np.empty(K), np.empty(K)
        for s in range(0, K, 8192):
            sl = slice(s, s + 8192)
            c, ax, h = centres[tp[sl], box[sl]], axes[tp[sl], box[sl]], half[box[sl]]
            local = np.matmul(verts[piece[sl]] - c[:, None, :], ax.transpose(0, 2, 1))  # (k, v, 3) in the box's frame
            q = np.abs(local) - h[:, None, :]
            g = np.maximum(local.min(axis=1) - h, -h - local.max(axis=1))
            gap[sl] = np.maximum(np.maximum(g[:, 0], g[:, 1]), g[:, 2])
            out = np.maximum(q, 0)
            inner = np.minimum(np.maximum(np.maximum(q[..., 0], q[..., 1]), q[..., 2]), 0)
            ub[sl] = (np.sqrt(np.einsum("kvi,kvi->kv", out, out)) + inner).min(axis=1)
        return gap, ub

    def _owner(self, piece: np.ndarray) -> np.ndarray:
        _, _, _, _, _, _, bstart, bcount = self._finish()
        if not hasattr(self, "_piece_body") or len(self._piece_body) != len(self.pieces):
            pb = np.zeros(len(self.pieces), int)
            for e, (s, c) in enumerate(self.body_pieces):
                pb[s:s + c] = e
            self._piece_body = pb
        return self._piece_body[piece]

    def _result(self, best, best_id, surface, n) -> Result:
        per = self._per
        far = best >= self.cutoff
        best_id = np.where(far, C.NO_OBSTACLE, best_id)
        link = pose = None
        if per == 1:
            link = np.where(far, -1, self._best_link)
        if self._grouped:
            pose = np.where(far, -1, self._best_pose)
        k = len(best) // per
        return Result(np.minimum(best, self.cutoff).reshape(k, per), best_id.reshape(k, per), surface, link, pose)

    def _exact(self, c, ax, h, piece, verts, pc) -> np.ndarray:
        """The signed distance of each (box, piece) pair: batched GJK apart; overlapping (or within 0.1 um), minus the
        separating axis's penetration depth (clearance.box_piece_distance)."""
        d, overlap = gjk_batch(c, ax, h, verts[piece], c - pc[piece])
        for k in np.flatnonzero(overlap | (d <= 1e-7)):
            d[k] = -C.penetration(C.Box(c[k], ax[k], h[k]), self.pieces[piece[k]])
        return d


# --- GJK, vectorised over pairs ------------------------------------------------------------------------------------------------

_SUBSETS = [(0,), (1,), (2,), (3,), (0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3), (0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3),
            (0, 1, 2, 3)]


def _closest(W: np.ndarray, m: np.ndarray):
    """The point of each simplex (``W`` (k, 4, 3), its first ``m`` points) nearest the origin, by every face's own projection
    (Johnson's distance subalgorithm by brute force): the nearest of the faces whose projection has no negative barycentric
    coordinate. Returns (point, new simplex, its size, whether the origin is inside the tetrahedron)."""
    k = len(W)
    best = np.full(k, np.inf)
    point = np.zeros((k, 3))
    choice = np.zeros(k, int)
    inside = np.zeros(k, bool)
    for si, s in enumerate(_SUBSETS):
        ok = m > max(s)
        if not ok.any():
            continue
        a = W[:, s[0]]
        if len(s) == 1:
            x, valid = a, ok
        elif len(s) == 2:
            e = W[:, s[1]] - a
            ee = np.einsum("ki,ki->k", e, e)
            t = -np.einsum("ki,ki->k", a, e) / np.where(ee > 1e-30, ee, 1.0)
            valid = ok & (ee > 1e-30) & (t >= 0) & (t <= 1)
            x = a + t[:, None] * e
        elif len(s) == 3:
            e1, e2 = W[:, s[1]] - a, W[:, s[2]] - a
            a11, a12, a22 = (np.einsum("ki,ki->k", e1, e1), np.einsum("ki,ki->k", e1, e2), np.einsum("ki,ki->k", e2, e2))
            b1, b2 = -np.einsum("ki,ki->k", a, e1), -np.einsum("ki,ki->k", a, e2)
            det = a11 * a22 - a12 * a12
            good = det > 1e-24 * (a11 * a22 + 1e-300)
            dd = np.where(good, det, 1.0)
            u, v = (b1 * a22 - b2 * a12) / dd, (a11 * b2 - a12 * b1) / dd
            valid = ok & good & (u >= 0) & (v >= 0) & (u + v <= 1)
            x = a + u[:, None] * e1 + v[:, None] * e2
        else:
            e1, e2, e3 = W[:, 1] - a, W[:, 2] - a, W[:, 3] - a
            det = np.einsum("ki,ki->k", e1, np.cross(e2, e3))
            scale = (np.linalg.norm(e1, axis=1) * np.linalg.norm(e2, axis=1) * np.linalg.norm(e3, axis=1)) + 1e-300
            good = np.abs(det) > 1e-12 * scale
            dd = np.where(good, det, 1.0)
            # Cramer's rule for e1 u + e2 v + e3 w = -a.
            u = np.einsum("ki,ki->k", -a, np.cross(e2, e3)) / dd
            v = np.einsum("ki,ki->k", e1, np.cross(-a, e3)) / dd
            w = np.einsum("ki,ki->k", e1, np.cross(e2, -a)) / dd
            valid = ok & good & (u >= 0) & (v >= 0) & (w >= 0) & (u + v + w <= 1)
            inside |= valid
            x = np.zeros((k, 3))
        nn = np.einsum("ki,ki->k", x, x)
        better = valid & (nn < best)
        best[better] = nn[better]
        point[better] = x[better]
        choice[better] = si
    new = np.zeros_like(W)
    size = np.zeros(k, int)
    for si, s in enumerate(_SUBSETS):
        rows = choice == si
        if rows.any():
            new[rows, :len(s)] = W[rows][:, list(s)]
            size[rows] = len(s)
    return point, new, size, inside


def gjk_batch(c: np.ndarray, ax: np.ndarray, h: np.ndarray, verts: np.ndarray, direction: np.ndarray, iterations: int = 64):
    """:func:`clearance.gjk` for many (box, convex vertex set) pairs at once, with its stopping rules: boxes ``c`` (k, 3),
    ``ax`` (k, 3, 3) rows, ``h`` (k, 3); vertex sets ``verts`` (k, v, 3). Returns (distance, overlap)."""
    k = len(c)
    rows = np.arange(k)

    def support(d, sel):
        s = np.where(np.einsum("kij,kj->ki", ax[sel], d) >= 0, 1.0, -1.0)
        pa = c[sel] + np.einsum("ki,kij->kj", s * h[sel], ax[sel])
        pb = verts[sel, np.argmax(np.einsum("kvj,kj->kv", verts[sel], -d), axis=1)]
        return pa - pb

    v = support(direction, rows)
    W = np.zeros((k, 4, 3))
    W[:, 0] = v
    m = np.ones(k, int)
    vv = np.einsum("ki,ki->k", v, v)
    overlap = np.zeros(k, bool)
    active = rows.copy()
    for _ in range(iterations):
        if not len(active):
            break
        hit = vv[active] <= 1e-20
        overlap[active[hit]] = True
        active = active[~hit]
        if not len(active):
            break
        va = v[active]
        w = support(-va, active)
        conv = vv[active] - np.einsum("ki,ki->k", va, w) <= 1e-9 * vv[active] + 1e-18
        active, w = active[~conv], w[~conv]
        if not len(active):
            break
        Wa, ma = W[active], m[active]
        Wa[np.arange(len(active)), ma] = w
        x, Wn, mn, inside = _closest(Wa, ma + 1)
        overlap[active[inside]] = True
        new = np.einsum("ki,ki->k", x, x)
        progress = ~inside & (new < vv[active])
        upd = active[progress]
        vv[upd], v[upd], W[upd], m[upd] = new[progress], x[progress], Wn[progress], mn[progress]
        active = upd
    return np.sqrt(vv), overlap


# --- a take's commands and samples --------------------------------------------------------------------------------------------

@dataclass
class Tables:
    """The two sidecars' columns for one episode (without its index), and how they were made."""

    commands: Dict[str, object]
    samples: Dict[str, object]
    seed: int
    unavailable: Optional[str]
    seconds: float


def seed_of(layout_seed, session: str) -> int:
    """The pose samples' seed for an episode: from its layout seed and its take's session id (a replay shares its source's
    layout seed), so a rebuild draws the same poses whatever the episode's index."""
    digest = hashlib.sha256(f"{layout_seed}|{session}".encode()).digest()
    return int.from_bytes(digest[:8], "little") & SEED_MASK


def _distance_columns(out: Dict[str, object], links: Sequence[str], res: Optional[Result], n: int, moving: np.ndarray,
                      prefix: str = "") -> None:
    """Per link ``clearance_mm.<link>``, ``clearance_id.<link>``, ``surface_mm.<link>`` (mm, frames.parquet's names), and
    over the ``moving`` links (every link a joint moves: not the base) ``min_clearance_mm``, ``min_clearance_id``,
    ``min_clearance_link``, ``min_surface_mm``. NaN (and null ids) without ``res``."""
    if res is None:
        for link in links:
            out[f"{prefix}clearance_mm.{link}"] = np.full(n, np.nan)
            out[f"{prefix}clearance_id.{link}"] = np.full(n, -1)
            out[f"{prefix}surface_mm.{link}"] = np.full(n, np.nan)
        out["min_clearance_mm"], out["min_clearance_id"] = np.full(n, np.nan), np.full(n, -1)
        out["min_clearance_link"], out["min_surface_mm"] = [None] * n, np.full(n, np.nan)
        return
    mm = np.round(res.obstacle * 1000.0, 2)
    sm = np.round(res.surface * 1000.0, 2)
    for li, link in enumerate(links):
        out[f"{prefix}clearance_mm.{link}"] = mm[:, li]
        out[f"{prefix}clearance_id.{link}"] = res.obstacle_id[:, li]
        out[f"{prefix}surface_mm.{link}"] = sm[:, li]
    idx = np.flatnonzero(moving)
    sub = np.where(np.isnan(mm[:, idx]), np.inf, mm[:, idx])
    arg = idx[np.argmin(sub, axis=1)] if n else np.zeros(0, int)
    rows = np.arange(n)
    none = np.isinf(sub.min(axis=1)) if n else np.zeros(0, bool)
    out["min_clearance_mm"] = np.where(none, np.nan, mm[rows, arg]) if n else np.zeros(0)
    out["min_clearance_id"] = np.where(none, -1, res.obstacle_id[rows, arg]) if n else np.zeros(0, int)
    # Nothing within the cutoff (every moving link at it, no object): no nearest link. Until 2026-11-25 it named the first
    # moving link, the turntable, with no object.
    far = res.obstacle_id[rows, arg] < 0 if n else np.zeros(0, bool)
    out["min_clearance_link"] = [None if (z or f) else links[a] for a, z, f in zip(arg, none, far)]
    out["min_surface_mm"] = _nanmin_rows(sm[:, idx])


def _way(engine: Engine, arm: Arm, path: np.ndarray, scenes: np.ndarray, path_ok: np.ndarray) -> Result:
    """The nearest over the first PATH_STEPS - 1 poses of each way (``path`` (n, PATH_STEPS, 6), way i in ``scenes[i]``), one
    group a way: (n, 1) results, ``Result.pose`` the index into the (n x (PATH_STEPS - 1)) poses, every pose's surfaces."""
    n = len(path)
    mid = path[:, :-1].reshape(-1, 6)
    return _evaluate(engine, arm, mid, np.repeat(scenes, PATH_STEPS - 1), path_ok[:, :-1].reshape(-1),
                     groups=np.repeat(np.arange(n), PATH_STEPS - 1), count=n)


def _path_columns(out: Dict[str, object], links: Sequence[str], engine: Optional["Engine"], res_end: Optional[Result],
                  res_mid: Optional[Result], path_ok: np.ndarray, n: int, moving: np.ndarray) -> None:
    """``path_min_clearance_mm``, ``_id``, ``_link``, ``path_min_step`` (1..PATH_STEPS, 0 without a way) and
    ``path_min_surface_mm``: the smallest over the :data:`PATH_STEPS` poses of each row's way (``res_mid``: :func:`_way`'s,
    the first PATH_STEPS - 1; ``res_end``, per link, the last), over the ``moving`` links; a tie goes to the earlier pose.
    NaN (ids -1, links null) where a pose of the way is unknown (``path_ok`` (n, PATH_STEPS) false), and a null link with
    nothing within the cutoff."""
    if res_end is None or res_mid is None:
        out["path_min_clearance_mm"] = np.full(n, np.nan)
        out["path_min_clearance_id"] = np.full(n, -1)
        out["path_min_clearance_link"] = [None] * n
        out["path_min_step"] = np.zeros(n, int)
        out["path_min_surface_mm"] = np.full(n, np.nan)
        return
    L = len(links)
    last = np.where(np.isnan(res_end.obstacle) | ~moving[None, :], np.inf, res_end.obstacle)
    last_link = np.argmin(last, axis=1) if n else np.zeros(0, int)
    rows = np.arange(n)
    end_d, end_id = res_end.obstacle[rows, last_link], res_end.obstacle_id[rows, last_link]
    mid_d = res_mid.obstacle[:, 0]
    on_way = ~(np.where(np.isnan(mid_d), np.inf, mid_d) > np.where(np.isnan(end_d), np.inf, end_d))  # the way's, or a tie
    step = np.where(on_way, np.where(res_mid.pose >= 0, res_mid.pose % (PATH_STEPS - 1), 0) + 1, PATH_STEPS)
    dist = np.where(on_way, mid_d, end_d)
    best_id = np.where(on_way, res_mid.obstacle_id[:, 0], end_id)
    best_link = np.where(on_way, res_mid.link, last_link)
    bad = ~path_ok.all(axis=1)
    out["path_min_clearance_mm"] = np.where(bad, np.nan, np.round(dist * 1000.0, 2))
    out["path_min_clearance_id"] = np.where(bad, -1, best_id)
    far = (dist >= engine.cutoff) | (best_id < 0) | (best_link < 0)
    out["path_min_clearance_link"] = [None if (b or f) else links[k] for k, b, f in zip(best_link, bad, far)]
    out["path_min_step"] = np.where(bad, 0, step)
    sf = np.concatenate([res_mid.surface.reshape(n, PATH_STEPS - 1, L), res_end.surface[:, None, :]], axis=1)
    out["path_min_surface_mm"] = np.where(bad, np.nan, np.round(_nanmin_rows(sf.reshape(n, -1)) * 1000.0, 2))


def path_poses(start: np.ndarray, cmd: np.ndarray, pps: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """The way from ``start`` to ``cmd`` (pulses, (n, 6)) as the servos go, each at its own rate ``pps`` (pulses per second,
    0: it jumps): :data:`PATH_STEPS` poses (n, steps, 6) at 1/8..8/8 of the time ``T`` (n,) the slowest servo needs, each
    servo ramping linearly to its target and holding there (``TimedSetpoint``); the last pose is ``cmd``."""
    n = len(start)
    delta = cmd - start
    with np.errstate(all="ignore"):
        need = np.where(pps > 0, np.abs(delta) / pps, 0.0)
    need = np.where(np.isnan(need), 0.0, need)
    T = need.max(axis=1) if n else np.zeros(0)
    tk = T[:, None] * (np.arange(1, PATH_STEPS + 1) / PATH_STEPS)[None, :]  # (n, steps)
    with np.errstate(all="ignore"):
        frac = np.where(need[:, None, :] > 0, np.minimum(1.0, tk[:, :, None] / np.where(need > 0, need, 1.0)[:, None, :]), 1.0)
    path = start[:, None, :] + frac * delta[:, None, :]
    path[:, -1] = cmd
    return path, T


def _nanmin_rows(a: np.ndarray) -> np.ndarray:
    """Each row's smallest value that is not NaN; NaN for a row of NaN."""
    if a.shape[0] == 0:
        return np.zeros(0)
    filled = np.where(np.isnan(a), np.inf, a)
    m = filled.min(axis=1)
    return np.where(np.isinf(m), np.nan, m)


def _measured(host, motors=JETROVER_MOTORS):
    """Each servo's valid readings (``RI``), and the gripper's relay targets (``RO``) for when it has none."""
    return _tracks(host.tag("RI"), 0, 2, valid_part=1), _tracks(host.tag("RO"), 0, 1)


def _state_at(read, relay, t: int) -> Tuple[np.ndarray, int]:
    """The arm's measured pulses at ``t`` (each servo's newest valid reading at or before it; the gripper's newest relay
    target when it has never read), NaN for a servo with neither, and the oldest reading's time."""
    out = np.full(len(JETROVER_MOTORS), np.nan)
    oldest = t
    for i, m in enumerate(JETROVER_MOTORS):
        got = read.get(m) and read[m].at(t)
        if not got and m == JETROVER_MOTORS[-1]:
            got = relay.get(m) and relay[m].at(t)
        if got:
            out[i] = got[1]
            oldest = min(oldest, got[0])
    return out, oldest


def action_events(path: str, host) -> List[Tuple[int, str, Dict[int, float], Dict[int, float], Optional[int]]]:
    """The take's action events, one per line ``raw_ros_events.jsonl`` gives type ``action`` (``takes.raw_events``): every
    ``RO`` and every command RobotHost refused (``JC`` ``R``) as the plant would have sent it, in time order, those naming all
    six servos. Each is (time, ``A`` sent to the relay or ``R`` refused, its pulses, each servo's rate in pulses a second (the
    ``RO``'s own, a refused command's speed x pulsePerRadian), the ``JC`` sequence it carries: an ``RO``'s the newest accepted
    command delivered at or before it, None for one before any). Checked against ``takes._jetrover_actions``, the action
    column's own events, value for value."""
    ro = sorted(((int(r[0]), "A", {int(it[0]): float(it[1]) for it in items(r[1])},
                  {int(it[0]): float(it[2]) for it in items(r[1]) if len(it) > 2}) for r in host.tag("RO")), key=lambda e: e[0])
    commands = host.tag("JC")
    accepted = sorted((int(r[1]), int(r[2])) for r in commands if r[4] == "A")
    out = [e + (None,) for e in ro]
    refused = [r for r in commands if r[4] == "R"]
    if refused:
        convert = _jetrover_pulses(path)
        for r in refused:
            values = convert(r[5])
            if values:
                speeds = {}
                for it in items(r[5]):
                    try:
                        speeds.setdefault(int(it[0]), float(it[2]) * PULSE_PER_RADIAN)
                    except (IndexError, ValueError):
                        pass
                out.append((int(r[1]), "R", values, speeds, int(r[2])))
        out.sort(key=lambda e: e[0])
    want = _jetrover_actions(path, host)
    if [(e[0], e[2]) for e in out] != [(t, v) for t, v in want]:
        raise Unavailable("the action events here differ from takes.py's: the two readings of RO and JC have parted")
    times = [t for t, _ in accepted]
    full = []
    for t, kind, values, rates, seq in out:
        if all(m in values for m in JETROVER_MOTORS):
            if kind == "A":
                k = bisect.bisect_right(times, t) - 1
                seq = accepted[k][1] if k >= 0 else None
            full.append((t, kind, values, rates, seq))
    return full


def compute(take: Take, geometry_dir: Optional[str], layout_seed=None, host=None, cutoff_mm: float = C.CUTOFF_MM) -> Tables:
    """The episode's ``commands.parquet`` and ``pose_samples.parquet`` columns (a JetRover take's). Without the geometry (or a
    record they need), the times, outcomes and servo values are still given, with NaN distances, and ``unavailable`` says
    why (as :func:`sidecars.clearance` words it)."""
    import time as _time

    if take.robot != "jetrover":
        raise Unavailable(f"command distances are the JetRover's; this take is {take.robot}'s")
    started = _time.perf_counter()
    seed = seed_of(layout_seed, take.session_id)
    rng = np.random.default_rng(seed)
    host = host or read_jlog(os.path.join(take.path, "robothost.jlog"), "robothost")
    why = None
    world = engine = arm = None
    try:  # needs no geometry: the samples' joint limits come from it too
        arm = Arm(take.path)
    except Unavailable as e:
        why = str(e)
    folder = os.path.expanduser(geometry_dir) if geometry_dir else None
    if folder is None:
        why = "no geometry folder given"
    elif not os.path.isdir(folder):
        why = f"no geometry at {geometry_dir}"
    if why is None:
        try:
            world = C.TakeWorld(take.path, folder)
            engine = Engine(world, cutoff_mm)
        except C.ClearanceUnavailable as e:
            why = f"ClearanceUnavailable: {e}"
    links = list(world.robot.links) if world is not None else list(_LINKS)
    read, relay = _measured(host)
    frame_t = [int(f.timing["sensed"]) for f in take.frames]

    # --- commands: the action events, as raw_ros_events.jsonl has them
    events = action_events(take.path, host)
    n = len(events)
    received = np.array([e[0] for e in events], np.int64)
    cmd = np.array([[e[2][m] for m in JETROVER_MOTORS] for e in events], float).reshape(n, 6)
    pps = np.array([[e[3].get(m, 0.0) for m in JETROVER_MOTORS] for e in events], float).reshape(n, 6)
    pps = np.where(np.isfinite(pps) & (pps > 0), pps, 0.0)
    start = np.full((n, 6), np.nan)
    state_ns = np.zeros(n, np.int64)
    for i, t in enumerate(received):
        start[i], state_ns[i] = _state_at(read, relay, int(t))
    commands: Dict[str, object] = {
        "frame_index": [(bisect.bisect_right(frame_t, t) - 1) if frame_t and t >= frame_t[0] else None for t in received],
        "received_ns": received, "seq": [e[4] for e in events], "outcome": [e[1] for e in events],
        "accepted": np.array([e[1] == "A" for e in events], bool)}
    for j, name in enumerate(SERVO_NAMES):
        commands[name] = cmd[:, j]
    for j, name in enumerate(SERVO_NAMES):
        commands["state." + name] = start[:, j]
    commands["state_ns"] = state_ns
    path, T = path_poses(start, cmd, pps)
    commands["path_s"] = np.round(T, 6)
    path_ok = ~np.isnan(path).any(axis=2)
    res_cmd = res_mid = None
    if engine is not None and n:
        scenes = np.array([engine.scene(float(t)) for t in received])
        res_cmd = _evaluate(engine, arm, cmd, scenes, path_ok[:, -1])
        res_mid = _way(engine, arm, path, scenes, path_ok)
    moving = ~engine.fixed if engine is not None else np.array([x != "base_link" for x in links])
    _distance_columns(commands, links, res_cmd, n, moving)
    _path_columns(commands, links, engine, res_cmd, res_mid, path_ok, n, moving)

    # --- pose samples, each with its way from the frame's measured pose at the rates of the command in force there (the
    # frame's action: the newest accepted row at or before it)
    fallback = np.array([c * PULSE_PER_RADIAN for c in CAPS_RAD_S] + [GRIPPER_PPS])  # a servo the command gives no rate
    rates = np.tile(fallback, (len(frame_t), 1))
    acc = np.array([i for i, e in enumerate(events) if e[1] == "A"], int)
    if len(acc) and len(frame_t):
        k = np.searchsorted(received[acc], np.asarray(frame_t, np.int64), side="right") - 1
        have = k >= 0
        got = pps[acc[k[have]]]
        rates[have] = np.where(got > 0, got, fallback[None, :])
    samples = _samples(take, arm, engine, rng, links, frame_t, rates)
    return Tables(commands, samples, seed, why, _time.perf_counter() - started)


_LINKS = ("base_link", "turntable", "proximal_link", "distal_link", "wrist_link", "gripper_link", "left_finger", "right_finger")


def _evaluate(engine: Engine, arm: Arm, pulses: np.ndarray, scenes: np.ndarray, ok: Optional[np.ndarray] = None,
              min_only: bool = False, groups: Optional[np.ndarray] = None, count: int = 0) -> Result:
    """Distances for poses in pulses; a pose with a NaN servo gets NaN (ids NO_OBSTACLE, link -1). With ``groups`` ((n,) of
    0..count-1): the nearest over each group's poses (:meth:`Engine.distances`), NaN for a group with no known pose, and
    ``Result.pose`` the index into ``pulses``."""
    n = len(pulses)
    ok = np.ones(n, bool) if ok is None else ok
    L, per = engine.L, (1 if (min_only or groups is not None) else engine.L)
    rows = count if groups is not None else n
    out = Result(np.full((rows, per), np.nan), np.full((rows, per), C.NO_OBSTACLE, int), np.full((n, L), np.nan),
                 np.full(rows, -1, int) if per == 1 else None, np.full(rows, -1, int) if groups is not None else None)
    idx = np.flatnonzero(ok)
    if len(idx):
        if groups is None:
            r = engine.distances(arm.values(pulses[idx]), scenes[idx], min_only)
            out.obstacle[idx], out.obstacle_id[idx], out.surface[idx] = r.obstacle, r.obstacle_id, r.surface
            if min_only:
                out.link[idx] = r.link
        else:
            g = np.asarray(groups)[idx]
            r = engine.distances(arm.values(pulses[idx]), scenes[idx], groups=g)
            seen = np.unique(g)
            out.obstacle[seen], out.obstacle_id[seen], out.link[seen] = r.obstacle[seen], r.obstacle_id[seen], r.link[seen]
            out.pose[seen] = np.where(r.pose[seen] >= 0, idx[np.maximum(r.pose[seen], 0)], -1)
            out.surface[idx] = r.surface
    return out


def _samples(take: Take, arm: Optional[Arm], engine: Optional[Engine], rng, links, frame_t,
             rates: Optional[np.ndarray] = None) -> Dict[str, object]:
    """Every frame's :data:`SAMPLES` poses and their distances; with ``rates`` (nf, 6: each servo's pulses a second at that
    frame), also the smallest over the way to each from the frame's measured pose (:func:`path_poses`), as
    commands.parquet's ``path_min_`` columns."""
    nf = len(take.frames)
    total = sum(c for _, c in SAMPLES)
    kinds = [k for k, c in SAMPLES for _ in range(c)]
    state = np.array([f.state for f in take.frames], float).reshape(nf, 6)
    lo = arm.lo if arm is not None else np.array([0, 0, 0, 0, 0, 116.0])
    hi = arm.hi if arm is not None else np.array([1000.0] * 5 + [645.0])
    # The command in force at each frame is its action: the newest action event at or before it (commands.parquet's rows).
    action = np.array([f.action for f in take.frames], float).reshape(nf, 6)
    poses = np.full((nf, total, 6), np.nan)
    scale = np.full((nf, total), np.nan)
    target = np.full((nf, total), np.nan)
    target_obj = np.full((nf, total), -1)
    reach = np.array([c * PULSE_PER_RADIAN * REACH_SECONDS for c in CAPS_RAD_S] + [GRIPPER_PPS * REACH_SECONDS])
    kind_at = {k: [i for i, x in enumerate(kinds) if x == k] for k, _ in SAMPLES}
    for fi in range(nf):
        q = state[fi]
        c = np.where(np.isnan(action[fi]), q, action[fi])
        b = kind_at["beyond"]
        s = BEYOND[0] + (BEYOND[1] - BEYOND[0]) * (np.arange(len(b)) + rng.random(len(b))) / len(b)
        poses[fi, b] = np.clip(q + s[:, None] * (c - q), lo, hi)
        scale[fi, b] = s
        r_ = kind_at["reachable"]
        a, z = np.maximum(lo, q - reach), np.minimum(hi, q + reach)
        poses[fi, r_] = a + rng.random((len(r_), 6)) * (z - a)
        j = kind_at["jump"]
        poses[fi, j] = lo + rng.random((len(j), 6)) * (hi - lo)
    near = kind_at["near"]
    targets = NEAR_MM[0] + rng.random((nf, len(near))) * (NEAR_MM[1] - NEAR_MM[0])
    res = res_mid = None
    if engine is not None and nf:
        scenes = np.array([engine.scene(float(t)) for t in frame_t])
        _near_search(engine, arm, rng, state, scenes, poses, target, target_obj, near, targets, lo, hi, reach)
        flat = poses.reshape(-1, 6)
        res = _evaluate(engine, arm, flat, np.repeat(scenes, total), ~np.isnan(flat).any(axis=1))
    # The way to each sample from the frame's measured pose, as a command's (the start itself is not one of the poses).
    path = path_ok = T = None
    if rates is not None:
        start = np.repeat(state, total, axis=0)
        path, T = path_poses(start, poses.reshape(-1, 6), np.repeat(rates, total, axis=0))
        path_ok = ~np.isnan(path).any(axis=2)
        if res is not None:
            res_mid = _way(engine, arm, path, np.repeat(scenes, total), path_ok)
    out: Dict[str, object] = {"frame_index": np.repeat(np.arange(nf), total), "sample": np.tile(np.arange(total), nf),
                              "kind": kinds * nf}
    flat = poses.reshape(-1, 6)
    for jj, name in enumerate(SERVO_NAMES):
        out[name] = np.round(flat[:, jj], 4)
    out["scale"] = np.round(scale.reshape(-1), 6)
    out["target_mm"] = np.round(target.reshape(-1), 3)
    out["target_id"] = target_obj.reshape(-1)
    moving = ~engine.fixed if engine is not None else np.array([x != "base_link" for x in links])
    _distance_columns(out, links, res, nf * total, moving)
    if rates is not None:
        out["path_s"] = np.round(T, 6)
        _path_columns(out, links, engine, res, res_mid, path_ok, nf * total, moving)
    return out


def _tool_points(engine: Engine, arm: Arm, pulses: np.ndarray) -> np.ndarray:
    centres, _ = engine.boxes(arm.values(pulses), len(pulses))
    r = engine.robot
    fingers = np.concatenate([engine.link_boxes[r.links.index(x)] for x in ("left_finger", "right_finger") if x in r.links])
    return centres[:, fingers].mean(axis=1)


def _near_search(engine, arm, rng, state, scenes, poses, target, target_obj, near, targets, lo, hi, reach) -> None:
    """Each frame's near samples (in place): toward a pose whose fingers sit inside an obstacle in reach, along a straight line
    in joint space from the measured pose (the first and third) or a reachable pose (the second and fourth), bisected until
    the minimum distance over the links is within NEAR_TOLERANCE_MM of a target drawn from NEAR_MM. A frame with no obstacle
    in reach searches toward the nearest one anyway. When both ends lie on one side of the target, the start is drawn once more
    from anywhere within the limits; a line that still does not straddle it keeps the pose nearest the target."""
    nf = len(state)
    grid = lo + rng.random((GRID_POSES, 6)) * (hi - lo)
    tips = _tool_points(engine, arm, grid)
    verts, pc, pr, bc, br, bid, bstart, bcount = engine._finish()
    me = _tool_points(engine, arm, state)
    a_list, b_list, slot = [], [], []
    for fi in range(nf):
        bodies = engine.scenes[scenes[fi]]
        if not len(bodies):
            continue
        inreach = bodies[np.linalg.norm(bc[bodies], axis=1) - br[bodies] < REACH_M]
        pool = inreach if len(inreach) else bodies
        nearest = pool[np.argmin(np.linalg.norm(bc[pool] - me[fi], axis=1) - br[pool])]
        for k, col in enumerate(near):
            e = nearest if k < 2 else pool[rng.integers(len(pool))]
            off = rng.normal(size=3)
            off *= 0.3 * br[e] * rng.random() ** (1 / 3) / (np.linalg.norm(off) or 1.0)
            point = bc[e] + off
            closest = np.argsort(np.linalg.norm(tips - point, axis=1))[:4]
            b = grid[closest[rng.integers(len(closest))]].copy()
            if k % 2 == 0:
                a = state[fi].copy()
            else:
                lo_, hi_ = np.maximum(lo, state[fi] - reach), np.minimum(hi, state[fi] + reach)
                a = lo_ + rng.random(6) * (hi_ - lo_)
            if np.isnan(a).any():
                continue
            a_list.append(a); b_list.append(b); slot.append((fi, col, k, int(bid[e])))
    if not slot:
        return
    A, Bp = np.array(a_list), np.array(b_list)
    fis = np.array([s[0] for s in slot])
    tau = np.array([targets[s[0], s[2]] for s in slot]) / 1000.0
    sc = scenes[fis]

    def dmin(p, sel):
        return engine.distances(arm.values(p), sc[sel], min_only=True).obstacle[:, 0]

    m = len(slot)
    allsel = np.arange(m)
    da, db = dmin(A, allsel), dmin(Bp, allsel)
    best_p = np.where((np.abs(da - tau) <= np.abs(db - tau))[:, None], A, Bp)
    best_e = np.minimum(np.abs(da - tau), np.abs(db - tau))
    # Ends on one side of the target (the arm already that close, or the end not inside): once more from anywhere in the limits.
    again = np.flatnonzero((da >= tau) == (db >= tau))
    if len(again):
        A[again] = lo + rng.random((len(again), 6)) * (hi - lo)
        d2 = dmin(A[again], again)
        e2 = np.abs(d2 - tau[again])
        better = e2 < best_e[again]
        best_e[again[better]], best_p[again[better]] = e2[better], A[again][better]
        da[again] = d2
    lo_s, hi_s = np.zeros(m), np.ones(m)  # d(lo_s) on the start's side of the target, d(hi_s) on the other
    side = da >= tau
    active = np.flatnonzero((side != (db >= tau)) & (best_e > NEAR_TOLERANCE_MM / 1000.0))
    for _ in range(NEAR_STEPS):
        if not len(active):
            break
        s = (lo_s[active] + hi_s[active]) / 2
        p = A[active] + s[:, None] * (Bp[active] - A[active])
        d = dmin(p, active)
        e = np.abs(d - tau[active])
        better = e < best_e[active]
        best_e[active[better]] = e[better]
        best_p[active[better]] = p[better]
        same = (d >= tau[active]) == side[active]
        lo_s[active[same]] = s[same]
        hi_s[active[~same]] = s[~same]
        active = active[e > NEAR_TOLERANCE_MM / 1000.0]
    for i, (fi, col, k, obj) in enumerate(slot):
        poses[fi, col] = best_p[i]
        target[fi, col] = tau[i] * 1000.0
        target_obj[fi, col] = obj


def metadata(cutoff_mm: float = C.CUTOFF_MM) -> Dict[str, Dict]:
    """What each file's parquet metadata says about it (``teleop`` key)."""
    common = {"method": METHOD, "cutoffMm": cutoff_mm, "units": "servo values in the dataset's action units (relay pulses; "
              "the gripper 116 open to 645 closed); distances in mm, negative overlapping, capped at cutoffMm (id -1 there)",
              "links": list(_LINKS), "source": "docs/adr/0038 L"}
    return {
        "commands": {**common, "pathMethod": PATH_METHOD, "pathSteps": PATH_STEPS,
                     "clocks": "received_ns (raw_ros_events.jsonl's pc_received_mono_ns of the action event) and state_ns: "
                               "RobotHost's CLOCK_MONOTONIC, capture_timing.jsonl's and frames.parquet's sensed_ns clock; "
                               "frame_index is the last frame captured at or before received_ns (null before the first)",
                     "rows": "one per action event of raw_ros_events.jsonl, at the commands' own rate: every RO (what RobotHost "
                             "sent the relay for an accepted command, or of its own while parking) and every refused command as "
                             "the plant would have sent it; each frame's action is the last row received at or before it",
                     "outcome": "A: an RO, sent to the relay; R: a command RobotHost refused (it would collide, or the servo "
                                "path from the measured pose would); seq: the JC it carries (an RO's: the newest accepted one)"},
        "pose_samples": {**common, "pathMethod": SAMPLE_PATH_METHOD, "pathSteps": PATH_STEPS, "samples": dict(SAMPLES), "beyond": list(BEYOND), "reachSeconds": REACH_SECONDS,
                         "capsRadPerS": list(CAPS_RAD_S), "gripperPulsesPerS": GRIPPER_PPS, "nearMm": list(NEAR_MM),
                         "nearToleranceMm": NEAR_TOLERANCE_MM,
                         "kinds": "beyond: measured pose + scale x (current command - measured pose), scale stratified "
                                  "in [1.5, 3]; reachable: uniform within REACH_SECONDS at the caps of the measured pose, "
                                  "within the limits; jump: uniform within the joint limits; near: bisected along a line in "
                                  "joint space toward a pose with the fingers inside an obstacle until the minimum over links "
                                  "is target_mm (drawn uniform in nearMm) within nearToleranceMm, when the line brackets it",
                         "seed": "episodes.jsonl poseSampleSeed: sha256 of '<layoutSeed>|<session>', its first 8 bytes "
                                 "little-endian, 53 bits"}}


ACTION_TOLERANCE = 0.01  # pulses: takes.PULSE_TOLERANCE, and the data parquet's float32 holds a pulse to about 3e-5


def action_agreement(commands, frames) -> Tuple[int, int, List[Tuple[int, int]]]:
    """The pilot check of world-models' commands.parquet spec: each frame's ``action`` (``frames``: ``episode_index``,
    ``frame_index``, ``sensed_ns``, ``action``) must be the last command (``commands``: commands.parquet's rows) received at
    or before the frame was captured, within :data:`ACTION_TOLERANCE`. Returns (frames that agree, frames, the first 20
    (episode, frame) that do not)."""
    agree, total, bad = 0, 0, []
    by_ep = {int(ep): g for ep, g in commands.groupby("episode_index", sort=False)}
    for ep, g in frames.groupby("episode_index", sort=False):
        c = by_ep.get(int(ep))
        acts = np.array([np.asarray(a, float) for a in g["action"]]).reshape(len(g), -1)
        total += len(g)
        if c is None:
            bad.extend((int(ep), int(f)) for f in g["frame_index"][:20 - len(bad)])
            continue
        received = c["received_ns"].to_numpy()
        vals = c[list(SERVO_NAMES)].to_numpy(float)
        idx = np.searchsorted(received, g["sensed_ns"].to_numpy(), side="right") - 1
        ok = (idx >= 0) & (np.abs(acts - vals[np.maximum(idx, 0)]) <= ACTION_TOLERANCE).all(axis=1)
        agree += int(ok.sum())
        for f in g["frame_index"].to_numpy()[~ok]:
            if len(bad) < 20:
                bad.append((int(ep), int(f)))
    return agree, total, bad


TEXT_COLUMNS = ("outcome", "kind", "min_clearance_link", "path_min_clearance_link")  # every other column is a number


def _column(name: str, values: list):
    import pandas as pd

    if name in ("received_ns", "state_ns", "sample", "episode_index"):
        return pd.array(np.asarray(values, np.int64), dtype="int64")
    if name in ("frame_index", "seq"):
        return pd.array([None if v is None else int(v) for v in values], dtype="Int64")
    if "clearance_id" in name or name == "target_id":
        return pd.array([None if v is None or int(v) < 0 else int(v) for v in values], dtype="Int32")
    if name == "path_min_step":
        return pd.array(np.asarray(values, np.int64), dtype="int8")
    if name == "accepted":
        return pd.array(np.asarray(values, bool), dtype="bool")
    if name in TEXT_COLUMNS:
        return pd.array([None if v is None else str(v) for v in values], dtype="object")
    # Every other column is a number: a distance, a servo value, a time, a scale. Until 2026-11-25 a name ending in "_link"
    # was taken for a link's name, so clearance_mm.<link>_link and surface_mm.<link>_link were written as text.
    return pd.array(np.asarray(values, float), dtype="float64")  # None becomes NaN


def write_parquet(path: str, written: Sequence[Tuple[int, Dict[str, object]]], meta: Dict) -> int:
    """One sidecar from every episode's columns ((episode index, columns) each, in order), its ``meta`` as the parquet file's
    ``teleop`` metadata (JSON). Returns the row count."""
    import pandas as pd
    import pyarrow as pa
    import pyarrow.parquet as pq

    order: List[str] = []
    for _, cols in written:
        for k in cols:
            if k not in order:
                order.append(k)
    data: Dict[str, list] = {"episode_index": []}
    for k in order:
        data[k] = []
    for ep, cols in written:
        n = len(next(iter(cols.values()))) if cols else 0
        data["episode_index"].extend([ep] * n)
        for k in order:
            v = cols.get(k)
            data[k].extend(list(v) if v is not None else [None] * n)
    df = pd.DataFrame({k: _column(k, v) for k, v in data.items()})
    table = pa.Table.from_pandas(df, preserve_index=False)
    table = table.replace_schema_metadata({**(table.schema.metadata or {}), b"teleop": json.dumps(meta).encode()})
    pq.write_table(table, path)
    return len(df)


__all__ = ["ACTION_TOLERANCE", "action_agreement", "action_events", "Arm", "Engine", "METHOD", "PATH_STEPS", "Result", "SAMPLES", "Tables", "Unavailable", "compute", "gjk_batch",
           "metadata", "path_poses", "seed_of", "write_parquet"]
