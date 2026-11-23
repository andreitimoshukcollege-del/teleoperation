"""How far the JetRover's arm reaches over its table, from its URDF and its collision model (docs/adr/0038 decision 3).

A scene places obstacles where the arm can meet them and distractors where it never can. Both regions come from here: the
arm's joints are swept over their whole ranges (the URDF's limits, which are Core's collision model's), every pose whose
bodies stay the model's margin above the table is kept (Core's ``ArmCollision`` floor rule: each body's bounds at least
``margin`` above ``floorZ``), and every corner of every collider the twin has (the URDF's boxes, its finger pads included)
is measured from the base's yaw axis. Self-collision is not checked, so the figures err on the far side: a pose the arm's
own body would refuse still counts as reaching.

Run it anywhere with numpy, the URDF named on the command line, or through ssh stdin on the GPU machine::

    ssh andrei@<gpu> "~/isaacsim/lerobot-venv/bin/python - <repo>/robot/jetrover/urdf/jetrover.urdf" < robot/isaac/reach_envelope.py

It prints JSON:

- ``reachBelow``: for each height above the table top (m), the farthest any collider corner at or below that height comes
  from the yaw axis (horizontally, m), over every pose kept. An object no taller than the height whose footprint stays
  farther than that is out of every pose's reach.
- ``nearestBelow``: the nearest such corner (the folded arm's), the inner edge of where the arm meets a low object.
- ``bearing``: the widest bearing (rad) any corner below 0.10 m reaches, the yaw joint's limit plus the colliders' spread.
- ``farthest``: the farthest any corner comes at any height, and the heights of the table top and the floor rule.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

FLOOR_Z = -0.1962  # Core's JetRoverCollisionModel: the table top, root frame
MARGIN = 0.010     # and its margin: every body's bounds at least this far above it
HEIGHTS = (0.01, 0.02, 0.03, 0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.70)
ARM = ("proximal_link", "distal_link", "wrist_link", "gripper_link", "left_finger", "right_finger")
# Core's model's two CAD fingers (gripper.1, gripper.2), which its floor rule checks in place of the twin's pads: centre,
# quaternion x, y, z, w (System.Numerics order), half extents, in the gripper link's frame.
CAD_FINGERS = (((0.1271, 0.0172, 0.0022), (-0.701666, 0.036105, 0.706733, 0.083003), (0.0068, 0.0199, 0.0376)),
               ((0.1271, -0.0177, 0.0022), (-0.076137, 0.702773, 0.034469, 0.706488), (0.0068, 0.0196, 0.0375)))


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_matrices(axis, angles):
    """Rotations about a unit axis by each angle: (n, 3, 3)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s = np.cos(angles)[:, None, None], np.sin(angles)[:, None, None]
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3)[None] * c + s * k[None] + (1 - c) * np.outer(a, a)[None]


def read(path):
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        joints[j.find("child").get("link")] = {
            "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
            "xyz": np.array([float(v) for v in o.get("xyz", "0 0 0").split()]),
            "rpy": rpy_matrix(*[float(v) for v in o.get("rpy", "0 0 0").split()]),
            "axis": [float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()],
            "limits": (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else (0.0, 0.0)}
    boxes = {}
    for link in root.findall("link"):
        for c in link.findall("collision"):
            o, g = c.find("origin"), c.find("geometry").find("box")
            if g is None:
                continue
            boxes.setdefault(link.get("name"), []).append(
                (np.array([float(v) for v in o.get("xyz").split()]), rpy_matrix(*[float(v) for v in o.get("rpy").split()]),
                 np.array([float(v) for v in g.get("size").split()]) / 2))
    return joints, boxes


def corners(rot, pos, centre, turn, half):
    """The 8 corners of a box (centre, turn, half extents in a link's frame) for each link pose: (n, 8, 3)."""
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    local = centre[None] + (signs * half[None]) @ turn.T
    return pos[:, None, :] + np.einsum("nij,kj->nki", rot, local)


def main(path):
    joints, boxes = read(path)
    lim = {j["name"]: j["limits"] for j in joints.values()}
    step = math.radians
    grid = np.meshgrid(np.arange(lim["proximal"][0], lim["proximal"][1] + 1e-9, step(2.0)),
                       np.arange(lim["distal"][0], lim["distal"][1] + 1e-9, step(2.5)),
                       np.arange(lim["wrist"][0], lim["wrist"][1] + 1e-9, step(3.0)), indexing="ij")
    prox, dist, wrist = (g.ravel() for g in grid)
    rolls = np.linspace(lim["roll"][0], lim["roll"][1], 9)
    grips = (lim["gripper"][0], lim["gripper"][1])
    reach = {h: 0.0 for h in HEIGHTS}
    nearest = {h: math.inf for h in HEIGHTS}
    widest, farthest, kept, poses = 0.0, 0.0, 0, 0
    floor = FLOOR_Z + MARGIN
    for start in range(0, len(prox), 40000):
        n = len(prox[start:start + 40000])
        values = {"proximal": prox[start:start + n], "distal": dist[start:start + n], "wrist": wrist[start:start + n]}
        frames = {"base_link": (np.broadcast_to(np.eye(3), (n, 3, 3)), np.zeros((n, 3)))}
        for link in ("turntable", "proximal_link", "distal_link", "wrist_link"):
            j = joints[link]
            rot, pos = frames[j["parent"]]
            pos = pos + rot @ j["xyz"]
            rot = rot @ j["rpy"]
            if j["name"] in values:
                rot = rot @ axis_matrices(j["axis"], values[j["name"]])
            frames[link] = (rot, pos)
        upper = [corners(*frames[link], *b) for link in ("proximal_link", "distal_link", "wrist_link") for b in boxes.get(link, [])]
        upper = np.concatenate(upper, axis=1)
        for roll in rolls:
            for grip in grips:
                rot_w, pos_w = frames["wrist_link"]
                jr = joints["gripper_link"]
                rot_g = rot_w @ jr["rpy"] @ axis_matrices(jr["axis"], np.full(n, roll))
                pos_g = pos_w + rot_w @ jr["xyz"]
                parts = [corners(rot_g, pos_g, *b) for b in boxes.get("gripper_link", [])]
                for finger in ("left_finger", "right_finger"):
                    jf = joints[finger]
                    shift = rot_g @ (np.asarray(jf["axis"]) * grip + jf["xyz"])
                    parts += [corners(rot_g @ jf["rpy"], pos_g + shift, *b) for b in boxes.get(finger, [])]
                twin = np.concatenate([upper] + parts, axis=1)  # every collider the twin has
                cad = np.concatenate([upper] + [corners(rot_g, pos_g, np.array(c), quat_matrix(*q), np.array(h))
                                                for c, q, h in CAD_FINGERS] + parts[:len(boxes.get("gripper_link", []))], axis=1)
                ok = cad[:, :, 2].min(axis=1) >= floor  # Core's floor rule, on Core's bodies
                poses += n
                kept += int(ok.sum())
                pts = twin[ok]
                if len(pts) == 0:
                    continue
                d = np.hypot(pts[:, :, 0], pts[:, :, 1])
                z = pts[:, :, 2] - FLOOR_Z
                farthest = max(farthest, float(d.max()))
                low = z <= 0.10
                if low.any():
                    widest = max(widest, float(np.abs(np.arctan2(pts[:, :, 1], pts[:, :, 0])[low]).max()))
                for h in HEIGHTS:
                    below = z <= h
                    if below.any():
                        reach[h] = max(reach[h], float(d[below].max()))
                        nearest[h] = min(nearest[h], float(d[below].min()))
    yaw = max(abs(v) for v in lim["base_yaw"])
    print(json.dumps({
        "about": "robot/isaac/reach_envelope.py: the JetRover arm's colliders over the table, every pose of a joint grid that "
                 "keeps Core's floor rule (self-collision not checked: the figures err on the far side)",
        "urdf": path, "poses": poses, "kept": kept, "tableTop": FLOOR_Z, "floorRule": floor,
        "reachBelow": {f"{h:.2f}": round(reach[h], 4) for h in HEIGHTS},
        "nearestBelow": {f"{h:.2f}": round(nearest[h], 4) for h in HEIGHTS if math.isfinite(nearest[h])},
        "bearing": round(yaw + widest, 4), "yawLimit": round(yaw, 4), "farthest": round(farthest, 4)}, indent=1))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "robot/jetrover/urdf/jetrover.urdf")
