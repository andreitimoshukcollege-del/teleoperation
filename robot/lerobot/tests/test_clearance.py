"""The per-frame clearance (docs/adr/0038 decision 4): signed distances between boxes and convex pieces with known answers,
the arm's forward kinematics against poses worked out by hand, and a tiny take end to end."""

import json
import math
import os
import random
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conftest import MS, REPO, Sidecar, finish, session_json, write_jlog
from teleop_lerobot import clearance as C
from teleop_lerobot.takes import read_take

URDF = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.urdf")


def box(centre, size, rotation=np.eye(3)):
    return C.Box(np.asarray(centre, float), np.asarray(rotation, float).T.copy(), np.asarray(size, float) / 2)


def turned(rng):
    q = rng.normal(size=4)
    return C._quat_matrix(q / np.linalg.norm(q))


# --- distances ---------------------------------------------------------------------------------------------------------------

def test_two_boxes_apart_along_an_axis():
    piece = C.box_piece([0.1, 0.1, 0.1])
    assert C.box_piece_distance(box([0.1, 0, 0], [0.08, 0.02, 0.02]), piece) == pytest.approx(0.01, abs=1e-7)


def test_two_boxes_corner_to_corner_are_the_diagonal_apart():
    piece = C.box_piece([0.1, 0.1, 0.1])
    g = 0.004  # each axis: a separating-axis gap would say 4 mm, the corners are 4 * sqrt(3) apart
    b = box([0.05 + g + 0.01, 0.05 + g + 0.01, 0.05 + g + 0.01], [0.02, 0.02, 0.02])
    assert C.box_piece_distance(b, piece) == pytest.approx(g * math.sqrt(3), abs=1e-7)


def test_overlapping_boxes_are_minus_their_least_overlap():
    piece = C.box_piece([0.1, 0.1, 0.1])
    b = box([0.05 + 0.01 - 0.003, 0.0, 0.0], [0.02, 0.02, 0.02])  # 3 mm in along x, wholly inside along y and z
    assert C.box_piece_distance(b, piece) == pytest.approx(-0.003, abs=1e-9)
    deep = box([0.0, 0.0, 0.049], [0.02, 0.02, 0.02])  # its top 9 mm out of the piece's top, 41 mm in from the others
    assert C.box_piece_distance(deep, piece) == pytest.approx(-(0.05 - 0.049 + 0.01), abs=1e-9)


def _segment_distance(p1, q1, p2, q2):
    """Ericson's closest points of two segments: their distance."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-18 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t, s = 0.0, np.clip(-c / a, 0, 1)
    elif t > 1:
        t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return float(np.linalg.norm(p1 + d1 * s - (p2 + d2 * t)))


def _reference(a: C.Box, b: C.Box) -> float:
    """Two disjoint boxes' distance, feature pair by feature pair: each one's corners against the other, and every edge pair."""
    ca, cb = a.corners(), b.corners()
    best = min(C.point_box_distance(ca, b.center, b.axes, b.half).min(), C.point_box_distance(cb, a.center, a.axes, a.half).min())
    def edges(c):
        return [(c[i], c[j]) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1]
    for p1, q1 in edges(ca):
        for p2, q2 in edges(cb):
            best = min(best, _segment_distance(p1, q1, p2, q2))
    return float(best)


def test_turned_boxes_apart_agree_with_every_feature_pair():
    rng = np.random.default_rng(3)
    checked = 0
    for _ in range(300):
        a = box(rng.uniform(-0.05, 0.05, 3), rng.uniform(0.01, 0.08, 3), turned(rng))
        size = rng.uniform(0.01, 0.08, 3)
        R, x = turned(rng), rng.uniform(-0.12, 0.12, 3)
        piece = C.box_piece(size).placed(R, x, 1.0)
        b = C.Box(x, R.T.copy(), size / 2)
        d = C.box_piece_distance(a, piece)
        if C.penetration(a, piece) >= 0:  # they overlap: covered by the next test
            assert d <= 1e-7
            continue
        assert d == pytest.approx(_reference(a, b), abs=2e-6)
        checked += 1
    assert checked > 100


def test_turned_boxes_overlapping_are_negative_and_no_deeper_than_the_box_axes_say():
    rng = np.random.default_rng(5)
    for _ in range(100):
        a = box(rng.uniform(-0.01, 0.01, 3), rng.uniform(0.02, 0.06, 3), turned(rng))
        size = rng.uniform(0.02, 0.06, 3)
        piece = C.box_piece(size).placed(turned(rng), rng.uniform(-0.01, 0.01, 3), 1.0)
        d = C.box_piece_distance(a, piece)
        assert d < 0
        # Any axis's overlap bounds the depth from above; the box's own axes are among them.
        proj = piece.vertices @ a.axes.T
        along = np.minimum(a.axes @ a.center + a.half - proj.min(0), proj.max(0) - (a.axes @ a.center - a.half))
        assert -d <= along.min() + 1e-9


def test_a_cylinder_and_a_sphere():
    cyl = C.cylinder_piece(0.1, 0.2)  # a corner of the 32-gon at +x
    assert C.box_piece_distance(box([0.05 + 0.02 + 0.01, 0, 0], [0.02, 0.02, 0.02]), cyl) == pytest.approx(0.02, abs=1e-7)
    # Facing a side's middle, the prism is r * cos(pi / 32) out: 0.12 mm inside the circle at r = 5 cm.
    mid = math.pi / 32
    c = np.array([math.cos(mid), math.sin(mid), 0.0])
    b = box(c * (0.05 + 0.02 + 0.01), [0.02, 0.02, 0.02], np.array([c, [-c[1], c[0], 0], [0, 0, 1]]).T)
    assert C.box_piece_distance(b, cyl) == pytest.approx(0.02 + 0.05 * (1 - math.cos(mid)), abs=1e-7)
    assert C.box_sphere_distance(box([0, 0, 0], [0.02, 0.02, 0.02]), np.array([0.0, 0.0, 0.05]), 0.03) == pytest.approx(0.01)
    assert C.box_sphere_distance(box([0, 0, 0], [0.02, 0.02, 0.02]), np.array([0.0, 0.0, 0.035]), 0.03) == pytest.approx(-0.005)


def octahedron(a):
    v = [[a, 0, 0], [-a, 0, 0], [0, a, 0], [0, -a, 0], [0, 0, a], [0, 0, -a]]
    faces = []
    for sx, x in ((1, 0), (-1, 1)):
        for sy, y in ((1, 2), (-1, 3)):
            for sz, z in ((1, 4), (-1, 5)):
                faces.append([x, y, z] if sx * sy * sz > 0 else [x, z, y])
    return v, faces


def test_a_convex_mesh_piece_its_planes_and_distances():
    v, f = octahedron(0.05)
    piece = C.piece_from_mesh(v, f)
    assert len(piece.normals) == 8 and np.all(piece.offsets > 0)  # outward
    assert np.allclose(np.abs(piece.normals), 1 / math.sqrt(3))
    # Above its top vertex, a flat box 7 mm clear; then pushed 2 mm into it from the side, across an edge.
    assert C.box_piece_distance(box([0, 0, 0.05 + 0.007 + 0.005], [0.04, 0.04, 0.01]), piece) == pytest.approx(0.007, abs=1e-7)
    side = box([0.025 + 0.01 / math.sqrt(2) * 0, 0.025, 0], [0.01, 0.01, 0.01], C._quat_matrix([math.cos(math.pi / 8), 0, 0, math.sin(math.pi / 8)]))
    assert C.box_piece_distance(side, piece) < 0
    with pytest.raises(ValueError):
        C.piece_from_mesh(v[:3], f[:1])


def test_point_to_box_is_signed():
    d = C.point_box_distance(np.array([[0.0, 0.0, 0.0], [0.03, 0.0, 0.0], [0.03, 0.04, 0.0]]), np.zeros(3), np.eye(3),
                             np.array([0.01, 0.02, 0.03]))
    assert d == pytest.approx([-0.01, 0.02, math.hypot(0.02, 0.02)])


# --- the arm -------------------------------------------------------------------------------------------------------------------

def urdf_robot():
    """The robot file teleop_isaac.clearance_geometry writes, from the URDF alone."""
    root = ET.parse(URDF).getroot()
    links = [lk.get("name") for lk in root.findall("link")]
    joints, boxes = [], []
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        joints.append({"name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                       "child": j.find("child").get("link"), "xyz": [float(x) for x in o.get("xyz").split()],
                       "rpy": [float(x) for x in o.get("rpy").split()], "axis": [float(x) for x in a.get("xyz").split()]})
    for lk in root.findall("link"):
        for c in lk.findall("collision"):
            o, size = c.find("origin"), [float(x) for x in c.find("geometry/box").get("size").split()]
            m = C._rpy(*[float(x) for x in o.get("rpy").split()])
            boxes.append({"link": lk.get("name"), "name": c.get("name"), "center": [float(x) for x in o.get("xyz").split()],
                          "axes": m.T.tolist(), "half": [s / 2 for s in size]})
    return {"format": "teleop-clearance-robot", "version": 1, "robot": "jetrover", "links": links, "joints": joints,
            "fixed": ["base_link"], "boxes": boxes}


@pytest.fixture
def geometry(tmp_path):
    d = tmp_path / "clearance"
    (d / "assets" / "test").mkdir(parents=True)
    with open(d / "robot-jetrover.json", "w", encoding="utf-8") as f:
        json.dump(urdf_robot(), f)
    cube = [[x, y, z] for x in (-0.025, 0.025) for y in (-0.025, 0.025) for z in (-0.025, 0.025)]
    faces = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4],
             [1, 5, 7], [1, 7, 3]]
    with open(d / "assets" / "test" / "cube.json", "w", encoding="utf-8") as f:
        json.dump({"format": "teleop-clearance-geometry", "version": 1, "asset": "test/cube", "size": [0.05, 0.05, 0.05],
                   "method": "a test cube", "pieces": [{"vertices": cube, "faces": faces}]}, f)
    C._CACHE.clear()
    return str(d)


def poses(geometry_dir, **values):
    robot = C.load_robot(geometry_dir)
    R, p = robot.link_poses({k: np.array([v]) for k, v in values.items()}, 1)
    return robot, R[0], p[0]


def test_the_arm_at_zero_lies_along_x(geometry):
    robot, R, p = poses(geometry)
    at = dict(zip(robot.links, p))
    assert at["proximal_link"] == pytest.approx([0, 0, 0.035])
    assert at["wrist_link"] == pytest.approx([0.26, 0, 0.035])
    assert at["gripper_link"] == pytest.approx([0.26, 0, 0.035])
    centres, _ = robot.boxes(R, p)
    pad = centres[[b for b in range(len(robot.box_link)) if robot.links[robot.box_link[b]] == "right_finger"][0]]
    assert pad == pytest.approx([0.26 + 0.147565, -0.02266, 0.035 + 0.003051])


def test_the_shoulder_up_the_turntable_round_and_the_fingers_apart(geometry):
    _, _, p = poses(geometry, proximal=math.pi / 2)  # about -y: +x turns to +z
    assert p[3] == pytest.approx([0, 0, 0.035 + 0.13], abs=1e-12) and p[4] == pytest.approx([0, 0, 0.035 + 0.26], abs=1e-12)
    _, _, p = poses(geometry, base_yaw=math.pi / 2)
    assert p[5] == pytest.approx([0, 0.26, 0.035], abs=1e-12)
    _, _, p = poses(geometry, distal=-math.pi / 2)  # the forearm down
    assert p[4] == pytest.approx([0.13, 0, 0.035 - 0.13], abs=1e-12)
    robot, R, p = poses(geometry, gripper=0.01, gripper_mirror=0.01)  # the left finger along -y, the right along +y
    assert p[6] == pytest.approx([0.26, -0.01, 0.035]) and p[7] == pytest.approx([0.26, 0.01, 0.035])
    _, R, _ = poses(geometry, roll=math.pi / 2)  # about the gripper's x: its y turns to z
    assert R[5] @ np.array([0, 1.0, 0]) == pytest.approx([0, 0, 1], abs=1e-12)


# --- a tiny take ---------------------------------------------------------------------------------------------------------------

FRAME_START_MS = 500
JOINTS = ("base_yaw", "distal", "gripper", "gripper_mirror", "proximal", "roll", "wrist")
# The arm straight out along x at zero: the left pad's tip at x = 0.407724 + 0.014998, the right's at 0.407565 + 0.0148935.
LEFT_TIP, RIGHT_TIP = 0.407724 + 0.014998, 0.407565 + 0.0148935


def tiny_take(root, wz="31:1;32:1;33:1", sz=None, vq=True, moving=None):
    """A take whose arm stays at zero: a box object 10 mm beyond the left pad's tip, a cube asset to one side, a parked object,
    and a table 34 mm below the pads. ``moving``: (t0 ms, y0, t1 ms, y1) for the cube's WO poses."""
    take = os.path.join(root, "mixed", "20261122T100000Z-take001")
    os.makedirs(take)
    sid = session_json(take, "jetrover-session", 1, "isaac", "mixed")
    scene = {"format": "teleop-scene", "version": 2, "name": "tiny", "robotProfile": "jetrover",
             "surfaces": [{"id": "table", "center": [0.35, 0, -0.02], "size": [0.5, 1.0, 0.04], "colour": [0.5, 0.5, 0.5]},
                          {"id": "floor", "center": [0, 0, -1.0], "size": [4, 4, 0.02], "colour": [0.5, 0.5, 0.5],
                           "collides": True}],
             "objects": [{"id": 31, "name": "wall", "shape": "box", "size": [0.04, 0.2, 0.04], "kind": "fixed"},
                         {"id": 32, "name": "cube", "shape": "mesh", "asset": "test/cube", "size": [0.1, 0.1, 0.1], "kind": "movable"},
                         {"id": 33, "name": "parked", "shape": "box", "size": [0.05, 0.05, 0.05], "kind": "movable"}]}
    with open(os.path.join(take, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(scene, f)
    rows = []
    for k in range(0, 41):
        t = (1000 + 50 * k) * MS
        rows.append(["JC", t, t, k, t, "A", "1:0.1:1"])
    for k in range(0, 116):
        t = (900 + 20 * k) * MS
        rows.append(["RO", t, ";".join(f"{m}:500:300" for m in (1, 2, 3, 4, 10, 5))])
        rows.append(["RI", t + 5 * MS, ";".join(f"{m}:1:500" for m in (1, 2, 3, 4, 5, 10))])
    wall = f"31:{LEFT_TIP + 0.010 + 0.02}:0:0.035:1:0:0:0"
    parked = "33:17:-15:-4.97:1:0:0:0"
    if moving:
        t0, y0, t1, y1 = moving
        rows.append(["WO", t0 * MS + MS, t0 * MS, 1, f"{wall};32:0.3:{y0}:0.035:1:0:0:0;{parked}"])
        rows.append(["WO", t1 * MS + MS, t1 * MS, 1, f"{wall};32:0.3:{y1}:0.035:1:0:0:0;{parked}"])
    else:
        rows.append(["WO", 100 * MS, 90 * MS, 1, f"{wall};32:0.3:0.4:0.035:1:0:0:0;{parked}"])
    if wz is not None:
        rows.append(["WZ", 95 * MS, 1, wz])
    rows.sort(key=lambda r: int(r[1]))
    write_jlog(os.path.join(take, "robothost.jlog"), "robothost", sid, rows)
    side = Sidecar(os.path.join(take, "isaac", "wrist.rgb.frames"))
    frames = [["CM", "wrist", 64, 48, 1, 1, 32, 24, 0.01, 6, 0, 0, 0, 1, 0, 0, 0, "wrist_link", "rgb", "mount"]]
    if sz is not None:
        frames.append(["SZ", 0, sz])
    for n in range(0, 91):
        sensed = FRAME_START_MS * MS + round(n * 1e9 / 30)
        off, length = side.add(f"frame-{n}".encode())
        if vq:
            frames.append(["VQ", n, sensed, ";".join(f"{j}:0" for j in JOINTS)])
        frames.append(["VF", sensed + 100 * MS, n, sensed, "", f"wrist.rgb:{off}:{length}"])
    side.close()
    write_jlog(os.path.join(take, "isaac.jlog"), "isaac", sid, frames)
    finish(take)
    return take


def test_a_tiny_take_end_to_end(scenarios, geometry):
    take = read_take(tiny_take(scenarios))
    c = C.compute(take, geometry)
    assert c.links == ("base_link", "turntable", "proximal_link", "distal_link", "wrist_link", "gripper_link", "left_finger",
                       "right_finger")
    assert len(c.obstacle_mm) == len(c.obstacle_id) == len(c.surface_mm) == len(take.frames)
    d, ids, s = np.array(c.obstacle_mm), np.array(c.obstacle_id), np.array(c.surface_mm)
    assert d[:, 6] == pytest.approx(10.0, abs=0.01) and set(ids[:, 6]) == {31}  # the left pad, 10 mm short of the wall
    assert d[:, 7] == pytest.approx((LEFT_TIP + 0.01 - RIGHT_TIP) * 1000, abs=0.01)
    assert d.max() <= c.cutoff_mm == C.CUTOFF_MM and (ids[d >= c.cutoff_mm] == C.NO_OBSTACLE).all()
    assert np.isnan(s[:, 0]).all()  # the base stands in the bench: PhysX never collides it with a surface
    assert s[:, 6] == pytest.approx((0.035 + 0.002667 - 0.00739 / 2) * 1000, abs=0.01)  # the left pad's underside over the table top
    assert s[:, 7] == pytest.approx((0.035 + 0.003051 - 0.007074 / 2) * 1000, abs=0.01)
    assert c.method == C.METHOD + "; mesh pieces by a test cube"
    world = C.TakeWorld(take.path, geometry)
    t = take.frames[3].timing["sensed"]
    assert C.link_object_distance(world, t, 6, 31) * 1000 == pytest.approx(10.0, abs=0.01)
    assert C.link_surface_distance(world, t, 6) * 1000 == pytest.approx(s[3, 6], abs=0.01)
    assert math.isnan(C.link_surface_distance(world, t, 0))


def test_the_size_factor_in_force_scales_an_object(scenarios, geometry):
    c = C.compute(read_take(tiny_take(scenarios, wz="31:1.25;32:1;33:1")), geometry)
    assert np.array(c.obstacle_mm)[:, 6] == pytest.approx(10.0 - 5.0, abs=0.01)  # 25 mm wider each way: 5 mm nearer


def test_without_wz_the_renderers_sizes_or_the_files(scenarios, geometry):
    c = C.compute(read_take(tiny_take(scenarios, wz=None, sz="31:1.25;32:1")), geometry)
    assert np.array(c.obstacle_mm)[:, 6] == pytest.approx(5.0, abs=0.01)
    os.rename(scenarios, scenarios + "-1")
    os.makedirs(scenarios)
    c = C.compute(read_take(tiny_take(scenarios, wz=None)), geometry)
    assert np.array(c.obstacle_mm)[:, 6] == pytest.approx(10.0, abs=0.01)


def test_object_poses_are_interpolated_and_the_parked_left_out(scenarios, geometry):
    path = tiny_take(scenarios, moving=(1000, 0.4, 2000, 0.2))
    world = C.TakeWorld(path, geometry)
    assert world.poses.at(32, 1500 * MS)[1] == pytest.approx(0.3)
    assert world.poses.at(32, 500 * MS)[1] == pytest.approx(0.4) and world.poses.at(32, 2500 * MS)[1] == pytest.approx(0.2)
    assert {o[0].id for o in world.obstacles_at(1500 * MS)} == {31, 32}
    # The cube's mesh asset is 5 cm; the entry is 10 cm, so it is scaled by 2: its near face 5 cm from its centre.
    _, _, placed = world.obstacles_at(1500 * MS)[1][1:]
    assert np.ptp(placed[0].vertices, axis=0) == pytest.approx([0.1, 0.1, 0.1])


def test_a_take_that_lacks_what_it_needs_is_unavailable(scenarios, geometry, tmp_path):
    with pytest.raises(C.ClearanceUnavailable, match="VQ"):
        C.compute(read_take(tiny_take(scenarios, vq=False)), geometry)
    take = read_take(os.path.join(scenarios, "mixed", "20261122T100000Z-take001"))
    empty = tmp_path / "empty"
    empty.mkdir()
    C._CACHE.clear()
    with pytest.raises(C.ClearanceUnavailable, match="robot geometry"):
        C.compute(take, str(empty))
    with open(empty / "robot-jetrover.json", "w", encoding="utf-8") as f:
        json.dump(urdf_robot(), f)
    with pytest.raises(C.ClearanceUnavailable, match="test/cube"):
        C.compute(take, str(empty))


def test_a_300_frame_take_with_thirty_objects_takes_seconds_at_most(scenarios, geometry):
    """The build's budget (docs/adr/0038): a few seconds a take. Here every link is near one object or another."""
    import time

    take = read_take(tiny_take(scenarios))
    world = C.TakeWorld(take.path, geometry)
    rng = random.Random(1)
    extra = []
    for k in range(30):
        x, y = rng.uniform(0.1, 0.45), rng.uniform(-0.25, 0.25)
        extra.append(C.Body(100 + k, f"o{k}", [C.box_piece([0.04, 0.04, 0.08])], 1.0, 0.0, 0.05))
        world.poses.t[100 + k] = [0]
        world.poses.p[100 + k] = [(x, y, 0.04, 1.0, 0.0, 0.0, 0.0)]
    world.bodies = world.bodies + extra
    t0 = time.monotonic()
    frames = take.frames * 10  # about 300
    values = {k: v[[world.vq_frame[int(f.timing["frame"])] for f in frames]] for k, v in world.vq.items()}
    centres, axes = world.link_boxes(values, len(frames))
    per = C._box_index(world.robot)
    for fi, f in enumerate(frames):
        obstacles = world.obstacles_at(float(f.timing["sensed"]))
        for li in range(len(world.robot.links)):
            C._nearest([C.Box(centres[fi, b], axes[fi, b], world.robot.box_half[b]) for b in per[li]], obstacles, 0.15)
    assert len(frames) >= 280 and time.monotonic() - t0 < 10.0
