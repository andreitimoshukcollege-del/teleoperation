"""Distances at every command and at made-up poses (docs/adr/0038 L, ``commands.parquet`` and ``pose_samples.parquet``):
the vectorised GJK and engine against clearance.py's own distance, a synthetic take with a wall at a known gap end to end,
the servo path, the samples' kinds, and the sidecars written by a build."""

import json
import math
import os

import numpy as np
import pytest

from conftest import MS, REPO, Sidecar, finish, session_json, write_jlog
from teleop_lerobot import clearance as C
from teleop_lerobot import command_clearance as K
from teleop_lerobot import sidecars
from teleop_lerobot.takes import PULSE_PER_RADIAN, raw_events, read_take
from test_clearance import LEFT_TIP, geometry, octahedron, turned  # noqa: F401  (geometry is a fixture)

PROFILE = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.sim.json")
OFFSET = -0.64494798  # the proximal servo's zeroOffsetRadians, as the real profile has it
JOINTS = ("base_yaw", "distal", "gripper", "gripper_mirror", "proximal", "roll", "wrist")
OPEN = -0.009781  # each finger's travel at the open pulse (jetrover.sim.json)
ZERO = {1: 500.0, 2: 500.0 + OFFSET * PULSE_PER_RADIAN, 3: 500.0, 4: 500.0, 5: 500.0, 10: 116.0}  # the arm at zero, open


def targets(angles=None, speed=2.5, gripper=0.0):
    a = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, **(angles or {})}
    return ";".join(f"{m}:{a[m]}:{speed}" for m in (1, 2, 3, 4)) + f";10:{gripper}:{speed};5:{a[5]}:{speed}"


def pulses(text):
    out = {}
    for it in text.split(";"):
        m, v, _ = it.split(":")
        m, v = int(m), float(v)
        out[m] = (116.0 + min(1, max(0, v)) * (645.0 - 116.0)) if m == 10 else 500.0 + (v + (OFFSET if m == 2 else 0.0)) * PULSE_PER_RADIAN
    return out


def cmd_take(root, commands, gap=0.010, moving=None, name="20261124T100000Z-take001", host_check=None):
    """The arm at zero with its fingers open throughout (VQ and RI), a box wall ``gap`` beyond the left pad's tip along x, a
    cube asset (``moving``: (t0 ms, y0, t1 ms, y1) of its WO poses, else at y 0.4), a parked object, a table and the floor.
    ``commands``: (t ms, outcome, targets); an accepted one gets its RO 1 us after its arrival, a clamped one ``(t ms, "C",
    targets, (sent targets, alpha))`` the sent targets' RO and its JK. ``host_check``: the session's HK fields."""
    take = os.path.join(root, "mixed", name)
    os.makedirs(take)
    sid = session_json(take, "jetrover-session", 1, "isaac", "mixed")
    scene = {"format": "teleop-scene", "version": 2, "name": "tiny", "robotProfile": "jetrover",
             "surfaces": [{"id": "table", "center": [0.35, 0, -0.02], "size": [0.5, 1.0, 0.04], "colour": [0.5, 0.5, 0.5]},
                          {"id": "floor", "center": [0, 0, -1.0], "size": [4, 4, 0.02], "colour": [0.5, 0.5, 0.5]}],
             "objects": [{"id": 31, "name": "wall", "shape": "box", "size": [0.04, 0.2, 0.04], "kind": "fixed"},
                         {"id": 32, "name": "cube", "shape": "mesh", "asset": "test/cube", "size": [0.1, 0.1, 0.1], "kind": "movable"},
                         {"id": 33, "name": "parked", "shape": "box", "size": [0.05, 0.05, 0.05], "kind": "movable"}]}
    with open(os.path.join(take, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(scene, f)
    with open(os.path.join(take, "robot-profile.json"), "w", encoding="utf-8") as f:
        json.dump({"name": "jetrover", "joints": [{"motorId": 1, "role": "BaseYaw"}, {"motorId": 2, "role": "Proximal",
                   "zeroOffsetRadians": OFFSET}, {"motorId": 3, "role": "Distal"}, {"motorId": 4, "role": "Wrist"},
                   {"motorId": 10, "role": "GripperMain"}, {"motorId": 5, "role": "GripperRotate"}]}, f)
    zero = ";".join(f"{m}:{ZERO[m]}:596.831" for m in (1, 2, 3, 4, 10, 5))
    rows = [["RO", 900 * MS, zero], ["KA", 800 * MS, 123_000_000, 10_000_000]]
    if host_check:
        rows.append(["HK", 700 * MS] + list(host_check))
    for seq, (t, outcome, text, *clamp) in enumerate(commands):
        rows.append(["JC", t * MS, t * MS, seq, 50_000_000 + t * 10_000, outcome, text])
        if outcome == "A":
            rows.append(["RO", t * MS + 1000, ";".join(f"{m}:{v}:596.831" for m, v in pulses(text).items())])
        elif outcome == "C":
            rows.append(["RO", t * MS + 1000, ";".join(f"{m}:{v}:596.831" for m, v in pulses(clamp[0][0]).items())])
            rows.append(["JK", t * MS + 2000, seq, clamp[0][1]])
    for k in range(0, 160):
        t = (900 + 20 * k) * MS
        rows.append(["RI", t + 5 * MS, ";".join(f"{m}:1:{ZERO[m]}" for m in (1, 2, 3, 4, 5, 10))])
    wall = f"31:{LEFT_TIP + gap + 0.02}:0:0.035:1:0:0:0"
    parked = "33:17:-15:-4.97:1:0:0:0"
    if moving:
        t0, y0, t1, y1 = moving
        rows.append(["WO", t0 * MS + MS, t0 * MS, 1, f"{wall};32:0.3:{y0}:0.035:1:0:0:0;{parked}"])
        rows.append(["WO", t1 * MS + MS, t1 * MS, 1, f"{wall};32:0.3:{y1}:0.035:1:0:0:0;{parked}"])
    else:
        rows.append(["WO", 100 * MS, 90 * MS, 1, f"{wall};32:0.3:0.4:0.035:1:0:0:0;{parked}"])
    rows.append(["WL", 95 * MS, 1, 777, "31;32"])
    rows.append(["WZ", 95 * MS, 1, "31:1;32:1;33:1"])
    rows.sort(key=lambda r: int(r[1]))
    write_jlog(os.path.join(take, "robothost.jlog"), "robothost", sid, rows)
    side = Sidecar(os.path.join(take, "isaac", "wrist.rgb.frames"))
    frames = [["CM", "wrist", 64, 48, 1, 1, 32, 24, 0.01, 6, 0, 0, 0, 1, 0, 0, 0, "wrist_link", "rgb", "mount"]]
    for n in range(0, 91):
        sensed = 500 * MS + round(n * 1e9 / 30)
        off, length = side.add(f"frame-{n}".encode())
        values = {j: (OPEN if j.startswith("gripper") else 0) for j in JOINTS}
        frames.append(["VQ", n, sensed, ";".join(f"{j}:{values[j]}" for j in JOINTS)])
        frames.append(["VF", sensed + 100 * MS, n, sensed, "", f"wrist.rgb:{off}:{length}"])
    side.close()
    write_jlog(os.path.join(take, "isaac.jlog"), "isaac", sid, frames)
    finish(take)
    return take


COMMANDS = [(950, "S", targets()), (1000, "A", targets()), (1100, "A", targets({1: 0.3})), (1200, "R", targets({1: -0.2, 3: 0.4})),
            (1300, "S", targets({1: 0.1})), (2000, "A", targets())]


# --- the distance itself ---------------------------------------------------------------------------------------------------------

def test_the_vectorised_gjk_is_clearances_gjk():
    rng = np.random.default_rng(7)
    pieces = [C.box_piece([0.05, 0.03, 0.08]), C.piece_from_mesh(*octahedron(0.04)), C.cylinder_piece(0.06, 0.1)]
    boxes, sets, refs = [], [], []
    for k in range(300):
        p = pieces[k % 3]
        R = turned(rng)
        q = p.placed(R, rng.normal(size=3) * 0.06, 1.0)
        b = C.Box(rng.normal(size=3) * 0.02, turned(rng), np.abs(rng.normal(size=3)) * 0.02 + 0.005)
        boxes.append(b)
        sets.append(q)
        refs.append(C.box_piece_distance(b, q))
    vmax = max(len(q.vertices) for q in sets)
    verts = np.array([np.concatenate([q.vertices, np.repeat(q.vertices[:1], vmax - len(q.vertices), axis=0)]) for q in sets])
    c = np.array([b.center for b in boxes])
    d, overlap = K.gjk_batch(c, np.array([b.axes for b in boxes]), np.array([b.half for b in boxes]), verts,
                             c - np.array([q.center for q in sets]))
    refs = np.array(refs)
    apart = refs > 1e-6
    assert apart.sum() > 50 and (~apart).sum() > 50  # both kinds drawn
    assert np.abs(d[apart] - refs[apart]).max() < 2e-6
    assert overlap[refs < -1e-6].all() and not overlap[refs > 1e-6].any()


def test_the_engine_gives_clearances_distances_for_any_pose(scenarios, geometry):
    take = read_take(cmd_take(scenarios, COMMANDS, moving=(900, 0.4, 1400, 0.06)))
    world = C.TakeWorld(take.path, geometry)
    engine, arm = K.Engine(world), K.Arm(take.path)
    rng = np.random.default_rng(3)
    poses = np.array([ZERO[m] for m in (1, 2, 3, 4, 5, 10)]) + rng.normal(size=(200, 6)) * np.array([60, 60, 120, 120, 200, 100])
    poses = np.clip(poses, arm.lo, arm.hi)
    times = rng.uniform(900, 1500, size=200) * MS
    scenes = np.array([engine.scene(float(t)) for t in times])
    res = engine.distances(arm.values(poses), scenes)
    least = engine.distances(arm.values(poses), scenes, min_only=True)
    centres, axes = world.link_boxes(arm.values(poses), len(poses))
    per = C._box_index(world.robot)
    for i in range(len(poses)):
        obstacles = world.obstacles_at(float(times[i]))
        for li in range(len(world.robot.links)):
            boxes = [C.Box(centres[i, b], axes[i, b], world.robot.box_half[b]) for b in per[li]]
            d, oid = C._nearest(boxes, obstacles, 0.15)
            assert res.obstacle[i, li] == pytest.approx(min(d, 0.15), abs=1e-9)
            assert res.obstacle_id[i, li] == (oid if d < 0.15 else C.NO_OBSTACLE)
            if world.robot.links[li] in world.robot.fixed:
                assert math.isnan(res.surface[i, li])
            else:
                s = C.surface_distance(np.concatenate([b.corners() for b in boxes]), world.surfaces)
                assert res.surface[i, li] == pytest.approx(min(s, 0.15), abs=1e-9)
    moving = ~engine.fixed  # the nearest of the links a joint moves: not the base
    assert least.obstacle[:, 0] == pytest.approx(res.obstacle[:, moving].min(axis=1), abs=1e-12)
    near = least.obstacle[:, 0] < 0.15
    assert near.sum() > 20 and (res.obstacle[near, least.link[near]] == least.obstacle[near, 0]).all()
    assert (res.obstacle < 0.15).any() and (res.obstacle < 0).any()  # near and overlapping poses both drawn


def test_pulses_become_the_twins_joint_values(scenarios):
    arm = K.Arm(cmd_take(scenarios, COMMANDS))
    v = arm.values(np.array([[ZERO[m] for m in (1, 2, 3, 4, 5, 10)], [600, 500, 400, 700, 300, 645]]))
    assert v["proximal"][0] == pytest.approx(0, abs=1e-7) and v["gripper"][0] == pytest.approx(OPEN)
    assert v["gripper_mirror"][1] == pytest.approx(0.016241)  # closed: the table's last travel
    assert v["base_yaw"][1] == pytest.approx(100 / PULSE_PER_RADIAN) and v["proximal"][1] == pytest.approx(-OFFSET)
    assert v["roll"][1] == pytest.approx(-200 / PULSE_PER_RADIAN)
    # Limits: the URDF's within the pulse range (the proximal's shifted by its offset).
    assert arm.lo[0] == 0 and arm.hi[0] == 1000
    assert arm.lo[1] == pytest.approx(500 + (-0.5236 + OFFSET) * PULSE_PER_RADIAN)
    assert (arm.lo[5], arm.hi[5]) == (116, 645)


def test_the_way_there_ramps_each_servo_at_its_own_speed():
    start = np.array([[500.0, 500, 500, 500, 500, 116]])
    cmd = np.array([[600.0, 450, 500, 500, 500, 116]])
    pps = np.array([[100.0, 200, 100, 100, 100, 100]])  # servo 1 needs 1 s, servo 2 0.25 s
    path, T = K.path_poses(start, cmd, pps)
    assert T[0] == pytest.approx(1.0) and path.shape == (1, K.PATH_STEPS, 6)
    assert path[0, :, 0] == pytest.approx(500 + 100 * np.arange(1, 9) / 8)
    assert path[0, :, 1] == pytest.approx([475, 450, 450, 450, 450, 450, 450, 450])
    assert (path[0, -1] == cmd[0]).all()
    jump, T = K.path_poses(start, cmd, np.zeros((1, 6)))  # no speed: the servo jumps its setpoint
    assert T[0] == 0 and (jump[0] == cmd[0]).all()


# --- a take end to end -------------------------------------------------------------------------------------------------------------

def rows_of(c):
    """commands.parquet's columns of one episode as a DataFrame, as the pilot check reads them."""
    pd = pytest.importorskip("pandas")
    names = list(K.SERVO_NAMES) + ["sent." + n for n in K.SERVO_NAMES]
    return pd.DataFrame({"episode_index": 0, "received_ns": c["received_ns"],
                         "sent_ns": pd.array(c["sent_ns"], dtype="Int64"), **{n: c[n] for n in names}})


def frames_of(take):
    pd = pytest.importorskip("pandas")
    return pd.DataFrame({"episode_index": 0, "frame_index": range(len(take.frames)),
                         "sensed_ns": [f.timing["sensed"] for f in take.frames], "action": [f.action for f in take.frames]})


def test_every_command_robothost_judged_gets_its_row_and_its_distances(scenarios, geometry):
    path = cmd_take(scenarios, COMMANDS)
    take = read_take(path)
    t = K.compute(take, geometry, layout_seed=777)
    c = t.commands
    # One row per command RobotHost judged (not the stale ones, not RobotHost's own RO before any command), at its arrival.
    assert t.unavailable is None and len(c["received_ns"]) == 4
    assert list(c["received_ns"]) == [1000 * MS, 1100 * MS, 1200 * MS, 2000 * MS]
    assert c["outcome"] == ["A", "A", "R", "A"] and list(c["accepted"]) == [True, True, False, True]
    assert c["seq"] == [1, 2, 3, 5]
    # servo_*: the command received (an accepted one's RO exactly; the refused one as the plant converts it). sent.*: what went to
    # the servos, the refused one's the RO in force (the 1.1 s command's), as the dataset's action and raw events have it.
    received = [[c[n][i] for n in K.SERVO_NAMES] for i in range(4)]
    assert received[2] == pytest.approx([pulses(COMMANDS[3][2])[m] for m in (1, 2, 3, 4, 5, 10)])
    sent = [[c["sent." + n][i] for n in K.SERVO_NAMES] for i in range(4)]
    raw = [e for e in raw_events(take) if e["type"] == "action"]
    by_time = {e["pc_received_mono_ns"]: e["values"] for e in raw}
    assert c["sent_ns"] == [1000 * MS + 1000, 1100 * MS + 1000, 1100 * MS + 1000, 2000 * MS + 1000]
    assert sent == [by_time[ns] for ns in c["sent_ns"]] and sent[2] == sent[1] == received[1]
    assert [received[i] == sent[i] for i in range(4)] == [True, True, False, True]
    assert list(c["clamp_fraction"]) == [1.0, 1.0, 0.0, 1.0]
    for i in range(4):
        assert [c["state." + n][i] for n in K.SERVO_NAMES] == pytest.approx([ZERO[m] for m in (1, 2, 3, 4, 5, 10)])
    frame_t = [f.timing["sensed"] for f in take.frames]
    assert c["frame_index"][0] == 0  # at the first frame (1000 ms)
    assert c["frame_index"][1] == max(i for i, ft in enumerate(frame_t) if ft <= 1100 * MS)
    # The pilot check: each frame's action is the newest command sent at or before the frame's capture.
    rows, frames = rows_of(c), frames_of(take)
    assert K.action_agreement(rows, frames) == (len(frame_t), len(frame_t), [])
    frames.at[3, "action"] = [v + 1 for v in frames.at[3, "action"]]
    assert K.action_agreement(rows, frames) == (len(frame_t) - 1, len(frame_t), [(0, 3)])
    # The received command is not the action: the refused one's frames would disagree.
    wrong = rows.copy()
    for n in K.SERVO_NAMES:
        wrong["sent." + n] = wrong[n]
    assert K.action_agreement(wrong, frames_of(take))[0] < len(frame_t)
    # The sent pose's distances: an accepted command's are its own, a refused one sent nothing.
    for name in ("min_clearance_mm", "path_min_clearance_mm"):
        got, own = np.asarray(c["sent_" + name]), np.asarray(c[name])
        assert got[[0, 1, 3]] == pytest.approx(own[[0, 1, 3]]) and np.isnan(got[2])
    # The commanded zero pose is the frames' pose: the same distances as the per-frame clearance, link by link.
    frame = C.compute(take, geometry)
    for li, link in enumerate(frame.links):
        assert c[f"clearance_mm.{link}"][0] == frame.obstacle_mm[0][li]  # row 0: the first accepted command, the zero pose
        assert c[f"clearance_id.{link}"][0] == frame.obstacle_id[0][li]
        assert c[f"surface_mm.{link}"][0] == pytest.approx(frame.surface_mm[0][li], nan_ok=True)
    assert c["min_clearance_mm"][0] == pytest.approx(10.0, abs=0.01) and c["min_clearance_id"][0] == 31
    assert c["min_clearance_link"][0] == "left_finger" and c["min_surface_mm"][0] == pytest.approx(min(frame.surface_mm[0][1:]))
    # Turning the yaw 0.3 rad: 0.12 s at 2.5 rad/s, the way there starting next to the wall and leaving it.
    assert c["path_s"][1] == pytest.approx(0.3 * PULSE_PER_RADIAN / 596.831, abs=1e-6) and c["path_s"][0] == 0
    assert c["path_s"][2] == pytest.approx(0.4 / 2.5, abs=1e-6)  # refused: its own speed, the elbow's 0.4 rad the longest
    assert c["path_min_clearance_mm"][1] < c["min_clearance_mm"][1] and 1 <= c["path_min_step"][1] < K.PATH_STEPS
    assert c["path_min_clearance_id"][1] == 31 and c["path_min_clearance_link"][1] in ("left_finger", "right_finger")
    engine, arm = K.Engine(C.TakeWorld(path, geometry)), K.Arm(path)
    start = np.array([[ZERO[m] for m in (1, 2, 3, 4, 5, 10)]])
    want, _ = K.path_poses(start, np.array([[c[n][1] for n in K.SERVO_NAMES]]), np.full((1, 6), 596.831))  # the RO's rate
    res = engine.distances(arm.values(want[0]), [engine.scene(1100 * MS)] * K.PATH_STEPS)
    assert c["path_min_clearance_mm"][1] == pytest.approx(round(res.obstacle.min() * 1000, 2))
    assert c["path_min_surface_mm"][1] == pytest.approx(round(np.nanmin(res.surface) * 1000, 2))
    pm, m = np.asarray(c["path_min_clearance_mm"]), np.asarray(c["min_clearance_mm"])
    assert (pm <= m + 1e-9).all()


def test_a_clamped_command_has_what_was_sent_and_how_far(scenarios, geometry):
    # RobotHost clamping (HK measured clamp 1): the 1.1 s command, a yaw turn of 0.3 rad, sent 40% of the way, a 1.2 s one refused.
    commands = [(1000, "A", targets()), (1100, "C", targets({1: 0.3}), (targets({1: 0.12}), 0.4)),
                (1200, "R", targets({1: -0.2, 3: 0.4})), (2000, "A", targets())]
    path = cmd_take(scenarios, commands, host_check=("measured", "clamp", 1))
    take = read_take(path)
    c = K.compute(take, geometry, layout_seed=777).commands
    assert c["outcome"] == ["A", "C", "R", "A"] and list(c["accepted"]) == [True, False, False, True]
    assert list(c["clamp_fraction"]) == [1.0, 0.4, 0.0, 1.0]
    clamped = [pulses(targets({1: 0.12}))[m] for m in (1, 2, 3, 4, 5, 10)]
    assert [c[n][1] for n in K.SERVO_NAMES] == pytest.approx([pulses(targets({1: 0.3}))[m] for m in (1, 2, 3, 4, 5, 10)])
    assert [c["sent." + n][1] for n in K.SERVO_NAMES] == clamped and [c["sent." + n][2] for n in K.SERVO_NAMES] == clamped
    assert c["sent_ns"][1] == c["sent_ns"][2] == 1100 * MS + 1000
    # The clamped point's distances, and the way there from the measured state at its RO's rate.
    engine, arm = K.Engine(C.TakeWorld(path, geometry)), K.Arm(path)
    moving = ~engine.fixed
    pose = np.array([clamped])
    res = engine.distances(arm.values(pose), [engine.scene(1100 * MS)])
    assert c["sent_min_clearance_mm"][1] == pytest.approx(round(res.obstacle[0, moving].min() * 1000, 2), abs=0.011)
    way, _ = K.path_poses(np.array([[ZERO[m] for m in (1, 2, 3, 4, 5, 10)]]), pose, np.full((1, 6), 596.831))
    res = engine.distances(arm.values(way[0]), [engine.scene(1100 * MS)] * K.PATH_STEPS)
    assert c["sent_path_min_clearance_mm"][1] == pytest.approx(round(res.obstacle[:, moving].min() * 1000, 2), abs=0.011)
    assert c["sent_min_clearance_mm"][1] != c["min_clearance_mm"][1]  # not the received command's
    assert np.isnan(c["sent_min_clearance_mm"][2]) and c["sent_min_clearance_mm"][0] == c["min_clearance_mm"][0]
    # The action is the clamped point, and the pilot check holds.
    assert K.action_agreement(rows_of(c), frames_of(take))[:2] == (len(take.frames),) * 2
    e = sidecars.episode(take, sidecars.Options(geometry_dir=geometry))
    assert e.line["hostCheck"] == {"start": "measured", "onFail": "clamp", "version": 1}


def test_a_refused_commands_received_values_need_robothosts_own_conversion(scenarios):
    from conftest import jetrover_take, relay_pulse
    from teleop_lerobot.takes import TakeError

    motion = {1: 0.1, 2: 0.3, 3: -1.2, 4: 0.0, 10: 0.2, 5: 0.0}
    cmds = [(ms, "R" if 2000 <= ms < 2100 else "A", motion) for ms in range(1007, 3001, 20)]
    off = lambda m, v: relay_pulse(m, v) + (1.0 if m == 3 else 0.0)
    take = read_take(jetrover_take(scenarios, commands=cmds, relay=off))
    with pytest.raises(TakeError, match="conversion has changed"):
        K.compute(take, None)
    e = sidecars.episode(take, sidecars.Options(geometry_dir=None))
    assert e.commands is None and "conversion has changed" in e.line["commandClearanceUnavailable"]


def test_a_command_into_the_wall_reads_negative(scenarios, geometry):
    t = K.compute(read_take(cmd_take(scenarios, COMMANDS, gap=-0.005)), geometry, layout_seed=777)
    c = t.commands
    assert c["min_clearance_mm"][1] == pytest.approx(-5.0, abs=0.01) and c["min_clearance_id"][1] == 31
    assert c["path_min_clearance_mm"][1] == pytest.approx(-5.0, abs=0.01)


def test_with_nothing_within_the_cutoff_no_link_is_named(scenarios, geometry):
    # Until 2026-11-25 such a row's min_clearance_link was the first moving link, the turntable, with no object.
    t = K.compute(read_take(cmd_take(scenarios, COMMANDS, gap=0.2)), geometry, layout_seed=777)
    for name, cols in (("commands", t.commands), ("pose_samples", t.samples)):
        ids, links = np.asarray(cols["min_clearance_id"]), cols["min_clearance_link"]
        mm = np.asarray(cols["min_clearance_mm"])
        far = ids < 0
        assert far.any() and (name == "commands" or (~far).any()), name  # every command far; some samples near
        assert all(link is None for link, f in zip(links, far) if f) and (mm[far] == C.CUTOFF_MM).all(), name
        assert all(link is not None for link, f in zip(links, far) if not f), name
        if "path_min_clearance_id" in cols:
            pids, plinks = np.asarray(cols["path_min_clearance_id"]), cols["path_min_clearance_link"]
            assert all((link is None) == (i < 0) for link, i in zip(plinks, pids)), name


def test_object_poses_are_the_ones_at_each_commands_arrival(scenarios, geometry):
    commands = [(1000, "A", targets()), (1300, "A", targets()), (1350, "A", targets()), (2000, "A", targets())]
    path = cmd_take(scenarios, commands, gap=0.2, moving=(1000, 0.4, 1400, 0.06))
    c = K.compute(read_take(path), geometry, layout_seed=1).commands
    world = C.TakeWorld(path, geometry)
    links = world.robot.links
    seen = 0
    for i, t in enumerate(c["received_ns"]):
        for li, link in enumerate(links):
            if c[f"clearance_id.{link}"][i] == 32:
                want = C.link_object_distance(world, int(t), li, 32)
                assert c[f"clearance_mm.{link}"][i] == pytest.approx(round(want * 1000, 2), abs=0.011)
                seen += 1
    assert seen >= 2 and c["min_clearance_mm"][1] > c["min_clearance_mm"][2]  # the cube closing in


def test_the_pose_samples_of_every_frame(scenarios, geometry):
    path = cmd_take(scenarios, COMMANDS)
    take = read_take(path)
    t = K.compute(take, geometry, layout_seed=777)
    s, nf, total = t.samples, len(take.frames), sum(n for _, n in K.SAMPLES)
    assert len(s["kind"]) == nf * total and s["kind"][:total] == [k for k, n in K.SAMPLES for _ in range(n)]
    assert list(s["frame_index"][:total]) == [0] * total and list(s["sample"][:total]) == list(range(total))
    arm = K.Arm(path)
    poses = np.array([s[n] for n in K.SERVO_NAMES]).T
    kinds = np.array(s["kind"])
    state = np.array([f.state for f in take.frames])
    q = np.repeat(state, total, axis=0)
    assert (poses >= arm.lo - 1e-3).all() and (poses <= arm.hi + 1e-3).all()
    b = kinds == "beyond"
    scale = np.asarray(s["scale"])
    assert ((scale[b] >= 1.5) & (scale[b] <= 3.0)).all() and np.isnan(scale[~b]).all()
    # Frame 2's action (1133 ms, 15 fps from 1000 ms) is the yaw 0.3 turn: beyond it, 1.5 to 3 times as far.
    assert take.frames[2].action == pytest.approx([pulses(targets({1: 0.3}))[m] for m in (1, 2, 3, 4, 5, 10)])
    i3 = 2 * total + np.flatnonzero(b[:total])
    cmd = np.array([pulses(targets({1: 0.3}))[m] for m in (1, 2, 3, 4, 5, 10)])
    assert poses[i3] == pytest.approx(np.clip(state[2] + scale[i3][:, None] * (cmd - state[2]), arm.lo, arm.hi), abs=1e-3)
    r = kinds == "reachable"
    reach = np.array([v * PULSE_PER_RADIAN * K.REACH_SECONDS for v in K.CAPS_RAD_S] + [K.GRIPPER_PPS * K.REACH_SECONDS])
    assert (np.abs(poses[r] - q[r]) <= reach + 1e-3).all()
    near = kinds == "near"
    d, target = np.asarray(s["min_clearance_mm"]), np.asarray(s["target_mm"])
    assert ((target[near] >= 0) & (target[near] <= 50)).all() and np.isnan(target[~near]).all()
    landed = (d[near] >= 0) & (d[near] <= 50)
    assert landed.mean() > 0.75, landed.mean()
    # Here the arm starts 10 mm from the wall, so a line often has both ends on one side (the real takes: 86% within).
    assert (np.abs(d[near] - target[near]) <= K.NEAR_TOLERANCE_MM + 0.01).mean() > 0.4
    # The way to each sample from the frame's measured pose (world-models' request, 2026-11-25), as a command's: 8 poses at the
    # rate of the command in force (the frame's action, an RO at 596.831 pulses a second), the last the sample itself.
    pm, step = np.asarray(s["path_min_clearance_mm"]), np.asarray(s["path_min_step"])
    assert np.isfinite(pm).all() and ((step >= 1) & (step <= K.PATH_STEPS)).all()
    assert (pm <= d + 1e-9).all() and (pm < d - 1).any()  # never farther than the sample itself, nearer on some ways
    engine, i = K.Engine(C.TakeWorld(path, geometry)), int(np.argmax(d - pm))
    fi = i // total
    want, T = K.path_poses(state[fi][None], poses[i][None], np.full((1, 6), 596.831))
    assert s["path_s"][i] == pytest.approx(T[0], abs=1e-6)
    res = engine.distances(arm.values(want[0]), [engine.scene(take.frames[fi].timing["sensed"])] * K.PATH_STEPS)
    moving = ~engine.fixed
    assert pm[i] == pytest.approx(round(res.obstacle[:, moving].min() * 1000, 2), abs=0.011)
    assert s["path_min_surface_mm"][i] == pytest.approx(round(np.nanmin(res.surface) * 1000, 2), abs=0.011)
    assert (s["path_min_clearance_link"][i] is None) == (s["path_min_clearance_id"][i] < 0)
    # Seeded: the same take draws the same poses; another session other ones.
    again = K.compute(take, geometry, layout_seed=777).samples
    assert all(np.array_equal(np.asarray(again[n]), np.asarray(s[n]), equal_nan=True) for n in K.SERVO_NAMES)
    assert t.seed == K.seed_of(777, take.session_id) != K.seed_of(777, "another") and t.seed < 2 ** 53


def test_without_geometry_the_rows_stay_and_the_distances_are_nan(scenarios):
    take = read_take(cmd_take(scenarios, COMMANDS))
    t = K.compute(take, None, layout_seed=777)
    assert t.unavailable == "no geometry folder given"
    assert len(t.commands["seq"]) == 4 and np.isnan(t.commands["min_clearance_mm"]).all()
    assert np.isfinite(np.asarray(t.commands["servo_1"])).all()
    kinds = np.array(t.samples["kind"])
    poses = np.array([t.samples[n] for n in K.SERVO_NAMES]).T
    assert np.isnan(poses[kinds == "near"]).all() and np.isfinite(poses[kinds != "near"]).all()
    assert K.compute(take, "/nowhere").unavailable == "no geometry at /nowhere"


def test_a_build_writes_both_files_and_safety_report_sums_them_up(scenarios, geometry, tmp_path):
    pd = pytest.importorskip("pandas")
    pq = pytest.importorskip("pyarrow.parquet")
    take = read_take(cmd_take(scenarios, COMMANDS, gap=-0.005))
    e = sidecars.episode(take, sidecars.Options(geometry_dir=geometry))
    assert e.line["poseSampleSeed"] == K.seed_of(777, take.session_id) and e.line["commandClearanceUnavailable"] is None
    root = str(tmp_path / "ds")
    rep = sidecars.write(root, [(4, e)], sidecars.Options(geometry_dir=geometry))
    assert rep["options"]["commandClearance"] is True
    # A take without HK (every take before RobotHost's clamping): its RobotHost refused a failing command.
    assert e.line["hostCheck"] == {"start": "measured", "onFail": "refuse"}
    assert rep["options"]["hostCheck"] == {"start": "measured", "onFail": "refuse"}
    teleop = os.path.join(root, "meta", "teleop")
    cmd = pd.read_parquet(os.path.join(teleop, "commands.parquet"))
    smp = pd.read_parquet(os.path.join(teleop, "pose_samples.parquet"))
    assert len(cmd) == 4 and set(cmd["episode_index"]) == {4} and len(smp) == len(take.frames) * 32
    assert str(cmd["frame_index"].dtype) == "Int64" and cmd["frame_index"].isna().sum() == 0
    assert str(cmd["min_clearance_id"].dtype) == "Int32" and cmd["clearance_id.base_link"].isna().all()  # none within the cutoff
    assert cmd["accepted"].dtype == bool and list(cmd["outcome"]) == ["A", "A", "R", "A"]
    assert str(cmd["seq"].dtype) == "Int64" and cmd["seq"].isna().sum() == 0 and str(cmd["sent_ns"].dtype) == "Int64"
    assert {str(cmd[c].dtype) for c in ("clamp_fraction",) + tuple("sent." + n for n in K.SERVO_NAMES)} == {"float64"}
    # Every distance column is a float64 in every sidecar, NaN for missing: until 2026-11-25 clearance_mm.<link> and
    # surface_mm.<link> of the links whose names end in "_link" were text in these two files.
    frames = pd.read_parquet(os.path.join(teleop, "frames.parquet"))
    for name, df in (("commands", cmd), ("pose_samples", smp), ("frames", frames)):
        distance = [c for c in df.columns if "_mm" in c]
        assert len(distance) >= 18, (name, distance)
        assert {c: str(df[c].dtype) for c in distance if str(df[c].dtype) != "float64"} == {}, name
        assert str(df["clearance_mm.base_link"].dtype) == "float64" and df["surface_mm.base_link"].isna().all()
    assert {str(cmd[c].dtype) for c in K.SERVO_NAMES + tuple("state." + n for n in K.SERVO_NAMES) + ("path_s",)} == {"float64"}
    meta = json.loads(pq.read_schema(os.path.join(teleop, "commands.parquet")).metadata[b"teleop"])
    assert meta["pathSteps"] == K.PATH_STEPS and "received_ns" in meta["clocks"]
    assert json.loads(pq.read_schema(os.path.join(teleop, "pose_samples.parquet")).metadata[b"teleop"])["samples"]["near"] == 4
    data = os.path.join(root, "data", "chunk-000")
    os.makedirs(data)
    pd.DataFrame({"episode_index": 4, "frame_index": range(len(take.frames)), "action": [f.action for f in take.frames]}) \
        .to_parquet(os.path.join(data, "file-000.parquet"))
    lines, faults = [], []
    sidecars.command_line(teleop, "ds", [{**e.line, "episode": 4}], lines, faults)
    assert faults == [] and "commands.parquet 4 rows (1 episodes)" in lines[0] and "commanded pose into an obstacle" in lines[0]
    assert "near median" in lines[0] and "outcomes A 3, C 0, R 1 (clamped 0.0%)" in lines[0]
    assert f"the newest command sent at or before it in {len(take.frames)} of {len(take.frames)} frames" in lines[0]
    pd.DataFrame({"episode_index": 4, "frame_index": range(len(take.frames)), "action": [[0.0] * 6] * len(take.frames)}) \
        .to_parquet(os.path.join(data, "file-000.parquet"))
    sidecars.command_line(teleop, "ds", [{**e.line, "episode": 4}], lines, faults)
    assert len(faults) == 1 and "the action is not the newest command sent" in faults[0]
    faults.clear()
    off = sidecars.episode(take, sidecars.Options(geometry_dir=geometry, command_clearance=False))
    assert off.commands is None and "poseSampleSeed" not in off.line
    root2 = str(tmp_path / "off")
    sidecars.write(root2, [(0, off)], sidecars.Options(geometry_dir=geometry, command_clearance=False))
    assert not os.path.exists(os.path.join(root2, "meta", "teleop", "commands.parquet"))
    lines = []
    sidecars.command_line(os.path.join(root2, "meta", "teleop"), "off", [off.line], lines, faults)
    assert lines == ["off: no command distances (built without them)"] and faults == []
    os.remove(os.path.join(teleop, "pose_samples.parquet"))
    sidecars.command_line(teleop, "ds", [{**e.line, "episode": 4}], lines, faults)
    assert faults == ["ds: built with command distances, but pose_samples.parquet missing"]
