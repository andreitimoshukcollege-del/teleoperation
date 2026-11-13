"""The world in Isaac, end to end (docs/adr/0031 amendment 2, ``just isaac-scene-check``): starts the real bridge with a scene
and talks to it as RobotHost would, over its socket. Run from robot/isaac on the GPU machine; prints one ``RESULT {...}``
line and exits non-zero when any check fails or any number is missing.

1. Welcome, then WORLD_INFO with the scene file's sha256 and every object's id.
2. WORLD at 50 +- 5 Hz, every object at its nominal pose (a movable one settled within 3 mm of it).
3. Placements: one applied, and seen in the next WORLD (PhysX has it, not only USD); off the table, onto another object and
   an unknown id refused.
4. A post moved under the left hand, then the left elbow driven 0.6 rad down through it: the arm stalls on the post, so
   what it measures stays more than 0.05 rad short of the command, and the post does not move. The same motion with the
   post moved away falls short by less than 0.01 rad.
5. While the hand rests on the post: from the headset, ARM_TOUCHING; as a layout, OVERLAPS_ARM.
6. A movable object (docs/adr/0031 amendment 4): the other post under the hand, the same motion knocks it, so it moves
   more than 2 cm or tips more than 15 degrees, and the arm falls short by less than 0.05 rad; placed again, it stands
   upright at rest where it was put (within 3 mm).
7. The bridge's late steps, at most 1 a second over the whole run.

Checks 4 and 5 need an object that does not move, so the bridge runs a copy of the scene with post-left made fixed
(written beside the bridge's log); every other object is as the scene file has it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from teleop_isaac import profile as profile_module  # noqa: E402
from teleop_isaac import scene as scene_module  # noqa: E402
from teleop_isaac import wire  # noqa: E402
from teleop_isaac.bridge import start_pose  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
ELBOW = "left_fr3v2_joint4"
ELBOW_TRAVEL = -0.6  # rad from the start pose: the left tool goes down and in, through (0.34, 0.20) at about z = 0.31
POST_ID = 1  # post-left, 0.30 m tall; made fixed in the copy the bridge runs
KNOCK_ID = 2  # post-right, 0.30 m tall, movable
UNDER_HAND = (0.34, 0.20)


def at(x, y, z=0.15, yaw=0.0):
    return (x, y, z, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


class Client:
    def __init__(self, path: str, server: str):
        if os.path.exists(path):
            os.unlink(path)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(path)
        self.sock.settimeout(0.002)
        self.path, self.server = path, server
        self.sequence = 0
        self.request = 0
        self.world = []  # (arrival s, sensed, snapshot, revision, objects)
        self.placed = {}  # request -> (outcome, revision, pose)
        self.state = None  # {motor: (q, v, shaped)}
        self.health = []
        self.info = None
        self.welcome = None

    def send(self, data: bytes) -> None:
        self.sock.sendto(data, self.server)

    def next(self) -> int:
        self.sequence += 1
        return self.sequence

    def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                d = self.sock.recv(wire.MAX_BYTES)
            except socket.timeout:
                continue
            k = wire.kind_of(d)
            if k == wire.WELCOME:
                self.welcome = wire.decode_welcome(d)
            elif k == wire.WORLD_INFO:
                self.info = wire.decode_world_info(d)
            elif k == wire.WORLD:
                w = wire.decode_world(d)
                if w is not None:
                    self.world.append((time.monotonic(), w[0], w[1], w[2], {o.id: o.pose for o in w[5]}))
            elif k == wire.PLACED:
                p = wire.decode_placed(d)
                if p is not None:
                    self.placed[p[0]] = (p[2], p[3], p[4])
            elif k == wire.STATE:
                s = wire.decode_state(d)
                if s is not None:
                    self.state = {m: (q, v, shaped) for m, q, v, shaped in s[3]}
            elif k == wire.HEALTH:
                h = wire.decode_health(d)
                if h is not None:
                    self.health.append(h)

    def place(self, cause: int, object_id: int, pose) -> int:
        self.request += 1
        r = self.request
        self.send(wire.encode_place(self.next(), time.monotonic_ns(), r, cause, object_id, pose))
        for _ in range(500):
            self.pump(0.01)
            if r in self.placed:
                return self.placed[r][0]
        return -1

    def hold(self, targets, seconds: float) -> None:
        """Sends the targets every 10 ms for ``seconds``, as RobotHost re-sends its newest."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(wire.encode_targets(self.next(), time.monotonic_ns(), list(targets.items())))
            self.pump(0.01)


def main() -> int:
    a = argparse.ArgumentParser()
    a.add_argument("--scene", default=os.path.join(REPO, "core", "Scenes", "fr3-duo-bench.json"))
    a.add_argument("--python", default=os.path.expanduser("~/isaacsim/venv/bin/python"))
    a.add_argument("--cpus", default="2-5", help="taskset for the bridge")
    args = a.parse_args()

    p = profile_module.load(os.path.join(REPO, "core", "RobotProfiles", "fr3-duo.json"))
    with open(args.scene, encoding="utf-8") as f:
        doc = json.load(f)
    for o in doc["objects"]:
        if o["id"] == POST_ID:
            o["kind"] = "fixed"
            o.pop("mass", None)
    scene_path = os.path.expanduser("~/isaacsim/scene-check-scene.json")
    with open(scene_path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
    scene = scene_module.load(scene_path)
    if not any(o.id == KNOCK_ID and o.movable for o in scene.objects):
        print("RESULT " + json.dumps({"passes": False, "missing": ["movable object"], "error": f"object {KNOCK_ID} is not movable in {args.scene}"}))
        return 1
    sha = hashlib.sha256(open(scene_path, "rb").read()).digest()
    motors = p.motor_ids()
    start = dict(zip(motors, start_pose(p)))
    elbow = next(j.motor_id for arm in p.arms for j in arm.joints if j.name == ELBOW)
    nominal = {o.id: o.nominal for o in scene.objects}

    user = os.environ.get("USER", "me")
    server = f"/tmp/teleop-scene-check-{user}.sock"
    mine = f"/tmp/teleop-scene-check-{user}-client.sock"
    log_path = os.path.expanduser("~/isaacsim/scene-check-bridge.log")
    log = open(log_path, "w")
    bridge = subprocess.Popen(["taskset", "-c", args.cpus, args.python, "-m", "teleop_isaac.bridge", "--socket", server,
                               "--scene", scene_path, "--seconds", "260"], cwd=HERE, stdout=log, stderr=subprocess.STDOUT)
    out = {"scene": scene.name, "scene_sha256": sha.hex()}
    checks = {}
    try:
        for _ in range(240):
            if bridge.poll() is not None:
                break
            if "listening on" in open(log_path, errors="replace").read():
                break
            time.sleep(1)
        if not os.path.exists(server):
            tail = open(log_path, errors="replace").read().splitlines()[-30:]
            print("RESULT " + json.dumps({"passes": False, "missing": ["bridge"], "error": "the bridge did not start"}))
            print("\n".join(tail), file=sys.stderr)
            return 1
        c = Client(mine, server)
        t0 = time.monotonic()
        c.send(wire.encode_hello(c.next(), time.monotonic_ns(), 1_000_000_000, p.name))
        c.pump(2.0)
        checks["welcome"] = c.welcome is not None
        checks["world_info"] = c.info is not None and c.info[0] == sha and list(c.info[1]) == [o.id for o in scene.objects]

        c.world.clear()
        c.hold(start, 3.0)
        if len(c.world) >= 2:
            span = c.world[-1][0] - c.world[0][0]
            out["world_hz"] = round((len(c.world) - 1) / span, 2) if span > 0 else None
            sensed = [w[1] for w in c.world]
            out["world_sensed_hz"] = round((len(sensed) - 1) / ((sensed[-1] - sensed[0]) / 1e9), 2)
            checks["world_rate"] = 45.0 <= out["world_sensed_hz"] <= 55.0
            movable = {o.id for o in scene.objects if o.movable}
            out["nominal_err_m"] = max(math.dist(w[4][i][:3], nominal[i][:3]) for w in c.world for i in nominal if i not in movable)
            out["nominal_movable_err_m"] = max(math.dist(w[4][i][:3], nominal[i][:3]) for w in c.world[len(c.world) // 2:] for i in movable)
            checks["world_nominal"] = out["nominal_err_m"] < 1e-4 and out["nominal_movable_err_m"] < 3e-3
        else:
            out["world_hz"] = None

        # 3. placements
        target = at(0.52, 0.32, yaw=0.4)
        outcome = c.place(wire.CAUSE_LAYOUT, POST_ID, target)
        checks["place_applied"] = outcome == wire.APPLIED
        revision = c.placed.get(c.request, (0, 0, None))[1]
        c.world.clear()
        c.hold(start, 0.3)
        seen = [w for w in c.world if w[3] >= revision]
        out["placed_seen_err_m"] = math.dist(seen[-1][4][POST_ID][:3], (0.52, 0.32, scene.table.top + 0.15)) if seen else None
        out["placed_seen_yaw"] = (2 * math.atan2(seen[-1][4][POST_ID][6], seen[-1][4][POST_ID][3])) if seen else None
        checks["place_seen_in_world"] = bool(seen) and out["placed_seen_err_m"] < 1e-4 and abs(out["placed_seen_yaw"] - 0.4) < 1e-3
        checks["refuse_outside"] = c.place(wire.CAUSE_LAYOUT, POST_ID, at(1.5, 0.0)) == wire.OUTSIDE_REGION
        right = nominal[2]
        checks["refuse_overlap_object"] = c.place(wire.CAUSE_LAYOUT, POST_ID, at(right[0], right[1])) == wire.OVERLAPS_OBJECT
        checks["refuse_unknown"] = c.place(wire.CAUSE_LAYOUT, 200, at(0.5, 0.0)) == wire.UNKNOWN_OBJECT

        # 4. the hand into the post, and the same motion without it
        def drive() -> float:
            down = dict(start)
            down[elbow] = start[elbow] + ELBOW_TRAVEL
            c.hold(down, 3.0)
            q = c.state[elbow][0] if c.state else float("nan")
            return abs(down[elbow] - q)

        checks["post_under_hand"] = c.place(wire.CAUSE_LAYOUT, POST_ID, at(*UNDER_HAND)) == wire.APPLIED
        c.world.clear()
        out["blocked_short_rad"] = round(drive(), 4)
        checks["blocked_stalls"] = out["blocked_short_rad"] > 0.05
        moved = [math.dist(w[4][POST_ID][:3], (*UNDER_HAND, scene.table.top + 0.15)) for w in c.world]
        out["post_moved_m"] = max(moved) if moved else None
        checks["post_stays"] = bool(moved) and out["post_moved_m"] < 1e-4

        # 5. while touching
        out["touching_headset_outcome"] = c.place(wire.CAUSE_HEADSET, POST_ID, at(0.5, 0.3))
        checks["refuse_arm_touching"] = out["touching_headset_outcome"] == wire.ARM_TOUCHING
        out["touching_layout_outcome"] = c.place(wire.CAUSE_LAYOUT, POST_ID, at(*UNDER_HAND))
        checks["refuse_overlaps_arm"] = out["touching_layout_outcome"] == wire.OVERLAPS_ARM

        c.hold(start, 3.0)
        checks["post_moved_away"] = c.place(wire.CAUSE_LAYOUT, POST_ID, nominal[POST_ID]) == wire.APPLIED
        out["free_short_rad"] = round(drive(), 4)
        checks["free_tracks"] = out["free_short_rad"] < 0.01
        c.hold(start, 3.0)

        # 6. a movable object: knocked, then stood up again
        checks["knock_under_hand"] = c.place(wire.CAUSE_LAYOUT, KNOCK_ID, at(*UNDER_HAND)) == wire.APPLIED
        c.hold(start, 1.0)
        c.world.clear()
        out["knock_short_rad"] = round(drive(), 4)
        last = c.world[-1][4][KNOCK_ID] if c.world else None
        if last is not None:
            out["knock_moved_m"] = round(math.dist(last[:3], (*UNDER_HAND, scene.table.top + 0.15)), 4)
            up = 1.0 - 2.0 * (last[4] ** 2 + last[5] ** 2)  # the post's own z axis against the vertical
            out["knock_tilt_deg"] = round(math.degrees(math.acos(max(-1.0, min(1.0, up)))), 1)
        else:
            out["knock_moved_m"] = out["knock_tilt_deg"] = None
        checks["knocked"] = last is not None and (out["knock_moved_m"] > 0.02 or out["knock_tilt_deg"] > 15) and out["knock_short_rad"] < 0.05
        c.hold(start, 3.0)
        back = nominal[KNOCK_ID]
        out["restand_outcome"] = c.place(wire.CAUSE_LAYOUT, KNOCK_ID, back)
        c.world.clear()
        c.hold(start, 1.0)
        if c.world:
            now_pose = c.world[-1][4][KNOCK_ID]
            out["restand_err_m"] = round(math.dist(now_pose[:3], back[:3]), 4)
        else:
            out["restand_err_m"] = None
        checks["restands"] = out["restand_outcome"] == wire.APPLIED and out["restand_err_m"] is not None and out["restand_err_m"] < 3e-3

        # 7. late steps
        seconds = time.monotonic() - t0
        if c.health:
            first, last = c.health[0], c.health[-1]
            out["late_steps"] = last[2] - first[2]
            out["late_per_s"] = round(out["late_steps"] / seconds, 3)
            out["real_time_factor_min"] = round(min(h[0] for h in c.health), 4)
            checks["late_ok"] = out["late_per_s"] <= 1.0
        else:
            out["late_steps"] = None
        out["seconds"] = round(seconds, 1)
    finally:
        bridge.terminate()
        try:
            bridge.wait(30)
        except subprocess.TimeoutExpired:
            bridge.kill()
        log.close()
        for path in (mine,):
            if os.path.exists(path):
                os.unlink(path)

    text = open(log_path, errors="replace").read().splitlines()
    out["bridge"] = [line for line in text if line.startswith("[isaac]")][:12]
    out["checks"] = checks
    expected = 20
    out["missing"] = sorted(k for k, v in out.items() if v is None) + ([f"{expected - len(checks)} checks"] if len(checks) < expected else [])
    out["passes"] = not out["missing"] and all(checks.values())
    print("RESULT " + json.dumps(out, sort_keys=True))
    if not out["passes"]:
        print("\n".join(text[-30:]), file=sys.stderr)
    return 0 if out["passes"] else 1


if __name__ == "__main__":
    sys.exit(main())
