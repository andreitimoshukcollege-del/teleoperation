"""The Isaac renderer (docs/adr/0032 decisions 2 and 3): the scene's cameras, rendered beside the physics bridge, as live
feeds on ADR 0014's camera wire.

Run on the GPU machine with Isaac's Python, after the bridge (``just isaac-host`` does both):

    ~/isaacsim/venv/bin/python -m teleop_isaac.renderer --scene ../../core/Scenes/fr3-duo-bench.json --tap /tmp/teleop-isaac-tap.sock

A second, headless Kit process. It opens the bridge's converted robot USD with the importer's ``Physics`` variant set to
``none``, so it is visual only and nothing steps, and builds the scene file's surfaces and objects as visuals. The bridge
sends it a copy of every state and world message (``--tap``); each frame it poses the robot from the newest state (each
moving link's rest pose times its joint's motion, ``kinematics.local_motion``) and the objects from the newest world
message, renders, and hands each camera's colour image to an encoder thread, which sends it as JPEG to whoever
subscribed to that camera's port.

**Frame association.** An image read after posing frame k shows frame k - lag (the spike measured lag 2, stable in every
sample). At start-up the renderer measures the lag with a test cube moved to a new place every frame, and stamps each
frame's ``captureTicks`` with the sensed time of the state it shows: CLOCK_MONOTONIC nanoseconds, RobotHost's clock.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import socket
import sys
import threading
import time
import traceback
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from . import camfeed, camwire, kinematics, wire
from . import profile as profile_module
from . import scene as scene_module

APERTURE_MM = 20.955  # horizontal aperture; the focal length follows from the field of view
# A mount link's frame looks along +x with z up; a USD camera looks along its -z with +y up.
MOUNT_TO_CAMERA = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))  # columns: the camera's x, y, z in the mount frame
LAG_CUBE = 0.06


# --- what needs no Isaac (tests/test_renderer.py) ---------------------------------------------------------------------

class MotorMap:
    """From a state's per-motor values to the URDF's joint values: an arm joint's radians as they are, a hand's 0..1
    opening as each finger's travel (both fingers, the second mimicking the first)."""

    def __init__(self, p: profile_module.Profile):
        self.joints: Dict[int, Tuple[str, ...]] = {}
        self.scale: Dict[int, float] = {}
        for arm in p.arms:
            for j in arm.joints:
                self.joints[j.motor_id] = (j.name,)
                self.scale[j.motor_id] = 1.0
            if arm.gripper is not None:
                g = arm.gripper
                second = g.name[:-1] + "2" if g.name.endswith("1") else None
                self.joints[g.motor_id] = tuple(n for n in (g.name, second) if n)
                self.scale[g.motor_id] = g.finger_travel

    def values(self, motors: Sequence[Tuple[int, float]]) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for m, v in motors:
            for name in self.joints.get(m, ()):
                out[name] = v * self.scale[m]
        return out


def estimate_lag(samples: Sequence[int]) -> Tuple[Optional[int], float]:
    """(the most common lag, the share of samples that agree), or (None, 0) without samples."""
    if not samples:
        return None, 0.0
    mode = max(set(samples), key=list(samples).count)
    return mode, list(samples).count(mode) / len(samples)


def intrinsics(width: int, height: int, hfov_deg: float) -> Dict[str, float]:
    """A pinhole with square pixels, its principal point at the centre and pixel centres at +0.5."""
    fx = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    return {"fx": fx, "fy": fx, "cx": width / 2.0, "cy": height / 2.0}


def look_at(position: Sequence[float], target: Sequence[float], up=(0.0, 0.0, 1.0)) -> Tuple[float, float, float, float]:
    """The orientation (w, x, y, z) of a USD camera at ``position`` looking at ``target``."""
    f = [t - p for t, p in zip(target, position)]
    n = math.sqrt(sum(c * c for c in f))
    z = [-c / n for c in f]
    x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]]
    n = math.sqrt(sum(c * c for c in x))
    x = [c / n for c in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return matrix_to_quat(((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2])))


def matrix_to_quat(m) -> Tuple[float, float, float, float]:
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def mount_camera_orientation() -> Tuple[float, float, float, float]:
    return matrix_to_quat(MOUNT_TO_CAMERA)


class Stamps:
    """The sensed time of the state posed at each frame, so an image read at frame k is stamped with frame k - lag's."""

    def __init__(self, keep: int = 16):
        self._posed: deque = deque(maxlen=keep)

    def posed(self, sensed: int) -> None:
        self._posed.append(sensed)

    def shown(self, lag: int) -> int:
        if lag < 0 or lag >= len(self._posed):
            return camwire.UNSET
        return self._posed[-1 - lag]


# --- the renderer ----------------------------------------------------------------------------------------------------

def parse_args(argv=None) -> argparse.Namespace:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", "..", ".."))
    a = argparse.ArgumentParser(description="the scene's cameras as live feeds (docs/adr/0032)")
    a.add_argument("--scene", required=True)
    a.add_argument("--profile", default=os.path.join(repo, "core", "RobotProfiles", "fr3-duo.json"))
    a.add_argument("--urdf", default=os.path.join(repo, "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf"))
    a.add_argument("--work", default=os.path.expanduser("~/isaacsim/work"), help="where the bridge keeps the URDF's USD")
    a.add_argument("--tap", default="/tmp/teleop-isaac-tap.sock", help="the Unix datagram socket the bridge copies its messages to")
    a.add_argument("--lag", type=int, default=-1, help="the render lag in frames; -1: measure it at start-up")
    a.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: until stopped)")
    a.add_argument("--bind", default="0.0.0.0", help="the address the feeds listen on")
    return a.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    sys.argv = sys.argv[:1]
    p = profile_module.load(args.profile)
    scene = scene_module.load(args.scene)
    cameras = scene.extra.get("cameras") or []
    if not cameras:
        raise SystemExit(f"error: {args.scene} has no cameras")
    fps = max(c.get("fps", 30) for c in cameras)
    joints = kinematics.read_urdf(args.urdf)
    motors = MotorMap(p)

    if os.path.exists(args.tap):
        os.unlink(args.tap)
    tap = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    tap.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    tap.bind(args.tap)
    tap.setblocking(False)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        return run(app, args, p, scene, cameras, fps, joints, motors, tap)
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        tap.close()
        if os.path.exists(args.tap):
            os.unlink(args.tap)
        app.close()


def run(app, args, p, scene, cameras, fps, joints, motors, tap) -> int:
    import io

    import numpy as np
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from importlib.metadata import version
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.rendering_manager import RenderingManager
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdSemantics

    from .bridge import convert_urdf
    from .world import IsaacWorld

    usd = convert_urdf(args.urdf, args.work)
    stage_utils.open_stage(usd)
    while stage_utils.is_stage_loading():
        app.update()
    stage = stage_utils.get_current_stage()
    robot = stage.GetDefaultPrim()
    robot.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
    for _ in range(3):
        app.update()
    stage.SetEditTarget(stage.GetSessionLayer())  # what this process authors never touches the bridge's USD

    world = IsaacWorld(scene, str(robot.GetPath()))
    world.build(stage, physics=False)
    object_ops = {}
    for o, path in zip(scene.objects, world.object_paths):
        ops = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
        object_ops[o.id] = (ops[0], ops[1])
    for light in scene.extra.get("lights") or []:
        if light.get("kind") == "dome":
            UsdLux.DomeLight.Define(stage, f"/World/Lights/{light['id']}").CreateIntensityAttr(float(light.get("intensity", 800)))
        elif light.get("kind") == "distant":
            d = UsdLux.DistantLight.Define(stage, f"/World/Lights/{light['id']}")
            d.CreateIntensityAttr(float(light.get("intensity", 2500)))
            UsdGeom.Xformable(d).AddRotateXYZOp().Set(Gf.Vec3f(*light.get("rotateXYZDeg", [0, 0, 0])))

    # The lag cube, labelled so the head camera's segmentation finds it; hidden once the lag is measured.
    cube = UsdGeom.Cube.Define(stage, "/World/LagCube")
    cube.CreateSizeAttr(1.0)
    cube_xf = UsdGeom.Xformable(cube)
    cube_t = cube_xf.AddTranslateOp()
    cube_xf.AddScaleOp().Set(Gf.Vec3f(LAG_CUBE, LAG_CUBE, LAG_CUBE))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.2, 0.1)])
    UsdSemantics.LabelsAPI.Apply(cube.GetPrim(), "class").CreateLabelsAttr().Set(["lag_cube"])

    links = {x.GetName(): x for x in Usd.PrimRange(robot) if x.GetTypeName() == "Xform"}
    moving = [j for j in joints.values() if j.type in ("revolute", "prismatic", "continuous") and j.child in links]
    view = XformPrim([str(links[j.child].GetPath()) for j in moving])
    rest_t, rest_q = (np.asarray(x.numpy() if hasattr(x, "numpy") else x, float) for x in view.get_local_poses())

    sensors, feeds, specs = [], [], []
    for i, c in enumerate(cameras):
        w, h, hfov = int(c["width"]), int(c["height"]), float(c["hfovDeg"])
        focal = APERTURE_MM / (2 * math.tan(math.radians(hfov) / 2))
        attrs = {"focalLength": focal, "horizontalAperture": APERTURE_MM, "verticalAperture": APERTURE_MM * h / w,
                 "clippingRange": Gf.Vec2f(float(c.get("near", 0.05)), float(c.get("far", 10.0)))}
        if "mount" in c:
            if c["mount"] not in links:
                raise SystemExit(f"error: camera {c['id']}'s mount {c['mount']!r} is not a link of the robot")
            cam = RtxCamera(f"{links[c['mount']].GetPath()}/Cam_{c['id']}", translations=[[0, 0, 0]],
                            orientations=[list(mount_camera_orientation())], attributes=attrs)
        else:
            cam = RtxCamera(f"/World/Cameras/Cam_{c['id']}", positions=[list(c["position"])],
                            orientations=[list(look_at(c["position"], c["lookAt"]))], attributes=attrs)
        annotators = ["rgb", "semantic_segmentation"] if i == 0 else ["rgb"]
        sensors.append((cam, CameraSensor(cam, resolution=(h, w), annotators=annotators)))
        feeds.append(camfeed.CameraFeed(c["id"], int(c["feedPort"]), bind=args.bind))
        specs.append({"id": c["id"], "width": w, "height": h, "quality": int(c.get("feedQuality", 80)), **intrinsics(w, h, hfov)})
    app_utils.play(commit=True)  # sensors deliver only while the timeline plays; with Physics at "none" nothing steps

    def read(sensor, name):
        data, info = sensor.get_data(name)
        if data is None:
            return None, info
        a = data.numpy() if hasattr(data, "numpy") else np.asarray(data)
        return (a[..., 0] if a.ndim == 3 and a.shape[-1] == 1 else a), info

    for k in range(600):
        RenderingManager.render()
        if read(sensors[0][1], "rgb")[0] is not None and k > 5:
            break

    # --- the render lag --------------------------------------------------------------------------------------------
    head_cam, head_sensor = sensors[0]
    hp, hq = (np.asarray(x.numpy() if hasattr(x, "numpy") else x, float).reshape(-1) for x in head_cam.get_world_poses())
    hrot = np.array([[1 - 2 * (hq[2] ** 2 + hq[3] ** 2), 2 * (hq[1] * hq[2] - hq[0] * hq[3]), 2 * (hq[1] * hq[3] + hq[0] * hq[2])],
                     [2 * (hq[1] * hq[2] + hq[0] * hq[3]), 1 - 2 * (hq[1] ** 2 + hq[3] ** 2), 2 * (hq[2] * hq[3] - hq[0] * hq[1])],
                     [2 * (hq[1] * hq[3] - hq[0] * hq[2]), 2 * (hq[2] * hq[3] + hq[0] * hq[1]), 1 - 2 * (hq[1] ** 2 + hq[2] ** 2)]])
    s0 = specs[0]

    def project(x):
        xc = hrot.T @ (np.asarray(x, float) - hp[:3])
        if xc[2] >= -1e-6:
            return None
        return s0["cx"] + s0["fx"] * xc[0] / -xc[2], s0["cy"] - s0["fy"] * xc[1] / -xc[2]

    lag, agree, samples = args.lag, 1.0, []
    if args.lag < 0:
        # On the table where the head camera looks (20-62 degrees down), with the scene's objects hidden so none hides it.
        objects_root = UsdGeom.Imageable(stage.GetPrimAtPath(f"{IsaacWorld.ROOT}/Objects"))
        objects_root.MakeInvisible()
        history = []
        top = scene.table.top
        for k in range(90):
            pos = (0.35 + 0.4 * ((k * 7) % 11) / 10, -0.3 + 0.6 * ((k * 3) % 7) / 6, top + LAG_CUBE / 2 + 0.001)
            cube_t.Set(Gf.Vec3d(*pos))
            history.append(pos)
            RenderingManager.render()
            seg, info = read(head_sensor, "semantic_segmentation")
            ids = {v.get("class"): int(i) for i, v in ((info or {}).get("idToLabels") or {}).items()}
            if seg is None or "lag_cube" not in ids:
                continue
            vv, uu = np.nonzero(seg == ids["lag_cube"])
            if len(uu) < 30:
                continue
            centre = (uu.mean() + 0.5, vv.mean() + 0.5)
            best = None
            for j in range(min(4, k + 1)):
                pr = project(history[k - j])
                if pr is not None:
                    e = math.hypot(pr[0] - centre[0], pr[1] - centre[1])
                    if best is None or e < best[0]:
                        best = (e, j)
            if best is not None and best[0] < 8:
                samples.append(best[1])
        objects_root.MakeVisible()
        lag, agree = estimate_lag(samples)
        if lag is None:
            print("[render] warning: the render lag could not be measured (the cube was never seen); using 2", flush=True)
            lag = 2
    UsdGeom.Imageable(cube).MakeInvisible()
    cube_t.Set(Gf.Vec3d(0.0, 0.0, -5.0))
    print(f"[render] Isaac Sim {version('isaacsim')}; render lag {lag} frame(s) ({agree:.1%} of {len(samples)} samples); "
          f"{len(moving)} moving links", flush=True)

    # --- streaming -------------------------------------------------------------------------------------------------
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    encoders = [ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"jpeg-{s['id']}") for s in specs]
    pending = [0] * len(specs)
    skipped = [0] * len(specs)
    pending_lock = threading.Lock()

    def encode_and_send(i: int, rgb, capture: int) -> None:
        try:
            buf = io.BytesIO()
            Image.fromarray(rgb).save(buf, "JPEG", quality=specs[i]["quality"])
            feeds[i].offer(buf.getvalue(), capture, specs[i]["width"], specs[i]["height"])
        finally:
            with pending_lock:
                pending[i] -= 1

    stamps = Stamps()
    state_sensed, state_values = camwire.UNSET, {}
    states_in = worlds_in = 0
    frame_s = 1.0 / fps
    start = time.perf_counter()
    stop_at = start + args.seconds if args.seconds > 0 else None
    report_at = start + 5.0
    frames = window_frames = 0
    for c, f in zip(specs, feeds):
        print(f"[render] camera {c['id']}: {c['width']}x{c['height']}, fx {c['fx']:.1f}, feed on UDP :{f.port}", flush=True)
    print(f"[render] streaming at {fps} fps", flush=True)
    while not stop and (stop_at is None or time.perf_counter() < stop_at):
        while True:
            try:
                d = tap.recv(wire.MAX_BYTES)
            except (BlockingIOError, InterruptedError):
                break
            k = wire.kind_of(d)
            if k == wire.STATE:
                s = wire.decode_state(d)
                if s is not None and s[0] > state_sensed:
                    state_sensed, state_values = s[0], motors.values([(m, q) for m, q, _, _ in s[3]])
                    states_in += 1
            elif k == wire.WORLD:
                w = wire.decode_world(d)
                if w is not None:
                    worlds_in += 1
                    for o in w[5]:
                        if o.id in object_ops:
                            t_op, q_op = object_ops[o.id]
                            t_op.Set(Gf.Vec3d(*o.pose[:3]))
                            q_op.Set(Gf.Quatd(*o.pose[3:]))
        for f in feeds:
            f.poll()

        ts, qs = rest_t.copy(), rest_q.copy()
        for i, j in enumerate(moving):
            t_i, q_i = kinematics.local_motion(j, tuple(rest_t[i]), tuple(rest_q[i]), state_values.get(j.name, 0.0))
            ts[i], qs[i] = t_i, q_i
        view.set_local_poses(ts, qs)
        stamps.posed(state_sensed)
        RenderingManager.render()
        capture = stamps.shown(lag)
        for i, (_, sensor) in enumerate(sensors):
            if not feeds[i].wanted():
                continue
            with pending_lock:
                behind = pending[i] >= 2
            if behind:
                skipped[i] += 1  # the encoder is behind: drop this frame rather than queue it
                continue
            rgb, _ = read(sensor, "rgb")
            if rgb is None:
                continue
            with pending_lock:
                pending[i] += 1
            encoders[i].submit(encode_and_send, i, np.ascontiguousarray(rgb[..., :3]), capture)
        frames += 1
        window_frames += 1

        now = time.perf_counter()
        if now >= report_at:
            age = (time.monotonic_ns() - state_sensed) / 1e6 if state_sensed != camwire.UNSET else float("nan")
            sent = ", ".join(f"{f.name} {f.frames_sent} sent ({f.bytes_sent / 1e6:.1f} MB)" for f in feeds)
            print(f"[render] {window_frames / (now - report_at + 5.0):.1f} fps; states {states_in}, world {worlds_in}; newest state "
                  f"{age:.0f} ms old; {sent}; skipped {skipped}", flush=True)
            report_at, window_frames = now + 5.0, 0
        wait = start + frames * frame_s - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        elif wait < -0.5:
            start = time.perf_counter() - frames * frame_s  # far behind: don't try to catch up

    for e in encoders:
        e.shutdown(wait=True)
    summary = "; ".join(f"{f.name}: {f.frames_sent} frames sent, {f.frames_paced} paced, {f.frames_failed} failed" for f in feeds)
    print(f"[render] stopped after {frames} frames; {summary}", flush=True)
    for f in feeds:
        f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
