"""The render spike on the GPU machine (docs/adr/0032 phase P0, `just isaac-render-check`).

Measures what the choice of renderer rests on, and prints one JSON line (``RESULT {...}``). It exits 0 only
when every field was measured: a missing number is a failed check, never a pass (CLAUDE.md invariant 10).

Two modes:

- ``renderer``: a visual-only copy of the robot (the importer's ``Physics`` variant set to ``none``, so
  nothing can step), posed from joint angles by rotating each moving link about its URDF axis, with the
  phase-2 scene's table, backdrop surfaces and a test cube, and two RTX cameras (the head camera on the
  URDF's ``head_camera_mounting_point``, and a third-person camera). It measures frame time, posing, readback,
  the render lag (how many frames after posing a pose shows up), masks, depth against the table plane,
  FK against the rendered robot, encode sizes, and how long a backdrop texture takes to appear.
- ``inprocess``: the bridge's physics (1 kHz, results to USD off) with the head camera rendered every
  1/30 s in the same loop: how late the physics runs, and whether the render sees the arm move.

The script is sent to Isaac's Python on stdin, as robot/isaac/check.py is; it imports nothing from the repo.
"""

import argparse
import glob
import json
import math
import os
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

# Two real poses from the 2026-11-01 Isaac sessions (SerialChainTeleopTests replays them): both hands in
# front of the robot, where the head camera looks.
LEFT_POSE = [-0.0882, -0.7412, -0.2330, -2.6143, 2.7212, 2.1025, 0.5537]
RIGHT_POSE = [-0.0578, -0.7210, -0.1975, -2.4718, 1.5825, 4.3749, -2.0]

TABLE = {"center": (0.55, 0.0, -0.02), "size": (0.70, 1.60, 0.04)}       # x 0.20-0.90, top at z = 0
SCREEN = {"center": (1.30, 0.0, -0.10), "size": (2.40, 1.00)}            # beyond the table's far edge
CUBE = 0.06                                                              # the association test cube, m


# --- rotations (quaternions are w, x, y, z) -------------------------------------------------------------

def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def q_axis(axis, angle):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    s = math.sin(angle / 2) / n
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def q_rpy(r, p, y):
    return q_mul(q_axis((0, 0, 1), y), q_mul(q_axis((0, 1, 0), p), q_axis((1, 0, 0), r)))


def q_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def q_from_matrix(m):
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def camera_rotation(forward, up):
    """The USD camera's orientation (it looks along its -Z, +Y up) that looks along ``forward``."""
    f = np.asarray(forward, float)
    f /= np.linalg.norm(f)
    z = -f
    x = np.cross(np.asarray(up, float), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return q_from_matrix(np.column_stack([x, y, z]))


# --- the URDF -------------------------------------------------------------------------------------------

def read_urdf(path):
    joints = {}
    for j in ET.parse(path).getroot().findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        a = j.find("axis")
        axis = [float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()]
        joints[j.get("name")] = {"type": j.get("type"), "parent": j.find("parent").get("link"),
                                 "child": j.find("child").get("link"), "xyz": xyz, "rpy": rpy, "axis": axis}
    return joints


def fk(joints, angles, link):
    """World pose (position, quaternion) of ``link`` for joint ``angles`` (name -> rad or m): the URDF chain."""
    by_child = {j["child"]: (n, j) for n, j in joints.items()}
    chain = []
    while link in by_child:
        n, j = by_child[link]
        chain.append((n, j))
        link = j["parent"]
    p, q = np.zeros(3), (1.0, 0.0, 0.0, 0.0)
    for n, j in reversed(chain):
        p = p + q_matrix(q) @ np.asarray(j["xyz"])
        q = q_mul(q, q_rpy(*j["rpy"]))
        v = angles.get(n, 0.0)
        if j["type"] in ("revolute", "continuous"):
            q = q_mul(q, q_axis(j["axis"], v))
        elif j["type"] == "prismatic":
            p = p + q_matrix(q) @ (np.asarray(j["axis"]) * v)
    return p, q


def pose_at(t, names):
    """Joint angles at time ``t``: the base poses, joints 1, 2 and 4 swinging by up to 0.15 rad."""
    out = {}
    for side, base in (("left", LEFT_POSE), ("right", RIGHT_POSE)):
        for k in range(7):
            swing = 0.15 * math.sin(2 * math.pi * (0.25 + 0.05 * k) * t) if k in (0, 1, 3) else 0.0
            out[f"{side}_fr3v2_joint{k + 1}"] = base[k] + swing
        out[f"{side}_fr3v2_finger_joint1"] = 0.02 + 0.015 * math.sin(2 * math.pi * 0.5 * t)
        out[f"{side}_fr3v2_finger_joint2"] = out[f"{side}_fr3v2_finger_joint1"]
    return {n: out.get(n, 0.0) for n in names}


def pct(xs, p):
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


def own_gpu_mib():
    try:
        out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout
        for line in out.splitlines():
            pid, mem = (x.strip() for x in line.split(","))
            if int(pid) == os.getpid():
                return int(mem)
    except Exception:
        pass
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="the render spike (docs/adr/0032 P0)")
    ap.add_argument("--mode", choices=("renderer", "inprocess"), default="renderer")
    ap.add_argument("--usd", default="", help="the bridge's converted robot USD (default: the newest under ~/isaacsim/work)")
    ap.add_argument("--urdf", default=os.path.expanduser("~/teleoperation-isaac/robot/franka/fr3_duo/urdf/fr3_duo.urdf"))
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--hfov", type=float, default=69.0, help="horizontal field of view, degrees")
    ap.add_argument("--out", default=os.path.expanduser("~/isaacsim/render-check/latest"))
    ap.add_argument("--fabric", action="store_true", help="inprocess: let PhysX write to Fabric, so the renderer sees motion")
    ap.add_argument("--baseline", type=float, default=5.0, help="inprocess: seconds of physics alone before rendering starts")
    args = ap.parse_args()
    sys.argv = sys.argv[:1]
    os.makedirs(args.out, exist_ok=True)
    usd = args.usd or sorted(glob.glob(os.path.expanduser("~/isaacsim/work/fr3_duo-*/fr3_duo/fr3_duo.usda")))[-1]
    joints = read_urdf(args.urdf)
    result = {"mode": args.mode, "usd": usd, "width": args.width, "height": args.height, "fps_target": args.fps}

    launch = time.perf_counter()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    result["launch_s"] = round(time.perf_counter() - launch, 1)
    try:
        if args.mode == "renderer":
            run_renderer(app, args, usd, joints, result)
        else:
            run_inprocess(app, args, usd, joints, result)
    finally:
        print("RESULT " + json.dumps(result, sort_keys=True), flush=True)
        app.close()
    missing = [k for k, v in result.items() if v is None]
    if missing:
        print(f"[rc] missing: {missing}", flush=True)
    return 1 if missing else 0


def run_renderer(app, args, usd, joints, result):
    import io

    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from importlib.metadata import version
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.rendering_manager import RenderingManager
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    from PIL import Image
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdSemantics, UsdShade

    result["isaac_version"] = version("isaacsim")
    stage_utils.open_stage(usd)
    while stage_utils.is_stage_loading():
        app.update()
    stage = stage_utils.get_current_stage()
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
    for _ in range(3):
        app.update()

    # Two test textures for the backdrop surfaces: noise, and a green ramp.
    rng = np.random.default_rng(1)
    tex_a, tex_b = os.path.join(args.out, "tex_a.png"), os.path.join(args.out, "tex_b.png")
    Image.fromarray((rng.random((256, 512, 3)) * 255).astype(np.uint8)).save(tex_a)
    ramp = np.zeros((256, 512, 3), np.uint8)
    ramp[..., 1] = np.linspace(40, 255, 512, dtype=np.float32)[None, :].astype(np.uint8)
    Image.fromarray(ramp).save(tex_b)

    UsdLux.DomeLight.Define(stage, "/Scene/Dome").CreateIntensityAttr(800)
    key = UsdLux.DistantLight.Define(stage, "/Scene/Key")
    key.CreateIntensityAttr(2500)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-40, 25, 0))

    def box(path, center, size, colour):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(cube)
        x.AddTranslateOp().Set(Gf.Vec3d(*center))
        x.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        return cube

    def textured_quad(path, corners, image):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*c) for c in corners])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateDoubleSidedAttr(True)
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
        mat = UsdShade.Material.Define(stage, path + "Material")
        surf = UsdShade.Shader.Define(stage, path + "Material/Surface")
        surf.CreateIdAttr("UsdPreviewSurface")
        surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        tex = UsdShade.Shader.Define(stage, path + "Material/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        file_in = tex.CreateInput("file", Sdf.ValueTypeNames.Asset)
        file_in.Set(image)
        reader = UsdShade.Shader.Define(stage, path + "Material/St")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        return mesh, file_in

    tx, ty, tz = TABLE["center"]
    sx, sy, sz = TABLE["size"]
    box("/Scene/Table", TABLE["center"], TABLE["size"], (0.55, 0.55, 0.55))
    top = tz + sz / 2 + 0.0005
    _, mat_file = textured_quad("/Scene/Mat", [(tx - sx / 2, ty - sy / 2, top), (tx + sx / 2, ty - sy / 2, top),
                                               (tx + sx / 2, ty + sy / 2, top), (tx - sx / 2, ty + sy / 2, top)], tex_a)
    cx, cy, cz = SCREEN["center"]
    w, h = SCREEN["size"]
    _, screen_file = textured_quad("/Scene/Screen", [(cx, cy + w / 2, cz - h / 2), (cx, cy - w / 2, cz - h / 2),
                                                     (cx, cy - w / 2, cz + h / 2), (cx, cy + w / 2, cz + h / 2)], tex_a)
    cube = box("/Scene/TestCube", (0.5, 0.0, CUBE / 2), (CUBE, CUBE, CUBE), (0.9, 0.2, 0.1))
    cube_xf = UsdGeom.Xformable(cube).GetOrderedXformOps()[0]

    def label(prim, name):
        UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr().Set([name])

    links = {p.GetName(): p for p in stage.Traverse() if p.GetTypeName() == "Xform"}
    for link, name in (("left_base", "left_arm"), ("right_base", "right_arm"), ("left_fr3v2_hand", "left_hand"),
                       ("right_fr3v2_hand", "right_hand"), ("head_link", "head")):
        label(links[link], name)
    for path, name in (("/Scene/Table", "table"), ("/Scene/Mat", "mat"), ("/Scene/Screen", "screen"), ("/Scene/TestCube", "cube")):
        label(stage.GetPrimAtPath(path), name)
    result["semantics_api"] = "UsdSemantics.LabelsAPI"

    moving = [(n, j) for n, j in joints.items() if j["type"] in ("revolute", "prismatic") and j["child"] in links]
    view = XformPrim([str(links[j["child"]].GetPath()) for _, j in moving])
    rest_t, rest_q = (np.asarray(a.numpy() if hasattr(a, "numpy") else a, float) for a in view.get_local_poses())
    result["moving_links"] = len(moving)
    tcps = {side: links[f"{side}_fr3v2_hand_tcp"] for side in ("left", "right")}

    focal = 20.955 / (2 * math.tan(math.radians(args.hfov) / 2))
    attrs = {"focalLength": focal, "horizontalAperture": 20.955,
             "verticalAperture": 20.955 * args.height / args.width, "clippingRange": Gf.Vec2f(0.05, 10.0)}
    mount = links["head_camera_mounting_point"]
    head_q = q_from_matrix(np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]))
    head = RtxCamera(str(mount.GetPath()) + "/HeadCam", translations=[[0, 0, 0]], orientations=[list(head_q)], attributes=attrs)
    third_pos = (1.15, 0.75, 0.60)
    third_q = camera_rotation(np.subtract((0.45, 0.0, 0.05), third_pos), (0, 0, 1))
    third = RtxCamera("/Scene/ThirdCam", positions=[list(third_pos)], orientations=[list(third_q)], attributes=attrs)
    res = (args.height, args.width)  # (height, width), NumPy's order
    head_sensor = CameraSensor(head, resolution=res, annotators=["rgb", "distance_to_image_plane", "semantic_segmentation"])
    third_sensor = CameraSensor(third, resolution=res, annotators=["rgb", "semantic_segmentation"])
    app_utils.play(commit=True)  # sensors deliver only while the timeline plays; with Physics at "none" nothing steps

    fx = focal / 20.955 * args.width
    cxp, cyp = args.width / 2.0, args.height / 2.0
    result["intrinsics"] = {"fx": round(fx, 2), "fy": round(fx, 2), "cx": cxp, "cy": cyp}

    def world_pose(cam):
        p, q = cam.get_world_poses()
        p = np.asarray(p.numpy() if hasattr(p, "numpy") else p, float).reshape(-1)[:3]
        q = np.asarray(q.numpy() if hasattr(q, "numpy") else q, float).reshape(-1)[:4]
        return p, tuple(q)

    def project(cam, X):
        p, q = cam
        xc = q_matrix(q).T @ (np.asarray(X, float) - p)
        if xc[2] >= -1e-6:
            return None
        return cxp + fx * xc[0] / -xc[2], cyp - fx * xc[1] / -xc[2]

    def read(sensor, name):
        data, info = sensor.get_data(name)
        if data is None:
            return None, info
        a = data.numpy() if hasattr(data, "numpy") else np.asarray(data)
        return (a[..., 0] if a.ndim == 3 and a.shape[-1] == 1 else a), info

    warm = time.perf_counter()
    for i in range(600):
        RenderingManager.render()
        a, _ = read(head_sensor, "rgb")
        if a is not None and i > 5:
            break
    result["warmup_frames"] = i + 1
    result["warmup_s"] = round(time.perf_counter() - warm, 1)
    head_cam, third_cam = world_pose(head), world_pose(third)
    result["head_camera_world"] = [round(float(v), 4) for v in list(head_cam[0]) + list(head_cam[1])]
    mount_p, mount_q = fk(joints, {}, "head_camera_mounting_point")
    result["head_mount_fk"] = [round(float(v), 4) for v in list(mount_p) + list(mount_q)]

    names = [n for n, _ in moving]
    frame_s = 1.0 / args.fps
    pose_ms, render_ms, read_ms, frame_ms, analysis_ms = [], [], [], [], []
    lags, depth_err_mm, fk_err_mm = [], [], []
    proj = {j: [] for j in range(4)}
    box_err = {j: [] for j in range(4)}
    angle_hist, cube_hist = [], []
    seen = {}
    n_frames = int(args.seconds * args.fps)
    print("[rc] rendering", flush=True)
    start = time.perf_counter()
    us, vs = np.meshgrid(np.arange(0, args.width, 8), np.arange(0, args.height, 8))
    for k in range(n_frames):
        t0 = time.perf_counter()
        angles = pose_at(k * frame_s, names)
        qs = np.empty_like(rest_q)
        ts = rest_t.copy()
        for i, (n, j) in enumerate(moving):
            if j["type"] == "revolute":
                qs[i] = q_mul(tuple(rest_q[i]), q_axis(j["axis"], angles[n]))
            else:
                qs[i] = rest_q[i]
                ts[i] = rest_t[i] + q_matrix(tuple(rest_q[i])) @ (np.asarray(j["axis"]) * angles[n])
        view.set_local_poses(ts, qs)
        cube_p = (0.35 + 0.4 * ((k * 7) % 11) / 10, -0.3 + 0.6 * ((k * 3) % 7) / 6, CUBE / 2)
        cube_xf.Set(Gf.Vec3d(*cube_p))
        angle_hist.append(angles)
        cube_hist.append(cube_p)
        t1 = time.perf_counter()
        RenderingManager.render()
        t2 = time.perf_counter()
        rgb, _ = read(head_sensor, "rgb")
        depth, _ = read(head_sensor, "distance_to_image_plane")
        seg, info = read(head_sensor, "semantic_segmentation")
        read(third_sensor, "rgb")
        tseg, tinfo = read(third_sensor, "semantic_segmentation")
        t3 = time.perf_counter()
        if seg is not None and depth is not None:
            ids = {v.get("class"): int(i) for i, v in (info.get("idToLabels") or {}).items()}
            for nm in ids:
                seen[nm] = seen.get(nm, 0) + 1
            # Projection: the cube's eight projected corners bound the same pixels as its mask, at each lag.
            if "cube" in ids:
                vv, uu = np.nonzero(seg == ids["cube"])
                if len(uu) > 30 and uu.min() > 2 and vv.min() > 2 and uu.max() < args.width - 3 and vv.max() < args.height - 3:
                    for j in range(4):
                        if k - j < 0:
                            continue
                        cp = cube_hist[k - j]
                        pts = [project(head_cam, (cp[0] + dx * CUBE / 2, cp[1] + dy * CUBE / 2, cp[2] + dz * CUBE / 2))
                               for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)]
                        if all(p_ is not None for p_ in pts):
                            pu, pv = [p_[0] for p_ in pts], [p_[1] for p_ in pts]
                            box_err[j].append(max(abs(min(pu) - uu.min()), abs(max(pu) - (uu.max() + 1)),
                                                  abs(min(pv) - vv.min()), abs(max(pv) - (vv.max() + 1))))
            # The render lag: which recent cube position the mask shows.
            if "cube" in ids:
                vv, uu = np.nonzero(seg == ids["cube"])
                if len(uu) > 30:
                    c = (uu.mean() + 0.5, vv.mean() + 0.5)
                    best = None
                    for j in range(4):
                        if k - j < 0:
                            continue
                        pr = project(head_cam, cube_hist[k - j])
                        if pr is not None:
                            e = math.hypot(pr[0] - c[0], pr[1] - c[1])
                            if best is None or e < best[0]:
                                best = (e, j)
                    if best is not None and best[0] < 8:
                        lags.append(best[1])
            # Depth on the mat against the table plane (z = mat top).
            if "mat" in ids:
                sel = seg[vs, us] == ids["mat"]
                if sel.sum() > 50:
                    u, v = us[sel] + 0.5, vs[sel] + 0.5
                    d_cam = np.stack([(u - cxp) / fx, -(v - cyp) / fx, -np.ones_like(u)], axis=0)
                    d_w = q_matrix(head_cam[1]) @ d_cam
                    tpar = (top - head_cam[0][2]) / d_w[2]
                    err = np.abs(depth[vs[sel], us[sel]] - tpar) * 1000
                    depth_err_mm.extend(err[np.isfinite(err)].tolist()[::4])
            # FK against the rendered robot: the tool point near each hand's mask, for each candidate lag, in
            # both cameras. Nested labels come out joined ("left_arm,left_hand").
            for cam, sa, ia in ((head_cam, seg, info), (third_cam, tseg, tinfo)):
                if sa is None:
                    continue
                cls = {v.get("class"): int(i) for i, v in (ia.get("idToLabels") or {}).items()}
                for side in ("left", "right"):
                    hids = [i for c, i in cls.items() if c and f"{side}_hand" in c.split(",")]
                    if not hids:
                        continue
                    vv, uu = np.nonzero(np.isin(sa, hids))
                    if len(uu) < 30:
                        continue
                    for j in range(4):
                        if k - j < 0:
                            continue
                        tp, _ = fk(joints, angle_hist[k - j], f"{side}_fr3v2_hand_tcp")
                        pr = project(cam, tp)
                        if pr is not None and 0 <= pr[0] < args.width and 0 <= pr[1] < args.height:
                            proj[j].append(float(np.min(np.hypot(uu + 0.5 - pr[0], vv + 0.5 - pr[1]))))
            # FK against USD itself (what the renderer was handed).
            for side, prim in tcps.items():
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                usd_p = np.array(m.ExtractTranslation())
                fk_p, _ = fk(joints, angles, f"{side}_fr3v2_hand_tcp")
                fk_err_mm.append(float(np.linalg.norm(usd_p - fk_p) * 1000))
        t4 = time.perf_counter()
        pose_ms.append((t1 - t0) * 1e3)
        render_ms.append((t2 - t1) * 1e3)
        read_ms.append((t3 - t2) * 1e3)
        frame_ms.append((t3 - t0) * 1e3)
        analysis_ms.append((t4 - t3) * 1e3)
        wait = start + (k + 1) * frame_s - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
    elapsed = time.perf_counter() - start
    result["frames"] = n_frames
    result["fps"] = round(n_frames / elapsed, 2)
    for nm, xs in (("pose_ms", pose_ms), ("render_ms", render_ms), ("read_ms", read_ms), ("frame_ms", frame_ms),
                   ("analysis_ms", analysis_ms)):
        result[nm + "_p50"] = round(pct(xs, 50), 2)
        result[nm + "_p99"] = round(pct(xs, 99), 2)
    result["gpu_mib"] = own_gpu_mib()
    result["classes_seen_head"] = seen
    if lags:
        mode = max(set(lags), key=lags.count)
        result["render_lag_frames"] = mode
        result["render_lag_stable_fraction"] = round(lags.count(mode) / len(lags), 4)
        result["render_lag_samples"] = len(lags)
    else:
        result["render_lag_frames"] = None
        result["render_lag_stable_fraction"] = None
    result["depth_err_mm_p50"] = round(pct(depth_err_mm, 50), 3) if depth_err_mm else None
    result["depth_err_mm_p99"] = round(pct(depth_err_mm, 99), 3) if depth_err_mm else None
    result["fk_vs_usd_mm_max"] = round(max(fk_err_mm), 4) if fk_err_mm else None
    lag = result["render_lag_frames"] if result["render_lag_frames"] is not None else 0
    result["tcp_to_hand_mask_px_p50"] = round(pct(proj[lag], 50), 2) if proj[lag] else None
    result["tcp_to_hand_mask_px_p99"] = round(pct(proj[lag], 99), 2) if proj[lag] else None
    result["tcp_to_hand_mask_px_by_lag"] = {j: (round(pct(v, 50), 2) if v else None) for j, v in proj.items()}
    result["projection_err_px_p50"] = round(pct(box_err[lag], 50), 2) if box_err[lag] else None
    result["projection_err_px_p99"] = round(pct(box_err[lag], 99), 2) if box_err[lag] else None
    result["projection_err_px_by_lag"] = {j: (round(pct(v, 50), 2) if v else None) for j, v in box_err.items()}
    third_classes = (tinfo or {}).get("idToLabels") or {}
    lr = {v.get("class"): i for i, v in third_classes.items()}
    result["masks_left_right_distinct"] = ("left_arm" in lr and "right_arm" in lr and lr["left_arm"] != lr["right_arm"]) if lr else None

    # What each camera sees, as text (images stay on this machine).
    def thumb(seg_a, info_a, cols=80, rows=22):
        if seg_a is None:
            return None
        names_ = {int(i): v.get("class") for i, v in (info_a.get("idToLabels") or {}).items()}
        chars = {"BACKGROUND": " ", "UNLABELLED": ".", "table": "t", "mat": "m", "screen": "s", "cube": "C",
                 "left_arm": "L", "right_arm": "R", "left_hand": "l", "right_hand": "r", "head": "h"}
        hh, ww = seg_a.shape[:2]
        out = []
        for r in range(rows):
            line = ""
            for c in range(cols):
                line += chars.get(names_.get(int(seg_a[int((r + 0.5) * hh / rows), int((c + 0.5) * ww / cols)])), "?")
            out.append(line)
        cover = {names_.get(int(i), str(i)): round(float((seg_a == i).mean()), 4) for i in np.unique(seg_a)}
        return out, cover
    for nm, sa, ia in (("head", seg, info), ("third", tseg, tinfo)):
        th = thumb(sa, ia)
        if th:
            result[f"coverage_{nm}"] = th[1]
            print(f"[rc] {nm} camera, last frame:", flush=True)
            for line in th[0]:
                print(f"[rc] |{line}|", flush=True)

    # Encode sizes and times on the last frame.
    if rgb is not None:
        import cv2
        im = np.ascontiguousarray(rgb[..., :3])
        def timed(fn, n=20):
            out, ts_ = None, []
            for _ in range(n):
                t = time.perf_counter()
                out = fn()
                ts_.append((time.perf_counter() - t) * 1e3)
            return out, round(statistics.median(ts_), 2)

        def pil_jpeg(q):
            buf = io.BytesIO()
            Image.fromarray(im).save(buf, "JPEG", quality=q)
            return buf.tell()
        for q in (80, 92):
            size, result[f"jpeg_q{q}_pil_ms"] = timed(lambda: pil_jpeg(q))
            result[f"jpeg_q{q}_kb"] = round(size / 1024, 1)
            _, result[f"jpeg_q{q}_cv2_ms"] = timed(lambda: cv2.imencode(".jpg", im[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, q]))
        # Depth and masks of a frame with the arms in it (the third camera's), as recording would encode them.
        dsrc = depth
        dmm = np.where(np.isfinite(dsrc), np.clip(np.round(dsrc * 1000), 0, 65535), 0).astype(np.uint16)
        enc, result["png16_depth_ms"] = timed(lambda: cv2.imencode(".png", dmm)[1])
        result["png16_depth_kb"] = round(len(enc) / 1024, 1)
        msrc = tseg if tseg is not None else seg
        enc, result["png16_mask_ms"] = timed(lambda: cv2.imencode(".png", msrc.astype(np.uint16))[1])
        result["png16_mask_kb"] = round(len(enc) / 1024, 1)
        Image.fromarray(im).save(os.path.join(args.out, "head_rgb.png"))

    # A backdrop swap: frames until the mat's new texture shows in the head camera.
    def mat_mean():
        RenderingManager.render()
        r, _ = read(head_sensor, "rgb")
        s_, i_ = read(head_sensor, "semantic_segmentation")
        ids_ = {v.get("class"): int(i) for i, v in (i_.get("idToLabels") or {}).items()}
        if r is None or s_ is None or "mat" not in ids_:
            return None
        sel_ = s_ == ids_["mat"]
        return r[..., :3][sel_].mean(axis=0) if sel_.sum() > 50 else None
    # A photo-sized image, like the dataset's (about 1 MB of JPEG), smooth enough to compress like one.
    big = os.path.join(args.out, "tex_big.jpg")
    yy, xx = np.mgrid[0:1536, 0:2048]
    photo = np.stack([(128 + 100 * np.sin(xx / 97.0 + yy / 151.0)), (90 + 60 * np.cos(yy / 83.0)),
                      (160 + 80 * np.sin((xx + yy) / 211.0))], axis=-1)
    photo += rng.normal(0, 12, photo.shape)
    Image.fromarray(np.clip(photo, 0, 255).astype(np.uint8)).save(big, "JPEG", quality=90)
    result["texture_big_kb"] = round(os.path.getsize(big) / 1024, 1)

    def settle_after(image):
        before = mat_mean()
        mat_file.Set(image)
        screen_file.Set(image)
        means = [mat_mean() for _ in range(90)]
        if before is None or any(m is None for m in means):
            return None, None
        final = means[-1]
        total = np.linalg.norm(final - before)
        return (next((i + 1 for i, m in enumerate(means) if np.linalg.norm(m - final) < 0.05 * max(total, 1e-6)), None),
                round(float(total), 1))
    result["texture_settle_frames"], result["texture_change_rgb"] = settle_after(tex_b)
    result["texture_settle_frames_big"], result["texture_change_rgb_big"] = settle_after(big)


def run_inprocess(app, args, usd, joints, result):
    """The bridge's physics with the head camera rendered every 1/fps s in the same loop."""
    import carb
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    import omni.kit.app
    from importlib.metadata import version
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.rendering_manager import RenderingManager
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    result["isaac_version"] = version("isaacsim")
    result["fabric"] = bool(args.fabric)
    if args.fabric:
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.physx.fabric", True)
    stage_utils.open_stage(usd)
    while stage_utils.is_stage_loading():
        app.update()
    stage = stage_utils.get_current_stage()
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    for _ in range(3):
        app.update()
    roots = [str(x.GetPath()) for x in stage.Traverse() if x.HasAPI(UsdPhysics.ArticulationRootAPI)]
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
    art_api.CreateSolverPositionIterationCountAttr(8)
    art_api.CreateSolverVelocityIterationCountAttr(1)
    art_api.CreateEnabledSelfCollisionsAttr(False)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
    settings = carb.settings.get_settings()
    for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateParticlesToUsd", "/physics/updateResidualsToUsd"):
        settings.set(key, False)
    UsdLux.DomeLight.Define(stage, "/Scene/Dome").CreateIntensityAttr(800)
    focal = 20.955 / (2 * math.tan(math.radians(args.hfov) / 2))
    attrs = {"focalLength": focal, "horizontalAperture": 20.955,
             "verticalAperture": 20.955 * args.height / args.width, "clippingRange": Gf.Vec2f(0.05, 10.0)}
    third_pos = (1.15, 0.75, 0.60)
    third = RtxCamera("/Scene/ThirdCam", positions=[list(third_pos)],
                      orientations=[list(camera_rotation(np.subtract((0.45, 0.0, 0.05), third_pos), (0, 0, 1)))], attributes=attrs)
    sensor = CameraSensor(third, resolution=(args.height, args.width), annotators=["rgb"])
    SimulationManager.setup_simulation(dt=0.001, device="cpu")
    app_utils.play()
    app.update()
    art = Articulation(roots[0])
    names = list(art.dof_names)
    kp = np.array([[2000.0 if "finger" in n else 16000.0 for n in names]])
    kd = np.array([[100.0 if "finger" in n else 250.0 for n in names]])
    art.set_dof_gains(kp, kd)
    target = np.array([[pose_at(0.0, names)[n] for n in names]])
    art.set_dof_positions(target)
    art.set_dof_position_targets(target)

    def frame():
        RenderingManager.render()
        d, _ = sensor.get_data("rgb")
        return None if d is None else (d.numpy() if hasattr(d, "numpy") else np.asarray(d)).astype(np.int16)
    for _ in range(30):
        frame()

    dt_ns = 1_000_000
    render_every = max(1, int(round(1000 / args.fps)))
    total = int((args.baseline + args.seconds) * 1000)
    late = {"base": 0, "render": 0}
    gaps = {"base": [], "render": []}
    stalls, frames, lag_ms, resyncs = [], [], 0.0, 0
    q_first = q_last = None
    start = time.monotonic_ns()
    last_state = None
    for steps in range(1, total + 1):
        window = "base" if steps <= args.baseline * 1000 else "render"
        if steps % 4 == 0:
            t_ = pose_at(steps * 1e-3, names)
            art.set_dof_position_targets(np.array([[t_[n] for n in names]]))
        SimulationManager.step(steps=1)
        now = time.monotonic_ns()
        if steps % 4 == 0:
            q = np.asarray(art.get_dof_positions().numpy()).reshape(-1)
            if window == "render":
                q_first = q if q_first is None else q_first
                q_last = q
            if last_state is not None:
                gaps[window].append((now - last_state) / 1e6)
            last_state = now
        if window == "render" and steps % render_every == 0:
            t0 = time.monotonic_ns()
            img = frame()
            stalls.append((time.monotonic_ns() - t0) / 1e6)
            if img is not None:
                frames.append(img)
            now = time.monotonic_ns()
        due = start + steps * dt_ns + int(lag_ms * 1e6)
        behind = now - due
        if behind > dt_ns:
            late[window] += 1
            if behind > 50 * dt_ns:
                lag_ms += behind / 1e6
                resyncs += 1
        else:
            while time.monotonic_ns() < due:
                pass
    result["baseline_s"] = args.baseline
    result["render_s"] = args.seconds
    result["late_steps_per_s_baseline"] = round(late["base"] / args.baseline, 2)
    result["late_steps_per_s_rendering"] = round(late["render"] / args.seconds, 2)
    result["resyncs_over_50ms"] = resyncs
    result["lag_added_ms"] = round(lag_ms, 1)
    result["render_stall_ms_p50"] = round(pct(stalls, 50), 2)
    result["render_stall_ms_p99"] = round(pct(stalls, 99), 2)
    result["render_stall_ms_max"] = round(max(stalls), 2)
    for w in ("base", "render"):
        result[f"state_gap_ms_p99_{w}"] = round(pct(gaps[w], 99), 2)
        result[f"state_gap_ms_max_{w}"] = round(max(gaps[w]), 2)
    result["arm_moved_rad"] = round(float(np.max(np.abs(q_last - q_first))), 4) if q_first is not None else None
    # Did the renders see the arm move? Mean pixel change across 0.5 s, against the change between neighbours.
    if len(frames) > 20:
        far = [float(np.abs(frames[i + 15] - frames[i]).mean()) for i in range(0, len(frames) - 15, 15)]
        near = [float(np.abs(frames[i + 1] - frames[i]).mean()) for i in range(0, len(frames) - 1, 15)]
        result["render_change_halfsecond"] = round(statistics.median(far), 3)
        result["render_change_neighbour"] = round(statistics.median(near), 3)
        result["render_sees_motion"] = statistics.median(far) > 3 * max(statistics.median(near), 0.05)
    else:
        result["render_change_halfsecond"] = result["render_change_neighbour"] = result["render_sees_motion"] = None


if __name__ == "__main__":
    sys.exit(main())
