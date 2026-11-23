"""The geometry PhysX collides with, for the per-frame clearance (docs/adr/0038 decision 4): every arm link's collision boxes
and every mesh asset's collider, each in its own body frame at size factor 1, kept in a cache that the dataset build reads
(``teleop_lerobot.clearance``).

Run on the GPU machine with Isaac's Python and no Kit (pxr, numpy, scipy and CoACD, which Isaac's own core ships), for a
scene file (``just isaac-clearance-geometry <scene>``):

    ~/isaacsim/venv/bin/python -m teleop_isaac.clearance_geometry core/Scenes/jetrover-bench.json [--out DIR]

**What PhysX collides with** (``world.py``, ``jetrover_bridge.py``, ``urdf_usd.py``):

- **The arm.** Isaac's URDF importer makes every ``<collision><box>`` of ``jetrover.urdf`` a ``UsdGeom.Cube`` collider of its
  link (size 1, scaled to the box, turned and placed by the box's origin), the finger pads included. Read from the converted
  USD when it is on disk (``~/isaacsim/work/jetrover-<digest>``), else from the URDF, which the USD matches box for box
  (checked when both are there). Boxes are exact. The articulation's self-collision is off (``jetrover_bridge.py``).
- **Surfaces** (the table, the bench, the floor): ``UsdGeom.Cube`` boxes of the scene file's size, exact; nothing to export.
- **A box, cylinder or sphere object**: the primitive at the scene file's size times its size factor; nothing to export.
- **A mesh object** (``world.py``): every ``UsdGeom.Mesh`` under its asset gets a ``convexDecomposition`` collider, with
  ``PhysxConvexDecompositionCollisionAPI``'s defaults (omni.physx 110.4: at most 32 hulls of at most 64 vertices, voxel
  resolution 500000, error 10 %, no shrink wrap). Only a running Kit cooks that decomposition (V-HACD), and this export
  starts none, so it decomposes each mesh itself with CoACD at the same budget (32 hulls of 64 vertices; :data:`METHOD`).
  CoACD's hulls follow the mesh's concave parts as V-HACD's do (a bowl's inside, a mug's handle stay open); they are not
  V-HACD's hulls, which lie up to a voxel outside the mesh. ``--method hull`` gives each mesh one convex hull instead.

**The cache** (``--out``, default :data:`DEFAULT_OUT`):

- ``robot-<robot>.json``: the links in the URDF's order (``WC``'s link index), the joints (for forward kinematics), the bodies no
  joint moves (``fixed``: the surfaces filter them, so they have no distance to the table), and every collision box in its
  link's frame: ``center``, ``axes`` (the box's x, y and z as rows) and ``half`` (half its sides), metres.
- ``assets/<library>/<name>.json``: one mesh asset's convex pieces in its body frame at k = 1, the frame ``world.py`` stands it
  in (its box centred on the origin, standing, at the manifest's size; an object of the scene scales it by its entry's size
  over the asset's, times its size factor). Each piece is a convex hull: ``vertices`` and outward ``faces`` (triangles). Its
  ``key`` is the sha256 of the asset's manifest entry (its files' sha256 among them) and of the method: an asset that changed
  is exported again, one that did not is left as it is.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

FORMAT, ROBOT_FORMAT, VERSION = "teleop-clearance-geometry", "teleop-clearance-robot", 1
DEFAULT_OUT = os.path.expanduser("~/isaacsim/assets/clearance")
ASSETS = os.path.expanduser("~/isaacsim/assets")
WORK = os.path.expanduser("~/isaacsim/work")
# PhysxConvexDecompositionCollisionAPI's defaults (Isaac Sim 6.2, omni.physx 110.4), which world.py leaves as they are.
MAX_HULLS, HULL_VERTICES = 32, 64
# CoACD at PhysX's budget. Its concavity threshold is its own default; seed 0 makes it the same every run.
COACD = {"threshold": 0.05, "max_convex_hull": MAX_HULLS, "max_ch_vertex": HULL_VERTICES, "preprocess_mode": "auto",
         "preprocess_resolution": 50, "resolution": 2000, "seed": 0}
METHODS = ("coacd", "hull")


# --- the arm ------------------------------------------------------------------------------------------------------------

def _rpy_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """URDF's fixed-axis roll, pitch, yaw as a rotation matrix (columns are the turned frame's axes)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _floats(text: Optional[str], default: str) -> List[float]:
    return [float(v) for v in (text if text is not None else default).split()]


def urdf_robot(path: str) -> Dict:
    """The URDF's links (in file order), joints, the links no joint moves, and every ``<collision><box>`` as a box in its
    link's frame (``center``, ``axes`` as rows, ``half``). A collision that is not a box is refused: the importer would make a
    mesh collider of it, which this module does not export."""
    root = ET.parse(path).getroot()
    links = [link.get("name") for link in root.findall("link")]
    joints, boxes = [], []
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        joints.append({"name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                       "child": j.find("child").get("link"),
                       "xyz": _floats(o.get("xyz") if o is not None else None, "0 0 0"),
                       "rpy": _floats(o.get("rpy") if o is not None else None, "0 0 0"),
                       "axis": _floats(a.get("xyz") if a is not None else None, "1 0 0")})
    for link in root.findall("link"):
        for c in link.findall("collision"):
            box = c.find("geometry/box")
            if box is None:
                raise ValueError(f"{path}: link {link.get('name')}'s collision {c.get('name')} is not a box")
            o = c.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None, "0 0 0")
            rpy = _floats(o.get("rpy") if o is not None else None, "0 0 0")
            m = _rpy_matrix(*rpy)
            size = _floats(box.get("size"), "0 0 0")
            boxes.append({"link": link.get("name"), "name": c.get("name") or "", "center": xyz,
                          "axes": [[m[0][k], m[1][k], m[2][k]] for k in range(3)], "half": [s / 2 for s in size]})
    # The base is pinned (the importer's fix_base): it and whatever fixed joints bolt to it are the robot's fixed bodies,
    # which world.py filters against the surfaces (world.IsaacWorld.fixed_bodies).
    children = {lk for lk in links if any(j["child"] == lk for j in joints)}
    fixed = {lk for lk in links if lk not in children}
    grown = True
    while grown:
        grown = False
        for j in joints:
            if j["type"] == "fixed" and (j["parent"] in fixed) != (j["child"] in fixed):
                fixed |= {j["parent"], j["child"]}
                grown = True
    return {"links": links, "joints": joints, "fixed": [lk for lk in links if lk in fixed], "boxes": boxes}


def usd_boxes(usd: str) -> List[Dict]:
    """Every ``UsdGeom.Cube`` collider of the converted robot as a box in its rigid body's frame (needs pxr): what PhysX
    collides with. A collider of another type is refused."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(usd)
    robot = stage.GetDefaultPrim()
    variants = robot.GetVariantSets()
    if variants.HasVariantSet("Physics"):
        variants.GetVariantSet("Physics").SetVariantSelection("physx")  # as jetrover_bridge.py selects it
    cache = UsdGeom.XformCache()
    out = []
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        body = prim
        while body and body.IsValid() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body or not body.IsValid():
            raise ValueError(f"{usd}: collider {prim.GetPath()} is on no rigid body")
        if not prim.IsA(UsdGeom.Cube):
            raise ValueError(f"{usd}: collider {prim.GetPath()} is a {prim.GetTypeName()}, not a box")
        rel = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse()  # row vectors
        size = float(UsdGeom.Cube(prim).GetSizeAttr().Get())
        rows = [[rel[r][c] for c in range(3)] for r in range(3)]
        norms = [math.sqrt(sum(v * v for v in row)) for row in rows]
        out.append({"link": body.GetName(), "name": prim.GetName(), "center": [rel[3][c] for c in range(3)],
                    "axes": [[v / n for v in row] for row, n in zip(rows, norms)], "half": [n * size / 2 for n in norms]})
    return out


def same_boxes(a: Sequence[Dict], b: Sequence[Dict], tolerance: float = 2e-5) -> List[str]:
    """How two lists of link boxes differ, matched link by link by centre and half sides (an axis may be flipped or the sides
    permuted with it): empty when every box of one is a box of the other."""
    def corners(box):
        out = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    out.append(tuple(round(box["center"][k] + sum(s * box["half"][i] * box["axes"][i][k]
                                                                 for i, s in enumerate((sx, sy, sz))), 4) for k in range(3)))
        return sorted(out)

    problems = []
    for link in sorted({x["link"] for x in list(a) + list(b)}):
        left = [corners(x) for x in a if x["link"] == link]
        right = [corners(x) for x in b if x["link"] == link]
        if len(left) != len(right):
            problems.append(f"{link}: {len(left)} boxes against {len(right)}")
            continue
        unmatched = list(right)
        for c in left:
            hit = next((r for r in unmatched if all(math.dist(p, q) <= tolerance + 1.5e-4 for p, q in zip(c, r))), None)
            if hit is None:
                problems.append(f"{link}: a box at {c[0]}.. has no match")
            else:
                unmatched.remove(hit)
    return problems


def urdf_digest(urdf: str) -> str:
    from .urdf_usd import urdf_digest as digest

    return digest(urdf)


def robot_geometry(urdf: str, work: str = WORK, name: str = "jetrover") -> Dict:
    """The robot's file: the URDF's links and joints, and its boxes as the importer's USD has them when that conversion is on
    disk (checked against the URDF's), else the URDF's."""
    robot = urdf_robot(urdf)
    digest = urdf_digest(urdf)
    usd = os.path.join(work, f"{name}-{digest}", name, f"{name}.usda")
    source = f"the URDF's <collision> boxes ({os.path.basename(urdf)}; no conversion at {usd})"
    if os.path.isfile(usd):
        try:
            boxes = usd_boxes(usd)
        except ImportError:
            boxes = None
        if boxes is not None:
            problems = same_boxes(robot["boxes"], boxes)
            if problems:
                raise ValueError(f"the importer's colliders are not the URDF's boxes: {'; '.join(problems)}")
            robot["boxes"] = boxes
            source = f"the URDF importer's UsdGeom.Cube colliders ({usd}), the URDF's boxes box for box"
    return {"format": ROBOT_FORMAT, "version": VERSION, "robot": name, "urdf": os.path.basename(urdf), "urdfDigest": digest,
            "source": source, "selfCollision": False, **robot}


# --- mesh assets ---------------------------------------------------------------------------------------------------------

def manifest_entry(root: str, asset: str) -> Dict:
    library, name = asset.split("/")
    with open(os.path.join(root, library, "manifest.json"), encoding="utf-8") as f:
        entries = {e["name"]: e for e in json.load(f)["objects"]}
    if name not in entries:
        raise KeyError(f"{asset} is not in {library}'s manifest")
    return entries[name]


def method_description(method: str) -> str:
    if method == "hull":
        return "hull: one convex hull of each UsdGeom.Mesh"
    import coacd

    version = getattr(coacd, "__version__", "")
    if not version:
        try:
            from importlib.metadata import version as dist_version

            version = dist_version("coacd")
        except Exception:  # noqa: BLE001 -- the version is a label; the parameters are what matter
            version = "?"
    return f"coacd {version} " + " ".join(f"{k} {v}" for k, v in sorted(COACD.items())) + ", each UsdGeom.Mesh on its own"


def asset_key(entry: Dict, method: str) -> str:
    """What an export of the asset is made of: its manifest entry (its files' sha256, how it stands, its box) and the method."""
    text = json.dumps({"entry": entry, "method": method, "format": FORMAT, "version": VERSION}, sort_keys=True)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def asset_meshes(root: str, asset: str) -> Tuple[List[Tuple["object", "object"]], Dict]:
    """Each ``UsdGeom.Mesh`` under the asset as (vertices, triangles) in its body frame at k = 1, as ``world.py`` builds its
    Shape prim (translate ``-centre``, orient by the manifest's rotation, under the body), and the manifest entry. Needs pxr."""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom

    entry = manifest_entry(root, asset)
    library = asset.split("/")[0]
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/Body")
    g = UsdGeom.Xform.Define(stage, "/Body/Shape")
    gx = UsdGeom.Xformable(g)
    gx.AddTranslateOp().Set(Gf.Vec3d(*(-c for c in entry["centre"])))
    gx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*entry["rotation"]))
    gx.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    g.GetPrim().GetReferences().AddReference(os.path.join(root, library, entry["usd"]))
    cache = UsdGeom.XformCache()
    out = []
    for prim in Usd.PrimRange(g.GetPrim()):  # the prims world.py gives colliders
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts, counts, index = m.GetPointsAttr().Get(), m.GetFaceVertexCountsAttr().Get(), m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        pts, counts, index = np.array(pts, float), np.array(counts, int), np.array(index, int)
        M = np.array(cache.GetLocalToWorldTransform(prim))  # row vectors: p_body = [p, 1] @ M
        pw = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3]
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
        n_tri = np.maximum(counts - 2, 0)
        face = np.repeat(np.arange(len(counts)), n_tri)
        j = np.arange(n_tri.sum()) - np.repeat(np.cumsum(n_tri) - n_tri, n_tri) + 1
        tri = np.stack([index[starts[face]], index[starts[face] + j], index[starts[face] + j + 1]], axis=1)
        out.append((pw, tri))
    if not out:
        raise ValueError(f"{asset}: no mesh under its prim")
    return out, entry


def hull_piece(points) -> Dict:
    """A convex piece: the hull of ``points`` as its vertices and outward triangles (scipy's qhull)."""
    import numpy as np
    from scipy.spatial import ConvexHull

    p = np.asarray(points, float)
    h = ConvexHull(p)
    keep = sorted(set(int(i) for i in h.vertices))
    new = {old: k for k, old in enumerate(keep)}
    faces = []
    for simplex, eq in zip(h.simplices, h.equations):
        a, b, c = (int(i) for i in simplex)
        n = np.cross(p[b] - p[a], p[c] - p[a])
        if float(n @ eq[:3]) < 0:  # qhull's simplices are not oriented; its equations' normals point out
            b, c = c, b
        faces.append([new[a], new[b], new[c]])
    return {"vertices": [[round(float(v), 6) for v in p[i]] for i in keep], "faces": faces}


def decompose(vertices, triangles, method: str) -> List[Dict]:
    """One mesh's convex pieces: CoACD's parts at PhysX's budget, or the mesh's hull."""
    import numpy as np

    if method == "hull":
        return [hull_piece(vertices)]
    import coacd

    coacd.set_log_level("error")
    parts = coacd.run_coacd(coacd.Mesh(np.asarray(vertices, float), np.asarray(triangles, int)), **COACD)
    pieces = []
    for v, _ in parts:
        v = np.asarray(v, float)
        if len(v) >= 4 and np.linalg.matrix_rank(v - v.mean(0), tol=1e-7) == 3:
            pieces.append(hull_piece(v))
    if not pieces:
        raise ValueError("CoACD gave no solid part")
    return pieces


def export_asset(root: str, asset: str, method: str, description: str) -> Dict:
    t0 = time.monotonic()
    meshes, entry = asset_meshes(root, asset)
    pieces = []
    for v, f in meshes:
        pieces += decompose(v, f, method)
    lo = [min(p[k] for piece in pieces for p in piece["vertices"]) for k in range(3)]
    hi = [max(p[k] for piece in pieces for p in piece["vertices"]) for k in range(3)]
    return {"format": FORMAT, "version": VERSION, "asset": asset, "key": asset_key(entry, description), "size": entry["size"],
            "frame": "the body's at k = 1: the asset's box centred on the origin, standing as world.py stands it, at the manifest's size",
            "collider": f"PhysX convexDecomposition of each UsdGeom.Mesh (V-HACD, at most {MAX_HULLS} hulls of {HULL_VERTICES} vertices)",
            "method": description, "meshes": len(meshes), "triangles": int(sum(len(f) for _, f in meshes)),
            "bounds": [[round(v, 5) for v in lo], [round(v, 5) for v in hi]], "seconds": round(time.monotonic() - t0, 1),
            "pieces": pieces}


def scene_assets(scene_path: str) -> List[str]:
    with open(scene_path, encoding="utf-8") as f:
        return sorted({o["asset"] for o in json.load(f)["objects"] if o.get("shape") == "mesh"})


def write_json(path: str, doc: Dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    os.replace(tmp, path)


def export(scene_path: str, out: str = DEFAULT_OUT, root: str = ASSETS, urdf: str = "", work: str = WORK, method: str = "coacd",
           assets: Sequence[str] = (), say=print) -> Dict:
    """Writes the robot's file and every mesh asset of the scene that is missing or stale; returns a summary."""
    if method not in METHODS:
        raise ValueError(f"method {method!r}: one of {', '.join(METHODS)}")
    summary = {"out": out, "method": method, "exported": [], "kept": [], "failed": []}
    if urdf:
        robot = robot_geometry(urdf, work)
        write_json(os.path.join(out, f"robot-{robot['robot']}.json"), robot)
        say(f"[clearance-geometry] robot {robot['robot']} ({robot['urdfDigest']}): {len(robot['boxes'])} boxes on {len(robot['links'])} "
            f"links from {robot['source']}")
    description = method_description(method)
    for asset in (assets or scene_assets(scene_path)):
        library, name = asset.split("/")
        path = os.path.join(out, "assets", library, f"{name}.json")
        key = asset_key(manifest_entry(root, asset), description)
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                if json.load(f).get("key") == key:
                    summary["kept"].append(asset)
                    continue
        try:
            doc = export_asset(root, asset, method, description)
        except Exception as e:  # noqa: BLE001 -- one asset's failure is reported and the rest go on
            summary["failed"].append(asset)
            say(f"[clearance-geometry] {asset}: failed ({e})")
            continue
        write_json(path, doc)
        summary["exported"].append(asset)
        say(f"[clearance-geometry] {asset}: {len(doc['pieces'])} pieces, {sum(len(p['vertices']) for p in doc['pieces'])} vertices "
            f"from {doc['meshes']} mesh(es) of {doc['triangles']} triangles in {doc['seconds']} s")
    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    a = argparse.ArgumentParser(description="The geometry PhysX collides with, for the clearance (docs/adr/0038 decision 4)")
    a.add_argument("scene", help="a scene file (core/Scenes/*.json): its mesh assets")
    a.add_argument("--out", default=DEFAULT_OUT, help="the cache")
    a.add_argument("--assets-root", default=ASSETS)
    a.add_argument("--urdf", default=os.path.join(here, "..", "..", "jetrover", "urdf", "jetrover.urdf"))
    a.add_argument("--work", default=WORK, help="where the URDF's USD conversion is kept")
    a.add_argument("--method", choices=METHODS, default="coacd")
    a.add_argument("--asset", action="append", default=[], help="only this asset (library/name); repeat for more")
    args = a.parse_args(argv)
    s = export(args.scene, os.path.expanduser(args.out), os.path.expanduser(args.assets_root), os.path.abspath(args.urdf),
               os.path.expanduser(args.work), args.method, args.asset)
    print(f"[clearance-geometry] {len(s['exported'])} exported, {len(s['kept'])} up to date, {len(s['failed'])} failed, in {s['out']}")
    return 1 if s["failed"] else 0


if __name__ == "__main__":
    sys.exit(main())
