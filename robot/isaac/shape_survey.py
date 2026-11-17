"""Each mesh object's own shape, as slabs, for joint-drive's touch planner (docs/adr/0037 amendment 5, third revision).

joint-drive plans collide_and_recover's touch against an object's shape, not its box: a vase, a gnome or a kettle is far
smaller than its box, and a touch planned 20 mm into the box stopped 2-10 cm short of the real thing. This survey gives
each mesh asset its shape: the asset cut along its standing z into slabs, each slab a convex polygon extruded from its z0
to its z1.

Run it in Isaac Sim's Python on the GPU machine, the script read from standard input, the JSON printed to standard output
(it writes nothing there), and commit the output beside the scene:

    ssh andrei@<gpu> "~/isaacsim/venv/bin/python - $(python3 robot/isaac/shape_survey.py --assets-of core/Scenes/jetrover-bench.json)" \
        < robot/isaac/shape_survey.py > core/Scenes/jetrover-bench.shapes.json

(``--assets-of`` runs anywhere and lists the scene's mesh assets). Only pxr and numpy are used.

**What it measures.** Every ``UsdGeom.Mesh`` under the asset, as ``world.py`` gives each one a convexDecomposition collider,
in the frame ``world.py`` stands the asset in (its Shape prim: translate ``-centre*k``, orient by the manifest's rotation,
scale ``k``), at ``k`` = 1. That is the render mesh: PhysX's convex decomposition is cooked only inside a running Kit, which
this survey does not start, and lies a few millimetres outside the mesh.

**The slabs.** The asset's height is cut into thin layers (``LAYER_M``). A layer's section is the convex hull of the mesh's
vertices inside it and of the points where triangle edges cross its two planes, so a thin layer is never empty, and its
hull holds every section of the mesh within it. Neighbouring layers are merged into slabs, a slab's polygon the hull of its
layers'. The cuts are chosen to keep the largest **excess** of any slab smallest: how far its prism can stand off the mesh,
its polygon's overhang past the narrowest of its layers' sections in the worst direction, but at most the slab's height
(a surface that overhangs by more than that is nearly level, and is within that height of the prism, above or below it).
The bottom ``FLOOR_BAND_M`` is a slab of its own, left out (the collision model keeps the arm 10 mm above the table). The
fewest slabs from ``MIN_SLABS`` to ``MAX_SLABS`` whose excess is within ``EXCESS_GOAL_M`` (or within ``EXCESS_SLACK_M`` of the
most slabs') are kept, and ``excessMm`` says what it came to, at the manifest's size. A hull also fills in what a section has
of concave (a handle's gap): that is not in the excess. A polygon's corners that stick out less than ``CORNER_TOLERANCE_M``
past their neighbours are cut (the polygon only shrinks), so it has at most ``MAX_CORNERS``.

**The format** (``teleop-object-shapes`` version 1)::

    {"format": "teleop-object-shapes", "version": 1, "source": "...",
     "assets": {"<lib/name>": {"size": [x, y, z], "excessMm": e,
                               "slabs": [{"z0": .., "z1": .., "outline": [[x, y], ...]}, ...]}}}

``size`` is the manifest's box (metres); every coordinate is a fraction of it (-0.5..0.5 about the box's centre), so an
entry scaled to another size (a delivery box at 0.30-0.85 of its asset) uses the same slabs. Slabs run bottom to top, each
starting where the one below ends; each outline is convex and counter-clockwise, seen from above.
"""
import json
import math
import os
import sys

ASSETS = os.path.expanduser("~/isaacsim/assets")
LAYER_M = 0.002         # the thin layers' height, metres at the manifest's size (64 to 256 of them)
MIN_LAYERS, MAX_LAYERS = 64, 256
MIN_SLABS, MAX_SLABS = 8, 48
EXCESS_GOAL_M = 0.003
EXCESS_SLACK_M = 0.0005
FLOOR_BAND_M = 0.008
CORNER_TOLERANCE_M = 0.0003
MAX_CORNERS = 24
DIRECTIONS = 180        # the directions a section's extent is measured in
DECIMALS = 4            # a fraction to 0.0001: 0.04 mm on the largest asset


def hull(points):
    """The convex hull of 2-D points, counter-clockwise, without collinear points (Andrew's monotone chain)."""
    import numpy as np
    p = np.unique(np.round(np.asarray(points, float), 9), axis=0)
    if len(p) <= 2:
        return p
    # Akl-Toussaint: drop what lies inside the octagon of the extreme points before the chain.
    ext = [p[np.argmin(p[:, 0])], p[np.argmax(p[:, 0])], p[np.argmin(p[:, 1])], p[np.argmax(p[:, 1])],
           p[np.argmin(p.sum(1))], p[np.argmax(p.sum(1))], p[np.argmin(p[:, 0] - p[:, 1])], p[np.argmax(p[:, 0] - p[:, 1])]]
    q = np.unique(np.array(ext), axis=0)
    if len(q) >= 3:
        oct_ = _chain(q)
        if len(oct_) >= 3:
            inside = np.ones(len(p), bool)
            for i in range(len(oct_)):
                a, b = oct_[i], oct_[(i + 1) % len(oct_)]
                inside &= (b[0] - a[0]) * (p[:, 1] - a[1]) - (b[1] - a[1]) * (p[:, 0] - a[0]) > 1e-12
            p = p[~inside]
    return _chain(p)


def _chain(p):
    import numpy as np
    pts = sorted(map(tuple, p))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def cut_corners(poly, tolerance, max_corners):
    """Cuts the corners of a convex polygon that stick out less than ``tolerance`` past the line joining their neighbours
    (and then the least prominent, until at most ``max_corners`` are left): the polygon only shrinks, by at most what it cut."""
    import numpy as np
    pts = [tuple(v) for v in poly]

    def prominence(i):
        a, b, c = np.array(pts[i - 1]), np.array(pts[i]), np.array(pts[(i + 1) % len(pts)])
        ac = c - a
        n = np.hypot(*ac)
        return abs(ac[0] * (b[1] - a[1]) - ac[1] * (b[0] - a[0])) / n if n > 0 else 0.0

    while len(pts) > 3:
        prom = [prominence(i) for i in range(len(pts))]
        i = int(np.argmin(prom))
        if prom[i] >= tolerance and len(pts) <= max_corners:
            break
        del pts[i]
    return np.array(pts)


def asset_mesh(root, asset):
    """The asset's triangles (vertices, faces) in its body frame, metres at k = 1, as world.py builds its Shape prim, and its
    manifest entry."""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom
    lib, name = asset.split("/")
    with open(os.path.join(root, lib, "manifest.json"), encoding="utf-8") as f:
        entry = {e["name"]: e for e in json.load(f)["objects"]}[name]
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/Body")
    g = UsdGeom.Xform.Define(stage, "/Body/Shape")
    gx = UsdGeom.Xformable(g)
    gx.AddTranslateOp().Set(Gf.Vec3d(*(-c for c in entry["centre"])))
    gx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*entry["rotation"]))
    gx.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    g.GetPrim().GetReferences().AddReference(os.path.join(root, lib, entry["usd"]))
    cache = UsdGeom.XformCache()
    verts, faces, meshes, base = [], [], 0, 0
    for prim in Usd.PrimRange(g.GetPrim()):  # the prims world.py gives colliders
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        index = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        pts = np.array(pts, float)
        counts = np.array(counts, int)
        index = np.array(index, int)
        M = np.array(cache.GetLocalToWorldTransform(prim))  # row vectors: p_world = [p, 1] @ M
        pw = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3]
        # Fan triangles of each polygon: (first, j, j + 1).
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
        n_tri = np.maximum(counts - 2, 0)
        face = np.repeat(np.arange(len(counts)), n_tri)
        j = np.arange(n_tri.sum()) - np.repeat(np.cumsum(n_tri) - n_tri, n_tri) + 1
        tri = np.stack([index[starts[face]], index[starts[face] + j], index[starts[face] + j + 1]], axis=1)
        verts.append(pw)
        faces.append(tri + base)
        base += len(pw)
        meshes += 1
    if not verts:
        raise ValueError(f"{asset}: no mesh under its prim")
    return np.concatenate(verts), np.concatenate(faces), meshes, entry


def layer_sections(V, F, size, layers):
    """Each thin layer's section: the hull (metres, at the manifest's size) of the vertices inside it and of the triangle edges'
    crossings of its two planes. Returns the planes' z (metres) and the hulls."""
    import numpy as np
    z = V[:, 2]
    z_lo, z_hi = float(z.min()), float(z.max())
    planes = np.linspace(z_lo, z_hi, layers + 1)
    dz = (z_hi - z_lo) / layers
    edges = np.unique(np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1), axis=0)
    za, zb = z[edges[:, 0]], z[edges[:, 1]]
    lo, hi = np.minimum(za, zb), np.maximum(za, zb)
    first = np.clip(np.floor((lo - z_lo) / dz).astype(int) + 1, 0, layers)  # the first plane above the edge's low end
    last = np.clip(np.ceil((hi - z_lo) / dz).astype(int) - 1, -1, layers)   # the last plane below its high end
    count = np.maximum(last - first + 1, 0)
    e = np.repeat(np.arange(len(edges)), count)
    plane = np.repeat(first, count) + (np.arange(count.sum()) - np.repeat(np.cumsum(count) - count, count))
    keep = (lo[e] < planes[plane]) & (planes[plane] < hi[e])
    e, plane = e[keep], plane[keep]
    a, b = V[edges[e, 0]], V[edges[e, 1]]
    t = (planes[plane] - a[:, 2]) / (b[:, 2] - a[:, 2])
    cross = a[:, :2] + t[:, None] * (b[:, :2] - a[:, :2])
    order = np.argsort(plane, kind="stable")
    cross, plane = cross[order], plane[order]
    bounds = np.searchsorted(plane, np.arange(layers + 2))
    at_plane = [cross[bounds[p]:bounds[p + 1]] for p in range(layers + 1)]
    which = np.clip(np.floor((z - z_lo) / dz).astype(int), 0, layers - 1)
    sections = []
    for i in range(layers):
        inside = V[(which == i) | (np.abs(z - planes[i]) < 1e-9) | (np.abs(z - planes[i + 1]) < 1e-9), :2]
        pts = np.concatenate([inside, at_plane[i], at_plane[i + 1]])
        h = hull(pts) if len(pts) else np.zeros((0, 2))
        if len(h) < 3:  # a degenerate section (a point or a segment): a hair-thin box round it
            c = pts.mean(0) if len(pts) else np.zeros(2)
            lo2, hi2 = (pts.min(0), pts.max(0)) if len(pts) else (c, c)
            lo2, hi2 = lo2 - 1e-5, hi2 + 1e-5
            h = np.array([[lo2[0], lo2[1]], [hi2[0], lo2[1]], [hi2[0], hi2[1]], [lo2[0], hi2[1]]])
        sections.append(h)
    return planes, sections


def slabs_of(planes, sections, floor_band):
    """The cut into slabs (lists of layer indices) and the largest excess of a slab (see the module's notes). The layers below
    ``floor_band`` (metres above the bottom) are one slab of their own, left out of the cost: the arm never comes that low
    (the collision model keeps every link 10 mm above the table)."""
    import numpy as np
    L = len(sections)
    ang = np.linspace(0.0, 2 * math.pi, DIRECTIONS, endpoint=False)
    dirs = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    H = np.array([(s @ dirs.T).max(axis=0) for s in sections])  # support of each section along each direction
    band = int(np.searchsorted(planes[1:], planes[0] + floor_band, side="right"))  # layers wholly within the band
    first = min(band, L - MIN_SLABS)
    n = L - first
    # excess[a][b]: how far layers a..b's hull can be from the mesh (from ``first``): its overhang past the narrowest of them,
    # direction by direction, but at most the slab's height. Where the overhang is more than the height the surface is
    # nearly level (a ledge, a shoulder seen from below), and the mesh is within that height of the prism's face, above or
    # below it: a wall leaning by an angle overhangs by height x tan, and is height x sin from the face.
    excess = np.full((n, n), np.inf)
    for a in range(n):
        hi = H[first + a].copy()
        lo = H[first + a].copy()
        for b in range(a, n):
            np.maximum(hi, H[first + b], out=hi)
            np.minimum(lo, H[first + b], out=lo)
            excess[a, b] = min(float((hi - lo).max()), float(planes[first + b + 1] - planes[first + a]))
    most = MAX_SLABS - (1 if first > 0 else 0)
    # cost[s][b]: the least worst overhang cutting layers 0..b into s slabs; cut[s][b]: where the last of them starts.
    cost = np.full((most + 1, n), np.inf)
    cut = np.zeros((most + 1, n), int)
    cost[1] = excess[0]
    for s in range(2, most + 1):
        for b in range(s - 1, n):
            cand = np.maximum(cost[s - 1, s - 2:b], excess[s - 1:b + 1, b])
            i = int(np.argmin(cand))
            cost[s, b] = cand[i]
            cut[s, b] = s - 1 + i
    least = MIN_SLABS - (1 if first > 0 else 0)
    goal = max(EXCESS_GOAL_M, float(cost[most, n - 1])) + EXCESS_SLACK_M
    slabs = min(s for s in range(least, most + 1) if cost[s, n - 1] <= goal)
    groups, b = [], n - 1
    for s in range(slabs, 0, -1):
        a = cut[s, b] if s > 1 else 0
        groups.append(list(range(first + a, first + b + 1)))
        b = a - 1
    groups = groups[::-1]
    if first > 0:
        groups.insert(0, list(range(first)))
    return groups, float(cost[slabs, n - 1])


def fraction(value, size):
    """A coordinate as a fraction of the asset's size, to DECIMALS places (never -0.0)."""
    return round(float(value) / float(size), DECIMALS) + 0.0


def survey(root, asset):
    import numpy as np
    V, F, meshes, entry = asset_mesh(root, asset)
    size = np.array(entry["size"], float)
    layers = int(min(MAX_LAYERS, max(MIN_LAYERS, math.ceil(size[2] / LAYER_M))))
    planes, sections = layer_sections(V, F, size, layers)
    groups, excess = slabs_of(planes, sections, FLOOR_BAND_M)
    slabs = []
    corners = 0
    for g in groups:
        poly = hull(np.concatenate([sections[i] for i in g]))
        poly = cut_corners(poly, CORNER_TOLERANCE_M, MAX_CORNERS)
        corners = max(corners, len(poly))
        slabs.append({"z0": fraction(planes[g[0]], size[2]), "z1": fraction(planes[g[-1] + 1], size[2]),
                      "outline": [[fraction(x, size[0]), fraction(y, size[1])] for x, y in poly]})
    lo, hi = V.min(0) / size, V.max(0) / size
    info = {"meshes": meshes, "triangles": int(len(F)), "bounds": [[round(float(v), 4) for v in lo], [round(float(v), 4) for v in hi]],
            "layers": layers, "slabs": len(slabs), "corners": corners, "excessMm": round(excess * 1000, 2)}
    print(f"[shape-survey] {asset}: {json.dumps(info)}", file=sys.stderr)
    return {"size": [float(v) for v in entry["size"]], "excessMm": round(excess * 1000, 1), "slabs": slabs}


def main(argv):
    if argv[:1] == ["--assets-of"]:  # the scene's mesh assets, as arguments for the survey (runs anywhere)
        with open(argv[1], encoding="utf-8") as f:
            print(" ".join(sorted({o["asset"] for o in json.load(f)["objects"] if o.get("shape") == "mesh"})))
        return 0
    root = ASSETS
    if argv[:1] == ["--assets-root"]:
        root, argv = os.path.expanduser(argv[1]), argv[2:]
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    out = {"format": "teleop-object-shapes", "version": 1,
           "source": ("robot/isaac/shape_survey.py over each asset's render mesh (every UsdGeom.Mesh under it, as world.py "
                      "stands it), not PhysX's convex decomposition, which lies a few millimetres outside it"),
           "assets": {}}
    for asset in sorted(set(argv)):
        out["assets"][asset] = survey(root, asset)
    sys.stdout.write(dumps(out))
    return 0


def dumps(out):
    """The shapes as JSON, one slab a line."""
    lines = ["{", f' "format": {json.dumps(out["format"])},', f' "version": {out["version"]},', f' "source": {json.dumps(out["source"])},',
             ' "assets": {']
    names = sorted(out["assets"])
    for i, name in enumerate(names):
        a = out["assets"][name]
        lines.append(f'  {json.dumps(name)}: {{"size": {json.dumps(a["size"])}, "excessMm": {a["excessMm"]}, "slabs": [')
        for j, slab in enumerate(a["slabs"]):
            text = json.dumps(slab, separators=(",", ":"))
            lines.append("   " + text + ("," if j + 1 < len(a["slabs"]) else ""))
        lines.append("  ]}" + ("," if i + 1 < len(names) else ""))
    lines += [" }", "}", ""]
    return "\n".join(lines)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
