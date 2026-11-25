"""A Femto Bolt-like depth sensor for the twin's cameras (docs/adr/0039): Isaac's exact depth turned into the depth an
Orbbec Femto Bolt would record, as a new stream beside the exact one. Opt-in (``renderer.py --depth-model FILE``); without
it the renderer never imports this module's work and records exactly what it did before.

The Femto Bolt is Microsoft's Azure Kinect depth module with the same depth engine (Orbbec's documentation), so its modes,
ranges and errors are the Azure Kinect's. The model file (``robot/isaac/depth_models/*.json``) holds the mode and every
fitted number; ADR 0039 says where each comes from. In order, for each pixel of the depth sensor's image:

1. **The depth engine's smoothing.** The exact depth (mm) is box-filtered over a small window of the surface it belongs
   to (a centred 5-tap ``[0.5, 1, 1, 1, 0.5]`` kernel for ``blurPx`` 4, normalised over pixels with a hit). On a plane
   this changes nothing; across a small depth step it gives the ramp of intermediate ("flying") depths the real engine
   leaves. Where a window spans more than about ``mixMm`` the smoothing is blended out: the engine keeps big steps sharp
   and invalidates them instead.
2. **Invalidation (0),** each cause an independent draw:
   - outside the active illumination (a polygon in the pinhole's normalised coordinates: the NFOV hexagon);
   - out of the hard range ``[minM, maxM]``, or no hit at all (the room is an HDRI at infinity);
   - at a depth step: a pixel whose window spans more than ``jumpMinMm`` is invalid with probability
     ``g(range) * (bg if it lies on the far side else fg)``, ``g`` a logistic in the window's depth range;
   - low signal: the returned signal ``a = reflectance * cos(incidence) / Z^2`` (Z in m) through a logistic in ``ln a``;
   - grazing incidence: a logistic in the incidence angle;
   - a small rate of isolated dropouts.
3. **Systematic error, per take:** an offset, a scale and a smooth field over the image.
4. **Random error:** a per-frame field with ``sigma^2 = sigma0^2 + sigma1^2 Z^2 / (reflectance cos(incidence))`` and a
   static per-take field of ``staticMm``, both white noise box-filtered over ``kernelPx`` (the depth engine's spatial
   correlation), each normalised to unit variance before scaling.
5. **Quantisation** to whole millimetres; uint16, 0 = invalid.

Everything random comes from ``numpy.random.SeedSequence`` keyed by the take's seed (``BK``'s), its change, the camera and
the frame number, so a take's modelled depth is a pure function of its exact depth. Pure numpy; no Isaac.
"""

from __future__ import annotations

import functools
import hashlib
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

FORMAT = "teleop-depth-model"
VERSION = 1
STREAM = "depth_femto"  # the depth sensor's own image
ALIGNED = "depth_femto_aligned"  # the same, warped into the colour camera's image (the SDK's depth-to-colour transform)
GEOMETRIES = ("sensor", "same")
_TAG_FRAME, _TAG_TAKE = 1, 2  # SeedSequence keys: a frame's draws and a take's draws never share a key


@dataclass(frozen=True)
class Mode:
    """The depth sensor's raw image as the device delivers it: OpenCV's camera model (pixel centres at integer
    coordinates, Orbbec's and the Azure Kinect SDK's convention) with the rational distortion k1..k6, p1, p2."""
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Tuple[float, ...]  # k1 k2 k3 k4 k5 k6 p1 p2
    render_scale: float  # the centred pinhole Isaac renders has focal length render_scale * fx
    min_m: float
    max_m: float


@dataclass(frozen=True)
class Rig:
    camera: str  # the scene camera (CM id) or view camera (view<k>) the depth sensor belongs to
    geometry: str  # "sensor": rendered from the depth sensor's own pose and mode; "same": the camera's own exact depth
    streams: Tuple[str, ...]
    mask: bool  # apply the illumination polygon (always for "sensor")


@dataclass(frozen=True)
class Spec:
    name: str
    sha256: str
    mode: Mode
    rigs: Tuple[Rig, ...]
    views: bool  # every view camera gets a "same" rig (docs/adr/0038 "V, views")
    rotation: Tuple[Tuple[float, ...], ...]  # depth -> colour, OpenCV frames (x right, y down, z forward)
    translation: Tuple[float, float, float]  # m
    polygon: Tuple[Tuple[float, float], ...]
    noise: Mapping
    systematic: Mapping
    reflectance: Mapping
    signal: Mapping
    grazing: Mapping
    dropout: float
    edges: Mapping
    quant_mm: float
    record_every: int  # record the modelled streams at frames whose number this divides (2: 15 fps of 30)
    doc: Mapping

    def rig(self, camera: str) -> Optional[Rig]:
        return next((r for r in self.rigs if r.camera == camera), None)


def _num(d: Mapping, key: str, lo: float, hi: float, where: str) -> float:
    if key not in d:
        raise ValueError(f"{where}: {key} is missing")
    v = d[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
        raise ValueError(f"{where}: {key} must be a number in [{lo}, {hi}], not {v!r}")
    return float(v)


def canonical_sha256(doc: Mapping) -> str:
    return hashlib.sha256(json.dumps(doc, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def parse(doc: Mapping, scene_cameras: Optional[Sequence[Mapping]] = None) -> Spec:
    """A model file's document, checked. ``scene_cameras`` (the scene file's ``cameras``) checks the rigs' cameras."""
    if doc.get("format") != FORMAT or doc.get("version") != VERSION:
        raise ValueError(f"not a {FORMAT} version {VERSION} document")
    m = doc.get("mode") or {}
    dist = m.get("distortion") or {}
    names = ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")
    if any(k not in names for k in dist):
        raise ValueError(f"mode: distortion takes only {names}")
    mode = Mode(str(m.get("name", "")), int(_num(m, "width", 16, 4096, "mode")), int(_num(m, "height", 16, 4096, "mode")),
                _num(m, "fx", 1, 1e5, "mode"), _num(m, "fy", 1, 1e5, "mode"), _num(m, "cx", 0, 4096, "mode"),
                _num(m, "cy", 0, 4096, "mode"), tuple(_num(dist, k, -100, 100, "mode.distortion") if k in dist else 0.0 for k in names),
                _num(m, "renderScale", 0.25, 2.0, "mode"), _num(m, "minM", 0.0, 100.0, "mode"), _num(m, "maxM", 0.01, 65.0, "mode"))
    if mode.min_m >= mode.max_m:
        raise ValueError("mode: minM must be below maxM")
    if not (0 < mode.cx < mode.width and 0 < mode.cy < mode.height):
        raise ValueError("mode: the principal point must be inside the image")
    ext = doc.get("depthToColour") or {}
    rot = ext.get("rotation")
    if not (isinstance(rot, list) and len(rot) == 3 and all(isinstance(r, list) and len(r) == 3 for r in rot)):
        raise ValueError("depthToColour: rotation must be a 3x3 list")
    rotation = tuple(tuple(float(x) for x in r) for r in rot)
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-3:
                raise ValueError("depthToColour: rotation is not orthonormal")
    t = ext.get("translationM")
    if not (isinstance(t, list) and len(t) == 3 and all(isinstance(x, (int, float)) and abs(x) < 0.5 for x in t)):
        raise ValueError("depthToColour: translationM must be 3 numbers under 0.5 m")
    poly = (doc.get("illumination") or {}).get("polygonTan")
    if not (isinstance(poly, list) and len(poly) >= 3 and all(isinstance(p, list) and len(p) == 2 for p in poly)):
        raise ValueError("illumination: polygonTan must list at least 3 [x, y] points")
    n, s, r, g, e = (doc.get(k) or {} for k in ("noise", "systematic", "reflectance", "signal", "edges"))
    gz = doc.get("grazing") or {}
    for key, lo, hi in (("sigma0Mm", 0, 50), ("sigma1MmPerM", 0, 50), ("staticMm", 0, 50), ("kernelPx", 1, 9)):
        _num(n, key, lo, hi, "noise")
    for key, lo, hi in (("offsetSdMm", 0, 50), ("scaleSd", 0, 0.05), ("fieldSdMm", 0, 50)):
        _num(s, key, lo, hi, "systematic")
    if r.get("source") not in ("luminance", "constant"):
        raise ValueError("reflectance: source must be luminance or constant")
    for key in ("lo", "hi", "white", "constant"):
        _num(r, key, 0.001, 1.0, "reflectance")
    _num(g, "a50", 1e-9, 100, "signal")
    _num(g, "width", 0.01, 10, "signal")
    if g.get("saturation") is not None:
        _num(g, "saturation", 1e-6, 1e9, "signal")
    _num(gz, "deg50", 0, 90, "grazing")
    _num(gz, "widthDeg", 0.1, 45, "grazing")
    for key, lo, hi in (("blurPx", 1, 9), ("windowPx", 3, 15), ("jumpMinMm", 1, 10000), ("bg", 0, 1), ("fg", 0, 1),
                        ("r50Mm", 0, 100000), ("rWidthMm", 1, 100000), ("mixMm", 1, 1e6), ("mixWidthMm", 1, 1e5)):
        _num(e, key, lo, hi, "edges")
    if int(e["windowPx"]) % 2 != 1:
        raise ValueError("edges: windowPx must be odd")
    names = {c["id"]: c for c in (scene_cameras or [])}
    if not _monotonic(mode):
        raise ValueError("mode: the distortion folds over inside the image (not invertible)")
    rigs = []
    for c in doc.get("cameras") or []:
        geometry = c.get("geometry", "sensor")
        if geometry not in GEOMETRIES:
            raise ValueError(f"cameras: geometry must be one of {GEOMETRIES}, not {geometry!r}")
        streams = tuple(c.get("streams") or [STREAM])
        if not streams or any(st not in (STREAM, ALIGNED) for st in streams) or len(set(streams)) != len(streams):
            raise ValueError(f"cameras: streams must be {STREAM} and optionally {ALIGNED}")
        if ALIGNED in streams and geometry != "sensor":
            raise ValueError(f"cameras: {ALIGNED} needs geometry sensor (with same the depth is already the camera's)")
        cid = str(c.get("camera", ""))
        if scene_cameras is not None:
            if cid not in names:
                raise ValueError(f"cameras: {cid!r} is not a camera of the scene file")
            if geometry == "sensor" and names[cid].get("mount"):
                raise ValueError(f"cameras: {cid!r} rides on {names[cid]['mount']}; geometry sensor is for a camera fixed in the room")
            if geometry == "same" and "depth" not in (names[cid].get("record") or []):
                raise ValueError(f"cameras: {cid!r} records no depth for geometry same to use")
        rigs.append(Rig(cid, geometry, streams, True if geometry == "sensor" else bool(c.get("mask", False))))
    if len({r.camera for r in rigs}) != len(rigs):
        raise ValueError("cameras: a camera is listed twice")
    if not rigs and not doc.get("views"):
        raise ValueError("cameras: none, and views is off: the model would record nothing")
    return Spec(str(doc.get("name", "")), canonical_sha256(doc), mode, tuple(rigs), bool(doc.get("views", False)), rotation,
                tuple(float(x) for x in t), tuple((float(p[0]), float(p[1])) for p in poly), n, s, r, g, gz,
                _num(doc, "dropout", 0, 0.5, "model"), e, _num(doc, "quantMm", 0.01, 100, "model"),
                int(_num(doc, "recordEvery", 1, 30, "model")) if "recordEvery" in doc else 1, doc)


def load(path: str, scene_cameras: Optional[Sequence[Mapping]] = None) -> Optional[Spec]:
    """The model file at ``path``, or None for "" (the model off: the renderer records exactly as before)."""
    if not path:
        return None
    with open(path, encoding="utf-8") as f:
        return parse(json.load(f), scene_cameras)


def model_json(spec: Spec, source: str) -> bytes:
    """``isaac/depth_model.json``: the model in force, as read, its file name and the sha256 ``FM`` names."""
    doc = {"format": "teleop-depth-model-used", "version": 1, "source": source, "sha256": spec.sha256, "spec": spec.doc}
    return (json.dumps(doc, indent=1, sort_keys=True) + "\n").encode("utf-8")


# --- poses ---------------------------------------------------------------------------------------------------------------

def _q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _q_rot(q, v):
    w, x, y, z = q
    p = _q_mul(_q_mul(q, (0.0, v[0], v[1], v[2])), (w, -x, -y, -z))
    return p[1], p[2], p[3]


def _q_from_matrix(m) -> Tuple[float, float, float, float]:
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
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
    q = tuple(c / n for c in q)
    return q if q[0] >= 0 else tuple(-c for c in q)


def sensor_local(spec: Spec) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """The depth sensor's pose in its colour camera's USD frame (looking along -z, +y up), as the renderer's poses are.

    The model file's extrinsic is the SDK's depth-to-colour one: a point x_d in the depth sensor's OpenCV frame (x right,
    y down, z forward) is R x_d + t in the colour camera's. So the depth sensor stands at t in the colour camera's frame,
    its axes R's columns. A USD camera frame is the OpenCV one with y and z flipped (F = diag(1, -1, -1)): position F t,
    rotation F R F."""
    t, r = spec.translation, spec.rotation
    f = (1.0, -1.0, -1.0)
    pos = (f[0] * t[0], f[1] * t[1], f[2] * t[2])
    m = [[f[i] * r[i][j] * f[j] for j in range(3)] for i in range(3)]
    return pos, _q_from_matrix(m)


def compose(parent, local):
    """``parent`` then ``local``: (position, quaternion w x y z) pairs, as renderer.compose."""
    pp, pq = parent
    lp, lq = local
    r = _q_rot(pq, lp)
    return (pp[0] + r[0], pp[1] + r[1], pp[2] + r[2]), _q_mul(pq, lq)


def distort(mode: Mode, x, y):
    """OpenCV's rational model: normalised undistorted (x, y) to normalised distorted."""
    k1, k2, k3, k4, k5, k6, p1, p2 = mode.distortion
    r2 = x * x + y * y
    radial = (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1 + r2 * (k4 + r2 * (k5 + r2 * k6)))
    return (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)


def undistort(mode: Mode, xd, yd, iterations: int = 30):
    """The inverse of :func:`distort` by fixed-point iteration, as cv2.undistortPoints does."""
    k1, k2, k3, k4, k5, k6, p1, p2 = mode.distortion
    x, y = xd, yd
    for _ in range(iterations):
        r2 = x * x + y * y
        icdist = (1 + r2 * (k4 + r2 * (k5 + r2 * k6))) / (1 + r2 * (k1 + r2 * (k2 + r2 * k3)))
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = (xd - dx) * icdist, (yd - dy) * icdist
    return x, y


def _monotonic(mode: Mode) -> bool:
    """The raw image's corners and edge midpoints undistort and distort back to themselves (the model is invertible
    over the image)."""
    for u in (0.0, mode.width / 2, mode.width - 1.0):
        for v in (0.0, mode.height / 2, mode.height - 1.0):
            xd, yd = (u - mode.cx) / mode.fx, (v - mode.cy) / mode.fy
            x, y = undistort(mode, xd, yd)
            bx, by = distort(mode, x, y)
            if not (math.isfinite(x) and math.isfinite(y)) or abs(bx - xd) > 1e-6 or abs(by - yd) > 1e-6:
                return False
    return True


@dataclass(frozen=True)
class Geometry:
    """How the raw image is made from Isaac's centred pinhole render: each raw pixel's ray (undistorted normalised x, y,
    z = 1) and the render pixel nearest it (``inside`` False where the ray falls outside the render)."""
    render_width: int
    render_height: int
    render_f: float
    x: object
    y: object
    row: object
    col: object
    inside: object
    flat: object  # row * render_width + col, for np.take


@functools.lru_cache(maxsize=8)
def geometry(mode: Mode) -> Geometry:
    """The raw grid's rays and the centred pinhole that covers them: focal length ``render_scale * fx``, sized to the
    largest undistorted extent of the raw image's border in each axis (Isaac renders a principal point at the image's
    centre). Pixel centres: integers in the raw image (OpenCV), +0.5 in Isaac's render."""
    import numpy as np

    v, u = np.mgrid[0:mode.height, 0:mode.width].astype(np.float64)
    x, y = undistort(mode, (u - mode.cx) / mode.fx, (v - mode.cy) / mode.fy)
    f = mode.render_scale * mode.fx
    border = np.zeros((mode.height, mode.width), bool)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    hw = int(math.ceil(float(np.abs(x[border]).max()) * f + 0.5))
    hh = int(math.ceil(float(np.abs(y[border]).max()) * f + 0.5))
    col = np.floor(hw + f * x).astype(np.int64)
    row = np.floor(hh + f * y).astype(np.int64)
    inside = (col >= 0) & (col < 2 * hw) & (row >= 0) & (row < 2 * hh)
    col, row = np.clip(col, 0, 2 * hw - 1), np.clip(row, 0, 2 * hh - 1)
    flat = (row * (2 * hw) + col).astype(np.intp)
    for a in (x, y, row, col, inside, flat):
        a.setflags(write=False)
    return Geometry(2 * hw, 2 * hh, f, x.astype(np.float32), y.astype(np.float32), row, col, inside, flat)


def to_raw(mode: Mode, render):
    """A render (H_r x W_r [x C]) resampled onto the raw grid, nearest pixel (no depth is ever blended); depth outside
    the render is inf."""
    import numpy as np

    g = geometry(mode)
    a = np.asarray(render)
    if a.shape[:2] != (g.render_height, g.render_width):
        raise ValueError(f"the render is {a.shape[:2]}, the mode's geometry wants {(g.render_height, g.render_width)}")
    if a.ndim == 2:
        out = np.take(a.reshape(-1), g.flat)
    else:
        out = np.take(a.reshape(-1, a.shape[2]), g.flat, axis=0)
    if out.ndim == 2 and np.issubdtype(out.dtype, np.floating):
        out = np.where(g.inside, out, np.inf).astype(np.float32)
    return out


def pinhole_rays(intr: Mapping, shape: Tuple[int, int]):
    """A pinhole camera's rays as CM gives it (pixel centres at +0.5)."""
    import numpy as np

    h, w = shape
    x = ((np.arange(w, dtype=np.float32) + 0.5 - np.float32(intr["cx"])) / np.float32(intr["fx"]))[None, :]
    y = ((np.arange(h, dtype=np.float32) + 0.5 - np.float32(intr["cy"])) / np.float32(intr["fy"]))[:, None]
    return np.ascontiguousarray(np.broadcast_to(x, (h, w))), np.ascontiguousarray(np.broadcast_to(y, (h, w)))


def raw_intrinsics(mode: Mode) -> Dict[str, object]:
    return {"width": mode.width, "height": mode.height, "fx": mode.fx, "fy": mode.fy, "cx": mode.cx, "cy": mode.cy,
            "distortion": mode.distortion, "pixelCentre": 0.0}


# --- the model -----------------------------------------------------------------------------------------------------------

def camera_key(camera: str) -> int:
    return int(hashlib.sha256(camera.encode("utf-8")).hexdigest()[:8], 16)


def frame_rng(seed: int, change: int, camera: str, frame: int):
    import numpy as np

    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([_TAG_FRAME, int(seed) & (2 ** 63 - 1), max(0, int(change)),
                                                                        camera_key(camera), max(0, int(frame))])))


def take_rng(seed: int, change: int, camera: str):
    import numpy as np

    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([_TAG_TAKE, int(seed) & (2 ** 63 - 1), max(0, int(change)),
                                                                        camera_key(camera)])))


def correlated_noise(rng, shape: Tuple[int, int], k: int):
    """Unit-variance white noise box-filtered over k x k pixels: the depth engine's spatial correlation (ADR 0039: HILO's
    raw depth has rho 0.73, 0.46, 0.22, 0.05 at lags 1-4 px, a 4-px box's 0.75, 0.5, 0.25, 0)."""
    import cv2
    import numpy as np

    h, w = shape
    white = rng.standard_normal((h + k - 1, w + k - 1), dtype=np.float32)
    summed = cv2.boxFilter(white, -1, (k, k), anchor=(0, 0), normalize=False, borderType=cv2.BORDER_CONSTANT)
    return summed[:h, :w] * np.float32(1.0 / k)


@functools.lru_cache(maxsize=16)
def _take_fields(seed: int, change: int, camera: str, h: int, w: int, kernel: int, static_mm: float, field_mm: float,
                 offset_sd: float, scale_sd: float):
    """The take's systematic error and static noise for one camera: (offset mm, scale, smooth field + static field in mm).
    A pure function of its key, cached since every frame of a take reuses it."""
    import numpy as np

    rng = take_rng(seed, change, camera)
    offset = float(rng.normal() * offset_sd)
    scale = float(rng.normal() * scale_sd)
    yy, xx = np.meshgrid(np.linspace(0.0, 1.0, h, dtype=np.float32), np.linspace(0.0, 1.0, w, dtype=np.float32), indexing="ij")
    field = np.zeros((h, w), np.float32)
    waves = 4
    for _ in range(waves):  # a few slow waves (0.3-1 cycles over the image), together sd field_mm
        f = rng.uniform(0.3, 1.0)
        a = rng.uniform(0.0, 2 * math.pi)
        phase = rng.uniform(0.0, 2 * math.pi)
        field += np.float32(field_mm * math.sqrt(2.0 / waves)) * np.cos(2 * math.pi * f * (math.cos(a) * xx + math.sin(a) * yy) + phase)
    field += np.float32(static_mm) * correlated_noise(rng, (h, w), kernel)
    field.setflags(write=False)
    return offset, scale, field


def take_systematic(spec: Spec, seed: int, change: int, camera: str, shape: Tuple[int, int]) -> Tuple[float, float]:
    """This take's offset (mm) and scale for one camera, as FM records them."""
    s, n = spec.systematic, spec.noise
    off, sc, _ = _take_fields(int(seed), int(change), camera, int(shape[0]), int(shape[1]), int(n["kernelPx"]),
                              float(n["staticMm"]), float(s["fieldSdMm"]), float(s["offsetSdMm"]), float(s["scaleSd"]))
    return off, sc


def _window_max(a, r: int):
    """The maximum over a (2r + 1)^2 window (outside the image: ignored)."""
    import cv2
    import numpy as np

    return cv2.dilate(a, np.ones((2 * r + 1, 2 * r + 1), np.uint8), borderType=cv2.BORDER_REPLICATE)


def _blur5(z, hit, k: int):
    """The depth engine's smoothing: z averaged over the hits of a centred window, ``[0.5, 1, ..., 1, 0.5]`` of width k
    (k + 1 taps; k = 1 is no smoothing)."""
    import cv2
    import numpy as np

    if k <= 1:
        return z
    r = (k + 1) // 2 if k % 2 == 0 else k // 2
    taps = np.ones(2 * r + 1, np.float32)
    if k % 2 == 0:
        taps[0] = taps[-1] = 0.5
    zs = np.where(hit, z, np.float32(0.0)).astype(np.float32)
    hs = hit.astype(np.float32)
    acc_z = cv2.sepFilter2D(zs, -1, taps, taps, borderType=cv2.BORDER_CONSTANT)
    acc_h = cv2.sepFilter2D(hs, -1, taps, taps, borderType=cv2.BORDER_CONSTANT)
    return np.where(acc_h > 0.5, acc_z / np.maximum(acc_h, np.float32(1e-6)), z).astype(np.float32)


def illumination_mask(polygon: Sequence[Tuple[float, float]], x, y):
    """Pixels whose ray (x, y) lies inside the polygon (even-odd rule)."""
    import numpy as np

    inside = np.zeros(np.shape(x), bool)
    n = len(polygon)
    for i in range(n):
        (x1, y1), (x2, y2) = polygon[i], polygon[(i + 1) % n]
        if y1 == y2:
            continue
        cross = ((y1 > y) != (y2 > y)) & (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1)
        inside ^= cross
    return inside


_MASKS: Dict[tuple, object] = {}


def _mask_for(polygon, key: tuple, x, y):
    m = _MASKS.get((polygon, key))
    if m is None:
        m = illumination_mask(polygon, x, y)
        m.setflags(write=False)
        if len(_MASKS) > 16:
            _MASKS.clear()
        _MASKS[(polygon, key)] = m
    return m


def incidence_cos(z, x, y):
    """|cos| of the angle between each pixel's ray (x, y, 1) and the surface normal, from the depth image's own points.
    Along each axis the tangent is the one-sided difference to whichever neighbour is nearer in depth, so a pixel at a
    depth step takes its own surface's slope, not the step's. nan depth gives 1."""
    import numpy as np

    z = np.asarray(z, np.float32)
    px, py = x * z, y * z
    zp, xp, yp = (np.pad(a, 1, mode="edge") for a in (z, px, py))

    def tangent(fwd, back):
        (fz, fx_, fy_), (bz, bx, by) = fwd, back
        use_f = ~(np.abs(bz) < np.abs(fz))  # nan compares False: the forward side unless the back one is nearer
        return np.where(use_f, fx_, bx), np.where(use_f, fy_, by), np.where(use_f, fz, bz)

    c = (slice(1, -1), slice(1, -1))
    with np.errstate(invalid="ignore", divide="ignore"):  # nan (no hit) and inf take part and come out as nan
        ux, uy, uz = tangent(tuple(a[1:-1, 2:] - a[c] for a in (zp, xp, yp)), tuple(a[c] - a[1:-1, :-2] for a in (zp, xp, yp)))
        vx, vy, vz = tangent(tuple(a[2:, 1:-1] - a[c] for a in (zp, xp, yp)), tuple(a[c] - a[:-2, 1:-1] for a in (zp, xp, yp)))
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        num = np.abs(nx * x + ny * y + nz)  # n . (x, y, 1)
        den = np.sqrt((nx * nx + ny * ny + nz * nz) * (x * x + y * y + 1.0))
        cs = num / den
    return np.where(np.isfinite(cs), np.minimum(cs, np.float32(1.0)), np.float32(1.0)).astype(np.float32)


def reflectance(spec: Spec, rgb, shape: Tuple[int, int]):
    """Each pixel's reflectance at 850 nm, from the visible image's linear luminance when the model says so (a proxy: the
    NIR reflectance of a material is not its colour), else the constant."""
    import numpy as np

    r = spec.reflectance
    if r["source"] != "luminance" or rgb is None:
        return np.full(shape, np.float32(r["constant"]), np.float32)
    lut = _srgb_lut()
    a = np.asarray(rgb)[..., :3]
    lum = 0.2126 * lut[a[..., 0]] + 0.7152 * lut[a[..., 1]] + 0.0722 * lut[a[..., 2]]
    t = np.clip(lum / np.float32(r["white"]), 0.0, 1.0)
    return (np.float32(r["lo"]) + np.float32(r["hi"] - r["lo"]) * t).astype(np.float32)


@functools.lru_cache(maxsize=1)
def _srgb_lut():
    import numpy as np

    c = np.arange(256, dtype=np.float64) / 255.0
    lut = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4).astype(np.float32)
    lut.setflags(write=False)
    return lut


def _logistic(x):
    import numpy as np

    return (1.0 / (1.0 + np.exp(-np.clip(x, -60, 60)))).astype(np.float32)


@dataclass(frozen=True)
class Frame:
    """What the model needs of one frame besides the exact depth: whose it is and how to draw its noise."""
    camera: str
    seed: int
    change: int
    frame: int
    x: object  # each pixel's ray, undistorted normalised (x, y, 1): geometry(mode) or pinhole_rays(CM)
    y: object
    key: tuple  # names the ray grid, for the cached illumination mask
    mask: bool = True  # apply the illumination polygon


def apply(spec: Spec, depth_m, fr: Frame, rgb=None, causes: Optional[Dict[str, object]] = None):
    """The Femto-like depth (uint16 mm, 0 invalid) of an exact depth image (float metres to the image plane; inf, nan or
    <= 0 where nothing was hit). ``rgb``: the same view's colour (uint8), for the reflectance. ``causes``, when given, gets
    each invalidation cause's boolean image (for tests and the twin check's report). The input is not modified."""
    import numpy as np

    z_in = np.asarray(depth_m, dtype=np.float32)
    h, w = z_in.shape
    mode, e, n = spec.mode, spec.edges, spec.noise
    hit = np.isfinite(z_in) & (z_in > 0)
    z = np.where(hit, z_in * np.float32(1000.0), np.float32(0.0)).astype(np.float32)  # mm
    rng = frame_rng(fr.seed, fr.change, fr.camera, fr.frame)
    kernel = int(n["kernelPx"])
    noise_t = correlated_noise(rng, (h, w), kernel)
    u = rng.random((4, h, w), dtype=np.float32)

    # 1. the engine's smoothing, and the depth steps
    r = int(e["windowPx"]) // 2
    big = np.float32(1e7)
    zmax = _window_max(np.where(hit, z, -big).astype(np.float32), r)
    zmin = -_window_max(np.where(hit, -z, -big).astype(np.float32), r)
    nohit_near = _window_max((~hit).astype(np.uint8), r).astype(bool)
    span = np.where(nohit_near, big, zmax - zmin)
    straddle = hit & (span > np.float32(e["jumpMinMm"]))
    far = hit & ~nohit_near & (z > 0.5 * (zmax + zmin))
    g = _logistic((span - np.float32(e["r50Mm"])) / np.float32(e["rWidthMm"]))
    # The smoothing mixes a step's two sides only while the step is small (HILO: 3.4 flying px per 40-100 mm step, 0.7 per
    # step over 300 mm, which the engine invalidates instead): blended out above mixMm.
    mix = _logistic((np.float32(e["mixMm"]) - span) / np.float32(e["mixWidthMm"]))
    zb = z + mix * (_blur5(z, hit, int(e["blurPx"])) - z)
    p_edge = np.where(straddle, g * np.where(far, np.float32(e["bg"]), np.float32(e["fg"])), np.float32(0.0))

    # 2. signal and incidence
    cosi = incidence_cos(np.where(hit, z, np.nan), fr.x, fr.y)
    rho = reflectance(spec, rgb, (h, w))
    zm = np.maximum(z, np.float32(1.0)) / np.float32(1000.0)
    a = rho * np.maximum(cosi, np.float32(1e-4)) / (zm * zm)
    sig = spec.signal
    p_sig = _logistic(-(np.log(a) - np.float32(math.log(sig["a50"]))) / np.float32(sig["width"]))
    ang = np.degrees(np.arccos(np.clip(cosi, 0.0, 1.0)))
    p_graze = _logistic((ang - np.float32(spec.grazing["deg50"])) / np.float32(spec.grazing["widthDeg"]))

    illum = _mask_for(spec.polygon, fr.key, fr.x, fr.y) if fr.mask else np.ones((h, w), bool)
    out_range = hit & ((z < np.float32(mode.min_m * 1000.0)) | (z > np.float32(mode.max_m * 1000.0)))
    edge_inv = u[0] < p_edge
    sig_inv = u[1] < p_sig
    if sig.get("saturation") is not None:
        sig_inv |= a > np.float32(sig["saturation"])
    graze_inv = u[2] < p_graze
    drop_inv = u[3] < np.float32(spec.dropout)
    invalid = ~hit | ~illum | out_range | edge_inv | sig_inv | graze_inv | drop_inv

    # 3-4. systematic and random error
    s = spec.systematic
    offset, scale, fields = _take_fields(int(fr.seed), int(fr.change), fr.camera, h, w, kernel, float(n["staticMm"]),
                                         float(s["fieldSdMm"]), float(s["offsetSdMm"]), float(s["scaleSd"]))
    sigma = np.sqrt(np.float32(n["sigma0Mm"]) ** 2 + np.float32(n["sigma1MmPerM"]) ** 2 * zm * zm / np.maximum(rho * cosi, np.float32(1e-3)))
    d = zb * np.float32(1.0 + scale) + np.float32(offset) + fields + sigma * noise_t

    # 5. quantisation
    q = np.float32(spec.quant_mm)
    mm = np.round(d / q) * q
    out = np.where(invalid | (mm < 1) | (mm > 65535), 0, mm).astype(np.uint16)
    if causes is not None:
        causes.update({"nohit": ~hit, "illumination": hit & ~illum, "range": out_range, "edge": hit & edge_inv,
                       "signal": hit & sig_inv, "grazing": hit & graze_inv, "dropout": hit & drop_inv, "straddle": straddle})
    return out


def align_to_colour(depth_mm, x_rays, y_rays, colour_intr: Mapping, rotation, translation):
    """The SDK's depth-to-colour transform (k4a_transformation_depth_image_to_color_camera, Orbbec's D2C): every valid
    depth pixel back-projected along its ray, moved into the colour camera's frame (R x + t) and splatted onto its four nearest colour
    pixels, the nearest depth winning; uint16 mm of Z in the colour camera's frame, 0 where nothing landed."""
    import numpy as np

    d = np.asarray(depth_mm)
    v, u_ = np.nonzero(d > 0)
    z = d[v, u_].astype(np.float64) / 1000.0
    x = np.asarray(x_rays)[v, u_] * z
    y = np.asarray(y_rays)[v, u_] * z
    rot = np.asarray(rotation, float)
    p = rot @ np.stack([x, y, z]) + np.asarray(translation, float)[:, None]
    ok = p[2] > 1e-6
    p = p[:, ok]
    uc = colour_intr["cx"] + colour_intr["fx"] * p[0] / p[2] - 0.5  # continuous pixel index (centres at integers)
    vc = colour_intr["cy"] + colour_intr["fy"] * p[1] / p[2] - 0.5
    wc, hc = int(colour_intr["width"]), int(colour_intr["height"])
    zc = np.round(p[2] * 1000.0)
    best = np.full(hc * wc, np.inf)
    u0, v0 = np.floor(uc).astype(np.int64), np.floor(vc).astype(np.int64)
    for du in (0, 1):
        for dv in (0, 1):
            uu, vv = u0 + du, v0 + dv
            sel = (uu >= 0) & (uu < wc) & (vv >= 0) & (vv < hc)
            np.minimum.at(best, vv[sel] * wc + uu[sel], zc[sel])
    out = np.where(np.isfinite(best) & (best <= 65535), best, 0).astype(np.uint16)
    return out.reshape(hc, wc)


# --- the renderer's side -----------------------------------------------------------------------------------------------

@dataclass
class Job:
    """One modelled stream of one frame, built on the render thread from arrays already read, finished on an encode
    thread (:func:`finish`)."""
    spec: Spec
    frame: Frame
    depth_m: object  # the exact depth (float m): the depth sensor's render (raw=False) or the camera's own image
    rgb: object = None
    raw: bool = False  # depth_m and rgb are Isaac's centred pinhole render, to be resampled onto the raw grid first
    aligned_to: Optional[Mapping] = None  # the colour camera's intrinsics (CM), for the aligned stream


def finish(job: Job):
    """The job's uint16 image: the modelled depth, or with ``aligned_to`` that depth in the colour camera's image."""
    depth, rgb = job.depth_m, job.rgb
    if job.raw:
        depth = to_raw(job.spec.mode, depth)
        rgb = None if rgb is None else to_raw(job.spec.mode, rgb)
    out = apply(job.spec, depth, job.frame, rgb)
    if job.aligned_to is not None:
        out = align_to_colour(out, job.frame.x, job.frame.y, job.aligned_to, job.spec.rotation, job.spec.translation)
    return out


def sensor_frame(spec: Spec, camera: str, seed: int, change: int, frame: int) -> Frame:
    """A depth sensor's frame on the raw grid."""
    g = geometry(spec.mode)
    return Frame(camera, seed, change, frame, g.x, g.y, ("raw", spec.mode), True)


def same_frame(spec: Spec, rig_mask: bool, camera: str, intr: Mapping, seed: int, change: int, frame: int) -> Frame:
    """A camera's own pinhole (geometry same): its CM intrinsics, pixel centres at +0.5."""
    shape = (int(intr["height"]), int(intr["width"]))
    key = ("pinhole", float(intr["fx"]), float(intr["fy"]), float(intr["cx"]), float(intr["cy"]), shape)
    x, y = _PINHOLE.get(key) or _PINHOLE.setdefault(key, pinhole_rays(intr, shape))
    return Frame(camera, seed, change, frame, x, y, key, rig_mask)


_PINHOLE: Dict[tuple, tuple] = {}


def due(spec: Optional[Spec], frame: int) -> bool:
    """Whether the modelled streams are recorded at this frame (``recordEvery``)."""
    return spec is not None and int(frame) % spec.record_every == 0


def streams(spec: Optional[Spec], view_ids: Sequence[str] = ()) -> List[str]:
    """The sidecar streams the model adds, ``camera.stream``: none when off."""
    if spec is None:
        return []
    out = [f"{r.camera}.{st}" for r in spec.rigs for st in r.streams]
    if spec.views:
        out += [f"{v}.{STREAM}" for v in view_ids if spec.rig(v) is None]
    return out


def fm_fields(spec: Spec, rig: Rig, stream: str, intr: Mapping, pose: Sequence[float], pose_frame: str, seed: int, change: int,
              shape: Tuple[int, int]) -> tuple:
    """One ``FM`` record's fields (trainer/contract/SESSION_FORMAT.md): the stream's camera model (with its distortion and
    pixel-centre convention) and pose, the model's sha256 and this take's systematic draws for the camera. ``shape`` is
    the modelled image's (the raw grid's for geometry sensor, whose draws the aligned stream shares)."""
    off, sc = take_systematic(spec, seed, change, rig.camera, shape)
    dist = intr.get("distortion") or ()
    return (rig.camera, stream, rig.geometry if stream == STREAM else "aligned", int(intr["width"]), int(intr["height"]),
            round(float(intr["fx"]), 6), round(float(intr["fy"]), 6), round(float(intr["cx"]), 6), round(float(intr["cy"]), 6),
            float(intr.get("pixelCentre", 0.5)), ":".join(repr(float(d)) for d in dist), spec.mode.min_m, spec.mode.max_m,
            *[round(float(v), 7) for v in pose], pose_frame, spec.sha256, round(off, 4), round(sc, 7))


def default_dir() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "depth_models")


# --- our own Femto Bolts' calibration -------------------------------------------------------------------------------------

ORBBEC_MODELS = {0: "none", 3: "brown-conrady", 4: "brown-conrady-k6"}  # OBCameraDistortionModel values the rational model covers


def from_calibration(doc: Mapping, cal: Mapping) -> Dict:
    """A model document with the depth camera model, distortion and depth-to-colour extrinsic of ``cal``, a
    ``just femto-record`` calibration.json (robot/femto/femto_record.cpp: Orbbec SDK v2's intrinsics, distortion and
    ``depth_to_color`` extrinsic in mm). The illumination polygon and every fitted number stay as they are."""
    depth = (cal.get("streams") or {}).get("depth")
    if not depth:
        raise ValueError("calibration.json has no depth stream")
    k, dist = depth["intrinsic"], depth.get("distortion") or {}
    model = int(dist.get("model", 0))
    if model not in ORBBEC_MODELS:
        raise ValueError(f"distortion model {model} is not one the rational model covers ({ORBBEC_MODELS})")
    ext = (cal.get("extrinsics") or {}).get("depth_to_color")
    if not ext:
        raise ValueError("calibration.json has no depth_to_color extrinsic")
    out = json.loads(json.dumps(doc))
    m = out["mode"]
    m.update({"width": int(depth["width"]), "height": int(depth["height"]), "fx": float(k["fx"]), "fy": float(k["fy"]),
              "cx": float(k["cx"]), "cy": float(k["cy"]),
              "distortion": {n: float(dist.get(n, 0.0)) for n in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")}})
    rot = [float(v) for v in ext["rot"]]
    out["depthToColour"] = {"rotation": [rot[0:3], rot[3:6], rot[6:9]], "translationM": [float(v) / 1000.0 for v in ext["trans"]],
                            "source": f"femto-record calibration.json, serial {cal.get('serial', '?')} ({cal.get('label', '?')})"}
    parse(out)  # still a valid model
    return out


def main(argv=None) -> int:
    import argparse

    a = argparse.ArgumentParser(description="the twin's Femto Bolt depth model (docs/adr/0039)")
    a.add_argument("--from-calibration", required=True, help="a just femto-record calibration.json")
    a.add_argument("--model", default=os.path.join(default_dir(), "femto-nfov.json"), help="the model file to start from")
    args = a.parse_args(argv)
    with open(args.model, encoding="utf-8") as f:
        doc = json.load(f)
    with open(args.from_calibration, encoding="utf-8") as f:
        cal = json.load(f)
    print(json.dumps(from_calibration(doc, cal), indent=1))
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
