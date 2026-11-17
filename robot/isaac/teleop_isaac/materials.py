"""Real materials for the twin's simple shapes and surfaces (docs/adr/0037 amendment 5, and its section "Surfaces and flat
clutter"): Poly Haven's CC0 PBR textures, and four this module makes itself, so a box is a wooden block, a cylinder a metal
or wooden can, and the table a lab bench's black resin, a laminate, wood, a cutting mat, fabric, kraft paper, cardboard or
paper, not a flat colour.

Run on the GPU machine (``just isaac-materials-fetch``, ``just isaac-materials-status``):

    python -m teleop_isaac.materials fetch
    python -m teleop_isaac.materials status

**Downloads are untrusted data.**
- **The source.** Each texture's file list comes from ``api.polyhaven.com``; its maps (colour, roughness, OpenGL normal,
  1K) are fetched only from ``https://dl.polyhaven.org/``.
- **What is kept.** A map is kept only if it matches the size and md5 the API gave, is at most :data:`MAX_BYTES`, and starts
  like the JPEG or PNG its name says. It is stored as ``<root>/<id>/<map>.<ext>``, with its sha256 in the manifest.

**Textures made here** (:data:`PROCEDURAL`). Poly Haven has no CC0 cardboard, paper or cutting mat, so ``fetch`` makes
those itself: noise on a torus (white noise blurred in the frequency domain, so every map wraps at every edge), a fixed seed
each, written as PNG with zlib. Nothing is downloaded, so they are CC0 by construction. Each manifest entry names the
:data:`GENERATOR` that made it, and ``fetch`` makes a texture again when that differs.

:data:`TEXTURES` names each texture and its class: ``wood`` or ``metal`` for the shapes, ``surface`` for the scene's looks
alone (objects never draw that class). The manifest records each one's real-world size (``sizeM``: Poly Haven's, or the
tile's for one made here), so the renderer tiles it at its true scale. :func:`pick` chooses an object's material for a
change, from the seed, the change and the object's id alone; :func:`pick_look` a surface's look, by the looks' weights.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import re
import struct
import sys
import tempfile
import urllib.parse
import urllib.request
import zlib
from typing import Dict, Mapping, Optional, Sequence, Tuple

API = "https://api.polyhaven.com"
DOWNLOAD_HOST = "dl.polyhaven.org"
LICENSE = "CC0 1.0 (Poly Haven, polyhaven.com)"
USER_AGENT = "teleop-research-renderer/1.0"
RESOLUTION = "1k"
MAX_BYTES = 30 * 1024 * 1024
MAX_API_BYTES = 8 * 1024 * 1024
DEFAULT_ROOT = os.path.expanduser("~/isaacsim/assets/textures/polyhaven")
ASSET_ID = re.compile(r"^[a-z0-9_]{1,80}$")

# The textures and what they make: wooden blocks and planks, metal cans, and table and bench tops. grey_oak_veneer_01 is also
# the table's grey laminate; oak_veneer_01 one of its woods.
TEXTURES: Dict[str, str] = {
    "american_walnut_veneer": "wood", "ash_veneer": "wood", "bamboo_veneer": "wood", "black_oak_veneer": "wood",
    "cherry_veneer": "wood", "coated_pine": "wood", "fine_grained_wood": "wood", "oak_veneer_01": "wood",
    "mocha_oak_veneer": "wood", "natural_walnut_veneer": "wood", "blue_painted_planks": "wood",
    "distressed_painted_planks": "wood", "grey_oak_veneer_01": "wood",
    "metal_plate": "metal", "metal_plate_02": "metal", "blue_metal_plate": "metal", "green_metal_rust": "metal",
    "rusty_metal": "metal", "rusty_painted_metal": "metal", "rusty_metal_02": "metal",
    # Table and bench tops (docs/adr/0037, "Surfaces and flat clutter"): the scene's looks wear them, objects never draw them.
    # Poly Haven's, CC0, each checked on its API on 2026-11-17 (a texture, with 1K colour, roughness and OpenGL normal maps).
    "smooth_concrete_floor": "surface", "laminate_floor_02": "surface", "kitchen_wood": "surface", "wood_table_001": "surface",
    "plywood": "surface", "rough_linen": "surface", "denim_fabric_03": "surface", "hessian_230": "surface",
    # Made here (PROCEDURAL), not downloaded.
    "teleop_cutting_mat": "surface", "teleop_kraft_paper": "surface", "teleop_cardboard": "surface",
    "teleop_white_paper": "surface",
}

# Each map: the files API's key, the format fetched, and the name it is stored under.
MAPS = (("Diffuse", "jpg", "diffuse.jpg"), ("Rough", "jpg", "roughness.jpg"), ("nor_gl", "png", "normal.png"))

GENERATOR = "teleop_isaac.materials procedural 1"
PROCEDURAL_LICENSE = "CC0 1.0 (made by teleop_isaac.materials from a seed; no source material)"
PROCEDURAL_PX = 1024
# id: (the tile's real size in metres, its seed, its name). The cutting mat's tile is 30 of its 1 cm squares.
PROCEDURAL: Dict[str, Tuple[float, int, str]] = {
    "teleop_cutting_mat": (0.30, 3701, "Cutting mat (green, 1 cm grid, bold lines every 5 cm)"),
    "teleop_kraft_paper": (0.40, 3702, "Kraft paper"),
    "teleop_cardboard": (0.36, 3703, "Corrugated cardboard (4.5 mm flutes)"),
    "teleop_white_paper": (0.30, 3704, "White paper"),
}


def map_ok(name: str, data: bytes) -> bool:
    if len(data) > MAX_BYTES:
        return False
    return data.startswith(b"\x89PNG\r\n\x1a\n") if name.endswith(".png") else data.startswith(b"\xff\xd8\xff")


def url_ok(url: str) -> bool:
    u = urllib.parse.urlsplit(url)
    return u.scheme == "https" and u.hostname == DOWNLOAD_HOST and ".." not in u.path


def _download(url: str, limit: int) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read(limit + 1)
    if len(data) > limit:
        raise ValueError(f"{url} is over {limit} bytes")
    return data


def _api(path: str, download) -> Dict:
    return json.loads(download(API + path, MAX_API_BYTES).decode("utf-8"))


def load_manifest(root: str) -> Dict:
    path = os.path.join(root, "manifest.json")
    if not os.path.exists(path):
        return {"format": "teleop-materials", "version": 1, "license": LICENSE, "textures": []}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_manifest(root: str, manifest: Dict) -> None:
    path = os.path.join(root, "manifest.json")
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1, sort_keys=True)
        f.write("\n")
    os.replace(path + ".tmp", path)


def _write(path: str, data: bytes) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix=".part")
    with os.fdopen(fd, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    return hashlib.sha256(data).hexdigest()


# --- textures made here ----------------------------------------------------------------------------------------------

def png(pixels) -> bytes:
    """An 8-bit PNG of ``pixels``, greyscale (h, w) or RGB (h, w, 3): one zlib stream, filter 0 on every row."""
    import numpy as np

    a = np.ascontiguousarray(pixels, dtype=np.uint8)
    if a.ndim == 2:
        colour_type, channels = 0, 1
    elif a.ndim == 3 and a.shape[2] == 3:
        colour_type, channels = 2, 3
    else:
        raise ValueError("a PNG here is greyscale (h, w) or RGB (h, w, 3)")
    h, w = a.shape[:2]
    raw = np.zeros((h, 1 + w * channels), np.uint8)
    raw[:, 1:] = a.reshape(h, w * channels)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, colour_type, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw.tobytes(), 6)) + chunk(b"IEND", b""))


def _blur(a, sx: float, sy: Optional[float] = None):
    """``a`` (n, n) blurred by a Gaussian of ``sx`` pixels along x (columns) and ``sy`` along y (rows), on the torus: it
    wraps at every edge."""
    import numpy as np

    n = a.shape[0]
    sy = sx if sy is None else sy
    fx = np.fft.fftfreq(n)[None, :]
    fy = np.fft.fftfreq(n)[:, None]
    g = np.exp(-2.0 * np.pi ** 2 * ((sx * fx) ** 2 + (sy * fy) ** 2))
    return np.real(np.fft.ifft2(np.fft.fft2(a) * g))


def periodic_noise(n: int, rng, sx: float, sy: Optional[float] = None):
    """Tileable noise, (n, n), mean 0 and spread 1: white noise from ``rng`` (a numpy ``RandomState``, whose stream never
    changes between numpy versions) blurred by :func:`_blur`."""
    out = _blur(rng.standard_normal((n, n)), sx, sy)
    out -= out.mean()
    sd = out.std()
    return out / sd if sd > 0 else out


def _specks(n: int, rng, density: float, sigma: float):
    """Sparse soft dots, 0 to 1, on the torus: about ``density`` of the pixels, each blurred by ``sigma`` pixels."""
    import numpy as np

    dots = (rng.random_sample((n, n)) < density).astype(np.float64)
    return np.clip(_blur(dots, sigma) * 2.0 * np.pi * sigma * sigma, 0.0, 1.0)


def _strokes(n: int, rng, count: int, shortest: float, longest: float, sigma: float):
    """Thin straight strokes, 0 to 1, wrapping at the edges (a cutting mat's knife marks): ``count`` of them, ``shortest``
    to ``longest`` pixels long, mostly along the grid, each softened by ``sigma`` pixels."""
    import numpy as np

    acc = np.zeros((n, n))
    for _ in range(count):
        x0, y0 = rng.uniform(0.0, n, 2)
        a = float(rng.choice([0.0, np.pi / 2])) if rng.random_sample() < 0.7 else float(rng.uniform(0.0, np.pi))
        t = np.arange(0.0, float(rng.uniform(shortest, longest)), 0.5)
        xs = np.mod(np.round(x0 + t * np.cos(a)).astype(np.int64), n)
        ys = np.mod(np.round(y0 + t * np.sin(a)).astype(np.int64), n)
        acc[ys, xs] = 1.0
    return np.clip(_blur(acc, sigma) * np.sqrt(2.0 * np.pi) * sigma, 0.0, 1.0)


def normal_map(height, pixel_m: float):
    """An OpenGL (+Y up) tangent-space normal map, uint8 RGB, of a height field in metres whose pixels are ``pixel_m``
    across; its slopes wrap at the edges, as the field does."""
    import numpy as np

    du = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) / (2.0 * pixel_m)  # u runs right
    dv = (np.roll(height, 1, axis=0) - np.roll(height, -1, axis=0)) / (2.0 * pixel_m)  # v runs up; rows run down
    k = 1.0 / np.sqrt(du * du + dv * dv + 1.0)
    n = np.stack([-du * k, -dv * k, k], axis=-1)
    return np.clip(np.round((n * 0.5 + 0.5) * 255.0), 0, 255).astype(np.uint8)


def _cutting_mat(n: int, size_m: float, rng):
    import numpy as np

    px = size_m / n
    u = (np.arange(n) + 0.5) * px

    def lines(pitch: float, width: float):
        d = np.abs((u / pitch + 0.5) % 1.0 - 0.5) * pitch  # to the nearest line, m
        return np.clip((width / 2.0 - d) / px + 0.5, 0.0, 1.0)  # each pixel's share of it

    thin, bold = lines(0.01, 0.0004), lines(0.05, 0.001)
    thin2 = np.maximum(thin[None, :], thin[:, None])
    bold2 = np.maximum(bold[None, :], bold[:, None])
    mottle = 0.03 * periodic_noise(n, rng, 48.0) + 0.02 * periodic_noise(n, rng, 2.0)
    rgb = np.array([0.15, 0.40, 0.27])[None, None, :] * (1.0 + mottle[..., None])
    for cover, colour in ((thin2, (0.62, 0.78, 0.66)), (bold2, (0.84, 0.90, 0.82))):
        rgb = rgb * (1.0 - cover[..., None]) + np.array(colour)[None, None, :] * cover[..., None]
    cuts = _strokes(n, rng, 60, 0.02 / px, 0.15 / px, 0.7)
    rgb = rgb * (1.0 - 0.35 * cuts[..., None]) + 0.35 * cuts[..., None] * np.array([0.55, 0.68, 0.58])[None, None, :]
    rough = 0.62 + 0.04 * periodic_noise(n, rng, 1.5) - 0.10 * np.maximum(thin2, bold2) + 0.10 * cuts
    height = 12e-6 * periodic_noise(n, rng, 1.2) - 40e-6 * cuts
    return rgb, rough, height


def _kraft_paper(n: int, size_m: float, rng):
    import numpy as np

    fibres, fine, mottle = periodic_noise(n, rng, 7.0, 0.9), periodic_noise(n, rng, 0.8), periodic_noise(n, rng, 70.0)
    specks = _specks(n, rng, 0.0006, 1.0)
    shade = 1.0 + 0.06 * mottle + 0.035 * fibres + 0.02 * fine - 0.35 * specks
    rgb = np.array([0.66, 0.50, 0.33])[None, None, :] * shade[..., None]
    rough = 0.80 + 0.04 * fibres + 0.02 * fine
    height = 6e-6 * fibres + 3e-6 * fine + 120e-6 * periodic_noise(n, rng, 90.0)
    return rgb, rough, height


def _cardboard(n: int, size_m: float, rng):
    import numpy as np

    u = (np.arange(n) + 0.5) * (size_m / n)
    flute = np.cos(2.0 * np.pi * u / 0.0045)[None, :] * np.ones((n, 1))  # the corrugation under the liner; 80 to a tile
    fibres, fine, mottle = periodic_noise(n, rng, 5.0, 0.9), periodic_noise(n, rng, 0.8), periodic_noise(n, rng, 60.0)
    specks = _specks(n, rng, 0.0004, 1.0)
    shade = 1.0 + 0.025 * flute + 0.05 * mottle + 0.03 * fibres + 0.015 * fine - 0.3 * specks
    rgb = np.array([0.62, 0.46, 0.30])[None, None, :] * shade[..., None]
    rough = 0.84 + 0.03 * fibres + 0.01 * fine
    height = 45e-6 * flute + 5e-6 * fibres
    return rgb, rough, height


def _white_paper(n: int, size_m: float, rng):
    import numpy as np

    fibres, fine, mottle = periodic_noise(n, rng, 2.5, 0.9), periodic_noise(n, rng, 0.7), periodic_noise(n, rng, 40.0)
    shade = 1.0 + 0.012 * mottle + 0.008 * fibres + 0.006 * fine
    rgb = np.array([0.93, 0.93, 0.915])[None, None, :] * shade[..., None]
    rough = 0.68 + 0.03 * fibres
    height = 3e-6 * fibres + 25e-6 * periodic_noise(n, rng, 110.0)
    return rgb, rough, height


_MAKERS = {"teleop_cutting_mat": _cutting_mat, "teleop_kraft_paper": _kraft_paper, "teleop_cardboard": _cardboard,
           "teleop_white_paper": _white_paper}


def procedural_maps(tid: str, n: int = PROCEDURAL_PX) -> Dict[str, bytes]:
    """The maps of texture ``tid`` made here, as the files they are stored as: ``diffuse.png`` (sRGB), ``roughness.png``
    (greyscale) and ``normal.png`` (OpenGL), each ``n`` pixels square. The same bytes every time with the same numpy."""
    import numpy as np

    size_m, seed, _ = PROCEDURAL[tid]
    rgb, rough, height = _MAKERS[tid](n, size_m, np.random.RandomState(seed))

    def srgb8(a):
        return np.clip(np.round(a * 255.0), 0, 255).astype(np.uint8)

    return {"diffuse.png": png(srgb8(rgb)), "roughness.png": png(srgb8(rough)), "normal.png": png(normal_map(height, size_m / n))}


# --- fetch, status -----------------------------------------------------------------------------------------------------

def _make(root: str, tid: str) -> Dict:
    size_m, seed, name = PROCEDURAL[tid]
    kept = {fname: _write(os.path.join(root, tid, fname), data) for fname, data in procedural_maps(tid).items()}
    return {"id": tid, "class": TEXTURES[tid], "files": kept, "sizeM": size_m, "name": name, "authors": [],
            "license": PROCEDURAL_LICENSE, "landingUrl": "", "generator": GENERATOR, "seed": seed,
            "source": f"made by teleop_isaac.materials from seed {seed}; nothing downloaded"}


def _fetch_one(root: str, tid: str, download) -> Dict:
    info = _api(f"/info/{tid}", download)
    files = _api(f"/files/{tid}", download)
    kept = {}
    for key, fmt, name in MAPS:
        spec = files[key][RESOLUTION][fmt]
        url, size, md5 = str(spec["url"]), int(spec["size"]), str(spec["md5"])
        if not url_ok(url) or not 0 < size <= MAX_BYTES or not re.fullmatch(r"[0-9a-fA-F]{32}", md5):
            raise ValueError(f"{key}: its file list is not what was expected")
        data = download(url, MAX_BYTES)
        if len(data) != size or hashlib.md5(data).hexdigest() != md5.lower() or not map_ok(name, data):
            raise ValueError(f"{key}: refused (size, md5 or content)")
        kept[name] = _write(os.path.join(root, tid, name), data)
    dims = info.get("dimensions") or [1000, 1000]
    size_m = round(float(max(dims[0], dims[1])) / 1000.0, 4)
    return {"id": tid, "class": TEXTURES[tid], "files": kept, "sizeM": size_m, "name": str(info.get("name", ""))[:200],
            "authors": sorted(str(a)[:200] for a in (info.get("authors") or {})),
            "license": LICENSE, "landingUrl": f"https://polyhaven.com/a/{tid}"}


def fetch(root: str, ids: Sequence[str] = tuple(TEXTURES), download=_download, say=print) -> Dict:
    """Every texture of ``ids`` not in ``root``'s manifest yet: Poly Haven's downloaded and checked, ours made (again, when
    its entry names another :data:`GENERATOR`). One texture's failure is logged and the rest go on."""
    manifest = load_manifest(root)
    have = {e["id"]: e for e in manifest["textures"]}
    for tid in ids:
        if not ASSET_ID.match(tid) or tid not in TEXTURES:
            raise ValueError(f"not one of the textures: {tid!r}")
        made = tid in PROCEDURAL
        if tid in have and (not made or have[tid].get("generator") == GENERATOR):
            continue
        try:
            entry = _make(root, tid) if made else _fetch_one(root, tid, download)
        except Exception as e:  # noqa: BLE001 -- one texture's failure is logged and the rest go on
            say(f"[materials] {tid}: failed ({e})")
            continue
        manifest["textures"] = sorted([e for e in manifest["textures"] if e["id"] != tid] + [entry], key=lambda x: x["id"])
        have[tid] = entry
        save_manifest(root, manifest)
        say(f"[materials] {tid} ({TEXTURES[tid]}{', made here' if made else ''}): {entry['sizeM']} m a tile, {len(entry['files'])} maps")
    return manifest


def status(root: str) -> Dict:
    m = load_manifest(root)
    bad = []
    for e in m["textures"]:
        for name, sha in e["files"].items():
            path = os.path.join(root, e["id"], name)
            try:
                with open(path, "rb") as f:
                    if hashlib.sha256(f.read()).hexdigest() != sha:
                        bad.append(path)
            except OSError:
                bad.append(path)
    classes: Dict[str, int] = {}
    for e in m["textures"]:
        classes[e["class"]] = classes.get(e["class"], 0) + 1
    missing = sorted(set(TEXTURES) - {e["id"] for e in m["textures"]})
    return {"root": root, "textures": len(m["textures"]), "classes": classes, "missing": missing, "bad": bad}


# --- the draws ---------------------------------------------------------------------------------------------------------

def pick(textures: Sequence[Dict], classes: Sequence[str], seed: int, change: int, object_id: int) -> Optional[Dict]:
    """The material object ``object_id`` wears from change ``change``: one of ``textures`` whose class is in ``classes``,
    drawn from the seed, the change and the id alone (None when there is none of those classes)."""
    pool = [t for t in textures if t["class"] in classes]
    if not pool:
        return None
    return random.Random(f"teleop-material:{seed}:{change}:{object_id}").choice(sorted(pool, key=lambda t: t["id"]))


def pick_look(looks: Mapping[str, Mapping], ids: Sequence[str], key: str) -> str:
    """One of the look ids ``ids`` (in the scene file's order), drawn by each look's ``weight`` (1 when it has none) from
    ``random.Random(key)`` alone."""
    if not ids:
        raise ValueError("no looks to draw from")
    weights = [float(looks[i].get("weight", 1.0)) for i in ids]
    u = random.Random(key).random() * sum(weights)
    for lid, w in zip(ids, weights):
        if u < w:
            return lid
        u -= w
    return ids[-1]


def surface_look(looks: Mapping[str, Mapping], ids: Sequence[str], seed: int, change: int, surface: str) -> str:
    """The look surface ``surface`` wears from change ``change`` (each surface its own draw)."""
    return pick_look(looks, ids, f"teleop-surface:{seed}:{change}:{surface}")


def object_look(looks: Mapping[str, Mapping], ids: Sequence[str], seed: int, change: int, name: str) -> str:
    """The look the object called ``name`` wears from change ``change``, for an object the scene gives looks."""
    return pick_look(looks, ids, f"teleop-object-look:{seed}:{change}:{name}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PBR textures for the twin's shapes and surfaces (docs/adr/0037 amendment 5)")
    ap.add_argument("verb", choices=("fetch", "status"))
    ap.add_argument("--root", default=DEFAULT_ROOT)
    args = ap.parse_args(argv)
    if args.verb == "fetch":
        m = fetch(args.root)
        print(json.dumps(status(args.root)))
        return 0 if {e["id"] for e in m["textures"]} >= set(TEXTURES) else 1
    s = status(args.root)
    print(json.dumps(s))
    return 0 if s["textures"] and not s["bad"] else 1


if __name__ == "__main__":
    sys.exit(main())
