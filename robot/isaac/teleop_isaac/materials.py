"""Real materials for the twin's simple shapes and surfaces (docs/adr/0037 amendment 5): Poly Haven's CC0 PBR textures, so a
box is a wooden block and a cylinder a metal or wooden can, not a flat colour.

Run on the GPU machine (``just isaac-materials-fetch``, ``just isaac-materials-status``):

    python -m teleop_isaac.materials fetch
    python -m teleop_isaac.materials status

**Downloads are untrusted data.**
- **The source.** Each texture's file list comes from ``api.polyhaven.com``; its maps (colour, roughness, OpenGL normal,
  1K) are fetched only from ``https://dl.polyhaven.org/``.
- **What is kept.** A map is kept only if it matches the size and md5 the API gave, is at most :data:`MAX_BYTES`, and starts
  like the JPEG or PNG its name says. It is stored as ``<root>/<id>/<map>.<ext>``, with its sha256 in the manifest.

:data:`TEXTURES` names each texture and its class (``wood`` or ``metal``); the manifest records its real-world size from
Poly Haven (``sizeM``), so the renderer tiles it at its true scale. :func:`pick` chooses an object's material for a change,
from the seed, the change and the object's id alone.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import re
import sys
import tempfile
import urllib.parse
import urllib.request
from typing import Dict, List, Optional, Sequence

API = "https://api.polyhaven.com"
DOWNLOAD_HOST = "dl.polyhaven.org"
LICENSE = "CC0 1.0 (Poly Haven, polyhaven.com)"
USER_AGENT = "teleop-research-renderer/1.0"
RESOLUTION = "1k"
MAX_BYTES = 30 * 1024 * 1024
MAX_API_BYTES = 8 * 1024 * 1024
DEFAULT_ROOT = os.path.expanduser("~/isaacsim/assets/textures/polyhaven")
ASSET_ID = re.compile(r"^[a-z0-9_]{1,80}$")

# The textures and what they make: wooden blocks and planks, metal cans. grey_oak_veneer_01 is also the table's laminate.
TEXTURES: Dict[str, str] = {
    "american_walnut_veneer": "wood", "ash_veneer": "wood", "bamboo_veneer": "wood", "black_oak_veneer": "wood",
    "cherry_veneer": "wood", "coated_pine": "wood", "fine_grained_wood": "wood", "oak_veneer_01": "wood",
    "mocha_oak_veneer": "wood", "natural_walnut_veneer": "wood", "blue_painted_planks": "wood",
    "distressed_painted_planks": "wood", "grey_oak_veneer_01": "wood",
    "metal_plate": "metal", "metal_plate_02": "metal", "blue_metal_plate": "metal", "green_metal_rust": "metal",
    "rusty_metal": "metal", "rusty_painted_metal": "metal", "rusty_metal_02": "metal",
}

# Each map: the files API's key, the format fetched, and the name it is stored under.
MAPS = (("Diffuse", "jpg", "diffuse.jpg"), ("Rough", "jpg", "roughness.jpg"), ("nor_gl", "png", "normal.png"))


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


def fetch(root: str, ids: Sequence[str] = tuple(TEXTURES), download=_download, say=print) -> Dict:
    manifest = load_manifest(root)
    have = {e["id"] for e in manifest["textures"]}
    for tid in ids:
        if not ASSET_ID.match(tid) or tid not in TEXTURES:
            raise ValueError(f"not one of the textures: {tid!r}")
        if tid in have:
            continue
        try:
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
        except Exception as e:  # noqa: BLE001 -- one texture's failure is logged and the rest go on
            say(f"[materials] {tid}: failed ({e})")
            continue
        dims = info.get("dimensions") or [1000, 1000]
        size_m = round(float(max(dims[0], dims[1])) / 1000.0, 4)
        manifest["textures"].append({"id": tid, "class": TEXTURES[tid], "files": kept, "sizeM": size_m,
                                     "name": str(info.get("name", ""))[:200],
                                     "authors": sorted(str(a)[:200] for a in (info.get("authors") or {})),
                                     "license": LICENSE, "landingUrl": f"https://polyhaven.com/a/{tid}"})
        manifest["textures"].sort(key=lambda x: x["id"])
        save_manifest(root, manifest)
        say(f"[materials] {tid} ({TEXTURES[tid]}): {size_m} m a tile, {len(kept)} maps")
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
    return {"root": root, "textures": len(m["textures"]), "classes": classes, "bad": bad}


def pick(textures: Sequence[Dict], classes: Sequence[str], seed: int, change: int, object_id: int) -> Optional[Dict]:
    """The material object ``object_id`` wears from change ``change``: one of ``textures`` whose class is in ``classes``,
    drawn from the seed, the change and the id alone (None when there is none of those classes)."""
    pool = [t for t in textures if t["class"] in classes]
    if not pool:
        return None
    return random.Random(f"teleop-material:{seed}:{change}:{object_id}").choice(sorted(pool, key=lambda t: t["id"]))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PBR textures for the twin's shapes and surfaces (docs/adr/0037 amendment 5)")
    ap.add_argument("verb", choices=("fetch", "status"))
    ap.add_argument("--root", default=DEFAULT_ROOT)
    args = ap.parse_args(argv)
    if args.verb == "fetch":
        m = fetch(args.root)
        print(json.dumps(status(args.root)))
        return 0 if len(m["textures"]) == len(TEXTURES) else 1
    s = status(args.root)
    print(json.dumps(s))
    return 0 if s["textures"] and not s["bad"] else 1


if __name__ == "__main__":
    sys.exit(main())
