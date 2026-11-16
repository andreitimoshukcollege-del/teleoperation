"""Real objects for the twin's table (docs/adr/0037 decision 4, P3): the YCB object set, as NVIDIA's Isaac assets convert it
(``Isaac/Props/YCB/Axis_Aligned/``), with each object's measured mass from the YCB benchmark (Calli et al., 2015).

Run on the GPU machine with Isaac's Python (``just isaac-ycb-fetch``, ``just isaac-ycb-status``):

    python -m teleop_isaac.ycb fetch
    python -m teleop_isaac.ycb status

**Downloads are untrusted data.**
- **The source.** Each object's USD comes from :data:`ASSETS` only, by a name in :data:`OBJECTS`. It is kept only if it is
  at most :data:`MAX_BYTES` and starts like a USD crate file.
- **Textures.** The files a USD names that are not on disk are fetched beside it only if they lie under ``Materials/``,
  end in ``.png`` or ``.jpg``, and start like one.
- **On disk:** ``<root>/<name>/<name>.usd`` with its textures at the paths it names, so it opens as it was written. The
  manifest records every file's sha256.

``manifest.json`` lists each object with what the scene file needs: how it stands (the assets are axis-aligned, but some
stand along their y and some along their z: :data:`OBJECTS` names each one's up, its most stable resting side), the rotation
that stands it so, its axis-aligned size in metres standing, the offset of that box's centre from the asset's origin, its
mass, its material and the friction that goes with it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import tempfile
import urllib.parse
import urllib.request
from typing import Callable, Dict, List, Optional, Sequence, Tuple

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/YCB/Axis_Aligned/"
LICENSE = "YCB object set: CC BY 4.0 (ycbbenchmarks.com); USD conversion: NVIDIA Isaac Sim assets"
USER_AGENT = "teleop-research-renderer/1.0"
MAX_BYTES = 40 * 1024 * 1024
DEFAULT_ROOT = os.path.expanduser("~/isaacsim/assets/ycb")

# The 21 objects NVIDIA converts: each one's YCB mass (kg), what its surface is, and which of its asset's axes points up when
# it rests as it is used (a can on its base, a box on its largest face, a mug upright; the marker on its end): "y" or "z".
OBJECTS: Dict[str, Tuple[float, str, str]] = {
    "002_master_chef_can": (0.414, "metal", "y"), "003_cracker_box": (0.411, "cardboard", "z"), "004_sugar_box": (0.514, "cardboard", "z"),
    "005_tomato_soup_can": (0.349, "metal", "y"), "006_mustard_bottle": (0.603, "plastic", "z"), "007_tuna_fish_can": (0.171, "metal", "y"),
    "008_pudding_box": (0.187, "cardboard", "z"), "009_gelatin_box": (0.097, "cardboard", "z"), "010_potted_meat_can": (0.370, "metal", "y"),
    "011_banana": (0.066, "fruit", "y"), "019_pitcher_base": (0.178, "plastic", "y"), "021_bleach_cleanser": (1.131, "plastic", "z"),
    "024_bowl": (0.147, "plastic", "y"), "025_mug": (0.118, "ceramic", "y"), "035_power_drill": (0.895, "plastic", "z"),
    "036_wood_block": (0.729, "wood", "z"), "037_scissors": (0.082, "metal", "z"), "040_large_marker": (0.0158, "plastic", "y"),
    "051_large_clamp": (0.125, "plastic", "z"), "052_extra_large_clamp": (0.202, "plastic", "z"), "061_foam_brick": (0.028, "foam", "z"),
}

_HALF = 0.7071067811865476
# The rotation that stands an asset on its up axis (w, x, y, z): y up is +90 degrees about x (y to z, z to -y).
UPRIGHT = {"z": (1.0, 0.0, 0.0, 0.0), "y": (_HALF, _HALF, 0.0, 0.0)}


def stand(up: str, v: Sequence[float]) -> Tuple[float, float, float]:
    """A vector of the asset's frame in the standing frame (:data:`UPRIGHT`)."""
    x, y, z = v
    return (x, y, z) if up == "z" else (x, -z, y)

# (static, dynamic) friction by surface, against the table's 0.6 / 0.5: typical dry values, not measured.
FRICTION: Dict[str, Tuple[float, float]] = {
    "metal": (0.35, 0.3), "cardboard": (0.5, 0.4), "plastic": (0.4, 0.35), "ceramic": (0.5, 0.4), "fruit": (0.6, 0.5),
    "wood": (0.5, 0.4), "foam": (0.9, 0.8),
}

_NAME = re.compile(r"^[0-9]{3}_[a-z_]{1,40}$")
_TEXTURE = re.compile(r"^Materials/[A-Za-z0-9_./-]{1,200}\.(png|jpg)$")


def usd_ok(data: bytes) -> bool:
    return len(data) <= MAX_BYTES and (data.startswith(b"PXR-USDC") or data.startswith(b"#usda "))


def texture_ok(path: str, data: bytes) -> bool:
    if not _TEXTURE.match(path) or ".." in path.split("/") or len(data) > MAX_BYTES:
        return False
    return data.startswith(b"\x89PNG\r\n\x1a\n") if path.endswith(".png") else data.startswith(b"\xff\xd8\xff")


def entry(name: str, files: Dict[str, str], lo: Sequence[float], hi: Sequence[float]) -> Dict:
    """The manifest's record of one object: ``lo``/``hi`` are its axis-aligned bounds in the asset's frame (metres)."""
    mass, material, up = OBJECTS[name]
    raw = [h - l for l, h in zip(lo, hi)]
    size = [round(abs(v), 5) for v in stand(up, raw)]
    centre = [round(v, 5) for v in stand(up, [(h + l) / 2 for l, h in zip(lo, hi)])]
    static, dynamic = FRICTION[material]
    return {"name": name, "usd": f"{name}/{name}.usd", "files": files, "up": up, "rotation": list(UPRIGHT[up]), "size": size,
            "centre": centre, "assetSize": [round(v, 5) for v in raw], "mass": mass, "material": material,
            "staticFriction": static, "dynamicFriction": dynamic, "license": LICENSE, "url": ASSETS + f"{name}.usd"}


def scene_object(e: Dict, object_id: int, x: float, y: float, table_top: float, colour: Sequence[float] = (0.7, 0.7, 0.7)) -> Dict:
    """A scene file entry for manifest entry ``e``: shape ``mesh``, its box as its size, resting on the table."""
    sx, sy, sz = e["size"]
    return {"id": object_id, "name": e["name"].split("_", 1)[1].replace("_", "-"), "shape": "mesh", "asset": f"ycb/{e['name']}",
            "size": [sx, sy, sz], "position": [x, y, round(table_top + sz / 2, 4)], "rpy": [0, 0, 0], "kind": "movable",
            "mass": e["mass"], "operatorMovable": True, "staticFriction": e["staticFriction"],
            "dynamicFriction": e["dynamicFriction"], "colour": list(colour), "jitter": [0, 0, 0]}


def load_manifest(root: str) -> Dict:
    path = os.path.join(root, "manifest.json")
    if not os.path.exists(path):
        return {"format": "teleop-ycb", "version": 1, "license": LICENSE, "objects": []}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_manifest(root: str, manifest: Dict) -> None:
    path = os.path.join(root, "manifest.json")
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1, sort_keys=True)
        f.write("\n")
    os.replace(path + ".tmp", path)


def _download(url: str, limit: int = MAX_BYTES) -> bytes:
    u = urllib.parse.urlsplit(url)
    if u.scheme != "https" or not url.startswith(ASSETS):
        raise ValueError(f"not an Isaac YCB asset URL: {url}")
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read(limit + 1)
    if len(data) > limit:
        raise ValueError(f"{url} is over {limit} bytes")
    return data


def _write(path: str, data: bytes) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix=".part")
    with os.fdopen(fd, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    return hashlib.sha256(data).hexdigest()


def _inspect(usd_path: str) -> Tuple[List[str], Tuple[float, ...], Tuple[float, ...]]:
    """(the relative files it names that are missing, bbox min, bbox max) of a USD, in metres, Z up. Needs Isaac's pxr."""
    from pxr import Usd, UsdGeom, UsdUtils

    _, _, unresolved = UsdUtils.ComputeAllDependencies(usd_path)
    stage = Usd.Stage.Open(usd_path)
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise ValueError("not Z up")
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    lo, hi = tuple(float(v) * mpu for v in box.GetMin()), tuple(float(v) * mpu for v in box.GetMax())
    return [str(u) for u in unresolved if not str(u).endswith(".mdl")], lo, hi


def fetch(root: str, names: Sequence[str] = tuple(OBJECTS), download: Callable[[str], bytes] = _download,
          inspect: Callable[[str], Tuple[List[str], Tuple[float, ...], Tuple[float, ...]]] = _inspect, say=print) -> Dict:
    manifest = load_manifest(root)
    have = {e["name"] for e in manifest["objects"]}
    for name in names:
        if not _NAME.match(name) or name not in OBJECTS:
            raise ValueError(f"not one of the YCB objects: {name!r}")
        if name in have:
            continue
        try:
            data = download(ASSETS + f"{name}.usd")
            if not usd_ok(data):
                say(f"[ycb] {name}: refused (not a USD file)")
                continue
            usd = os.path.join(root, name, f"{name}.usd")
            files = {f"{name}.usd": _write(usd, data)}
            missing, lo, hi = inspect(usd)
            refused = False
            for rel in missing:
                tex = download(ASSETS + rel) if _TEXTURE.match(rel) else b""
                if not texture_ok(rel, tex):
                    say(f"[ycb] {name}: refused its file {rel!r}")
                    refused = True
                    break
                files[rel] = _write(os.path.join(root, name, rel), tex)
            if refused:
                continue
            if missing:
                missing, lo, hi = inspect(usd)  # with its textures in place now
            if missing:
                say(f"[ycb] {name}: still missing {missing}")
                continue
        except Exception as e:  # noqa: BLE001 -- one object's failure is logged and the rest go on
            say(f"[ycb] {name}: failed ({e})")
            continue
        manifest["objects"].append(entry(name, files, lo, hi))
        manifest["objects"].sort(key=lambda x: x["name"])
        save_manifest(root, manifest)
        e = manifest["objects"][-1] if manifest["objects"][-1]["name"] == name else next(x for x in manifest["objects"] if x["name"] == name)
        say(f"[ycb] {name}: {', '.join(f'{v * 100:.1f}' for v in e['size'])} cm, {e['mass'] * 1000:.0f} g, {len(files)} file(s)")
    return manifest


def status(root: str) -> Dict:
    m = load_manifest(root)
    bad = []
    for e in m["objects"]:
        for rel, sha in e["files"].items():
            path = os.path.join(root, e["name"], rel)
            try:
                with open(path, "rb") as f:
                    if hashlib.sha256(f.read()).hexdigest() != sha:
                        bad.append(path)
            except OSError:
                bad.append(path)
    return {"root": root, "objects": len(m["objects"]), "bad": bad,
            "sizesCm": {e["name"]: [round(v * 100, 1) for v in e["size"]] for e in m["objects"]}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YCB objects for the twin's table (docs/adr/0037 decision 4)")
    ap.add_argument("verb", choices=("fetch", "status"))
    ap.add_argument("--root", default=DEFAULT_ROOT)
    args = ap.parse_args(argv)
    if args.verb == "fetch":
        m = fetch(args.root)
        print(json.dumps(status(args.root)))
        return 0 if len(m["objects"]) == len(OBJECTS) else 1
    s = status(args.root)
    print(json.dumps(s))
    return 0 if s["objects"] and not s["bad"] else 1


if __name__ == "__main__":
    sys.exit(main())
