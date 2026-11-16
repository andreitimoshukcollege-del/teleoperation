"""Real fixtures for the twin's table (docs/adr/0037 amendment 5): Poly Haven's CC0 3D models, as USD with their textures.

Run on the GPU machine with Isaac's Python (``just isaac-models-survey``, ``just isaac-models-fetch``, ``just isaac-models-status``):

    python -m teleop_isaac.models survey containers electronics appliances plants decorative
    python -m teleop_isaac.models fetch <id> <id> ...
    python -m teleop_isaac.models status

**Downloads are untrusted data.** The asset list and each asset's file list come from ``api.polyhaven.com``.
- **USD and textures** are fetched only from ``https://dl.polyhaven.org/``. A file is kept only if it matches the size and
  md5 the API gave and is at most :data:`MAX_BYTES`.
- **The USD** must start like a USD crate or text file.
- **Each texture** must lie under ``textures/``, end in ``.jpg``, ``.png`` or ``.exr``, and start like one.
- **On disk:** ``<root>/<id>/<id>.usd`` with its textures at the paths it names. The manifest records every file's sha256.

``survey`` fetches only each candidate's USD (no textures) into a scratch folder and prints its measured size standing,
so fixtures can be picked by size. ``manifest.json`` has the same entries as ``ycb.py``'s (how it stands, its size and box
centre standing), so ``world.mesh_asset`` reads either library.
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
from typing import Callable, Dict, List, Sequence, Tuple

API = "https://api.polyhaven.com"
DOWNLOAD_HOST = "dl.polyhaven.org"
CC0 = "https://creativecommons.org/publicdomain/zero/1.0/"
LICENSE = "CC0 1.0 (Poly Haven, polyhaven.com)"
USER_AGENT = "teleop-research-renderer/1.0"
RESOLUTION = "1k"
MAX_BYTES = 60 * 1024 * 1024
MAX_API_BYTES = 8 * 1024 * 1024
DEFAULT_ROOT = os.path.expanduser("~/isaacsim/assets/polyhaven")
ASSET_ID = re.compile(r"^[A-Za-z0-9_]{1,80}$")
_TEXTURE = re.compile(r"^textures/[A-Za-z0-9_.-]{1,160}\.(jpg|png|exr)$")
_HALF = 0.7071067811865476
UPRIGHT = {"z": (1.0, 0.0, 0.0, 0.0), "y": (_HALF, _HALF, 0.0, 0.0)}  # y up: +90 degrees about x


def stand(up: str, v: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z = v
    return (x, y, z) if up == "z" else (x, -z, y)


def file_ok(rel: str, data: bytes) -> bool:
    """A model's file: its USD, or a texture under ``textures/`` that starts like what its name says."""
    if len(data) > MAX_BYTES:
        return False
    if rel.endswith(".usd") or rel.endswith(".usdc") or rel.endswith(".usda"):
        return data.startswith(b"PXR-USDC") or data.startswith(b"#usda ")
    if not _TEXTURE.match(rel) or ".." in rel.split("/"):
        return False
    if rel.endswith(".png"):
        return data.startswith(b"\x89PNG\r\n\x1a\n")
    if rel.endswith(".jpg"):
        return data.startswith(b"\xff\xd8\xff")
    return data.startswith(b"\x76\x2f\x31\x01")  # OpenEXR


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


def usd_files(files: Dict) -> Tuple[Dict, Dict[str, Dict]]:
    """(the USD's own spec, {relative path: spec} of what it includes) from a ``/files/<id>`` answer, at :data:`RESOLUTION`."""
    spec = files["usd"][RESOLUTION]["usd"]
    return spec, dict(spec.get("include") or {})


def _inspect(usd_path: str) -> Tuple[str, Tuple[float, ...], Tuple[float, ...]]:
    """(up axis, bbox min, bbox max) of a USD in metres. Needs Isaac's pxr."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(usd_path)
    up = "z" if UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z else "y"
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(root).ComputeAlignedRange()
    return up, tuple(float(v) * mpu for v in box.GetMin()), tuple(float(v) * mpu for v in box.GetMax())


def entry(asset_id: str, info: Dict, files: Dict[str, str], up: str, lo: Sequence[float], hi: Sequence[float]) -> Dict:
    size = [round(abs(v), 5) for v in stand(up, [h - l for l, h in zip(lo, hi)])]
    centre = [round(v, 5) for v in stand(up, [(h + l) / 2 for l, h in zip(lo, hi)])]
    return {"name": asset_id, "usd": f"{asset_id}/{asset_id}.usd", "files": files, "up": up, "rotation": list(UPRIGHT[up]),
            "size": size, "centre": centre, "title": str(info.get("name", ""))[:200],
            "authors": sorted(str(a)[:200] for a in (info.get("authors") or {})),
            "categories": [str(c)[:80] for c in (info.get("categories") or [])][:20], "license": LICENSE,
            "landingUrl": f"https://polyhaven.com/a/{asset_id}"}


def load_manifest(root: str) -> Dict:
    path = os.path.join(root, "manifest.json")
    if not os.path.exists(path):
        return {"format": "teleop-polyhaven-models", "version": 1, "license": LICENSE, "objects": []}
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


def _get(spec: Dict, rel: str, download) -> bytes:
    url, size, md5 = str(spec["url"]), int(spec["size"]), str(spec["md5"])
    if not url_ok(url) or not 0 < size <= MAX_BYTES or not re.fullmatch(r"[0-9a-fA-F]{32}", md5):
        raise ValueError(f"{rel}: its file list is not what was expected")
    data = download(url, MAX_BYTES)
    if len(data) != size or hashlib.md5(data).hexdigest() != md5.lower() or not file_ok(rel, data):
        raise ValueError(f"{rel}: refused (size, md5 or content)")
    return data


def survey(categories: Sequence[str], scratch: str, download=_download, inspect=_inspect, say=print) -> List[Dict]:
    """Each model of ``categories``: its id and measured size standing (m), from its USD alone (its textures are not fetched)."""
    seen, out = set(), []
    for c in categories:
        if not re.fullmatch(r"[a-z ]{1,40}", c):
            raise ValueError(f"not a category: {c!r}")
        assets = _api("/assets?t=models&c=" + urllib.parse.quote(c), download)
        for asset_id in sorted(assets):
            if asset_id in seen or not ASSET_ID.match(asset_id):
                continue
            seen.add(asset_id)
            try:
                spec, _ = usd_files(_api(f"/files/{asset_id}", download))
                path = os.path.join(scratch, f"{asset_id}.usd")
                _write(path, _get(spec, f"{asset_id}.usd", download))
                up, lo, hi = inspect(path)
            except Exception as e:  # noqa: BLE001 -- one model's failure is logged and the rest go on
                say(f"[models] {asset_id}: skipped ({e})")
                continue
            size = [round(abs(v), 3) for v in stand(up, [h - l for l, h in zip(lo, hi)])]
            out.append({"id": asset_id, "category": c, "up": up, "size": size})
            say(f"[models] {asset_id} ({c}): {size[0] * 100:.0f} x {size[1] * 100:.0f} x {size[2] * 100:.0f} cm standing ({up} up)")
    return out


def fetch(root: str, ids: Sequence[str], download=_download, inspect=_inspect, say=print) -> Dict:
    manifest = load_manifest(root)
    have = {e["name"] for e in manifest["objects"]}
    for asset_id in ids:
        if not ASSET_ID.match(asset_id):
            raise ValueError(f"not a Poly Haven id: {asset_id!r}")
        if asset_id in have:
            continue
        try:
            info = _api(f"/info/{asset_id}", download)
            spec, include = usd_files(_api(f"/files/{asset_id}", download))
            files = {f"{asset_id}.usd": _write(os.path.join(root, asset_id, f"{asset_id}.usd"), _get(spec, f"{asset_id}.usd", download))}
            for rel, s in sorted(include.items()):
                if not _TEXTURE.match(rel):
                    raise ValueError(f"it includes {rel!r}, not a texture under textures/")
                files[rel] = _write(os.path.join(root, asset_id, rel), _get(s, rel, download))
            up, lo, hi = inspect(os.path.join(root, asset_id, f"{asset_id}.usd"))
        except Exception as e:  # noqa: BLE001
            say(f"[models] {asset_id}: failed ({e})")
            continue
        manifest["objects"].append(entry(asset_id, info, files, up, lo, hi))
        manifest["objects"].sort(key=lambda x: x["name"])
        save_manifest(root, manifest)
        e = next(x for x in manifest["objects"] if x["name"] == asset_id)
        say(f"[models] {asset_id}: {', '.join(f'{v * 100:.0f}' for v in e['size'])} cm standing, {len(files)} file(s)")
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
            "sizesCm": {e["name"]: [round(v * 100) for v in e["size"]] for e in m["objects"]}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Poly Haven models for the twin's table (docs/adr/0037 amendment 5)")
    ap.add_argument("verb", choices=("survey", "fetch", "status"))
    ap.add_argument("names", nargs="*", help="survey: categories; fetch: model ids")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    args = ap.parse_args(argv)
    if args.verb == "survey":
        scratch = tempfile.mkdtemp(prefix="models-survey-")
        rows = survey(args.names or ["containers"], scratch)
        print(json.dumps(rows))
        return 0 if rows else 1
    if args.verb == "fetch":
        m = fetch(args.root, args.names)
        print(json.dumps(status(args.root)))
        return 0 if all(any(e["name"] == n for e in m["objects"]) for n in args.names) else 1
    s = status(args.root)
    print(json.dumps(s))
    return 0 if s["objects"] and not s["bad"] else 1


if __name__ == "__main__":
    sys.exit(main())
