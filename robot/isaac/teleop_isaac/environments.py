"""The rooms around the JetRover twin's table (docs/adr/0037 decision 2): Poly Haven's indoor HDRIs, CC0, each a 360°
photograph of a real room that the renderer's dome light shows beyond the table and lights the scene with.

Run on the GPU machine (``just isaac-environments-fetch``, ``just isaac-environments-status``):

    python -m teleop_isaac.environments fetch
    python -m teleop_isaac.environments status

**Downloads are untrusted data.** The asset list and each asset's file list come from ``api.polyhaven.com``. An HDRI is
fetched only from ``https://dl.polyhaven.org/`` into a temporary file. It is kept only if:

- it is at most :data:`MAX_BYTES`;
- it matches the size and md5 the API gave;
- it is a Radiance RGBE file that :func:`read_rgbe` decodes to a 2:1 panorama.

It is stored as ``hdri/<sha256>.hdr``, so no name on disk comes from anything the network sent. ``manifest.json`` lists
every kept room: its id, name, sha256, size, authors, categories and page, and its **mean luminance**
(:func:`mean_luminance`). The renderer divides the dome's brightness by that, so a dim room and a sunlit one light the
table alike, as a camera's auto-exposure would.

:func:`pick` chooses the room of a change (0 at start-up, then one per closed session) and its turn about the vertical, from
a seed and the change's number alone: every room once before any comes back.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import re
import sys
import tempfile
import time
import urllib.parse
import urllib.request
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

API = "https://api.polyhaven.com"
DOWNLOAD_HOST = "dl.polyhaven.org"
CC0 = "https://creativecommons.org/publicdomain/zero/1.0/"
SOURCE = "Poly Haven (polyhaven.com), HDRIs in the indoor category"
USER_AGENT = "teleop-research-renderer/1.0"
RESOLUTION = "2k"
MAX_BYTES = 40 * 1024 * 1024  # a 2K Radiance file is 4-9 MB
MAX_API_BYTES = 8 * 1024 * 1024
DEFAULT_ROOT = os.path.expanduser("~/isaacsim/assets/hdri/polyhaven-indoor")
ASSET_ID = re.compile(r"^[a-z0-9_]{1,80}$")


# --- Radiance RGBE (.hdr) ---------------------------------------------------------------------------------------------

def read_rgbe(data: bytes) -> np.ndarray:
    """A Radiance ``.hdr`` file's pixels as float32 RGB, (height, width, 3), top row first. Only the standard orientation
    (``-Y h +X w``), flat or new-style run-length scanlines. Raises ValueError on anything else."""
    if not (data.startswith(b"#?RADIANCE") or data.startswith(b"#?RGBE")):
        raise ValueError("not a Radiance file")
    at, fmt = 0, None
    while True:
        end = data.find(b"\n", at)
        if end < 0 or end - at > 4096:
            raise ValueError("the header does not end")
        line = data[at:end].strip()
        at = end + 1
        if not line:
            break
        if line.startswith(b"FORMAT="):
            fmt = line[7:]
    if fmt not in (None, b"32-bit_rle_rgbe"):
        raise ValueError(f"unsupported format {fmt!r}")
    end = data.find(b"\n", at)
    if end < 0:
        raise ValueError("no resolution line")
    m = re.fullmatch(rb"-Y (\d{1,5}) \+X (\d{1,5})", data[at:end].strip())
    if m is None:
        raise ValueError("unsupported orientation or resolution line")
    h, w = int(m.group(1)), int(m.group(2))
    if not (1 <= h <= 16384 and 1 <= w <= 32768):
        raise ValueError(f"unreasonable size {w}x{h}")
    at = end + 1
    buf = memoryview(data)
    out = np.empty((h, w, 4), np.uint8)
    for y in range(h):
        if at + 4 > len(data):
            raise ValueError("the pixels end early")
        if 8 <= w < 32768 and data[at] == 2 and data[at + 1] == 2 and (data[at + 2] << 8 | data[at + 3]) == w and data[at + 2] < 128:
            at += 4
            for c in range(4):
                x = 0
                row = out[y, :, c]
                while x < w:
                    if at >= len(data):
                        raise ValueError("a scanline ends early")
                    n = data[at]
                    at += 1
                    if n > 128:  # a run
                        n -= 128
                        if x + n > w or at >= len(data):
                            raise ValueError("a run overflows its scanline")
                        row[x:x + n] = data[at]
                        at += 1
                    else:  # literals
                        if n == 0 or x + n > w or at + n > len(data):
                            raise ValueError("a literal overflows its scanline")
                        row[x:x + n] = np.frombuffer(buf[at:at + n], np.uint8)
                        at += n
                    x += n
        else:  # flat: four bytes a pixel (old-style runs are not supported)
            if at + 4 * w > len(data):
                raise ValueError("a flat scanline ends early")
            out[y] = np.frombuffer(buf[at:at + 4 * w], np.uint8).reshape(w, 4)
            at += 4 * w
    e = out[..., 3].astype(np.int32)
    scale = np.where(e > 0, np.ldexp(1.0, e - (128 + 8)), 0.0).astype(np.float32)
    return (out[..., :3].astype(np.float32) + 0.5) * scale[..., None] * (e[..., None] > 0)


def write_rgbe_flat(rgb: np.ndarray) -> bytes:
    """A flat (unencoded) Radiance file of float RGB (height, width, 3): for tests, and for nothing else."""
    rgb = np.asarray(rgb, np.float64)
    h, w, _ = rgb.shape
    peak = rgb.max(axis=2)
    mant, exp = np.frexp(peak)
    ok = peak > 1e-32
    scale = np.where(ok, mant * 256.0 / np.where(ok, peak, 1.0), 0.0)
    px = np.zeros((h, w, 4), np.uint8)
    px[..., :3] = np.clip(rgb * scale[..., None], 0, 255).astype(np.uint8)
    px[..., 3] = np.where(ok, exp + 128, 0).astype(np.uint8)
    head = f"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y {h} +X {w}\n".encode("ascii")
    return head + px.tobytes()


def mean_luminance(rgb: np.ndarray) -> float:
    """The panorama's mean luminance over the sphere (Rec. 709 weights, each row weighted by its share of the sphere: an
    equirectangular row's solid angle goes with the cosine of its latitude)."""
    h = rgb.shape[0]
    y = rgb[..., 0] * 0.2126 + rgb[..., 1] * 0.7152 + rgb[..., 2] * 0.0722
    lat = (0.5 - (np.arange(h) + 0.5) / h) * math.pi
    wgt = np.cos(lat)
    return float((y.mean(axis=1) * wgt).sum() / wgt.sum())


def check_hdri(data: bytes, size: Optional[int] = None, md5: Optional[str] = None) -> Optional[Tuple[int, int, float]]:
    """(width, height, mean luminance) of an acceptable 2:1 panorama, or None."""
    if len(data) > MAX_BYTES:
        return None
    if size is not None and len(data) != size:
        return None
    if md5 is not None and hashlib.md5(data).hexdigest() != md5.lower():
        return None
    try:
        rgb = read_rgbe(data)
    except ValueError:
        return None
    h, w, _ = rgb.shape
    if w != 2 * h or h < 64 or not np.isfinite(rgb).all():
        return None
    lum = mean_luminance(rgb)
    if not (lum > 0 and math.isfinite(lum)):
        return None
    return w, h, lum


# --- the manifest -----------------------------------------------------------------------------------------------------

def download_url_ok(url: str) -> bool:
    u = urllib.parse.urlsplit(url)
    return u.scheme == "https" and u.hostname == DOWNLOAD_HOST and u.path.endswith(".hdr") and ".." not in u.path


def entry(asset_id: str, info: Dict, sha: str, size: int, wh_lum: Tuple[int, int, float], url: str) -> Dict:
    w, h, lum = wh_lum
    return {"id": asset_id, "file": f"hdri/{sha}.hdr", "sha256": sha, "bytes": size, "width": w, "height": h,
            "meanLuminance": round(lum, 6), "name": str(info.get("name", ""))[:200],
            "authors": sorted(str(a)[:200] for a in (info.get("authors") or {})),
            "categories": [str(c)[:80] for c in (info.get("categories") or [])][:20],
            "license": CC0, "source": SOURCE, "url": url, "landingUrl": f"https://polyhaven.com/a/{asset_id}"}


def load_manifest(root: str) -> Dict:
    path = os.path.join(root, "manifest.json")
    if not os.path.exists(path):
        return {"format": "teleop-environments", "version": 1, "source": SOURCE, "license": CC0, "resolution": RESOLUTION,
                "hdris": []}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_manifest(root: str, manifest: Dict) -> None:
    path = os.path.join(root, "manifest.json")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)


def _download(url: str, limit: int, timeout: float = 120.0) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        data = r.read(limit + 1)
    if len(data) > limit:
        raise RuntimeError(f"{url} is over {limit} bytes")
    return data


def _api(path: str, download: Callable[[str, int], bytes]) -> Dict:
    return json.loads(download(API + path, MAX_API_BYTES).decode("utf-8"))


def fetch(root: str, count: int = 1000, download: Callable[[str, int], bytes] = _download, say=print) -> Dict:
    """Keep up to ``count`` indoor HDRIs (all of them by default), resuming where an earlier fetch stopped."""
    os.makedirs(os.path.join(root, "hdri"), exist_ok=True)
    assets = _api("/assets?t=hdris&c=indoor", download)
    ids = sorted(a for a in assets if ASSET_ID.match(a))  # an id goes into a URL: only what a Poly Haven id can be
    manifest = load_manifest(root)
    have = {e["id"] for e in manifest["hdris"]}
    say(f"[environments] {len(ids)} indoor HDRIs listed; {len(have)} kept already; want {min(count, len(ids))}")
    refused = failed = 0
    started = time.monotonic()
    for asset_id in ids:
        if len(manifest["hdris"]) >= count:
            break
        if asset_id in have:
            continue
        try:
            files = _api(f"/files/{asset_id}", download)
            spec = files["hdri"][RESOLUTION]["hdr"]
            url, size, md5 = str(spec["url"]), int(spec["size"]), str(spec["md5"])
            if not download_url_ok(url) or not re.fullmatch(r"[0-9a-fA-F]{32}", md5) or not 0 < size <= MAX_BYTES:
                refused += 1
                say(f"[environments] {asset_id}: refused (its file list is not what was expected)")
                continue
            data = download(url, MAX_BYTES)
        except Exception as e:  # noqa: BLE001 -- one asset's failure is logged and the rest go on
            failed += 1
            say(f"[environments] {asset_id}: failed ({e})")
            continue
        got = check_hdri(data, size, md5)
        if got is None:
            refused += 1
            say(f"[environments] {asset_id}: refused (size, md5 or decode)")
            continue
        sha = hashlib.sha256(data).hexdigest()
        path = os.path.join(root, "hdri", f"{sha}.hdr")
        if not os.path.exists(path):
            fd, tmp = tempfile.mkstemp(dir=os.path.join(root, "hdri"), suffix=".part")
            with os.fdopen(fd, "wb") as f:
                f.write(data)
            os.replace(tmp, path)
        manifest["hdris"].append(entry(asset_id, assets[asset_id], sha, len(data), got, url))
        have.add(asset_id)
        save_manifest(root, manifest)
        say(f"[environments] {len(manifest['hdris'])} kept ({asset_id}, mean luminance {got[2]:.3f}), {refused} refused, "
            f"{failed} failed, {time.monotonic() - started:.0f} s")
    save_manifest(root, manifest)
    return manifest


def status(root: str, verify: bool = True) -> Dict:
    m = load_manifest(root)
    bad, total = [], 0
    for e in m["hdris"]:
        path = os.path.join(root, e["file"])
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            bad.append(e["file"])
            continue
        total += len(data)
        if verify and hashlib.sha256(data).hexdigest() != e["sha256"]:
            bad.append(e["file"])
    lums = sorted(e["meanLuminance"] for e in m["hdris"])
    return {"root": root, "hdris": len(m["hdris"]), "bytes": total, "licences": sorted({e["license"] for e in m["hdris"]}),
            "bad": bad, "meanLuminance": {"min": lums[0], "p50": lums[len(lums) // 2], "max": lums[-1]} if lums else None}


def _order(seed: int, round_no: int, n: int) -> List[int]:
    idx = list(range(n))
    random.Random(f"teleop-environment:{seed}:{round_no}").shuffle(idx)
    return idx


def pick(hdris: Sequence[Dict], seed: int, change: int, yaw_range: Sequence[float] = (0.0, 360.0)) -> Tuple[Dict, float]:
    """The room of change ``change`` and its turn about the vertical in degrees, from ``seed`` and ``change`` alone.

    The rooms come in rounds: each round is every room once, in an order drawn from the seed and the round's number. So no
    room comes back before all the others have shown, and two changes in a row never show the same room (a round that
    would start with the room the last one ended on starts with its second instead)."""
    n = len(hdris)
    if n == 0:
        raise ValueError("no HDRIs")
    round_no, at = divmod(change, n)
    order = _order(seed, round_no, n)
    if n == 2:
        order = [seed % 2, 1 - seed % 2]  # two rooms simply take turns
    elif round_no > 0 and n >= 3 and order[0] == _order(seed, round_no - 1, n)[-1]:
        order[0], order[1] = order[1], order[0]
    yaw = random.Random(f"teleop-environment-yaw:{seed}:{change}").uniform(float(yaw_range[0]), float(yaw_range[1]))
    return hdris[order[at]], round(yaw, 3)


def dome_intensity(mean_radiance: float, hdri: Dict) -> float:
    """The dome light's intensity that gives the room ``mean_radiance`` on average (radiance = intensity x texture)."""
    return float(mean_radiance) / float(hdri["meanLuminance"])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="the indoor HDRIs around the twin's table (docs/adr/0037 decision 2)")
    ap.add_argument("verb", choices=("fetch", "status"))
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--count", type=int, default=1000, help="keep at most this many (default: every indoor HDRI)")
    args = ap.parse_args(argv)
    if args.verb == "fetch":
        m = fetch(args.root, args.count)
        s = status(args.root, verify=False)
        print(json.dumps(s))
        return 0 if len(m["hdris"]) >= 2 else 1
    s = status(args.root)
    print(json.dumps(s))
    return 0 if s["hdris"] >= 2 and not s["bad"] else 1


if __name__ == "__main__":
    sys.exit(main())
