"""Backdrop images for the renderer (docs/adr/0032 decision 4): Open Images V7's validation split, only images licensed
CC BY 2.0, each recorded with its author and source.

Run on the GPU machine (``just isaac-backdrops-fetch``, ``just isaac-backdrops-status``):

    python -m teleop_isaac.backdrops fetch --count 2000
    python -m teleop_isaac.backdrops status

**Downloads are untrusted data.** Each image is fetched into a temporary file and kept only if it is at most 20 MB, starts
like a JPEG, and decodes (with Pillow, when it is there) to at least 256 x 256. It is stored as ``images/<sha256>.jpg``:
no name on disk comes from anything the network sent. ``manifest.json`` lists every kept image: its sha256, size, licence,
author, title and pages. The images come from the CVDF mirror (``open-images-dataset.s3.amazonaws.com``), which resizes
them, so Open Images' ``OriginalMD5`` (the original's) does not apply; the sha256 is ours.

:func:`pick` chooses the table mat's and the screen's images for a backdrop change, from a seed and the change's number.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
import random
import sys
import tempfile
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional, Sequence, Tuple

METADATA_URL = "https://storage.googleapis.com/openimages/2018_04/validation/validation-images-with-rotation.csv"
IMAGE_URL = "https://open-images-dataset.s3.amazonaws.com/validation/{image_id}.jpg"
CC_BY_2 = "https://creativecommons.org/licenses/by/2.0/"
SOURCE = "Open Images V7, validation split (CVDF mirror)"
MAX_BYTES = 20 * 1024 * 1024
MIN_SIDE = 256
DEFAULT_ROOT = os.path.expanduser("~/isaacsim/backdrops/openimages-v7-val")


def candidates(metadata_csv: str) -> List[Dict[str, str]]:
    """Rows licensed CC BY 2.0 that need no rotation, in a fixed order (by image id)."""
    out = []
    with open(metadata_csv, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("License") != CC_BY_2:
                continue
            if row.get("Rotation", "") not in ("", "0", "0.0"):
                continue
            if not all(c in "0123456789abcdef" for c in row.get("ImageID", "")) or len(row["ImageID"]) != 16:
                continue  # the id goes into a URL: only what an Open Images id can be
            out.append(row)
    out.sort(key=lambda r: r["ImageID"])
    return out


def check_image(data: bytes) -> Optional[Tuple[int, int]]:
    """(width, height) of an acceptable JPEG, or None."""
    if len(data) > MAX_BYTES or not data.startswith(b"\xff\xd8\xff"):
        return None
    try:
        from PIL import Image
    except ImportError:
        return (0, 0)  # no Pillow: the magic check is all there is
    try:
        with Image.open(io.BytesIO(data)) as im:
            im.verify()
        with Image.open(io.BytesIO(data)) as im:
            w, h = im.size
            if im.format != "JPEG":
                return None
    except Exception:  # noqa: BLE001 -- anything Pillow cannot read is refused
        return None
    return (w, h) if min(w, h) >= MIN_SIDE else None


def entry(row: Dict[str, str], sha: str, size: int, wh: Tuple[int, int]) -> Dict:
    return {"file": f"images/{sha}.jpg", "sha256": sha, "bytes": size, "width": wh[0], "height": wh[1],
            "imageId": row["ImageID"], "license": row["License"], "author": row.get("Author", ""),
            "authorProfileUrl": row.get("AuthorProfileURL", ""), "title": row.get("Title", ""),
            "landingUrl": row.get("OriginalLandingURL", ""), "source": SOURCE,
            "url": IMAGE_URL.format(image_id=row["ImageID"])}


def load_manifest(root: str) -> Dict:
    path = os.path.join(root, "manifest.json")
    if not os.path.exists(path):
        return {"format": "teleop-backdrops", "version": 1, "source": SOURCE, "license": CC_BY_2, "images": []}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_manifest(root: str, manifest: Dict) -> None:
    path = os.path.join(root, "manifest.json")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)


def _download(url: str, timeout: float = 60.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        data = r.read(MAX_BYTES + 1)
    return data


def fetch(root: str, count: int, seed: int = 1, workers: int = 8, say=print) -> Dict:
    os.makedirs(os.path.join(root, "images"), exist_ok=True)
    meta = os.path.join(root, "validation-images-with-rotation.csv")
    if not os.path.exists(meta):
        say(f"[backdrops] fetching the metadata ({METADATA_URL})")
        data = _download(METADATA_URL, timeout=300)
        if len(data) > MAX_BYTES:
            raise RuntimeError(f"the metadata is over {MAX_BYTES} bytes: not what was expected")
        with open(meta + ".tmp", "wb") as f:
            f.write(data)
        os.replace(meta + ".tmp", meta)
    rows = candidates(meta)
    random.Random(seed).shuffle(rows)
    manifest = load_manifest(root)
    have = {e["imageId"] for e in manifest["images"]}
    say(f"[backdrops] {len(rows)} CC BY 2.0 images in the split; {len(have)} kept already; want {count}")
    todo = [r for r in rows if r["ImageID"] not in have]
    refused = failed = 0
    started = time.monotonic()

    def one(row):
        try:
            data = _download(IMAGE_URL.format(image_id=row["ImageID"]))
        except Exception as e:  # noqa: BLE001
            return row, None, str(e)
        wh = check_image(data)
        if wh is None:
            return row, None, "refused"
        sha = hashlib.sha256(data).hexdigest()
        path = os.path.join(root, "images", f"{sha}.jpg")
        if not os.path.exists(path):
            fd, tmp = tempfile.mkstemp(dir=os.path.join(root, "images"), suffix=".part")
            with os.fdopen(fd, "wb") as f:
                f.write(data)
            os.replace(tmp, path)
        return row, entry(row, sha, len(data), wh), None

    with ThreadPoolExecutor(max_workers=workers) as pool:
        at = 0
        while len(manifest["images"]) < count and at < len(todo):
            batch = todo[at:at + workers * 4]
            at += len(batch)
            for row, e, err in pool.map(one, batch):
                if e is not None and len(manifest["images"]) < count:
                    if e["sha256"] not in {x["sha256"] for x in manifest["images"]}:
                        manifest["images"].append(e)
                elif err == "refused":
                    refused += 1
                elif err:
                    failed += 1
            save_manifest(root, manifest)
            say(f"[backdrops] {len(manifest['images'])}/{count} kept, {refused} refused, {failed} failed, "
                f"{time.monotonic() - started:.0f} s")
    return manifest


def status(root: str, verify: bool = True) -> Dict:
    m = load_manifest(root)
    bad = []
    total = 0
    for e in m["images"]:
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
    licences = sorted({e["license"] for e in m["images"]})
    return {"root": root, "images": len(m["images"]), "bytes": total, "licences": licences, "bad": bad,
            "authors": len({e["author"] for e in m["images"]})}


def pick(images: Sequence[Dict], seed: int, change: int) -> Tuple[Dict, Dict]:
    """The mat's and the screen's images for backdrop change ``change`` (0 at start-up, then one per session closed):
    two different images, drawn from ``seed`` and ``change`` alone."""
    if len(images) < 2:
        raise ValueError("at least two backdrop images are needed")
    rng = random.Random(f"teleop-backdrop:{seed}:{change}")
    a, b = rng.sample(range(len(images)), 2)
    return images[a], images[b]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="backdrop images for the Isaac renderer (docs/adr/0032 decision 4)")
    ap.add_argument("verb", choices=("fetch", "status"))
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--count", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)
    if args.verb == "fetch":
        m = fetch(args.root, args.count, args.seed)
        s = status(args.root, verify=False)
        print(json.dumps(s))
        return 0 if len(m["images"]) >= args.count else 1
    s = status(args.root)
    print(json.dumps(s))
    return 0 if s["images"] >= 2 and not s["bad"] else 1


if __name__ == "__main__":
    sys.exit(main())
