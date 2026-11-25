"""How big a take's downsampled depth and masks are stored two ways (docs/adr/0038 "V, views", "In the safety datasets"):
as ``teleop_lerobot.depth_masks``' per-episode ``.npz`` and as LeRobot array features in a dataset of their own. Run on the
GPU machine with the LeRobot venv, from robot/lerobot, writing only under ``--scratch``:

    ~/isaacsim/lerobot-venv/bin/python ../../scripts/gpu/depth_mask_sizes.py <take>... --scratch /tmp/agent-x/sizes [--size 160x90]

Prints one line per take and storage, then ``RESULT {...}``: bytes an episode and a frame each way, LeRobot's split into its
data parquet, its episodes metadata and stats.json, and how long each took to write and to read back.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time

sys.path.insert(0, os.getcwd())

from teleop_lerobot import depth_masks as D  # noqa: E402
from teleop_lerobot.takes import read_take  # noqa: E402


def folder_bytes(root: str, sub: str = "") -> int:
    return sum(os.path.getsize(os.path.join(d, n)) for d, _, ns in os.walk(os.path.join(root, sub)) for n in ns)


def lerobot_size(arrays, frames: int, root: str):
    import numpy as np
    from lerobot.datasets import LeRobotDataset

    keys = [k for k in arrays if k.endswith(".depth") or k.endswith(".mask")]
    feats = {"observation." + k.replace(".", "_"): {"dtype": "uint16", "shape": tuple(arrays[k].shape[1:]),
                                                    "names": ["height", "width"]} for k in keys}
    if os.path.exists(root):
        shutil.rmtree(root)
    t0 = time.time()
    ds = LeRobotDataset.create(repo_id="local/sizes", fps=15, features=feats, root=root, robot_type="jetrover", use_videos=False)
    for i in range(frames):
        ds.add_frame({**{"observation." + k.replace(".", "_"): np.asarray(arrays[k][i]) for k in keys}, "task": "move the arm"})
    ds.save_episode()
    ds.finalize()
    write_s = time.time() - t0
    t0 = time.time()
    back = LeRobotDataset("local/sizes", root=root)
    item = back[frames // 2]
    read_s = time.time() - t0
    k0 = "observation." + keys[0].replace(".", "_")
    same = bool((np.asarray(item[k0]) == arrays[keys[0]][frames // 2]).all())
    stats = os.path.join(root, "meta", "stats.json")
    return {"total": folder_bytes(root), "data": folder_bytes(root, "data"), "episodesMeta": folder_bytes(root, os.path.join("meta", "episodes")),
            "stats": os.path.getsize(stats) if os.path.exists(stats) else 0, "writeS": round(write_s, 2), "readS": round(read_s, 2),
            "readBackSame": same, "dtypeBack": str(np.asarray(item[k0]).dtype)}


def main(argv=None) -> int:
    a = argparse.ArgumentParser()
    a.add_argument("takes", nargs="+")
    a.add_argument("--scratch", required=True)
    a.add_argument("--size", default="160x90")
    args = a.parse_args(argv)
    size = D.parse_size(args.size)
    os.makedirs(args.scratch, exist_ok=True)
    out = []
    for path in args.takes:
        try:
            take = read_take(path, views=True)
        except Exception:  # noqa: BLE001  (a take without views: its three cameras alone)
            take = read_take(path)
        t0 = time.time()
        arrays = D.episode_arrays(take, size, {"wrist": "front"})
        decode_s = time.time() - t0
        npz = os.path.join(args.scratch, os.path.basename(path) + ".npz")
        t0 = time.time()
        n = D.write_npz(npz, arrays)
        npz_s = time.time() - t0
        t0 = time.time()
        import numpy as np

        loaded = np.load(npz)
        _ = {k: loaded[k] for k in loaded.files}
        load_s = time.time() - t0
        import zipfile

        with zipfile.ZipFile(npz) as z:
            entries = {i.filename[:-4]: i.compress_size for i in z.infolist()}
        by_stream = {st: sum(v for k, v in entries.items() if k.endswith("." + st)) for st in ("depth", "mask")}
        frames = len(take.frames)
        cams = [k[:-6] for k in arrays if k.endswith(".depth")]
        per_stream = {k: int(arrays[k].nbytes) for k in arrays if "." in k}
        lr = lerobot_size(arrays, frames, os.path.join(args.scratch, os.path.basename(path) + ".lerobot"))
        row = {"take": os.path.basename(path), "frames": frames, "cameras": cams, "raw": sum(per_stream.values()),
               "npz": n, "npzByStream": by_stream, "npzKbFrame": round(n / frames / 1e3, 1), "decodeS": round(decode_s, 2), "npzWriteS": round(npz_s, 2),
               "npzLoadS": round(load_s, 2), "lerobot": lr, "lerobotKbFrame": round(lr["total"] / frames / 1e3, 1)}
        print(json.dumps(row), flush=True)
        out.append(row)
    print("RESULT " + json.dumps({"size": list(size), "takes": out}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
