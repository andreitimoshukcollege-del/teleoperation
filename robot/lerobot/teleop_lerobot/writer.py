"""A scenario's LeRobot dataset: takes appended as episodes, or the whole dataset rebuilt from its takes (docs/adr/0036
decisions 5 and 6). Needs ``lerobot`` (the version in ../requirements.txt) and Pillow, which it brings.

Beside LeRobot's own files, ``meta/teleop/`` holds what the dataset was made from:

- ``takes.jsonl``: one line per episode, in episode order;
- ``episode_<n>.timing.jsonl``: one line per frame, with its source times.

At the dataset's top, as the lab's recorder writes them (SINRG-Lab/world-models, ``record_jetrover.py``), so its tools
and habits carry over: ``capture_timing.jsonl`` (one line per frame, every episode) and ``raw_ros_events.jsonl`` (every
take's commands and readings, ``takes.raw_events``). Their times are the session's one clock; the lab's ``*_wall_ns``
fields are null, since a take keeps no wall clock per record.
"""

from __future__ import annotations

import io
import json
import os
import shutil
import time
from typing import Callable, Dict, List, Optional

from .takes import Take, TakeError, checksum_id, raw_events

TELEOP = os.path.join("meta", "teleop")


def features(take: Take) -> Dict[str, dict]:
    """The lab's feature names (SINRG-Lab/world-models data_collection), with this take's cameras."""
    n = len(take.names)
    out = {
        "observation.state": {"dtype": "float32", "shape": (n,), "names": list(take.names)},
        "action": {"dtype": "float32", "shape": (n,), "names": list(take.names)},
    }
    for cam in take.cameras:
        out["observation.images." + cam.name] = {"dtype": "video", "shape": (cam.height, cam.width, 3),
                                                 "names": ["height", "width", "channels"]}
    return out


def read_ledger(root: str) -> List[dict]:
    path = os.path.join(root, TELEOP, "takes.jsonl")
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def recorded_episodes(root: str) -> Optional[int]:
    """How many episodes LeRobot's own metadata says the dataset has, or None without one."""
    path = os.path.join(root, "meta", "info.json")
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return int(json.load(f).get("total_episodes", 0))


def repo_id(group: str, scenario: str) -> str:
    return f"local/{group}_{scenario}"


class LeRobotWriter:
    """Writes episodes with ``lerobot``. Imported lazily, so the rest of the package runs without it."""

    def __init__(self, vcodec: str = "libsvtav1"):
        import numpy as np  # noqa: F401  (lerobot's own dependency)
        from PIL import Image  # noqa: F401
        from lerobot.datasets import LeRobotDataset  # noqa: F401  (the import the lab's recorders use)

        self.vcodec = vcodec

    def append(self, root: str, repo: str, take: Take) -> int:
        """Appends ``take`` as the next episode of the dataset at ``root`` (created if absent). Returns its index."""
        import numpy as np
        from PIL import Image
        from lerobot.datasets import LeRobotDataset

        if recorded_episodes(root) is None:
            if os.path.exists(root):
                shutil.rmtree(root)  # a folder with no info.json: a write that died before LeRobot's first episode
            ds = LeRobotDataset.create(repo_id=repo, fps=take.fps, features=features(take), root=root,
                                       robot_type=take.robot, use_videos=True, vcodec=self.vcodec)
        else:
            ds = LeRobotDataset.resume(repo_id=repo, root=root, vcodec=self.vcodec)
            want, have = features(take), ds.meta.features
            images = {k for k in have if k.startswith("observation.images.")}
            if (ds.meta.fps != take.fps or images != {k for k in want if k.startswith("observation.images.")}
                    or any(k not in have or tuple(have[k]["shape"]) != tuple(want[k]["shape"]) for k in want)):
                ds.finalize()
                raise TakeError(f"{take.path}: its cameras, sizes or rate differ from the dataset's at {root}")
        try:
            episode = int(ds.meta.total_episodes)
            for frame in take.frames:
                row = {"observation.state": np.asarray(frame.state, dtype=np.float32),
                       "action": np.asarray(frame.action, dtype=np.float32),
                       "task": take.task}
                for cam in take.cameras:
                    image = Image.open(io.BytesIO(take.jpeg(frame, cam))).convert("RGB")
                    if image.size != (cam.width, cam.height):
                        raise TakeError(f"{take.path}: a {cam.name} frame is {image.size}, not {cam.width}x{cam.height}")
                    row["observation.images." + cam.name] = np.asarray(image)
                ds.add_frame(row)
            ds.save_episode(parallel_encoding=False)  # no forked encoder pool: this process runs for days, and torch makes it multi-threaded
        finally:
            ds.finalize()
        return episode


def append_take(writer, root: str, take: Take) -> dict:
    """Writes one take as an episode and records it in ``meta/teleop``. Returns its ledger line."""
    group, scenario = take.dataset_group, take.scenario
    episode = writer.append(root, repo_id(group, scenario), take)
    teleop = os.path.join(root, TELEOP)
    os.makedirs(teleop, exist_ok=True)
    with open(os.path.join(teleop, f"episode_{episode:06d}.timing.jsonl"), "w", encoding="utf-8") as f:
        for i, frame in enumerate(take.frames):
            f.write(json.dumps({"frameIndex": i, **frame.timing}, separators=(",", ":")) + "\n")
    # The lab recorder's two files (see the module doc): its field names, filled from the take's one clock.
    with open(os.path.join(root, "capture_timing.jsonl"), "a", encoding="utf-8") as f:
        for i, frame in enumerate(take.frames):
            t = frame.timing
            f.write(json.dumps({
                "episode_saved_so_far": episode,
                "frame_in_episode": i,
                "pc_sample_mono_ns": t["grid"],
                "jetson_state_wall_ns": None,
                "jetson_action_wall_ns": None,
                "pc_state_recv_mono_ns": t["sensed"] - t["stateAge"],
                "pc_action_recv_mono_ns": t["sensed"] - t["actionAge"],
                "pc_camera_read_mono_ns": t["sensed"],
            }, separators=(",", ":")) + "\n")
    with open(os.path.join(root, "raw_ros_events.jsonl"), "a", encoding="utf-8") as f:
        for event in raw_events(take):
            f.write(json.dumps(event, separators=(",", ":")) + "\n")
    line = {"episode": episode, "take": os.path.abspath(take.path), "session": take.session_id,
            "filesSha256": checksum_id(take.path), "frames": len(take.frames),
            "filled": sum(1 for f in take.frames if f.timing["filled"]), "fps": take.fps, "task": take.task,
            "convertedUtc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    with open(os.path.join(teleop, "takes.jsonl"), "a", encoding="utf-8") as f:
        f.write(json.dumps(line, separators=(",", ":")) + "\n")
    return line


def rebuild(writer, root: str, takes: List[Take], log: Callable[[str], None] = print) -> List[dict]:
    """Writes ``takes`` as a fresh dataset beside ``root``, then swaps it in. LeRobot has no episode deletion, and a
    rebuild from the same takes gives the same episodes."""
    stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    fresh, old = f"{root}.rebuild-{stamp}", f"{root}.old-{stamp}"
    lines = [append_take(writer, fresh, take) for take in takes]
    if os.path.exists(root):
        os.rename(root, old)
    if lines:
        os.rename(fresh, root)
    if os.path.exists(old):
        shutil.rmtree(old)
    log(f"rebuilt {root}: {len(lines)} episode(s)")
    return lines
