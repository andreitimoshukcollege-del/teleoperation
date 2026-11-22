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

import collections
import contextlib
import functools
import io
import json
import os
import shutil
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterator, List, Optional, Sequence, Tuple

from .takes import Take, TakeError, checksum_id, raw_events

TELEOP = os.path.join("meta", "teleop")

# The safety datasets' video encoding (docs/adr/0036 amendment 2). LeRobot's own (AV1, CRF 30, preset 12) kept 89% of the
# recorded frames' detail on the twin's wrist camera, 37.6 dB from them while it moves; CRF 18 at preset 8 keeps 96%, 39.9 dB,
# for 1.6x the size (15.7 MB a minute a camera at 640x360, 15 fps; measured 2026-11-15).
SAFETY_CRF = 18
SAFETY_PRESET = 8


@contextlib.contextmanager
def video_encoding(crf: Optional[int] = None, preset: Optional[int] = None) -> Iterator[None]:
    """LeRobot 0.5.1 encodes each episode's videos with ``encode_video_frames``'s defaults and passes no quality option on from
    ``LeRobotDataset``; inside this block, its writer's call gets ``crf`` and ``preset`` instead. Nothing changes when both
    are None. A lerobot without that call is an error, never a silent default."""
    if crf is None and preset is None:
        yield
        return
    from lerobot.datasets import dataset_writer

    original = getattr(dataset_writer, "encode_video_frames", None)
    if original is None:
        raise RuntimeError("lerobot.datasets.dataset_writer has no encode_video_frames to give a CRF to: "
                           "not the lerobot in requirements.txt")

    def encode(*args, **kwargs):
        if crf is not None:
            kwargs["crf"] = crf
        if preset is not None:
            kwargs["preset"] = preset
        return original(*args, **kwargs)

    dataset_writer.encode_video_frames = encode
    try:
        yield
    finally:
        dataset_writer.encode_video_frames = original


@dataclass
class Prepared:
    """One episode's frames and videos made ahead of its append (docs/adr/0036 amendment 6), by :func:`prepare_episode` in
    a worker process: per camera feature, each frame as the PNG LeRobot's ``add_frame`` would have written and the video
    LeRobot's writer would have encoded from those PNGs. ``folder`` holds them all and goes once the episode is written."""

    episode: int
    folder: str
    images: Dict[str, List[str]]  # "observation.images.<cam>" -> a PNG a frame
    videos: Dict[str, str]  # "observation.images.<cam>" -> its mp4, alone in a folder of its own (LeRobot removes it)


def prepare_episode(take: Take, episode: int, folder: str, vcodec: str = "libsvtav1", crf: Optional[int] = None,
                    preset: Optional[int] = None) -> Prepared:
    """What :meth:`LeRobotWriter.append` and LeRobot 0.5.1 do to a take's pictures, done ahead in a worker: each JPEG decoded
    and written as a PNG by LeRobot's own ``write_image`` at the level ``add_frame`` writes a video feature's (1), then each
    camera encoded from those PNGs by LeRobot's own ``encode_video_frames`` with the arguments its writer passes (and
    ``crf``/``preset`` as :func:`video_encoding` gives them). The same pixels into the same encoder with the same settings:
    the episode's files are the ones a sequential append writes (tested byte for byte)."""
    import numpy as np
    from PIL import Image
    from lerobot.datasets.image_writer import write_image
    from lerobot.datasets.video_utils import encode_video_frames

    images: Dict[str, List[str]] = {}
    videos: Dict[str, str] = {}
    for cam in take.cameras:
        key = "observation.images." + cam.name
        pngs = os.path.join(folder, key)
        os.makedirs(pngs, exist_ok=True)
        paths = []
        for i, frame in enumerate(take.frames):
            image = Image.open(io.BytesIO(take.jpeg(frame, cam))).convert("RGB")
            if image.size != (cam.width, cam.height):
                raise TakeError(f"{take.path}: a {cam.name} frame is {image.size}, not {cam.width}x{cam.height}")
            path = os.path.join(pngs, f"frame-{i:06d}.png")
            write_image(np.asarray(image), path, compress_level=1)
            if not os.path.exists(path):  # write_image logs a failure and returns
                raise OSError(f"{path}: not written")
            paths.append(path)
        images[key] = paths
        video = os.path.join(folder, key + ".video", f"{key}_{episode:03d}.mp4")
        quality = {k: v for k, v in (("crf", crf), ("preset", preset)) if v is not None}
        encode_video_frames(pngs, video, take.fps, vcodec=vcodec, overwrite=True, encoder_threads=None, **quality)
        videos[key] = video
    return Prepared(episode, folder, images, videos)


_PREPARED_IMAGE: list = []  # the PIL.Image.Image subclass, made on first use (PIL is imported lazily, as lerobot is)


def _prepared_image(source: str):
    """A frame :func:`prepare_episode` already wrote, for ``add_frame``'s row: LeRobot's frame check takes any PIL image, and
    the writer's ``_save_image`` (see :meth:`LeRobotWriter._use_prepared`) moves ``source`` into place. It has no pixels of
    its own, so a writer that tried to save it as an image would fail, never write a blank frame."""
    if not _PREPARED_IMAGE:
        from PIL import Image

        _PREPARED_IMAGE.append(type("PreparedImage", (Image.Image,), {}))
    image = _PREPARED_IMAGE[0]()
    image.prepared_source = source
    return image


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

    def __init__(self, vcodec: str = "libsvtav1", crf: Optional[int] = None, preset: Optional[int] = None):
        import numpy as np  # noqa: F401  (lerobot's own dependency)
        from PIL import Image  # noqa: F401
        from lerobot.datasets import LeRobotDataset  # noqa: F401  (the import the lab's recorders use)

        self.vcodec = vcodec
        self.crf = crf
        self.preset = preset

    def encoding(self) -> dict:
        """How this writer's videos are encoded (``meta/teleop/encoding.json``): None is LeRobot's own default."""
        return {"vcodec": self.vcodec, "crf": self.crf, "preset": self.preset, "pixFmt": "yuv420p", "gop": 2}

    def prepare_job(self) -> Callable[[Take, int, str], Prepared]:
        """:func:`prepare_episode` with this writer's encoding, picklable for a worker process."""
        return functools.partial(prepare_episode, vcodec=self.vcodec, crf=self.crf, preset=self.preset)

    @staticmethod
    def _use_prepared(ds, prepared: Prepared) -> None:
        """Points this episode's writer at ``prepared``: ``add_frame``'s image save moves the prepared PNG to the path LeRobot
        chose (its stats read them there), and the episode's encode hands LeRobot the prepared video (removing the PNGs, as
        LeRobot's own encode does). Everything else, parquet, stats, metadata and file layout, is LeRobot's as before. A
        LeRobot without these two methods is an error, never a silent fallback."""
        from pathlib import Path

        from lerobot.datasets.utils import DEFAULT_IMAGE_PATH

        writer = ds.writer
        for name in ("_save_image", "_encode_temporary_episode_video"):
            if not callable(getattr(writer, name, None)):
                raise RuntimeError(f"lerobot's DatasetWriter has no {name} to hand prepared frames to: not the lerobot in "
                                   "requirements.txt")

        def save_image(image, fpath, compress_level=1):
            source = getattr(image, "prepared_source", None)
            if source is None:
                raise TypeError(f"{fpath}: a prepared episode's frame is not a prepared PNG")
            os.replace(source, fpath)

        def encode(video_key, episode_index):
            if int(episode_index) != prepared.episode:
                raise RuntimeError(f"episode {episode_index} was given the videos prepared for episode {prepared.episode}")
            frames = Path(ds.root) / DEFAULT_IMAGE_PATH.format(image_key=video_key, episode_index=episode_index, frame_index=0)
            shutil.rmtree(frames.parent)
            return Path(prepared.videos[video_key])

        writer._save_image = save_image
        writer._encode_temporary_episode_video = encode

    def append(self, root: str, repo: str, take: Take, prepared: Optional[Prepared] = None) -> int:
        """Appends ``take`` as the next episode of the dataset at ``root`` (created if absent). Returns its index.
        ``prepared``: its frames and videos, made ahead by :func:`prepare_episode` (a one-shot build's workers, never the
        watcher); without it, this process decodes and encodes them."""
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
            if prepared is not None:
                if prepared.episode != episode:
                    raise RuntimeError(f"{take.path}: prepared as episode {prepared.episode}, but the dataset's next is {episode}")
                self._use_prepared(ds, prepared)
            for i, frame in enumerate(take.frames):
                row = {"observation.state": np.asarray(frame.state, dtype=np.float32),
                       "action": np.asarray(frame.action, dtype=np.float32),
                       "task": take.task}
                for cam in take.cameras:
                    if prepared is not None:
                        row["observation.images." + cam.name] = _prepared_image(prepared.images["observation.images." + cam.name][i])
                        continue
                    image = Image.open(io.BytesIO(take.jpeg(frame, cam))).convert("RGB")
                    if image.size != (cam.width, cam.height):
                        raise TakeError(f"{take.path}: a {cam.name} frame is {image.size}, not {cam.width}x{cam.height}")
                    row["observation.images." + cam.name] = np.asarray(image)
                ds.add_frame(row)
            with video_encoding(self.crf, self.preset):
                ds.save_episode(parallel_encoding=False)  # no forked encoder pool: this process runs for days, and torch makes it multi-threaded
        finally:
            with video_encoding(self.crf, self.preset):
                ds.finalize()
        os.makedirs(os.path.join(root, TELEOP), exist_ok=True)
        with open(os.path.join(root, TELEOP, "encoding.json"), "w", encoding="utf-8") as f:
            json.dump(self.encoding(), f)
        return episode


PREPARE_SUFFIX = ".prepare"  # beside the dataset (same file system: its PNGs and videos are moved in, not copied)


def prepared_in_order(writer, root: str, takes: Sequence[Take], jobs: int, executor=None,
                      ahead: Optional[int] = None) -> Iterator[Tuple[Take, Prepared]]:
    """Each take with its :class:`Prepared` episode, in the order given, while workers prepare the ones after it
    (docs/adr/0036 amendment 6). Besides the one being written, at most ``ahead`` (default ``jobs + 2``) are prepared or
    being prepared, which bounds the disk their PNGs hold (about 300 MB an episode of the twin's three cameras). Episode numbers start at the dataset's next.
    The workers are ``jobs`` spawned processes (not forked: LeRobot brings torch), or ``executor`` (tests). Close the
    iterator (``contextlib.closing``) when done or failed: that cancels what is queued and removes ``<root>.prepare``."""
    folder = root + PREPARE_SUFFIX
    if os.path.exists(folder):
        shutil.rmtree(folder)  # left by a build that died
    os.makedirs(folder)
    first = recorded_episodes(root) or 0
    ahead = max(1, jobs + 2 if ahead is None else ahead)
    own = executor is None
    if own:
        import multiprocessing
        from concurrent.futures import ProcessPoolExecutor

        executor = ProcessPoolExecutor(max_workers=jobs, mp_context=multiprocessing.get_context("spawn"))
    job = writer.prepare_job()
    queue: "collections.deque" = collections.deque()
    todo = iter(enumerate(takes))

    def submit() -> None:
        nxt = next(todo, None)
        if nxt is not None:
            episode = first + nxt[0]
            queue.append((nxt[1], executor.submit(job, nxt[1], episode, os.path.join(folder, f"episode-{episode:06d}"))))

    try:
        for _ in range(ahead):
            submit()
        while queue:
            take, future = queue.popleft()
            prepared = future.result()
            submit()
            yield take, prepared
            shutil.rmtree(prepared.folder, ignore_errors=True)
    finally:
        for _, future in queue:
            future.cancel()
        if own:
            executor.shutdown(wait=True, cancel_futures=True)
        else:
            for _, future in queue:  # a test's executor: let what runs finish before its files go
                if not future.cancelled():
                    with contextlib.suppress(Exception):
                        future.result()
        shutil.rmtree(folder, ignore_errors=True)


def append_take(writer, root: str, take: Take, repo: Optional[str] = None, prepared: Optional[Prepared] = None) -> dict:
    """Writes one take as an episode and records it in ``meta/teleop``. Returns its ledger line. ``repo`` names the
    dataset when its episodes are of several scenarios (a safety dataset); by default it is the take's scenario's.
    ``prepared``: the episode's frames and videos made ahead (:func:`prepared_in_order`)."""
    group, scenario = take.dataset_group, take.scenario
    if prepared is None:
        episode = writer.append(root, repo or repo_id(group, scenario), take)
    else:
        episode = writer.append(root, repo or repo_id(group, scenario), take, prepared=prepared)
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
