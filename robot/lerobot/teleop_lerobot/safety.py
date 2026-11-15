"""Safety datasets from the JetRover twin's scripted takes, labelled by physics (docs/adr/0036 amendment 1).

The four categories are the labmate's (SINRG-Lab/world-models, ``data_collection/labeling/label_safety.py``); each is a
take scenario of the same name, driven by ``joint-drive --script <category>``. A frame is **unsafe** when the arm touched an
object at any moment of its frame period (half a frame either side of the time it shows): the contacts are the take's
``WC`` records (the Isaac bridge's PhysX contacts, RobotHost's clock), and a frame's time is the state it shows
(``Frame.timing["sensed"]``, the same clock). At 15 fps a frame lasts 67 ms and a recovering touch can be shorter, so the
instant alone could miss it.

A take is checked against its category before it is kept (:func:`check`), and the dataset is written in his format, file
for file (:func:`write_labels`):

- ``manual_safety_labels.json``: ``{"<episode_index>": [[start, end], ...]}``, inclusive, episode-relative unsafe ranges;
  an episode with none is left out (his convention: all safe);
- ``meta/safety_labels.parquet``: the rows ``label_safety.py`` writes from that JSON (``episode_index``, ``frame_index``,
  ``label``, ``category``, ``effective_category``, ``overridden``, ``source``), ``source`` ``physics``;
- ``safety_events.jsonl``: ``contact`` (the first unsafe frame) and, for ``collide_and_recover``, ``clear`` (the first safe
  frame after the last unsafe one), in his ``record_jetrover.py`` fields, so his lead-window labels are one command away.

No LeRobot or pandas here but in :func:`write_labels`'s parquet, so the tests run with plain Python.
"""

from __future__ import annotations

import json
import math
import os
import shutil
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .jlog import items, read_jlog
from .takes import Camera, Frame, Take

REACH, STOP, RECOVER, COLLIDE = "reach_no_obstacle", "stop_before_collision", "collide_and_recover", "collide_with_obstacle"
CATEGORIES = (REACH, STOP, RECOVER, COLLIDE)
SOURCE = "physics"
FRONT = "wrist"  # the twin's camera that is the real robot's (on the arm): his datasets' observation.images.front
TIPPED_DEGREES = 30.0  # collide_with_obstacle: a touched object tipped at least this far, or
MOVED_METERS = 0.03  # moved at least this far
UPRIGHT_DEGREES = 20.0  # collide_and_recover: every touched object still within this of where it stood


@dataclass(frozen=True)
class Contact:
    """One stretch of the arm touching objects: from the first ``WC`` naming a contact to the ``WC`` that ends it (or the
    take's end), and which objects it touched (``link:object`` pairs as RobotHost records them)."""

    start: int
    end: int
    objects: Tuple[int, ...]


def contacts(path: str) -> List[Contact]:
    """The take's contact stretches from its ``WC`` records, by their ``sensed`` times."""
    host = read_jlog(os.path.join(path, "robothost.jlog"), "robothost")
    rows = sorted(((int(r[1]), r[2] if len(r) > 2 else "") for r in host.tag("WC")), key=lambda x: x[0])
    out, start, touched = [], None, set()
    for sensed, text in rows:
        pairs = [it for it in items(text) if len(it) >= 3]
        if pairs:
            if start is None:
                start, touched = sensed, set()
            touched.update(int(p[2]) for p in pairs)
        elif start is not None:
            out.append(Contact(start, sensed, tuple(sorted(touched))))
            start = None
    if start is not None:
        out.append(Contact(start, OPEN_END, tuple(sorted(touched))))  # still touching when the take ended: to its end
    return out


OPEN_END = 2 ** 62  # a contact no WC ended


def frame_labels(take: Take, stretches: Sequence[Contact]) -> List[int]:
    """1 for each frame whose period (its time, half a frame either side) overlaps a contact, else 0."""
    tps = 1_000_000_000
    half = tps // (2 * take.fps)
    out = []
    for f in take.frames:
        t = int(f.timing["sensed"])
        out.append(1 if any(c.start <= t + half and c.end >= t - half for c in stretches) else 0)
    return out


def object_poses(path: str) -> Dict[int, List[Tuple[int, Tuple[float, ...]]]]:
    """Each object's recorded poses (``WO``), (sensed, (x, y, z, qw, qx, qy, qz)), in time order."""
    host = read_jlog(os.path.join(path, "robothost.jlog"), "robothost")
    out: Dict[int, List[Tuple[int, Tuple[float, ...]]]] = {}
    for r in host.tag("WO"):
        sensed = int(r[1])
        for it in items(r[3] if len(r) > 3 else ""):
            if len(it) >= 8:
                out.setdefault(int(it[0]), []).append((sensed, tuple(float(v) for v in it[1:8])))
    for poses in out.values():
        poses.sort(key=lambda p: p[0])
    return out


def _tilt_degrees(a: Tuple[float, ...], b: Tuple[float, ...]) -> float:
    """The angle between two poses' up axes (z), degrees: how far the object tipped from ``a`` to ``b``."""
    def up(q):
        w, x, y, z = q[3:7]
        return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    ua, ub = up(a), up(b)
    dot = max(-1.0, min(1.0, sum(p * q for p, q in zip(ua, ub))))
    return math.degrees(math.acos(dot))


def _at(poses: List[Tuple[int, Tuple[float, ...]]], t: int) -> Optional[Tuple[float, ...]]:
    before = [p for s, p in poses if s <= t]
    return before[-1] if before else (poses[0][1] if poses else None)


@dataclass(frozen=True)
class Verdict:
    ok: bool
    reason: str
    unsafe_frames: int
    touched: Tuple[int, ...]


def check(take: Take, labels: Sequence[int], stretches: Sequence[Contact],
          poses: Dict[int, List[Tuple[int, Tuple[float, ...]]]]) -> Verdict:
    """Whether the take is what its category says (docs/adr/0036 amendment 1, decision 3)."""
    category = take.scenario
    unsafe = sum(labels)
    half = 1_000_000_000 // (2 * take.fps)  # as frame_labels: a frame's period reaches half a frame past its time
    t0, t1 = int(take.frames[0].timing["sensed"]) - half, int(take.frames[-1].timing["sensed"]) + half
    inside = [c for c in stretches if c.end >= t0 and c.start <= t1]
    touched = tuple(sorted({o for c in inside for o in c.objects}))

    def moved(obj: int) -> Tuple[float, float]:
        p = poses.get(obj, [])
        a, b = _at(p, min(c.start for c in inside)), _at(p, t1)
        if a is None or b is None:
            return 0.0, 0.0
        return _tilt_degrees(a, b), math.dist(a[:3], b[:3])

    if category not in CATEGORIES:
        return Verdict(False, f"{category!r} is not a safety category", unsafe, touched)
    if category in (REACH, STOP):
        if unsafe or inside:
            return Verdict(False, f"{category} touched object(s) {list(touched)} ({unsafe} unsafe frames)", unsafe, touched)
        return Verdict(True, "no contact", 0, ())
    if not unsafe:
        return Verdict(False, f"{category} never touched an object", 0, ())
    if category == RECOVER:
        if labels[-1]:
            return Verdict(False, "collide_and_recover was still touching at the episode's end: no recovery", unsafe, touched)
        for obj in touched:
            tilt, dist = moved(obj)
            if tilt > UPRIGHT_DEGREES:
                return Verdict(False, f"collide_and_recover tipped object {obj} by {tilt:.0f} degrees", unsafe, touched)
        return Verdict(True, f"touched {list(touched)} and backed off", unsafe, touched)
    knocked = [(obj, moved(obj)) for obj in touched]
    if not any(tilt >= TIPPED_DEGREES or dist >= MOVED_METERS for _, (tilt, dist) in knocked):
        worst = max(knocked, key=lambda k: (k[1][0], k[1][1]))
        return Verdict(False, f"collide_with_obstacle touched {list(touched)} but tipped none (object {worst[0]}: "
                              f"{worst[1][0]:.0f} degrees, {worst[1][1] * 100:.1f} cm)", unsafe, touched)
    return Verdict(True, f"knocked {[o for o, (tl, d) in knocked if tl >= TIPPED_DEGREES or d >= MOVED_METERS]}", unsafe, touched)


def as_safety(take: Take) -> Take:
    """The take as the safety dataset writes it: its ``wrist`` camera named ``front`` (his tools read that), and its task
    string its category (his ``label_safety.py`` reads an episode's category from it)."""
    if not any(c.name == FRONT for c in take.cameras):
        raise ValueError(f"{take.path}: no {FRONT} camera to be the front one")
    rename = {FRONT: "front"}
    cameras = [Camera(rename.get(c.name, c.name), c.width, c.height, c.sidecar) for c in take.cameras]
    frames = [Frame({rename.get(k, k): v for k, v in f.parts.items()}, f.state, f.action, f.timing) for f in take.frames]
    return Take(take.path, take.session_id, take.robot, take.source, take.scenario, take.scenario, take.names, take.fps,
                cameras, frames)


def ranges(labels: Sequence[int]) -> List[List[int]]:
    """The unsafe frames as inclusive [start, end] ranges."""
    out, start = [], None
    for i, v in enumerate(list(labels) + [0]):
        if v and start is None:
            start = i
        elif not v and start is not None:
            out.append([start, i - 1])
            start = None
    return out


def events(episode: int, category: str, labels: Sequence[int]) -> List[dict]:
    """His ``safety_events.jsonl`` markers for one episode: ``contact`` at the first unsafe frame, and for
    ``collide_and_recover`` ``clear`` at the first safe frame after the last unsafe one."""
    unsafe = [i for i, v in enumerate(labels) if v]
    if not unsafe:
        return []
    out = [{"episode": episode, "kind": "contact", "frame_in_episode": unsafe[0], "source": SOURCE}]
    if category == RECOVER and unsafe[-1] + 1 < len(labels):
        out.append({"episode": episode, "kind": "clear", "frame_in_episode": unsafe[-1] + 1, "source": SOURCE})
    return out


def label_rows(episode: int, category: str, labels: Sequence[int]) -> List[dict]:
    """``meta/safety_labels.parquet``'s rows for one episode, as ``label_safety.py`` writes them."""
    return [{"episode_index": episode, "frame_index": i, "label": int(v), "category": category,
             "effective_category": category, "overridden": False, "source": SOURCE} for i, v in enumerate(labels)]


def write_labels(root: str, episodes: Sequence[Tuple[int, str, Sequence[int]]]) -> None:
    """Writes his three label files at the dataset ``root`` for ``episodes`` ((index, category, frame labels) each),
    replacing any there."""
    manual = {str(ep): ranges(labels) for ep, _, labels in episodes if any(labels)}
    with open(os.path.join(root, "manual_safety_labels.json"), "w", encoding="utf-8") as f:
        json.dump(manual, f, indent=2)
    with open(os.path.join(root, "safety_events.jsonl"), "w", encoding="utf-8") as f:
        for ep, category, labels in episodes:
            for e in events(ep, category, labels):
                f.write(json.dumps(e, separators=(",", ":")) + "\n")
    import pandas as pd  # lerobot's own dependency

    rows = [r for ep, category, labels in episodes for r in label_rows(ep, category, labels)]
    os.makedirs(os.path.join(root, "meta"), exist_ok=True)
    pd.DataFrame(rows, columns=["episode_index", "frame_index", "label", "category", "effective_category", "overridden",
                                "source"]).to_parquet(os.path.join(root, "meta", "safety_labels.parquet"), index=False)


def set_aside(take_path: str, verdict: Verdict) -> str:
    """Moves a take that failed its check to ``<root>/_discarded/<scenario>/<take>`` (where DISCARD LAST puts one), with
    the verdict beside it as ``safety-check.json``. Returns the new path."""
    folder = os.path.dirname(os.path.abspath(take_path))
    root, scenario, name = os.path.dirname(folder), os.path.basename(folder), os.path.basename(take_path.rstrip("/"))
    dest = os.path.join(root, "_discarded", scenario, name)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.move(take_path, dest)
    with open(os.path.join(dest, "safety-check.json"), "w", encoding="utf-8") as f:
        json.dump({"ok": verdict.ok, "reason": verdict.reason, "unsafeFrames": verdict.unsafe_frames,
                   "touched": list(verdict.touched)}, f, indent=2)
    return dest


def build(writer, root: str, takes: Sequence[Take], log=print) -> List[Tuple[int, str, List[int]]]:
    """Writes ``takes`` (already checked) as one safety dataset at ``root``: each an episode, in order, then his label
    files. A dataset already there is refused: a batch's dataset is written once, from its kept takes."""
    from .writer import append_take

    if os.path.exists(root):
        raise FileExistsError(f"{root} exists: a safety dataset is written once (rebuild it into a new folder)")
    written = []
    for take in takes:
        labels = frame_labels(take, contacts(take.path))
        line = append_take(writer, root, as_safety(take), repo=f"local/{take.dataset_group}-safety_{os.path.basename(root)}")
        written.append((int(line["episode"]), take.scenario, labels))
        log(f"episode {line['episode']}: {take.scenario}, {len(labels)} frames, {sum(labels)} unsafe ({os.path.basename(take.path)})")
    write_labels(root, written)
    return written


__all__ = ["CATEGORIES", "Contact", "Verdict", "as_safety", "build", "check", "contacts", "events", "frame_labels",
           "label_rows", "object_poses", "ranges", "set_aside", "write_labels"]
