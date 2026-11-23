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
- ``safety_events.jsonl``: ``contact`` at the first frame of each unsafe range and, for ``collide_and_recover``, ``clear`` at
  the first safe frame after it, in his ``record_jetrover.py`` fields, so his lead-window labels are one command away.

Since 2026-11-21 (docs/adr/0036 amendment 5) a take of an event category holds 1-3 events anywhere in it, each a stretch of
contact (or, for a stop, of coming close), and the take records what was planned (``SE``, :func:`planned_events`). Every
check holds for several: a recover take backs off from every touch, and each touched object is measured from its own first
touch.

Since 2026-11-22 (docs/adr/0038 decision 6):

- **The ``mixed`` script** (one scenario, physics decides the outcome) is never checked against a category: a take is set
  aside only when it is broken (:func:`broken`). What happened is computed afterwards and kept as metadata (``sidecars``).
- **The table is object 0** of ``WC``. It is never a contact for the labels or the checks (:func:`contacts`), so the four
  old categories behave as before; the sidecars label it on its own, and ``Options.table_unsafe`` counts it for anyone who
  wants that.
- **Captions:** every episode's task is ``move the arm`` (``sidecars.NEUTRAL_CAPTION``) unless the build asks for the old
  category captions (``caption="category"``, for rebuild comparisons). The scenario is in ``meta/teleop/episodes.jsonl``.
- **Labels:** ``safety_ranges.json`` at the root, world-models' format (exactly ``source``, ``notes``, ``unsafe_ranges``),
  beside the three files above, which are kept.
- **Sidecars** under ``meta/teleop/`` (``episodes.jsonl``, ``scenes.jsonl``, ``contacts.jsonl``, ``frames.parquet``,
  ``report.json``), and a build can split one batch into several datasets by layout (:func:`build_split`).

No LeRobot or pandas here but in :func:`write_labels`'s parquet, so the tests run with plain Python.
"""

from __future__ import annotations

import contextlib
import json
import math
import os
import shutil
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .jlog import JlogError, items, read_jlog
from .takes import Camera, Frame, Take, TakeError

REACH, STOP, RECOVER, COLLIDE = "reach_no_obstacle", "stop_before_collision", "collide_and_recover", "collide_with_obstacle"
CATEGORIES = (REACH, STOP, RECOVER, COLLIDE)
MIXED = "mixed"  # docs/adr/0038 decision 1: one script, physics decides; checked only for being broken
SCENARIOS = CATEGORIES + (MIXED,)
TABLE = 0  # WC's object 0: the table, never a contact for the labels or the checks (docs/adr/0038 decision 4)
FROZEN_SECONDS = 1.0  # a mixed take whose episode repeats one render this long is broken (the twin's are single repeats)
SOURCE = "physics"
GROUP_SUFFIX = "-safety"  # the datasets' folder, <out>/<robot>-<source>-safety/<batch>/: the watcher leaves it alone
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


def contacts(path: str, table: bool = False, obstacles: bool = True) -> List[Contact]:
    """The take's contact stretches from its ``WC`` records, by their ``sensed`` times: the arm touching any object but the
    table (object 0, docs/adr/0038 decision 4), which is never a contact for the labels or the checks. ``table`` adds the
    table's pairs, and ``obstacles=False`` leaves the objects out (the table's own stretches). A ``WC`` lists every pair
    touching then, so one naming only pairs left out ends a stretch."""
    host = read_jlog(os.path.join(path, "robothost.jlog"), "robothost")
    rows = sorted(((int(r[1]), r[2] if len(r) > 2 else "") for r in host.tag("WC")), key=lambda x: x[0])
    out, start, touched = [], None, set()
    for sensed, text in rows:
        pairs = [it for it in items(text) if len(it) >= 3 and (table if int(it[2]) == TABLE else obstacles)]
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
    """How far the object tipped from ``a`` to ``b``, degrees: the angle between the vertical as each pose's own frame sees it
    (its rotation's third row). For an object that starts upright that is the angle its z axis turned. A turn about the
    vertical alone is no tip: an object a layout lays on its side (docs/adr/0037 amendment 6) has a level z axis, which a push
    that only spins it on the table turns (measured with the z axes, a 60-degree spin was a 60-degree tip)."""
    def up(q):
        w, x, y, z = q[3:7]
        return (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    ua, ub = up(a), up(b)
    dot = max(-1.0, min(1.0, sum(p * q for p, q in zip(ua, ub))))
    return math.degrees(math.acos(dot))


def _at(poses: List[Tuple[int, Tuple[float, ...]]], t: int) -> Optional[Tuple[float, ...]]:
    before = [p for s, p in poses if s <= t]
    return before[-1] if before else (poses[0][1] if poses else None)


def first_touches(stretches: Sequence[Contact]) -> Dict[int, int]:
    """Each touched object's first touch: the start of the first stretch that names it."""
    first: Dict[int, int] = {}
    for c in sorted(stretches, key=lambda c: c.start):
        for o in c.objects:
            first.setdefault(o, c.start)
    return first


@dataclass(frozen=True)
class Verdict:
    ok: bool
    reason: str
    unsafe_frames: int
    touched: Tuple[int, ...]
    contacts: int = 0  # stretches of contact in the episode: one a touch or a knock


def check(take: Take, labels: Sequence[int], stretches: Sequence[Contact],
          poses: Dict[int, List[Tuple[int, Tuple[float, ...]]]]) -> Verdict:
    """Whether the take is what its category says (docs/adr/0036 amendment 1, decision 3), with any number of events
    (amendment 5): no contact for reach and stop; for recover every touch backed off from (each stretch of contact ends, and
    the episode does not end touching) and no touched object tipped; for a knock at least one touched object tipped or moved.
    Each touched object is measured from its own first touch to the episode's end. A ``mixed`` take (docs/adr/0038) has no
    category to meet and always passes here, with what it touched and knocked as the reason; :func:`broken` is its check."""
    category = take.scenario
    unsafe = sum(labels)
    half = 1_000_000_000 // (2 * take.fps)  # as frame_labels: a frame's period reaches half a frame past its time
    t0, t1 = int(take.frames[0].timing["sensed"]) - half, int(take.frames[-1].timing["sensed"]) + half
    inside = [c for c in stretches if c.end >= t0 and c.start <= t1]
    touched = tuple(sorted({o for c in inside for o in c.objects}))
    first = first_touches(inside)
    n = len(inside)

    def moved(obj: int) -> Tuple[float, float]:
        p = poses.get(obj, [])
        a, b = _at(p, first[obj]), _at(p, t1)
        if a is None or b is None:
            return 0.0, 0.0
        return _tilt_degrees(a, b), math.dist(a[:3], b[:3])

    if category == MIXED:  # docs/adr/0038 decision 1: no category to meet; broken() sets a broken take aside
        over = [o for o in touched if moved(o)[0] >= TIPPED_DEGREES or moved(o)[1] >= MOVED_METERS]
        what = f"knocked {over}" if over else (f"touched {list(touched)}" if touched else "no contact")
        return Verdict(True, f"{what} ({n} contact{'s' if n != 1 else ''})", unsafe, touched, n)
    if category not in CATEGORIES:
        return Verdict(False, f"{category!r} is not a safety category", unsafe, touched, n)
    if category in (REACH, STOP):
        if unsafe or inside:
            return Verdict(False, f"{category} touched object(s) {list(touched)} ({unsafe} unsafe frames)", unsafe, touched, n)
        return Verdict(True, "no contact", 0, (), 0)
    if not unsafe:
        return Verdict(False, f"{category} never touched an object", 0, (), n)
    if category == RECOVER:
        held = [c for c in inside if c.end == OPEN_END]
        if labels[-1] or held:
            return Verdict(False, "collide_and_recover was still touching at the episode's end: no recovery", unsafe, touched, n)
        for obj in touched:
            tilt, dist = moved(obj)
            if tilt > UPRIGHT_DEGREES:
                return Verdict(False, f"collide_and_recover tipped object {obj} by {tilt:.0f} degrees", unsafe, touched, n)
        return Verdict(True, f"touched {list(touched)} and backed off ({n} touch{'es' if n != 1 else ''})", unsafe, touched, n)
    knocked = [(obj, moved(obj)) for obj in touched]
    if not any(tilt >= TIPPED_DEGREES or dist >= MOVED_METERS for _, (tilt, dist) in knocked):
        worst = max(knocked, key=lambda k: (k[1][0], k[1][1]))
        return Verdict(False, f"collide_with_obstacle touched {list(touched)} but tipped none (object {worst[0]}: "
                              f"{worst[1][0]:.0f} degrees, {worst[1][1] * 100:.1f} cm)", unsafe, touched, n)
    over = [o for o, (tl, d) in knocked if tl >= TIPPED_DEGREES or d >= MOVED_METERS]
    return Verdict(True, f"knocked {over} ({n} contact{'s' if n != 1 else ''})", unsafe, touched, n)


# joint-drive's exit codes that mean the take was cut short (JointDriveCommand: 2 a stall, 3 the joint-state stream failed;
# 130 stopped by hand). 0 is a clean finish, 4 a take RobotHost stopped itself (its episode still ends at the last command).
DRIVE_EXITS = {2: "a joint stalled and joint-drive's watchdog stopped the drive (exit 2)",
               3: "the joint-state stream failed and joint-drive's watchdog stopped the drive (exit 3)",
               130: "joint-drive was stopped by hand (exit 130)"}


def scenario_of(path: str) -> Optional[str]:
    """A take's scenario, from its session.json, else its folder's name (``<root>/<scenario>/<take>``); None if neither."""
    try:
        with open(os.path.join(path, "session.json"), encoding="utf-8") as f:
            name = (json.load(f).get("scenario") or {}).get("name")
        if name:
            return str(name)
    except (OSError, ValueError, AttributeError):
        pass
    folder = os.path.basename(os.path.dirname(os.path.abspath(path)))
    return folder or None


def broken(path: str, take: Optional[Take], drive_exit: Optional[int] = None, error: Optional[BaseException] = None) -> List[str]:
    """Why a ``mixed`` take cannot be an episode (docs/adr/0038 decision 1: the only reasons one is set aside), empty when
    it can:

    - it cannot be read as an episode (``error``, from ``read_take``): no scenario, a ``.jlog`` cut off or failing its trailer,
      no accepted command, no camera recording colour, or no frame with its joints between the first and last command;
    - joint-drive cut it short (``drive_exit``, :data:`DRIVE_EXITS`): a stall, a failed joint-state stream, or by hand;
    - its frames ended early (an ``ER`` record: a nearly full disk, an I/O error);
    - a writer dropped records (``robothost.jlog`` or the frames log's trailer): contacts or poses may be missing;
    - no object poses (``WO``): not a take of a scene, so nothing can say what happened;
    - its episode repeats one render for :data:`FROZEN_SECONDS` or more (the renderer stalled; the twin's repeats are single
      frames, 1.3% of them, measured over 177 episodes on 2026-11-22)."""
    reasons = []
    if error is not None:
        reasons.append(f"cannot be read as an episode ({error})")
    if drive_exit is not None and drive_exit not in (0, 1, 4):
        reasons.append(DRIVE_EXITS.get(drive_exit, f"joint-drive exited {drive_exit}"))
    for writer in ("robothost", "isaac", "camera"):
        p = os.path.join(path, writer + ".jlog")
        if not os.path.exists(p):
            continue
        try:
            log = read_jlog(p, writer)
        except (JlogError, OSError, ValueError) as e:
            if error is None:
                reasons.append(f"{writer}.jlog cannot be read ({e})")
            continue
        if log.dropped:
            reasons.append(f"{writer}.jlog dropped {log.dropped} record(s): its contacts or poses may be missing")
        for r in log.tag("ER"):
            reasons.append(f"its frames ended early ({writer}.jlog ER: {r[1] if len(r) > 1 else '?'})")
        if writer == "robothost" and take is not None and not log.tag("WO"):
            reasons.append("no object poses (WO): not a take of a scene")
    if take is not None:
        run = longest = 0
        for f in take.frames:
            run = run + 1 if f.timing.get("filled") else 0
            longest = max(longest, run)
        if longest and longest / take.fps >= FROZEN_SECONDS:
            reasons.append(f"its frames repeat one render for {longest / take.fps:.1f} s (the renderer stalled)")
    return reasons


def planned_events(path: str) -> List[Dict]:
    """The events the script planned (docs/adr/0036 amendment 5), from the take's ``SE`` records, in order: ``index``,
    ``drawn``, ``kind``, ``object``, and their times on the take's clock (ns): ``approach`` (the record's ``t``: the arm left
    free movement for the object), ``final``, ``contact`` (a stop: its closest approach; None when none was planned), ``end``
    and ``clear``. Takes recorded before then have none."""
    host = read_jlog(os.path.join(path, "robothost.jlog"), "robothost")
    out = []
    for r in host.tag("SE"):
        t = int(r[0])

        def at(seconds: str) -> Optional[int]:
            v = float(seconds)
            return None if math.isnan(v) else t + round(v * 1e9)

        out.append({"index": int(r[1]), "drawn": int(r[2]), "kind": r[3], "object": int(r[4]), "approach": t,
                    "final": at(r[5]), "contact": at(r[6]), "end": at(r[7]), "clear": at(r[8])})
    return sorted(out, key=lambda e: e["index"])


def event_timing(take: Take, stretches: Sequence[Contact], planned: Sequence[Dict]) -> List[Dict]:
    """Each planned event against what physics did, seconds from the episode's first frame: when its contact was planned,
    and when the first stretch of contact between its approach and the next event's began (None: no contact then), with
    the objects that stretch touched. For checking that events land where they were planned, from the take alone."""
    start = int(take.frames[0].timing["sensed"])
    rows = []
    for i, e in enumerate(planned):
        until = planned[i + 1]["approach"] if i + 1 < len(planned) else OPEN_END
        hit = next((c for c in sorted(stretches, key=lambda c: c.start) if e["approach"] <= c.start < until), None)
        rows.append({"index": e["index"], "kind": e["kind"], "object": e["object"],
                     "planned": None if e["contact"] is None else round((e["contact"] - start) / 1e9, 3),
                     "contact": None if hit is None else round((hit.start - start) / 1e9, 3),
                     "touched": [] if hit is None else list(hit.objects)})
    return rows


def knocks(path: str) -> List[Dict]:
    """Every object the take touched (docs/adr/0037 P4: what can be knocked over), with what it was: its size factor (the
    renderer's first ``SZ``), its mass at that size (the scene file's mass times the cube of the factor, as Isaac's bodies
    have it), how far it tipped and moved from its first touch to the take's end, and whether that is knocked over by
    :func:`check`'s rule (tipped at least ``TIPPED_DEGREES`` or moved at least ``MOVED_METERS``)."""
    with open(os.path.join(path, "scene.json"), encoding="utf-8") as f:
        scene = {o["id"]: o for o in json.load(f)["objects"]}
    scales = sizes(read_jlog(os.path.join(path, "isaac.jlog"), "isaac").tag("SZ"))
    stretches = contacts(path)
    poses = object_poses(path)
    end = max((p[-1][0] for p in poses.values() if p), default=0)
    first = first_touches(stretches)  # each object from its own first touch: a take may knock several (amendment 5)
    out = []
    for obj, t in sorted(first.items()):
        p = poses.get(obj, [])
        a, b = _at(p, t), _at(p, end)
        tilt, dist = (_tilt_degrees(a, b), math.dist(a[:3], b[:3])) if a is not None and b is not None else (0.0, 0.0)
        o = scene.get(obj, {})
        scale = scales.get(obj, 1.0)
        out.append({"take": os.path.abspath(path), "object": obj, "name": o.get("name", "?"), "kind": o.get("kind", "?"),
                    "shape": o.get("shape", "?"), "scale": scale, "massKg": round(float(o.get("mass", 0.0)) * scale ** 3, 4),
                    "tiltDeg": round(tilt, 1), "movedCm": round(dist * 100, 2),
                    "knocked": bool(tilt >= TIPPED_DEGREES or dist >= MOVED_METERS)})
    return out


def sizes(sz_records: Sequence[Sequence[str]]) -> Dict[int, float]:
    """Each object's size factor from the renderer's first ``SZ`` record (fields after the tag: revision, ``id:factor;...``)."""
    if not sz_records or len(sz_records[0]) < 2:
        return {}
    return {int(it[0]): float(it[1]) for it in items(sz_records[0][1]) if len(it) >= 2}


MASS_BINS_KG = (0.03, 0.06, 0.1, 0.2, 0.4, 0.8)


def knock_summary(rows: Sequence[Dict]) -> List[str]:
    """How often a touched movable object was knocked over, by its mass at its size (bins up to each of
    :data:`MASS_BINS_KG`, then above the last)."""
    out = []
    edges = (0.0,) + MASS_BINS_KG + (float("inf"),)
    movable = [r for r in rows if r["kind"] == "movable"]
    for lo, hi in zip(edges, edges[1:]):
        b = [r for r in movable if lo < r["massKg"] <= hi]
        if b:
            k = sum(r["knocked"] for r in b)
            names = sorted({r["name"] for r in b})
            out.append(f"{lo * 1000:5.0f}-{hi * 1000:5.0f} g: {k}/{len(b)} knocked ({100 * k / len(b):.0f}%): {', '.join(names)}")
    fixed = [r for r in rows if r["kind"] != "movable"]
    if fixed:
        out.append(f"fixed: {sum(r['knocked'] for r in fixed)}/{len(fixed)} knocked")
    return out


def as_safety(take: Take, caption: str = "neutral") -> Take:
    """The take as the safety dataset writes it: its ``wrist`` camera named ``front`` (his tools read that), and its task
    string ``move the arm`` (docs/adr/0038 decision 6: a caption naming the scenario gave the outcome away), or with
    ``caption="category"`` its scenario, as datasets built before 2026-11-22 have it (his ``label_safety.py`` reads an
    episode's category from it)."""
    from .sidecars import Options

    if not any(c.name == FRONT for c in take.cameras):
        raise ValueError(f"{take.path}: no {FRONT} camera to be the front one")
    rename = {FRONT: "front"}
    cameras = [Camera(rename.get(c.name, c.name), c.width, c.height, c.sidecar) for c in take.cameras]
    frames = [Frame({rename.get(k, k): v for k, v in f.parts.items()}, f.state, f.action, f.timing) for f in take.frames]
    return Take(take.path, take.session_id, take.robot, take.source, take.scenario, Options(caption=caption).task(take.scenario),
                take.names, take.fps, cameras, frames)


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
    """His ``safety_events.jsonl`` markers for one episode: ``contact`` at the first frame of each unsafe range, and for
    ``collide_and_recover`` and ``mixed`` (docs/adr/0038) ``clear`` at the first safe frame after it. With one event a take (until 2026-11-21) that was one
    pair; with several (docs/adr/0036 amendment 5), each range has its own, so a lead window before a contact never spans the
    free movement between two events."""
    out = []
    for start, end in ranges(labels):
        out.append({"episode": episode, "kind": "contact", "frame_in_episode": start, "source": SOURCE})
        if category in (RECOVER, MIXED) and end + 1 < len(labels):
            out.append({"episode": episode, "kind": "clear", "frame_in_episode": end + 1, "source": SOURCE})
    return out


def label_rows(episode: int, category: str, labels: Sequence[int]) -> List[dict]:
    """``meta/safety_labels.parquet``'s rows for one episode, as ``label_safety.py`` writes them."""
    return [{"episode_index": episode, "frame_index": i, "label": int(v), "category": category,
             "effective_category": category, "overridden": False, "source": SOURCE} for i, v in enumerate(labels)]


NOTES = ("Physics labels from the JetRover's Isaac twin (teleoperation docs/adr/0036 amendment 1, docs/adr/0038 decision 6): "
         "a frame is unsafe when any arm link touched a scene object other than the table at any moment of its frame period "
         "(its time, half a frame either side); a touch is a contact point within 1 mm or an impulse (docs/adr/0036 amendment 4). "
         "Table contacts are not unsafe: meta/teleop/frames.parquet (in_contact_table) and contacts.jsonl list them.")
NOTES_TABLE = NOTES.replace("other than the table ", "or the table ").replace(
    "Table contacts are not unsafe: meta/teleop/frames.parquet (in_contact_table) and contacts.jsonl list them.",
    "Built with table contacts counted as unsafe (--table-unsafe).")
RANGES_KEYS = ("source", "notes", "unsafe_ranges")  # world-models' loader (wm/labeling/ranges.py) rejects any other key


def write_labels(root: str, episodes: Sequence[Tuple[int, str, Sequence[int]]], notes: str = NOTES) -> None:
    """Writes the label files at the dataset ``root`` for ``episodes`` ((index, category, frame labels) each), replacing any
    there: world-models' ``safety_ranges.json`` (docs/adr/0038 decision 6: exactly ``source``, ``notes`` and
    ``unsafe_ranges``), and his three, whose ranges are the same."""
    manual = {str(ep): ranges(labels) for ep, _, labels in episodes if any(labels)}
    with open(os.path.join(root, "safety_ranges.json"), "w", encoding="utf-8") as f:
        json.dump(dict(zip(RANGES_KEYS, (SOURCE, notes, manual))), f, indent=2)
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
                   "touched": list(verdict.touched), "contacts": verdict.contacts}, f, indent=2)
    return dest


def _episode(take: Take, options):
    """``sidecars.episode``, a failure naming its take (a worker's traceback stays in the worker)."""
    from . import sidecars

    try:
        return sidecars.episode(take, options)
    except Exception as e:  # noqa: BLE001  (re-raised with the take's path)
        raise RuntimeError(f"{take.path}: its sidecars could not be made ({type(e).__name__}: {e})") from e


def episodes_of(takes: Sequence[Take], options=None, jobs: int = 1, executor=None) -> list:
    """Each take as a safety episode (``sidecars.episode``: its labels and sidecar lines), in order. ``jobs`` above 1: that
    many spawned processes read the takes side by side (each take's records, and its clearance when there is geometry), or
    ``executor`` does (tests)."""
    from . import sidecars

    options = options or sidecars.Options()
    if executor is not None and len(takes) > 1:
        return list(executor.map(_episode, takes, [options] * len(takes)))
    if jobs > 1 and len(takes) > 1:
        import multiprocessing
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=min(jobs, len(takes)), mp_context=multiprocessing.get_context("spawn")) as pool:
            return list(pool.map(_episode, takes, [options] * len(takes)))
    return [_episode(t, options) for t in takes]


def build(writer, root: str, takes: Sequence[Take], log=print, jobs: int = 1, executor=None, options=None, episodes=None,
          split_of: Optional[Tuple[int, int]] = None) -> List[Tuple[int, str, List[int]]]:
    """Writes ``takes`` (already checked) as one safety dataset at ``root``: each an episode, in order, then the label files
    and the sidecars (docs/adr/0038 decision 6). A dataset already there is refused: a batch's dataset is written once, from
    its kept takes. Returns (episode, scenario, frame labels) for each.

    ``jobs`` above 1 (docs/adr/0036 amendment 6): that many worker processes decode each take's frames and encode its videos
    ahead (``writer.prepare_job``), while this process appends the episodes in order with LeRobot as before; the same files
    either way. ``executor`` replaces the workers (tests). ``options`` (``sidecars.Options``): the caption, whether table
    contacts are unsafe, the clearance geometry, a stop model. ``episodes``: the takes' ``sidecars.episode`` already made
    (:func:`build_split`); ``split_of``: (part, parts) for its report."""
    from . import sidecars
    from .writer import append_take, prepared_in_order

    options = options or sidecars.Options()
    if os.path.exists(root):
        raise FileExistsError(f"{root} exists: a safety dataset is written once (rebuild it into a new folder)")
    infos = list(episodes) if episodes is not None else episodes_of(takes, options, jobs, executor)
    shown = [as_safety(take, options.caption) for take in takes]
    if jobs > 1 and hasattr(writer, "prepare_job"):
        stream = prepared_in_order(writer, root, shown, jobs, executor=executor)
    else:
        stream = ((t, None) for t in shown)
    written, sides = [], []
    with contextlib.closing(stream):
        for take, info, (safe, prepared) in zip(takes, infos, stream):
            labels = info.labels
            line = append_take(writer, root, safe, repo=f"local/{take.dataset_group}-safety_{os.path.basename(root)}",
                               prepared=prepared)
            written.append((int(line["episode"]), take.scenario, labels))
            sides.append((int(line["episode"]), info))
            log(f"episode {line['episode']}: {take.scenario}, {len(labels)} frames, {sum(labels)} unsafe ({os.path.basename(take.path)})"
                f", {info.line['outcome']}")
    write_labels(root, written, NOTES_TABLE if options.table_unsafe else NOTES)
    rep = sidecars.write(root, sides, options, split_of)
    for ep, info in sides:
        if info.flags:
            log(f"[timing] episode {ep}: {'; '.join(info.flags)}")
    log("[mix] " + sidecars.mix(os.path.basename(root), rep))
    return written


def split_roots(out: str, parts: int) -> List[str]:
    """The datasets a split build writes for ``out``: ``<out>_<k>of<parts>``."""
    return [f"{out}_{k}of{parts}" for k in range(1, parts + 1)]


def build_split(writer, out: str, takes: Sequence[Take], parts: int, log=print, jobs: int = 1, executor=None,
                options=None) -> List[str]:
    """Writes ``takes`` as ``parts`` smaller safety datasets (docs/adr/0038 decision 6, items 20 and 21), each by
    :func:`build` at ``<out>_<k>of<parts>``, every take in exactly one. Takes that share a replay group (``RP``), a layout
    hash, or a scene and layout seed stay in one dataset (``sidecars.split``), so no layout and no replay is on two sides.
    Each dataset's ``meta/teleop/report.json`` holds its scenario, outcome and contact counts, and the log its mix. Returns
    the datasets' folders. None is written unless every take finds its place."""
    from . import sidecars

    options = options or sidecars.Options()
    if parts < 1:
        raise ValueError("a split build writes at least one dataset")
    roots = split_roots(out, parts)
    there = [r for r in roots if os.path.exists(r)]
    if there:
        raise FileExistsError(f"{there[0]} exists: a safety dataset is written once (rebuild it into a new folder)")
    infos = episodes_of(takes, options, jobs, executor)
    bins = sidecars.split([e.line for e in infos], parts)
    for k, (root, members) in enumerate(zip(roots, bins), start=1):
        log(f"[split] {os.path.basename(root)}: {len(members)} episode(s)")
        build(writer, root, [takes[i] for i in members], log, jobs, executor, options, [infos[i] for i in members], (k, parts))
    return roots


# What a rebuild of a dataset's first episodes holds that is not file for file the reference's (compare_datasets):
_TELEOP = os.path.join("meta", "teleop")
_LINES = {"capture_timing.jsonl": "episode_saved_so_far", "safety_events.jsonl": "episode", "raw_ros_events.jsonl": None,
          os.path.join(_TELEOP, "takes.jsonl"): "episode", os.path.join(_TELEOP, "episodes.jsonl"): "episode",
          os.path.join(_TELEOP, "scenes.jsonl"): "episode", os.path.join(_TELEOP, "contacts.jsonl"): "episode"}  # one line a record
_TOTALS = {os.path.join("meta", "info.json"), os.path.join("meta", "stats.json"), os.path.join("meta", "tasks.parquet"),
           os.path.join(_TELEOP, "report.json")}
_PARQUETS = {os.path.join("meta", "safety_labels.parquet"), os.path.join(_TELEOP, "frames.parquet")}  # rows by episode_index
# docs/adr/0038's files: a reference built before them has none, and is compared without them.
_ADR38 = {"safety_ranges.json", os.path.join(_TELEOP, "episodes.jsonl"), os.path.join(_TELEOP, "scenes.jsonl"),
          os.path.join(_TELEOP, "contacts.jsonl"), os.path.join(_TELEOP, "frames.parquet"), os.path.join(_TELEOP, "report.json"),
          os.path.join(_TELEOP, "unavoidable_ranges.json")}


def _first_ranges(ranges_by_episode: Dict[str, list], n: int) -> Dict[str, list]:
    return {k: v for k, v in ranges_by_episode.items() if int(k) < n}


def compare_datasets(reference: str, rebuilt: str, notes: Optional[List[str]] = None) -> List[str]:
    """How ``rebuilt``, a safety dataset written from the first takes of ``reference``'s ledger (all of them, or fewer),
    differs from it: every file of the rebuilt one, byte for byte, but ``takes.jsonl``'s ``convertedUtc`` (when it was
    written) and JSON files as their values (key order aside); with fewer episodes the files that run on through every
    episode (``_LINES``, the label files, the parquet rows) as the reference's lines, rows and ranges of those episodes, and
    the totals (``_TOTALS``) not at all. A ``frames.parquet`` column only one of them has must be empty there. A reference
    from before docs/adr/0038 has none of its files (``_ADR38``): those are left out, and said so in ``notes``. Empty when
    they are the same: how a build's speed-up is checked to change nothing (docs/adr/0036 amendment 6). Rebuild an old
    reference with ``--caption category``: its captions were its categories."""
    from .writer import recorded_episodes

    na, nb = recorded_episodes(reference), recorded_episodes(rebuilt)
    if na is None or nb is None:
        return [f"{reference if na is None else rebuilt}: no LeRobot dataset there"]
    if nb > na:
        return [f"{rebuilt} has {nb} episodes, more than {reference}'s {na}"]
    old = not os.path.exists(os.path.join(reference, "safety_ranges.json"))
    out, skipped = [], []
    for folder, _, names in os.walk(rebuilt):
        for name in names:
            path = os.path.join(folder, name)
            rel = os.path.relpath(path, rebuilt)
            ref = os.path.join(reference, rel)
            if old and rel in _ADR38:
                skipped.append(rel)
            elif not os.path.exists(ref):
                out.append(f"{rel}: not in the reference")
            elif rel in _LINES:
                field = _LINES[rel]

                def lines(p):
                    with open(p, encoding="utf-8") as f:
                        rows = [json.loads(x) for x in f if x.strip()]
                    for r in rows:
                        r.pop("convertedUtc", None)
                    return rows
                mine, theirs = lines(path), lines(ref)
                if field is not None:
                    theirs = [r for r in theirs if int(r[field]) < nb]
                elif nb < na:
                    theirs = theirs[:len(mine)]  # no episode field: the reference's first lines
                if mine != theirs:
                    first = next((i for i, (x, y) in enumerate(zip(mine, theirs)) if x != y), min(len(mine), len(theirs)))
                    out.append(f"{rel}: differs from line {first + 1} ({len(mine)} lines, the reference's {len(theirs)})")
            elif rel == "manual_safety_labels.json":
                with open(path, encoding="utf-8") as f, open(ref, encoding="utf-8") as g:
                    mine, theirs = json.load(f), _first_ranges(json.load(g), nb)
                if mine != theirs:
                    out.append(f"{rel}: differs (episodes {sorted(int(k) for k in set(mine) | set(theirs) if mine.get(k) != theirs.get(k))})")
            elif rel in ("safety_ranges.json", os.path.join(_TELEOP, "unavoidable_ranges.json")):
                key = "unsafe_ranges" if rel == "safety_ranges.json" else "unavoidable_ranges"
                with open(path, encoding="utf-8") as f, open(ref, encoding="utf-8") as g:
                    mine, theirs = json.load(f), json.load(g)
                theirs[key] = _first_ranges(theirs.get(key, {}), nb)
                if "unmeasured" in theirs:
                    theirs["unmeasured"] = [e for e in theirs["unmeasured"] if e < nb]
                if mine != theirs:
                    out.append(f"{rel}: differs")
            elif rel in _PARQUETS:
                import pandas as pd

                mine, theirs = pd.read_parquet(path), pd.read_parquet(ref)
                theirs = theirs[theirs["episode_index"] < nb].reset_index(drop=True)
                lone = [c for c in mine.columns if c not in theirs.columns and mine[c].notna().any()]
                lone += [c for c in theirs.columns if c not in mine.columns and theirs[c].notna().any()]
                common = [c for c in theirs.columns if c in mine.columns]
                if lone or not mine[common].equals(theirs[common]):
                    out.append(f"{rel}: differs ({len(mine)} rows, the reference's {len(theirs)} for those episodes"
                               + (f"; columns {sorted(lone)} only on one side" if lone else "") + ")")
            elif rel in _TOTALS and nb < na:
                continue
            elif rel.endswith(".json"):  # as JSON: LeRobot's stats.json orders its keys by a set's, so by the run's hash seed
                with open(path, encoding="utf-8") as f, open(ref, encoding="utf-8") as g:
                    if json.load(f) != json.load(g):
                        out.append(f"{rel}: differs")
            else:
                with open(path, "rb") as f, open(ref, "rb") as g:
                    if f.read() != g.read():
                        out.append(f"{rel}: differs")
    if skipped and notes is not None:
        notes.append(f"the reference predates docs/adr/0038: left out {', '.join(sorted(skipped))}")
    return sorted(out)


__all__ = ["CATEGORIES", "MIXED", "SCENARIOS", "compare_datasets", "Contact", "Verdict", "as_safety", "broken", "build",
           "build_split", "check", "contacts", "episodes_of", "event_timing", "events", "first_touches", "frame_labels",
           "label_rows", "object_poses", "planned_events", "ranges", "scenario_of", "set_aside", "split_roots", "write_labels"]
