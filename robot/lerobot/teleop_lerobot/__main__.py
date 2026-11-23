"""``python -m teleop_lerobot <command>`` (docs/adr/0036):

- ``watch``: keep every scenario's dataset up to date, for good (``--once`` for one pass);
- ``inspect TAKE``: what a take would become, as JSON (no LeRobot needed);
- ``rebuild DATASET``: rebuild one dataset from its kept takes;
- ``safety-check TAKE...``: each safety take's labels and verdict against its category, as JSON lines, with its planned
  events (``SE``) against the contacts physics reported (``--set-aside`` moves a failing one to ``_discarded``; no LeRobot
  needed). A ``mixed`` take (docs/adr/0038) fails only when it is broken (``--drive-exit``: joint-drive's exit code), and
  its line says what happened (its outcome);
- ``safety-build --out DATASET TAKE...``: one safety dataset from checked takes, in the lab's label format
  (docs/adr/0036 amendment 1) and world-models' (``safety_ranges.json``), with the sidecars of docs/adr/0038 decision 6,
  ``--jobs`` worker processes preparing episodes ahead (amendment 6), ``--split K`` for K smaller datasets by layout, the
  table counted as unsafe unless ``--table-safe``, and unavoidable-contact ranges for the twin's three stop times unless
  ``--stop-time-s`` gives one or ``--no-unavoidable`` none;
- ``safety-compare REFERENCE REBUILT``: whether a safety dataset rebuilt from a reference's first takes holds the same
  files (amendment 6; no LeRobot needed);
- ``safety-options DATASET``: the ``safety-build`` options that rebuild a safety dataset as it was built, from its
  ``report.json`` (``just isaac-safety-build-check``; no LeRobot needed);
- ``safety-report DATASET...``: what a set of safety datasets holds (a split build's parts, or a whole train, val and test
  set), and any fault in their labels or sidecars, or a replay group, layout hash or layout seed in more than one of them
  (docs/adr/0038; the twin check);
- ``knock-report TAKE...``: every object each take touched, its mass at its size and whether it was knocked over, as JSON
  lines, then a summary by mass (docs/adr/0037 P4; no LeRobot needed).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

from .sidecars import GEOMETRY_DIR
from .takes import read_take


def _log(message: str) -> None:
    print(time.strftime("%Y-%m-%dT%H:%M:%SZ ", time.gmtime()) + message, flush=True)


def default_jobs() -> int:
    """safety-build's workers: a quarter of the cores (each encoder runs threads of its own), at most 8 (the writer, one
    process, keeps up with about that many; docs/adr/0036 amendment 6)."""
    return max(1, min(8, (os.cpu_count() or 1) // 4))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="teleop_lerobot")
    sub = p.add_subparsers(dest="command", required=True)
    for name in ("watch", "rebuild"):
        s = sub.add_parser(name)
        s.add_argument("--root", action="append", required=True, help="a scenario root (repeatable)")
        s.add_argument("--out", required=True, help="where the datasets go: <out>/<robot>-<source>/<scenario>/")
        s.add_argument("--fps", type=int, default=15)
        s.add_argument("--settle", type=float, default=60.0, help="seconds a take must be complete before converting")
        s.add_argument("--vcodec", default="libsvtav1")
        if name == "watch":
            s.add_argument("--every", type=float, default=10.0)
            s.add_argument("--once", action="store_true")
            s.add_argument("--pull-robot", default="", help="copy the real robot's complete takes into this scenario root "
                                                          "(pull.py); it should also be a --root")
            s.add_argument("--robot-hosts-file", default="", help="the robot's addresses, one per line (just lerobot-robot-key)")
            s.add_argument("--robot-mdns", default="jetrover-sinrg.local")
            s.add_argument("--robot-key", default="", help="the ssh key the robot accepts from this machine")
        else:
            s.add_argument("dataset", help="<robot>-<source>/<scenario>, under --out")
    s = sub.add_parser("inspect")
    s.add_argument("take")
    s.add_argument("--fps", type=int, default=15)
    s = sub.add_parser("safety-check")
    s.add_argument("takes", nargs="+")
    s.add_argument("--fps", type=int, default=15)
    s.add_argument("--set-aside", action="store_true", help="move a take that fails its category to _discarded")
    s.add_argument("--drive-exit", type=int, default=None,
                   help="joint-drive's exit code for the take: a mixed take it cut short (2, 3, 130) is broken (docs/adr/0038)")
    s.add_argument("--geometry-dir", default="", help="the twin's collision geometry, to tell a near miss from no contact "
                                                      "in a mixed take's outcome (default: none, the outcome says unknown)")
    s = sub.add_parser("knock-report")
    s.add_argument("takes", nargs="*")
    s = sub.add_parser("safety-build")
    s.add_argument("takes", nargs="+")
    s.add_argument("--out", required=True, help="the new dataset's folder, e.g. ~/data/lerobot/jetrover-isaac-safety/jetrover_<UTC>_<id>")
    s.add_argument("--fps", type=int, default=15)
    s.add_argument("--vcodec", default="libsvtav1")
    s.add_argument("--crf", type=int, default=None, help="the videos' CRF (default: writer.SAFETY_CRF)")
    s.add_argument("--preset", type=int, default=None, help="the AV1 encoder's preset (default: writer.SAFETY_PRESET)")
    s.add_argument("--jobs", type=int, default=default_jobs(),
                   help="worker processes decoding and encoding episodes ahead of the writer (1: all in this process, as "
                        "before); the files are the same either way (docs/adr/0036 amendment 6). Default: a quarter of the "
                        "cores, at most 8")
    s.add_argument("--caption", choices=("neutral", "category"), default="neutral",
                   help="every episode's task: 'move the arm' (docs/adr/0038), or its scenario's name as before 2026-11-22 "
                        "(to compare a rebuild with an older dataset)")
    s.add_argument("--table-safe", action="store_true",
                   help="leave the arm touching the table (WC object 0) out of the labels, as builds did until 2026-11-23 "
                        "(by default it is unsafe: on the real robot a hard table hit is a collision)")
    s.add_argument("--geometry-dir", default=GEOMETRY_DIR,
                   help="the twin's collision geometry for each frame's clearance (workstream C's export; '' for none: "
                        "clearance NaN). Default: %(default)s")
    s.add_argument("--split", type=int, default=1, metavar="K",
                   help="write K datasets, <out>_<k>of<K>, split by layout (a replay group and a layout never on two sides)")
    stops = s.add_mutually_exclusive_group()
    stops.add_argument("--stop-time-s", type=float, default=None, metavar="SECONDS",
                      help="meta/teleop/unavoidable_ranges.json (docs/adr/0038 item 17) for this one stop time, named 'given', "
                           "instead of the default three: after a stop the arm goes on for this many seconds of its speed "
                           "(travel = speed x time)")
    stops.add_argument("--no-unavoidable", action="store_true",
                      help="no meta/teleop/unavoidable_ranges.json, as builds before 2026-11-23 (by default it holds ranges for "
                           "the twin's fast, median and slow stops, 0.17, 0.245 and 0.31 s: twin stop-latency 2026-11-23, "
                           "provisional until the real arm is measured)")
    s = sub.add_parser("safety-report")
    s.add_argument("datasets", nargs="+", help="safety datasets: every part of a split build, or every dataset of a train, val "
                                                "and test set (a replay group or a layout in two of them is a fault)")
    s = sub.add_parser("safety-compare")
    s.add_argument("reference", help="a safety dataset")
    s.add_argument("rebuilt", help="one written from the first takes of the reference's meta/teleop/takes.jsonl")
    s = sub.add_parser("safety-options")
    s.add_argument("dataset", help="a safety dataset")
    a = p.parse_args(argv)

    if a.command == "safety-options":
        from . import safety

        print(" ".join(safety.rebuild_options(os.path.expanduser(a.dataset))))
        return 0

    if a.command == "safety-compare":
        from . import safety

        notes = []
        diffs = safety.compare_datasets(os.path.expanduser(a.reference), os.path.expanduser(a.rebuilt), notes)
        for n in notes:
            print("[compare] note: " + n)
        for d in diffs:
            print("[compare] " + d)
        print(f"[compare] {'the same' if not diffs else f'{len(diffs)} difference(s)'}: {a.rebuilt} against {a.reference}")
        return 1 if diffs else 0

    if a.command == "safety-report":
        from . import sidecars

        lines, faults = sidecars.check_datasets([os.path.expanduser(d) for d in a.datasets])
        for line in lines:
            print("[report] " + line)
        for fault in faults:
            print("[report] FAULT: " + fault)
        print(f"[report] {'no faults' if not faults else f'{len(faults)} fault(s)'} in {len(a.datasets)} dataset(s)")
        return 1 if faults else 0

    if a.command == "safety-check":
        from . import safety, sidecars
        from .jlog import JlogError
        from .takes import TakeError

        failed = 0
        for path in a.takes:
            try:
                t = read_take(path, a.fps)
            except (TakeError, JlogError, OSError, ValueError, KeyError) as e:
                if safety.scenario_of(path) != safety.MIXED:
                    raise  # the four categories as before
                reasons = safety.broken(path, None, a.drive_exit, error=e)
                v = safety.Verdict(False, "broken: " + "; ".join(reasons), 0, (), 0)
                moved = safety.set_aside(path, v) if a.set_aside else None
                failed += 1
                print(json.dumps({"take": os.path.abspath(path), "category": safety.MIXED, "ok": False, "reason": v.reason,
                                  "frames": 0, "unsafeFrames": 0, "touched": [], "unsafeRanges": [], "contacts": 0,
                                  "eventsDrawn": None, "events": [], "broken": reasons, "setAside": moved}), flush=True)
                continue
            stretches = safety.contacts(path)
            labels = safety.frame_labels(t, stretches)
            v = safety.check(t, labels, stretches, safety.object_poses(path))
            reasons = safety.broken(path, t, a.drive_exit) if t.scenario == safety.MIXED else []
            if reasons:
                v = safety.Verdict(False, "broken: " + "; ".join(reasons), v.unsafe_frames, v.touched, v.contacts)
            planned = safety.planned_events(path)  # read before a failing take is moved
            what = {}
            try:  # what happened (docs/adr/0038 decision 6), for every take; never a reason to set one aside
                e = sidecars.episode(t, sidecars.Options(geometry_dir=a.geometry_dir or None, stop=None))
                what = {k: e.line[k] for k in ("outcome", "tableContact", "tableContacts", "knocked", "minClearanceMm",
                                               "firstContactS", "afterLastContactS", "timingFlags", "group", "role", "edit")}
            except Exception as err:  # noqa: BLE001
                what = {"outcomeError": f"{type(err).__name__}: {err}"}
            moved = safety.set_aside(path, v) if (a.set_aside and not v.ok) else None
            failed += 0 if v.ok else 1
            print(json.dumps({"take": os.path.abspath(path), "category": t.scenario, "ok": v.ok, "reason": v.reason,
                              "frames": len(labels), "unsafeFrames": v.unsafe_frames, "touched": list(v.touched),
                              "unsafeRanges": safety.ranges(labels), "contacts": v.contacts,
                              "eventsDrawn": planned[0]["drawn"] if planned else None, "events": safety.event_timing(t, stretches, planned),
                              **({"broken": reasons} if t.scenario == safety.MIXED else {}), **what,
                              "setAside": moved}), flush=True)
        return 1 if failed else 0

    if a.command == "knock-report":
        from . import safety

        rows = []
        for path in a.takes:
            try:
                rows.extend(safety.knocks(path))
            except (OSError, ValueError, KeyError) as e:
                print(f"[knock] {path}: skipped ({e})", file=sys.stderr)
        for r in rows:
            print(json.dumps(r), flush=True)
        for line in safety.knock_summary(rows):
            print("[knock] " + line, file=sys.stderr)
        return 0 if rows else 1

    if a.command == "safety-build":
        from . import safety, sidecars
        from .writer import SAFETY_CRF, SAFETY_PRESET, LeRobotWriter

        if a.jobs < 1:
            p.error("--jobs is at least 1")
        if a.split < 1:
            p.error("--split is at least 1")
        stop = None if a.no_unavoidable else sidecars.StopModel()
        if a.stop_time_s is not None:
            try:
                stop = sidecars.StopModel.given(a.stop_time_s)
            except ValueError as e:
                p.error(str(e))
        options = sidecars.Options(caption=a.caption, table_safe=a.table_safe, geometry_dir=a.geometry_dir or None, stop=stop)
        takes = [read_take(path, a.fps) for path in a.takes]
        writer = LeRobotWriter(a.vcodec, crf=SAFETY_CRF if a.crf is None else a.crf, preset=SAFETY_PRESET if a.preset is None else a.preset)
        _log(f"building {len(takes)} episode(s) with {a.jobs} worker(s): captions {a.caption}, table contacts "
             f"{'not unsafe (--table-safe)' if a.table_safe else 'unsafe'}, clearance from {a.geometry_dir or 'nothing'}"
             + (f", {a.split} datasets" if a.split > 1 else "") + ", unavoidable ranges "
             + (", ".join(f"{n} {t:g} s" for n, t in stop.times) if stop else "off"))
        if a.split > 1:
            roots = safety.build_split(writer, os.path.expanduser(a.out), takes, a.split, _log, jobs=a.jobs, options=options)
            _log("datasets: " + " ".join(roots))
        else:
            safety.build(writer, os.path.expanduser(a.out), takes, _log, jobs=a.jobs, options=options)
        return 0

    if a.command == "inspect":
        t = read_take(a.take, a.fps)
        filled = sum(1 for f in t.frames if f.timing["filled"])
        ages = sorted(f.timing["stateAge"] for f in t.frames)
        print(json.dumps({"dataset": f"{t.dataset_group}/{t.scenario}", "task": t.task, "robot": t.robot,
                          "frames": len(t.frames), "seconds": len(t.frames) / t.fps, "filled": filled,
                          "cameras": [[c.name, c.width, c.height] for c in t.cameras], "names": t.names,
                          "stateAgeMsP50": ages[len(ages) // 2] / 1e6, "first": t.frames[0].state}, indent=2))
        return 0

    from .watch import Converter
    from .writer import LeRobotWriter

    roots = [os.path.expanduser(r) for r in a.root]
    out = os.path.expanduser(a.out)
    puller = None
    if a.command == "watch" and a.pull_robot:
        from .pull import RobotPuller

        puller = RobotPuller(os.path.expanduser(a.pull_robot), hosts_file=os.path.expanduser(a.robot_hosts_file) or None,
                             mdns=a.robot_mdns, key=os.path.expanduser(a.robot_key) or None, log=_log)
    conv = Converter(roots, out, LeRobotWriter(a.vcodec), fps=a.fps, settle=a.settle, log=_log, puller=puller)
    if a.command == "watch":
        conv.run(every=a.every, once=a.once)
        return 0
    from .writer import rebuild

    root = os.path.join(out, a.dataset)
    takes = conv.kept().get(root, [])
    rebuild(conv.writer, root, [read_take(t, a.fps) for t in takes], _log)
    return 0


if __name__ == "__main__":
    sys.exit(main())
