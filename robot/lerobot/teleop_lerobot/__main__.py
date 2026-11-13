"""``python -m teleop_lerobot <command>`` (docs/adr/0036):

- ``watch``: keep every scenario's dataset up to date, for good (``--once`` for one pass);
- ``inspect TAKE``: what a take would become, as JSON (no LeRobot needed);
- ``rebuild DATASET``: rebuild one dataset from its kept takes.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

from .takes import read_take


def _log(message: str) -> None:
    print(time.strftime("%Y-%m-%dT%H:%M:%SZ ", time.gmtime()) + message, flush=True)


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
    a = p.parse_args(argv)

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
