#!/usr/bin/env python3
"""Records a list of sessions one after another (`just pilot`, docs/adr/0021).

Reads a plan (robot/pilot/pilot-v1.txt's format) and, for each session, says what a person has to do
first, waits for Enter, then records it through the repo's recipes: `just record-start` / `just
record-stop` around a timed teleop session, or `just record-drive` for one the arm drives itself.
After each it reads the battery from the session's own log and stops below 11.0 V. At the end it runs
`just fetch-recordings`, which copies the sessions here and checks them.

Everything that touches the robot goes through a `just` recipe (root CLAUDE.md, "Testing the real
robot"); this file only sequences them. Stdlib only, so it runs with the system python3.
"""

from __future__ import annotations

import argparse
import select
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

MIN_BATTERY_MV = 11000


@dataclass
class Session:
    label: str
    slider: str
    minutes: float
    motion: str
    options: List[str]
    prompt: str


def read_plan(path: str) -> List[Session]:
    sessions = []
    with open(path, encoding="utf-8") as f:
        for number, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = [x.strip() for x in line.split("|")]
            if len(fields) != 6:
                sys.exit(f"{path}:{number}: expected 6 |-separated fields, got {len(fields)}")
            label, slider, minutes, motion, options, prompt = fields
            if motion not in ("teleop", "drive:waypoints", "drive:steps"):
                sys.exit(f"{path}:{number}: motion must be teleop, drive:waypoints or drive:steps, got '{motion}'")
            float(minutes)
            if slider != "varied":
                float(slider)
            sessions.append(Session(label, slider, float(minutes), motion, shlex.split(options), prompt))
    labels = [s.label for s in sessions]
    if len(set(labels)) != len(labels):
        sys.exit(f"{path}: labels must be unique")
    return sessions


def just(*args: str) -> int:
    print("$ just " + " ".join(shlex.quote(a) for a in args), flush=True)
    return subprocess.call(["just", *args])


def ask(question: str, choices: str) -> str:
    while True:
        answer = input(question).strip().lower()
        if answer == "" or answer in choices:
            return answer
        print(f"  type one of: Enter, {', '.join(choices)}")


def countdown(seconds: float) -> None:
    """Waits, printing the time left every 30 s; Enter ends it early."""
    end = time.monotonic() + seconds
    print(f"  recording for {seconds / 60:.0f} min. Press Enter to stop early.", flush=True)
    next_report = time.monotonic() + 30
    while True:
        left = end - time.monotonic()
        if left <= 0:
            return
        ready, _, _ = select.select([sys.stdin], [], [], min(1.0, left))
        if ready:
            sys.stdin.readline()
            print("  stopped early.")
            return
        if time.monotonic() >= next_report:
            print(f"  {left / 60:4.1f} min left", flush=True)
            next_report += 30


def battery_mv(host: str) -> Optional[int]:
    out = subprocess.run(["just", "recording-battery", host], capture_output=True, text=True)
    text = out.stdout.strip().splitlines()[-1:] if out.stdout.strip() else []
    try:
        return int(text[0]) if text else None
    except ValueError:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("plan")
    parser.add_argument("--from", dest="start", default="", help="resume at this label")
    parser.add_argument("--host", default="auto")
    parser.add_argument("--only", choices=("all", "teleop", "drive"), default="all",
                        help="teleop: leave the drive sessions to just pilot-on-robot")
    args = parser.parse_args()

    sessions = read_plan(args.plan)
    if args.only != "all":
        sessions = [s for s in sessions if (s.motion == "teleop") == (args.only == "teleop")]
    if args.start:
        labels = [s.label for s in sessions]
        if args.start not in labels:
            sys.exit(f"no session '{args.start}' in {args.plan}")
        sessions = sessions[labels.index(args.start):]

    total = sum(s.minutes for s in sessions)
    print(f"{len(sessions)} sessions, {total:.0f} min of recording, from {args.plan}:")
    for s in sessions:
        print(f"  {s.label:18} {s.motion:16} slider {s.slider:5} {s.minutes:4.0f} min  {' '.join(s.options)}")
    print("\nSomeone watches the arm for every session. Stop at any servo too hot to hold (about 60 °C):")
    print("nothing on the robot reports servo temperature.")

    host = subprocess.run(["just", "_robot-host", args.host], capture_output=True, text=True).stdout.strip()
    if not host:
        sys.exit("could not find the robot (just robot-ip)")
    print(f"robot: {host}\n")
    if just("robot-recordings", host) != 0:
        return 1

    done = []
    for index, s in enumerate(sessions, 1):
        print(f"\n=== [{index}/{len(sessions)}] {s.label}: {s.motion}, slider {s.slider}, {s.minutes:.0f} min {' '.join(s.options)}")
        print(f"  first: {s.prompt}")
        answer = ask("  Enter to start, s to skip, q to quit: ", "sq")
        if answer == "q":
            break
        if answer == "s":
            continue

        if s.motion == "teleop":
            if just("record-start", s.label, s.slider, "motion=teleop", f"remote_host={host}", *s.options) != 0:
                print("  record-start failed; nothing is recording.")
                if ask("  Enter to go on to the next session, q to quit: ", "q") == "q":
                    break
                continue
            try:
                countdown(s.minutes * 60)
            except KeyboardInterrupt:
                print("\n  interrupted: stopping the recording.")
                just("record-stop", host)
                return 130
            status = just("record-stop", host)
        else:
            script = s.motion.split(":", 1)[1]
            status = just("record-drive", s.label, script, s.slider, f"seconds={s.minutes * 60:.0f}", f"remote_host={host}", *s.options)
            if status != 0:
                print(f"  record-drive exited {status}: the drive stopped early or the recording failed (read above). "
                      "A session cut short is still recorded and checked.")

        if status == 0:
            done.append(s.label)
        mv = battery_mv(host)
        print(f"  battery: {mv / 1000:.2f} V" if mv else "  battery: no reading in that session's log")
        if mv is not None and mv < MIN_BATTERY_MV:
            following = sessions[index].label if index < len(sessions) else None
            print(f"  below {MIN_BATTERY_MV / 1000:.1f} V: stopping. Charge or swap the pack"
                  + (f", then resume with: just pilot from={following}" if following else "."))
            break
        if index < len(sessions) and ask("  Enter for the next session, q to quit: ", "q") == "q":
            break

    print(f"\nrecorded {len(done)} session(s): {' '.join(done)}")
    return just("fetch-recordings", host)


if __name__ == "__main__":
    sys.exit(main())
