#!/usr/bin/env python3
"""Records the drive sessions of a pilot plan on the robot itself (`just pilot-on-robot`, docs/adr/0021).

Runs on the Jetson, detached, so the laptop can close: for each `drive:` session in the plan it does what
`just record-drive` does from the laptop, locally. That means it writes session.json and robot-profile.json,
opens the recording (`~/recordings/.active`, which RobotHost and the driver watch), sets the robot-side
uplink, runs joint-drive against RobotHost on 127.0.0.1 with its operator.jlog straight into the session,
then closes the recording as `just record-stop` does: END trailers, stoppedUtc, files.sha256. Teleop
sessions are skipped; they need the headset and `just pilot only=teleop`.

**The session format is written by two writers, kept the same by hand:** session.json here must match
`just record-start`'s, and closing must match `just record-stop`'s. `just recording-check` checks
what both produce, on fetch.

**Someone watches the arm the whole time** (root CLAUDE.md). Nothing here watches servo temperature, a
silent board or the space around the arm. The stops are:
- `just pilot-stop` (any machine), which touches `~/recordings/.pilot-stop`. joint-drive holds the arm at
  once, and this runner closes the session and exits;
- the robot's power switch: the arm goes limp and the shoulder sags;
- joint-drive's own watchdog (a stall, or a stream lost for 3 s of 10), which holds the arm. This runner
  then closes the session and starts no other: any early exit wants a person to look first.

It stops the run below 11.0 V (the last battery report in the session's driver.jlog), and rests the
servos between sessions. Stdlib only, Python 3.8 (Ubuntu 20.04, ROS 2 Foxy).
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pilot import read_plan  # noqa: E402  (the plan format is pilot.py's)

HOME = os.path.expanduser("~")
RECORDINGS = os.path.join(HOME, "recordings")
ACTIVE = os.path.join(RECORDINGS, ".active")
UPLINK = os.path.join(RECORDINGS, ".uplink")
STOP = os.path.join(RECORDINGS, ".pilot-stop")
DOTNET = os.path.join(HOME, ".dotnet", "dotnet")
JOINT_DRIVE = os.path.join(HOME, "jointdrive_current", "Teleop.Eval.dll")
MIN_BATTERY_MV = 11000


def log(message: str) -> None:
    stamp = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"{stamp} {message}", flush=True)


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def build_info(path: str) -> dict:
    try:
        with open(path) as f:
            return json.loads(f.read())
    except (OSError, ValueError):
        return {}


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def options(session) -> dict:
    values = {"load": "none", "payload_g": "0", "network": "lan", "added_delay_ms": "0", "notes": ""}
    for opt in session.options:
        key, _, value = opt.partition("=")
        if key not in values:
            raise SystemExit(f"{session.label}: option '{key}' is not one a drive session takes here ({', '.join(values)})")
        values[key] = value
    if values["load"] not in ("none", "payload", "hand"):
        raise SystemExit(f"{session.label}: load must be none, payload or hand")
    return values


def session_json(sid: str, session, o: dict, profile_sha: str) -> dict:
    """As `just record-start` writes it (keep the two the same)."""
    network = o["network"]
    uplink = {"uplinkProfile": network, "uplinkSeed": 0, "addedDelayMs": float(o["added_delay_ms"])}
    if network.startswith("schedule:"):
        uplink.update({"uplinkProfile": "schedule", "uplinkSchedule": network[len("schedule:"):], "uplinkSeed": 1})
    notes = "joint-drive ran on the robot, to RobotHost over loopback (no Wi-Fi)"
    if o["notes"]:
        notes += "; " + o["notes"]
    return {
        "format": "jetrover-session", "version": 1, "id": sid, "source": "robot", "label": session.label,
        "startedUtc": utc_now(),
        "conditions": {
            "speedSliderRadPerSec": session.slider if session.slider == "varied" else float(session.slider),
            "load": {"kind": o["load"], "payloadGrams": float(o["payload_g"])},
            "network": uplink,
            "motion": "joint-drive:" + session.motion.split(":", 1)[1], "operator": "joint-drive@robot",
        },
        "builds": {
            "robothost": build_info(os.path.join(HOME, "robothost_current", "BUILD_INFO")),
            "driver": build_info(os.path.join(HOME, "teleop_ros_current", "BUILD_INFO")),
        },
        "robotProfileSha256": profile_sha, "notes": notes,
    }


def open_session(session, profile_path: str) -> str:
    o = options(session)
    sid = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + session.label
    directory = os.path.join(RECORDINGS, "jetrover", sid)
    os.makedirs(directory)
    with open(os.path.join(directory, "session.json"), "w") as f:
        json.dump(session_json(sid, session, o, sha256_file(profile_path)), f, indent=2)
    with open(profile_path, "rb") as src, open(os.path.join(directory, "robot-profile.json"), "wb") as dst:
        dst.write(src.read())
    with open(ACTIVE, "w") as f:
        f.write(directory + "\n")
    time.sleep(2.5)
    for name in ("robothost.jlog", "driver.jlog"):
        path = os.path.join(directory, name)
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            close_session(directory)
            raise SystemExit(f"{name} is not being written: {'just deploy-robothost' if name.startswith('robothost') else 'just deploy-ros'} "
                             "(someone watching), then start again")
    # As just uplink-impair / uplink-schedule write it, once the logs are open so the change is recorded (UP).
    network = o["network"]
    if network.startswith("schedule:"):
        with open(UPLINK, "w") as f:
            f.write(f"schedule 1 {network[len('schedule:'):]}\n")
    elif network not in ("lan", "none"):
        with open(UPLINK, "w") as f:
            f.write(f"{network} 1\n")
    return directory


def close_session(directory: str) -> None:
    """As `just record-stop` does (keep the two the same)."""
    if os.path.exists(UPLINK):
        os.remove(UPLINK)
        time.sleep(1)
    if os.path.exists(ACTIVE):
        os.remove(ACTIVE)
    for _ in range(60):
        if all(trailer(os.path.join(directory, n)).startswith("END|") for n in ("robothost.jlog", "driver.jlog")):
            break
        time.sleep(0.25)
    path = os.path.join(directory, "session.json")
    with open(path) as f:
        meta = json.load(f)
    meta["stoppedUtc"] = utc_now()
    slider_from_commands(directory, meta)
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)
    names = ["session.json", "robot-profile.json"] + sorted(n for n in os.listdir(directory) if n.endswith(".jlog"))
    with open(os.path.join(directory, "files.sha256"), "w") as f:
        for n in names:
            f.write(f"{sha256_file(os.path.join(directory, n))}  {n}\n")
    for n in names[2:]:
        log(f"  {n:16} {trailer(os.path.join(directory, n))}")


def slider_from_commands(directory: str, meta: dict) -> None:
    """The slider is what the commands carried (as `just record-stop` reads it; keep the two the same)."""
    speeds = set()
    with open(os.path.join(directory, "robothost.jlog")) as f:
        for line in f:
            if line.startswith("JC|"):
                for target in line.rstrip("\n").split("|")[6].split(";"):
                    motor, _, speed = target.split(":")
                    if motor != "10" and float(speed) > 0:
                        speeds.add(round(float(speed), 2))
    if not speeds:
        return
    asked = meta["conditions"]["speedSliderRadPerSec"]
    measured = next(iter(speeds)) if len(speeds) == 1 else "varied"
    meta["conditions"]["speedSliderRadPerSec"] = measured
    if measured != asked:
        note = f"slider from the commands: {measured}; the plan said {asked}"
        meta["notes"] = (meta["notes"] + "; " if meta.get("notes") else "") + note
        log(f"WARNING: the slider was {measured}, not {asked}")


def trailer(path: str) -> str:
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 200))
            lines = f.read().decode("ascii", "replace").rstrip("\n").split("\n")
            return lines[-1] if lines else ""
    except OSError:
        return ""


def battery_mv(directory: str):
    last = None
    try:
        with open(os.path.join(directory, "driver.jlog")) as f:
            for line in f:
                if line.startswith("BV|"):
                    last = line
    except OSError:
        return None
    try:
        return int(last.split("|")[2]) if last else None
    except (IndexError, ValueError):
        return None


def wait(seconds: float) -> bool:
    """Sleeps, waking early (False) if the stop file appears."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if os.path.exists(STOP):
            return False
        time.sleep(0.5)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("plan")
    parser.add_argument("--profile", required=True, help="the robot profile record-start would copy (core/RobotProfiles/jetrover.json)")
    parser.add_argument("--from", dest="start", default="")
    parser.add_argument("--rest", type=float, default=60, help="seconds between sessions, for the servos")
    parser.add_argument("--start-delay", type=float, default=30, help="seconds before the first session")
    args = parser.parse_args()

    sessions = [s for s in read_plan(args.plan) if s.motion.startswith("drive:")]
    if args.start:
        labels = [s.label for s in sessions]
        if args.start not in labels:
            raise SystemExit(f"no drive session '{args.start}' in {args.plan}")
        sessions = sessions[labels.index(args.start):]
    for s in sessions:
        options(s)
    if os.path.exists(ACTIVE) and open(ACTIVE).read().strip():
        raise SystemExit(f"a session is already recording ({open(ACTIVE).read().strip()}): just record-stop first")
    if not os.path.exists(JOINT_DRIVE):
        raise SystemExit(f"{JOINT_DRIVE} is missing: just deploy-joint-drive")
    log(f"{len(sessions)} drive sessions, {sum(s.minutes for s in sessions):.0f} min, from {args.plan}; "
        f"joint-drive build {build_info(os.path.join(HOME, 'jointdrive_current', 'BUILD_INFO'))}")
    for s in sessions:
        log(f"  {s.label:18} {s.motion:16} {s.slider:>5} rad/s {s.minutes:4.0f} min  {' '.join(s.options)}")
    log(f"first session in {args.start_delay:.0f} s. Stop: just pilot-stop, or the power switch.")
    if not wait(args.start_delay):
        log("stopped before the first session (just pilot-stop)")
        return 130

    done = []
    for index, s in enumerate(sessions, 1):
        log(f"=== [{index}/{len(sessions)}] {s.label}")
        directory = open_session(s, args.profile)
        sid = os.path.basename(directory)
        log(f"recording {sid}")
        status = 1
        try:
            status = subprocess.call(
                [DOTNET, JOINT_DRIVE, "joint-drive", "--script", s.motion.split(":", 1)[1], "--speed", s.slider,
                 "--seconds", f"{s.minutes * 60:.0f}", "--remote-host", "127.0.0.1", "--confirm-hardware-motion",
                 "--operator-log", os.path.join(directory, "operator.jlog"), "--session-id", sid, "--stop-file", STOP],
                stdin=subprocess.DEVNULL)
        finally:
            close_session(directory)
        log(f"stopped {sid}: joint-drive exited {status}")
        if status == 0:
            done.append(s.label)
        mv = battery_mv(directory)
        log(f"battery: {mv / 1000:.2f} V" if mv else "battery: no report in the session's log")
        if os.path.exists(STOP):
            log("just pilot-stop: not starting another session")
            break
        if status != 0:
            # 2 a stall, 3 the stream lost, 1 no reading or no clear path, 130 stopped: each wants a person
            # to look before the arm moves again, and a silent board would only fail every next session.
            log("joint-drive stopped early (read its lines above): not starting another session")
            break
        if mv is not None and mv < MIN_BATTERY_MV:
            log(f"below {MIN_BATTERY_MV / 1000:.1f} V: stopping. Charge or swap the pack"
                + (f", then just pilot-on-robot from={sessions[index].label}" if index < len(sessions) else ""))
            break
        if index < len(sessions):
            log(f"resting the servos for {args.rest:.0f} s")
            if not wait(args.rest):
                log("just pilot-stop: not starting another session")
                break

    log(f"done: {len(done)} of {len(sessions)} sessions completed ({' '.join(done)}). Copy them: just fetch-recordings")
    return 0


if __name__ == "__main__":
    sys.exit(main())
