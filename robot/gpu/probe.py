"""What runs in our account on the GPU machine, as one JSON object on stdout.

robot/gpu/dashboard.py pipes this file over ssh (``python3 - [args]``) every few seconds and shows the answer. Standard
library only, so it runs on the machine's own Python. It only reads, except ``act``, which signals one job's process group
after checking again that it is ours and of the kind asked for.

    python3 probe.py                 the status JSON
    python3 probe.py act KIND SIGNAL PGID
                                     SIGNAL (STOP, CONT or TERM) to process group PGID, if every process in it is ours,
                                     at least one is a KIND job (training, netfetch, simfit) and none is another job's;
                                     a TERM is followed by a CONT, so a paused job stops too

Paths follow docs/gpu-machine.md, with the layout before 2026-11-15 as a fallback.
"""

from __future__ import annotations

import glob
import json
import os
import re
import signal
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    import pwd
except ImportError:  # Windows, where only robot/gpu/tests import this file (their pure functions)
    pwd = None  # type: ignore[assignment]

HOME = os.path.expanduser("~")

# A job kind, the patterns in a command line that make a process one of its processes, and the actions it allows. Every
# other process of ours is listed as "other" unless it is the machinery itself (ssh, this probe, a shell). The first kind
# that matches wins: netfetch comes before training, whose "-m armpred" also matches "-m armpred.net.datasets".
KINDS: List[Tuple[str, Tuple[str, ...]]] = [
    ("twin", ("teleop_isaac.jetrover_bridge", "teleop_isaac.bridge", "teleop_isaac.renderer", "Teleop.RobotHost.dll")),
    ("watcher", ("teleop_lerobot watch",)),
    ("safety-build", ("teleop_lerobot safety-build",)),
    ("safety-check", ("teleop_lerobot safety-check",)),
    ("rebuild", ("teleop_lerobot rebuild",)),
    ("netfetch", ("armpred.net.datasets",)),
    ("training", ("armpred finetune", "train-sim.sh", "-m armpred", "bin/armpred")),
    ("simfit", ("teleop_isaac.simfit_run",)),
    ("isaac-setup", ("isaacsim[all", "pip install 'isaacsim")),
]
SIGNALLABLE = {"training", "netfetch", "simfit"}
MACHINERY = ("sshd", "probe.py", "-bash", "bash -s", "sleep ", "systemd", "(sd-pam)", "dbus-daemon", "tail -n", "crontab")
# A shell running an inline script (isaac-host's `bash -c <script>`, a fetch's `setsid bash -c "...; ..."`) only names
# the commands it starts; those processes are the job, the shell is machinery whatever its script mentions.
INLINE_SHELL = re.compile(r"^(?:\S*/)?(?:ba|da|z)?sh -c ")
# This probe as the dashboard runs it: `python3 -` or `python3 - act ...`, but not `python3 -m ...` or `python3 -c ...`.
PIPED_PROBE = re.compile(r"^python3 -(?: |$)")


def first_existing(*paths: str) -> str:
    """The first of ``paths`` that exists, else the first (the layout of docs/gpu-machine.md, then the one before it)."""
    for p in paths:
        if os.path.exists(p):
            return p
    return paths[0]


def paths(home: str = HOME) -> Dict[str, str]:
    return {
        "isaac_logs": first_existing(f"{home}/logs/isaac", f"{home}/isaacsim"),
        "lerobot_logs": first_existing(f"{home}/logs/lerobot", f"{home}/data/lerobot", f"{home}/isaacsim/lerobot"),
        "training_logs": first_existing(f"{home}/logs/training", f"{home}/logs"),
        "lerobot": first_existing(f"{home}/data/lerobot", f"{home}/isaacsim/lerobot"),
        "sessions_jetrover": first_existing(f"{home}/data/sessions/jetrover-isaac", f"{home}/isaacsim/jetrover-recordings"),
        "sessions_fr3": first_existing(f"{home}/data/sessions/fr3_duo-isaac", f"{home}/isaacsim/recordings"),
        "simfit": first_existing(f"{home}/analysis/simfit", f"{home}/isaacsim/simfit"),
        "trainer": f"{home}/teleoperation/trainer",
    }


# ---------------------------------------------------------------------------------------------------------------------
# Processes


def classify(cmdline: str) -> str:
    if INLINE_SHELL.match(cmdline) or PIPED_PROBE.match(cmdline):
        return "machinery"
    for kind, patterns in KINDS:
        if any(p in cmdline for p in patterns):
            return kind
    if any(m in cmdline for m in MACHINERY):
        return "machinery"
    return "other"


def boot_time() -> float:
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("btime "):
                return float(line.split()[1])
    return 0.0


def processes(uid: Optional[int] = None) -> List[dict]:
    """Every process of ``uid`` (ours by default): pid, process group, state, start time, CPU seconds, memory, command."""
    uid = os.getuid() if uid is None else uid
    tick = os.sysconf("SC_CLK_TCK")
    btime = boot_time()
    out = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        pid = int(d)
        try:
            if os.stat(f"/proc/{pid}").st_uid != uid:
                continue
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
            with open(f"/proc/{pid}/stat") as f:
                stat = f.read()
            rss = 0
            with open(f"/proc/{pid}/status") as f:
                for line in f:
                    if line.startswith("VmRSS:"):
                        rss = int(line.split()[1]) * 1024
                        break
        except (OSError, ValueError):
            continue
        if not cmd or pid == os.getpid():
            continue
        # The fields after the command name (which may hold spaces and parentheses).
        rest = stat[stat.rindex(")") + 2:].split()
        state, ppid, pgid = rest[0], int(rest[1]), int(rest[2])
        cpu = (int(rest[11]) + int(rest[12])) / tick
        started = btime + int(rest[19]) / tick
        out.append({"pid": pid, "ppid": ppid, "pgid": pgid, "state": state, "started": started, "cpu_s": cpu, "rss": rss,
                    "cmd": cmd, "kind": classify(cmd)})
    return out


# ---------------------------------------------------------------------------------------------------------------------
# Logs and progress


def tail(path: str, n: int = 4000) -> List[str]:
    """The last ``n`` bytes of a text file, as lines (none when it cannot be read)."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - n))
            data = f.read().decode("utf-8", "replace")
    except OSError:
        return []
    lines = data.splitlines()
    return lines[1:] if size > n and lines else lines


def mtime(path: str) -> float:
    """A file's modification time, 0 when it is gone (a log can be moved between the glob and the look)."""
    try:
        return os.path.getmtime(path)
    except OSError:
        return 0.0


def newest(pattern: str) -> Optional[str]:
    files = glob.glob(pattern)
    return max(files, key=mtime) if files else None


STEP = re.compile(r"\bstep\s+(\d+)\b.*?([0-9.]+)s\s*$")
SIMULATING = re.compile(r"simulating (\d+) (?:\w+ )?sessions")  # "simulating 204 sessions", "simulating 96 FR3 sessions"
SIM_DONE = re.compile(r"(\d+)/(\d+) sessions \((\d+)s\)")
EVALUATING = re.compile(r"^\[(\d\d):(\d\d):(\d\d)\] evaluating (\d+) views")
EVAL_DONE = re.compile(r"evaluation done \((\d+)s\)")
RUN = re.compile(r"^run: (trainer/runs/\S+)")


def training_progress(lines: List[str], total_steps: Optional[int], eval_seconds_per_view: Optional[float], now_clock: Optional[Tuple[int, int, int]] = None) -> dict:
    """Where a training run's log says it is: its phase, how far into it, and about how long is left of that phase and of
    what follows (seconds; None when nothing tells). ``now_clock`` is the wall clock (h, m, s) the log's stamps are on."""
    phase, done, total, eta, note = "starting", None, None, None, ""
    for line in lines:
        if SIMULATING.search(line):
            phase = "simulating"
        m = SIM_DONE.search(line)
        if m:
            phase, done, total = "simulating", int(m.group(1)), int(m.group(2))
            secs = int(m.group(3))
            eta = secs / done * (total - done) if done else None
        m = STEP.search(line)
        if m and "loss" in line:
            phase, done, total = "training", int(m.group(1)), total_steps
            secs = float(m.group(2))
            eta = secs / done * (total - done) if (done and total) else None
        if "exported model.onnx" in line:
            phase, done, total, eta = "exported", None, None, None
        m = EVALUATING.search(line)
        if m:
            phase, done, total = "evaluating", None, int(m.group(4))
            started = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))
            if eval_seconds_per_view and now_clock:
                now = now_clock[0] * 3600 + now_clock[1] * 60 + now_clock[2]
                elapsed = (now - started) % 86400
                eta = max(0.0, eval_seconds_per_view * total - elapsed)
                note = f"estimated from the last evaluation's {eval_seconds_per_view:.1f} s a view"
            else:
                eta, note = None, "no earlier evaluation to estimate from"
        if EVAL_DONE.search(line):
            phase, done, total, eta, note = "done", None, None, 0.0, ""
    return {"phase": phase, "done": done, "total": total, "eta_s": eta, "note": note}


def eval_rate(log_files: List[str]) -> Optional[float]:
    """Seconds a view, from the newest log with a finished evaluation."""
    for path in sorted(log_files, key=mtime, reverse=True):
        views, secs = None, None
        for line in tail(path, 200000):
            m = EVALUATING.search(line)
            if m:
                views = int(m.group(4))
            m = EVAL_DONE.search(line)
            if m and views:
                secs = int(m.group(1))
        if views and secs:
            return secs / views
    return None


def toml_steps(path: str) -> Optional[int]:
    """``steps`` in a run config's [train] table (no TOML parser before Python 3.11 is assumed; a line scan suffices)."""
    try:
        section = ""
        with open(path, encoding="utf-8") as f:
            for line in f:
                s = line.split("#", 1)[0].strip()  # a header or a value may carry a comment: [train]  # the loop
                if s.startswith("["):
                    section = s.replace(" ", "")
                elif section == "[train]" and re.match(r"steps\s*=", s):
                    return int(s.split("=", 1)[1].strip())
    except (OSError, ValueError):
        pass
    return None


def run_steps(run_dir: str, cmd: str) -> Optional[int]:
    """A run's training steps: a ``train.steps=N`` it was started with (``--set``, kept in its overrides.json; the
    command line's own, while the run has not written that yet), else its config.toml's."""
    sets: List[str] = []
    try:
        with open(os.path.join(run_dir, "overrides.json"), encoding="utf-8") as f:
            sets = [s for s in json.load(f) if isinstance(s, str)]
    except (OSError, ValueError):
        pass
    args = cmd.split()
    sets += [args[i + 1] for i, a in enumerate(args[:-1]) if a == "--set"]
    for s in reversed(sets):  # the last one given wins
        m = re.match(r"train\.steps\s*=\s*([0-9_]+)\s*$", s)
        if m:
            return int(m.group(1))
    return toml_steps(os.path.join(run_dir, "config.toml"))


def crontab_has(tag: str) -> bool:
    try:
        return tag in subprocess.run(["crontab", "-l"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return False


# ---------------------------------------------------------------------------------------------------------------------
# Jobs


def twin_role(cmd: str) -> str:
    if "Teleop.RobotHost.dll" in cmd:
        return "RobotHost"
    return "renderer" if "teleop_isaac.renderer" in cmd else "bridge"


# A JetRover twin's processes all name its record control file (docs/adr/0037 amendment 7): twin 0's in
# ~/data/sessions/jetrover-isaac, twin N's in ~/data/sessions/jetrover-isaac-twinN.
TWIN_CONTROL = re.compile(r"--record-control \S*/jetrover-isaac(?:-twin(\d+))?/\.active(?:\s|$)")


def twin_of(cmd: str) -> Optional[int]:
    """Which JetRover twin a process belongs to, from the record control file it names; None for none (the FR3 Duo's)."""
    m = TWIN_CONTROL.search(cmd)
    return None if m is None else int(m.group(1) or 0)


def job_twins(procs: List[dict], p: Dict[str, str]) -> List[dict]:
    """Each Isaac host as a job: every JetRover twin (by the record control file its processes name), and the FR3 Duo's."""
    mine = [x for x in procs if x["kind"] == "twin"]
    groups: Dict[Optional[int], List[dict]] = {}
    for x in mine:
        n = twin_of(x["cmd"])
        # A JetRover process that names no control file (an older one): twin 0's, as isaac-stop took it before twins.
        if n is None and ("jetrover_bridge" in x["cmd"] or "--relay-socket" in x["cmd"]):
            n = 0
        groups.setdefault(n, []).append(x)
    jobs = []
    for n in sorted(groups, key=lambda k: -1 if k is None else k):
        jobs.append(_twin_job(groups[n], p, n))
    return jobs


def _twin_job(mine: List[dict], p: Dict[str, str], twin: Optional[int]) -> dict:
    robot = "FR3 Duo" if twin is None else "JetRover"
    name = f"{robot} twin" if not twin else f"{robot} twin {twin}"
    roles = sorted({twin_role(x["cmd"]) for x in mine})
    lines = [f"{name}: {', '.join(roles)}" + ("" if "renderer" in roles else " (no renderer: no cameras)")]
    if twin:
        lines[0] += f"; joint port {6002 + 100 * twin}"
    log = "renderer.log" if not twin else f"renderer-{twin}.log"
    fps = [l for l in tail(os.path.join(p["isaac_logs"], log), 6000) if re.match(r"^\[render\] [0-9.]+ fps", l)]
    if fps and "renderer" in roles:
        lines.append(fps[-1][:160])
    sessions = p["sessions_fr3"] if twin is None else p["sessions_jetrover"] if twin == 0 else \
        os.path.join(os.path.dirname(p["sessions_jetrover"]), f"jetrover-isaac-twin{twin}")
    try:
        recording = open(os.path.join(sessions, ".active")).read().strip()
    except OSError:
        recording = ""
    if recording:
        lines.append(("recording a take: " if "take" in os.path.basename(recording) else "recording: ") + recording.replace(HOME, "~"))
    else:
        lines.append("not recording")
    return {"id": "twin" if not twin else f"twin-{twin}", "kind": "twin", "twin": twin or 0,
            "title": f"Isaac twin ({robot})" if not twin else f"Isaac twin {twin} ({robot})", "status": "running",
            "started": min(x["started"] for x in mine), "pids": [x["pid"] for x in mine], "detail": lines,
            "progress": None, "eta_s": None, "eta_note": "runs until stopped", "actions": ["stop"]}


def job_watcher(procs: List[dict], p: Dict[str, str]) -> dict:
    mine = [x for x in procs if x["kind"] == "watcher"]
    keeper = crontab_has("teleop-lerobot-keeper")
    log = [l for l in tail(os.path.join(p["lerobot_logs"], "watch.log"), 3000) if l.strip()][-3:]
    lines = [("keeper on: cron restarts it within 5 min" if keeper else "keeper off: nothing restarts it after a reboot")] + log
    if not mine:
        return {"id": "watcher", "kind": "watcher", "title": "LeRobot watcher", "status": "stopped", "started": None, "pids": [],
                "detail": lines, "progress": None, "eta_s": None, "eta_note": "", "actions": ["start"]}
    return {"id": "watcher", "kind": "watcher", "title": "LeRobot watcher", "status": "running",
            "started": min(x["started"] for x in mine), "pids": [x["pid"] for x in mine], "detail": lines, "progress": None,
            "eta_s": None, "eta_note": "converts each take a minute after it is complete", "actions": ["stop"]}


EPISODE = re.compile(r"\bepisode (\d+):")


def episodes_done(lines: List[str]) -> int:
    """How many episodes a safety build has written, from its log's ``episode N:`` lines (N counts from 0). The highest
    N, not the number of lines: the encoder writes about 4 KB between two of them, so a tail of a long build's log does
    not hold them all."""
    ns = [int(m.group(1)) for m in map(EPISODE.search, lines) if m]
    return max(ns) + 1 if ns else 0


def build_log(out: str) -> Optional[str]:
    """The newest log of the safety build writing ``out``: ``<name>.build-<attempt>.log`` beside it. The recipe's retry
    writes ``<name>-retry`` and logs to ``<name>.build-2.log``."""
    base = re.sub(r"-retry$", "", os.path.basename(out.rstrip("/")))
    return newest(os.path.join(os.path.dirname(out.rstrip("/")), base + ".build-*.log"))


def job_safety_build(procs: List[dict], p: Dict[str, str], now: float) -> Optional[dict]:
    # The builder itself (python), not a shell whose text names it.
    mine = [x for x in procs if x["kind"] == "safety-build" and ("lerobot-venv" in x["cmd"] or x["cmd"].startswith(("python", "/")))]
    if not mine:
        return None
    x = min(mine, key=lambda y: y["started"])
    args = x["cmd"].split()
    out = args[args.index("--out") + 1] if "--out" in args[:-1] else ""
    after = args[args.index("safety-build") + 1:] if "safety-build" in args else []
    takes = [a for a in after if os.path.isabs(a) and a != out]
    log = build_log(out) if out else None
    done = episodes_done(tail(log, 64000)) if log else 0
    elapsed = now - x["started"]
    eta = elapsed / done * (len(takes) - done) if done else None
    return {"id": f"safety-build-{x['pgid']}", "kind": "safety-build", "title": "Safety dataset build",
            "status": "running", "started": x["started"], "pids": [y["pid"] for y in mine],
            "detail": [f"into {out.replace(HOME, '~')}", f"log {log.replace(HOME, '~')}" if log else "no log yet"],
            "progress": {"done": done, "total": len(takes), "unit": "episodes"}, "eta_s": eta, "eta_note": "", "actions": []}


def job_training(procs: List[dict], p: Dict[str, str], now: float) -> List[dict]:
    jobs = []
    groups: Dict[int, List[dict]] = {}
    for x in procs:
        if x["kind"] == "training":
            groups.setdefault(x["pgid"], []).append(x)
    logs = glob.glob(os.path.join(p["training_logs"], "train-sim-*.log"))
    rate = eval_rate(logs) if groups else None
    for pgid, mine in groups.items():
        lead = min(mine, key=lambda y: y["started"])
        log = None
        for path in sorted(logs, key=mtime, reverse=True):
            if mtime(path) >= lead["started"] - 120:
                log = path
                break
        lines = tail(log, 400000) if log else []
        run = next((m.group(1) for m in map(RUN.search, lines) if m), None)
        if run is None:
            args = lead["cmd"].split()
            run = f"trainer/runs/{args[args.index('--out') + 1].split('/')[-1]}" if "--out" in args[:-1] else "?"
        steps = run_steps(os.path.join(p["trainer"], run.replace("trainer/", "", 1)), lead["cmd"])
        clock = time.localtime(now)
        prog = training_progress(lines, steps, rate, (clock.tm_hour, clock.tm_min, clock.tm_sec))
        paused = all(y["state"] == "T" for y in mine)
        detail = [f"run {run}", f"phase: {prog['phase']}"] + ([l[:160] for l in lines[-2:]] if lines else ["no log found"])
        progress = {"done": prog["done"], "total": prog["total"], "unit": "steps" if prog["phase"] == "training" else
                    "sessions" if prog["phase"] == "simulating" else "views"} if prog["done"] is not None and prog["total"] else None
        # The estimate covers the phase it is in; the evaluating phase's note says what it is estimated from.
        note = {"simulating": "of the simulation; training and evaluation follow",
                "training": "of the training; evaluation follows"}.get(prog["phase"], prog["note"])
        jobs.append({"id": f"training-{pgid}", "kind": "training", "pgid": pgid, "title": f"Training: {run.split('/')[-1]}",
                     "status": "paused" if paused else "running", "started": lead["started"], "pids": [y["pid"] for y in mine],
                     "detail": detail, "progress": progress, "eta_s": prog["eta_s"], "eta_note": note,
                     "actions": ["resume", "stop"] if paused else ["pause", "stop"]})
    return jobs


def job_signalled(procs: List[dict], kind: str, title: str, log: Optional[str]) -> List[dict]:
    jobs = []
    groups: Dict[int, List[dict]] = {}
    for x in procs:
        if x["kind"] == kind:
            groups.setdefault(x["pgid"], []).append(x)
    for pgid, mine in groups.items():
        lines = [l[:160] for l in tail(log, 3000) if l.strip()][-3:] if log else []
        paused = all(y["state"] == "T" for y in mine)
        jobs.append({"id": f"{kind}-{pgid}", "kind": kind, "pgid": pgid, "title": title,
                     "status": "paused" if paused else "running", "started": min(y["started"] for y in mine),
                     "pids": [y["pid"] for y in mine], "detail": lines or ["no log found"], "progress": None, "eta_s": None,
                     "eta_note": "", "actions": ["stop"]})
    return jobs


def job_transient(procs: List[dict], kind: str, title: str) -> List[dict]:
    mine = [x for x in procs if x["kind"] == kind]
    if not mine:
        return []
    return [{"id": kind, "kind": kind, "title": title, "status": "running", "started": min(x["started"] for x in mine),
             "pids": [x["pid"] for x in mine], "detail": [x["cmd"][:160] for x in mine][:2], "progress": None, "eta_s": None,
             "eta_note": "", "actions": []}]


def gpu() -> dict:
    out = {"name": None, "util": None, "mem_used": None, "mem_total": None, "temp": None, "apps": []}
    try:
        q = subprocess.run(["nvidia-smi", "--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu",
                            "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=10).stdout.strip().splitlines()
        if q:
            name, util, used, total, temp = [s.strip() for s in q[0].split(",")]
            out.update(name=name, util=int(util), mem_used=int(used), mem_total=int(total), temp=int(temp))
        apps = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
                              capture_output=True, text=True, timeout=10).stdout.strip().splitlines()
        for line in apps:
            try:  # one odd line ("[N/A]") must not cost the others
                pid, mem = [int(s.strip()) for s in line.split(",")]
            except ValueError:
                continue
            try:
                user = pwd.getpwuid(os.stat(f"/proc/{pid}").st_uid).pw_name
            except (OSError, KeyError):
                user = "?"
            out["apps"].append({"pid": pid, "mem": mem, "user": user})
    except (OSError, subprocess.SubprocessError, ValueError):
        pass
    return out


def status() -> dict:
    now = time.time()
    p = paths()
    procs = processes()
    jobs: List[dict] = []
    jobs += job_twins(procs, p)
    sb = job_safety_build(procs, p, now)
    if sb:
        jobs.append(sb)
    jobs += job_transient(procs, "safety-check", "Safety check of a take")
    jobs += job_training(procs, p, now)
    jobs += job_signalled(procs, "netfetch", "Network datasets fetch", os.path.join(p["training_logs"], "network-datasets.log"))
    jobs += job_signalled(procs, "simfit", "Sim fit", os.path.join(p["simfit"], "fit.log"))
    jobs += job_transient(procs, "rebuild", "Dataset rebuild")
    jobs += job_transient(procs, "isaac-setup", "Isaac Sim install")
    jobs.append(job_watcher(procs, p))
    other = [{"pid": x["pid"], "cmd": x["cmd"][:200], "started": x["started"], "rss": x["rss"], "state": x["state"]}
             for x in procs if x["kind"] == "other"]
    st = os.statvfs(HOME)
    me = pwd.getpwuid(os.getuid()).pw_name
    g = gpu()
    for a in g["apps"]:
        a["ours"] = a["user"] == me
    return {"time": now, "host": os.uname().nodename, "user": me, "load": list(os.getloadavg()), "cores": os.cpu_count(),
            "disk": {"free": st.f_bavail * st.f_frsize, "total": st.f_blocks * st.f_frsize}, "gpu": g, "jobs": jobs,
            "other": sorted(other, key=lambda o: -o["rss"])[:30], "layout": "new" if os.path.isdir(f"{HOME}/data") else "old"}


def foreign_in_group(pgid: int) -> bool:
    """Whether process group ``pgid`` holds a process of another account."""
    me = os.getuid()
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat") as f:
                s = f.read()
            if int(s[s.rindex(")") + 2:].split()[2]) == pgid and os.stat(f"/proc/{d}").st_uid != me:
                return True
        except (OSError, ValueError):
            continue
    return False


def act(kind: str, sig: str, pgid: int) -> dict:
    """``sig`` to process group ``pgid``, only if every process in it is ours, one of them is a ``kind`` job and none is
    another job's (a TERM meant for a training run must never reach the twin)."""
    if kind not in SIGNALLABLE or sig not in ("STOP", "CONT", "TERM"):
        return {"ok": False, "message": f"not allowed: {sig} to a {kind} job"}
    if pgid <= 1:  # killpg(1) is kill(-1): every process this account may signal; killpg(0) is the caller's own group
        return {"ok": False, "message": f"not a process group to signal: {pgid}"}
    group = [x for x in processes() if x["pgid"] == pgid]
    if not group or not any(x["kind"] == kind for x in group):
        return {"ok": False, "message": f"process group {pgid} is no {kind} job of ours (any more)"}
    others = sorted({x["kind"] for x in group} - {kind, "machinery", "other"})
    if others:
        return {"ok": False, "message": f"process group {pgid} also holds a {' and a '.join(others)} process: not signalling it"}
    if foreign_in_group(pgid):
        return {"ok": False, "message": f"process group {pgid} holds another account's process"}
    try:
        os.killpg(pgid, getattr(signal, "SIG" + sig))
        if sig == "TERM":
            # A paused (stopped) process acts on SIGTERM only once it runs again; to a running one CONT is nothing.
            os.killpg(pgid, signal.SIGCONT)
    except OSError as e:
        return {"ok": False, "message": f"SIG{sig} to process group {pgid} failed: {e}"}
    return {"ok": True, "message": f"SIG{sig} sent to process group {pgid} ({len(group)} process(es))"}


def main(argv: List[str]) -> dict:
    if argv[:1] == ["act"]:
        if len(argv) != 4:
            return {"ok": False, "message": "usage: act KIND SIGNAL PGID"}
        try:
            pgid = int(argv[3])
        except ValueError:
            return {"ok": False, "message": f"not a process group: {argv[3]!r}"}
        return act(argv[1], argv[2], pgid)
    return status()


if __name__ == "__main__":
    print(json.dumps(main(sys.argv[1:])))
