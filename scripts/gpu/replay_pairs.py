"""How exactly each counterfactual replay repeats its source's commands, stage by stage (docs/adr/0038 decision 2), read-only.

    ssh andrei@10.188.58.133 'python3 - <dataset or take dir>...' < scripts/gpu/replay_pairs.py
    ssh andrei@10.188.58.133 '~/isaacsim/lerobot-venv/bin/python - <dataset>' < scripts/gpu/replay_pairs.py   (adds `dataset`)

A dataset (a folder with meta/teleop/takes.jsonl) stands for its takes, with their episode numbers. Takes are paired by their
`RP` record: each replay with its group's source. Standard library only.

Per pair it compares, at each stage a command passes (trainer/contract/SESSION_FORMAT.md). **A replay and its source are
meant to match on the commands received** (`JC`, the rows of the dataset's `meta/teleop/commands.parquet`, its `servo_*`):
the replay sends its source's commands. **The per-frame `action` may differ** wherever RobotHost's collision check judged a
command otherwise in the two takes (docs/adr/0038 L, "Sent commands"): it is the newest command sent (`RO`, commands.parquet's
`sent.*`), and RobotHost judges the servo path from where the arm is measured, so whether it refuses or clamps depends on
physics.

- **received**: every `JC`, whatever its outcome: what joint-drive sent and RobotHost received. The two takes' commands are
  matched by joint-drive's sequence number counted from each take's first (shifted by up to 3 when that does better: a lost
  first datagram). `identical` counts matched commands whose targets are equal to the last digit; `lost` the commands one
  take has and the other does not (UDP datagrams that never reached RobotHost); `dt` how much later than its source's the
  replay's command arrived after the take's first, ms.
- **outcome**: `JC`'s outcome letters (A accepted, C clamped, R refused, S stale, P parked) in each take, the runs of refused
  or clamped commands (from-to seconds into the take), and at how many matched commands the outcomes differ. A refused command
  never reaches the relay (`RO`) and the arm keeps its last target; a clamped one reaches it as the farthest passing point
  toward it.
- **per frame**, by the episode frame's index (frames as robot/lerobot/teleop_lerobot/takes.py picks them: a 15 fps grid over
  the recorded frames between the first and the last command RobotHost judged (A, C or R), each with every motor's value), the
  largest difference over the six motors between the replay's frame k and its source's, in servo units, p50 / p95 / max, and
  the share over 20 units:
  - `received`: the newest command received (A, C or R), in relay units as the plant converts it: commands.parquet's
    `servo_*`, the column a replay is meant to match on. What differs is the frames' phase against the commands;
  - `tape`: the same, both read at the source's frame times counted from each take's first such command: what is left
    without the frames' own phase (the renderer's frame clock runs free): the commands' own arrival jitter (`dt`) remains;
  - `action`: the newest relay target (`RO`): the dataset's `action` column and commands.parquet's `sent.*` (as it was until
    2026-11-24 and is again since 2026-11-25; in between a refused command counted too). It differs where the outcomes do;
  - `dataset`: the dataset's own `action` column (with pyarrow, as the lerobot venv has it): equal to `action` but for a
    dataset built 2026-11-24 to 2026-11-25.
"""
import bisect
import json
import math
import os
import statistics
import sys

PULSE_PER_RADIAN = 1000.0 / math.radians(240.0)  # GenericArmPlantConfig.Default
ZERO_PULSE = 500.0
GRIPPER_OPEN, GRIPPER_CLOSED = 116.0, 645.0
MOTORS = (1, 2, 3, 4, 5, 10)
SENSED = (1, 2, 3, 4, 5)
GRIPPER = 10


def records(path, tags):
    out = {t: [] for t in tags}
    with open(path, errors="replace") as f:
        for line in f:
            tag = line[:2]
            if tag in out and line[2:3] == "|":
                out[tag].append(line.rstrip("\n").split("|")[1:])
    return out


def items(s):
    return [it.split(":") for it in s.split(";") if it]


def converter(take):
    with open(os.path.join(take, "robot-profile.json")) as f:
        p = json.load(f)
    joints = {}
    for j in p["joints"]:
        j = {k.lower(): v for k, v in j.items()}
        lo = ZERO_PULSE + j["minangleradians"] * PULSE_PER_RADIAN if j.get("minangleradians") is not None else 0.0
        hi = ZERO_PULSE + j["maxangleradians"] * PULSE_PER_RADIAN if j.get("maxangleradians") is not None else 1000.0
        joints[int(j["motorid"])] = (str(j["role"]).lower(), float(j.get("zerooffsetradians") or 0.0), lo, hi)

    def pulses(targets):
        out = {}
        for it in items(targets):
            m, a = int(it[0]), float(it[1])
            if m not in joints or m in out or not math.isfinite(a):
                continue
            role, offset, lo, hi = joints[m]
            if role == "grippermain":
                out[m] = GRIPPER_OPEN + min(1.0, max(0.0, a)) * (GRIPPER_CLOSED - GRIPPER_OPEN)
            else:
                out[m] = min(hi, max(lo, ZERO_PULSE + (a + offset) * PULSE_PER_RADIAN))
        return out
    return pulses


class Track:
    """Per motor, the newest value at or before a time."""

    def __init__(self, events):  # (t, {motor: value})
        self.t, self.v = {}, {}
        for t, values in sorted(events, key=lambda e: e[0]):
            for m, v in values.items():
                self.t.setdefault(m, []).append(t)
                self.v.setdefault(m, []).append(v)

    def at(self, t):
        out = {}
        for m in self.t:
            i = bisect.bisect_right(self.t[m], t) - 1
            if i >= 0:
                out[m] = self.v[m][i]
        return out


def read(take):
    host = records(os.path.join(take, "robothost.jlog"), ("RP", "JC", "RO", "RI", "WC"))
    pulses = converter(take)
    rp = host["RP"][0] if host["RP"] else None
    jc = [(int(r[1]), int(r[2]), r[4], r[5]) for r in host["JC"]]  # tDeliver, seq, outcome, targets
    ro = [(int(r[0]), {int(it[0]): float(it[1]) for it in items(r[1])}) for r in host["RO"]]
    relay = Track(ro)  # the action: the newest command sent
    judged = [(t, o, targets) for t, _, o, targets in jc if o in ("A", "C", "R")]
    received = Track([(t, pulses(targets)) for t, _, targets in judged])  # commands.parquet's servo_*
    readings = {}
    for r in host["RI"]:
        for it in items(r[1]):
            if it[1] == "1":
                readings.setdefault(int(it[0]), []).append(int(r[0]))
    driven = [t for t, _, _ in judged]
    recorded = []
    with open(os.path.join(take, "isaac.jlog"), errors="replace") as f:
        for line in f:
            if line.startswith("VF|"):
                r = line.rstrip("\n").split("|")[1:]
                if "B" not in r[3]:
                    recorded.append(int(r[2]))
    sensed = lambda t: all(readings.get(m) and readings[m][0] <= t for m in SENSED)
    frames = grid(recorded, driven, lambda t: sensed(t) and all(m in relay.at(t) for m in MOTORS))
    first = min(driven) if driven else 0
    runs, start = [], None
    t_first = jc[0][0] if jc else 0
    for k, (t, _, o, _) in enumerate(jc + [(0, 0, "", "")]):
        if o in ("R", "C") and start is None:
            start = k
        elif o not in ("R", "C") and start is not None:
            runs.append((start, k - 1, (jc[start][0] - t_first) / 1e9, (jc[k - 1][0] - t_first) / 1e9))
            start = None
    return {"rp": rp, "jc": jc, "relay": relay, "received": received, "frames": frames, "first": first, "runs": runs,
            "contacts": len(host["WC"])}


def grid(recorded, commands, usable):
    """The episode's frames as robot/lerobot/teleop_lerobot/takes.py picks them: the recorded frames between the first and last
    of ``commands`` that ``usable`` takes, on a 15 fps grid (the nearest within half a step, else the last again)."""
    if not commands:
        return []
    lo, hi = min(commands), max(commands)
    frames = sorted({t for t in recorded if lo <= t <= hi and usable(t)})
    out = []
    if frames:
        step, t0, i, previous, k = 1e9 / 15, frames[0], 0, None, 0
        while t0 + k * step <= frames[-1] + step / 2:
            g = t0 + k * step
            while i + 1 < len(frames) and abs(frames[i + 1] - g) < abs(frames[i] - g):
                i += 1
            chosen = i if (abs(frames[i] - g) <= step / 2 or previous is None) else previous
            out.append(frames[chosen])
            previous = chosen
            k += 1
    return out


def pct(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q / 100 * (len(s) - 1))))]


def gap(va, vb):
    return max(abs(va.get(m, 0.0) - vb.get(m, 0.0)) for m in MOTORS)


def compare(src, rep):
    s0, r0 = src["jc"][0][1], rep["jc"][0][1]
    a = {seq - s0: (t, o, targets) for t, seq, o, targets in src["jc"]}
    b = {seq - r0: (t, o, targets) for t, seq, o, targets in rep["jc"]}
    shift = max(range(-3, 4), key=lambda d: (sum(1 for k in a if k + d in b and a[k][2] == b[k + d][2]), -abs(d)))
    matched = [k for k in a if k + shift in b]
    identical = sum(1 for k in matched if a[k][2] == b[k + shift][2])
    ta0, tb0 = src["jc"][0][0], rep["jc"][0][0]
    dts = [((b[k + shift][0] - tb0) - (a[k][0] - ta0)) / 1e6 for k in matched]
    differ = sum(1 for k in matched if a[k][1] != b[k + shift][1])
    letters = lambda t: {o: sum(1 for _, _, x, _ in t["jc"] if x == o) for o in "ACRSP" if any(x == o for _, _, x, _ in t["jc"])}
    pairs = list(zip(src["frames"], rep["frames"]))
    action = [gap(src["relay"].at(x), rep["relay"].at(y)) for x, y in pairs]
    received = [gap(src["received"].at(x), rep["received"].at(y)) for x, y in pairs]
    tape = [gap(src["received"].at(x), rep["received"].at(rep["first"] + (x - src["first"]))) for x in src["frames"]]
    return {
        "n": (len(src["jc"]), len(rep["jc"])), "matched": len(matched), "identical": identical, "shift": shift,
        "lost": (len(b) - len(matched), len(a) - len(matched)),  # in the source, in the replay
        "dt_p50": statistics.median(dts) if dts else float("nan"), "dt_p95": pct([abs(d) for d in dts], 95),
        "outcome": (letters(src), letters(rep)), "differ": differ,
        "received": received, "tape": tape, "action": action, "frames": (len(src["frames"]), len(rep["frames"])),
    }


def summary(xs):
    if not xs:
        return "-"
    over = sum(1 for x in xs if x > 20) / len(xs)
    return f"{pct(xs, 50):.0f} / {pct(xs, 95):.0f} / {max(xs):.0f}, {100 * over:.0f}% over 20"


def runs_text(runs):
    if not runs:
        return "none"
    longest = max(runs, key=lambda r: r[1] - r[0])
    return f"{len(runs)} run(s), longest {longest[1] - longest[0] + 1} commands ({longest[2]:.2f}-{longest[3]:.2f} s)"


def dataset_actions(path):
    """The dataset's own action per episode and frame, {episode: {frame: [values]}}, or None without pyarrow."""
    try:
        import glob
        import pyarrow.parquet as pq
    except ImportError:
        return None
    out = {}
    for f in sorted(glob.glob(os.path.join(path, "data", "*", "*.parquet"))):
        t = pq.read_table(f, columns=["episode_index", "frame_index", "action"]).to_pydict()
        for e, k, a in zip(t["episode_index"], t["frame_index"], t["action"]):
            out.setdefault(e, {})[k] = a
    return out


def main(args):
    takes, episodes, actions = [], {}, {}
    for path in args:
        path = os.path.expanduser(path)
        listing = os.path.join(path, "meta", "teleop", "takes.jsonl")
        if os.path.exists(listing):
            stored = dataset_actions(path)
            if stored is not None:
                actions.update(stored)
            for line in open(listing):
                j = json.loads(line)
                takes.append(j["take"])
                episodes[j["take"]] = j["episode"]
        else:
            takes.append(path)
    data = {}
    for t in takes:
        try:
            data[t] = read(t)
        except (OSError, IndexError, KeyError, ValueError) as e:
            print(f"{t}: unreadable ({e})")
    name = lambda t: (f"ep {episodes[t]} " if t in episodes else "") + os.path.basename(t)
    sources = {d["rp"][1]: t for t, d in data.items() if d["rp"] and d["rp"][2] == "source"}
    pairs = [(sources.get(d["rp"][1]), t) for t, d in data.items() if d["rp"] and d["rp"][2] == "replay"]
    every = {"received": [], "tape": [], "action": [], "dataset": []}
    worst, totals = [], {"n": 0, "matched": 0, "identical": 0, "lost": 0}
    for s, r in pairs:
        if s is None:
            print(f"{name(r)}: group {data[r]['rp'][1]} has no source here")
            continue
        c = compare(data[s], data[r])
        es, er = episodes.get(s), episodes.get(r)
        c["dataset"] = ([max(abs(x - y) for x, y in zip(actions[es][k], actions[er][k])) for k in sorted(actions[er]) if k in actions[es]]
                        if es in actions and er in actions else [])
        for k in every:
            every[k] += c[k]
        totals["n"] += c["n"][1]
        totals["matched"] += c["matched"]
        totals["identical"] += c["identical"]
        totals["lost"] += sum(c["lost"])
        worst.append((pct(c["action"], 95), pct(c["tape"], 95), name(r)))
        print(f"group {data[r]['rp'][1]} {data[r]['rp'][3]}: {name(s)} -> {name(r)}; contact records (WC) {data[s]['contacts']} -> {data[r]['contacts']}")
        print(f"  received: {c['n'][0]} / {c['n'][1]} commands, {c['matched']} matched (shift {c['shift']}), {c['identical']} identical;"
              f" lost {c['lost'][0]} in the source, {c['lost'][1]} in the replay; dt p50 {c['dt_p50']:+.1f} ms, |dt| p95 {c['dt_p95']:.1f} ms")
        print(f"  outcome: source {c['outcome'][0]}, refused or clamped {runs_text(data[s]['runs'])}; replay {c['outcome'][1]}, refused or clamped {runs_text(data[r]['runs'])};"
              f" outcomes differ at {c['differ']} matched commands")
        print(f"  per frame ({c['frames'][0]} / {c['frames'][1]} frames), units p50 / p95 / max: received {summary(c['received'])};"
              f" tape {summary(c['tape'])}; action {summary(c['action'])}"
              + (f"; dataset {summary(c['dataset'])}" if c["dataset"] else ""))
    print()
    print(f"{len(pairs)} replay pairs: {totals['identical']} of {totals['matched']} matched commands identical, {totals['lost']} lost;"
          f" {len(every['action'])} paired frames")
    print(f"  received per frame (commands.parquet servo_*: what a replay matches on): {summary(every['received'])}")
    print(f"  the same at the source's frame times (no frame phase):                 {summary(every['tape'])}")
    print(f"  action per frame (RO, commands.parquet sent.*: differs where outcomes do): {summary(every['action'])}")
    if every["dataset"]:
        print(f"  the dataset's own action column:                                       {summary(every['dataset'])}")
    print("  pairs by p95, action / tape: " + ", ".join(f"{n} {p:.0f}/{q:.0f}" for p, q, n in sorted(worst, reverse=True)))
    for letter, word in (("R", "refused"), ("C", "clamped")):
        counts = [(sum(1 for _, _, o, _ in d["jc"] if o == letter), len(d["jc"])) for d in data.values()]
        if not counts:
            continue
        hit, total = sum(r for r, _ in counts), sum(n for _, n in counts)
        print(f"  {word} commands: {hit} of {total} ({100 * hit / max(1, total):.1f}%) in {sum(1 for r, _ in counts if r)} of"
              f" {len(counts)} takes; per take p50 / p90 / max {pct([r for r, _ in counts], 50)} / {pct([r for r, _ in counts], 90)} /"
              f" {max(r for r, _ in counts)}")


if __name__ == "__main__":
    main(sys.argv[1:])
