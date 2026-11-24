"""How exactly each counterfactual replay repeats its source's commands, stage by stage (docs/adr/0038 decision 2), read-only.

    ssh andrei@10.188.58.133 'python3 - <dataset or take dir>...' < scripts/gpu/replay_pairs.py
    ssh andrei@10.188.58.133 '~/isaacsim/lerobot-venv/bin/python - <dataset>' < scripts/gpu/replay_pairs.py   (adds `dataset`)

A dataset (a folder with meta/teleop/takes.jsonl) stands for its takes, with their episode numbers. Takes are paired by their
`RP` record: each replay with its group's source. Standard library only.

Per pair it compares, at each stage a command passes (trainer/contract/SESSION_FORMAT.md):

- **sent**: every `JC`, whatever its outcome: what joint-drive sent and RobotHost received. The two takes' commands are matched
  by joint-drive's sequence number counted from each take's first (shifted by up to 3 when that does better: a lost first
  datagram). `identical` counts matched commands whose targets are equal to the last digit; `lost` the commands one take has
  and the other does not (UDP datagrams that never reached RobotHost); `dt` how much later than its source's the replay's
  command arrived after the take's first, ms.
- **outcome**: `JC`'s outcome letters (A accepted, R refused, S stale, P parked) in each take, the runs of refused commands
  (from-to seconds into the take), and at how many matched commands the outcomes differ. A refused command never reaches the
  relay (`RO`): RobotHost's collision backstop (docs/adr/0020) refuses a command whose servo path from where the arm is
  measured would meet the robot's own model, and the arm keeps its last accepted target.
- **action per frame**, by the episode frame's index (frames as robot/lerobot/teleop_lerobot/takes.py picks them: a 15 fps
  grid over the recorded frames between the first and the last command, each with every motor's value), the largest
  difference over the six motors between the replay's frame k and its source's, in servo units, p50 / p95 / max, and the
  share over 20 units:
  - `relay`: the newest relay target (`RO`) at the frame's time, over the frames between the first and last accepted command:
    the dataset's `action` until 2026-11-24;
  - `sent`: the newest command sent, accepted or refused, in relay units (every `RO`, and every refused `JC` turned into pulses
    as the plant would have; not a parked one, while RobotHost's own park is the `RO`), over the frames between the first
    and last command accepted or refused: the dataset's `action` since 2026-11-24 (docs/adr/0038 "M, motion");
  - `tape`: the same, both read at the source's frame times counted from each take's first such command: what is left
    without the frames' own phase against the commands (the renderer's frame clock runs free).
  - `dataset`: the dataset's own `action` column (with pyarrow, as the lerobot venv has it): equal to `sent` when the dataset
    was built since 2026-11-24, to `relay` before.
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
    relay = Track(ro)
    sent = Track(ro + [(t, pulses(targets)) for t, _, o, targets in jc if o == "R"])
    readings = {}
    for r in host["RI"]:
        for it in items(r[1]):
            if it[1] == "1":
                readings.setdefault(int(it[0]), []).append(int(r[0]))
    accepted = [t for t, _, o, _ in jc if o == "A"]
    driven = [t for t, _, o, _ in jc if o in ("A", "R")]
    recorded = []
    with open(os.path.join(take, "isaac.jlog"), errors="replace") as f:
        for line in f:
            if line.startswith("VF|"):
                r = line.rstrip("\n").split("|")[1:]
                if "B" not in r[3]:
                    recorded.append(int(r[2]))
    sensed = lambda t: all(readings.get(m) and readings[m][0] <= t for m in SENSED)
    # Until 2026-11-24 (relay): frames between the first and last accepted command, every motor with a relay target.
    old = grid(recorded, accepted, lambda t: sensed(t) and all(m in relay.at(t) for m in MOTORS))
    # Since (sent): between the first and last command accepted or refused, every motor with an action, the gripper a relay target.
    new = grid(recorded, driven, lambda t: sensed(t) and all(m in sent.at(t) for m in MOTORS) and GRIPPER in relay.at(t))
    first = min(driven) if driven else 0
    runs, start = [], None
    t_first = jc[0][0] if jc else 0
    for k, (t, _, o, _) in enumerate(jc + [(0, 0, "", "")]):
        if o == "R" and start is None:
            start = k
        elif o != "R" and start is not None:
            runs.append((start, k - 1, (jc[start][0] - t_first) / 1e9, (jc[k - 1][0] - t_first) / 1e9))
            start = None
    return {"rp": rp, "jc": jc, "relay": relay, "sent": sent, "old": old, "frames": new, "first": first, "runs": runs,
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
    letters = lambda t: {o: sum(1 for _, _, x, _ in t["jc"] if x == o) for o in "ARSP" if any(x == o for _, _, x, _ in t["jc"])}
    relay = [gap(src["relay"].at(x), rep["relay"].at(y)) for x, y in zip(src["old"], rep["old"])]
    sent = [gap(src["sent"].at(x), rep["sent"].at(y)) for x, y in zip(src["frames"], rep["frames"])]
    tape = [gap(src["sent"].at(x), rep["sent"].at(rep["first"] + (x - src["first"]))) for x in src["frames"]]
    return {
        "n": (len(src["jc"]), len(rep["jc"])), "matched": len(matched), "identical": identical, "shift": shift,
        "lost": (len(b) - len(matched), len(a) - len(matched)),  # in the source, in the replay
        "dt_p50": statistics.median(dts) if dts else float("nan"), "dt_p95": pct([abs(d) for d in dts], 95),
        "outcome": (letters(src), letters(rep)), "differ": differ,
        "relay": relay, "sent": sent, "tape": tape, "frames": (len(src["frames"]), len(rep["frames"])),
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
    every = {"relay": [], "sent": [], "tape": [], "dataset": []}
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
        worst.append((pct(c["relay"], 95), pct(c["sent"], 95), name(r)))
        print(f"group {data[r]['rp'][1]} {data[r]['rp'][3]}: {name(s)} -> {name(r)}; contact records (WC) {data[s]['contacts']} -> {data[r]['contacts']}")
        print(f"  sent:    {c['n'][0]} / {c['n'][1]} commands, {c['matched']} matched (shift {c['shift']}), {c['identical']} identical;"
              f" lost {c['lost'][0]} in the source, {c['lost'][1]} in the replay; dt p50 {c['dt_p50']:+.1f} ms, |dt| p95 {c['dt_p95']:.1f} ms")
        print(f"  outcome: source {c['outcome'][0]}, refused {runs_text(data[s]['runs'])}; replay {c['outcome'][1]}, refused {runs_text(data[r]['runs'])};"
              f" outcomes differ at {c['differ']} matched commands")
        print(f"  action per frame ({c['frames'][0]} / {c['frames'][1]} frames), units p50 / p95 / max: relay {summary(c['relay'])};"
              f" sent {summary(c['sent'])}; tape {summary(c['tape'])}"
              + (f"; dataset {summary(c['dataset'])}" if c["dataset"] else ""))
    print()
    print(f"{len(pairs)} replay pairs: {totals['identical']} of {totals['matched']} matched commands identical, {totals['lost']} lost;"
          f" {sum(len(every['relay']) for _ in [0])} paired frames")
    print(f"  action per frame, relay (RO: the dataset's until 2026-11-24): {summary(every['relay'])}")
    print(f"  action per frame, sent (since 2026-11-24):                    {summary(every['sent'])}")
    print(f"  the same at the source's frame times (no frame phase):        {summary(every['tape'])}")
    if every["dataset"]:
        print(f"  the dataset's own action column:                             {summary(every['dataset'])}")
    print("  pairs by p95, relay / sent: " + ", ".join(f"{n} {p:.0f}/{q:.0f}" for p, q, n in sorted(worst, reverse=True)))
    refusing = [(sum(1 for _, _, o, _ in d["jc"] if o == "R"), len(d["jc"]), name(t)) for t, d in data.items()]
    total_r, total_n = sum(r for r, _, _ in refusing), sum(n for _, n, _ in refusing)
    print(f"  refused commands: {total_r} of {total_n} ({100 * total_r / max(1, total_n):.1f}%) in {sum(1 for r, _, _ in refusing if r)} of"
          f" {len(refusing)} takes; per take p50 / p90 / max {pct([r for r, _, _ in refusing], 50)} / {pct([r for r, _, _ in refusing], 90)} /"
          f" {max(r for r, _, _ in refusing)}")


if __name__ == "__main__":
    main(sys.argv[1:])
