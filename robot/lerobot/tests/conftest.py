"""Synthetic takes, written to the session format (trainer/contract/SESSION_FORMAT.md) byte for byte: jlog headers and
trailers with their sha256, TFRM sidecars, files.sha256."""

import hashlib
import json
import os
import struct
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MS = 1_000_000


def write_jlog(path, writer, sid, rows):
    lines = [f"JSES|1|{writer}|1000000000|CLOCK_MONOTONIC|{sid}"] + ["|".join(str(x) for x in r) for r in rows]
    body = ("\n".join(lines) + "\n").encode("ascii")
    with open(path, "wb") as f:
        f.write(body + f"END|{len(rows)}|0|{hashlib.sha256(body).hexdigest()}\n".encode("ascii"))


class Sidecar:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, "wb")
        self.f.write(b"TFRM" + struct.pack("<HH", 1, 0))

    def add(self, payload):
        self.f.write(struct.pack("<I", len(payload)))
        offset = self.f.tell()
        self.f.write(payload)
        return offset, len(payload)

    def close(self):
        self.f.close()


def finish(take):
    """files.sha256, as SessionFinalizer writes it."""
    names = []
    for dirpath, _, files in os.walk(take):
        for name in files:
            if name != "files.sha256":
                names.append(os.path.relpath(os.path.join(dirpath, name), take).replace(os.sep, "/"))
    with open(os.path.join(take, "files.sha256"), "w", encoding="utf-8") as f:
        for name in sorted(names):
            with open(os.path.join(take, name), "rb") as g:
                f.write(f"{hashlib.sha256(g.read()).hexdigest()}  {name}\n")


def session_json(take, fmt, version, source, scenario="reach", description="reach the red cube"):
    sid = os.path.basename(take)
    with open(os.path.join(take, "session.json"), "w", encoding="utf-8") as f:
        json.dump({"format": fmt, "version": version, "id": sid, "source": source, "label": f"scenario-{scenario}",
                   "scenario": {"name": scenario, "take": 1, "description": description}}, f)
    return sid


def frames_log(take, writer, folder, sid, cameras, vfs, image=lambda cam, n: f"{cam}-{n}".encode()):
    """``vfs``: (t ns, frame, stateSensed, flags). Every camera's rgb, one record per frame."""
    sidecars = {c: Sidecar(os.path.join(take, folder, c + ".rgb.frames")) for c in cameras}
    rows = [["CM", c, 64, 48, 1, 1, 32, 24, 0.01, 6, 0, 0, 0, 1, 0, 0, 0, "", "rgb;depth;mask", "base"] for c in cameras]
    for t, n, sensed, flags in vfs:
        parts = []
        for c in cameras:
            off, length = sidecars[c].add(image(c, n))
            parts.append(f"{c}.rgb:{off}:{length}")
        rows.append(["VF", t, n, sensed, flags, ";".join(parts)])
    for s in sidecars.values():
        s.close()
    write_jlog(os.path.join(take, writer + ".jlog"), writer, sid, rows)


def jetrover_take(root, name="20261112T100000Z-take001", scenario="reach", source="isaac", gap=None, image=None,
                  cameras=("third-left", "wrist")):
    """A JetRover take: commands accepted from 1.0 to 3.0 s, RO and RI every 20 ms, frames at 30 fps from 0.5 to
    3.5 s (the first two with B). ``gap``: (from, to) seconds with no frame recorded."""
    take = os.path.join(root, scenario, name)
    os.makedirs(take)
    sid = session_json(take, "jetrover-session", 1, source, scenario)
    rows = []
    for k in range(0, 41):  # JC every 50 ms, 1.0..3.0 s
        t = (1000 + 50 * k) * MS
        rows.append(["JC", t, t, k, t, "A", "1:0.1:1"])
    for k in range(0, 116):  # 0.9..3.2 s
        t = (900 + 20 * k) * MS
        ro = ";".join(f"{m}:{500 + m + k}.5:300" for m in (1, 2, 3, 4, 10, 5))
        rows.append(["RO", t, ro])
        ri = ";".join(f"{m}:{0 if (m == 3 and k == 60) else 1}:{400 + m + k}" for m in (1, 2, 3, 4, 5, 10))
        rows.append(["RI", t + 5 * MS, ri])
    rows.sort(key=lambda r: int(r[1]))
    write_jlog(os.path.join(take, "robothost.jlog"), "robothost", sid, rows)
    vfs = []
    for n in range(0, 91):
        sensed = 500 * MS + round(n * 1e9 / 30)
        if gap and gap[0] * 1e9 < sensed < gap[1] * 1e9:
            continue
        vfs.append((sensed + 100 * MS, n, sensed, "B" if n < 2 else ""))
    folder = "camera" if source == "robot" else "isaac"
    frames_log(take, "camera" if source == "robot" else "isaac", folder, sid,
               ["front"] if source == "robot" else list(cameras), vfs,
               **({"image": image} if image else {}))
    finish(take)
    return take


def fr3_take(root, name="20261112T110000Z-take001", scenario="handover"):
    """An FR3 Duo take: QS every 4 ms, AO every 20 ms, commands 1.0..2.0 s, frames at 30 fps on QS times."""
    take = os.path.join(root, scenario, name)
    os.makedirs(take)
    sid = session_json(take, "teleop-session", 2, "isaac", scenario, "")
    with open(os.path.join(REPO, "core", "RobotProfiles", "fr3-duo.json"), encoding="utf-8") as f:
        profile = json.load(f)
    with open(os.path.join(take, "robot-profile.json"), "w", encoding="utf-8") as f:
        json.dump(profile, f)
    motors = list(range(1, 9)) + list(range(11, 19))
    rows = []
    for k in range(0, 21):
        t = (1000 + 50 * k) * MS
        rows.append(["JC", t, t, k, t, "A", "1:0.1:1"])
    for k in range(0, 700):  # QS 0..2.8 s
        sensed = 4 * k * MS
        qs = ";".join(f"{m}:{0.5 if m in (8, 18) else 0.01 * m + 0.001 * k}" for m in motors)
        rows.append(["QS", sensed + MS, sensed, qs])
    for k in range(0, 120):
        t = (900 + 20 * k) * MS
        rows.append(["AO", t, ";".join(f"{m}:{1 if m in (8, 18) else -0.01 * m - 0.001 * k}" for m in motors)])
    rows.sort(key=lambda r: int(r[1]))
    write_jlog(os.path.join(take, "robothost.jlog"), "robothost", sid, rows)
    vfs = [(round(n * 1e9 / 30) + 100 * MS, n, 4 * MS * round(n * 250 / 30), "") for n in range(0, 75)]
    vfs.append((2600 * MS, 99, 4 * MS * 200 + 1, ""))  # a stateSensed with no QS: left out
    frames_log(take, "isaac", "isaac", sid, ["third-left", "wrist-left"], vfs)
    finish(take)
    return take


@pytest.fixture
def scenarios(tmp_path):
    root = tmp_path / "scenarios"
    root.mkdir()
    return str(root)
