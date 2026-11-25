"""Synthetic takes, written to the session format (trainer/contract/SESSION_FORMAT.md) byte for byte: jlog headers and
trailers with their sha256, TFRM sidecars, files.sha256."""

import hashlib
import json
import math
import os
import struct
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MS = 1_000_000

# Each record's fields that are times on the session's clock (positions in a row, its tag first; trainer/contract/
# SESSION_FORMAT.md): what a take's ``at_s`` moves. A record named in neither table cannot be moved, and says so.
TIMES = {"JC": (1, 2, 4), "RO": (1,), "RI": (1,), "WC": (1, 2), "WO": (1, 2), "WL": (1,), "WZ": (1,), "RP": (1,), "SE": (1,),
         "MV": (1,), "WP": (1,), "WE": (1,), "QS": (1, 2), "AO": (1,), "VF": (1, 3), "VQ": (2,), "EN": (1,), "ER": (1,),
         "JK": (1,), "HK": (1,), "VP": (2,)}
UNTIMED = {"CM", "LK", "MT", "CL", "LT", "CJ", "OA", "SZ", "SC", "VS", "VW"}


def moved(row, ns):
    """``row`` with its times ``ns`` later (:data:`TIMES`)."""
    if not ns or row[0] in UNTIMED:
        return list(row)
    if row[0] not in TIMES:
        raise ValueError(f"a {row[0]} record's times are not known here: add them to TIMES")
    return [int(v) + ns if i in TIMES[row[0]] else v for i, v in enumerate(row)]


def write_jlog(path, writer, sid, rows, dropped=0):
    lines = [f"JSES|1|{writer}|1000000000|CLOCK_MONOTONIC|{sid}"] + ["|".join(str(x) for x in r) for r in rows]
    body = ("\n".join(lines) + "\n").encode("ascii")
    with open(path, "wb") as f:
        f.write(body + f"END|{len(rows)}|{dropped}|{hashlib.sha256(body).hexdigest()}\n".encode("ascii"))


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


def frames_log(take, writer, folder, sid, cameras, vfs, image=lambda cam, n: f"{cam}-{n}".encode(), extra=(), vq=None, cm=None,
               shift=0, views=(), payload=None, view_pose=None):
    """``vfs``: (t ns, frame, stateSensed, flags). Every camera's rgb, one record per frame. ``extra``: more rows (``[tag,
    ...]``) after the cameras'; ``vq(n, sensed)``: frame n's posed joints ({joint: value}), written as a ``VQ`` before its
    ``VF``; ``cm(camera)``: that camera's ``CM`` row instead of the default. ``shift``: every time written that many ns later
    (``vq`` is given the frame's time before it). ``views``: view cameras (docs/adr/0038 "V, views"), no ``CM`` row: each
    frame also carries their rgb, depth and mask parts and a ``VP`` with their poses, as the renderer writes them.
    ``payload(camera, stream, n)``: a frame's depth or mask bytes (and a view's rgb), and every camera then records its
    depth and mask too; ``view_pose(view, n)``: a view's ``VP`` pose at frame n (x y z qw qx qy qz)."""
    sidecars = {c: Sidecar(os.path.join(take, folder, c + ".rgb.frames")) for c in cameras}
    if payload is not None:
        sidecars.update({f"{c}.{st}": Sidecar(os.path.join(take, folder, f"{c}.{st}.frames")) for c in cameras for st in ("depth", "mask")})
    view_sidecars = {f"{v}.{st}": Sidecar(os.path.join(take, folder, f"{v}.{st}.frames")) for v in views for st in ("rgb", "depth", "mask")}
    rows = [(cm(c) if cm else None) or ["CM", c, 64, 48, 1, 1, 32, 24, 0.01, 6, 0, 0, 0, 1, 0, 0, 0, "", "rgb;depth;mask", "base"]
            for c in cameras]
    rows.extend(moved(r, shift) for r in extra)
    for t, n, sensed, flags in vfs:
        parts = []
        for c in cameras:
            off, length = sidecars[c].add(image(c, n))
            parts.append(f"{c}.rgb:{off}:{length}")
            if payload is not None:
                for st in ("depth", "mask"):
                    off, length = sidecars[f"{c}.{st}"].add(payload(c, st, n))
                    parts.append(f"{c}.{st}:{off}:{length}")
        for name, sc in view_sidecars.items():
            off, length = sc.add(payload(*name.split("."), n) if payload is not None else f"{name}-{n}".encode())
            parts.append(f"{name}:{off}:{length}")
        if vq is not None:
            rows.append(moved(["VQ", n, sensed, ";".join(f"{k}:{v}" for k, v in sorted(vq(n, sensed).items()))], shift))
        if views:
            pose = view_pose or (lambda v, k: (1.0, 0.1 * k, 0.8, 1.0, 0.0, 0.0, 0.0))
            rows.append(moved(["VP", n, sensed, ";".join(f"{v}:" + ":".join(repr(float(x)) for x in pose(v, n)) for v in views)],
                              shift))
        rows.append(moved(["VF", t, n, sensed, flags, ";".join(parts)], shift))
    for s in list(sidecars.values()) + list(view_sidecars.values()):
        s.close()
    write_jlog(os.path.join(take, writer + ".jlog"), writer, sid, rows)


JETROVER_PROFILE = os.path.join(REPO, "core", "RobotProfiles", "jetrover.json")


def relay_pulse(motor, value):
    """What RobotHost sends the relay for one target of a JetRover joint command (GenericArmPlantConfig.Default and
    core/RobotProfiles/jetrover.json): 1000 pulses per 240 degrees from 500, the shoulder's mounting offset added; the gripper's
    fraction from 116 (open) to 645 (closed)."""
    if motor == 10:
        return 116.0 + value * (645.0 - 116.0)
    return 500.0 + (value + (-0.64494798 if motor == 2 else 0.0)) * 1000.0 / math.radians(240.0)


def jetrover_take(root, name="20261112T100000Z-take001", scenario="reach", source="isaac", gap=None, image=None,
                  cameras=("third-left", "wrist"), extra=(), jitter=0.0, isaac_extra=(), vq=None, cm=None, scene=None,
                  session=None, dropped=0, at_s=0.0, commands=None, relay=relay_pulse, host_check=None, views=(), payload=None,
                  view_pose=None):
    """A JetRover take: commands accepted from 1.0 to 3.0 s, RO and RI every 20 ms, frames at 30 fps from 0.5 to
    3.5 s (the first two with B). ``gap``: (from, to) seconds with no frame recorded. ``extra``: more robothost.jlog rows
    (``[tag, t, ...]``), such as contacts (WC) and object poses (WO). ``jitter``: each frame's time wobbles by up to that
    many ms (seeded), as the twin's renderer's do (sd ~3 ms, 2026-11-15). ``isaac_extra``, ``vq``, ``cm``: the frames log's
    (:func:`frames_log`). ``scene``: written as scene.json. ``session``: more session.json keys. ``dropped``: the
    robothost.jlog trailer's dropped records. ``at_s``: the take recorded that many seconds later, every time in it with it
    (``extra``'s and ``isaac_extra``'s too, as given from 0): takes of one dataset must not overlap on the session's clock
    (docs/adr/0038 L), and two at the same ``at_s`` do.

    ``commands``: the JC records instead, ``(ms, outcome, {motor: value})`` in order, each with the next sequence number and
    its time as its capture stamp; an accepted one's RO 30 us later, ``relay(motor, value)`` for each target, as RobotHost
    writes it (no other RO); and the take's robot-profile.json, as RobotHost's auto-record copies it. A clamped one (``C``)
    is ``(ms, "C", {motor: value}, ({motor: value sent}, alpha))``: its RO the targets sent, then its ``JK``.
    ``host_check``: ``(start, onFail, version)`` written as the session's ``HK`` preamble at 0.5 s (none: a RobotHost that
    refused)."""
    shift = round(at_s * 1e9)
    take = os.path.join(root, scenario, name)
    os.makedirs(take)
    sid = session_json(take, "jetrover-session", 1, source, scenario)
    if session:
        with open(os.path.join(take, "session.json"), encoding="utf-8") as f:
            meta = json.load(f)
        meta.update(session)
        with open(os.path.join(take, "session.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f)
    if scene is not None:
        with open(os.path.join(take, "scene.json"), "w", encoding="utf-8") as f:
            json.dump(scene, f)
    rows = []
    if commands is None:
        for k in range(0, 41):  # JC every 50 ms, 1.0..3.0 s
            t = (1000 + 50 * k) * MS
            rows.append(["JC", t, t, k, t, "A", "1:0.1:1"])
    else:
        for seq, (ms, outcome, values, *clamp) in enumerate(commands):
            t = round(ms * MS)
            rows.append(["JC", t, t, seq, t, outcome, ";".join(f"{m}:{v!r}:3.8" for m, v in values.items())])
            if outcome == "A":
                rows.append(["RO", t + 30_000, ";".join(f"{m}:{relay(m, v)!r}:907.1831" for m, v in values.items())])
            elif outcome == "C":
                sent, alpha = clamp[0]
                rows.append(["RO", t + 30_000, ";".join(f"{m}:{relay(m, v)!r}:907.1831" for m, v in sent.items())])
                rows.append(["JK", t + 40_000, seq, alpha])
        with open(JETROVER_PROFILE, encoding="utf-8") as f, open(os.path.join(take, "robot-profile.json"), "w", encoding="utf-8") as g:
            g.write(f.read())
    for k in range(0, 116):  # 0.9..3.2 s
        t = (900 + 20 * k) * MS
        if commands is None:
            ro = ";".join(f"{m}:{500 + m + k}.5:300" for m in (1, 2, 3, 4, 10, 5))
            rows.append(["RO", t, ro])
        ri = ";".join(f"{m}:{0 if (m == 3 and k == 60) else 1}:{400 + m + k}" for m in (1, 2, 3, 4, 5, 10))
        rows.append(["RI", t + 5 * MS, ri])
    rows.extend(list(r) for r in extra)
    if host_check is not None:
        rows.append(["HK", 500 * MS] + list(host_check))
    rows.sort(key=lambda r: int(r[1]))
    rows = [moved(r, shift) for r in rows]
    write_jlog(os.path.join(take, "robothost.jlog"), "robothost", sid, rows, dropped=dropped)
    vfs = []
    wobble = __import__("random").Random(7)
    for n in range(0, 91):
        sensed = 500 * MS + round(n * 1e9 / 30) + round(wobble.uniform(-jitter, jitter) * MS)
        if gap and gap[0] * 1e9 < sensed < gap[1] * 1e9:
            continue
        vfs.append((sensed + 100 * MS, n, sensed, "B" if n < 2 else ""))
    folder = "camera" if source == "robot" else "isaac"
    frames_log(take, "camera" if source == "robot" else "isaac", folder, sid,
               ["front"] if source == "robot" else list(cameras), vfs,
               **({"image": image} if image else {}), extra=isaac_extra, vq=vq, cm=cm, shift=shift, views=views, payload=payload,
               view_pose=view_pose)
    finish(take)
    return take


def fr3_sent(profile, motor, value):
    """What the FR3 Duo's plant sends the arm backend for one target (SerialChainPlant): within the joint's soft limits, a
    hand's fraction within 0..1."""
    for arm in profile["arms"]:
        if arm["gripper"]["motorId"] == motor:
            return min(1.0, max(0.0, value))
        for j in arm["joints"]:
            if j["motorId"] == motor:
                return min(j["softUpper"], max(j["softLower"], value))
    raise KeyError(motor)


def fr3_take(root, name="20261112T110000Z-take001", scenario="handover", commands=None):
    """An FR3 Duo take: QS every 4 ms, AO every 20 ms, commands 1.0..2.0 s, frames at 30 fps on QS times.

    ``commands``: the JC records instead, ``(ms, outcome, {motor: value})`` in order, each with the next sequence number and
    its time as its capture stamp; an accepted one's AO at the same time, its values within the soft limits (a hand's
    within 0..1), as SerialChainHost writes it (no other AO)."""
    take = os.path.join(root, scenario, name)
    os.makedirs(take)
    sid = session_json(take, "teleop-session", 2, "isaac", scenario, "")
    with open(os.path.join(REPO, "core", "RobotProfiles", "fr3-duo.json"), encoding="utf-8") as f:
        profile = json.load(f)
    with open(os.path.join(take, "robot-profile.json"), "w", encoding="utf-8") as f:
        json.dump(profile, f)
    motors = list(range(1, 9)) + list(range(11, 19))
    rows = []
    if commands is None:
        for k in range(0, 21):
            t = (1000 + 50 * k) * MS
            rows.append(["JC", t, t, k, t, "A", "1:0.1:1"])
    else:
        for seq, (ms, outcome, values) in enumerate(commands):
            t = round(ms * MS)
            rows.append(["JC", t, t, seq, t, outcome, ";".join(f"{m}:{v!r}:1" for m, v in values.items())])
            if outcome == "A":
                rows.append(["AO", t, ";".join(f"{m}:{fr3_sent(profile, m, v)!r}" for m, v in values.items())])
    for k in range(0, 700):  # QS 0..2.8 s
        sensed = 4 * k * MS
        qs = ";".join(f"{m}:{0.5 if m in (8, 18) else 0.01 * m + 0.001 * k}" for m in motors)
        rows.append(["QS", sensed + MS, sensed, qs])
    for k in range(0, 120 if commands is None else 0):
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
