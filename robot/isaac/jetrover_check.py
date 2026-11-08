"""The JetRover's arm in Isaac, end to end (docs/adr/0034 J1, ``just isaac-jetrover-check``): starts the bridge and talks to it
over relay protocol v5 as the JetRover's RobotHost would. Run from robot/isaac on the GPU machine; prints one ``RESULT {...}``
line and exits non-zero when any check fails or any number is missing.

1. Feedback arrives about every 50 ms, motors 1 to 5 valid, the gripper (10) never.
2. At home, each reading is near its home pulse (how far gravity pulls it is reported).
3. A move of the elbow by 100 pulses at 400 pulses/s gets there at the commanded pace: past halfway near 0.25 s, within
   3 pulses of the target by 0.8 s.
4. Torque off: the shoulder falls (more than 20 pulses within 2 s). Torque back on: it holds where it is (under 4 pulses
   of drift over the next second).
5. The bridge's late steps, at most 1 a second.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from teleop_isaac import servo  # noqa: E402
from teleop_isaac.jetrover_bridge import HOME  # noqa: E402


def main() -> int:
    a = argparse.ArgumentParser()
    a.add_argument("--python", default=os.path.expanduser("~/isaacsim/venv/bin/python"))
    a.add_argument("--cpus", default="2-5")
    args = a.parse_args()
    model = servo.SimModel.load()
    rp = servo.relay_protocol()
    home = servo.home_pulses(model, HOME)
    user = os.environ.get("USER", "me")
    relay = f"/tmp/teleop-jetrover-check-{user}.sock"
    mine = relay + ".robothost"
    log_path = os.path.expanduser("~/isaacsim/jetrover-check-bridge.log")
    log = open(log_path, "w")
    bridge = subprocess.Popen(["taskset", "-c", args.cpus, args.python, "-m", "teleop_isaac.jetrover_bridge", "--socket", relay, "--seconds", "120"],
                              cwd=HERE, stdout=log, stderr=subprocess.STDOUT)
    out, checks = {}, {}
    try:
        for _ in range(240):
            if bridge.poll() is not None or "listening on" in open(log_path, errors="replace").read():
                break
            time.sleep(1)
        if not os.path.exists(relay):
            print("RESULT " + json.dumps({"passes": False, "missing": ["bridge"]}))
            print("\n".join(open(log_path, errors="replace").read().splitlines()[-30:]), file=sys.stderr)
            return 1
        if os.path.exists(mine):
            os.unlink(mine)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.bind(mine)
        sock.settimeout(0.002)
        feeds = []  # (arrival s, {motor: (valid, pulse)})
        t0 = time.monotonic()

        def send(data: bytes) -> None:
            sock.sendto(data, relay)

        def command(pulses, pps=400.0):
            send(encode_command([(m, float(p), float(pps)) for m, p in pulses.items()]))

        def pump(seconds: float, resend=None) -> None:
            end = time.monotonic() + seconds
            last = 0.0
            while time.monotonic() < end:
                if resend is not None and time.monotonic() - last >= 0.02:  # RobotHost re-sends every loop
                    resend()
                    last = time.monotonic()
                try:
                    d = sock.recv(4096)
                except (socket.timeout, BlockingIOError):
                    continue
                if rp.kind_of(d) == rp.KIND_FEEDBACK:
                    entries = decode_feedback(rp, d)
                    if entries is not None:
                        feeds.append((time.monotonic() - t0, {m: (v, p) for m, v, p in entries}))

        target = dict(home)
        pump(3.0, resend=lambda: command(target))
        # 1. cadence and validity
        recent = [f for f in feeds if f[0] > 1.0]
        if len(recent) >= 2:
            out["feedback_hz"] = round((len(recent) - 1) / (recent[-1][0] - recent[0][0]), 2)
            checks["feedback_rate"] = 17.0 <= out["feedback_hz"] <= 23.0
            last = recent[-1][1]
            checks["arm_motors_valid"] = all(last.get(m, (False, 0))[0] for m in (1, 2, 3, 4, 5))
            checks["gripper_never_valid"] = all(not f[1].get(10, (False, 0))[0] for f in feeds)
            # 2. at home
            out["home_error_pulses"] = {m: round(last[m][1] - home[m], 1) for m in (1, 2, 3, 4, 5)}
            checks["at_home"] = all(abs(v) <= 15 for v in out["home_error_pulses"].values())
        else:
            out["feedback_hz"] = None

        # 3. a move at the commanded pace
        start3 = feeds[-1][1][3][1] if feeds else home[3]
        target[3] = home[3] + 100
        t_move = time.monotonic() - t0
        pump(1.0, resend=lambda: command(target, 400.0))
        moved = [(t - t_move, f[3][1]) for t, f in feeds if t >= t_move and f.get(3, (False, 0))[0]]
        half = next((dt for dt, p in moved if p - start3 >= 50), None)
        out["half_way_s"] = round(half, 3) if half is not None else None
        at = next((dt for dt, p in moved if abs(p - target[3]) <= 3), None)
        out["arrived_s"] = round(at, 3) if at is not None else None
        out["move_final_error_pulses"] = round(moved[-1][1] - target[3], 1) if moved else None
        checks["move_pace"] = half is not None and 0.15 <= half <= 0.40
        checks["move_arrives"] = at is not None and at <= 0.8

        # 4. torque off: it falls; back on: it holds
        send(rp.encode_torque(False))
        t_off = time.monotonic() - t0
        before = feeds[-1][1][2][1] if feeds else None
        pump(2.0)
        after = [f[2][1] for t, f in feeds if t > t_off and f.get(2, (False, 0))[0]]
        out["shoulder_fall_pulses"] = round(max(abs(p - before) for p in after), 1) if after and before is not None else None
        checks["torque_off_falls"] = out["shoulder_fall_pulses"] is not None and out["shoulder_fall_pulses"] > 20
        send(rp.encode_torque(True))
        pump(0.5)
        t_on = time.monotonic() - t0
        pump(1.0)
        held = [f[2][1] for t, f in feeds if t > t_on and f.get(2, (False, 0))[0]]
        out["held_drift_pulses"] = round(max(held) - min(held), 1) if held else None
        checks["torque_on_holds"] = out["held_drift_pulses"] is not None and out["held_drift_pulses"] < 4
        out["seconds"] = round(time.monotonic() - t0, 1)
    finally:
        bridge.terminate()
        try:
            bridge.wait(30)
        except subprocess.TimeoutExpired:
            bridge.kill()
        log.close()
        if os.path.exists(mine):
            os.unlink(mine)

    text = open(log_path, errors="replace").read().splitlines()
    out["bridge"] = [line for line in text if line.startswith("[jetrover]")][:6]
    stopped = [line for line in text if "stopped after" in line]
    if stopped:
        parts = stopped[-1].split()
        steps, late = int(parts[3]), int(parts[5])
        out["late_steps"], out["steps"] = late, steps
        checks["late_ok"] = late <= max(1.0, steps / 1000.0)
    else:
        out["late_steps"] = None
    out["checks"] = checks
    expected = 9
    out["missing"] = sorted(k for k, v in out.items() if v is None) + ([f"{expected - len(checks)} checks"] if len(checks) < expected else [])
    out["passes"] = not out["missing"] and all(checks.values())
    print("RESULT " + json.dumps(out, sort_keys=True))
    if not out["passes"]:
        print("\n".join(text[-30:]), file=sys.stderr)
    return 0 if out["passes"] else 1


def encode_command(targets) -> bytes:
    """RobotHost's side of a command (RelayProtocol.EncodeCommand): the ROS relay only decodes them."""
    import struct

    data = struct.pack("<BBB", 5, 1, len(targets))
    for m, p, s in targets:
        data += struct.pack("<Bff", m, p, s)
    return data


def decode_feedback(rp, data: bytes):
    """RobotHost's side of feedback (RelayProtocol.TryDecodeFeedback)."""
    import struct

    if len(data) < rp.HEADER_SIZE:
        return None
    _, _, count = struct.unpack_from("<BBB", data, 0)
    if len(data) < rp.feedback_encoded_size(count):
        return None
    out = []
    for k in range(count):
        m, v, p = struct.unpack_from("<Bbf", data, rp.HEADER_SIZE + k * rp.FEEDBACK_TUPLE_SIZE)
        out.append((m, bool(v), p))
    return out


if __name__ == "__main__":
    sys.exit(main())
