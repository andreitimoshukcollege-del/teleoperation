"""scripts/gpu/arm_response.py (docs/adr/0038 workstream R) on a synthetic session whose arm trails its command by 0.2 s: the
measures the ADR's numbers come from see that delay, its error and its stop."""

import importlib.util
import os

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def arm_response():
    spec = importlib.util.spec_from_file_location("arm_response", os.path.join(REPO, "scripts", "gpu", "arm_response.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


TPS = 1_000_000_000
HOME = {1: 500.0, 2: 650.0, 3: 125.0, 4: 625.0, 5: 500.0, 10: 200.0}
SPEED = 200.0   # units/s, the elbow's command
DELAY = 0.2     # s, the arm behind it


def elbow_command(t: float) -> float:
    """1 s still, 0.5 s at SPEED, then held 2 s."""
    return HOME[3] + SPEED * min(max(t - 1.0, 0.0), 0.5)


def write_session(d):
    lines = ["JSES|1|robothost|%d|CLOCK_MONOTONIC|synthetic" % TPS]
    t0 = 10 * TPS
    seq = 0
    for k in range(int(3.5 * 48)):  # commands at 48 Hz
        t = k / 48
        pulses = dict(HOME)
        pulses[3] = elbow_command(t)
        ts = t0 + int(t * TPS)
        angles = ";".join(f"{m}:{(p - 500) / 238.732407:.6f}:3.8" for m, p in pulses.items())
        lines.append(f"JC|{ts}|{ts}|{seq}|{ts}|A|{angles}")
        lines.append(f"RO|{ts + 1000}|" + ";".join(f"{m}:{p:.4f}:907.18" for m, p in pulses.items()))
        seq += 1
    for k in range(int(3.5 * 50)):  # readings at 50 Hz, whole pulses, DELAY late
        t = k / 50
        pulses = dict(HOME)
        pulses[3] = elbow_command(t - DELAY)
        ts = t0 + int(t * TPS) + 5000
        lines.append(f"RI|{ts}|" + ";".join(f"{m}:1:{round(p)}" for m, p in pulses.items()))
    lines.sort(key=lambda l: (int(l.split("|")[1]) if not l.startswith("JSES") else -1))
    with open(os.path.join(d, "robothost.jlog"), "w") as f:
        f.write("\n".join(lines) + "\n")


@pytest.fixture()
def session(tmp_path):
    write_session(str(tmp_path))
    return arm_response().Session(str(tmp_path))


def test_the_delay_between_command_and_reading_is_found(session):
    ar = arm_response()
    lag = ar.lag([session])
    assert lag["3"]["best_ms"] == pytest.approx(DELAY * 1000, abs=20)


def test_frames_over_the_threshold_are_the_moving_ones(session):
    ar = arm_response()
    eps = ar.session_episodes(session)
    t = ar.tracking(eps)
    # Trailing 0.2 s at 200 units/s is 40 units behind while it moves (0.5 s, plus the 0.2 s it takes to catch up): about 8
    # of the 52 frames are over 20 units.
    assert 0.08 < t["share_over_20"] < 0.25
    assert t["runs_over_20_frames"]["n"] == 1
    assert t["joint_error_units"]["3"]["p99"] == pytest.approx(SPEED * DELAY, abs=6)


def test_a_stop_keeps_moving_for_the_delay_and_as_far_as_it_trailed(session):
    ar = arm_response()
    s = ar.stops([session])
    assert s["n"] == 1
    j = s["joints"]["3"]
    assert j["stop_ms"]["p50"] == pytest.approx(DELAY * 1000, abs=30)
    assert j["travel_deg"]["p50"] == pytest.approx(SPEED * DELAY * ar.DEG_PER_UNIT, abs=1.5)  # the reading at the stop is up to 20 ms old
    assert j["past_target_deg"]["p50"] == pytest.approx(0.0, abs=0.3)


def test_summaries_name_each_percentile():
    ar = arm_response()
    assert set(ar.summary(list(range(100)), (0.5, 0.99, 0.999))) == {"p50", "p99", "p99.9", "n"}
