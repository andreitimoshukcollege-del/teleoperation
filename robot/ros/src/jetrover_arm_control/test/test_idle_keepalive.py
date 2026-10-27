"""The idle keep-alive experiment's decisions (servo_controller/idle_keepalive.py), with the time passed in."""

import os
import tempfile

from jetrover_arm_control.servo_controller import idle_keepalive as K
from jetrover_arm_control.servo_controller.idle_keepalive import IdleKeepalive


def ticks(start, seconds, hz=50.0):
    n = int(round(seconds * hz))
    return [start + i / hz for i in range(n)]


def test_off_polls_every_tick_and_never_writes():
    k = IdleKeepalive(0.0, "off")
    assert all(k.should_poll(t) for t in ticks(100.0, 5.0))
    assert not any(k.should_write(t) for t in ticks(100.0, 5.0))


def test_write_mode_writes_once_a_second_only_while_idle():
    k = IdleKeepalive(0.0, "write")
    k.on_command(10.0)
    assert not any(k.should_write(t) for t in ticks(10.0, K.WRITE_IDLE_SECONDS - 0.1))
    writes = [t for t in ticks(10.0 + K.WRITE_IDLE_SECONDS, 10.0) if k.should_write(t)]
    assert len(writes) == 10
    assert all(abs((b - a) - K.WRITE_PERIOD_SECONDS) < 0.05 for a, b in zip(writes, writes[1:]))
    k.on_command(30.0)
    assert not k.should_write(31.0)
    assert all(k.should_poll(t) for t in ticks(30.0, 40.0))  # write mode never slows the reads


def test_slow_poll_reads_at_the_slow_rate_while_idle_and_full_rate_on_a_command():
    k = IdleKeepalive(0.0, "slow-poll")
    k.on_command(0.0)
    assert all(k.should_poll(t) for t in ticks(0.0, K.POLL_IDLE_SECONDS - 0.1))
    idle = ticks(K.POLL_IDLE_SECONDS, 10.0)
    polled = [t for t in idle if k.should_poll(t)]
    assert abs(len(polled) - 10.0 * K.SLOW_POLL_HZ) <= 1
    # Never slower than the relay's 0.2 s staleness limit.
    assert max(b - a for a, b in zip(polled, polled[1:])) < 0.2
    k.on_command(idle[-1])
    assert all(k.should_poll(t) for t in ticks(idle[-1], 1.0))
    assert not any(k.should_write(t) for t in idle)


def test_counts_are_taken_and_cleared():
    k = IdleKeepalive(0.0, "slow-poll")
    for t in ticks(K.POLL_IDLE_SECONDS, 1.0):
        k.should_poll(t)
    writes, slow, skipped = k.take_counts()
    assert writes == 0 and slow > 0 and skipped > 0
    assert k.take_counts() == (0, 0, 0)


def test_switching_mode_keeps_the_idle_clock():
    k = IdleKeepalive(0.0, "off")
    k.set_mode("write")
    assert k.should_write(K.WRITE_IDLE_SECONDS + 1.0)
    k.set_mode("nonsense")
    assert k.mode == "write"


def test_read_mode_file():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "arm-keepalive")
        assert K.read_mode(path) == ("off", None)
        for mode in K.MODES:
            with open(path, "w") as f:
                f.write(mode + "\n")
            assert K.read_mode(path) == (mode, None)
        with open(path, "w") as f:
            f.write("ping-pong")
        mode, problem = K.read_mode(path)
        assert mode == "off" and "ping-pong" in problem
