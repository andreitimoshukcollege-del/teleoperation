"""The driver's write gate and single thread in front of the simulated servos (servo.DriverEmulator, docs/adr/0034
amendment 13), without Isaac."""

import os

import pytest

from teleop_isaac import servo


class Servos:
    """Records what the emulator asks of the servos."""

    def __init__(self):
        self.moves = []    # (t, motor, pulse, seconds)
        self.commands = []  # (t, motor, pulse, pps): ungated
        self.torques = []
        self.t = 0.0

    def move(self, motor, pulse, seconds):
        self.moves.append((self.t, motor, pulse, seconds))

    def command(self, motor, pulse, pps):
        self.commands.append((self.t, motor, pulse, pps))

    def torque(self, on):
        self.torques.append((self.t, on))


def emulator(gate=True):
    model = servo.SimModel.load()
    s = Servos()
    rows = []
    drv = servo.DriverEmulator(model, servo.FeedbackEmulator(model), servo_move=s.move, servo_command=s.command,
                               servo_torque=s.torque, gate=gate, log=lambda tag, t, *f: rows.append((tag,) + f),
                               clock_ns=lambda: 0)
    return drv, s, rows


def run(drv, s, until, commands=(), torques=(), reading=500.0):
    """Steps the driver every millisecond to ``until`` s, reading every motor at ``reading``; ``commands`` are
    ``(t, targets)`` and ``torques`` ``(t, on)``, delivered at their times."""
    todo = sorted([(t, "c", x) for t, x in commands] + [(t, "t", x) for t, x in torques], key=lambda e: e[0])
    t, i = 0.0, 0
    while t <= until + 1e-9:
        s.t = t
        while i < len(todo) and todo[i][0] <= t + 1e-9:
            _, kind, x = todo[i]
            drv.command(t, x) if kind == "c" else drv.torque(t, x)
            i += 1
        mid = drv.due_read(t)
        while mid is not None:
            drv.record(mid, reading, t)
            mid = drv.due_read(t)
        t = round(t + 0.001, 6)


def test_the_gate_is_the_real_drivers_own_module():
    wg = servo.write_gate()
    assert os.path.normpath(wg.__file__).endswith(os.path.normpath(
        "robot/ros/src/jetrover_arm_control/jetrover_arm_control/servo_controller/write_gate.py"))
    assert (wg.MIN_MEANINGFUL_PULSE_DELTA, wg.MIN_MOVE_DURATION_SECONDS) == (5, 0.05)


def test_a_command_waits_for_the_read_cycle_to_end():
    drv, s, rows = emulator()
    # The first cycle starts at 0 and reads six motors 3.5 ms apart: busy until 6 x 3.5 = 21 ms.
    run(drv, s, 0.03, commands=[(0.005, [(2, 700.0, 400.0)])])
    assert [(round(t, 3), m, p) for t, m, p, _ in s.moves] == [(0.021, 2, 700.0)]
    assert rows[0] == ("CI", [(2, 700, 400.0)])


def test_a_servos_first_move_takes_the_minimum_and_the_next_waits_out_its_cooldown():
    drv, s, rows = emulator()
    run(drv, s, 0.4, commands=[(0.019, [(2, 700.0, 400.0)]), (0.030, [(2, 800.0, 400.0)])])
    first, second = s.moves
    assert (round(first[0], 3), first[2], first[3]) == (0.021, 700.0, 0.05)  # no last pulse sent yet: the minimum
    # The second arrives in the cooldown (to 71 ms), waits, and goes out at the first read after it, at most one read
    # (3.5 ms) later: 100 pulses at 400/s.
    assert ("GP", 2, 800) in rows
    assert 0.071 - 1e-9 <= second[0] < 0.071 + 0.0035 + 1e-9 and (second[2], second[3]) == (800.0, 0.25)
    assert ("WR", 250, [(2, 800)]) in rows


def test_a_target_within_5_pulses_of_the_last_written_is_dropped_and_the_newest_waiting_target_wins():
    drv, s, rows = emulator()
    run(drv, s, 0.3, commands=[(0.019, [(3, 600.0, 400.0)]), (0.080, [(3, 604.0, 400.0)]),
                               (0.100, [(3, 650.0, 400.0)]), (0.101, [(3, 660.0, 400.0)])])
    assert ("GD", 3, 604) in rows
    # 650 goes out at once (600's cooldown is over) and starts its own, 125 ms; 660 waits it out and replaces nothing older.
    pulses = [p for _, m, p, _ in s.moves if m == 3]
    assert pulses == [600.0, 650.0, 660.0]


def test_servos_with_the_same_duration_share_a_packet():
    drv, s, rows = emulator()
    run(drv, s, 0.05, commands=[(0.019, [(1, 520.0, 400.0), (2, 540.0, 400.0), (3, 560.0, 400.0)])])
    assert [r for r in rows if r[0] == "WR"] == [("WR", 50, [(1, 520), (2, 540), (3, 560)])]
    assert drv.packets == 1


def test_a_torque_switch_drops_the_waiting_target_and_times_the_next_move_from_the_last_reading():
    drv, s, rows = emulator()
    run(drv, s, 0.3, commands=[(0.019, [(2, 700.0, 400.0)]), (0.030, [(2, 900.0, 400.0)]), (0.120, [(2, 600.0, 200.0)])],
        torques=[(0.040, False), (0.100, True)], reading=520.0)
    # Each switch waits for the cycle running when it arrives, then takes the thread before the next one.
    switched = [(round(t, 3), on) for t, on in s.torques]
    assert [on for _, on in switched] == [False, True]
    assert all(0 < t - sent < 0.021 + 0.003 for (t, _), sent in zip(switched, (0.040, 0.100)))
    assert ("TQ", 0) in rows and ("TQ", 1) in rows
    pulses = [(p, sec) for _, m, p, sec in s.moves if m == 2]
    # 900 was waiting when torque went off: dropped. 600 is timed from the last reading, 520: 80 pulses at 200/s.
    assert pulses == [(700.0, 0.05), (600.0, 0.4)]


def test_a_full_command_queue_drops_its_oldest():
    drv, s, rows = emulator()
    drv.due_read(0.0)  # a cycle starts: nothing runs until it ends
    for i, p in enumerate((610.0, 620.0, 630.0, 640.0)):
        drv.command(0.001 + i * 0.001, [(4, p, 400.0)])
    assert [r for r in rows if r[0] == "CI"] == []
    run(drv, s, 0.1)
    assert [r[1][0][1] for r in rows if r[0] == "CI"] == [620, 630, 640]


def test_the_thread_takes_one_waiting_command_between_two_cycles():
    # The real driver.jlog: the read timer is overdue when a cycle ends, so the executor runs one command array and then
    # the next cycle. A backlog drains one command per cycle (21 ms of reads and the command's 2.75 ms).
    drv, s, rows = emulator()
    at = []
    drv._clock_ns = lambda: round(s.t * 1e9)
    drv._log = lambda tag, t, *f: at.append((t / 1e9, f[0][0][1])) if tag == "CI" else None
    drv.due_read(0.0)
    for i, p in enumerate((620.0, 630.0, 640.0)):
        drv.command(0.001 + i * 0.001, [(4, p, 400.0)])
    run(drv, s, 0.1)
    assert [p for _, p in at] == [620, 630, 640]
    assert [round(t, 3) for t, _ in at] == [0.021, 0.045, 0.069]  # each after a cycle: 21 + 3 (the command) + 21 ms


def test_without_the_gate_every_target_goes_straight_to_the_servo():
    drv, s, rows = emulator(gate=False)
    drv.due_read(0.0)
    drv.command(0.001, [(2, 700.4, 400.0)])
    drv.command(0.002, [(2, 702.0, 400.0)])
    assert [(m, p, pps) for _, m, p, pps in s.commands] == [(2, 700.4, 400.0), (2, 702.0, 400.0)]
    assert s.moves == []


def test_the_relay_publishes_at_its_polls_every_torque_switch_and_the_newest_command():
    relay = servo.RelayEmulator()
    assert relay.due(0.0) == []
    relay.offer("command", [(2, 600.0, 400.0)])
    relay.offer("torque", True)
    relay.offer("command", [(2, 610.0, 400.0)])
    assert relay.due(0.019) == []  # between polls: waiting in the socket
    assert relay.due(0.020) == [("torque", True), ("command", [(2, 610.0, 400.0)])]
    assert relay.due(0.021) == []
