"""write_gate.WriteGate on its own: the gate ServoController drives the servos through, and the Isaac JetRover's bridge
imports (teleoperation docs/adr/0034 amendment 13). No board, no clock: times are passed in."""

from jetrover_arm_control.servo_controller import write_gate as wg


def test_it_imports_nothing_beyond_the_standard_library():
    import ast
    import inspect
    tree = ast.parse(inspect.getsource(wg))
    imported = {n.names[0].name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import)}
    imported |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
    assert imported <= {"math"}  # the bridge imports it outside ROS, without a serial port


def test_resolve_truncates_and_clamps_as_the_driver_writes_its_targets():
    assert wg.resolve([(1, 612.9, 400.0), (2, -3, 400.0), (3, 1200.5, 400.0)]) == [(1, 612, 400.0), (2, 0, 400.0), (3, 1000, 400.0)]


def test_resolve_drops_a_target_that_is_not_finite_or_has_no_positive_speed():
    # int(nan) raised in the driver's callback and killed the node (the 2026-11 bug hunt, C-02).
    nan, inf = float("nan"), float("inf")
    targets = [(1, nan, 400.0), (2, inf, 400.0), (3, 500.0, nan), (4, 500.0, 0.0), (5, 500.0, -1.0), (10, 300.0, 400.0)]
    assert wg.resolve(targets) == [(10, 300, 400.0)]


def test_a_move_never_outlasts_the_boards_16_bit_millisecond_duration():
    # 157 pulses at 2.39 pulses/s planned 65.764 s, which the board wrapped to 228 ms while the gate held the servo
    # for 65.8 s (the 2026-11 bug hunt, C-03).
    g = wg.WriteGate()
    g.offer([(3, 500, 400.0)], 0.0)
    moves, _ = g.offer([(3, 657, 2.39)], 1.0)
    assert moves == [(wg.MAX_MOVE_DURATION_SECONDS, 3, 657)]
    assert int(wg.MAX_MOVE_DURATION_SECONDS * 1000) <= 0xFFFF
    assert g.nextAllowedWriteAt[3] == 1.0 + wg.MAX_MOVE_DURATION_SECONDS


def test_first_move_minimum_then_distance_over_speed_and_a_cooldown_keeps_the_newest_pending():
    g = wg.WriteGate()
    assert g.offer([(2, 700, 400.0)], 0.0) == ([(0.05, 2, 700)], [])
    assert g.offer([(2, 800, 400.0)], 0.01) == ([], [("GP", 2, 800)])
    assert g.offer([(2, 820, 400.0)], 0.02) == ([], [("GP", 2, 820)])
    assert g.flush(0.04) == ([], [])  # still cooling down to 0.05
    assert g.flush(0.05) == ([(0.3, 2, 820)], [])  # 120 pulses at 400/s
    assert g.offer([(2, 823, 400.0)], 0.4) == ([], [("GD", 2, 823)])


def test_packets_group_by_duration_in_first_seen_order():
    assert wg.packets([(0.05, 1, 500), (0.2, 2, 600), (0.05, 3, 700)]) == [(0.05, [[1, 500], [3, 700]]), (0.2, [[2, 600]])]


def test_a_torque_switch_drops_pending_and_times_from_the_last_reading():
    g = wg.WriteGate()
    g.offer([(2, 700, 400.0)], 0.0)
    g.offer([(2, 900, 400.0)], 0.01)
    g.torque_switched({2: 650})
    assert g.pending == {} and g.flush(1.0) == ([], [])
    assert g.offer([(2, 750, 200.0)], 0.02) == ([(0.5, 2, 750)], [])
