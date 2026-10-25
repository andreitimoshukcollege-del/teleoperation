"""Unit tests for ReadBackoff: after a missed read, a servo is skipped for a while instead of
holding every feedback cycle for the board library's one-second timeout."""

from jetrover_arm_control.servo_controller.read_backoff import BACKOFF_SECONDS, ReadBackoff


def test_a_servo_that_answers_is_read_every_cycle():
    b = ReadBackoff()
    for i in range(100):
        now = i * 0.02
        assert b.should_read(10, now)
        b.result(10, True, now, 0.003)
    assert b.take_skipped() == {}


def test_after_a_miss_it_is_skipped_then_asked_again():
    b = ReadBackoff()
    b.result(10, False, 1.0, 1.0)
    assert not b.should_read(10, 1.0 + 0.02)
    assert not b.should_read(10, 1.0 + BACKOFF_SECONDS - 0.001)
    assert b.should_read(10, 1.0 + BACKOFF_SECONDS)
    assert b.take_skipped() == {10: 2}
    assert b.take_skipped() == {}


def test_a_servo_that_never_answers_costs_one_timeout_per_backoff_not_per_cycle():
    b = ReadBackoff()
    asked = 0
    for i in range(500):  # ten seconds of 50 Hz cycles
        now = i * 0.02
        if b.should_read(10, now):
            asked += 1
            b.result(10, False, now, 1.0)
    assert asked == 5


def test_the_other_servos_are_unaffected():
    b = ReadBackoff()
    b.result(10, False, 0.0, 1.0)
    assert b.should_read(1, 0.1)
    assert not b.should_read(10, 0.1)


def test_an_answer_after_the_backoff_clears_it():
    b = ReadBackoff()
    b.result(3, False, 0.0, 1.0)
    assert b.should_read(3, BACKOFF_SECONDS)
    b.result(3, True, BACKOFF_SECONDS, 0.003)
    assert b.should_read(3, BACKOFF_SECONDS + 0.02)


def test_a_fast_error_is_retried_next_cycle_not_backed_off():
    # The wrist roll on 2026-10-25: one read failed in 17 ms. Backing off hid the roll for 2 s.
    b = ReadBackoff()
    b.result(5, False, 1.0, 0.017)
    assert b.should_read(5, 1.02)
    assert b.take_skipped() == {}
