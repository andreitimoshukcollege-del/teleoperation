"""Which servos the feedback loop asks this cycle, after a read went unanswered.

The board library waits a full second for a servo that does not answer (BUS_SERVO_READ_TIMEOUT:
shorter stopped all feedback, 2026-10-20), and the feedback loop shares one thread with every
command. A servo that stops answering for good therefore used to hold every cycle, and every
command behind it, for a second: that is why the gripper (motor 10) was taken off the read list on
2026-10-20, when it did not answer. It answered every question within 3-9 ms on 2026-10-25 (`just
robot-servo-probe`), so it is read again, and this makes sure no servo can do that to the loop
again: after a miss that waited out the timeout, a servo is skipped for BACKOFF_SECONDS, then asked
again. Other servos are read as usual meanwhile. A servo that stops answering costs one second now and
then, not one second per cycle.

Only a slow miss backs off. A read can also fail fast, with an error answer instead of silence: on
2026-10-25 the wrist roll (motor 5) failed one read in 17 ms, and backing off from that hid it for 2 s,
which the relay reported as an invalid roll and the headset showed as a glitch. A fast failure costs
the loop nothing, so the servo is simply asked again next cycle.

Pure: the caller passes the time, so this is tested without a board, a clock or ROS.
"""

# How long a servo is left alone after a read it did not answer, seconds.
BACKOFF_SECONDS = 2.0

# A miss that took at least this long waited out the timeout (BUS_SERVO_READ_TIMEOUT is 1 s), seconds.
SLOW_MISS_SECONDS = 0.5


class ReadBackoff:

    def __init__(self, backoff_seconds=BACKOFF_SECONDS, slow_miss_seconds=SLOW_MISS_SECONDS):
        self._backoff = backoff_seconds
        self._slow = slow_miss_seconds
        self._skip_until = {}
        self.skipped = {}

    def should_read(self, motor_id, now):
        """Whether to ask motor_id this cycle; counts a skip in `skipped` when not."""
        until = self._skip_until.get(motor_id)
        if until is not None and now < until:
            self.skipped[motor_id] = self.skipped.get(motor_id, 0) + 1
            return False
        return True

    def result(self, motor_id, answered, now, took_seconds):
        """After a read that took `took_seconds`: an answer clears the backoff, a slow miss starts one from `now`."""
        if answered:
            self._skip_until.pop(motor_id, None)
        elif took_seconds >= self._slow:
            self._skip_until[motor_id] = now + self._backoff

    def take_skipped(self):
        """The skip counts since the last call, then zero: for the stats line."""
        skipped, self.skipped = self.skipped, {}
        return skipped
