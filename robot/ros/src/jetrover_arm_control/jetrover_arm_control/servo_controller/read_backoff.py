"""Which servos the feedback loop asks this cycle, after a read went unanswered.

The board library waits a full second for a servo that does not answer (BUS_SERVO_READ_TIMEOUT:
shorter stopped all feedback, 2026-10-20), and the feedback loop shares one thread with every
command. A servo that stops answering for good therefore used to hold every cycle, and every
command behind it, for a second: that is why the gripper (motor 10) was taken off the read list on
2026-10-20, when it did not answer. It answered every question within 3-9 ms on 2026-10-25 (`just
robot-servo-probe`), so it is read again, and this makes sure no servo can do that to the loop
again: after a miss, a servo is skipped for BACKOFF_SECONDS, then asked again. Other servos are
read as usual meanwhile. A servo that misses once costs one second, not one second per cycle.

Pure: the caller passes the time, so this is tested without a board, a clock or ROS.
"""

# How long a servo is left alone after a read it did not answer, seconds.
BACKOFF_SECONDS = 2.0


class ReadBackoff:

    def __init__(self, backoff_seconds=BACKOFF_SECONDS):
        self._backoff = backoff_seconds
        self._skip_until = {}
        self.skipped = {}

    def should_read(self, motor_id, now):
        """Whether to ask motor_id this cycle; counts a skip in `skipped` when not."""
        until = self._skip_until.get(motor_id)
        if until is not None and now < until:
            self.skipped[motor_id] = self.skipped.get(motor_id, 0) + 1
            return False
        return True

    def result(self, motor_id, answered, now):
        """After a read: an answer clears the backoff, a miss starts one from `now`."""
        if answered:
            self._skip_until.pop(motor_id, None)
        else:
            self._skip_until[motor_id] = now + self._backoff

    def take_skipped(self):
        """The skip counts since the last call, then zero: for the stats line."""
        skipped, self.skipped = self.skipped, {}
        return skipped
