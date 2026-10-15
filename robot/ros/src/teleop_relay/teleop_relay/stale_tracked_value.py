"""A feedback value that reports itself invalid once too much time has passed since the last
update -- see relay_node.py's module doc for why this exists.
"""


class StaleTrackedValue:
    """Tracks the latest value received for one feedback channel and reports it valid only if
    it arrived within staleness_timeout_seconds of `now`. Without this, a channel whose real
    feedback silently stops (e.g. the middle-arm servo's reads, which robot_controller_node.py's
    own comment says never succeed) would report its one lucky reading as "valid" forever -- the
    bug found and fixed here via move-arm 0.1 0.1 0.1 no longer converging on 2026-08-08/09.

    Takes `now` as an explicit float rather than reading a clock itself, so it can be unit
    tested without a live rclpy context -- the same discipline this repo's own
    servo_controller.py already uses for its write cooldown (tracked via injected
    time.monotonic(), tested with monkeypatch).
    """

    def __init__(self, staleness_timeout_seconds: float):
        self._staleness_timeout_seconds = staleness_timeout_seconds
        self._value = 0
        self._last_update = None

    def update(self, value, now: float) -> None:
        self._value = value
        self._last_update = now

    def read(self, now: float):
        """Returns (valid, value). valid is False if a value has never arrived, or if the last
        one is older than staleness_timeout_seconds."""
        valid = (
            self._last_update is not None
            and (now - self._last_update) <= self._staleness_timeout_seconds
        )
        return valid, self._value
