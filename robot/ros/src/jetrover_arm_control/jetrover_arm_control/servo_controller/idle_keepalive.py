"""What the driver does with the servo board while nobody is commanding the arm: an experiment on the
board going silent (robot/README.md, "The servo board goes silent"; started 2026-10-27).

The board stops answering after 127k-2M reads, and only a power cycle brings it back. Two theories, one
mode each, chosen in a file on the robot (MODE_FILE, `just robot-keepalive <mode>`) and re-read every
few seconds, so a switch needs no driver restart:

- ``write``: the board fails when it gets no *writes* for a long time (the user's theory, 2026-10-27:
  it has never gone silent while being operated). While no command has come for WRITE_IDLE_SECONDS,
  a harmless write every WRITE_PERIOD_SECONDS: a 5 ms flash of the board's own LED. No servo moves.
- ``slow-poll``: each read carries a tiny chance of wedging the board's firmware (silences follow a
  number of reads, each preceded by sporadic single misses). While no command has come for
  POLL_IDLE_SECONDS, the feedback loop reads at SLOW_POLL_HZ instead of every timer tick (50 Hz); the
  first command brings the full rate back at once. SLOW_POLL_HZ stays above the relay's staleness
  limit (0.2 s), so the robot never looks silent for it.
- ``off``: neither, as before.

The driver's [stats] line says which mode ran and what it did, so `just robot-driver-stats` can compare
the time to silence under each.

Pure: the caller passes the time, so this is tested without a board, a clock or ROS.
"""

import os

MODES = ("off", "write", "slow-poll")
MODE_FILE = os.path.expanduser("~/.teleop/arm-keepalive")

WRITE_IDLE_SECONDS = 5.0
WRITE_PERIOD_SECONDS = 1.0
LED_FLASH_SECONDS = 0.005

POLL_IDLE_SECONDS = 30.0
SLOW_POLL_HZ = 10.0


def read_mode(path=MODE_FILE):
    """The mode in `path`, and a problem to log (None when fine). A missing file is ``off``."""
    try:
        with open(path, "r", encoding="ascii") as f:
            text = f.read().strip()
    except FileNotFoundError:
        return "off", None
    except (OSError, UnicodeDecodeError) as e:
        return "off", f"cannot read {path}: {e}"
    if text in MODES:
        return text, None
    return "off", f"{path} says {text!r}; expected one of {', '.join(MODES)}"


class IdleKeepalive:

    def __init__(self, now, mode="off"):
        self.mode = mode if mode in MODES else "off"
        self._last_command = now
        self._last_write = None
        self._last_poll = None
        self.writes = 0
        self.slow_cycles = 0
        self.skipped_cycles = 0

    def set_mode(self, mode):
        """Switches the mode; the idle clocks carry on."""
        if mode in MODES:
            self.mode = mode

    def on_command(self, now):
        """A command reached the driver: the arm is being operated."""
        self._last_command = now

    def idle_seconds(self, now):
        return now - self._last_command

    def should_poll(self, now):
        """Whether the feedback loop reads this tick (always, except while idle in ``slow-poll``)."""
        if self.mode != "slow-poll" or self.idle_seconds(now) < POLL_IDLE_SECONDS:
            self._last_poll = now
            return True
        if self._last_poll is None or now - self._last_poll >= 1.0 / SLOW_POLL_HZ - 1e-6:
            self._last_poll = now
            self.slow_cycles += 1
            return True
        self.skipped_cycles += 1
        return False

    def should_write(self, now):
        """Whether to send the keep-alive write now (``write`` mode, idle, a period since the last)."""
        if self.mode != "write" or self.idle_seconds(now) < WRITE_IDLE_SECONDS:
            return False
        if self._last_write is not None and now - self._last_write < WRITE_PERIOD_SECONDS:
            return False
        self._last_write = now
        self.writes += 1
        return True

    def take_counts(self):
        """(writes, slow cycles, skipped cycles) since the last call, for the stats line."""
        counts = (self.writes, self.slow_cycles, self.skipped_cycles)
        self.writes = self.slow_cycles = self.skipped_cycles = 0
        return counts
