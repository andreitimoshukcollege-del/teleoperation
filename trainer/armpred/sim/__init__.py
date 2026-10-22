"""The JetRover simulator (docs/adr/0021 decision 6).

It writes session v1 (contract/SESSION_FORMAT.md), so a simulated session travels exactly the path
a recorded one does. What it copies from the real system is the code's behaviour: the headset's
48 Hz re-sends and keepalives, RobotHost's staleness check, pulse conversion and 50 Hz frames, the
relay's 20 ms newest-wins poll and 20 Hz feedback with its 0.2 s staleness, the driver's
single-threaded read cycle with its write gate. What it *guesses* is the servo itself -- speed, lag,
droop, load, hand resistance, read timing -- and those guesses are drawn wide per session
(``params.py``) until ``sysid`` re-centres them on recordings. Its numbers are never the robot's.
"""

from .params import SimParams, sample_params
from .robot import simulate_session

__all__ = ["SimParams", "sample_params", "simulate_session"]
