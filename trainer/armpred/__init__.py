"""armpred: JetRover arm-state prediction, trained outside the repo's build (docs/adr/0021).

Nothing in this package imports anything from the teleoperation repo. The only shared surface is
the file formats in ``contract/`` and the golden files the repo writes there.
"""

from __future__ import annotations

__version__ = "0.2.0"

# The versions of the file formats this package reads and writes (trainer/contract/*.md).
SESSION_FORMAT_VERSION = 1
VIEW_FORMAT_VERSION = 2
MODEL_FORMAT_VERSION = 1
FORECAST_FORMAT_VERSION = 1
