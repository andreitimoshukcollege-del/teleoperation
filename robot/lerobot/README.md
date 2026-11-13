# robot/lerobot: headset takes into LeRobot datasets (docs/adr/0036)

Every complete take from the headset's RECORD button becomes one episode of its scenario's LeRobot v3 dataset, in the
lab's schema (`SINRG-Lab/world-models`, `data_collection/`). The converter runs on the GPU machine:

| Recipe | What |
|---|---|
| `just lerobot-setup` | once: `~/isaacsim/lerobot-venv` with `requirements.txt`'s lerobot |
| `just lerobot-watch-start` / `-stop` / `-status` | the converter (`isaac-host` and `isaac-jetrover-host` start it if it is not running) |
| `just lerobot-rebuild <robot>-<source>/<scenario>` | one dataset again from its kept takes |
| `just lerobot-test` | the tests there, LeRobot round trip included |
| `just isaac-jetrover-take [scenario] [script] [speed] [seconds]` | a take of Isaac's JetRover with no headset: joint-drive drives it from the PC and records it, as RECORD would (needs `just isaac-jetrover-host`) |
| `just record-take <scenario> …` | the same on the real robot: **moves the arm**, someone watching |
| `just isaac-share-scenarios` | lets the labmate read the takes and the datasets |
| `just lerobot-robot-key` | once: lets the GPU machine copy the real robot's takes (a key of its own, the robot's address) |
| `just lerobot-robot-host` | the robot's address again, after DHCP moved it |
| `just robot-prune-takes [true]` | deletes robot takes the GPU machine holds identical copies of (dry run by default) |

Datasets: `~/isaacsim/lerobot/<robot>-<source>/<scenario>/`. Beside LeRobot's files, `meta/teleop/takes.jsonl` names
the take behind each episode, and `episode_<n>.timing.jsonl` gives each frame's source times and its state's and
action's ages. Takes that could not be converted are in `~/isaacsim/lerobot/skipped.jsonl`, with the reason.

| Module | |
|---|---|
| `teleop_lerobot/jlog.py` | a strict `.jlog` reader, to the spec (`trainer/` may not be imported) |
| `teleop_lerobot/takes.py` | a take into episode frames: alignment, 15 fps grid, trimming. No LeRobot, no numpy |
| `teleop_lerobot/writer.py` | the LeRobot side: append an episode, rebuild a dataset |
| `teleop_lerobot/watch.py` | finds complete takes, keeps each dataset in step with its kept takes |
| `teleop_lerobot/pull.py` | copies the real robot's complete takes into `~/isaacsim/scenarios-real`, checked before they land; follows its DISCARD LAST |

Tests: `python -m pytest robot/lerobot/tests` anywhere (the LeRobot round trip is skipped without lerobot).
