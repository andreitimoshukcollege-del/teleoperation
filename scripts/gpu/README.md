# scripts/gpu: working against the GPU machine from a clone

These are small tools the agents (and anyone else) use to check a branch, and to look at what the JetRover twin recorded, on the GPU
machine (`andrei@10.188.58.133`; `GPU_HOST` overrides it, as it does for the justfile). Each one reads; none deletes anything, except `cluster_archive.sh prune ... apply` (folders already identical on the cluster).
docs/agent-operations.md says when to use which.

| Tool | What it does | How to run it |
|---|---|---|
| `gate.sh` | Pushes the clone's branch to a scratch repo there and runs gates in a scratch checkout: `core`, `test`, `test=FILTER`, `verify`, `audit`, `bridge`, `isaac`, `lerobot`, `gpu` | `scripts/gpu/gate.sh core bridge isaac lerobot` (from inside the clone, after committing) |
| `take_report.py` | Per take: verdict, planned events, contact stretches (lead-in, peak force), layout (lying, edge, raised clutter), refused placements, looks, flat clutter; then the totals | `ssh andrei@10.188.58.133 'python3 - <since>' < scripts/gpu/take_report.py` |
| `take_timing.py` | Per take: length, idle at the start, when contacts begin, and the planned contact times from `SE` | `ssh andrei@10.188.58.133 'python3 - <since>' < scripts/gpu/take_timing.py` |
| `take_video.sh` | One take's cameras side by side as an mp4 | `ssh andrei@10.188.58.133 'bash -s <take_dir> /tmp/<name>.mp4' < scripts/gpu/take_video.sh` |
| `contact_sheet.sh` | One frame of each of up to 16 takes, 4 by 4 | `ssh andrei@10.188.58.133 'bash -s /tmp/<name>.jpg third-right 0.15 <take_dir>...' < scripts/gpu/contact_sheet.sh` |
| `arm_response.py` | How the arm follows its commands, real and twin (docs/adr/0038 workstream R): the share of frames over 20 units of `max|state - action|`, per-joint error and its stretches, the tool point's error, the commanded motion per 15 fps frame (joints, wrist pivot, tool point), error by commanded speed, the RO-to-RI delay, commanded joint speed and acceleration, step responses, and stops after the command holds. Reads sessions, takes and LeRobot datasets; standard library except the parquet | `python3 scripts/gpu/arm_response.py --group real=<sessions> --group twin=<dataset> --follow-takes --json out.json`, or on the GPU machine `ssh andrei@10.188.58.133 'TELEOP_REPO=~/repos/isaac ~/isaacsim/lerobot-venv/bin/python - <paths>' < scripts/gpu/arm_response.py` |
| `move_features.py` | How the mixed script's planned moves move, by kind (docs/adr/0038 "M, motion"): each per-move feature's distribution for aims, passes, free moves and reverses, and how well each feature, a decision stump and a logistic regression (cross-validated by take) tell aims and passes from free moves (AUC). numpy only | `cd core && dotnet run --project Teleop.Eval -- joint-drive --script mixed --speed 3.8 --seconds 15 --dry-run --scene Scenes/jetrover-mixed.json --layouts 1-200 --features /tmp/f.csv`, then `~/isaacsim/lerobot-venv/bin/python scripts/gpu/move_features.py /tmp/f.csv` on the GPU machine |
| `charuco_calibrate.py` | The real robot's cameras from a ChArUco recording (`just camera-calib-record`, board.json beside it): each camera's intrinsics, then all cameras' poses relative to one (pairwise, then a joint solve over every board position two or more cameras saw), with reprojection errors and a loop check; writes calibration.json into the recording. OpenCV with aruco, numpy | `ssh andrei@10.188.58.133 'cd ~/data/real-cameras/calibration && ~/isaacsim/lerobot-venv/bin/python - <recording>' < scripts/gpu/charuco_calibrate.py` |
| `cluster_archive.sh` | The GPU machine's `~/data` to Explorer's lab space (`archive`), what is safe to free here (`prune <days> [apply]`, only folders identical there), and sizes on both sides (`status`). Runs on the GPU machine; the cluster-* recipes stream it | `just cluster-archive`, `just cluster-prune 14`, `just cluster-data-status` |

Notes:
- **Reading a batch.** `<since>` is a take-name prefix in UTC, such as `20261122T0003`. Every take named at or after it is
  read, from both twins' takes folders (`~/data/takes/jetrover-isaac`, `~/data/takes/jetrover-isaac-twin1`). Pass other roots
  after it.
- **Getting a file back.** Fetch a video or sheet with `ssh andrei@10.188.58.133 'cat /tmp/<name>.mp4' > <local path>`, then
  copy it to the user's Downloads (`/mnt/c/Users/andre/Downloads/`).
- **Cost.** `gate.sh core` and `test` run the whole dotnet test suite there, about 10 minutes of the machine's CPU. The
  machine is shared, so ask before a heavy gate when the user has said it is busy. The other gates and tools are light.
