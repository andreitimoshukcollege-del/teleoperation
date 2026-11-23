# robot/lerobot: headset takes into LeRobot datasets (docs/adr/0036)

Every complete take from the headset's RECORD button becomes one episode of its scenario's LeRobot v3 dataset, in the
lab's schema (`SINRG-Lab/world-models`, `data_collection/`). The converter runs on the GPU machine:

| Recipe | What |
|---|---|
| `just lerobot-setup` | once: `~/isaacsim/lerobot-venv` with `requirements.txt`'s lerobot |
| `just lerobot-watch-start` / `-stop` / `-status` | the converter. Start also installs a cron keeper that brings it back within 5 min of a reboot or crash; stop removes it (`isaac-host` and `isaac-jetrover-host` start it too if it is not running) |
| `just lerobot-rebuild <robot>-<source>/<scenario>` | one dataset again from its kept takes |
| `just lerobot-test` | the tests there, LeRobot round trip included |
| `just isaac-jetrover-take [scenario] [script] [speed] [seconds]` | a take of Isaac's JetRover with no headset: joint-drive drives it from the PC and records it, as RECORD would (needs `just isaac-jetrover-host`) |
| `just record-take <scenario> …` | the same on the real robot: **moves the arm**, someone watching |
| `just isaac-share-scenarios` | lets the labmate read the takes and the datasets |
| `just lerobot-robot-key` | once: lets the GPU machine copy the real robot's takes (a key of its own, the robot's address) |
| `just lerobot-robot-host` | the robot's address again, after DHCP moved it |
| `just robot-prune-takes [true]` | deletes robot takes the GPU machine holds identical copies of (dry run by default) |

Datasets: `~/data/lerobot/<robot>-<source>/<scenario>/` (what lives where on the GPU machine: docs/gpu-machine.md; the
watcher's log and pid file are in `~/logs/lerobot/`). Beside LeRobot's files, `meta/teleop/takes.jsonl` names
the take behind each episode, and `episode_<n>.timing.jsonl` gives each frame's source times and its state's and
action's ages. At the dataset's top, as the lab's `record_jetrover.py` writes them: `capture_timing.jsonl` (one line
per frame of every episode, its field names) and `raw_ros_events.jsonl` (every take's commands as `action` and
readings as `state`, its bridge's fields). Their times are the session's one clock, CLOCK_MONOTONIC; the lab's
`*_wall_ns` fields are null, since a take keeps no wall clock per record. Takes that could not be converted are in `~/data/lerobot/skipped.jsonl`, with the reason.

| Module | |
|---|---|
| `teleop_lerobot/jlog.py` | a strict `.jlog` reader, to the spec (`trainer/` may not be imported) |
| `teleop_lerobot/takes.py` | a take into episode frames: alignment, 15 fps grid, trimming. No LeRobot, no numpy |
| `teleop_lerobot/writer.py` | the LeRobot side: append an episode, rebuild a dataset |
| `teleop_lerobot/watch.py` | finds complete takes, keeps each dataset in step with its kept takes |
| `teleop_lerobot/pull.py` | copies the real robot's complete takes into `~/data/takes/jetrover-real`, checked before they land; follows its DISCARD LAST |
| `teleop_lerobot/safety.py` | the twin's safety datasets: each take checked (`safety-check`), labelled by physics, built (`safety-build`), compared (`safety-compare`) |
| `teleop_lerobot/sidecars.py` | what each safety episode showed and what happened in it (the sidecars below), the layout hash, the split by layout |

Tests: `python -m pytest robot/lerobot/tests` anywhere (the LeRobot round trip is skipped without lerobot).

## Safety datasets: labels and sidecars (docs/adr/0036 amendment 1, docs/adr/0038 decision 6)

The twin's scripted takes become one dataset per batch, `~/data/lerobot/jetrover-isaac-safety/jetrover_<UTC>_<id>/`
(`just isaac-safety-build`; `split=K` writes K datasets `<name>_<k>of<K>` instead, by layout). `observation.images.front`
is the twin's wrist camera. Every episode's task is `move the arm` (`--caption category` gives the scenario's name, as
datasets built before 2026-11-22 have it); the scenario is in `meta/teleop/episodes.jsonl`.

**The checks** (`safety-check`). The four old categories are checked as before. A `mixed` take is set aside (to
`_discarded/mixed/`, its reasons in `safety-check.json`) only when it is broken: it cannot be read as an episode (no
scenario, a `.jlog` cut off or failing its trailer, no accepted command, no colour camera, no frame with its joints), joint-drive
cut it short (`--drive-exit` 2 a stall, 3 the joint-state stream, 130 by hand; 0 and 4 are fine), its frames ended early (`ER`),
a writer dropped records, it has no object poses (`WO`), or its episode repeats one render for 1 s or more. What happened
is never a reason. `WC` object 0 is the table: never a contact for the labels or the checks.

**Labels**, the same ranges in every file: a frame is unsafe when an arm link touched a scene object other than the table at
any moment of its frame period (its time, half a frame either side); `--table-unsafe` counts the table too.

| File | What |
|---|---|
| `safety_ranges.json` | world-models' (`wm/labeling/ranges.py`): exactly `source` (`physics`), `notes` (the rule) and `unsafe_ranges` (`{"<episode>": [[start, end], ...]}`, inclusive, episode-relative frames; an episode left out is all safe) |
| `manual_safety_labels.json` | the same ranges, the bare legacy map |
| `meta/safety_labels.parquet` | one row a frame: `episode_index`, `frame_index`, `label`, `category`, `effective_category` (the scenario), `overridden` (false), `source` (`physics`) |
| `safety_events.jsonl` | `contact` at each unsafe range's first frame and, for `collide_and_recover` and `mixed`, `clear` at the first safe frame after it |

**Sidecars** under `meta/teleop/`, beside `takes.jsonl` and the timing files. Times are seconds from the episode's first
frame (its `stateSensed`, the take's CLOCK_MONOTONIC); poses are `[x, y, z, qw, qx, qy, qz]` in the robot's base frame (m);
link names are the URDF's. JSON has no NaN: a value that is not there is `null` (NaN in the parquet).

`episodes.jsonl`, one line an episode:

| Field | What |
|---|---|
| `episode`, `take`, `session`, `scenario` | the episode, its take's folder and session id, the take's scenario (`mixed`, or an old category) |
| `sceneName`, `sceneSha256` | the scene file the take ran |
| `layout`, `layoutSeed` | `WL`'s layout number and seed at the first frame: one draw of the scene's layout |
| `layoutHash` | the layout the episode starts from (`sidecars.layout_hash`): sha256 of `{"objects":[[id,[sx,sy,sz],[x,y,z],[r00..r22]],...],"scene":"<sha>","v":1}` (sorted keys, no spaces) over the objects on the table sorted by id, with integer dimensions after the size factor and position in mm and the rotation matrix in thousandths, each rounded half up. Equal hashes: the same objects, sizes and start poses to 1 mm and about 0.1 degree. A draw placed twice can settle across a rounding boundary, so group by `layoutSeed` too when a layout must stay on one side |
| `outcome` | `knock`: an obstacle the arm touched moved at least 3 cm or tipped at least 30 degrees from its first touch to the episode's end (the old knock check's rule); `contact`: it touched an obstacle, no knock; `near_miss`: no contact, and some link came within 30 mm of an obstacle; `no_contact`: no contact and never within 30 mm; `unknown`: no contact, and no clearance to tell the last two apart |
| `tableContact`, `tableContacts` | whether, and how many stretches, the arm touched the table |
| `contacts`, `touched`, `knocked` | obstacle contact stretches (contacts.jsonl's lines), the objects touched, those knocked |
| `unsafeFrames`, `unsafeRanges` | the labels' frames and ranges |
| `minClearanceMm`, `minClearanceObject` | the smallest obstacle clearance over the episode's frames and its object; null without clearance |
| `clearanceMethod`, `clearanceCutoffMm`, `clearanceUnavailable` | the clearance module's method and cutoff, or why there is none |
| `group`, `role`, `edit`, `editParsed` | the take's `RP` (docs/adr/0038 decision 2): its replay group, `source` or `replay`, the edit as recorded and parsed (`op` `none`, `remove`, `shift` with `dx` `dy`, `move` with `x` `y` `yawRad`, and `object`); null for takes without one |
| `frames`, `seconds`, `filled` | its length, and how many frames repeat an earlier render |
| `firstContactS`, `afterLastContactS`, `timingFlags` | item 24: when the first obstacle contact began, how long after the last one ended the episode runs (0 when still touching), and the reasons it is reported: a first contact under 1.5 s or a last one ending under 1.0 s before the end (never set aside for it) |
| `speedSlider`, `operator` | from session.json's conditions |
| `room` | the `EN` in force: `change`, `seed`, `hdri`, `sha256`, `yawDeg`, `intensity`, `settleFrames`, `brightness`, `gains` (per recorded camera name), later fields under `extra` |
| `looks`, `materials`, `clutter` | `LK` and `MT` of that change (name to look, id to texture), and `CL` counted (`pieces`, `kinds`) |
| `cameraJitter` | per camera (dataset names), its `CM` pose against the scene file's: `mm` apart and the turn in `deg` |
| `moves` | the mixed script's planned moves (`MV`): `count`, `kinds`, and every record's fields as strings. Metadata, never labels |
| `events` | an old script's planned events (`SE`): `approachS` and its times after that |

`scenes.jsonl`, one line an episode: `episode`, `take`, the scene and layout fields above, and

- `objects`: every object the layout put on the table (not the parked ones): `id`, `name`, `kind` (`movable` or
  `fixture`), `shape`, `asset` (a mesh's) or `primitive`, `clutter`, `sizeFactor`, `dimensionsM` (the box after the size
  factor), `massKg` (at that size; null for a fixture), `reach` (`distractor` for a movable object by the bench's edge,
  else `in_reach`), `baseDistanceM` (its centre from the base axis), `start` and `end` poses, and `changes`: `[t, x, y, z,
  qw, qx, qy, qz]` each time it had moved 1 mm or turned 1 degree from the last listed pose;
- `surfaces`: the scene file's (`id`, `center`, `size`, `collides`);
- `cameras`: per recorded camera, its dataset `name`, the `recorded` one, `width`, `height`, `fx`, `fy`, `cx`, `cy`, `near`,
  `far`, `pose`, `mount` and `poseFrame` (`CM`, trainer/contract/SESSION_FORMAT.md: a wrist camera's pose is in its mount
  link's frame).

`contacts.jsonl`, one line per stretch of the arm touching one object without a break (from the first `WC` naming it to
the first that does not): `episode`, `object` (0 the table), `name`, `kind` (`object`, `fixture`, `table`), `links` and
`linkIndices`, `startFrame` and `endFrame` (the frames whose period overlaps it), `startS`, `endS` (null and `open` true
when no `WC` ended it), `peakForceN` (the largest of its `WC` records' summed force on the object), and for an object
`movedCm`, `tiltDeg`, `moved` (at least 3 cm) and `tipped` (at least 30 degrees) from the stretch's start to the next
stretch with it, or the episode's end.

`frames.parquet`, one row a frame: `episode_index`, `frame_index`, `sensed_ns` (the state the frame shows), `t_s`,
`in_contact` (an obstacle, the labels' rule), `in_contact_table`, `unsafe` (the labels, 0 or 1), per link
`clearance_mm.<link>` and `clearance_id.<link>` (signed distance to the nearest obstacle and its id) and `surface_mm.<link>`
(to the table surface), `min_clearance_mm`, `min_clearance_id`, `min_surface_mm`, `time_to_contact_s` (to the next obstacle
contact's start; 0 in contact, NaN when none follows in the episode), `filled` (the frame repeats an earlier render: the 15
fps grid point had no recorded frame within half a step, or the same one as the point before), `state_skew_ms` and
`action_skew_ms` (how much older the state reading and the command behind the frame, received at
`capture_timing.jsonl`'s `pc_action_recv_mono_ns`, are than the frame). The per-link columns are there when clearance was
computed (workstream C's `teleop_lerobot/clearance.py` and the geometry under `--geometry-dir`, default
`~/isaacsim/assets/clearance`); without, the `min_` columns are NaN.

`report.json`: the dataset's `episodes`, `frames`, `seconds`, `scenarios`, `outcomes`, `contactEpisodes`,
`tableContactEpisodes`, `contacts` by kind, `unsafeFrames`, `unsafeRanges`, `replayGroups`, `roles`, `layoutHashes`,
`layoutSeeds`, `clearanceEpisodes`, `timingFlagged` (episodes), `split` (`part`, `of`) and the build's `options`.

`unavoidable_ranges.json`, only with a stop model (`--stop-latency-ms` with `--stop-time-ms` or `--stop-decel-rad-s2`),
**provisional until the real arm's stop latency is measured** (docs/adr/0038 item 17): the frames from which stopping could
no longer avoid the next obstacle contact, `{"<episode>": [[start, end], ...]}`. A stop asked for at a frame takes effect
after the latency, then the arm slows linearly to rest over the stop time (or its fastest arm joint's speed at the frame,
from `VQ`, over the deceleration), so it rests where the recorded motion was half the stop time after the latency; a frame
not in contact is unavoidable when the next contact starts by then. `unmeasured` lists episodes without `VQ` under a
deceleration model.

**Comparing** (`safety-compare REFERENCE REBUILT`): a rebuild of a reference's first takes, file for file; the sidecars by
episode as the label files are. A reference from before docs/adr/0038 has no sidecars and is compared without them (rebuild
it with `--caption category`); `just isaac-safety-build-check` does that by itself.
