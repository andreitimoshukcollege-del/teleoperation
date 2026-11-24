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
readings as `state`, its bridge's fields, and since 2026-11-24 the episode in `episode_index`, a field the lab's recorder
does not write). Their times are the session's one clock, CLOCK_MONOTONIC; the lab's
`*_wall_ns` fields are null, since a take keeps no wall clock per record. Takes that could not be converted are in `~/data/lerobot/skipped.jsonl`, with the reason.

A frame's `action` is the command the operator sent, the newest at the frame's time, in the units the plant sends its arm:
what RobotHost passed on for an accepted command (`RO`, relay pulses, on the JetRover; `AO` on the FR3 Duo), and a command
it refused (`JC` outcome `R`) as its plant would have turned it into those units. Until 2026-11-24 it was the `RO` or `AO`
alone, so a refused command left the one before it in force, and a replay that sent its source's commands exactly could
get another action (docs/adr/0038 "M, motion"). A command ignored while the arm parked (`P`) is not the action: RobotHost's
own park targets are the `RO` then. An episode spans the commands accepted or refused (until then the accepted ones). The
JetRover state's gripper slot stays the gripper's newest `RO`. A take with no refused command reads exactly as before.

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

The twin's scripted takes become one dataset per batch and twin, `~/data/lerobot/jetrover-isaac-safety/jetrover_<UTC>_<id>/`
(`just isaac-safety-build <twins> <name> <out> <split> <caption>`, arguments by position; two twins' takes are two datasets,
`<name>_twin0` and `<name>_twin1`; a split of K writes K datasets of each, `<name>_<k>of<K>` or `<name>_twin0_<k>of<K>`, by
layout). `observation.images.front` is the twin's wrist camera. Every episode's task is `move the arm` (`--caption
category` gives the scenario's name, as datasets built before 2026-11-22 have it); the scenario is in
`meta/teleop/episodes.jsonl`.

**One dataset holds one collection stream** (docs/adr/0038 L, 2026-11-24). `capture_timing.jsonl` and
`raw_ros_events.jsonl` keep one clock across a dataset, and a reader that picks the raw log's commands by time
(world-models' path checkpoints) takes whichever episode's were recording then. The twins record at once on the GPU
machine's one CLOCK_MONOTONIC, so `safety-build` writes the takes of each takes root (twin 0's `~/data/takes/jetrover-isaac`,
twin 1's `jetrover-isaac-twin1`) as a dataset of its own: `<name>` when all come from one root, else `<name>_twin<N>` (a root
not named for a twin gives its folder's name), each split on its own. It refuses any dataset whose takes overlap in time on
that clock (exit 3, `[refused]` lines naming every pair, nothing written); a take's span runs from the earliest to the
latest time its lines in the two files hold. `safety-report` places each episode the same way, from the files (by
`episode_index`, or by each take's `ready` line in a dataset built before it), and faults overlapping pairs.
`just isaac-safety-rebuild <dataset>` rebuilds a dataset built from two twins before then: its takes, one dataset per twin,
with the options its report records.

**Splits keep each replay group and each layout inside one split part.** A source take and its replays send the same
commands, and takes of one layout share its scene, so either in two parts leaks between them. A split build never parts
episodes that share an `RP` group, a layout hash, or a scene and `WL` seed. `teleop_lerobot safety-report DATASET...`
checks any set of datasets the same way: give it a split build's parts, or every dataset of a train, val and test set at
once. It faults (exit 1) when a replay group, a layout hash, or a scene's layout seed is in more than one of them, naming
the datasets and episodes; also on episodes of one dataset that overlap in time, label files that disagree, a replay group
without one source, unavoidable ranges over an unsafe frame, a missing file, or a dataset given twice; and, for the command
distances, a build that asked for them without their files, an episode with no rows and no reason, a frame without its pose
samples, or a frame whose `action` is not the last `commands.parquet` row received at or before it (the pilot check). Its
line on them gives their rows, the share with no distance, the share of commands whose commanded pose or the way there goes
into an obstacle, and each sample kind's median and share under 0. Datasets from before
docs/adr/0038 have no sidecars to check, only their episodes' times.

**The checks** (`safety-check`). The four old categories are checked as before. A `mixed` take is set aside (to
`_discarded/mixed/`, its reasons in `safety-check.json`) only when it is broken: it cannot be read as an episode (no
scenario, a `.jlog` cut off or failing its trailer, no accepted command, no colour camera, no frame with its joints), joint-drive
cut it short (`--drive-exit` 2 a stall, 3 the joint-state stream, 130 by hand; 0 and 4 are fine), its frames ended early (`ER`),
a writer dropped records, it has no object poses (`WO`), or its episode repeats one render for 1 s or more. What happened
is never a reason. `WC` object 0 is the table: never a contact for the checks, so a reach or stop take that touches it is
kept and a recover take is judged by its obstacle touches alone, as before. The checker's line (`unsafeFrames`,
`unsafeRanges`) counts what it judged, the obstacle contacts.

**Labels**, the same ranges in every file: a frame is unsafe when an arm link touched a scene object **or the table** at any
moment of its frame period (its time, half a frame either side). The table counts since 2026-11-23, at world-models' request:
on the real robot a hard table hit is a collision. The trade-off is the one docs/adr/0038 decision 4 named: labels that
count it reward "the path goes low", the shortcut that ADR set out to remove. So the kinds stay apart in the sidecars
(`frames.parquet`'s `in_contact` and `in_contact_table`, `contacts.jsonl`'s kind `table`, `episodes.jsonl`'s
`tableContact`), and obstacle-only labels are `in_contact` alone. `--table-safe` builds them as the labels, as builds did
before 2026-11-23 (`safety_ranges.json`'s `notes` then says so, word for word as those builds did). A kept reach or stop take
that brushed the table has only those frames unsafe.

| File | What |
|---|---|
| `safety_ranges.json` | world-models' (`wm/labeling/ranges.py`): exactly `source` (`physics`), `notes` (the rule) and `unsafe_ranges` (`{"<episode>": [[start, end], ...]}`, inclusive, episode-relative frames; an episode left out is all safe) |
| `manual_safety_labels.json` | the same ranges, the bare legacy map |
| `meta/safety_labels.parquet` | one row a frame: `episode_index`, `frame_index`, `label`, `category`, `effective_category` (the scenario), `overridden` (false), `source` (`physics`) |
| `safety_events.jsonl` | `contact` at each unsafe range's first frame and `clear` at the first safe frame after it, but in `collide_with_obstacle` (the lab's recorder never clears a knock). Until 2026-11-23 only `collide_and_recover` and `mixed` had a `clear`; a reach or stop take can now have a range (a table brush, which ends), and the lab's `label_safety.py` keeps those two categories all safe whatever their markers |

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
| `looks`, `materials`, `clutter` | `LK` and `MT` of that change (name to look, id to texture), and `CL` (`pieces`, `kinds`, and `items`: `[kind, x, y, yaw]` each) |
| `lookChanges` | the change number each of `EN`, `LK`, `MT`, `CL`, `LT`, `CJ` and `OA` came from: a replay group shares one look, so its takes' numbers match |
| `cameraJitter` | per camera (dataset names), its `CM` pose against the scene file's: `mm` apart and the turn in `deg`; with the drawn offset (`CJ`: `dx`, `dy`, `dz` m along the camera's own axes, `pitchDeg`, `yawDeg`, `rollDeg`) when the take records it |
| `lighting`, `objectColours` | `LT` (`domeKelvin`, `exposure`, `key`: `azimuthDeg`, `elevationDeg`, `intensity`, `kelvin`, `angleDeg`) and `OA` (per object id: `method`, `brightness`, `desaturation`, `tint`) of the room's change, when the take has an appearance section |
| `moves` | the mixed script's planned moves (`MV`): `count`, `kinds`, and every record's fields as strings. Metadata, never labels |
| `events` | an old script's planned events (`SE`): `approachS` and its times after that |

`scenes.jsonl`, one line an episode: `episode`, `take`, the scene and layout fields above, and

- `objects`: every object the layout put on the table (not the parked ones): `id`, `name`, `kind` (`movable` or
  `fixture`), `shape`, `asset` (a mesh's) or `primitive`, `clutter`, `sizeFactor`, `dimensionsM` (the box after the size
  factor), `massKg` (at that size; null for a fixture), `role` (a mixed pool's: `obstacle`, `distractor`, `stand`, `plank`,
  `support`; null without), `reach` (`distractor` for a mixed pool's distractor or, in a scene without roles, a movable object
  by the bench's edge; else `in_reach`), `baseDistanceM` (its centre from the base axis), `start` and `end` poses, and
  `changes`: `[t, x, y, z, qw, qx, qy, qz]` each time it had moved 1 mm or turned 1 degree from the last listed pose;
- `placements` and `edits`: the take's `WP` (`t`, `cause`, `id`, `outcome`) and `WE` (`t`, `op`, `id`, `outcome`) records, so
  a replay's edit shows whether it was applied (it is usually made before the take starts, then neither is there);
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
`in_contact` (an obstacle), `in_contact_table` (the table), `unsafe` (the labels, 0 or 1: either of the two, or `in_contact`
alone when built `--table-safe`), per link
`clearance_mm.<link>` and `clearance_id.<link>` (signed distance to the nearest obstacle and its id) and `surface_mm.<link>`
(to the table surface; NaN for a link no joint moves), `min_clearance_mm`, `min_clearance_id` (null when nothing is within
the clearance's cutoff, 150 mm, where the distance is that cutoff), `min_surface_mm`, `time_to_contact_s` (to the next obstacle
contact's start; 0 in contact, NaN when none follows in the episode), `filled` (the frame repeats an earlier render: the 15
fps grid point had no recorded frame within half a step, or the same one as the point before), `state_skew_ms` and
`action_skew_ms` (how much older the state reading and the command behind the frame, received at
`capture_timing.jsonl`'s `pc_action_recv_mono_ns`, are than the frame). The per-link columns are there when clearance was
computed (workstream C's `teleop_lerobot/clearance.py` and the geometry under `--geometry-dir`, default
`~/isaacsim/assets/clearance`); without, the `min_` columns are NaN.

`report.json`: the dataset's `episodes`, `frames`, `seconds`, `scenarios`, `outcomes`, `contactEpisodes`,
`tableContactEpisodes`, `contacts` by kind, `unsafeFrames`, `unsafeRanges`, `replayGroups`, `roles`, `layoutHashes`,
`layoutSeeds`, `clearanceEpisodes`, `timingFlagged` (episodes), `split` (`part`, `of`) and the build's `options`
(`caption`, `tableSafe`, `geometryDir`, `stopModel`: `{"stopTimes": {name: seconds}}` or null, `commandClearance`).

`commands.parquet` and `pose_samples.parquet` (docs/adr/0038 L, since 2026-11-24; `--no-command-clearance` writes neither,
as builds before): obstacle distances for a robot-side safety check, from `teleop_lerobot/command_clearance.py`. The
distance is `clearance.py`'s exactly (checked to 1e-16 m against it), at made-up poses: the same link boxes and obstacle
pieces, GJK apart and the separating axis's depth overlapping, object poses `WO` interpolated to the row's time, capped at
150 mm (id null there). Every distance column is signed (negative: into an obstacle). The `min_` and `path_min_` columns
are over the links a joint moves: the base never moves, so an object placed near it would set every row's minimum (its own
distance is in its per-link columns). Without geometry (or the take's `robot-profile.json`), the rows are still written,
with NaN distances, and `episodes.jsonl`'s `commandClearanceUnavailable` says why. Each file's parquet metadata (`teleop`
key, JSON) describes it.

- **`commands.parquet`, one row per action event** of `raw_ros_events.jsonl`, at the commands' own rate, not resampled:
  every `RO` (what RobotHost sent the relay for an accepted command, or of its own while parking) and every command
  RobotHost refused, as the plant would have sent it. These events are what the frames' `action` is made of, so each
  frame's `action` equals the last row received at or before the frame was captured (`safety-report` checks it).
  - Columns: `episode_index`, `frame_index` (the last frame captured at or before `received_ns`; null before the first),
    `received_ns` (the event's `pc_received_mono_ns`: RobotHost's CLOCK_MONOTONIC, `capture_timing.jsonl`'s clock), `seq`
    (the `JC` it carries; an `RO`'s is the newest accepted one), `outcome` (`A` sent to the relay, `R` refused), `accepted`,
    and `servo_1` to `servo_5` and `servo_10`, exactly as the action has them (relay pulses).
  - `state.servo_*` and `state_ns`: the arm as measured when it arrived (each servo's newest valid `RI` reading at or
    before `received_ns`; the gripper's last relay target when it has none), and the oldest reading's time.
  - At the commanded pose: per link `clearance_mm.<link>`, `clearance_id.<link>`, `surface_mm.<link>` (as
    `frames.parquet`'s), then `min_clearance_mm`, `min_clearance_id`, `min_clearance_link` and `min_surface_mm`.
  - Along the way: `path_s` (how long the slowest servo takes), and `path_min_clearance_mm`, `_id`, `_link`, `path_min_step`
    (1-8) and `path_min_surface_mm`: the smallest over 8 poses from the measured state to the commanded pose, at 1/8..8/8 of
    `path_s`. Each servo ramps linearly in pulses at the event's rate (an `RO`'s pulses per second, a refused command's
    speed) and holds at its target, as the twin's servo setpoints do (`teleop_isaac/servo.py`, `TimedSetpoint`). Not
    modelled: the setpoint starts from where its last ramp stood, not the reading, and PhysX's drive trails it. NaN before
    the first reading.
- **`pose_samples.parquet`, 32 made-up poses a frame,** never executed: geometry only, in that frame's scene (object poses
  at its `sensed_ns`). Columns: `episode_index`, `frame_index`, `sample` (0-31), `kind`, `servo_*` (the pose, as the
  action), `scale`, `target_mm`, `target_id`, and the distance columns above. Kinds, in this order:
  - 8 `beyond`: the measured state plus `scale` (stratified in 1.5-3) times the frame's action less the state, past the
    command along its direction;
  - 16 `reachable`: uniform within what each servo reaches in 0.5 s at the mixed script's caps (`MixedOptions.DefaultCaps`,
    3.8/2.6/3.6/2.8/4.0 rad/s; the gripper at 2.5 rad/s), within the joint limits (the URDF's, within 0..1000);
  - 4 `jump`: uniform within the joint limits, as accident-like jumps;
  - 4 `near`: searched to land `target_mm` (drawn uniform in 0-50) from an obstacle, within 2.5 mm. It bisects a line in
    joint space from the measured state or a reachable pose to a pose with the fingers inside an obstacle (`target_id`;
    the nearest in reach for two, a random one for two), and draws the start again from anywhere when both ends lie on one
    side. With no obstacle in reach nothing can land, and the pose nearest the target is kept.
  - Seeded per episode: `episodes.jsonl`'s `poseSampleSeed` (sha256 of `<layoutSeed>|<session>`, 53 bits), so a rebuild
    draws the same poses whatever the episode's index.
- **Cost:** about 2 s of a worker an episode (2.05 s average, 2.92 s at most over 10 twin takes of 2026-11-24 with
  `--jobs 4`). It runs in the workers beside video encoding, so the build took no longer (49.7 s with, 51.9 s without).
(`caption`, `tableSafe`, `geometryDir`, `stopModel`: `{"stopTimes": {name: seconds}}` or null, `visibility`: true or
false; a report without it is false).

`visibility.parquet` (`teleop_lerobot/visibility.py`, since 2026-11-24; `--no-visibility` writes none, as builds before):
whether each object on the table could be seen, one row per (episode, frame, camera, object). World-models' safety model
sees only the images; this says how much of each obstacle each camera showed.

| Column | Meaning |
|---|---|
| `episode_index`, `frame_index` | the episode and its frame, as `frames.parquet`'s |
| `camera` | as the dataset names it: `front` (the twin's `wrist`), `third-left`, `third-right` |
| `object_id`, `role` | the scene object and its `role` in the scene file: `obstacle`, `stand`, `plank`, `support` (the obstacles in reach), `distractor` (in view, out of reach); empty in a scene without roles. Every object on the table at the frame (not parked); the table and the flat clutter are left out |
| `visible_px` | the object's pixels in that camera's instance mask (`isaac/<camera>.mask.frames`, id `100 + object_id`): what the camera showed of it |
| `expected_px` | its whole silhouette in the image with nothing in front: its collision geometry (the geometry cache's convex pieces for a mesh, the scene file's box or 32-sided cylinder) at the frame's pose (`WO` interpolated to the frame's `stateSensed`) and size (`WZ`), projected through the take's `CM` camera, clipped to the near plane, the union of the pieces' hulls, a pixel counted when its centre is inside. A room camera's pose is `CM`'s; the wrist camera's is `wrist_link`'s forward kinematics at the frame's `VQ` times `CM`'s. Pixels outside the image are not counted |
| `visible_fraction` | `visible_px / expected_px`; NaN when nothing is expected. Slightly under 1 for an unoccluded mesh (the convex pieces stand a little outside the render mesh) |
| `in_frustum` | the silhouette covers at least one pixel of the image (`expected_px > 0`) |
| `reason` | why a value is null, else null: no mask stream or no mask for the frame, no geometry for the object (`--geometry-dir`), no joint values for the wrist, or an object whose mask id is in no mask of the take though its silhouette reached 200 px (drawn without its label: the twin's masks of 2026-11-24 lose some objects for whole takes). An episode whose objects cannot be known (no `scene.json`, `WO` or `CM`) has one row with no camera or object, and the reason |

No depth test: the mask already resolves what stands in front. Checked against the masks where nothing stands in front of an
object (48 takes of 2026-11-24, every 25th frame, silhouettes of 150 px or more): visible over expected a median 1.000 for
boxes and cylinders (IoU 0.998) and 0.986 for meshes in the room cameras (IoU 0.982), 0.94 for meshes in the wrist camera,
which sees them closer (docs/adr/0038 L). It costs about 1.3 s a take of about 260 frames on one core, in the build's
`--jobs` workers.

`safety-report` sums it up per dataset: the share of obstacle-frames (an obstacle at one frame) at least 50% visible in two
or more cameras, in one, in none, and in no pixel of any camera; each camera's share and its frustum share; and the share of
contacts with an object whose object was at least 50% visible in at least one camera at some frame of the second before the
contact began (and at the frame before).

`unavoidable_ranges.json`, for analysis: the labels stay contact. Written by default since 2026-11-23 and **provisional
until the real arm's stop latency is measured** (docs/adr/0038 item 17). It gives the frames from which stopping could no
longer avoid the next contact:

```
{"source": "twin stop-latency 2026-11-23, provisional until the real arm is measured", "method": "...",
 "stopTimes": {"fast": 0.17, "median": 0.245, "slow": 0.31},
 "unavoidable_ranges": {"fast": {"<episode>": [[start, end], ...]}, "median": {...}, "slow": {...}}}
```

- **The rule.** Stopped at a frame, the arm goes on for the stop time of its speed (travel = speed x time) along the path it
  was on. So a frame that is not unsafe itself is unavoidable when the next contact the labels count starts within the stop
  time: an obstacle, or the table unless built `--table-safe`. This is exact at a constant speed; where the recorded motion
  speeds up or slows, it follows it.
- **The ranges** are inclusive and episode-relative, one set for each stop time; an episode with none is left out of a set.
- **The default stop times** are the twin's fast, median and slow stops: `just isaac-stop-latency` on twin 0 gave per-joint,
  per-speed medians of 170-307 ms, net of the network round trip.
- **Options.** `--stop-time-s SECONDS` gives one time instead, named `given` (its `source` says so); `--no-unavoidable`
  writes no file, as builds before 2026-11-23.
- **Obstacle contacts alone:** with a table-counting build, the obstacle-only ranges for a stop time T are the frames of
  `frames.parquet` with `in_contact` false and `time_to_contact_s` at most T.
- A system that detects and then stops adds its own delay to the stop time.

**Comparing** (`safety-compare REFERENCE REBUILT`): a rebuild of a reference's first takes, file for file; the sidecars by
episode as the label files are. Rebuild a reference with the options `safety-options REFERENCE` prints, from its
`report.json`; `just isaac-safety-build-check` does that by itself. A reference from before docs/adr/0038 has no sidecars
and is compared without them (`--caption category`). A report from before 2026-11-23 names the table's option
`tableUnsafe` (false: `--table-safe` now) and gives a stop time as `stopTimeS` (`stopModel` null: `--no-unavoidable` now);
`safety-compare` reads its options by today's names. It compares `unavoidable_ranges.json` set by set over the rebuilt
episodes, and leaves out which part of a split build a report was (`split`), since a rebuild of a part's takes is one
dataset.
