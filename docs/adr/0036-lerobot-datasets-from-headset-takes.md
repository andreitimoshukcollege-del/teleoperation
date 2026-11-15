# ADR 0036: LeRobot datasets from the headset's takes

## Status

Accepted, 2026-11-12.

## Context

The lab trains world models on LeRobot v3 datasets. Its recorders (`SINRG-Lab/world-models`, `data_collection/`) write
them live on a PC:

- **The JetRover's** joints come over SSH from a ROS bridge on the Jetson, and its camera from the robot's camera feed
  (ADR 0014).
- **The simulated FR3 Duo's** frames come over a TCP socket from an Isaac script.

State, action and camera are sampled in three independent loops and matched only by a staleness check of about 1 s.
Their README's TODO says the timing is not good enough for training: the Jetson and PC clocks have no known offset, and
nothing aligns the streams below a second. Their labeling tools (`qa_review.py`, `extract_episode_frames.py`) read the
camera `observation.images.front`.

**What this repo already records.** The headset's RECORD button (ADR 0033) records each take as a session:

- every writer stamps CLOCK_MONOTONIC, the clock RobotHost reads;
- in Isaac, every rendered frame names the physics state it shows (`VF.stateSensed`, ADR 0032 decision 6);
- a take is complete once RobotHost has checked every writer's trailer and written `files.sha256`.

What is missing is LeRobot output. On the real JetRover, RECORD is not available at all, and sessions hold no camera
frames. RECORD is unavailable because `deploy-robothost` does not pass `--scenarios`. The frames are missing because
CameraHost streams the camera's own MJPEG and writes nothing to disk.

**The user's choices (2026-11-12):**

- every headset take, on Isaac (FR3 Duo and JetRover) and on the real JetRover;
- one LeRobot dataset per scenario, each kept take one episode;
- converted automatically, with no step for the person recording;
- the lab's schema (keys and units as its recorders write them), with this repo's camera names.

## Decision

1. **Takes stay the source of truth; LeRobot is a copy made from them.** A converter on the GPU machine
   (`robot/lerobot/`, package `teleop_lerobot`) turns each complete take into one episode of its scenario's dataset.
   - Nothing records twice, and a dataset can always be rebuilt from its takes.
   - The converter reads the session's own records, which already share one clock. It never samples a live stream.
2. **The schema is the lab's.**

   | Robot | `observation.state`, `action` (float32) | `observation.images.*` (RGB video) |
   |---|---|---|
   | JetRover, real | 6: `servo_1` … `servo_5`, `servo_10`, raw pulses | `front`, the robot's camera |
   | JetRover, Isaac | the same | `third-left`, `third-right`, `wrist` |
   | FR3 Duo, Isaac | 16: `left_joint1` … `left_joint7`, `left_gripper`, `right_joint1` … `right_joint7`, `right_gripper` | `third-left`, `third-right`, `wrist-left`, `wrist-right` |

   `robot_type` is `jetrover` or `fr3_duo`. Every frame's `task` is the scenario's description from `scenarios.txt`, or
   its name when there is none.
3. **Each frame's joints are the ones at the time the frame shows.** Call that time `t`. In Isaac it is the frame's
   `stateSensed`; on the robot it is the camera's `captureTicks`, the start of exposure.
   - **JetRover `state`:** the newest valid `RI` pulse at or before `t` for motors 1–5. The gripper's slot holds its
     newest `RO` pulse, which is the lab's convention: the gripper as a commanded belief (ADR 0018).
   - **JetRover `action`:** the newest `RO` at or before `t`, all six motors.
   - **FR3 `state`:** the `QS` whose `sensed` is `t`, an exact join. Joints are in radians. Each hand's 0..1 opening is
     turned into one finger's travel in metres, the opening times the profile's `fingerTravel` (0.04 m).
   - **FR3 `action`:** the newest `AO` at or before `t`, in the same units.
4. **The rate is fixed.** Episodes are written at 15 fps by default, on a uniform grid from the episode's first frame.
   - Each grid point takes the newest recorded frame at or before it. A gap in the recording is filled by repeating the
     last frame, and the timing file flags it.
   - **Start and end:** an episode runs from the take's first accepted operator command (`JC` outcome `A`) to its last.
     Parking and the idle seconds around a take are left out.
   - **Frames left out:** frames that have no state (the session's last ~0.1 s, after RobotHost closed it), and frames
     whose backdrop may still be settling (`B`).
5. **Folders and bookkeeping.** Datasets live at `~/isaacsim/lerobot/<robot>-<source>/<scenario>/`, so sim and real
   data, and the two robots, never share a dataset. The three folders are `jetrover-real`, `jetrover-isaac` and
   `fr3_duo-isaac`.
   - **`meta/teleop/takes.jsonl`:** one line per episode. It gives the take's path, the sha256 of its `files.sha256`,
     the episode index, the number of frames and how many were filled.
   - **`meta/teleop/episode_<n>.timing.jsonl`:** one line per frame. It gives the frame's source time and frame number,
     and the state's and action's ages at `t`, so frames can be filtered before training.
   - **Sharing:** the folders are shared read-only with the labmate, as the takes are (ADR 0033 decision 6).
   - **The lab recorder's own files (2026-11-13):** `capture_timing.jsonl` and `raw_ros_events.jsonl` at the dataset's
     top, in `record_jetrover.py`'s field names, from the take's one clock. Its `*_wall_ns` fields are null: a take
     keeps no wall clock per record.
6. **When a take is converted.** A take is ready once it has `files.sha256`, `sha256sum -c` passes, and it has sat for
   60 s, so a DISCARD LAST pressed straight after it wins. The watcher scans the scenario roots every 10 s.
   - It runs from its own checkout of a pushed commit, in its own venv with the lab's `lerobot` version.
   - `just isaac-host` and `just isaac-jetrover-host` start it if it is not running.
   - A take already in `takes.jsonl` is never converted again.
   - **Rebuild:** a converted take that has since been set aside (moved to `_discarded`) makes the converter rebuild
     that scenario's dataset from its kept takes. It writes the new dataset into a new folder, then swaps it in. LeRobot
     has no episode deletion, and a rebuild is deterministic.
7. **The real robot records its camera** (ADR 0014 amendment 3) and gets the RECORD button (ADR 0033 amendment 1). Its
   complete takes are copied from the Jetson to the GPU machine by the watcher, with `rsync`. The rule is
   `fetch-recordings`': complete takes only, never the open one.
8. **What is not converted:** depth, masks, camera poses and intrinsics, the network conditions, and every other record.
   They stay in the take, which the dataset's `takes.jsonl` names. They include the privileged simulator labels (ADR
   0032) that must never be a model input.

## Consequences

- **The lab's tools read these datasets unchanged.** On the real robot the camera is `front`; the Isaac cameras keep
  their own names, which their labeling scripts would need to be told.
- **The timing problem of the live recorders does not arise.** A frame's joints are the ones at the frame's own time,
  on one clock. The timing file says how old each was.
- **The person recording still only presses RECORD.**
- **Disk:** the datasets add their encoded video next to the takes. The real camera adds about 4.5 GB an hour to a take
  (30 fps of ~42 KB MJPEG frames), on the Jetson until it is copied off.
- **The converter depends on two contracts:** `trainer/contract/SESSION_FORMAT.md` and the lab's schema. A change to
  either is a change here. A schema change is applied by rebuilding the datasets from their takes.
- **What only a person can check:** that the lab's code trains on these datasets as it does on its own.

## Alternatives considered

- **Their live recorders, run beside RobotHost.** They would sample three streams on two clocks, and CameraHost serves
  one subscriber (ADR 0014), so a second camera client would take the feed from the headset.
- **Writing LeRobot from RobotHost or the renderer as they record.** `lerobot` needs its own Python (it is not importable
  from Isaac's, and the Jetson's is 3.8). A crash mid-take would also leave a half-written dataset, with no take to
  rebuild it from.
- **Radians for the JetRover:** cleaner, but the lab's training and labeling code would need changes. The takes keep
  everything needed to write that later.
- **One dataset per take:** many small datasets, where LeRobot expects episodes within one.

## Amendment 1 (2026-11-14): safety datasets from the JetRover twin, labelled by physics

**Context.** The labmate's safety head (`SINRG-Lab/world-models`, `cosmos-JetRover/training/`) learns a per-frame
safe/unsafe label from JetRover video. His sessions are labelled by hand: `data_collection/labeling/label_safety.py` turns
`manual_safety_labels.json` (`{"<episode_index>": [[start, end], ...]}`, inclusive, episode-relative unsafe ranges; an
episode left out is all safe) into `meta/safety_labels.parquet` (one row per frame: `episode_index`, `frame_index`,
`label`, `category`, `effective_category`, `overridden`, `source`), which training reads (`episode_index`, `frame_index`,
`label`; 16-frame chunks, a chunk unsafe if any frame is). His memo calls the JSON the source of truth. An episode's
category is its task string, one of `reach_no_obstacle`, `stop_before_collision`, `go_around_obstacle`,
`collide_with_obstacle`, `collide_and_recover`; his older path labels from `safety_events.jsonl` `contact`/`clear` markers
with a 1 s lead window. His review tools read `observation.images.front`.

The user wants (2026-11-14) four scenarios recorded on the JetRover twin, scripted and unattended, every frame labelled
safe or unsafe, **unsafe while any arm link touches an object** (the table does not count), **in his format, unchanged**.

**Decisions.**

1. **The label is physics.** The Isaac bridge reports contacts between the arm's links and the scene's objects each
   physics step on the arm-server wire's existing contacts message; RobotHost records them as `WC` (SESSION_FORMAT.md),
   which Isaac did not send before. A frame is unsafe when the arm touched an object at any moment of its frame period:
   decision 3's `t`, half a frame either side (at 15 fps a frame lasts 67 ms, and a recovering touch can be shorter, so
   the instant alone could miss it). A contact no `WC` ends lasts to the take's end. No frame is labelled by eye.
2. **Four scripted scenarios** (`joint-drive`), each a take of the scenario named by its category:

   | Scenario | Category (take scenario and task string) | The script |
   |---|---|---|
   | free movement | `reach_no_obstacle` | moves about, every link kept a margin from every object |
   | close to the object | `stop_before_collision` | approaches an object and stops 1–3 cm short |
   | collide, then recover | `collide_and_recover` | touches an object and backs off before it tips |
   | full collision | `collide_with_obstacle` | drives through an object and knocks it over |

   The scripts aim at the objects where they are: RobotHost already streams the world's poses to the joint client, and
   the layout changes every take. Their takes are labelled a tool's (operator `tool <ip:port>`, the trainer's scripted).
   **Driven as an operator drives (2026-11-15, the user's ask):** a dragged point, not moves between poses. The wrist
   follows smooth strokes (through reachable points, a hand's speed profile at the clutch's half scale, pauses as when the
   clutch is let go, a small tremor), and the headset's IK is solved once a 48 Hz tick from the pose before, as the headset
   solves: the commands are a dense list of targets along a path, the input the lab's model is given. The sweep into an
   object is timed the same way (easing to a stop, easing into a touch and back, accelerating through), and a touch's
   depth is measured into the object's box, not along the sweep (a grazing sweep slides along a box).
3. **Each take is checked against its scenario before it is kept.** No contact for `reach_no_obstacle` and
   `stop_before_collision`; a contact that ends with the object upright for `collide_and_recover`; a contact after which
   the object has tipped or moved for `collide_with_obstacle`. A take that fails is set aside (`_discarded`), with the
   reason, and never relabelled into another category.
4. **His format, file for file.** One dataset per batch, `~/isaacsim/lerobot/jetrover-isaac-safety/jetrover_<UTC>_<id>/`,
   holding all four categories, as his sessions mix them (his training splits one session's episodes into train and
   test, so a dataset of one category has nothing to learn the boundary from):
   - `observation.images.front` is the twin's **wrist** camera: the real JetRover's camera rides the arm (ADR 0014; ADR
     0034 amendment 10), and that is his real sessions' `front`. The third-person cameras stay as `third-left` and
     `third-right`; his tools ignore them.
   - each episode's task string is its category;
   - `manual_safety_labels.json`: the physics contact ranges in his schema;
   - `meta/safety_labels.parquet`: the rows `label_safety.py` writes from that JSON, `effective_category` the category,
     `overridden` false, `source` `physics`;
   - `safety_events.jsonl`: per episode the first contact frame (`contact`) and the frame after the last (`clear`), so
     his `label_safety.py` without the JSON gives his lead-window labels;
   - decision 5's `capture_timing.jsonl` and `raw_ros_events.jsonl`, as for every dataset.

   The watcher leaves these four scenarios out of its per-scenario datasets (decision 5); the batch builds its dataset
   from its kept takes when it ends, and can rebuild it the same way.
5. **Recorded after the renderer stops smearing motion.** Isaac's real-time renderer blends each frame with earlier
   ones (DLSS Performance and temporal accumulation, measured 2026-11-14), which smears the moving arm across exactly
   the frames these labels are about. The render settings are fixed first, and every session records which it used.

**Consequences.**

- Labels are exact and free, and as true as the twin's contact geometry: the twin's collision shapes, not the real
  arm's (ADR 0034).
- Decision 2 of the main ADR (camera names ours) gives way for this dataset only: its `front` is a renamed `wrist`.
- What only a person can check: that the labmate's training reads these datasets as it reads his own, and whether he
  trains on the exact labels or his 1 s lead window (both are in the dataset).
