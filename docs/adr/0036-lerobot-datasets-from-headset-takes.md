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
   - Each grid point takes the recorded frame nearest it, the earlier on a tie, if one is within half a step. A gap in
     the recording is filled by repeating the last frame, and the timing file flags it. (Until 2026-11-15 a point took the
     newest frame at most 1 ms past it. The twin's frame times wobble by ~3 ms, so at 30 fps written as 15 that picked
     frames 1, 2 or 3 apart, a 5 Hz judder in 80 of the first safety batch's 85 episodes. The datasets were rebuilt.)
   - **Start and end:** an episode runs from the take's first accepted operator command (`JC` outcome `A`) to its last.
     Parking and the idle seconds around a take are left out.
   - **Frames left out:** frames that have no state (the session's last ~0.1 s, after RobotHost closed it), and frames
     whose backdrop may still be settling (`B`).
5. **Folders and bookkeeping.** Datasets live at `~/data/lerobot/<robot>-<source>/<scenario>/`, so sim and real
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
4. **His format, file for file.** One dataset per batch, `~/data/lerobot/jetrover-isaac-safety/jetrover_<UTC>_<id>/`,
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
   **Done 2026-11-15:** the renderer now runs the real-time path tracer with anti-aliasing and motion blur off and renders
   each pose twice before reading it (`--subframes 2`), recorded as `RX` in `isaac.jlog`. Measured on the 5090 with the
   bench scene's three cameras: 29 fps, its lag stable, still frames sharp and a swinging arm (2.5 rad/s) solid. Three
   subframes ran at 21 fps and ghosted more, its poses further apart.

**Consequences.**

- Labels are exact and free, and as true as the twin's contact geometry: the twin's collision shapes, not the real
  arm's (ADR 0034).
- Decision 2 of the main ADR (camera names ours) gives way for this dataset only: its `front` is a renamed `wrist`.
- What only a person can check: that the labmate's training reads these datasets as it reads his own, and whether he
  trains on the exact labels or his 1 s lead window (both are in the dataset).

## Amendment 2 (2026-11-15): driven as a person drives, and frames as sharp as a moving camera allows

**Context.** The user looked at the first drag-path dataset (amendment 1, decision 2) and found the motion robotic (it goes
to a point and stops, then to another and stops) and the video low quality and blurry in motion. Both were measured.

- **Motion.** 31 human headset sessions (12 on the robot, 2026-10-25..27; 19 on the twin, 2026-11-08..10; 55 min, 117k
  commands) against the scripted takes, on 45 measures with a band each from the human data (the robot's and the twin's
  operators agree in structure, within ~15%). The drag path was outside the band on 37 of 45, the pose-to-pose scripts
  before it on 32. People chain overlapping submovements (1.55 speed peaks a second of motion, 26% of bouts with one peak,
  the speed between peaks at 22% of them); the script drew one minimum-jerk bell per stroke to rest (0.01 a second, 88%).
  People let go of the clutch while still moving (75% of holds) and take it on the move (46%); the script never did.
  The script was 3-4x too slow (32 vs 98 mm/s while moving), its strokes straight (chord/path 0.96 vs 0.37), its tremor
  a fixed 8-12 Hz sinusoid (people's jitter is broadband and grows with speed, ~80 v^0.78 mm/s in 4-15 Hz), it worked too
  low and far out, and never used the roll or the gripper (people: roll moving 41% of the time, 14 gripper actions a
  minute).
- **Video.** Decision 5's "anti-aliasing off" never took effect: Isaac 6.2's Real-Time 2.0 takes only 3 (DLSS) or 4
  (DLAA) and turns anything else back into 3, and opening a stage puts its render settings back. Every frame since was
  DLSS Performance: each camera rendered at half its width and height and rebuilt from earlier frames, which smears
  everything in view when the camera moves, and the wrist camera, the datasets' `front`, always moves. Decision 5's test
  had watched a static camera. Against a supersampled reference of the same poses, its still wrist frames kept 66% of the
  detail, its moving ones 63%. The wrist frames averaged luma 32 of 255 (the real camera's 129), and LeRobot's AV1 at CRF
  30 kept 89% of what was recorded.

**Decisions.**

1. **A person, modelled** (`joint-drive`'s `HumanOperator`, `HandModel` holds every number). The hand's intended path is a
   sum of overlapping minimum-jerk submovements (each starts 40-80% into the one before; durations on the human main
   sequence, T ~ A^0.22; a skewed time base, so speed peaks early), grouped as an operator groups them: moves to a goal
   (an endpoint error, then corrections), back and forth (bowed, so a reversal keeps moving), small adjustments; between
   them the hand carries on, keeps still with the clutch held, or lets go. The clutch maps the hand onto the wrist's target
   as the headset's `ClutchMapping` does (relative to engage, kept in reach and above the floor, held bit-identical while
   let go, re-anchored on engage); a release mostly comes while the hand still moves (the rest of its submovement is lost)
   and the clutch is often taken again on the move. The hand as tracked drifts, and jitters in proportion to its speed
   (v^0.78). Pitch, roll and gripper move as people move them (the gripper only while the clutch is held). Every tick is
   solved by the headset's IK from the pose before and checked (envelope, collision model, the objects grown); a part that
   fails is drawn again. Fitted with the analysis's own measures: ten 90 s drives of free movement fall inside 41-43 of
   the 45 bands from run to run. Two are always out, the command rate's: `joint-drive` sends at 48 Hz, the headset in
   Direct Preview at 36 Hz (every second frame of 72 Hz), and an APK on the device may differ; unchanged. Two sit at a
   band's edge: single-peak bouts peak at 0.44-0.49 of their length (people 0.34, band to 0.45), and with holds capped at
   6 s (an episode lasts 15-25 s; two of the first batch's takes held still for 9 s) their p95 measures 3.4 s (people
   5 s, band from 3.6 s).
2. **The categories keep their geometry; the approach becomes a person's.** The sideways sweep round the base still says
   where an object is met. The final approach follows it as a chain of overlapping submovements through points at most
   4 cm apart (a straight move cuts 2.4 cm inside the arc over a 20 cm swipe), slows short of its aim and corrects (a
   touch is a gentle push in), and how far it goes is one number, bisected with every random draw held fixed until the
   collision model's closest approach, or depth, over every tick is the category's: 1-3 cm for a stop; the touch depth
   within 1.5 mm, in contact 0.12-1.2 s and clear within 0.6 s of pulling back; at least 12 mm deep for a swipe, which
   keeps the other objects at the approach clearance until it meets its own. The gripper is opened on the way, as the
   touch depth was measured; the roll is held through the approach. Free movement before an approach: 3-8 s.
3. **DLAA, set once the stage has loaded, and checked.** The renderer sets `/rtx/post/aa/op` 4 (DLAA: each camera's own
   resolution, settled in one frame), `/rtx/rtpt/spp` 2 and motion blur off after the stage loads, and stops if Isaac has
   not kept them, then and at the start of the stream; one render a pose (`--subframes 1`). `RX` records the samples a
   pixel too. Measured with the three cameras recording rgb, depth and masks: 30.0 fps, the lag check at 100%; still wrist
   frames keep 93-96% of the reference's detail, moving ones 71-72%.
4. **Brighter.** The bench scene's lights x3 (dome 1000, key 3000): the wrist camera's mean luma 69, the third-person
   cameras' 141, unclipped.
5. **Encoded at CRF 18.** The safety datasets' videos are AV1 at CRF 18, preset 8: 96% of the recorded detail, 1.6x the
   size (15.7 MB a minute a camera). LeRobot 0.5.1 takes no CRF from `LeRobotDataset`, so the writer hands one to its
   encoder call while an episode is saved, and `meta/teleop/encoding.json` says which. The watcher's per-scenario
   datasets keep LeRobot's default for now: their episodes are appended to existing files.

**Consequences.**

- Free movement is a person's by every measure but the command rate. An approach and its event are a person's in
  texture (overlapping submovements, corrections, releases on the move, jitter) on a script's geometry, and score as
  such on the measures the task sets (the wrist low, the pitch held through an approach, the gripper open).
- The datasets of amendment 1 are DLSS-smeared, dark and drag-path: kept, never mixed with new ones.
- One operator's statistics, probably, all through Direct Preview; a second operator or an APK session may move
  differently.
- A contact take takes ~2 s to plan; every category plans on at least 80% of random layouts.

## Amendment 3 (2026-11-16): the safety takes' motion flows, and takes run on from one another

The user judged the safety takes' motion stop-and-go: every take started and ended at the same home pose, and the hand
paused often. That was the fit of amendment 2: people pause between 37% of their moves and let go of the clutch in most
of those pauses, for holds up to 6 s. The user chose a continuous motion over that fit.

- **Pauses are rare and short.** The hand carries straight on 92% of the time (it was 63%). A pause is a brief dwell,
  lognormal with median 0.18 s and at most 0.5 s. The clutch is never let go mid-take: no releases, re-pinches or
  mid-move releases. A stop event holds 0.3-0.5 s.
- **No move home.**
  - A safety take starts wherever the last one left the arm, when that pose is inside the envelope and clear of the new
    layout's objects by the scripts' clearance; otherwise it goes home first, as before.
  - After its event, the hand moves on toward a point nearby, and the plan ends partway (50-80%) through that move, never
    during the event. The episode therefore ends with the arm moving, and DrivePlan adds no return leg.
- **What no longer holds.** The motion no longer matches amendment 2's human bands for holds and releases; the
  submovements still overlap as a person's do, at the speed people move.
- **Tests:**
  - `TheHand_MovesContinuously_InOverlappingSubmovements`: no release, at least 90% of ticks moving, no stillness over
    0.7 s with the smoothing, and the overlap and speed bands as before.
  - The scripts' own tests still hold every category to its event, with the cut never inside it.

## Amendment 4 (2026-11-16): a touch is a touch, not a pair inside PhysX's margin

**Measured** on the 98 takes of the first batch with turned boxes (read-only, after 2026-11-16T23:10Z):

| Category | Kept | Set aside | Takes with any contact record |
|---|---|---|---|
| reach_no_obstacle | 25 | 0 | 0 |
| stop_before_collision | 24 | 0 | 0 |
| collide_with_obstacle | 25 | 0 | 25 |
| collide_and_recover | 12 | 12 | 12 (the kept ones) |

- **Every contact stretch began with records at 0 N**, for 20–200 ms. That was PhysX reporting the pair once it came within the
  shapes' default contact offsets, before any push, so unsafe labels started up to 3 frames early. Stretches also went on
  at 0 N after the push while the arm hovered inside the margin.
- **5 of the 48 collide_with_obstacle stretches never had any force** (12–51 ms each). They were proximity, not touches.
- **Forces once touching:**
  - collide_and_recover: peak median 1.84 N (p10 0.96, p90 14.5), duration median 685 ms;
  - collide_with_obstacle: peak median 2.36 N (max 44.9 N).
- **No stop or reach take was set aside** for a weak or brief contact. The set-aside recover takes had no contact record
  at all: they fell short of the real mesh (amendment 5 of ADR 0037 treats that).
- **The colour images do not ghost.** The arm's colour edges sit within 1–1.5 px of its mask edges at every speed, under a
  tenth of the per-frame motion, so `--subframes 1` stays.

**Decision:**
- **The rule.** A link–object pair counts as touching on a physics step only if one of its contact points is at most
  1 mm apart (`contacts.TOUCH_SEPARATION_M`) or PhysX's solver applied an impulse between them that step. A pair that is
  reported but further apart is *near*: it is not a contact, and the bridge counts these near events in its report.
- **Offsets set explicitly** on every collider when contacts are reported: the robot's links, objects, surfaces and the
  shelf.
  - Contact offset is 5 mm, so PhysX generates a pair's contacts within 10 mm (the two offsets summed). That is above the
    1 mm threshold, and over 4× the 2.4 mm a link at 8 rad/s, 0.3 m out, closes in one 1 ms step.
  - Rest offset is 0, so bodies at rest touch at separation 0 and the 1 mm is a real gap.
- **Where.** The rule is applied on the Isaac side, in `contacts.ContactTracker`, which still imports nothing from Isaac.
  `world.contact_events` passes each event's smallest contact-point separation alongside its impulse. The labelling side
  (`teleop_lerobot/safety.py`) is unchanged: what it reads is now a touch.
- **Takes recorded before this** keep their early margin records. They are not relabelled.
