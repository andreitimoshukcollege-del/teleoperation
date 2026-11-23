# ADR 0038: Safety data where the scene decides

## Status

Proposed 2026-11-22. Work in progress: six workstreams (below) build it in parallel, each filling in its own "as built"
section. Extends docs/adr/0036 (safety datasets) and docs/adr/0037 (varied scenes); where they disagree, this one wins for
takes of the `mixed` script.

## Context

The world-models side trained the JetRover safety model on three twin datasets (split v2: train
`jetrover_20261122_005619_20ccb6`, val `jetrover_20261122_145748_82747d`, test `jetrover_20261116_232145_e2369d`). Its
finding (world-models `docs/ROADMAP.md` on branch `andrei-teleop-models`, "The Cosmos latent doesn't help yet") is that a
model that sees **only the commanded path** scores as well as one that also sees the image (test F1 0.60 both ways). The data
lets the path give the answer away:

- **Layouts repeat.** Val and test contacts sit a median 2.5 cm from the nearest train contact, and 90% are within 5 cm.
- **Contacts happen at one height,** with the tool point about 10 cm below the base. How low the path goes scores AP 0.30 on
  its own, against a chance level of 0.08.
- **The motion gives the scenario away.** Only the two collide scripts ever make contact. Each of the four scripts moves in
  its own way, and the task caption names the script.
- **Too little is logged** to test any of this. There are no obstacle geometry sidecars, no continuous clearance, and table
  contacts aren't reported.

They listed 39 changes. This ADR decides how the twin makes data **where only the image can tell the outcome**, and what it
logs so that can be measured.

## Decision

1. **One script, and physics decides the outcome** (items 6, 7, 8, 10, 23 and 24).
   - A new joint-drive script, `mixed`, replaces the four scripted categories for new data. Its scenario folder is also
     `mixed`. The four old scripts stay, for comparison only.
   - A take is a run of operator-like moves (HumanMotion's fitted operator) drawn from **two sources mixed together**:
     - **Layout-blind waypoints** anywhere in the reachable workspace, including low reaches near the table. These carry
       random speed, curvature through via points, pauses and reversals.
     - **Object-directed moves.** Each aims at a point on an object's own shape (`shapes.json`), from a drawn direction and
       height, with a **signed offset drawn from one continuous range**. The range runs from a few cm into the object (a
       touch, or a knock when deep) to several cm short of it (a near miss).
   - Because the offset is continuous, a reach that touches and a reach that stops short move the same way, and only the
     geometry separates them. Contacts may fall at a move's start, middle or end.
   - **Nothing checks a take against a category.** A take is set aside only when it is broken, for example no frames or a
     watchdog stop. What happened is computed afterwards (decision 6) and kept as metadata.
   - **Timing rules:**
     - no first contact before 1.5 s;
     - at least 1.0 s of recording after the last contact;
     - the arm moves from the first frame, with no idle start.
   - **Speed caps** follow the real arm's measured tracking and the real teleop motion scale (workstream R), not a guess.

2. **Counterfactual replays** (items 1 and 11).
   - After a `mixed` take, joint-drive may record `--replays N` more takes. Each re-sends **the same commanded joint stream**:
     the same targets at the same relative times, with no re-planning.
   - Before each replay, the source layout is restored (same objects, sizes and poses) and **one edit** is applied:
     - remove an object;
     - shift it a few cm along the approach direction (a hit/miss pair);
     - or move it elsewhere in reach.
   - Every take of the script writes one `RP` record (robothost.jlog) with fields `t group role edit`:
     - `group` is an id joint-drive draws for the source take (unique per batch, e.g. `<seed>.<n>`);
     - `role` is `source` or `replay`;
     - `edit` is `none`, `remove:<id>`, `shift:<id>:<dx>:<dy>` (m, base frame) or `move:<id>:<x>:<y>:<yawRad>`.
   - Splits must keep a group together. The dataset's episode metadata (decision 6) carries `group`, `role` and `edit`.

3. **Obstacles placed at random, varied, and counted** (items 2, 3, 4, 5 and 22).
   - **Placement:**
     - Each layout draws **0-3 obstacles in reach**, a uniform count, placed uniformly over the arm's reachable workspace on
       the table. That includes where layout-blind reaches go.
     - It also draws **distractors**: objects in camera view but out of reach.
   - **Obstacle shapes vary in size and height:**
     - tall and thin;
     - overhangs, which the arm can meet from the side or from below;
     - small ones;
     - objects raised on stands, so contacts happen at different tool heights.
     - Everything wears a real texture: no untextured primitives, per the user's earlier review.
   - **Layout identity and held-out layouts:**
     - Each layout's identity is recorded: the `WL` seed plus a layout hash, computed in the dataset (decision 6).
     - A **held-out layout family** reserves assets, shapes or placements that train layouts never use, for an
       unseen-layout test session.

4. **Clearance and contact details** (items 13, 14 and 18).
   - **Clearance:**
     - Each recorded frame gets, for each arm link, the **signed distance to the nearest obstacle** (any scene object but the
       table: movable objects and fixtures) and to the **table surface**.
     - It is computed from the take's own records (per-frame joint values `VQ`, object poses `WO`, sizes `WZ`, the scene file)
       and the twin's collision geometry.
     - It is capped at a documented cutoff and validated against PhysX contacts (about 0 at every `WC` touch).
   - **Table contacts:** the bridge also reports arm-table contacts, as `WC` object 0, which the format has always reserved for
     the table.
   - **The table is its own kind, not unsafe.** `safety_ranges.json` keeps the v1 rule (unsafe = the arm touches an obstacle).
     Table contacts are labelled separately, in the per-frame flags and the contacts sidecar. Counting them as unsafe would
     reward "the path goes low", the shortcut this ADR exists to remove. A builder flag can count them for anyone who wants
     that.

5. **Looks and cameras vary** (items 30 and 31).
   - Each take draws:
     - obstacle colour and texture, within realistic bounds for scanned objects;
     - lighting: intensity, colour temperature and key direction;
     - a small camera pose jitter for every camera, including the wrist (`front`).
   - All of it is recorded where the frames' records already describe the scene (`CM`, `LK`, `EN`, or a new record documented
     in trainer/contract/SESSION_FORMAT.md).

6. **The dataset says what happened, in the world-models format** (items 9, 12, 14, 15, 16, 17, 18, 24, 27, 28 and 36).
   - **Captions:** the task caption is neutral, `move the arm`, for every episode. The script name moves to metadata.
   - **Labels:**
     - `safety_ranges.json` at the dataset root: exactly `{"source": "physics", "notes": ..., "unsafe_ranges": {...}}`.
       world-models' loader rejects any other key.
     - The legacy `manual_safety_labels.json` (same ranges), `meta/safety_labels.parquet` and `safety_events.jsonl` are
       written too.
   - **New sidecars under `meta/teleop/`:**
     - `episodes.jsonl`, one line per episode:
       - take, scenario, layout seed, layout hash, scene sha;
       - `outcome`: `no_contact`, `near_miss` (minimum obstacle clearance under 30 mm with no contact), `contact` or `knock`
         (an obstacle moved at least 3 cm or tipped at least 30°), plus `table_contact`;
       - the replay group, role and edit;
       - the minimum clearance, the contact count, the room, looks and camera jitter;
       - a summary of the planned moves.
     - `scenes.jsonl`, one line per episode: every object's id, name, asset or primitive, dimensions after its size factors,
       mass, fixture or movable, in reach or a distractor, its pose at the start and its pose changes. Also the table surfaces
       and every camera's intrinsics and pose. This is enough for a geometric hard-safety baseline.
     - `contacts.jsonl`, one line per contact stretch: episode, start and end frame and time, link(s), object and kind
       (`object`, `fixture`, `table`), peak force, and whether the object moved or tipped afterwards.
     - `frames.parquet`, one row per frame:
       - `in_contact` (obstacle) and `in_contact_table`;
       - `unsafe` (the labels);
       - per-link obstacle clearance (mm) with the nearest object's id, and the minimum clearance;
       - time to the next obstacle contact (s; NaN when none follows in the episode);
       - `filled` (the frame repeats an earlier render);
       - state and action skew against the frame (ms).
   - **Unavoidable-contact ranges** (item 17), from the arm's stopping time and distance, go in a separate file. They stay off
     by default until the real arm's stop latency is measured (workstream R gives the tool; the run is the user's).
   - **Sessions:** a build can split one batch into several smaller sessions (datasets) by layout, so world-models gets more
     independent sessions (item 20). It reports the scenario and outcome mix of each (item 21).

7. **The twin arm follows its commands as the real one does** (items 26 and 29).
   - The twin's tracking lag (state behind action) is measured against real JetRover takes. The cause is then found and fixed:
     either the servo emulation or commanded motion faster than a person drives.
   - The same work measures the per-frame motion scale against real teleop and sets the `mixed` script's speed caps.

## Deferred, or the user's

- **Item 25:** what the deployed system knows of the commanded path (only the current command, or a planned next second).
  This is a question for the user and world-models. The twin keeps logging the commands as sent; nothing here depends on the
  answer.
- **Item 32:** stop latency on the real arm. Workstream R writes the recipe and tests it on the twin. Running it moves the real
  arm, so it waits for the user to confirm the workspace is clear.
- **Items 33-35:** the real collection protocol, twin-matched real scenes, and real camera calibration.
  - The Femto Bolt cameras are not placed for collection yet, and the user said not to use them.
  - Workstream R notes what the real servos can report for contact detection.
- **Items 19-22, coverage:** the collection plan itself (many smaller sessions, 200 or more contacts per evaluation session, a
  held-out-layout test session) runs after the twin check, with the user's go.
- **Items 36-39, world-models side.** This ADR supplies the per-episode layout seed and hash that item 36 needs. Their
  `docs/design/session-format.md` should list the new sidecars once these are built (item 39).

## Workstreams and ownership

Each workstream owns the files listed. It adds to its own "as built" section below and edits no one else's section.

| | Items | Owns |
|---|---|---|
| **S, scene** | 2, 3, 4, 5, 22 | `core/Scenes/jetrover-bench.json` (and its byte-identical headset copy), the shapes file, `core/Teleop.RobotArm/World/*`, RobotHost `World/*` (`WorldHost`, `StaticWorld`), `robot/isaac/teleop_isaac/scene.py`, `world.py` placement, scene vectors and goldens |
| **M, motion** | 1, 6, 7, 8, 10, 11, 23, 24, 29 (caps) | `core/Teleop.Eval/JointDrive/*` (the `mixed` script, replays, `RP` and move records), RobotHost's handling of `RP`, the `isaac-safety-batch` recipe (scenario `mixed`, `replays=`) |
| **C, clearance and contacts** | 13, 14, 18 (twin side) | `robot/isaac/teleop_isaac/contacts.py` and the bridge's contact reporting (table = object 0), the collision-geometry export (`robot/isaac/teleop_isaac/clearance_geometry.py`), `robot/lerobot/teleop_lerobot/clearance.py` (the per-frame computation) |
| **L, labels and dataset** | 9, 12, 14, 15, 16, 17, 18, 24, 27, 28, 36, 20, 21 | `robot/lerobot/teleop_lerobot/safety.py`, `writer.py` and `takes.py` as needed, the new sidecars, `safety-check` for `mixed`, the `isaac-safety-build` recipe |
| **R, arm response** | 26, 29 (measure), 32 (tool), 17 (twin numbers), 33 (servo signals) | `robot/isaac/teleop_isaac/servo.py` and `simfit.py`, analysis scripts (`scripts/gpu/` or `analysis/`), a stop-latency recipe |
| **A, appearance** | 30, 31 | `robot/isaac/teleop_isaac/renderer.py`, `materials.py`, `environments.py`, and the scene's `materials`, `lighting` and camera-jitter sections |

**Shared interfaces, fixed now so the workstreams can build in parallel:**

- `robot/lerobot/teleop_lerobot/clearance.py` (C) exposes `compute(take, geometry_dir) -> Clearance`, a frozen dataclass:
  - `links`: URDF link names, in order;
  - `obstacle_mm`, `obstacle_id` and `surface_mm`, each a per-frame list in `take.frames` order of per-link tuples;
  - `cutoff_mm` and `method`.
  - When a take lacks what it needs, it raises `ClearanceUnavailable`. L calls it and writes NaN when it can't.
- `RP` (M), as in decision 2. L reads it into `episodes.jsonl`.
- `WC` object 0 is the table (C). L labels it `table`, never unsafe by default.
- `SE` stays as documented. The `mixed` script records each planned move in a record of M's choosing, documented in
  trainer/contract/SESSION_FORMAT.md. L copies it into `episodes.jsonl` as metadata only, never as labels.

## Consequences

- **Scene sha:** every change to the scene file changes it, so the headset twin needs `just unity-xr-jetrover-world` and then
  `just build-galaxy` before it drives the twin (the user's step).
- **Old data stays readable:**
  - Old takes and the four old scripts still build. Their sidecars hold what their records allow (NaN clearance without
    geometry, no `RP`).
  - Datasets built before this ADR keep their category captions.
- **Volume:** the replays multiply recorded takes per planned take by `1 + N`. Each replay costs a take's render time, and
  disk on the GPU machine (211 GB free on 2026-11-22).

## As built

### S, scene

(pending)

### M, motion

(pending)

### C, clearance and contacts

(pending)

### L, labels and dataset

(pending)

### R, arm response

**What world-models saw.** In the twin datasets the arm is more than 20 units (4.8 degrees) behind its command on 65–74% of
frames, median about 8 degrees, against real reaches at that threshold of 3–6 frames.

**Measured** with `scripts/gpu/arm_response.py` (standard library, rerunnable; 15 fps frames: the newest reading `RI` against
the newest relay target `RO`, as the dataset builder takes them). Real: the 9 human headset sessions on the robot of
2026-10-25 and 27 (four teleop, three ghost-check, two course; 20,073 frames, 22.3 min of commands) and
`jetrover-real/free-play`, the real dataset world-models holds (2 takes of 2026-11-13, 1,120 frames). Twin: the three safety
datasets and their takes.

| | Real, 9 human sessions | Real, free-play | Twin val / train / test |
|---|---|---|---|
| Frames over 20 units | 62% (36–74% by session) | 75% | 73% / 74% / 65% |
| Largest joint error, p50 / p90 | 7.6 / 28.4 degrees | 12.5 / 32.4 | 9.2 / 30.0 (val) |
| Tool point error, p50 / p90 | 2.9 / 9.2 cm | 4.7 / 12.2 | 3.0 / 9.1 (val) |
| Stretches over 20 units, p50 / p90 | 9 / 29 frames | 8.5 / 30 | 8 / 22–24 |
| Delay from relay target to reading, joints 1–4 | 230 / 170 / 210 / 200 ms | 210 / 180 / 220 / 200 | 210 / 200 / 200 / 190 |

The same arm at the same commanded speed (yaw and pitch joints pooled; median error in units, and the share of joint-frames):

| Commanded speed, units/s | Real, 9 human | Real, free-play | Twin val |
|---|---|---|---|
| 0–25 | 4.3 (45%) | 5.2 (35%) | 3.3 (29%) |
| 50–100 | 15.0 (14%) | 13.9 (16%) | 14.1 (18%) |
| 100–200 | 29.1 (15%) | 27.4 (15%) | 26.7 (18%) |
| 200–400 | 55.0 (11%) | 55.6 (12%) | 51.3 (13%) |
| 400–800 | 102 (4%) | 101 (6%) | 98 (6%) |

Steps (a relay target jumps 40+ units and holds): real at 597 pulses/s reaches 10 / 50 / 90% at 162 / 254 / 340 ms, the twin
at 907 at 137 / 209 / 311 ms; both have about 145 ms of dead time before the ramp. Top speed 917 units/s real (asked 1432),
959 twin (asked 1194). The real servo overshoots 2–4 units and settles 3.0–3.3 from its target; the twin 0.1–0.3 and 1.2–1.9.

- **The emulation is not the cause, and `servo.py` is unchanged.** Same delay, same error at the same commanded speed (the twin
  1–4 units closer), same step timing and top speed. The overshoot and settle differences are one to three pulses.
  docs/adr/0034 amendment 13's fit (held-out p95 2.64 degrees) stands.
- **Being over 20 units most of the time is how this arm teleoperates.** Any arm here trails its command by about 0.2 s, so a
  joint commanded faster than about 100 units/s (0.42 rad/s) is over the threshold (82% of such joint-frames, real or twin),
  and people keep the arm moving 70–86% of the time. The real dataset world-models holds is at 75% itself.
- **The 3–6 frames** come from `jetrover_20261022_084431_89285d` (world-models' `wm/jetrover/windows.py`), a recording of
  2026-10-22 that is not in our data and predates the robot's feedback timing of 2026-10-25. No real recording we have agrees
  with it. World-models should say which recording and which columns it used.
- **History.** The twin's human sessions of 2026-11-08 and 09, before docs/adr/0034 amendment 13, answered in 50–70 ms and were
  over 20 units on 37% of frames. The driver emulation brought the delay to the real arm's.

**The commanded motion (item 29).** Per 15 fps frame, p50 / p90 / p95 / p99:

| | Real, 9 human | Real, free-play | Twin safety val / train / test | Twin, human, 2026-11-08/09 |
|---|---|---|---|---|
| Largest arm-joint step (joints 1–4), rad | 0.031 / 0.140 / 0.194 / 0.341 | 0.051 / 0.173 / 0.238 / 0.338 | 0.047 / 0.195 / 0.282 / 0.506 (val); p99 0.495, 0.453 | 0.012 / 0.166 / 0.257 / 0.535 |
| Wrist pivot step, mm | 3.7 / 14.9 / 19.7 / 35.1 | 5.7 / 19.7 / 23.9 / 32.6 | 5.2 / 18.9 / 26.0 / 44.0 (val); p99 45.6, 44.4 | 1.4 / 19.9 / 30.1 / 54.6 |
| Tool point step, cm | 0.62 / 2.71 / 3.75 / 6.26 | 1.20 / 4.27 / 5.42 / 7.53 | 0.86 / 3.27 / 4.56 / 8.16 (val) | 0.24 / 3.78 / 5.66 / 10.81 |

Workstream M's own figures (12 real human sessions, 21k frames: 0.029 / 0.139 / 0.193 / 0.348 rad and 3.4 / 14.7 / 19.4 /
35.7 mm; 19 twin human sessions: 0.014 / 0.177 / 0.272 / 0.530 rad and 1.6 / 20.9 / 31.2 / 56.3 mm) agree with these.

Per joint at the command rate (smoothed with a 25 ms Gaussian, as `~/analysis/motion/metrics.py` does): speed p50 / p90 / p99
in rad/s, acceleration p90 / p99 in rad/s².

| Joint | Real, 9 human: speed | accel | Real, free-play: speed | Twin val: speed | accel |
|---|---|---|---|---|---|
| Yaw | 0.14 / 1.27 / 3.75 | 10.3 / 40.4 | 0.53 / 2.27 / 4.50 | 0.22 / 1.01 / 2.85 | 9.3 / 27.3 |
| Shoulder | 0.14 / 1.02 / 2.62 | 9.7 / 31.5 | 0.14 / 0.69 / 1.50 | 0.24 / 1.26 / 3.20 | 13.7 / 44.5 |
| Elbow | 0.14 / 1.29 / 3.64 | 13.2 / 51.4 | 0.23 / 1.43 / 3.12 | 0.29 / 1.84 / 5.47 | 21.4 / 82.4 |
| Wrist pitch | 0.15 / 1.05 / 2.77 | 13.7 / 43.8 | 0.18 / 1.00 / 2.36 | 0.41 / 2.07 / 5.51 | 25.9 / 74.4 |
| Roll | 0.04 / 0.38 / 3.99 | 5.9 / 47.3 | 0.08 / 0.72 / 5.42 | 0.07 / 0.68 / 2.19 | 7.5 / 28.2 |

The command moves (2+ units on a joint per frame) 70% of frames real, 86% free-play, 87% twin; reversals of 3 degrees or more
on joints 1–4: 242 a minute real, 343 free-play, 345 twin; the tool point's speed p50 / p90 / p99: 0.10 / 0.40 / 0.87 m/s real,
0.19 / 0.61 / 1.12 free-play, 0.14 / 0.47 / 1.11 twin. The scripts are too fast in the elbow and the wrist pitch (speed about
1.5 times at p90 and p99, acceleration 1.6–1.9 times), a little in the shoulder (1.2 times), slower than people in the yaw
(0.8 times), and inside the real range at the tool point.

**Caps for the `mixed` script (M)**, from the 9 real human sessions:

- **Joint speed**, the real p99: yaw 3.8, shoulder 2.6, elbow 3.6, wrist pitch 2.8, roll 4.0 rad/s.
- **Joint acceleration**, the real p99: yaw 40, shoulder 32, elbow 51, wrist pitch 44, roll 47 rad/s².
- **Targets a batch should land on** (check with `arm_response.py --only motion,kinematics`): each joint's speed p90 1.0–1.3
  rad/s; per frame, the largest arm-joint step about 0.03 / 0.14 / 0.19 / 0.34 rad and the wrist pivot's step 3.7 / 14.9 / 19.7 /
  35 mm (p50 / p90 / p95 / p99); moving 70–86% of the time.
- The caps clip the top 1% of real motion. A script whose distribution is right under them (smaller and slower pitch changes
  per move) is better than one that leans on them.
- Expected effect: frames over 20 units fall to about 55–65%, where real teleop is, not to 3–6-frame reaches.

**Stops (items 32 and 17).** `stop-latency` (Teleop.Eval) with `just stop-latency` (the real arm, recorded as a session) and
`just isaac-stop-latency` (a running twin): torque on, home as `joint-drive` goes there, then 22 trials of one joint swinging
from home at 0.5, 1 or 2 rad/s (11, 23 or 46 degrees, the tool point up to 26 cm), its command stopping dead and holding 1.5 s.
Live figures per trial and per joint and speed, `--json`; the recorded session gives the same on the robot's clock
(`arm_response.py --only stops`). 15 tests (the plan inside the envelope and free, the dead stop, the meter on a delayed and on
an overshooting arm, the refusals). **Not run on the robot or on a twin.**

From recorded data meanwhile: moments the relay target stops after moving at least 60 units/s and holds 0.6 s or more. Stop
time (until the reading stays within 2 units of where it ends) and the tool point's travel after the stop, p50:

| Tool speed at the stop | Real, 9 human (104) | Real, joint-drive waypoints (148) | Twin, waypoints of 2026-11-14 (229) |
|---|---|---|---|
| under 0.1 m/s | 190 ms, 1.7 cm | 178 ms, 2.3 cm | 193 ms, 2.0 cm |
| 0.1–0.2 m/s | 271 ms, 2.7 cm | 193 ms, 4.0 cm | 223 ms, 4.1 cm |
| 0.2–0.4 m/s | 294 ms, 5.6 cm | 261 ms, 7.2 cm | 239 ms, 7.9 cm |
| 0.4 m/s and more | 375 ms, 10.1 cm | 337 ms, 15.9 cm | 254 ms, 13.5 cm |
| Travel over speed, p10 / p50 / p90 | 0.10 / 0.19 / 0.32 s | 0.21 / 0.26 / 0.36 s | 0.19 / 0.25 / 0.33 s |

Past the held command: p50 0 degrees, p90 0.4–1.9, real and twin. **Provisional for item 17:** after its command stops, the
tool goes on for about 0.25 s of its speed (p90 0.35 s) and stops within 0.2–0.4 s (p90 0.6 s), twin and real alike: the arm
finishes the distance it trailed by, and barely overshoots. Measured from RobotHost's last relay target to the readings'
arrival, so it includes the read pipeline (about 20 ms) and leaves out the operator's uplink. The real run of
`just stop-latency` replaces these numbers.

**Item 33, what the real servos report.** The HTD-35H bus servos answer, through the board's STM32 (`robot_controller_sdk.py`),
position (whole pulses), input voltage (mV), temperature (°C), torque state, offset and limits. There is no current or load
reading in the board's protocol. The driver reads only position (every ~21 ms, motors 1–5 and 10). `getServoTemp`, `getServoVolt`
and `getServoTorque` exist but nothing calls them, and temperature and voltage change over seconds, too slowly for a contact.
So the real contact signal is position error and the reading's rate. The 0.2 s lag dominates that error (median 15 units at
50–100 units/s), so a detector has to expect it: a blocked joint shows as a growing error with a still reading, which is the
stall rule `DriveWatchdog` already uses (0.25 rad for 1 s). A load signal would need other servos or board firmware.

**Not done or untested:** the stop tool against a robot or a twin; `isaac-stop-latency` runs over the VPN and takes the
measured round trip out, but not the read pipeline; the caps are M's to apply; the twin's physics truth (`VQ`) is not used,
only the readings, to compare with the real arm on the same terms.

### A, appearance

(pending)
