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
   - **The table is its own kind, and since 2026-11-23 it is unsafe.** The world-models side's review, after the twin check:
     on the real robot a hard table hit is a collision too. So `safety_ranges.json` counts an arm link touching an obstacle or
     the table. Table contacts stay labelled as their own kind (`in_contact_table` per frame, kind `table` in the contacts
     sidecar), so obstacle-only labels can be derived. `--table-safe` restores the first rule: unsafe = the arm touches an
     obstacle. It was the default until then, because counting the table rewards "the path goes low", the shortcut this ADR
     exists to remove; that trade-off still holds.

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
   - **Unavoidable-contact ranges** (item 17), from the arm's stopping time and distance, go in a separate file, for analysis
     only. Since 2026-11-23 they are on by default, for the twin's measured stop times: fast 0.17 s, median 0.245 s, slow
     0.31 s (`just isaac-stop-latency` on twin 0). They stay provisional until the real arm's stop latency is measured
     (workstream R gives the tool; the run is the user's). The main label stays contact.
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
- `WC` object 0 is the table (C). L labels it `table`, and since 2026-11-23 counts it as unsafe by default (decision 4).
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

**Two new scenes, the bench untouched.** `core/Scenes/jetrover-mixed.json` (29 objects) and its held-out family
`jetrover-mixed-holdout.json` (26), each with its shapes file (`shape_survey.py`: 15 and 16 assets; every asset the bench also has
came out byte for byte as in its file). `jetrover-bench.json` and its headset copy are unchanged, so the headset needs no rebake.
The twin runs one with `just isaac-jetrover-host jetrover-mixed`: the recipe, RobotHost, the bridge and the renderer take any scene
name and needed no change. joint-drive must be given the same file (M's batch recipe names `jetrover-bench`).

**A mixed pool**, opt-in: a pool whose `obstacleMax` or `distractorMax` is above 0. Every other scene draws exactly as before.
- **Fields.** Per object: `role` (`obstacle`, `distractor`, `stand`, `plank`, `support`), a plank's `supports` (one or two ids),
  and `steps` (`[min, max]` resize steps a layout draws a size from). In the pool: `obstacleMin`/`obstacleMax`, `reachBearing`,
  `standShare`, `rigShare`, `distractorMin`/`distractorMax`, `distractorRegions` and `distractorReach`, with the existing
  `reachMin`/`reachMax`. Both readers refuse the same 49 cases with the same keyword (`role`, `steps`, `pool`, `fits`); 4 more
  are accepted. A mixed pool has no movable, fixed or clutter counts, clusters, edge objects, fixed regions or pool sizes.
- **The draw** (`SceneLayout.DrawMixed`, one seeded generator, no allocation):
  - 0-3 obstacles, uniform. Each is a plank on its supports (`rigShare` 0.2, when one is free) or an obstacle object.
  - A movable obstacle is raised on a free stand that holds it (`standShare` 0.3).
  - Each is centred uniformly by area over the band (r is the square root of a uniform draw between `reachMin`² and
    `reachMax`², bearing uniform within `reachBearing`), at any yaw, where it fits and keeps clear (64 tries).
  - Then 2-6 distractors, each in its regions.
- **No new shape kinds.** A stand is a box with the obstacle on its top (aligned or turned a quarter, off its middle by up to the
  room its top leaves). An overhang is a box plank on box supports under its ends: two for a bridge, one for a cantilever, the
  same step for both legs, and a clear span of at least 10 cm under it.
- **The rules, in both readers and in Isaac's model.** In a mixed pool every footprint is its rectangle. Two objects one above the
  other never overlap (heights within 1 mm count as touching). RobotHost's own placements may stand an object above the table, a
  movable one only on a stand that holds it (its bottom within 2 mm of the stand's top, its centre over it); the headset's never.
  `WorldHost` places what stands on the table before what is raised on it. A refused part of a stand or of a rig parks its whole
  group; a lone object is redrawn first, as in any pool.
- **Sizes** are uniform resize steps per object (1.25^k on every axis), boxes and cylinders, fixed ones too: Isaac builds a
  kinematic body for each of its steps. They are in `WZ`, so the layout is the scene file, `WL`, `WZ` and the first `WO`. No
  per-axis dimensions, and no new record (trainer/contract/SESSION_FORMAT.md, "A mixed pool's layouts").

**The reachable workspace, measured** (`robot/isaac/reach_envelope.py`).
- **The sweep:** the URDF over a joint grid of 9.58 M poses. 8.68 M keep Core's floor rule; self-collision is not checked, so the
  figures err on the far side.
- **The reach:** the arm's colliders come 0.364 m from the base's axis at 1 cm above the table, 0.406 m at 10 cm, and 0.427 m at
  any height. The base turns 120° either way.
- **Obstacles** are centred 0.13-0.34 m out, within ±2.0944 rad. The robot's keep-out and the table's edges cut where they fit.
- **Distractors** keep their footprint 0.45 m from the axis, in four rectangles that at least one third-person camera sees: beside
  the robot on the bench and the table, |y| 0.47-0.72 m, and the table's far corners, x 0.28-0.52 m and |y| 0.28-0.47 m.
- **The bench's edge objects,** by the same measure, stand within the arm's reach: the planner could not reach them, but an
  arbitrary pose could.

**What stands in reach** (the train family). The arm's base frame is 19.6 cm above the table.
- **Scans:** 7 YCB objects and 2 Poly Haven props at their real size.
- **Small things, 2.4-4.6 cm:** the tape, a tin and a parcel scaled to a third and a tenth, and a wooden cube in three sizes.
- **Tall and thin:** a steel rod fixed to the table (14-34 cm tall, 1.5-3.8 cm across) and a free wooden dowel (16-31 cm).
- **Stands:** a block 3.8-11.7 cm tall and a crate 4-7.8 cm.
- **Overhangs:** a shelf on legs 4.1-19.5 cm up, and a plank cantilevered off a block 5.1-15.6 cm up.
- **Distractors:** pliers, screwdrivers, a phone, a pen, a parcel, a bowl and the kettle, and the bench's flat clutter.
- **Surfaces:** every simple shape wears looks by name (`materials.objects`, drawn per take, recorded as `LK`).

**The held-out family** shares nothing with the train family's objects:
- **Assets:**
  - train: YCB 005, 007, 009, 010, 025, 040 and 061, and Poly Haven's cleaner tin, elephant, tape, cardboard box, pliers,
    screwdrivers, wooden bowl and kettle;
  - held out: YCB 002, 003, 004, 006, 008, 011, 019, 021, 024, 035, 036, 037, 051 and 052, and Poly Haven's mousetrap and gnome.
- **Kinds:** a square post 36-45 cm tall, a slat on its edge, a rubber puck 2.8-4.4 cm.
- **Heights:** stands 12-15 cm (train: up to 11.7) and a 3.5-5.5 cm riser; overhangs 2.6-4 cm and 19-24 cm up (train: 4.1-19.5).
- **Textures:** the simple shapes wear 10 textures the train family's never do.
- **Ids and names:** none in common.

**Measured over 2000 layouts** (`SceneMixedTests`, seeds 1-2000):

| | `jetrover-mixed` | `jetrover-mixed-holdout` |
|---|---|---|
| obstacles a layout (layouts with 0 / 1 / 2 / 3) | 1.48 (522 / 482 / 511 / 485) | 1.47 (522 / 486 / 525 / 467) |
| distractors a layout (layouts with 2-6) | 3.80 (458 / 425 / 409 / 453 / 252) | 3.78 (430 / 413 / 497 / 494 / 166) |
| objects that found no spot, a layout | 0.117 | 0.214 |
| obstacles raised on a stand / planks | 0.215 / 0.195 | 0.193 / 0.210 |
| obstacle centres inside the band's middle radius (the area there) | 0.157 (0.186) | 0.101 (0.123) |
| obstacles more than 60° from straight ahead | 0.408 | 0.395 |
| obstacle tops above the table, 5 / 50 / 95 % | 2.5 / 9.3 / 25.0 cm | 1.8 / 6.5 / 36.0 cm |

- **Shortfalls.** Large distractors account for most of the misses: a 32 cm kettle at any yaw fits only by the table's side edge.
  With 2-8 distractors and two large ones, 8 fitted in 16 layouts of 2000, so the range is 2-6 with one large one per family.
- **Spread.** At the same number of objects (1000 earlier, 200 new), a new layout's obstacles stand a median 5.7 mm (90%:
  12.3 mm) from the nearest earlier one, against 3.3 mm (6.5 mm) for the bench's objects in reach: about three times the area.

**The bench draws as before.** `core/Scenes/testdata/layout-jetrover-bench.txt` holds the bench's first 25 pool layouts, written
by the commit before this work (f401b58, on a throwaway branch). The new code reproduces them to the last bit, and the rests
golden did not change.

**Tests.**
- C#: the shared vectors; the mixed scenes' fields, surfaces, shapes and the held-out family's split; 400 layouts of each (fit,
  clearance at heights, band and bearing, stands under raised objects, supports under planks); determinism and no allocation; the
  shares above; the frozen bench layouts; a mixed rules golden for Python; `WorldHost` over `StaticWorld` (60 layouts each, every
  placement applied, raised ones after what they rest on; a refused support parks its plank and the other support; a refused stand
  parks what it holds).
- Python: the vectors and the rules golden. Isaac's `WorldModel` replays the first 20 layouts of each scene as `WorldHost` sends
  them (`golden/scene-layouts-jetrover-mixed.json`, 1399 requests) and applies every one. It also checks a raised object on a stand
  only, the headset never raising, and a fixed object resized within its own steps.

**Not checked.**
- **On the twin, nothing yet.** Unchecked there: `OVERLAPS_ARM` with the arm at rest beside a plank or a tall rod; whether
  objects raised on stands and the free dowel come to rest within joint-drive's 20 s; the kinematic resize bodies' start-up cost.
- **Not mine to check:** the planner against planks and raised objects (M), textures on long thin boxes (A), and the clearance and
  labels (C, L).

**The second pass (2026-11-23, branch `work/v2-scene2`).** World-models reviewed the 16-episode pilot on `jetrover-mixed`. At contact
the tool point sat a median 14.5 cm below the base frame (5 cm above the table). Nothing tall was hit: no rod, shelf or plank, and the
steel rod was in reach once. The stand blocks took 18 of 41 contacts. So "how low the path goes" still predicted contact. M's part
(aims at every height, no aims at stands) is separate; this is the scene's. Where this differs from the description above, it
replaces it.
- **Eight pool fields in four groups, mixed pools only, all 0 by default.** With them all 0 a pool draws exactly what it drew
  before, and the bench is no mixed pool: `layout-jetrover-bench.txt` and the rests golden did not change. Both readers refuse the
  same 16 new cases with `pool` and accept 3 more (vectors.json).
  - `tallShare`: the chance that an obstacle which is not a plank is a tall thin one, when one is free. Tall means at least 15 cm at
    its file size (`WorldScene.TallHeight`, joint-drive's `TallObjectMeters`): the rod and the dowel, in the held-out family the post
    and the slat. One more draw.
  - `workMin`, `workMax`, `workBearing`: the working band, inside the obstacles' band. Tall obstacles and planks are centred there,
    uniform by area, and so are their redraws (WorldHost). It is 0.16-0.30 m from the base's axis and within 60 degrees: the pads
    reach 18-33 cm out (M), and inside 60 degrees an aim may come round the base from either side (MixedTake).
  - `standTopMin`, `standTopMax`: a stand holds an object only when its top's sides are each between these fractions of the object's
    narrower side (0.4 and 0.75), so the object overhangs it all round and is met first. The object stands upright and centred; a tall
    one is never raised. Its stand is drawn before its rest.
  - `rigUnderMin`, `rigUnderMax`: a rig's underside height is drawn uniform over this range, within the heights its supports' steps
    give, and the support step nearest it is taken. Before, uniform pairs of steps put half the rigs 5-10 cm up. The range is 4-20 cm
    in the train family and 2-25 cm in the held-out family, whose two rigs keep their own ranges (2.6-4 and 19-24 cm).
- **The scenes.**
  - `reachBearing` goes from 2.0944 to 1.658 (95 degrees). `SceneMixedMeasureTests.IkReach` measured joint-drive's IK
    (`ArmSolver` from home, 40 solves, the joint ranges less 15 degrees, clear by the model). It puts the wrist 90.0 degrees round
    at most from home, and nothing at 92.5, because `MaxApproach` is 90 degrees. Turned that way first, it reaches 105. The twin check's
    obstacles at 107-116 degrees were out of every aim.
  - In the train family, `stand-post` (82: 4×4×16 cm, steps -3..0, so 2-4 cm across and 8.2-16 cm tall) and `stand-pillar` (83:
    5×5×12 cm, -2..1, so 3.2-6.25 across and 7.7-15 tall) replace `stand-block` (75) and `stand-crate` (76).
  - In the held-out family, the pedestal becomes 4.5×4.5×17 cm (17-21 cm) and the riser 6×6×2.5 cm (2.5-3.9 cm), both outside the
    train family's 7.7-16. `tallShare` is 0.1 in both families.

Measured over 2000 layouts (`SceneMixedMeasureTests`, seeds 1-2000). "Before" is 6fb0c7b's draw. "The working band" is the one above
for both columns:

| | train before | train after | held-out before | held-out after |
|---|---|---|---|---|
| layouts with obstacles that have a tall obstacle or a rig in the working band (anywhere) | 0.163 (0.502) | 0.592 (0.592) | 0.179 (0.547) | 0.627 (0.627) |
| obstacles a layout (layouts with 0 / 1 / 2 / 3) | 1.48 (522/482/511/485) | 1.46 (522/492/533/453) | 1.47 (522/486/525/467) | 1.46 (522/488/538/452) |
| share of the obstacles: tall, rigs, raised | 0.106, 0.195, 0.215 | 0.178, 0.193, 0.162 | 0.130, 0.210, 0.193 | 0.202, 0.197, 0.138 |
| obstacles centred in the working band | 0.302 | 0.625 | 0.267 | 0.591 |
| obstacles by bearing, 15-degree bins from straight ahead | 588 493 417 254 259 361 342 245 | 782 645 531 339 246 282 92 0 | 621 529 405 221 249 381 356 175 | 762 705 529 315 212 287 110 0 |
| beyond 60 / beyond 95 degrees | 0.408 / 0.159 | 0.213 / 0 | 0.395 / 0.136 | 0.209 / 0 |
| obstacle tops above the table, 5 / 50 / 95 % | 2.5 / 9.3 / 25.0 cm | 2.5 / 12.0 / 27.5 | 1.8 / 6.5 / 36.0 | 1.8 / 8.6 / 45.0 |
| tall obstacles' and rigs' tops | 6.7 / 14.1 / 34.4 cm | 7.8 / 17.4 / 31.3 | 4.6 / 22.1 / 45.0 | 5.2 / 24.0 / 45.0 |
| raised objects' middles | 5.9 / 10.4 / 16.5 cm | 9.7 / 13.7 / 21.5 | 5.2 / 7.4 / 19.4 | 4.4 / 18.9 / 25.8 |
| stands' heights | 4.0 / 6.3 / 11.7 cm | 7.7 / 10.2 / 16.0 | 3.5 / 5.5 / 12.0 | 2.5 / 17.0 / 21.3 |
| a stand's top area over its object's footprint (narrower both ways) | 1.4 / 5.0 / 43 (none) | 0.1 / 0.3 / 0.5 (all) | 1.5 / 2.6 / 38 (none) | 0.1 / 0.2 / 0.4 (all) |
| rigs' undersides, 5 / 50 / 95 %; in 0-5, 5-10, 10-15, 15-20, 20+ cm | 4.1 / 8.0 / 19.5; 32 295 143 107 0 | 5.1 / 10.0 / 19.5; 10 184 228 142 0 | 2.6 / 19.0 / 23.8; 307 0 0 162 149 | 2.6 / 19.0 / 23.8; 271 0 0 148 157 |
| objects that found no spot, a layout | 0.117 | 0.127 | 0.214 | 0.201 |
| placements the arm at home would meet (1 cm), objects in layouts | 31 in 31 | 102 in 101 | 135 in 113 | 360 in 302 |

- **The choices.**
  - A `tallShare` of 0.25 put a tall obstacle or a rig in 0.71 of the layouts with obstacles, mostly the same two tall objects. At 0.1
    it is 0.59.
  - The first stands, 12 and 8 cm tall for their width, left the raised objects' middles where they were (median 10.6 cm). Taller
    ones (16 and 12 cm) raised it to 13.7.
  - The raised share falls from 0.215 to 0.162 because tall obstacles are never raised and more units are tall. `standShare` stays 0.3.
- **Planning, with M's planner as it was** (6fb0c7b; `joint-drive --script mixed --dry-run --layouts 1-200 --rows`, seed 1, speed
  3.8, 15 s; "before" is 6fb0c7b's scene file read by the new code, which draws as 6fb0c7b did):

  | | Before | After |
  |---|---|---|
  | Layouts planned | 196 of 200 | 198 of 200 |
  | Planned contacts | 131 in 80 takes | 147 in 94 takes |
  | On tall obstacles / rigs / stands / the rest | 17 / 32 / 29 / 53 | 29 / 42 / 24 / 52 |
  | Aims and passes at stands | 53 | 77 |
  | The arm's lowest point at a planned contact, p50 / p90 / max | 63 / 140 / 185 mm | 73 / 168 / 192 mm |

  More aims went at stands, yet fewer reached them: the object over the stand overhangs it. M3 changes the planner (aims at every
  height, none at stands), so these figures show only that the new layouts plan.
- **Home.** The last row counts placements that would meet the model's arm at home. Home holds the gripper level about 28 cm above
  the table, out to x 0.31. A take's tail ends with the wrist 0-10 cm above the base frame, and joint-drive asks for the next layout
  with the arm there. So tall objects in front make Isaac's `OVERLAPS_ARM` likelier. A lone object refused so is redrawn in its band,
  and a refused rig or stand parks its group. The held-out family's 36-45 cm post and its 17-21 cm pedestal make most of its count.
- **Tests:**
  - the vectors;
  - per layout over 400 layouts of each scene: tall obstacles and planks in the working band; raised objects upright, never tall,
    centred on a stand narrower than them within the fractions; obstacles within 95 degrees;
  - the stands' heights outside each other's ranges across the two families;
  - the shares above, pinned in `SceneMixedMeasureTests`;
  - the obstacles' bearing between the IK's 90 and 105 degrees;
  - the rules golden's pool row with the new fields, and each object's tall flag, read alike by `test_scene.py`;
  - Isaac's model applying every placement of the new first 20 layouts of each scene (`golden/scene-layouts-jetrover-mixed.json`,
    1391 requests).
- **Not checked on the twin:**
  - objects settling on stands narrower than them (a scan's hollow bottom or the elephant's legs may drop a few millimetres, or more);
  - how often `OVERLAPS_ARM` refuses a tall rod, a plank or a raised object;
  - the planner's figures with M3's changes.

### M, motion

**Built** (branch `work/v2-motion`; docs/adr/0038 decisions 1 and 2, items 1, 6, 7, 8, 10, 11, 23, 24 and 29's caps).

- **`joint-drive --script mixed`** (`DriveScript.Mixed`, `MixedTake`; scenario `mixed`). A take is a run of the hand model's moves
  from two sources, mixed in every take:
  - *layout-blind* parts (`HumanOperator.MixedFree`): moves to goals drawn anywhere in reach (`BlindReach`: the wrist 8-25 cm
    from the base axis, within 100 degrees of straight ahead; 40% low, the wrist at most 6 cm above the base frame with the
    gripper pitched -45 to -85 degrees, so the pads come within a few cm of the table; 80% of goals a lognormal distance from the
    hand, their parts out from the base and up scaled by 0.55, so the hand moves round the base as people do), back-and-forth
    strokes (`reverse`) and small adjustments, checked against the robot's own model only, so they may meet an object;
  - *object-directed* moves: an **aim** reaches at a point of an object's own shape (`shapes.json`; its box without one) at a
    drawn height (26 mm above its bottom, the pads clear of the 10 mm floor model, to 20 mm below its top or 20 cm up) and
    stops at a **signed offset drawn uniformly from -40 mm (into it, no deeper than the model measures that object) to +60 mm
    (short of it)**; a **pass** sweeps across an object at a drawn lateral offset (through a movable one, beside or grazing a
    fixture), so contact comes mid-move. A quarter of the aims start 4-20 mm from their object (`NearStartShare`), so a contact
    comes early in the move. Most aims and every pass follow an **arc round the base** at one radius (the arm's reach is a thin
    shell: pointing down the pads reach 18-24 cm out, 2.5-9 cm above the table; at -60 degrees 24-33 cm, 4-18 cm up), from
    either side, its height changing by a drawn slope (-12 to +8 degrees); a quarter of the aims come straight out from the base
    (within 25 degrees). The path is solved outward from where it meets the object, the wrist at the middle of the pitches that
    reach that point. The arm's clearance is measured as the old planner's is: `ShapeGap` (the twin's links, the pads where each
    tick's gripper holds them) against a shape, `GapLadder` against a box.
  - Every move draws its slowness from one lognormal (sd 0.30, 0.65-1.8); any move may be followed by a pause (30%, 0.15-0.7 s),
    an object-directed one by a pull back (`reverse`, 35%). The gripper does what it does in free movement; no opening for an
    approach. An object planned to be pushed into is aimed at no more in the take.
  - **Timing:** the arm moves from the first tick (`EngageOnTheMove`); every tick before 1.5 s and in the take's last 1.0 s keeps
    2 cm from every object's box (`HumanOperator.Guard`, the model's margin on top), so no contact is planned there; the tail
    begins with a move up and away that ends clear, and the take ends partway through a later move, at least 1.0 s after the
    last object-directed move ended. Nothing is checked against a category.
  - **Speed caps** (`MotionCaps`, `MixedOptions.DefaultCaps`): workstream R's real p99 per joint, speed and acceleration: yaw
    3.8 / 40, shoulder 2.6 / 32, elbow 3.6 / 51, wrist pitch 2.8 / 44, roll 4.0 / 47 (rad/s, rad/s²). Each is measured over
    three ticks (62 ms; the acceleration as the second difference over three ticks each side), about the 25 ms smoothing R
    measured people with. Checked tick to tick on the raw 48 Hz command instead, the hand's jitter met them on most fast moves,
    and refusing whole parts left only slow ones: every measure fell to about half of people's (per-frame p90 0.088 rad over
    200 layouts, below). So a part the caps refuse is drawn again with the same draws, a third slower each time up to 2.5
    times (`HumanOperator.Stretch`): the caps stretch fast moves instead of trading them for slow ones. With R's advice not to
    lean on them, a **hand style** (`HandStyle`, `MixedOptions.DefaultStyle`) brings the elbow and wrist pitch down toward
    people's: each move's pitch change 0.5 of the hand model's and its roll change 0.55, the pitch followed over the whole of a
    submovement (the hand model: 0.7 of it), every submovement 0.9 times as long, near goals round the base (above), and pauses
    after 30% of moves. Chosen from 12 settings over 40 layouts each (`--style`, `--blind`, `--pause` and the cap options set
    them; R's next measurement can change them without a build). Off (`--no-caps --no-style`) the script is the hand model's
    own pace.
  - The four old scripts are unchanged: every new hand-model option is off for them (`TouchMissesTests`, `SafetyScriptTests`
    and `SafetyTakeTests` as before).
- **Records** on the joint port, written by RobotHost once per sequence while a take records, documented field by field in
  trainer/contract/SESSION_FORMAT.md: `MV` (tag 0x8D, `ScriptMoveCodec`) for every planned move as it begins
  (`index count kind object offsetMm gapMm startS contactS closestS endS`), and `RP` (tag 0x8E, `ReplayCodec`) once a take of
  the script starts (`group role edit`, exactly decision 2; `group` is the take's `--seed`).
- **Replays** (`joint-drive ... --replays N`, `--simulated-robot` only). After the source take: reset-layout, checked against
  where every object stood at the source's start (1 cm, 5 degrees; three tries: a knocked object on another's spot blocks it
  once); motion on and a slow setup move back to where the source began, clear of the objects by 1 cm; one edit, drawn
  (`ReplayEdits`, deterministic from the group, the replay and the try): the replays of a group take remove, shift and move in
  an order drawn from the group, at the object the take came closest to (its aims' and passes' planned gap, else the pads'
  nearest pass); a removal prefers an object the source moved (it touched it) and parks it (**new world op 6, `park`**: WorldHost
  stands it on the shelf as a layout parks one; reset-layout brings it back; the headset never sends it); a shift moves an
  operator-movable, table-level object 10-40 mm along the approach of that closest move, toward the arm when the move stopped
  short and away when it went in (three times in four); a move stands one elsewhere in reach (15-30 cm out, within 60 degrees,
  in the placement region) at a drawn yaw on its rest. Nothing that holds another object is edited, and a raised one is only
  removed. An edit whose new box the replay's quiet start or end would meet, or that RobotHost refuses, is drawn again (six
  tries, then removal). Then the objects at rest (`RestWatch`), motion on, and a take that sends the source's commands at the
  same times from its start (`TapePlayer`: in order, none early; the clock held while the joint-state stream is unhealthy, as the
  source's was) with `RP replay` and the source's `MV` records. joint-drive prints `take recorded: ... ended <code>` for every
  take and `keep-look: on` / `off` (on as the source records, off as the last replay records).
- **Batch:** `just isaac-safety-batch ... <twin> <replays=0> <scene=jetrover-bench>` and `isaac-safety-batch-twins ... <replays>
  <scene>`. Categories may name `mixed`; a mixed take's replays count as takes; every take a run recorded is waited for and
  checked with its own exit code (`safety-check --set-aside --drive-exit <code>`, workstream L); the verdict's `outcome` and
  `tableContact` are printed. While a group records, the twin's sessions root holds `.keep-look` (made on `keep-look: on`,
  removed on `off` and after the run; workstream A's renderer keeps a look while it stands as a session closes), so replays keep
  the source's look. The scene is passed through (workstream S: mixed batches run on `jetrover-mixed`).
- **Dry run:** `joint-drive --script <safety script> --dry-run --scene <file> --layouts a-b [--rows]` plans on RobotHost's
  layouts and prints the figures below (`MixedDryRun`).

**Measured** offline (`joint-drive --script mixed --dry-run --scene Scenes/jetrover-bench.json --layouts 1-200`, seed 1, speed
3.8 rad/s, `--seconds 15`: RobotHost's layouts 1-200 of the bench, take seeds as a batch draws them, on the GPU machine).
"Before" is the hand model's own pace (`--no-caps --no-style`, at e2234df); "after" the defaults (at 847dd4f). People: the
human headset sessions on the real JetRover (R's 9: 0.031 / 0.140 / 0.194 / 0.341 rad and 3.7 / 14.9 / 19.7 / 35.1 mm; mine,
12 sessions, 21k frames: 0.029 / 0.139 / 0.193 / 0.348 rad and 3.4 / 14.7 / 19.4 / 35.7 mm).

| | Before | After | People |
|---|---|---|---|
| Layouts planned | 199 of 200 | 194 of 200 | |
| Plan time, p50 / p99 | 0.82 / 1.60 s | 0.85 / 1.68 s | |
| Take length, p50 / p90 / max | 17.1 / 18.4 / 20.4 s | 17.6 / 19.4 / 24.2 s | |
| Moves a take: free, aim, pass, pause, reverse | 13.6, 2.8, 0.4, 1.4, 4.3 | 12.3, 2.0, 0.3, 3.1, 3.1 | |
| Takes with a planned contact | 166 of 199 (275 contacts) | 142 of 194 (202 contacts) | |
| Largest arm-joint step a frame (15 fps), p50 / p90 / p95 / p99 | 0.052 / 0.188 / 0.260 / 0.447 rad | 0.036 / 0.112 / 0.138 / 0.185 rad | 0.03 / 0.14 / 0.19 / 0.34 |
| Wrist step a frame, p50 / p90 / p95 / p99 | 5.8 / 20.5 / 28.5 / 49.7 mm | 4.4 / 14.8 / 19.2 / 28.5 mm | 3.7 / 14.9 / 19.7 / 35 |
| The command moving (a joint 2 servo units a frame) | 93% of frames | 87% | 70-86% |
| Speed p90 (25 ms smoothed): yaw, shoulder, elbow, wrist pitch, roll | 1.14, 1.22, 1.73, 2.06, 0.68 rad/s | 0.84, 0.85, 1.12, 1.16, 0.28 | 1.27, 1.02, 1.29, 1.05, 0.38 |
| Speed p99, same joints (caps 3.8, 2.6, 3.6, 2.8, 4.0) | 3.26, 3.22, 5.00, 5.25, 2.34 | 2.01, 1.81, 2.37, 2.21, 0.91 | the caps |
| Acceleration p99, same joints (caps 40, 32, 51, 44, 47) | 30.0, 44.8, 75.9, 80.6, 33.0 rad/s² | 18.6, 18.9, 28.2, 29.6, 9.3 | the caps |

With the defaults (after):
- **Offsets,** aims, in 10 mm bins from -40 to +60 mm: 27 33 33 41 38 45 52 39 39 45 (392 aims, median 16 mm): one continuous
  range across the surface, every bin filled (fewer in the deepest, which some small objects cannot be measured into). Passes'
  closest points on their line, from below -40 mm: 10, then 5 16 8 14 5 2 4 (64). The closest approach over each object move
  as planned (`gapMm`, the hand's error included), from below -40 mm: 17, then 30 46 47 62 40 47 42 41 84.
- **Contacts** (planned, by the model): the first 5.8 s into a take at the median (p90 12.2 s, none before 1.5 s); by thirds of
  the take 74 / 91 / 37, and of their move 77 / 92 / 27 (a pass's mid-move, an aim started close), not only at a move's end.
- **Heights** (the arm's lowest point at a planned contact, above the table): p50 56 mm, p90 135, max 192; in 20 mm bins from
  0: 13 52 47 30 20 12 10 9 5 4. The arm cannot meet an object lower than 26 mm above its bottom from the side.
- **Approach directions:** 449 of 456 object moves sweep round the base from either side (207 and 242), 7 come straight out
  from it. Straight-out approaches seldom fit the thin shell the arm reaches: most that are drawn find no path.
- **The motion:** the wrist's step a frame sits on people's at p50-p95, every joint well under its cap; the largest joint step
  stays under people's at p99 (their tails include the headset IK's jumps, which the hand model does not make), the yaw and
  shoulder p90s about 0.7-0.85 of people's, the elbow and wrist pitch within 10%.
- **Not planned:** 6 of 200 layouts, each when a move or the tail's exit found no part the caps and the quiet margin let
  through; a batch draws another layout.

**Checked:**
- `MixedTakeTests`: deterministic plans, and moves numbered in the order they begin. On 24 random layouts: the timing rules
  (the arm moving within 0.3 s; nothing within 1 cm of an object's box before 1.5 s or in the last 1.0 s; every planned
  contact at least 1.0 s before the end), offsets on both sides of the surface with no gap over 25 mm, contacts at more than
  one height, some early in their move. The caps hold on every tick as they are measured (speed and acceleration over three
  ticks; a single tick at most 2.5 times a joint's speed cap, the commands' own jitter); the options are checked.
- `ReplaysTests`: a replay sends the same targets, bit for bit and in order, none early; the edits are deterministic, take
  their kinds in turn and keep to their rules; a removal takes the object the source moved; an edit in the quiet start's way is
  seen; a planned move's record carries its times from when it is sent.
- `ScriptMoveCodecTests` (the `MV` and `RP` wire), `AMixedTakesMovesAndReplayGroup_AreTheirSessionFormatLines`,
  `WorldHostParkTests` (op 6 parks any object, fixtures included, and reset-layout brings it back; refused while motion is on).
- The four old scripts' tests unchanged: `SafetyScriptTests` (the random-layout bars as they were), `SafetyTakeTests`,
  `TouchMissesTests`.

**Untested:**
- Nothing has run on a twin: the records as RobotHost writes them in a take; the replays' world edits, restores and setup
  moves; the keep-look timing against A's renderer; the batch's handling of several takes a run.
- The scene `jetrover-mixed` (S's reader is not on this branch); its distractors are targets until the integration line makes
  them scenery (`WorldView.TryObjects`).
- The real arm: `--replays` needs `--simulated-robot`, and the caps were set from R's human figures, not from the arm.

**After the twin check of 2026-11-23** (branch `work/v2-motion-fix`). Four fixes:
- **The start pose.** A mixed take needs only the quiet margin (2 cm) at its start, not the old scripts' 5 cm. When neither the
  arm's pose nor home has it, the take starts from a pose turned or raised from home, reached by a slow setup move clear of the
  objects by 1 cm. With `--new-layout`, a layout with no plan is replaced, up to 4 layouts a take. A hand cornered mid-take moves
  back toward the middle of reach before the take gives up.
- **Replays never repeat an edit.** Each kind takes the first object the group has not edited that way. A removal tries first
  what the take moved; a shift goes only along a move the take made at the object; a move uses a spot from the scene's own rules
  (`DrawSpot`, `FitsOnTable`, `InReach`, `Overlaps`). When no kind has anything new, the replay is skipped and logged.
- **Blind moves stop at fixtures.** From 0.3 s into a layout-blind move, it keeps to a fixed object's box, a drawn 5 mm into it to
  15 mm short (the check had a shelf rig touched for 99 of 262 frames). A move into a fixture always pulls back out. Aims and
  passes at fixtures, and every movable object, are as before.
- **Beside the robot.** The IK reaches no further round than about 95 degrees (nothing at 100), so beyond 60 degrees an arc
  comes from the front, and beyond 80 degrees a pass is an aim.

On `jetrover-mixed`, layouts 1-200 (dry run as above):

| | Before (5559eb2) | After (521603d) |
|---|---|---|
| Layouts planned | 176 of 200 | 195 of 200 |
| Refused for the start pose | 11 | 0 |
| Aims / passes a take | 1.1 / 0.2 | 1.4 / 0.2 |
| Planned contacts | 86 in 57 takes (0.49 a take) | 112 in 80 takes (0.57 a take) |

The 5 layouts left unplanned are 4 cornered hands and 1 tail. On the twin, `--new-layout` draws another layout for those.

**Aims move as free moves do, and an `add` replay (2026-11-23, branch `work/v2-motion-add`).** Two changes the world-models side
asked for after the twin check. Where they differ, they replace the description above.

- **The motion, measured first.** The dry run writes one row per planned move (`--features <csv>`, `MoveFeatures`), and
  `scripts/gpu/move_features.py` scores it. The features are measured on the pads' midpoint, smoothed 25 ms:
  - duration, path length, peak and mean speed, peak acceleration;
  - the last 30%'s mean speed against the peak, and the time to stop;
  - a pause or a pull back after the move;
  - the lowest height.

  Before the change, a logistic regression on these told aims from free moves at AUC 0.93, and a decision stump at 0.67 (both
  5-fold cross-validated by take). With roll, pitch and gripper ranges the regression reached 0.97. Compared with free moves, aims
  were:
  - shorter (path p50 8.4 cm against 11.8);
  - lower (69 mm against 179);
  - nearly fixed in roll and pitch (0.2 and 1.5 degrees against 4.4 and 14);
  - pulled back after far more often (47% against 7%).
- **Aims and passes.** Each is now made by the free moves' own constructor, `MixedTo`, as a free move to a goal is:
  - the same endpoint error, 10% undershoot and corrections to 4 mm;
  - curved through a via point or split into two pushes as often;
  - roll and gripper moving as they go.

  Before, they followed their line in 4 cm segments to a 2.5 mm stop. Other rules:
  - **Passes** go straight across the object, not round the arc. A pass's offset is its line's closest approach, and `gapMm` gives
    the closest it really came.
  - **The start.** Each starts as far back as a free move's goal is from the hand: the same lognormal, with 10% at a small
    adjustment's distance.
  - **The wrist.** The hand reaches the start at a pitch drawn as a low blind goal's is, with the pads where the line starts. It turns
    the wrist on the way, at most 300 degrees a metre of approach: larger turns broke the caps, which slowed those moves down.
- **One law after every move**, whatever its kind: a pull back with probability 0.15, else a pause with 0.35. A move that ends in a
  fixture, or within 5 mm of one, always pulls back out, and so now does a blind move. That pull back goes at least as deep as the arm
  went in, plus 1.5 cm, and the pause is still drawn after it.
- **Layout-blind goals are all low.** The wrist stays at most 5 cm above the base frame and the pads end 2-14 cm above the table.
  - This holds in the quiet start and tail too, falling back to goals that are not low when the guard refuses.
  - A low goal stays 3 cm inside the reach: a stretched arm by the table broke the caps whichever way it moved.
  - A blind part keeps its drawn kind for 20 of its 30 tries. Small adjustments, which seldom fail, had been 46% of the parts that
    are not strokes, not 27%.
  - A cornered hand first lifts straight up, ignoring the walls: a hand cornered beside a fixture was often inside its wall already.
    A failed exit lifts and tries again.
  - Object share 0.85 and pass share 0.45 keep the contacts a take.
- **Replays: `add`** (RP `add:<id>:<x>:<y>:<yawRad>`). It gives explicit safe-to-unsafe pairs; the other edits mostly turn unsafe
  takes safe. A movable obstacle is stood with its footprint centred under the arm's lowest point, at a moment of the source take
  when that point passed lower over the table than the object is tall less 15 mm. The moment comes from:
  - the measured joints when joint-drive has them, else the commanded ones;
  - neither the first 1.5 s nor the last 1.0 s (with 0.25 s more at each end);
  - a draw among the moments that work, not the lowest, so contacts come at every height the take passes.

  The spot must satisfy the scene's rules (`FitsOnTable`, `InReach`, the reach bearing, `Overlaps`) and stay clear of the quiet start
  and end, both as planned and as the arm went. The object is, in order of preference:
  - one of the layout's objects on the table that the take neither touched nor came within 1 cm of, holding nothing and not raised,
    moved there with op `move`;
  - else one the layout parked, stood there upright with a **new world op 7, `unpark`**. RobotHost places it as a reset does (the
    placement rules decide), lists it in `WL` while it stands there, and parks it again on reset-layout. The headset never sends it.

  The existing rules hold: objects holding another are never edited, a group never repeats an edit, and a replay with nothing new is
  skipped. In a group with two replays, the add is the first or the second (drawn from the group), and the other is remove, shift or
  move as before. `sidecars.parse_edit` reads `add` (it raised before, and the episode lost its group).

On `jetrover-mixed`, layouts 1-200, with the dry run as above. "Before" is 0457599's planner with the same feature definitions; "after"
is 8320e68. The main part runs from 1.5 s to the last 2.5 s, where aims happen; in the quiet start and the tail, a move's time alone
says it is free.

| | Before | After |
|---|---|---|
| Layouts planned | 195 of 200 | 196 of 200 |
| Aims / passes a take | 1.4 / 0.2 | 1.2 / 0.2 |
| Planned contacts | 112 in 80 takes (0.57 a take) | 131 in 80 takes (0.67 a take) |
| Aim against free, logistic AUC, whole take / main part | 0.932 / 0.896 | 0.689 / 0.629 |
| The same without `pullback_after` (fixtures' forced pull back) | 0.904 / 0.850 | 0.612 / 0.527 |
| The same with roll, pitch, gripper, speed peaks, chord, any reverse after | 0.973 / 0.961 | 0.793 / 0.727 |
| Decision stump AUC, whole / main | 0.673 / 0.678 | 0.585 / 0.582 |
| Pass against free, logistic AUC, whole / main | 0.922 / 0.908 | 0.796 / 0.800 |
| Strongest single features (AUC, whole take) | low 0.80, duration 0.74, pull back 0.70, path 0.68 | low 0.62, pull back 0.61, all else 0.55 or less |
| Aim / free p50: duration, path | 0.46 / 0.78 s, 8.4 / 11.8 cm | 0.87 / 0.82 s, 14.0 / 13.4 cm |
| Aim / free: lowest point p50, pull back after, pause after | 69 / 179 mm, 47% / 7%, 15% / 18% | 67 / 88 mm, 37% / 15%, 26% / 28% |
| Aim / free p50: roll range, pitch range | 0.2 / 4.4, 1.5 / 14.2 degrees | 4.5 / 4.4, 11.1 / 19.1 degrees |
| Largest arm-joint step a frame, p50 / p90 / p95 / p99 | 0.036 / 0.114 / 0.140 / 0.185 rad | 0.035 / 0.113 / 0.138 / 0.182 rad |
| The command moving | 88% of frames | 85% |
| Free and reverse moves into a movable object's box, by the model (pull backs out of one included) | 277 of 3191, in 78 takes | 267 of 2911, in 80 takes |

- **What is left.** The forced pull back out of fixtures fires after 0.39 aims and passes a take into one, 0.33 a take beyond the
  draw. In the main part, aims are followed by a pull back 37% of the time against 15% for free moves, which alone scores AUC 0.61.
  Without that feature, the main part separates at 0.53. The pitch range (0.67 in the main part) and a pass's length (0.83) are what
  the extras still see.
- **Replays**, two a group, drawn offline with nothing refused:
  - an add in 175 of 196 groups (89%): 16 move one of the layout's objects, 159 stand a parked one;
  - in all 175 the plan's arm meets the added box by the model: first at p50 5.0 s into the take, p90 10.2 s, max 14.2 s, never in
    the first 1.5 s;
  - the added object's top is p50 102 mm above the table, p90 200 mm;
  - the other replays: remove 148, shift 24, move 45.

  On `jetrover-bench`, whose placement region is the front rectangle, an add fits in 13 of 40 groups.
- **Checked:**
  - `MixedTakeTests`: blind moves pulled back from, aims turning the wrist, every aim into a fixture pulled back, blind goals low,
    the options;
  - `ReplaysTests`: an add's spot by the scene's rules, the arm meeting it after the quiet start, the add in the group's drawn replay
    with the other remove, shift or move, and the rules (touched, holding, raised, added already, nothing left);
  - `WorldHostParkTests`: unpark stands a parked object, `WL` lists it, reset-layout parks it again; refused for an object on the
    table, a fixture, or with motion on; op 7 crosses the wire, op 8 does not;
  - the RP and session-line tests; `test_sidecars`.
- **Not run on the twin.** Untested there: unpark against Isaac's placement rules; the measured track joint-drive gives the add; the
  physics contacts the low blind goals now make with movable objects; the motion's look.

**Contacts at every height, and replays that edit the path (2026-11-23, branch `work/v2-motion3`).** World-models' review of the pilot
dataset `jetrover_20261123_215338_c00c5a` (16 episodes, 41 contacts): at contact the tool point sat a median 14.5 cm below the base
frame (80% from -15.8 to -8.4 cm), no rod, shelf or plank was hit, the stands under raised objects took 18 contacts, and how low the path
goes scored AP 0.37 alone (chance 0.14); in one replay group a removal and a move left the take's one stand contact as it was. Where they
differ, these replace the description above.

- **Aims and passes** (`MixedTake`):
  - never at a stand or a plank's support: `SafetyObject.Role` carries the scene's role (`WorldView.Bounds`; the four old scripts do not
    read it);
  - at a height drawn uniformly over the object's own height (`HeightRange`): on the table from 26 mm above its bottom, raised on a stand
    from 10 mm above the stand's top, a plank from 3 cm under it (met from below) to its top; up to just below the top and to what the
    arm reaches (`ReachTop`, at most 29.6 cm above the table);
  - targets drawn in proportion to that height range to the power 1.5 (at least 3 cm): every object covers the low heights, only tall ones
    the high;
  - the wrist pitched for the height (`PitchAt`): the reach shell's pitch near the table (pointing down close in), toward level higher up
    (-35 degrees 10 cm above the table, -20 at 20 cm, -10 at 30 cm, blended from 5 to 15 cm up), so the pads lead into a tall object
    rather than the wrist above them. A plank's approach may rise up to 20 degrees, and a pass goes under a plank anywhere across it;
  - an approach to a raised object or a plank keeps 5 mm off what holds it up (the line from start to end, the move there and the move
    itself, by the model), or the aim is dropped;
  - a thin object's offsets short of it shrink with its depth (a plank: -8.5 to +15 mm, against -40 to +60 for a deep one), so it is
    touched as often as a thick one: before, 4 of 62 aims at planks touched them;
  - an aim's start pitch is a blind goal's for a height drawn from the blind bands (below), so aims turn the wrist on the way as free moves
    do.
- **Layout-blind goals** by the tool point's height (`BlindReach.ToolBands`, `--tool-bands`): base frame z -18 to -12 cm (60%), -12 to
  -7 (28%), -7 to +2 (12%), the table at -19.6, the wrist pitched by the same `PitchAt`. The quiet start the same, falling back to the
  bands without the low one; the tail after its exit without the low band, so the next take, which starts where this one ends, does not
  start by the table. Chosen from four settings so that where blind moves end follows where aims end: they still end about 2 cm higher at
  the median (low goals are refused more often by the floor and the caps).
- **Replays** (`ReplayEdits.Draw`): on the source's track (measured when joint-drive has it), the model sorts the objects
  (`PathProximity`, `ArmTouch`: the twin's links against each object's shape, else its box): touched or within 1 cm (or moved by the
  source), within 5 cm, farther.
  - The first are removed, shifted away until the track keeps 5 mm from them, or moved elsewhere 2 cm clear of it: a contact made a miss.
  - The second are shifted toward the path until the track goes 3 mm in, or moved onto it, under the arm's lowest point as an add stands one:
    a miss made a contact.
  - Nothing farther is edited: a replay with nothing else is skipped, and its log line lists the touched, near and far objects.
  - The add comes at its drawn replay, or at the first when that one has nothing near the path (before, such a group lost its add
    altogether: replay 1 found nothing and the rest were skipped), or at the second when the first found no spot; one add a group. Why an
    add found no spot is logged (no moment, no object, or the spots' tally: too high, beyond the bearing, the scene's rules, the quiet start
    and end). joint-drive prints the source's nearness to every object before its replays.
  - `Holds` takes a tilted object's box reaching below what it rests on: its bottom within 1.5 cm above or 3 cm below the stand's top. On the
    twin check an elephant lying on a stand read 1.5 cm low, the stand was taken for free and removed, and the layout could not be put back
    for the group's second replay.
  - Every replayed command's capture stamp is newer than the last (`TapePlayer.Stamp`). Behind its clock the replay sent the commands due in
    one pass with one stamp, and RobotHost dropped all but the first as stale: 4-19 commands in each of the 9 replays of the pilot, none in
    their sources.
- **The dry run** makes a mixed pool's distractors scenery, as `WorldView.TryObjects` does on the twin, and prints where contacts fall
  (planned, and by the model over every move with the move that made each), where moves end, aims by object kind, each replay edit's expected
  change and why groups lack an add. `--chain` plans each layout from where the last plan left the arm, as the twin's takes start; `--lag s`
  has the add look at the commanded way s late, as at a measured track.

On `jetrover-mixed`, layouts 1-200, the dry run as above. "Before" is 6fb0c7b's planner and edits, measured by b3f4403's dry run (which
only adds the figures, and makes distractors scenery as on the twin: the add share it reports is lower than the 89% above, which counted
moves of distractors and which the bug below hid less); "after" is 30cd829, and the edits 849f3cb's (the same plans; a move onto the path
must now go at least 5 cm, which turned one of them into a removal). Seeds 1, 2 and 3 where three figures are given.

| | Before | After |
|---|---|---|
| Layouts planned | 196, 198, 198 of 200 | 199, 198, 197 of 200 |
| Aims / passes a take | 1.2 / 0.2 | 0.9 / 0.1 |
| Planned contacts (aims and passes, by the model) | 131 in 79 takes | 94 in 76 takes |
| Their tool point, cm from the base frame, p10 / p50 / p90 | -15.5 / -11.7 / -3.0 | -15.6 / -10.6 / -0.6 |
| Their tool point above -8 cm | 23%, 30%, 25% | 34%, 34%, 40% |
| Planned contacts by kind | support 25%, other 23%, stand 22%, rod 20%, raised 10% | other 38%, rod 37%, raised 14%, plank 11% |
| Contacts by the model over every move (onsets) | 370 in 97 takes | 317 in 103 takes |
| Their tool point, p10 / p50 / p90; above -8 cm | -15.5 / -10.5 / -3.2; 27% | -15.2 / -10.3 / -1.1; 33% (39%, 38%) |
| By kind | other 24%, plank 21%, rod 19%, support 15%, raised 11%, stand 10% | other 43%, rod 24%, raised 17%, plank 11%, support 3%, stand 2% |
| By move | aimed 31%, staging 25%, blind 20%, aimed elsewhere 14% | blind 44%, aimed 27%, staging 13% |
| Where moves end, tool point p50: aims / blind goals | -10.8 / -10.3 cm | -11.3 / -7.9 cm |
| Aim against free, main part, logistic AUC | 0.632, 0.626, 0.682 | 0.632, 0.610, 0.608 |
| The same, whole take | 0.689 | 0.684 |
| With the extras (main part) | 0.728, 0.724, 0.759 | 0.754, 0.742, 0.742 |
| Aims' offsets mm, p50 / p90 | 13 / 48 | 5 / 28 |
| Largest arm-joint step a frame, p50 / p90 / p95 / p99 | 0.035 / 0.113 / 0.139 / 0.183 rad | 0.033 / 0.106 / 0.133 / 0.177 rad |
| The command moving | 85% of frames | 85% |
| Groups with an add, from home / chained (`--chain --lag 0.2`) | 152 of 196 (78%) / 141 of 189 (75%) | 171 of 199 (86%) / 151 of 196 (77%) |
| Non-add edits the model expects to change the edited object's contact | 67 of 162 (41%) | 112 of 127 (88%) |
| The same, of objects touched or within 1 cm / of the others | 63 of 106 (59%) / 4 of 56 (7%) | 106 of 121 (88%) / 6 of 6 |
| Non-add edits that change whether the take touches anything | 32 of 162 (20%) | 78 of 127 (61%) |

- **What the edits do now.** Every remove, shift and move of an object the model has the arm touch changes that contact; so does every
  shift or move of a near one. The 15 that do not are of objects within 1 cm that the model has the arm miss: the rule treats them as
  touched, since physics may touch them. No edit is of an object farther than 5 cm. 84 of 199 groups had fewer than two edits:
  nothing more was near the path.
- **Why groups lack an add,** from home: the spots under the arm met the quiet start or end (29 groups), or the arm passed too high for
  the parked objects (8). Chained, the quiet start meets more spots (50 groups), since a take starts where the last one ended; with the
  tail kept off the low band that fell from 65 to 50 (the add share chained from 70% to 77%).
- **The twin's adds** (3 of 6 groups in one batch, 5 of 7 in the pilot's): the logs show three causes, none of them the measured track (an
  add looking at the commanded way 0.2 s late finds as many spots as at the commanded one). (1) A group whose drawn add replay was the
  second, and whose first replay had nothing else to edit, skipped both: the pilot's group 400600003 (no object in reach). Fixed: the add
  then comes first. (2) Group 400400006 removed a stand (75) that held a lying elephant its rule did not see, and the layout could not be
  put back for the add. Fixed in `Holds`. (3) The start pose: chained takes start where the last one ended, low, and their quiet start
  sweeps the table there (above). The add's other failures are real: the spots meet the quiet windows.
- **Contacts and height.** At least a third of the planned contacts' tool points are above -8 cm on every seed, and of all contacts by the
  model on two of three (33%, 39%, 38%; chained, 29%): blind moves into low objects keep the share near a third. Stands and supports took
  5-7% of the contacts by the model (before 25%), no planned one. The planned contacts fell from 131 to 94 because the 62 with stands and
  supports are gone; the thin objects' offsets made up some of it.
- **Checked:** `MixedTakeTests` (on jetrover-mixed: no aim at a stand or a support, aims at raised objects keep off their stands, a fifth
  of the planned contacts above -8 cm; blind goals at every height; the bands are checked; blind goals ending low and high on the bench);
  `ReplaysTests` (every edit changes the model's contact for the edited object or is of an object within 1 cm, none farther than 5 cm; a
  group with nothing near its path gets its add at the first replay, and one add a group; a tilted object on a stand is held; each
  replayed command's stamp is newer than the last; the proximity measures the twin's links). The four old scripts' tests are unchanged.
- **Replay timing (measured, not changed but for the stale stamps):** on the pilot's 9 pairs, read only. Aligned by their targets, a
  replay's commands reach RobotHost +5 to +13 ms after their source's at the median (sd 4-9 ms, p95 +13 to +23 ms). Most of it is the
  sender: joint-drive's loop on Windows wakes on its 15.6 ms timer (the source's commands are 15.3 ms apart at the median, 30.6 at p95),
  and its send times differ by sd 4-7 ms; the VPN adds sd 2-4 ms. The renderer's frame clock runs free: a replay's first frame falls -76
  to +47 ms off its source's against the first command. So a replay's frame N samples the commands at another moment than its source's
  frame N: the newest relay target differs in 89-100% of pre-contact frames (p95 17-44 servo units), and in 29-48% even with the frames
  moved to the source's phase. Making pairs exact needs both the commands (joint-drive on the GPU machine, or a high-resolution timer) and
  the frames (lockstep, or the dataset taking each frame's action from the tape at the frame's own time and reporting each episode's frame
  phase); not done.
- **Not run on the twin.** Untested there: the edits as the measured track places them; physics against the model's expected changes;
  the aims near level at tall objects and planks against the caps and the watchdog; whether the stands' share of physics contacts falls as
  the model's did.

**A replay's action is the commands it sent, whatever RobotHost refused (2026-11-24, branch `work/v2-motion4`).** World-models' review of
`jetrover_20261124_021454_cb1e21`: most replay pairs matched, but episode 25 (group 530000007, `add:51`: the cleaner tin stood in the
arm's way and was knocked over) differed from its source, episode 23, by 258 servo units at p95.

- **Where the pair diverged,** stage by stage (`scripts/gpu/replay_pairs.py`, read-only):
  - *Sent* (`JC`): every replay re-sent its source's commands exactly, 16135 of 16135 matched commands identical in the 19 pairs (7 UDP
    datagrams lost), 814 of 814 in this one.
  - *RobotHost:* its collision backstop refused 146 of the source's commands (a run of 145, 1.42-4.41 s into the take) and 32 of the
    replay's (longest run 25, 1.45-1.94 s); the outcomes differ at 114 commands. Both runs began before the added tin mattered (first touched 9.4 s in). The shaper
    and the relay only follow what RobotHost passes on (`RO`).
  - *The dataset:* the action was the newest `RO`, which RobotHost writes only for an accepted command, so a refused command left the one
    before it in force: 258 units at p95.
- **Why RobotHost refused.** Each take's feedback (`RI`) and commands (`JC`) run through RobotHost's own `GenericArmPlant` give all 32131
  recorded outcomes of the dataset's 38 takes. Its 476 refusals (1.5%, in 33 takes) are all the servo-path check (docs/adr/0020, the 2026-11
  bug hunt's C-05); the end pose was clear every time. 310 have a gripper finger within the model's 1 cm floor margin, the table top; 166 a
  gripper, wrist or distal link within it of the robot's own body. The check samples the path every servo takes from where the arm is
  *measured* to the command, and the arm trails its command by about 0.2 s, on the twin as on the real arm (workstream R). At the first
  refusal of each of the 57 runs:
  - the arm was 11.8 degrees from the command (p50; p90 23.2), about 220 ms behind it;
  - the command was 1.6 degrees (p90 3.6) from the last accepted one, and 53 of the 57 were within 4 degrees of it, a step the check would
    not sample at all had the arm stood there. Judged from the last accepted command, 2 of the 57 would have been refused.
  - Refused, the arm finishes its last accepted command and holds there while the commands move on, and the path from it to them keeps
    crossing the margin. Runs last p50 5 commands, p90 16, max 145 (3.0 s, episode 23).
  - Whether a run starts, and how long it lasts, turns on the arm's measured pose to a fraction of a degree, that is on physics.
- **The refusal is right: the real robot does the same.** The robot runs the same RobotHost, and its arm lags as much. Six real headset
  sessions of 2026-10-25 and 27 predate the check and had no refusals. Replayed through today's plant, the four teleop sessions (24.5k
  commands) get none, and the two course sessions (21.1k) get 422 (2.0%) in 123 runs, begun the same way: the arm 9.8 degrees behind (p50,
  about 200 ms), the command 1.3 degrees from the last accepted one. So the refusals stay, as what the robot did. The backstop's own cost
  is for docs/adr/0020 and the user, not changed here:
  - in 18 of the 57 run starts (61 of the 123 real ones) the arm's path to the command it then holds crosses the model as well;
  - a hold can last seconds and end in a jump: 3.0 s in episode 23, the arm about 25 degrees from the command that ended it;
  - judged from the last accepted command, 55 of the 57 commands that began a run would have been taken (what followed would
    differ).

  Across all 641 twin takes on the GPU machine, 5146 commands (1.0%) were refused, in 263 takes; every twin dataset built before held its
  action through them.
- **The fix: the action is the command sent** (`takes.py` `_sent_actions`, begun in 735ab2f; docs/adr/0036 decision 3):
  - every `RO`, and every refused command (`JC` `R`) turned into pulses as the plant would have, at the time it reached the plant. The
    conversion is checked first against the take's own accepted commands and their `RO` (within 0.01 pulse), and pinned on both sides
    (`RelayPulseContractTests`, `test_takes.py`).
  - Not a command ignored while the arm parks (`P`). RobotHost parks the arm through the relay, so its own targets are the `RO` then, and
    counting both (as 735ab2f did) alternated between them.
  - The FR3 Duo the same way: its `AO`, and a refused command within the soft limits, as `SerialChainPlant` would have sent it.
  - An episode spans the commands accepted or refused (it ended at the last accepted one), so a replay spans what its source does.
  - `raw_ros_events.jsonl`'s actions, `capture_timing.jsonl`'s action time and `frames.parquet`'s action skew follow the same events. The
    JetRover state's gripper slot stays its newest `RO`. A take with no refused command reads exactly as before.
  - Replays send the tape unchanged, as before. joint-drive now says how much of it a replay sent (9392ab4), and RobotHost's refused-command
    line prints at last (7421ecf): its throttle overflowed, and the batch behind this dataset logged none of its 476 refusals.

The 19 pairs, per frame: the largest difference over the six motors between a replay's frame k and its source's, servo units. "Before" is
the dataset's own action; "after" is `read_take` at 1dff8ed. replay_pairs.py's `relay` and `sent` columns give the same figures.

| | Before | After |
|---|---|---|
| All 19 pairs (5029 frames): p50 / p95 / max, frames over 20 | 2 / 29 / 289, 8% | 2 / 26 / 183, 8% |
| Episode 25 against 23 | 5 / 258 / 258, 25% | 3 / 31 / 105, 11% |
| Other pairs whose refusals differed: the max | 289, 133, 132, 74, 68 (episodes 5, 4, 2, 36, 11) | 129, 67, 96, 45, 55 |
| Without the frames' phase (the `tape` column) | | 0 / 14 / 66, 3% |

What is left in most pairs (p95 12-38 units, episode 3 67) is the frames' phase and the send timing described above (episode 3 without
the phase: p95 28); this change does not touch it.

- **Checked:**
  - `test_takes.py`: a refused command is the action; a replay with other refusals has its source's action; the conversion pinned by
    hand; a stale command is not the action; while the arm parks the park is the action and the ignored commands are not; an episode runs
    to the last refused command; the FR3 Duo's refused command, clamped; the raw events.
  - `RelayPulseContractTests`: both plants' conversions are the vectors test_takes.py pins.
  - `LogThrottleTests`; `ServoPathCollisionTests` (the same command refused or accepted by where the arm is measured).
  - The dataset's 38 takes read with the new builder (10039 frames): every frame's action is the newest raw action event, and
    capture_timing's action time is that event's (0 exceptions); 152 frames (1.5%) have another action than before, all while
    RobotHost refused. replay_pairs.py run with the lerobot venv adds the dataset's own action column: equal to `relay` for this
    dataset, as it should be; for one built by the new builder it should equal `sent` (not yet seen).
- **Not run on the twin.** Untested there: RobotHost's refused-command line, joint-drive's replay line and a dataset built by the new
  builder.

### C, clearance and contacts

Built 2026-11-22 on `work/v2-contacts`. Checked offline against recorded takes; the table contacts are not yet run on the twin.

**Table contacts.**
- The arm's eight link bodies now take PhysX's contact report themselves (`PhysxContactReportAPI`, threshold 0, set in
  `world.IsaacWorld.build` with the offsets). A surface is a static collider, and the schema takes the report only on a rigid
  body or an articulation, so a link against the table reaches the tracker this way.
- `contacts.ContactTracker(surfaces=...)` reports a link against any colliding scene surface as `WC` object 0, the id the
  format always reserved for the table. That covers the table, the bench beside it (flush with it, the robot stands on it)
  and the floor.
- Unchanged: the touch rule (a contact point within 1 mm, or an impulse), the link index, the force (summed over shape pairs,
  so one link on table and bench is one contact), and the cadence (on change, and every 20 steps while one lasts).
- The base never appears. PhysX filters it against the surfaces, because the mount stands in the bench.
- RobotHost passes object ids through unchecked, so it records `0:<link>:0:<force>` with no change.
- Unit tests (`robot/isaac/tests/test_contacts.py`, synthetic events):
  - a link on the table or bench is object 0, in either order;
  - the touch rule applies to the table too;
  - one link on two surfaces is one contact;
  - objects resting on the table are still not contacts;
  - placing an object never forgets the table;
  - a surface named only by its shape path still counts;
  - without `surfaces` the tracker behaves as before;
  - object 0 round-trips the wire.
- **Fixtures already reported:** they are kinematic bodies with the report API. In the validation set, 41 of 178 touch onsets
  are with fixtures (take 119's right pad on the kettle, id 43, for one).
- **Self-collision is off:** `jetrover_bridge.py` sets `physxArticulation:enabledSelfCollisions` false. Left as it is: no
  link-link contact exists to report.

**The collision geometry** (`robot/isaac/teleop_isaac/clearance_geometry.py`, `just isaac-clearance-geometry <scene>
[coacd|hull]`).
- **How it runs.** Isaac's Python, with no Kit and no rendering: plain `pxr`, numpy, scipy and CoACD (which Isaac's core
  ships). It reads a scene file and writes a cache, by default `~/isaacsim/assets/clearance`.
- **The arm:** `robot-jetrover.json`.
  - Read from the URDF importer's own USD (`~/isaacsim/work/jetrover-<digest>`): 35 `UsdGeom.Cube` colliders on the 8
    links, the finger pads included. They match the URDF's `<collision><box>` box for box (checked on every export).
  - It also holds the links in the URDF's order (WC's index), the joints for forward kinematics, and the fixed bodies (the
    base).
- **Mesh assets:** `assets/<library>/<name>.json`.
  - **What PhysX uses.** `world.py` gives every `UsdGeom.Mesh` a `convexDecomposition` collider, with
    `PhysxConvexDecompositionCollisionAPI`'s defaults: V-HACD, at most 32 hulls of 64 vertices, voxel resolution 500000,
    error 10%, no shrink wrap. Only a running Kit cooks it.
  - **What the export does instead.** It decomposes each mesh with CoACD 1.0.7 at the same budget: 32 hulls, each decimated
    to 64 vertices, threshold 0.05, seed 0. Each piece is stored as a convex hull in its body frame at k = 1.
  - **Its key** is the sha256 of the asset's manifest entry and of the method: a changed asset is exported again, an
    unchanged one is kept.
  - **Cost.** jetrover-bench's 24 assets took 5.4 min on 4 threads. They became 2-70 pieces each: a box 2-3, the cans
    30-31, the round fixtures 32, the green jerrycan's five meshes 70.
  - **Fit** (`--fit`). For each asset, 20,000 points were sampled on its render mesh and measured against the pieces' face
    planes:
    - more than 1 mm outside: a median 1.0% of the surface over the 24 assets;
    - the exceptions: the marble bust 29% (the decimation to 64 vertices shrinks its 22 hulls) and the pudding box 12.7%;
    - more than 5 mm outside: at most 2.6%;
    - the worst point: the ceramic vase's, 31.4 mm out, a thin part the remesh lost; else at most 10.4 mm.
    - Typically the mesh lies 0.1-4.2 mm inside the pieces (CoACD's manifold remesh at resolution 50).
- **Primitives and surfaces** need no export: the clearance builds them from the scene file. A box is exact, a cylinder a
  32-sided prism, a sphere exact.

**The clearance** (`robot/lerobot/teleop_lerobot/clearance.py`).
- **Interface.** `compute(take, geometry_dir)` returns the shared `Clearance`, or raises `ClearanceUnavailable`. That
  happens with no `VQ`, no `WO`, no `scene.json`, or a cache missing the robot or one of the take's assets.
- **Inputs at each frame** (pure numpy):
  - the link poses: forward kinematics of the frame's `VQ`, by frame number;
  - object poses: `WO`, interpolated to the frame's `stateSensed`, with no interpolation to or from the shelf;
  - sizes: the `WZ` in force, else the first `SZ`, else 1;
  - obstacles: every scene object not parked (z below -2 m), clutter included.
- **The distance.**
  - Box against convex piece: GJK when apart, exact to about 1 µm. When they overlap, the separating-axis penetration depth,
    using both shapes' face normals and their edge-edge axes.
  - A link to a surface: its nearest box corner. That is exact wherever the surface's top face is the nearest part, which is
    everywhere the arm reaches.
  - Pairs are culled by bounding spheres and the box's own axes, and visited nearest first.
- **The cutoff is 150 mm.** A near miss (under 30 mm) and about 0.5-0.7 s of a typical approach fall inside it. Beyond it,
  the distance reads 150 and `obstacle_id` reads -1 (`NO_OBSTACLE`).
- **The base's surface distance is NaN,** because PhysX never collides it with a surface.
- **`method`** names the cache's mesh method. `link_object_distance` and `link_surface_distance` give one link's distance at
  any time, with joints interpolated between frames.
- **Tests** (`robot/lerobot/tests/test_clearance.py`):
  - box and box: apart, corner to corner, overlapping;
  - 100+ random turned box pairs against an exact feature-pair reference, within 2 µm;
  - a cylinder prism's corner and side, a sphere, and an octahedron mesh;
  - forward kinematics at hand-worked poses (zero, shoulder up, turntable round, forearm down, fingers apart, roll);
  - a tiny take end to end: a wall 10 mm from the left pad gives 10.00 mm, and 5.00 at factor 1.25 from `WZ` or `SZ`;
  - pose interpolation, parked objects left out, the unavailable cases;
  - 300 frames with 30 objects inside a time budget.

**Checked against PhysX's contacts** (`robot/lerobot/teleop_lerobot/clearance_check.py`, `just isaac-clearance-check
<dataset>`).
- **The set:** the 163 takes of `jetrover_20261122_145748_82747d` (world-models' val split), read-only: 40,685 frames and
  178 touch onsets.
- **A touch onset** is a `WC` record naming a (link, object) pair the record before it did not. PhysX's contact point is
  then within 1 mm, so the right answer is 0-1 mm. Distances are taken at the record's time.
- **Contact-free frames** have no object contact within half a frame plus 50 ms.

| | CoACD pieces (the default) | one hull a mesh (`hull`) |
|---|---|---|
| touch onsets, \|d\| median / p90 | 2.04 / 9.81 mm (signed median +1.94) | 1.81 / 8.20 mm (signed median +1.21) |
| movable objects (137) | 1.57 / 4.09 mm | 1.25 / 3.71 mm |
| fixtures (41) | 8.74 / 14.77 mm | 7.23 / 14.50 mm |
| frames in contact, movable (872 link-object frames) | median -0.76 mm (p10 -1.28, p90 1.84); 2.5% over 3 mm | median -0.36 mm (p10 -1.13, p90 1.49); 1.1% over 3 mm |
| frames in contact, fixtures (376) | median 7.10 mm (p10 -2.58, p90 13.71) | median 3.12 mm (p10 -10.82, p90 12.96) |
| contact-free frames (39,176) with a link overlapping an obstacle | 0.00% | 0.03% (0.01% deeper than 3 mm) |
| compute, a take of about 250 frames | 0.59 s (max 1.28; at b050ccb) | 0.44 s (max 1.68) |

- **Movable objects** agree with PhysX within about 1-2 mm at the onset and in contact, for both methods.
- **The large round fixtures are PhysX's own bulge.** By object, the signed onset medians are:
  - marble bust +14.8 mm, antique vase +12.3, brass vase +10.9, ceramic vase +9.6, gnome +7.9, kettle +4.9;
  - the box-like ones near 0: the jerrycans -1.8 and -2.1, the delivery box -3.5, the parcels -0.3 to +1.6.

  Traces show the arm resting 7-16 mm from the bust and vase meshes all through a contact. So V-HACD's hulls for these
  0.4-0.7 m scans stand that far outside the mesh. No mesh-faithful geometry reproduces that: only PhysX's cooked hulls,
  or a shrink-wrapped decomposition on the PhysX side, would (open question 1).
- **Why CoACD is the default.** It never shows an overlap without a contact. It also keeps concave parts open, as V-HACD
  does, where a hull bridges them: at the brass vase's neck the hull reads -14.7 mm at PhysX's onset.
- **Traces** (mm, from 300 ms before the onset to 300 ms after it; onset marked |):
  - foam brick, right pad: 9.2 5.9 4.1 1.8 0.9 0.6 0.3 | 0.0 -0.2 -0.7 -0.8 -0.5 -0.2;
  - large parcel, gripper: 20.5 14.9 6.9 2.9 1.1 0.5 -0.2 | -0.7 -1.1 -1.1 -1.1 -1.1 -1.0;
  - mug, left pad: 70.2 50.9 34.9 24.2 10.3 5.8 2.3 | 1.1 1.0 1.6 2.0 2.5 4.0;
  - kettle, right pad: 44.4 30.8 16.8 11.1 7.5 6.2 5.4 | 5.0 4.5 3.6 2.8 2.0 1.9;
  - marble bust, wrist: 28.2 23.3 20.1 17.9 16.8 16.5 16.4 | 16.2 16.0 15.7 15.5 15.4 16.7.
- **Surfaces.** The table was not reported in these takes, so this is a sanity check only. The nearest approach a take was a
  median 77 mm. One knock take put a finger pad 3.3-3.9 mm into the table for two frames.

**Untested, for the twin check.**
- The bridge's start line names the surfaces as object 0.
- Takes with low reaches record `WC` entries `0:<link>:0:<force>`, and object contacts are unchanged.
- `just isaac-clearance-check <dataset>` on those takes prints "table touch onsets": the link's surface distance at each
  should be 0-1 mm.
- Run `just isaac-clearance-geometry jetrover-bench` once (into `~/isaacsim/assets/clearance`) before the build, and again
  after S's scene adds assets.

**For L and the others.**
- Until `safety.py` leaves object 0 out, `safety.contacts()` counts a table touch as a contact stretch. A reach or stop take
  that brushes the table would then be set aside as "touched object(s) [0]", and its frames labelled unsafe. Object 0 must
  stay out of `unsafe` and out of the old categories' checks.
- `scripts/gpu/take_report.py` and `take_timing.py` likewise count it as a contact.

**Open questions.**
1. **Should PhysX hug the meshes?** `physxConvexDecompositionCollision:shrinkWrap` (or a higher voxel resolution) in
   `world.py` would bring the fixtures' colliders onto their meshes, and the clearance and the contacts within about 2 mm
   everywhere. It changes the twin's physics, and the touch planner's 9 mm depth was set against today's colliders.
2. **Should the export cook PhysX's own decomposition** in a headless Kit (no rendering), instead of CoACD? Not done: the
   brief asked for no Kit.

### L, labels and dataset

Built in `robot/lerobot/teleop_lerobot/` (`safety.py`, the new `sidecars.py`, `__main__.py`) and the `isaac-safety-build`
recipe; every field is defined in `robot/lerobot/README.md`, "Safety datasets: labels and sidecars".

- **The table is object 0 and never a contact** for the labels or the checks: `safety.contacts` drops `WC` pairs naming
  object 0, so a `WC` naming only the table ends a stretch. The four old categories check exactly as before.
- **`safety-check` for `mixed`** sets a take aside only when it is broken: it can't be read as an episode (no scenario, a
  `.jlog` cut off or failing its trailer, no accepted command, no colour camera, no frame with its joints), joint-drive cut it
  short (`--drive-exit` 2, 3 or 130; 0 and 4 are fine), an `ER` record, records dropped by a writer, no `WO`, or the episode
  repeating one render for 1 s or more. On the 340 episodes of the two datasets of 2026-11-22, the twin's repeats were
  single frames (598 of 45,039 and 560 of 40,685 frames, at most 10 an episode), so the last rule catches a stalled renderer
  only. An unreadable `mixed` take gets a verdict and is set aside; an unreadable old one still raises, as before. Every line
  also says what happened (`outcome`, table contact, timing flags, `RP`), never a reason to set a take aside.
- **Labels:** `safety_ranges.json` with exactly `source` (`physics`), `notes` and `unsafe_ranges`, beside the three legacy
  files, all the same ranges. world-models' own loader (`wm/labeling/ranges.py`, run read-only on a copy) reads the ones built
  from real takes without a warning. `safety_events.jsonl` gives `mixed` a `clear` after every range that ends, as recover had.
  `--table-unsafe` counts the table too.
- **Captions:** `move the arm` for every episode by default; `--caption category` for rebuild comparisons. LeRobot reads a
  neutral build back with one task (tested with LeRobot 0.5.1, and on 3 real takes: 3 episodes, 586 frames, `tasks.parquet`
  holding `move the arm` alone).
- **Sidecars** under `meta/teleop/`: `episodes.jsonl`, `scenes.jsonl`, `contacts.jsonl`, `frames.parquet` and `report.json`,
  read from the take alone. They cost 0.05 s a take with `read_take` (340 takes in 16.8 s on the GPU machine at load 5-6, no
  clearance), and build up front, with `--jobs` processes when there are several.
- **Outcome:** `knock` (an obstacle the arm touched moved at least 3 cm or tipped at least 30 degrees from its first touch to
  the episode's end: the old knock check's rule), `contact`, `near_miss` (no contact, minimum obstacle clearance under 30 mm),
  `no_contact`, and **`unknown`** when there is no contact and no clearance to tell the last two apart. Chosen over
  `no_contact` with a null clearance so that a count of outcomes never passes near misses off as clean negatives. Table
  contact is its own flag.
- **Layout hash (item 36):** sha256 of a canonical JSON of the scene's sha256 and, per object on the table (by `WL`, parked
  ones left out) sorted by id, its integer dimensions after the size factor and position in mm and its rotation matrix in
  thousandths (0.06 degrees), rounded half up, from its pose at the episode's first frame. Pinned by a test that writes the
  recipe out again. The same draw placed twice can settle across a rounding boundary and hash differently, so the split
  groups by the `WL` seed as well.
- **Clearance** comes from workstream C's `clearance.compute`, imported guarded: NaN, and the reason in
  `clearanceUnavailable`, when the geometry folder (`--geometry-dir`, default `~/isaacsim/assets/clearance`) or the module
  is missing or raises. Tested with a fake.
- **Item 17:** `meta/teleop/unavoidable_ranges.json` only with `--stop-time-s` (off by default). Stopped, the arm goes on for
  that time of its speed (travel = speed x time) along its recorded path: a frame not in contact is unavoidable when the next
  contact starts within it. Provisional 0.25 s (p90 0.35 s), workstream R's figure from recorded stops, until the real arm's
  `just stop-latency` run.
- **Item 24:** `firstContactS`, `afterLastContactS` and `timingFlags` per episode, the flagged episodes in `report.json` and
  the build log. Never set aside.
- **Items 20 and 21:** `safety-build --split K` and `just isaac-safety-build <twins> <name> <out> K` write `<name>_<k>of<K>`. Episodes
  that share an `RP` group, a layout hash or a scene and `WL` seed stay in one dataset; the parts go largest first to the
  dataset with the fewest episodes. Each dataset has its `report.json` and a `[mix]` line in the build log (run for real on 4
  takes into 2 datasets in scratch).
- **The LeRobot watcher** leaves `mixed` takes alone, as it leaves the four categories (`safety.SCENARIOS`): they become a
  batch's dataset, not one per scenario. The watcher running on the GPU machine (`~/repos/lerobot` at a2e543a, root
  `~/data/takes/jetrover-isaac`) predates this, so it must be restarted from this work before a `mixed` batch on twin 0.
- **`teleop_lerobot safety-report DATASET...`** checks one build's datasets from their files: the label keys, every replay
  group in one dataset with one source, one look per group (`lookChanges`), one layout seed, edited replays hashed apart,
  clearance at the frames in contact. Exit 1 on a fault. For the twin check.
- **`safety-compare`** compares the sidecars by episode and `frames.parquet` by rows. A reference from before this ADR is
  compared without them, with a note; `isaac-safety-build-check` rebuilds with the reference's own options (an older one: its
  category captions). A `--caption category` rebuild of the first 3 episodes of `jetrover_20261122_145748_82747d` compared
  "the same".

**Measured on the old data** (the 340 episodes of `jetrover_20261122_005619_20ccb6` and `jetrover_20261122_145748_82747d`,
read-only, before any `mixed` take exists):

| | 005619 (177) | 145748 (163) |
|---|---|---|
| labels equal to the dataset's | 177 | 163 |
| outcomes: recover | 37 contact, 3 knock | 34 contact, 1 knock |
| outcomes: knock, reach, stop | 41 knock; 49 and 47 unknown | 40 knock; 45 and 43 unknown |
| contact stretches (object, fixture) | 158, 27 | 118, 36 |
| first contact after the start | min 2.28 s, median 5.24 s | min 2.52 s, median 5.16 s |
| item 24 flags | 2 (last contact under 1.0 s before the end) | 1 (the same) |
| distinct layout hashes, `WL` seeds | 177, 177 | 163, 163 |
| table contacts | 0 (the twin did not report them yet) | 0 |

Four recover takes count as `knock`: their check allows an object moved a few cm while it stays within 20 degrees.

**Untested:** a `mixed` take, `RP` and `MV` from a real twin (synthetic records only; `MV`'s field names follow M's
description and are read leniently), table contacts from the bridge, clearance from C's real module, and a full-size split
build. The batch must pass joint-drive's exit code to `safety-check --drive-exit` (workstream M's recipe) for a watchdog stop
to count as broken.

**After world-models' review, 2026-11-23** (branch `work/v2-dataset-2`). Three changes. The first two reverse what this
section says above, and decision 4's "the table is its own kind, not unsafe" and decision 6's "off by default" for item 17.

1. **Table contacts are unsafe by default.** World-models leans that way: on the real robot a hard table hit is a collision.
   - **The rule:** a frame is unsafe when an arm link touched an obstacle or the table (`WC` object 0). `--table-safe`
     (`Options.table_safe`) builds the old rule, and `safety_ranges.json`'s `notes` then are word for word the old default's.
   - **The default `notes`** give the reason and keep decision 4's trade-off: labels that count the table reward "the path
     goes low", the shortcut this ADR set out to remove.
   - **The kinds stay apart:** `frames.parquet` keeps `in_contact`, `in_contact_table` and `unsafe`, `contacts.jsonl` its kind
     `table`, `episodes.jsonl` `tableContact`. Obstacle-only labels are `in_contact`.
   - **The old categories' verdicts are unchanged.** `safety.contacts` still leaves object 0 out, so a reach or stop take that
     touches the table is kept and a recover take is judged by its obstacle touches alone. `safety-check`'s line still counts
     what the check judged. Only such a take's table-touching frames become unsafe in the four label files (and
     `frames.parquet`'s `unsafe`, `episodes.jsonl`'s `unsafeFrames` and `unsafeRanges`).
   - **`safety_events.jsonl`:** every range gets its `clear` but in `collide_with_obstacle`, which the lab's recorder never
     clears. Until now only recover and `mixed` had one. A reach or stop take can now have a range (a table brush, which ends),
     and without a `clear` its markers would say it stayed unsafe. The lab's `label_safety.py` leaves those two categories all
     safe whatever their markers, so it reads them as before; world-models does not read the file.
   - **Rebuild checks:** `teleop_lerobot safety-options DATASET` gives the options a dataset was built with, from its report.
     `isaac-safety-build-check` uses it. A report from before this change says `tableUnsafe` (false: `--table-safe` now), and
     `safety-compare` reads it by today's names. It leaves out a report's `split`, since a rebuild of a part's takes is one
     dataset.
2. **Unavoidable-contact ranges are on by default,** for analysis only: the labels stay contact.
   - **The stop model** (`Options.stop`) has three named stop times: `fast` 0.17 s, `median` 0.245 s, `slow` 0.31 s. Source:
     "twin stop-latency 2026-11-23, provisional until the real arm is measured". `just isaac-stop-latency` on twin 0 gave
     per-joint, per-speed medians of 170-307 ms net of the round trip. They replace the 0.25 s (p90 0.35 s) above.
   - **The file:** `meta/teleop/unavoidable_ranges.json` is `{"source", "method", "stopTimes": {"fast": 0.17, "median":
     0.245, "slow": 0.31}, "unavoidable_ranges": {"fast": {...}, "median": {...}, "slow": {...}}}`, each set
     `{"<episode>": [[start, end], ...]}`. `--stop-time-s` gives one time instead, named `given`. `--no-unavoidable` writes
     none, as builds before did, so a rebuild of one compares.
   - **Before the contacts the labels count:** a frame that is not unsafe itself is unavoidable when the next counted contact
     starts within the stop time. That is an obstacle, or the table unless built `--table-safe`. If the labels are later
     extended backwards by the stop distance (world-models' roadmap), the file extends what they count. Obstacle-only ranges
     come from `frames.parquet` (`in_contact` false and `time_to_contact_s` at most the stop time), which stays to the next
     obstacle contact as decision 6 has it.
   - **`safety-compare`** restricts each named set to the rebuilt episodes. A report's old `stopTimeS` reads as one
     `given` time.
3. **Splits across datasets.** A source take and its replays share their commands, so a replay group or a layout in two
   split parts leaks. `safety-report` (`sidecars.check_datasets`) now faults when a replay group, a layout hash, or a (scene
   sha, layout seed) is in more than one of the datasets it is given, naming the datasets and episodes (the first 20 of
   each kind, then a count). So a whole train, val and test set can be checked at once. It also prints how many of each
   are in more than one, and each dataset's unavoidable ranges, faulting one that overlaps an unsafe frame.
   - **Fixed on the way:** two datasets with one folder name (`train/ds`, `val/ds`) were one name to the replay-group
     check, so a group parted between them went unseen. Such datasets are now named by their paths, and one given twice is a
     fault.

**Measured on the twin check's takes** (read-only: the 20 `mixed` takes of 2026-11-23 behind `v2check_20261123_133318_1of2`
and `_2of2`, rebuilt at aa1d3a7 on the GPU machine into scratch, `--jobs 4`, 85 s for both parts). "Before" is the twin
check's own build (at the integration tip, before these changes); "after" is today's defaults, `--split 2`.

| | Before | After |
|---|---|---|
| Split (episodes a part), mix | 10 / 10; outcomes 5 contact, 1 knock, 1 near miss, 3 none / 3 contact, 1 knock, 6 none | the same parts and mix |
| Unsafe ranges (frames) | 12 (127) / 8 (264) | the same: world-models' `load_ranges` reads equal ranges, no warning |
| Episodes with a table contact | 0 of 20 | 0 of 20 |
| Unavoidable ranges, fast / median / slow | none | 26 / 40 / 52 frames (6 episodes) and 17 / 25 / 31 (4): 2.1-2.2, 3.1-3.3 and 3.9-4.3 frames before each unsafe range |
| `safety-compare` against before | | 3 differences a part, all intended: `safety_ranges.json`'s notes, `report.json`'s options, and the new `unavoidable_ranges.json` |
| `safety-report` on the two parts | no faults (the old checker) | no faults: 8 replay groups, 16 layout hashes and 8 layout seeds, each in one part |

- **No take touched the table.** The twin's bridge did report surfaces as object 0, but none of the 20 takes, or the 14 later
  ones of that day, has a `WC` pair naming it. So the default labels and events changed nothing here, and the table rule is
  tested only on synthetic takes so far.
- **The options a report records rebuild it.** `safety-options` read each before-part's report, with its old names:
  `--caption neutral --geometry-dir=~/isaacsim/assets/clearance --table-safe --no-unavoidable`. The rebuilds compared
  "the same" file for file with both parts, videos included.
- **The cross-dataset check, on real data.** The after part 1 checked beside the before part 1 (the same takes) gives 20
  faults: 4 replay groups, 8 layout hashes and 4 layout seeds in two datasets (and each group then has two sources).

**Untested:** a real take that touches the table (its labels, `clear` markers and unavoidable ranges before it); the
three stop times against the real arm (`just stop-latency`, the user's run); a full-size train, val and test set.

**One collection stream a dataset, after world-models' review of 2026-11-24** (branch `work/v2-dataset3`). This reverses
docs/adr/0037 amendment 7's "one dataset" from two twins. `jetrover_20261124_021454_cb1e21` (`just
isaac-safety-batch-twins`) held both twins' takes in recorded order. The twins record at once on the GPU machine's one
CLOCK_MONOTONIC, the clock `capture_timing.jsonl` and `raw_ros_events.jsonl` share across a dataset. World-models picks its
path checkpoints from the raw log by time, so a third of them came from the other twin's arm. Per-frame commands, states,
images and labels were right.

- **One dataset per takes root.** Each twin records into its own root, one take after another. `safety-build`
  (`safety.build_streams`) writes each root's takes as a dataset: `<name>` when they all come from one root, else
  `<name>_twin0`, `<name>_twin1` (a root not named for a twin gives its folder's name). `--split K` splits each one on its
  own. The root says where a take was recorded, so grouping by it needs no guess from the times.
- **Refused:** a dataset whose takes overlap on that clock. A take's span runs from the earliest to the latest time its
  lines in the two files hold, ends included. The build exits 3, names every pair, and writes nothing; the recipe does not
  retry it.
- **`safety-report`** places each episode from the two files, by `episode_index` or, before it, by each take's `ready`
  line. It faults overlapping pairs in any dataset that has those files, sidecars or not.
- **`episode_index` on every `raw_ros_events.jsonl` line.** World-models' readers (`wm/jetrover/windows.py` at 70ca04b, and
  the lab's `command_windows.py` on `main`) read `type`, `pc_received_mono_ns` and `values` by name, so the extra field is
  ignored. `safety-compare` leaves it out against an older reference, and says so.
- **`just isaac-safety-rebuild <dataset>`** rebuilds a dataset's takes this way, with the options its report records.

Measured read-only at 0875ab8: the overlapping pairs `safety-report` finds, and the share of action lines a reader picks
by time (inside an episode's first to last frame) that belong to another episode.

| Dataset | Episodes (twin 0, twin 1) | Overlapping pairs | Actions from another episode |
|---|---|---|---|
| `jetrover_20261124_021454_cb1e21` | 38 (19, 19) | 25 | 41.0% (all 38 episodes) |
| `jetrover_20261122_005619_20ccb6` (v2 train) | 177 (90, 87) | 89 | 33.3% (141 episodes) |
| `jetrover_20261122_145748_82747d` (v2 val) | 163 (84, 79) | 76 | 31.3% (127 episodes) |
| `jetrover_20261116_232145_e2369d` (v2 test) | 86 (86, 0) | 0 | 0 |
| `jetrover_20261123_215338_c00c5a` (pilot) | 16 (16, 0) | 0 | 0 |

- **After.** cb1e21's 38 takes rebuilt into scratch with the rebuild recipe's commands and its own options (`--jobs 4`,
  140 s). Result: `_twin0` with 19 episodes and 5114 frames, and `_twin1` with 19 and 4925. Neither has an overlapping
  pair or an action from another episode, and `safety-report` finds no faults in the two. With `--split 2` (166 s), each
  twin's takes are parts of 9 and 10 episodes, each part one twin's, and every one of the 19 replay groups, 38 layout
  hashes and 19 layout seeds is in one of the four parts.
- **Nothing else changed.** Take by take, the labels, `safety_ranges.json`, unavoidable ranges, events, the four sidecars,
  the timing lines, the raw lines (less `episode_index`) and the data parquet's states, actions and times equal the merged
  dataset's, 38 of 38. Videos were not compared.
- **World-models' v2 train and val sets mix twins too,** so their path features had the same fault. The test set and the
  pilot come from one twin.

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
  0–4 units closer), same step timing and top speed. The overshoot and settle differences are one to three pulses.
  docs/adr/0034 amendment 13's fit (held-out p95 2.64 degrees) stands.
- **Being over 20 units most of the time is how this arm teleoperates.** Any arm here trails its command by about 0.2 s, so a
  joint commanded faster than about 100 units/s (0.42 rad/s) is over the threshold (78–82% of joint-frames at 100–200 units/s,
  real or twin, and nearly all above), and people keep the arm moving 70–86% of the time. The real dataset world-models holds
  is at 75% itself.
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

Past the held command: p50 0 degrees, p90 0.2–1.9, real and twin. **Provisional for item 17:** after its command stops, the
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

**Run on the twin (2026-11-23, twin 0 at 6e9d34d):** `just isaac-stop-latency 0` measured all 22 of 22 trials. Medians net
of the round trip (24 ms): 170-307 ms per joint and speed, about 245 ms overall. The arm travelled about the lag distance
(7°, 14° and 27° at 0.5, 1 and 2 rad/s) and overshot its held command by at most 0.4°
(`recordings/.stop-latency/20261123T133108Z-isaac-twin0.json`). These are the provisional stop times L's unavoidable ranges
use.

**Not done or untested:** the stop tool against the real robot. `isaac-stop-latency` runs over the VPN and takes the
measured round trip out, but not the read pipeline. The caps are applied (M). The twin's physics truth (`VQ`) is not used,
only the readings, so the twin compares with the real arm on the same terms.

### A, appearance

Built 2026-11-22 on `work/v2-looks`; nothing of it has run on the twin yet. Items 30, 31 and the renderer's side of 28.

**An optional scene section, `appearance`.** `robot/isaac/teleop_isaac/appearance.py` reads it and draws from it, and
`renderer.py` applies it at every change: at start-up and after each closed session, so once per take. **A scene file
without the section renders and records exactly as before**: no draw, no record, no wait. `core/Scenes/jetrover-bench.json`
does not get one, since the headset checks its sha. The coordinator adds this section to `core/Scenes/jetrover-mixed.json`
at integration. It is the same as `robot/isaac/tests/appearance-example.json`, which the tests read:

```json
"appearance": {
 "objects": {"brightness": [0.85, 1.1], "desaturation": [0.0, 0.25], "tint": 0.05},
 "lighting": {
  "colourTemperatureK": [5500, 7500],
  "exposure": [108, 128],
  "key": {"intensity": [0.5, 2.5], "azimuthDeg": [0, 360], "elevationDeg": [35, 75], "angleDeg": [1, 6],
          "colourTemperatureK": [3500, 6500]}
 },
 "cameras": {
  "room": {"translationM": 0.015, "rotationDeg": 2.0},
  "mounted": {"translationM": 0.003, "rotationDeg": 1.5}
 }
}
```

Every part is optional, and every value is checked against fixed bounds, for example brightness 0.5-1.5, tint at most 0.3
and a camera moved at most 5 cm and 10°. An unknown field is refused. Each draw is a pure function of `BK`'s seed, the change
and what it is for, from its own `random.Random` key, so a draw never moves another. Values are rounded before they are
applied, so the records hold exactly what was applied.

- **Obstacle colours** (`objects`, record `OA`). Every scanned object (`shape` `mesh`), fixtures and raised clutter
  included, gets its own colour each take:
  - its albedo is mixed toward its own luminance by `desaturation`;
  - it is multiplied by a tint, a slight cast of a random hue whose Rec. 709 luminance is 1, so the cast moves the hue and
    not the brightness;
  - it is multiplied by `brightness`.

  How depends on the network the renderer draws, the first of Kit's render contexts (`mdl`, `mtlx`, then the universal one)
  that the material has:
  - an OmniPBR material through its own `albedo_desaturation`, `diffuse_tint` and `albedo_brightness`;
  - a MaterialX surface through two nodes put in front of its `base_color`, `ND_saturate_color3` then `ND_multiply_color3`,
    neutral until set;
  - a UsdPreviewSurface through its colour texture's `scale`, which takes no desaturation.

  Checked offline with pxr against every scanned asset of the bench (24, referenced into a stage as `world.py` does):
  - the 9 YCB objects are OmniPBR;
  - the 15 Poly Haven models are MaterialX, 16 materials in all (the kettle has two): 9 `standard_surface` and 6 OpenPBR
    models. The marble bust's base colour comes from a Mix node;
  - none sits inside an instance, where it could not be edited.

  Simple shapes keep the looks `materials.objects` gives them. Alternate textures: none of the libraries has any. Each Poly
  Haven model and YCB object comes with one texture set, and the green jerrycan is an asset of its own, already in the pool.
  So nothing is downloaded and no licence changes.
- **The light** (`lighting`, record `LT`).
  - Every light the scene file lists (the HDRI dome) gets a colour temperature.
  - The metering aims at a drawn `exposure` instead of the environment's `targetBrightness`. The metering still runs after
    the light is set, so overall intensity is kept within what a camera's auto-exposure would allow.
  - A key light the section adds gives the take's key direction. It is a distant light, `/World/Lights/AppearanceKey`,
    with its azimuth (from +x toward +y, in the base frame), elevation, intensity, angular size and colour temperature drawn.
    Its intensity is a multiple of the environment's `meanRadiance`. The metering then lowers the dome to hold the exposure,
    so a stronger key gives harder shadows, not a brighter image.

  The HDRI's yaw was already drawn per take, which turned the room's own light about the vertical. The key adds a
  directional light that can come from any side and height.
- **Camera jitter** (`cameras`, records `CJ` and `CM`). Every camera is moved along each of its own axes and turned about
  each (x, then y, then z: tilt, pan, roll): up to 1.5 cm and 2° for `room` cameras and 3 mm and 1.5° for the `mounted` wrist
  camera (`front` in the datasets).
  - A room camera is set where Isaac renders it, and its pose is read back as a check.
  - The wrist camera's pose in `wrist_link`'s frame, which the render loop composes with the link every frame, is moved.
  - `CM` records the start-up pose moved by the same offset, so it is the pose the take was rendered from. `CM` is written
    in every session's preamble, and each take is a session.
  - Moving the pose in the mount frame and posing it by the link equals posing it and then moving it in its own frame (a
    test), so a reader's forward kinematics agrees with the renderer.
- **Order within a change.** Object colours go first, so the materials have the rest of the change to take. Then the
  existing materials, looks and clutter. Then the jitter and the light, before the room settles, so the metering and each
  camera's own gain see the take as it will be recorded.
- **A session waits for its change.** RobotHost can open the next take's session in the same 0.1 s poll that closes the
  last one. With the section, the session's preamble then waits, up to 30 s, until the change is in force, so its `CM`,
  `LT`, `CJ`, `OA`, `EN`, `LK` and `CL` describe its frames. Without the section, it opens at once, as before.
- **Counterfactual replays (a proposal to M; dormant until used).** Each replay is a new session, so the renderer would
  give it a new room, colours, light and camera pose, and a pair would differ by more than its edit.
  - While a file `.keep-look` sits beside RobotHost's control file (`~/data/sessions/jetrover-isaac[-twin1]/.keep-look`), a
    closed session brings no change, for at most 16 sessions in a row.
  - The replay's records then repeat its source's change number, so L can check a group shares one look.
  - Proposed use: joint-drive creates the file when it starts a source take with `--replays N`, and removes it when it
    starts the last replay. Nothing writes it yet.

**Frame pacing (item 28), measured read-only** over the last 120 takes of each twin (2026-11-22, `~/data/takes/jetrover-isaac*`,
106,893 recorded frames in 3,964 s of recording):

| | Twin 0 | Twin 1 |
|---|---|---|
| Frames recorded, against the recorded span at 30 fps | 54,456 of 60,208 (90.4%) | 52,437 of 58,702 (89.3%) |
| Frame interval (state stamps): p5 / median / p95 / p99 / max | 28.0 / 36.0 / 44.1 / 96.1 / 116.1 ms | 28.0 / 36.0 / 44.1 / 99.9 / 112.3 ms |
| Intervals over 55 ms | 803 (1.48%), 789 of them 80-150 ms | 863 (1.65%), 847 of them 80-150 ms |
| Renderer frame rate in its 5 s reports (`RH`): p5 / median | 24.2 / 28.1 fps | 23.8 / 27.2 fps |
| Frame queued for writing after its physics state (`VF` t − `stateSensed`): median / p99 | 114 / 193 ms | 117 / 194 ms |

- No frame was dropped (no `FD`, no gap in frame numbers), and no frame showed the same state as the one before.
- Frames with the same joint values as the one before (418 and 410) were the arm standing still, with new state stamps.
- So frames missing from a fixed 30 fps grid come from the render loop itself running slow while it records: about 36 ms a
  frame, against 29.5-30.4 fps at idle (amendment 7 of ADR 0037).
- On top of that, 1.5% of intervals are stalls of 96-116 ms, three frame times, at irregular moments (0.7-13 s apart), not
  at the 5 s reports or the world snapshots.

**Causes found, and the cheap fixes** (`renderer.py`). Measured on the GPU machine's CPU, one core, 640×360:

- **Masks** were made by comparing the frame with each label in turn: 2.3, 2.7 and 3.7 ms a camera for 8, 16 and 32
  labels, on the render thread, for every camera that records masks (three). One table lookup over the frame (`stable_mask`)
  takes 0.20 ms and gives the same ids (a test against the old way on random frames, signed and out-of-range ids included).
  That is about 7-10 ms back on every recorded frame.
- **A camera's exposure gain** goes through OpenCV's table lookup: 0.13 ms against 0.45 ms, the same bytes (a test).
- **Objects' poses**: a pose that has not changed is no longer set again. Parked and sleeping bodies resend theirs about
  100 times a second, for 30 objects.
- **Python's garbage collector**: a full collection over Kit's whole heap is one cause that fits the stalls. The heap is
  frozen out of later collections once the scene is built (`--gc-freeze`, on by default; `--no-gc-freeze` turns it off).
  This is not shown to be the cause yet. Each 5 s report now prints the collector's counts per generation and its longest
  pause.
- **Where each frame's time goes**: each 5 s report also gives the median and longest of `tap`, `pose`, `render`, `read`,
  `frame` and `change` in ms, so the twin check can attribute what is left.
- The writer thread's encoding is unchanged and kept up: PNG 2.1 ms for a mask and 3.2 ms for a depth image, JPEG 0.75 ms.

Expected, not measured: recording no longer costs the loop its frame rate, so a median interval near 33.3 ms and over 98% of
the grid recorded. If the 100 ms stalls remain with the collector's pauses short, their cause is elsewhere (the GPU shared by
two renderers, or Kit).

**Frame-rate cost of the look, estimated (nothing measured on the twin):**
- Object colours, colour temperature, exposure and camera jitter cost nothing per frame: each is set once per take.
  - A camera's DLAA history restarts when it moves, which happens inside the unrecorded settle.
  - A MaterialX material whose inputs change may be recompiled by RTX, once per take, between takes. How long that takes is
    unknown. It is the one risk to the take's first frames, so the check below measures it.
- The key light is the only per-frame cost: one more light in Real-Time 2.0 at 2 samples a pixel, about 1.4 M more shadow
  rays a frame for three 640×360 cameras. Estimated at under 3% of the render time.
  - Leaving `key` out of the section removes it.
  - The mask fix gives the render thread back several times that.

**Tests** (`robot/isaac/tests/test_appearance.py`, 34 cases, and `test_renderer.py`):
- the shipped scenes have no section;
- the example reads, and a bad section is refused (16 cases);
- every draw is deterministic, within its bounds and fills them;
- a tint keeps unit luminance and its own hue;
- each light quantity is drawn from its own stream;
- the key light shines from its drawn direction;
- a jitter moves the camera along and turns it about its own axes, within bounds, the same in the mount frame as in the
  room;
- `OA`, `LT` and `CJ` encode as documented;
- with pxr (Isaac's Python): an OmniPBR, a MaterialX, a preview-only and an unsupported material, each changed or left alone
  as above;
- the masks, the exposure table, the collector's and the loop's reports, `--gc-freeze` and `.keep-look`.

`scripts/gpu/gate.sh isaac` runs 213 tests, and skips the 4 pxr cases, since the gate's Python has no pxr. All 34
appearance cases pass under Isaac's Python on the GPU machine.

**A check for the takes, `robot/isaac/appearance_check.py`** (read-only, the LeRobot venv). It prints a `RESULT` line and
exits non-zero on a failure.
- **Records against the draws:** `LT`, `CJ` and `OA` must be the draws, and `CM` the scene file's pose moved by `CJ`.
- **Each camera's pose in its own depth:** every camera's table depth, back-projected through its `CM` pose (the wrist's
  through forward kinematics of the frame's `VQ`), must land on the table top within 3 mm at p99.
  - The same through the scene file's pose, ignoring the jitter, is reported beside it. A camera the renderer did not
    really move would match the file's pose instead.
- **Late colours and pacing:** a late colour shows as a jump between a take's first frame and a second later. The frame
  pacing is reported too.
- `--sheet` makes a contact sheet of each scanned object across takes, captioned with its drawn colour.
- On four takes of 2026-11-22 (no section, `--baseline`):
  - `CM` was within 0.00001° of the scene file's poses (0.004° as first measured: the arc-cosine artefact below);
  - table depth through `CM` was 0.31 mm at p99 for the room cameras and 0.49 mm for the wrist;
  - late-colour jumps were 2.5-5.0 (of 255; the arm's shadow);
  - median frame interval 32.1 ms.

**The twin check** (needs the user's go). After the merge, on `jetrover-mixed` with this section:
1. A short batch of `mixed` takes on one twin (M's `isaac-safety-batch`, 4 takes, no build). Then, on the GPU machine from `robot/isaac`:
   `~/isaacsim/lerobot-venv/bin/python appearance_check.py <the takes> --sheet /tmp/sheet.jpg`. It passes with no problem
   listed, every camera's `cm_p99` under 3 mm and `file_p99` larger.
2. **The renderer's log.**
   - At start: `[render] appearance: N scanned objects, their materials {'mtlx': .., 'omnipbr': ..} under render contexts
     ['mdl', 'mtlx', '']`, with `none to vary on none`, and `[render] Python heap frozen: ...`.
   - In each 5 s report: the frame rate, `read` at a few ms (the masks alone took about 7-10 ms of it before), and the
     collector's longest pause.
   - With two twins recording, the frame rate must hold about 30 fps. Compare `render` with and without `key` if it does not.
3. **The contact sheet and one video per camera.**
   - Objects differ in brightness, saturation and a slight hue across takes, and stay what they are: a soup can stays red.
   - None is black, magenta, flat grey or untextured. That would be a MaterialX network RTX did not compile.
   - The key light's shadows fall in different directions.
   - Colour casts stay mild, warm to cool.
   - The third-person and wrist views shift slightly between takes, with the gripper still in the wrist view's lower half.
4. **Late colours.** `colour_jump_worst` stays under about 8. Tens means a material took after the take began; the settle
   must then wait for it.
5. **Optional and decisive for the material path.** The same with exaggerated `objects` ranges (brightness [0.5, 1.5],
   desaturation [0, 1], tint 0.3) on a scratch copy of the scene. Some objects must come out grey and some clearly tinted,
   both YCB (OmniPBR) and Poly Haven (MaterialX) ones.

**The twin check, 2026-11-23** (20 `mixed` takes on one twin, at 65946e2). The checker flagged every take, twice. In both
cases `CM` was right and the checker was wrong. Fixed in `appearance_check.py` (and in the renderer's read-back warning),
measured read-only on the same takes:

- **Table depth through `CM` 4.8-21.3 mm off at p99** for third-left or third-right in 10 of 20 takes.
  - Through `CM`, the median height error was 0.00-0.01 mm in every one of them.
  - Every point off lay at x = 0.572 m, 3-40 mm below the top: the table's front face. The `table` mask covers the table's
    sides too, and a camera that its jitter turned or moved down sees that face in a bottom corner of its image (1-2.4% of
    its table pixels in the takes that failed).
  - None of the alternatives fitted better: the jitter's shift or turn in the room's frame, the turn alone, the shift alone,
    or the change before or after.
  - The checker now takes only table points that fall on the top's footprint, 1 cm inside its edges.
  - Through `CM`, every camera of every take is then 0.28-0.52 mm at p99 (0.28-0.30 for the room cameras, 0.48-0.52 for
    the wrist), against 2.4-66.5 mm (median 16.7) through the scene file's pose. A plane fitted to the top through `CM` tilts
    0.0031° at most (9 of the takes fitted).
- **`CM` 0.01-0.04° from the scene file's pose moved by `CJ`**, in every take.
  - The angle was measured as twice the arc cosine of the two quaternions' dot product, which near 1 turns the 1e-7
    rounding of `CM`'s 7 decimals into hundredths of a degree. It read exactly 0 wherever the rounded dot product reached 1.
  - Measured from the relative quaternion (`appearance.turn_deg`), it is 0.00001° and 0.0001 mm in every take. The check's
    bounds are now 0.01 mm and 0.001°.
  - The renderer's read-back warnings (0.020-0.022°, 3 of them in the run) were the same artefact. They use `turn_deg` now,
    at 0.001°.
- **Tests that would have caught it** (`tests/test_appearance_check.py`):
  - the bench's table is ray-cast from take 017's jittered pose, with its front face in view: flat to 0.6 mm through the
    pose rendered, more than 3 mm without the footprint, and more than 3 mm through four wrong poses (the scene file's, 0.5°
    more, 1 cm lower, the jitter's turn in the room's frame);
  - `CM` rounded to 7 decimals is within the check's bounds over 300 draws;
  - `turn_deg` is exact at small turns, where the arc cosine reads a 7-decimal rounding as up to 0.05°.
- **Frame pacing, the first measure since the fixes** (one twin recording, so not like for like with the 240 takes above,
  which ran beside a second twin): median interval 32.04 ms, 0.24% over 55 ms, all of the grid recorded. Late-colour jumps
  are at most 7.2.
- The renderer's look was right as recorded, so no take needs re-recording for this.

**Textures at their real size (2026-11-23).** After the twin check, the simple shapes looked pale or nearly flat white, their
grain stretched. These were the mixed scenes' stand blocks, crates, planks, legs and supports (boxes), and the rod, dowel and
puck (cylinders).

- **The cause, from the shader's own code** (`OmniPBR_ClearCoat.mdl`; `base::coordinate_projection`, `compute_cubic_transform`
  and `transform_coordinate` in NVIDIA's MDL SDK).
  - Projecting in object space, OmniPBR takes two of the shape's own coordinates as a face's texture coordinates, and
    `texture_scale` multiplies them.
  - A box (and the table and the bench) is a `UsdGeom.Cube` of size 1 scaled by its prim. Its object coordinates run ±0.5
    whatever its size, so every side of every face showed the same number of tiles: the 1.8 cm edge of a 30 × 8 × 1.8 cm
    plank as many as its 30 cm length.
  - A 1 m veneer was squeezed about 10× onto a 10 cm block and 55× onto that edge, and mip-mapping averaged it to its mean
    colour, hence the flat, pale look. The pale woods (ash, bamboo, pine) made it worse.
  - A cylinder is built at its size, so its object space is metres, but its tiling was multiplied by its size: a 22 cm
    rod's texture was 4.5× too large.
- **Measured on a recorded take** (`stop_before_collision/20261122T141557Z-take127`, table look `cutting-mat`, a 1 cm grid on
  a 0.30 m tile). The table top seen by each third-person camera was resampled top-down at 0.5 mm a pixel through `CM`.
  - The grid repeated every 1.55 cm along the table's 1.6 m (harmonics at 3.1, 4.7 and 6.3 cm), in both cameras, and
    about every 0.5 cm along its 0.47 m. That is the unit cube's 1.6 cm and 0.47 cm, not 1 cm both ways.
  - Every table and bench look since 2026-11-17 was stretched 3.4:1 the same way.
- **The fix** (`renderer.py`, the renderer only; the bridge and its colliders are unchanged).
  - A box, the table or the bench, when it wears a texture, becomes in place the same unit cube as a mesh, with texture
    coordinates in metres at its real size (`box_mesh`).
    - Each face has its own four corners and u runs along its longer side, so a plank's grain follows its length.
    - Each face's coordinates are turned, not mirrored, so a texture reads the right way round.
    - OmniPBR reads these coordinates instead of projecting.
  - The prim keeps its path, transform (its scale too), mask label, visibility and material binding (`metric_box`; a pxr
    test checks each), so geometry, depth and masks are unchanged.
  - A resize redoes the coordinates at the new size.
  - Every textured shape now tiles at `tileScale / sizeM` a metre (`tiling`), whatever its size; a cylinder stays projected
    in its own object space. Shapes and surfaces without a texture are untouched, so `fr3-duo-bench` renders as before.
  - The looks the mixed scenes give these shapes are real woods and metals, 0.3-2.5 m a tile, and are unchanged. At real
    size a face shows its own few centimetres of grain, at about 1 mm a texel. No scene file changed, and no golden either.
- **Tests:**
  - every face of `box_mesh` is the box's real size in metres, for five boxes from a cube to a 36 cm post: any two corners
    as far apart in texture coordinates as in the room, counter-clockwise from outside, not mirrored, u along the longer
    side;
  - the tiling is repeats a metre;
  - with pxr, a box turned into a mesh keeps its world bounds, transform, binding and label.
- **This changes every scene with textured boxes or surfaces,** `jetrover-bench` too: its table and bench looks, the phone
  and the pen.
- **What to look for on the next twin run** (any scene with looks; the contact sheet from `appearance_check.py --sheet`, and
  frames from each third-person camera):
  - **Planks** (shelf, cantilever and bench planks, slats): wood grain running along their length and unstretched on all
    faces. The thin edges show a thin strip of grain, not smeared stripes.
  - **Blocks, crates, legs, pedestals, risers:** visible wood or metal texture on every face, at the same scale on each
    face. None flat white or cream (pale ash or pine is fine if its grain shows).
  - **Rod, dowel, puck:** grain or metal at real scale. The four seams where the cube projection meets around a cylinder
    remain, unchanged.
  - **The table and bench:** the cutting mat's squares square (1 cm, a bold line every 5 cm). Laminate and wood grain at a
    believable scale. Fabric and paper weaves fine, not stretched sideways.
  - **Masks and depth unchanged:** `appearance_check.py` still has every camera's table depth through `CM` under 3 mm.
  - **The renderer's log:** `[render] textures at their real size: N boxes and M surfaces as meshes with coordinates in
    metres`, and no warning.

**Masks: every drawn object keeps its own mask id (2026-11-24, branch `work/v2-masks5`).** The visibility workstream found
objects drawn in rgb whose pixels carried mask id 0 for whole takes: twin0 lost 11 of its last 13 `mixed` takes from take092
on, twin1 5, and the held-out scene every object but the drill (125) in most takes. Read from the takes (each object's
pixels in every mask camera against where `OL` and `WO` put it, replays' `RP` removals set aside), the pattern is one rule:
**an object that came onto the table was unlabelled from then on, and one that stayed on the table kept its label.** The
renderer hid a parked object (`MakeInvisible` on the object's Xform, which carried the label its shapes inherited) and drew
it again when it was unparked, and it measured the render lag at start-up with every object hidden. Isaac 6.2's
segmentation (its semantic structure, which tracks labels, instances and prim counts frame to frame) then gave the shapes
back with no label, Isaac's UNLABELLED, which `stable_mask` writes as 0, until something rebuilt its tables (a later change
sometimes did: twin1 recovered at change 3, twin0's pen at change 4). The age of the twin is not the cause (the held-out run
lost its objects from its first take), and nor are resizes, looks or `metric_box` (a box turned into a mesh keeps its label;
scanned meshes lost theirs the same way). The fix, in `robot/isaac/teleop_isaac/renderer.py`:
- **No object is hidden and drawn again.** A parked object is left drawn on the shelf when no camera can see that far
  (`out_of_sight`: farther than every camera's far plane plus 0.5 m, or 2 m for a camera on the arm). The shelf is 14 m or
  more away, and the far planes are 6 m on the JetRover and 10 m on the FR3. A parked object is hidden only when a camera
  might see it, and is relabelled when shown again. A test holds every shipped scene's shelf slots out of sight. The lag is
  now measured with the objects moved a kilometre down. Nothing seen changes: the shelf was already beyond every far plane.
- **Each object's shapes carry its label themselves** (`label_object`), not its Xform: every `Gprim` under the object, a
  scanned asset's meshes, a textured box after it became a mesh, or an instanced asset's instance. A joined label
  (`object_37,object_37`, or an asset's own label beside the object's) still names its object (`mask_label_id`).
- **A check at each take's start**, and after an unpark or a resize once the render lag has passed. Over 6 recorded frames,
  each object on the table must have pixels of its id in some mask if a still camera sees its centre with nothing nearer
  in its depth there. An object with none is logged (`[render] warning: mask check ...`), recorded (`ML`,
  `trainer/contract/SESSION_FORMAT.md`) and relabelled once a take (its label cleared and set again two frames later, so
  Isaac resolves it afresh), then checked again. The check also reports, for each camera, how many pixels a frame were
  drawn with no id. At a take's start the stage's labels are also compared with what was authored, and any that are gone
  are put back. Cost: about 0.4 ms a mask camera on the check's frames (measured on the GPU machine's CPU), about 7 ms a
  take, and nothing on other frames.
- **Tests:** `robot/isaac/tests/test_masks.py` covers the joined labels, the unlabelled pixels, the shelf out of every
  scene's sight, the projection and the depth test, the check's window and the renderer's wiring. With pxr (Isaac's
  Python), it checks the labels on each object's shapes through a box turned into a mesh, a resize, a visibility toggle and
  a relabel, and on a scanned asset's meshes and an instanced one.
- **Not yet run on a twin.** What the twin check should show:
  - each take's `isaac.jlog` has `ML` records with an empty `missing` field;
  - `renderer.log` has `[render] mask labels: N shapes of M objects labelled themselves` at start-up, a
    `[render] mask check (take, <take>): ... every one in the masks` line for each take, no `mask check` warning and no
    `relabelled` line;
  - a per-take scan finds every object on the table with pixels of its id in some camera, the held-out scene's ids
    202-229 included.
  A `mask check` warning that names the same object again after its relabel would mean hiding is not the only cause.
