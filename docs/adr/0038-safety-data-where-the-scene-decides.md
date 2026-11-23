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

(pending)

### A, appearance

(pending)
