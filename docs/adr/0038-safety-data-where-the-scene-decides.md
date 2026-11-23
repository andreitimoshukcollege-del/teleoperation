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
  - **Fit.** Against 20,000 points sampled on each render mesh (6 assets), the pieces leave at most 1.0% of the surface more
    than 1 mm outside, and none more than 6.1 mm. The mesh lies a median 0.2-3.6 mm inside the pieces (CoACD's manifold
    remesh).
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
| compute, a take of about 250 frames | 0.62 s (max 1.36) | 0.44 s (max 1.68) |

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

**Not done or untested:** the stop tool against a robot or a twin; `isaac-stop-latency` runs over the VPN and takes the
measured round trip out, but not the read pipeline; the caps are M's to apply; the twin's physics truth (`VQ`) is not used,
only the readings, to compare with the real arm on the same terms.

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
  - `CM` was within 0.004° of the scene file's poses;
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
