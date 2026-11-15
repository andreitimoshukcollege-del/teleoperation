# ADR 0037: Varied scenes for the JetRover twin: real objects, walls and HDRI rooms

## Status

Accepted, 2026-11-15 (the user, on starting P1). P1 built 2026-11-15; P2 built 2026-11-15 (amendment 1).

## Context

The JetRover twin's safety datasets (ADR 0036 amendments 1 and 2) are recorded on one scene, `core/Scenes/jetrover-bench.json`:

- **Objects:** four primitive objects (a cube, a post, a can, a wall) with fixed sizes, colours and masses. Each take only
  moves them (a seeded layout, ±jitter around their nominal poses) or resizes them in steps.
- **Table:** a random Open Images photo lies on it as a mat.
- **Surroundings:** four more photos stand around it on flat screens, lit by a fixed dome and a distant light.

The world model trained on this sees the same four shapes every episode, a table that is a photo, and surroundings that
are pictures on flat screens. The lighting does not match any of them.

The user's asks (2026-11-15):

1. a plain grey table;
2. objects that differ in size, shape and everything else each episode, with immovable walls and movable objects of
   different sizes that look and behave like real objects;
3. surroundings that are constructed, continuous environments with lighting that matches them.

The user chose HDRI rooms, YCB objects, and this ADR before any code.

**How a scene is wired today** (surveyed 2026-11-15). A scene is a file whose identity is its sha256. RobotHost, the Isaac
bridge, the renderer, joint-drive and the headset each load their own copy at start-up. RobotHost only runs a world whose
bridge reports the same sha (WORLD_INFO), and joint-drive and the headset refuse a mismatch. Who builds what:

- **Isaac's physics stage** is built once, at start-up: a rigid body per object, plus 11 parked resize variants per
  movable object. PhysX cannot swap colliders while it steps.
- **The headset's twin** is baked in the Unity editor.
- **Object shape, size, mass and colour never travel on any wire**: every process joins on the object id. Per take,
  only the layout (poses) and the resize step change.
- **Limits:** at most 24 objects and ids 1-254, objects stand upright on the table, and layouts only jitter around
  nominal poses.

**Assets** (checked from the GPU machine, 2026-11-15):

- **NVIDIA's Isaac asset server has 21 scanned YCB objects** (`Isaac/Props/YCB/Axis_Aligned/`): cans, boxes, a mug, a
  banana, a foam brick, a wood block, a drill, clamps, scissors, a marker and others, with their real textures. Physics
  comes with only four of them. YCB publishes every object's measured mass.
- **Poly Haven serves 301 indoor HDRIs** (CC0): 360° photographs of real rooms.

## Decision

1. **The table is plain grey.** The mat backdrop goes; the table's colour is a neutral grey.

2. **The surroundings are an HDRI room, a different one each take.**
   - The renderer replaces the backdrop screens with a dome light textured with a Poly Haven indoor HDRI. The same photo
     is both what the cameras see beyond the table and the light that falls on the scene, so background and lighting
     match by construction.
   - The fixed distant light goes, and the table stays. The floor stays in the bridge, where it catches a falling object,
     but the renderer no longer draws it (the scene's `hideSurfaces`), so the room's own floor shows below the table.
   - The room changes when the backdrop does today: at start and after each closed session, and each take is a session.
     The draw is seeded (the backdrop's seed) and recorded in `isaac.jlog` (`EN`: change, seed, HDRI id and sha256, yaw,
     dome intensity, settle frames) and `isaac/environment.json` (every room's attribution), as backdrops are now.
   - Rooms come in rounds: every room once, in a seeded order, before any comes back. Each is turned by a seeded yaw.
   - HDRIs are fetched once into `~/isaacsim/assets/hdri/polyhaven-indoor/` by a recipe (2K Radiance `.hdr`: all 301
     indoor rooms, 1.9 GB, fetched in under 3 minutes on 2026-11-15). Each is kept only if it matches the API's size
     and md5 and decodes to a 2:1 panorama. It is stored with its measured mean luminance over the sphere, so the dome's
     intensity starts normalised (the scene's `meanRadiance` over it).
     - The rooms' own mean luminance spans 0.14 to 2.2, a factor of 16.
     - Normalised, the third-person camera's mean brightness stayed between 112 and 154 (of 255) across five rooms, but
       between 91 and 185 across the seven takes of the first batch, and the wrist camera's between 74 and 206: a white
       studio is bright in the image whatever light it gives.
     - So the renderer also **meters**, as a camera's auto-exposure does. Once a room has settled, it reads the first still
       camera's mean brightness and corrects the dome toward the scene's `targetBrightness` (118). That is at most 3
       steps, within 0.25-4x of the normalised intensity, and all before the session records. Over ten rooms that held
       the third-person camera between 115 and 130.
     - The wrist camera still ranged from 46 to 175: in a cave, the room beyond the table is bright and the table under
       it is dim. So every other camera also meters its own image when the room changes (the arm at rest between takes)
       and keeps that gain for the session, like a camera with its own auto-exposure locked per take. The gain is applied
       to its 8-bit colour in linear light, within 0.25-4x, and recorded in `EN`.
   - A new dome texture shows within a frame or two. After a change, the renderer re-renders the pose, unrecorded, until
     the image holds still: 5-6 frames, measured. Frames for one render lag more are flagged `B`.
   - The renderer alone does this: no wire or C# change. A photo at infinity has no parallax. If near-field geometry
     turns out to matter, a later phase can add 3D rooms (Isaac's environments, or rooms built from walls, windows and
     furniture).

3. **A scene is a pool, and each take's layout chooses which objects of it take part.**
   - The scene file lists a pool of up to 24 objects:
     - movable real objects (decision 4);
     - immovable walls and fixtures of several sizes and materials, each its own entry.
   - A layout now draws, from its seed:
     - how many objects of each role take part (for example 2-5 movable, 1-3 walls);
     - which ones;
     - where they stand, anywhere in the placement region that fits with clearance, not only jittered around a nominal
       pose.
   - The rest are **parked**: off the table, hidden from every camera, and out of every collision.
   - What changes:
     - **WorldScene:** a parked state that passes validation.
     - **SceneLayout:** subset-and-region sampling, seeded and identical in C# and `scene.py` (testdata vectors).
     - **WorldHost:** draws and records the active set (`WL` gains it; `WP`/`WO` flag parked).
     - **The world frames:** a parked flag per object (a spare flag bit, no new message).
     - **The Isaac bridge:** stands parked bodies on a hidden shelf, as it does resize variants.
     - **The renderer and the headset:** hide parked objects.
     - **joint-drive:** plans around the active objects only.
   - **What stays:**
     - the scene file, its sha check and the wire;
     - the physics stage, built once;
     - the baked headset twin (every pool object, shown or hidden);
     - the LeRobot safety checker, which reads object ids, `WO` and `WC` only.
   - Swapping whole scenes per take was considered and refused (Alternatives).

4. **Movable objects are real YCB objects at their real size and mass.** A new object shape, `mesh`:
   - **The file entry:** names the asset (`ycb/025_mug`); its `size` is the asset's axis-aligned bounds, which C# uses for
     footprints and collision checks and the headset draws as a box.
   - **In Isaac:** the asset is referenced for its look, with a convex-decomposition collider, YCB's published mass, and a
     friction from its material (cardboard, metal, ceramic, plastic, wood).
   - **Assets:** fetched once into `~/isaacsim/assets/ycb/`. YCB is CC-BY 4.0; the NVIDIA conversions are used under
     Isaac's asset terms.
   - **Fit for the JetRover:** its wrist reaches 14-24 cm out and its arm is weak, so the pool takes the objects that fit
     it, by role:
     - **pushable and knockable:** light or tippy objects (foam brick 28 g, marker 16 g, banana 66 g, gelatin box 97 g,
       mug 118 g, scissors, the small cans);
     - **heavy or tall ones:** fixtures in effect (wood block 729 g, mustard bottle 603 g, sugar box 514 g).
   - Real masses are heavier than today's 30-50 g primitives. Whether the twin's arm can still knock an object over is
     measured per object before it is given the knockable role.

5. **Walls are immovable boxes of varied size and real material.**
   - Each wall entry is a fixed body (kinematic) of its own dimensions: thickness 1.5-4 cm, length 8-25 cm, height 4-12 cm.
   - The renderer gives it a material (painted MDF, plywood, brushed metal, plastic) drawn per take from CC0 PBR textures
     fetched with the HDRIs.
   - Appearance is the renderer's alone; physics keeps each entry's friction.

6. **The safety scripts know roles.** The pool file tags each object `knockable`, `pushable` or `fixture`.
   - `collide_with_obstacle` aims only at knockable objects; `collide_and_recover` at pushable or knockable ones.
   - `stop_before_collision` and `reach_no_obstacle` treat every active object alike.
   - The take check stays physics: tipped, moved or untouched, by `WO` and `WC`.

7. **Phases, one PR each:**
   1. **P1:** grey table and HDRI rooms (renderer, recipes, records).
   2. **P2:** the pool and parked objects, first with primitive walls and blocks of varied size: Core, RobotHost, the
      bridge, the renderer, joint-drive and the headset, with testdata vectors pinning C# and Python layouts equal.
   3. **P3:** YCB `mesh` objects: assets, colliders, masses, roles measured on the twin.
   4. **P4:** role-aware safety scripts and a new batch.

## Consequences

- **What each episode differs in:**
  - which objects, how many, their shapes, sizes, masses and materials;
  - where they stand;
  - the room around the table and its light.

  The table is the same grey every time, and the robot and cameras do not change.
- **Only P2 touches every process.** P1 and P3 are mostly the renderer's and the bridge's.
- **Fixed per-pool caps:** a pool holds at most 24 objects, and a take uses a subset of them. Variety in shape comes from
  the pool's size; variety in size comes from it too, since only uniform resize steps exist and real objects are not
  rescaled.
- **The headset shows a mesh object as its box,** in the colour of the pool entry, until a later change gives it meshes.
- **Earlier recordings keep their meaning.** A scene with no `role` and no parked objects behaves as today (every object
  active), so `fr3-duo-bench`, the old JetRover bench and their recordings are unchanged.
- **Assets add about 2 GB** in `~/isaacsim/assets/` on the GPU machine.

## Alternatives considered

- **A whole new scene file per take.** Isaac's stage would be rebuilt between takes, which means stopping PhysX and
  reloading. The headset's editor-baked twin would have to be built at runtime from a scene sent over the joint port,
  and each scene's sha would be renegotiated. Much more change, for no variety the pool cannot give.
- **3D environments first** (Isaac's office, warehouse and simple room, or rooms built from parts). They have real
  parallax and occlusion, but there are few distinct rooms, each costs more frames, and lighting must be set up per room.
  This stays the next step if the HDRI's lack of parallax matters.
- **A 360° photo of our own lab.** It is the best match to the real robot's camera. It fits decision 2 as one more HDRI
  as soon as someone takes the photo.
- **Google Scanned Objects as well** (about 1000 more items, CC-BY). They come without masses, which would have to be
  estimated. Left for later.
- **Procedural objects only.** Unlimited variety, but they look less real; kept for walls and fixtures (decision 5).

## Amendment 1 (2026-11-15): P2 as built

- **The pool.** `jetrover-bench` lists 24 objects:
  - the four primitives it had;
  - 12 more movable ones: boxes, cylinders, a plate, a domino, a slab, from 1.2 to 12 cm and 15 to 140 g;
  - 8 fixed walls and fixtures, up to 25 cm long and 12 cm high.

  Its `pool` asks each layout for 2-5 movable objects and 1-2 fixed ones (the user's choice of "up to about 12 cm"). Scene
  files with no `pool` behave exactly as before, so their layouts are unchanged; `layout-seed1.txt` still pins them.
- **A layout** draws, from its seed alone:
  - how many objects of each kind;
  - which ones (a partial shuffle);
  - a spot for each, fixed ones first, uniform where its footprint fits and yaw uniform, redrawn while it would touch one
    already placed (64 tries).

  One that finds no spot is parked. A spot the backend then refuses is redrawn anywhere in the region; after the redraws the
  object is parked, not left where it stood. Reset puts the layout back as drawn.
- **Parking is a placement** with a new cause, `park` (5 on the Isaac wire; 4 stays RobotHost's resize record). The bridge
  stands the body on its existing hidden shelf (the slot of its current size), and reports the object with a new flag bit,
  `FlagParked` (2). A pool's objects start parked, so their nominal poses may share the table and validation skips the
  nominal-overlap rule for a pool.
- **The headset's world frame carries no flags** (its 0x86 format is unchanged). The headset's and joint-drive's
  `WorldState` mark a pose more than 2 m below the table top as parked: the shelf is 5 m down, and an object knocked to the
  floor lands under 1 m down. The headset hides a parked object; its grab gestures and joint-drive's plans skip it.
- **One piece of decision 6 came forward.** `collide_with_obstacle` aims only at movable objects, since a fixed wall cannot
  be knocked over; walls remain obstacles to every script. Roles (`knockable`, `pushable`, `fixture`) are still P4's.
- **Tests:**
  - shared scene vectors (pool rules, both readers);
  - the pool's layouts (counts, apart, on the table, varied, deterministic, no allocation);
  - WorldHost over StaticWorld (parking, records, reset, headset refusals, redraw-then-park);
  - the bridge's model and wire;
  - the safety scripts, which still plan on at least 80% of 16 pool layouts per category.
- **The shelf floats what rests on it.** Parked cylinders tipped on the flat shelf, rolled off its edge and fell for good.
  One was found 1.9 km down, so joint-drive never saw the world at rest. A body put on the shelf (parked, or a size not in
  use) now has gravity off and no velocity; one brought back has gravity again.
- **On the twin (2026-11-15, an 8-take batch):** layouts of 4 to 7 objects. 6 takes were kept, 1 set aside and 1 not
  planned. The objects touched were the big block, the wedge and the wide can, where the old bench only ever had the post
  and the wall touched. The take set aside was `collide_with_obstacle` pushing the 140 g big block 2.6 cm without tipping
  it: what can be knocked over is P4's to measure.
- **Still for the user:** the headset's world twin is baked in the editor (`just unity-xr-jetrover-world`) and must be
  rebaked for the new objects, then the Galaxy app rebuilt.
