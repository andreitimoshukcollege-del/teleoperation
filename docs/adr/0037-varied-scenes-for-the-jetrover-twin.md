# ADR 0037: Varied scenes for the JetRover twin: real objects, walls and HDRI rooms

## Status

Accepted, 2026-11-15 (the user, on starting P1). P1 built 2026-11-15; P2 built 2026-11-15 (amendments 1 and 2); P3 built 2026-11-15 (amendment 3); P4 built 2026-11-15 (amendment 4). Real fixtures and materials 2026-11-16 (amendment 5). Tables that look used 2026-11-17 (amendment 6). Surfaces and flat clutter 2026-11-17, raised clutter 2026-11-21 (their own sections). Two twins at once 2026-11-21 (amendment 7).

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

## Amendment 2 (2026-11-15): a size per object per take

The user asked for random sizes per take.

- **The draw.** A pool may set `sizeMin` and `sizeMax` (m; `jetrover-bench`: 0.02 and 0.12). Then each layout also draws a
  size for every movable object the headset may resize:
  - one of the existing resize steps (1.25 to the power −3 to 8), uniform among those that keep its largest side in that
    range;
  - its spot is drawn at that size.

  The 2.5 cm cube runs from 2 cm to about 12 cm. Fixed walls keep their sizes; they have no resize bodies, and their sizes
  already vary across the pool. Mass grows with the cube of the size, as the bridge's resize bodies already have it.
- **Sizes change on the shelf, never on the table.** A new layout:
  1. parks every object;
  2. resizes on the shelf each one whose drawn size differs (a resize of a parked object only swaps its shelf body, with no
     table rule to pass);
  3. places the drawn ones.

  The backend takes these in order, so the table is empty while it is filled and no placement meets a stale object.
  RobotHost still refuses a resize of a parked object from the headset.
- **Recorded as before.** `WZ` holds the sizes, and the headset scales its twin from the SIZES message.

## Amendment 3 (2026-11-15): P3 as built, real YCB objects

- **Assets.** `teleop_isaac.ycb` (`just isaac-ycb-fetch`) fetched all 21 objects NVIDIA converts into
  `~/isaacsim/assets/ycb/`, each USD with its one colour texture, checked and recorded by sha256.
  - The assets are axis-aligned but not consistently upright: the cans and the mug are tall along their y, the boxes and
    tools along z. The fetch table names each one's up, and the manifest records the rotation that stands it so, with its
    size and box centre in that standing frame.
  - Masses are YCB's published ones. Friction is by material, typical dry values that were not measured.
- **The `mesh` shape.** A scene object with `"shape": "mesh"` names its `asset` (`ycb/025_mug`); its `size` is its standing
  box.
  - **In Isaac:** the asset is referenced under the body, rotated and centred, with a convex decomposition collider per mesh
    and the object's own friction bound above the asset's render material.
  - **Everywhere else:** footprints, the headset's twin (a box in the entry's colour) and joint-drive's plans use the box.
  - **No resizing:** a real object has no resize bodies and layouts never resize it. Real objects keep their real size.
- **The pool.** `jetrover-bench` swaps eight primitives for the eight YCB objects within about 12 cm, ids 31-38:
  - tomato soup can (349 g), tuna can (171 g), pudding box (187 g), gelatin box (97 g);
  - potted meat can (370 g), mug (118 g), the large marker standing on its end (16 g), foam brick (28 g).

  The pool is still 24: 8 primitives that layouts resize, 8 real objects and 8 fixed walls. Larger YCB objects (the cracker
  and sugar boxes, the bottles, the drill) are fetched but not in the pool: they are beyond the user's 12 cm.
- **On the twin, an 8-take batch:** all 8 kept. `collide_with_obstacle` knocked over the YCB marker and the foam brick.
- **A reach band.** Uniform over the whole region, the bigger objects often stood out of the arm's reach:
  - the wrist reaches 14-24 cm from the base, and the region runs to 44 cm;
  - the safety scripts' random-layout test fell to 9 and 14 of 16 for the two collide categories, against its bar of 80%.

  A pool may set `reachMin`/`reachMax` (m from the robot's base, horizontally; `jetrover-bench`: 0.15 and 0.30): its movable
  objects stand only there. Walls keep the whole region (the long ones need it to fit; they are obstacles). The test plans
  every category on at least 80% of layouts again, at the bar it always had.
- **Large objects take part less often.** Their round footprint more often finds no free spot in the band. Over 2000
  layouts, the mug was on the table in 7.5% and the pudding box in 9.6%, against about 20% for the small primitives.

## Amendment 4 (2026-11-15): P4 as built, roles from measurement

Decision 6 tagged each object with one role. With sizes per take (amendment 2), one tag cannot be right: a 2 cm cube tips,
a 12 cm one weighs about 2 kg. Roles are therefore decided per take, from the object at its drawn size, against a threshold
measured on the twin.

- **The measurement.**
  - `teleop_lerobot knock-report` lists every object a take touched: its size (the renderer's `SZ`), its mass at that size,
    how far it tipped and moved from its first touch, and whether that is knocked over by the take check's own rule
    (tipped at least 30 degrees or moved at least 3 cm).
  - `just isaac-knock-probe` runs `collide_with_obstacle` takes alone (`isaac-safety-batch` now takes `categories=` and
    `build=`) and reports them.
- **What it found** (48 collide takes, 51 touched movable objects, 2026-11-15):

  | Mass at size | Knocked over |
  |---|---|
  | 0–30 g | 17 of 20 |
  | 30–60 g | 1 of 1 |
  | 60–100 g | 8 of 8 |
  | 100–200 g | 7 of 9 |
  | 200–400 g | 10 of 12 |
  | 400–800 g | 1 of 1 (a 670 g can) |
  | Fixed walls | 0 of 6 |

  - Every miss was a graze, whatever the mass.
  - The twin's arm shoves objects of several hundred grams; "knocked" counts a 3 cm shove.
  - Nothing heavier than 670 g was touched, so the data says nothing above it.
- **The knock role.** `collide_with_obstacle` aims only at **knockable** objects: movable, and at most the pool's
  `knockMaxMass` at its size. For `jetrover-bench` that is 0.7 kg, the heaviest mass measured knocked over. A scene without
  it keeps the old rule (any movable object).
- **The steady role.** `collide_and_recover` fails a take that tips its object, so it aims only at **steady** objects: a
  wall, a movable one above `knockMaxMass`, or a squat one.
  - An object pushed at mid-height tips before it slides when its height exceeds its narrower side over its friction
    (about 0.8): taller than 1.25 times its narrower side is **tippy**.
  - The recover takes agree. The tall post (height over narrow side 4.0) and the marker (6.4) tipped; the wedge (0.7) and
    the foam brick (1.03) slid.
  - This corrects decision 6, which had collide_and_recover aim at pushable or knockable objects: a knockable object is
    exactly what fails it.
- **The world preamble.** The JetRover host never wrote WorldHost's preamble (`WL`, `WZ`, `WO`) into its sessions and takes,
  as the serial-chain host does. It does now, so a take says which objects were on the table and at what size.
- **Tests:**
  - the roles (each collide script plans only against objects of its role);
  - the random-layout test, which every category passes at its 80% bar with roles on;
  - the knock report and its size parse;
  - the pool's `knockMaxMass` in the shared vectors.

## Amendment 5 (2026-11-16): fixtures bigger than the robot, and nothing without a real surface

The user's judgement after P4:
1. In real life, immovable things near a robot are big, bigger than the robot itself; the thin walls were tiny.
2. Nothing should be a flat colour, since such objects cannot exist in real life.

The user chose furniture-scale fixtures at the edges, and real materials together with the scans.

- **Fixtures.** The 8 thin walls become 8 Poly Haven models (CC0), each 30-69 cm tall: two jerrycans, an electric kettle, two
  vases, a marble bust, a garden gnome and a brass vase. They stay `fixed` (kinematic) and keep their real look and size.
  - **The fetcher.** `teleop_isaac.models` fetches them as USD with their textures. Each file is checked against the API's
    size and md5, and is fetched only from Poly Haven's download host and only under `textures/`.
  - **Texture paths.** Poly Haven's USD names its textures by an absolute path on Poly Haven's own machines. The fetch
    points each reference at the file downloaded beside it, or at the same map in the format the download included, and
    refuses a model that names one it lacks.
  - **Choosing them.** `survey` measured 270 models of the fitting categories. These 8 stand sensibly on a table, are
    taller than the robot, and fit a wing.
- **Where fixtures stand.** A pool's `fixedRegions` are rectangles where its fixed objects stand instead of the placement
  region. `jetrover-bench` uses the table's two wings, |y| 0.36-0.80 m.
  - The third-person cameras sit beyond the table's far edge, looking back at the robot, so a fixture there would block
    their view.
  - A fixture is drawn facing the robot, give or take 20 degrees.
  - Every placement rule (Core's `FitsOnTable`, the bridge's `Scene.fits_on_table`) uses the regions, and the shared
    vectors hold the two readers equal.
- **Materials.** The simple shapes wear real CC0 PBR textures, drawn per take with the room (seeded, recorded as `MT`, and
  attributed in `environment.json`).
  - Boxes are wooden blocks: walnut, ash, bamboo, oak, cherry and pine veneers, and painted planks.
  - Cylinders are metal or wooden cans: plain and rusted plates, painted metal.
  - The table and bench are a grey oak laminate (`grey_oak_veneer_01`, desaturated and tinted grey).
  - `teleop_isaac.materials` fetches the 20 textures, checked the same way, with each one's real-world size so it tiles
    at its true scale. The shapes have no UVs, so the renderer projects in object space (OmniPBR's `project_uvw`), which
    keeps the texture on as the object moves.
- **What is left.** Every object in the scene now has a scanned or real-material surface; only the hidden floor and the
  shelf have none.
- **On the twin, a 4-take check:** 3 kept and 1 not planned. Fixtures and textures rendered with every texture found.

### Amendment 5, revised the same day: fixtures within reach, scans only, delivery boxes

The user's judgement of the first check:
- the fixtures stood on the wings, 36-80 cm from the arm's base, out of the reach of a wrist that reaches 14-24 cm;
- the small simple shapes still looked untextured: a 2-5 cm block shows only a few pixels of a texture that spans 1.5-2 m;
- and the user asked for cardboard delivery boxes.

The user chose fixtures beside the robot, and real scans in place of the simple shapes.

- **Fixtures beside the robot.**
  - A fixed region now bounds the object's **centre**, and the footprint must lie on the table or on the bench beside it
    (flush with it). `jetrover-bench` puts the regions beside the robot's body: x -0.15 to 0.06 m, |y| 0.25-0.50 m.
  - Two more pool fields:
    - `robotKeepOut`: no footprint may reach the robot, at x -0.21 to 0.14 m and |y| up to 0.20 m. That is the chassis, plus
      room for the elbow to swing; fixtures hugging the chassis took the arm's paths.
    - `fixedNearMax` (0.22 m): a fixture's near side comes within the arm's reach.
  - The third-person cameras see the robot from the front, so a fixture beside it blocks nothing of their view.
- **One fixture a take** (`fixedMax` 1). Two, one on each side, boxed the arm in.
- **Every object is a real scan.**
  - The 8 simple shapes are gone. In their place are five small Poly Haven props (a cleaner tin, a carved wooden elephant,
    a wooden bowl) and **delivery parcels**: Poly Haven's cardboard box at 0.30, 0.34, 0.38 and 0.42 of its size. The two
    flattest props (a mousetrap 1.8 cm tall, a roll of tape 2.5 cm) were dropped, too low for a touch.
  - A mesh entry may be its asset uniformly scaled (`world.mesh_factor`; every axis must agree within 3%).
  - The same box at 0.85, 33 x 44 x 29 cm, is a fixture.
  - Masses for Poly Haven props are estimates: Poly Haven publishes none (a parcel 120-250 g, as an empty box).
  - No object is resizable now (meshes keep their size), so the pool has no size range.
- **`collide_and_recover`** prefers steady objects, and tries the others only when none can be approached. With fixtures
  beside the robot and many low objects, steady ones alone left half the layouts unplannable; a tipped object sets the
  take aside at its check instead.
- **What a big object within reach costs.** It takes the arm's approach paths away. Over 40 layouts, with the scene's roles:

  | Category | Layouts planned |
  |---|---|
  | `reach_no_obstacle` | 100% |
  | `collide_with_obstacle` | 80% |
  | `stop_before_collision` | 70% |
  | `collide_and_recover` | 63% |

  Without the fixture, nearly all layouts plan. The random-layout test's bars were 80% for every category. They are now 80%,
  60%, 50% and 50% respectively, with this reason in the test: a layout a script cannot use costs the batch a few seconds
  and another layout, never a take.

### Amendment 5, second revision (2026-11-16): fixtures are touched, and the cameras come closer

The user saw that the fixtures still looked out of reach, and asked whether the arm ever touched one. It never had. No take
since the fixtures came touched one (ids 41-49), and a fixture standing alone beside the robot planned no stop and no touch
at any spot:
- the safety scripts approached an object only on arcs **above its top**, and every fixture's top (28 cm and up) is out of a
  wrist that reaches only 2 cm below to 10 cm above the arm base frame's origin, 16-24 cm out, within 90° of straight ahead
  (mapped through the headset's IK);
- and a fixture beside the robot is wide and near: seen from the base it spans 20-30° each side of its middle, so a level
  hand already reaches its corner where the arc began, 0.6 rad off its middle, and no arc found a clear start.

So an object at least 15 cm tall (`SafetyScript.TallObjectMeters`) is also swept **along its side**: the wrist at 16-24 cm
and 2 cm below to 6 cm above the frame's origin, at least 4 cm below the object's top, the hand level or tilted down by up to
0.6 rad, so the gripper reaching past the wrist meets the object's side. The arc begins 0.6 rad beyond the object's edge
as seen from the base, not its middle, and goes as far past its far edge.

A fixture is always steady, and now nearly always in reach, so `collide_and_recover` would have touched it in 34 of 40
layouts. It tries the fixture first in half of its takes (`FixtureFirstShare`) and after every movable object in the rest.
Over 40 layouts, with the scene's roles:

| Category | Layouts planned | The fixture the target |
|---|---|---|
| `reach_no_obstacle` | 100% | (none touched) |
| `collide_with_obstacle` | 85% | never: a fixture cannot be knocked over |
| `stop_before_collision` | 98% | 22 |
| `collide_and_recover` | 98% | 21 |

The random-layout test's bars for the stop and the touch go back to 80%. A new test plans both against a 22 × 22 × 50 cm box
alone beside the robot, in front of it and off to either side.

The third-person cameras move a quarter of the way in towards the point they look at: from (1.15, ±0.68, 0.40) m to
(0.88, ±0.51, 0.32) m, the user's request, so the robot and the objects fill more of the frame.

**Objects as they are turned (the same day).** The first check on the twin aimed a touch at a fixture, a jerrycan turned
95°, and Isaac reported no contact. Each object was its axis-aligned bounds. Measured offline at the jerrycan's pose, the
touches went 18-20 mm into those bounds but between 10 mm short of the can's own box and 5 mm into it: the arc meets a box at
a corner, and a turned object's real corner lies up to 3 cm inside its bounds' corner. The collision model's arm boxes
are also about 15 mm larger than Isaac's links (the reason for the 20 mm depth), so the arm stopped just short. A turned
movable object had the same shortfall, only less often seen.

So `CollisionBox` may be turned (`Teleop.RobotArm`): its own axes, centre and half extents, with `Min`/`Max` kept as its
bounds for the broad phase. The separating-axis test already took two turned boxes, and an axis-aligned box computes
exactly as before. `SafetyObject` carries the object's rotation from its world pose. A touch now goes 19-20 mm into the
jerrycan's own box. Sweeps are still placed from the bounds.
Over the same 40 layouts the fixture is now the target of 17 stops and 16 touches (it was 22 and 21): its own box is
smaller than its bounds, so fewer arcs meet it.

### Amendment 5, third revision (2026-11-17): a touch is planned against the object's own shape

The batch after the second revision (`just isaac-safety-batch 25 10 all true`, joint-drive at 94bb0a7) set aside 12 of 25
`collide_and_recover` takes as "never touched an object", none with a contact record: takes 059, 060, 064, 065, 066, 070,
071, 075, 076, 079, 080 and 082 of that scenario. Measured offline against the objects' meshes:

- **The objects are far smaller than their boxes.** Eleven of the misses aimed at a round or irregular scan: the garden
  gnome, the three vases, the marble bust, the kettle (fixtures) and the wooden elephant (movable). The twelfth aimed at the
  large marker. The arm went 19.5-21 mm into the target's box and stayed 18-103 mm from its mesh, mostly from the mesh's
  convex hull too (the table below). Only take 059 reached its mesh, the elephant's, by 0.3 mm, and it recorded no contact.
- **The model's fingers are not the twin's.** The C# model's arm matches the twin's URDF (`robot/jetrover/urdf/jetrover.urdf`)
  link for link, but for the fingers. Its `gripper.1` and `gripper.2` (two CAD boxes at x 0.127 m) do not exist in Isaac,
  which has two pads, about 30 x 11 x 7 mm, on prismatic joints, about 32 mm either side of the gripper's axis when it is
  open. Take 066 aimed at the large marker (19 mm across) with the CAD's fingers, and the twin's open fingers passed either
  side of it.
- In the 12 takes that were kept, Isaac's links (from the twin's own joint values) came between 3.8 mm inside and 2.4 mm
  outside the real mesh, and the link that went deepest was a finger pad in 8. So the reason the second revision gave for
  the 20 mm depth ("the model's boxes are about 15 mm larger than Isaac's links") was wrong: they are Isaac's links, but for
  the fingers. The shortfall was the objects' own shapes.

**The shapes.** `robot/isaac/shape_survey.py` measures each mesh asset as `world.py` stands it. Its output is committed beside
the scene as `core/Scenes/jetrover-bench.shapes.json` (format `teleop-object-shapes`, version 1).

- Each asset is cut along its standing z into slabs. Each slab is a convex polygon, counter-clockwise, extruded from its `z0`
  to its `z1`. Every coordinate is a fraction of the asset's box (its manifest's `size`, -0.5..0.5 about the box's centre),
  so one entry serves every size of the asset (the five cardboard boxes are one entry).
- It is measured from the **render mesh**: every `UsdGeom.Mesh` under the asset, not PhysX's convex decomposition. Only a
  running Kit cooks the decomposition, and the survey starts none: it runs in Isaac's Python over ssh stdin, prints JSON,
  and writes nothing on the GPU machine. The decomposition lies a few millimetres outside the mesh.
- A thin layer's section is the hull of the mesh's vertices in it and of the points where triangle edges cross its planes,
  so no layer is empty. Layers are merged into slabs so that the largest **excess** of any slab is as small as it can be.
  The excess is how far a slab's prism can stand off the mesh: its overhang past the narrowest section in it, but at most
  the slab's height (a surface that overhangs by more is nearly level, and is within that height above or below).
- The bottom 8 mm is a slab of its own, left out of that cost: the collision model keeps the arm 10 mm above the table.
- **8 to 48 slabs, not 8 to 16.** The survey's first cut, at most 16 slabs with the overhang alone as the cost, left six
  assets overhanging their mesh by 11-33 mm (the bust 33, the gnome 20, the brass vase 15, the kettle 13, the green jerrycan
  12, the cardboard box 11): as much as the touch depth, or more. With up to 48 slabs, the worst are the bust (8.7 mm), the
  gnome (6.5) and the brass vase (5.8), and 17 of the 20 assets are within 4.4 mm. The slab count costs nothing per check:
  a box is tested only against the slabs its height meets.
- **Checked against the meshes**, points on every slab's sides (above the floor band) stand off the mesh by a median of
  0.5-2.7 mm per asset. The tails are where a hull fills in a concave section: the kettle's handle and spout (90th
  percentile 13.5 mm, worst 33 mm), the jerrycans' handles (99th percentile 20-24 mm), the cardboard box's flaps, the gnome,
  the bowl, and the elephant's legs and trunk (90th percentile 7.1 mm). The excess does not see these.
- The 24 objects are 20 assets, not the 16 counted while the misses were measured.

**The planner.** `SafetyObject` has an optional `Shape` (`ObjectShape`: the asset's slabs scaled to the object's box).
`WorldView.LoadScene` loads the shapes file beside the scene when there is one, and `WorldView.Bounds` and `TryObjects` give
each scanned object its shape, so joint-drive plans with it (its dry run too). Only Teleop.Eval reads the file.

- Only `collide_and_recover`'s touch uses the shape. Its sweep must reach the shape itself (the line's first contact), and
  the touch goes `ShapeTouchDepthMeters`, **9 mm**, into it, but no deeper than 3 mm short of the inner radius of the slab
  the line meets (the large marker gets about 6 mm).
- The depth is measured by `ShapeGap`, as `GapLadder` measures a box: a separating-axis test between the arm's boxes and
  each slab's prism, the shape grown or shrunk in 0.5 mm steps (each slab's polygon offset edge by edge, and the stack's
  bottom with it).
- **A touch goes into the object's side, never down through its top.** For a touch the shape stops 10 mm below its top
  (`ObjectShape.TouchTop`), and no offset moves that. With the top counted, two re-planned touches missed the mesh by
  coming down through it: a pad 8.5 mm below the wooden bowl's rim and 28 mm inside its outline was in the bowl, 45 mm from
  the wood; another, 7.7 mm below a parcel's concave top, was 10 mm from the cardboard. A slab's polygon covers whatever
  hollow is under its top. A pad 10 mm below the top got there moving sideways, so it crossed the object's wall.
- **The arm for a touch is the twin's**: the model's bodies without `gripper.1` and `gripper.2`, plus the URDF's two pads,
  placed where each tick's commanded gripper holds them (the sidecar's finger travel table). The approach opens the gripper,
  but the hand model's staging grip can still be ramping during the final approach: in a test, a touch met its can with the
  gripper at 0.88.
- Why 9 mm: Isaac now counts a touch only within 1 mm, or with an impulse. Its colliders have a contact offset of 5 mm and
  a rest offset of 0, the mesh colliders lie a little outside the render mesh, and a slab's prism can stand off a sloping
  mesh by the survey's excess.
- `stop_before_collision`, every clearance check (where the final approach starts, the path there) and the other objects
  stay on the full boxes with the model's bodies, so a stop's clearance does not shrink. An object without a shape is
  touched 20 mm into its box, as before.

**The misses, re-planned** (`TouchMissesTests`). Each take's layout, start pose, seed and speed are in the test project's
`testdata/touch-misses.json`. Given the same record, the old planner reproduces every command the twin was sent (within
0.0002 rad), so the record is right.
- "Box" and "shape" are how far the plan goes into the object's box (the collision model's bodies) and into its slab shape
  (`ShapeGap`). -40 means 40 mm short or more.
- "Mesh" is the GPU machine's check, approach.py's method: Isaac's link boxes, the pads where the commanded gripper held
  them, against the mesh's surface. Positive is how deep the mesh's surface lies inside an arm box, which saturates at a
  pad's half thickness, 3.5-3.7 mm ("pad"). Negative is how far short of the surface the arm stayed.

| Take | Old target | Its shape | Old: box | Old: shape | Old: mesh | New target | New: shape | New: mesh |
|---|---|---|---|---|---|---|---|---|
| 059 | wooden elephant | movable, legs and trunk | 19.5 | 7.0 | 0.3 | wooden bowl | 9.0 | 3.7 pad |
| 060 | garden gnome | fixture, irregular | 19.5 | -40 | -51.5 | garden gnome | 9.0 | 3.5 pad |
| 064 | antique vase | fixture, round | 19.5 | -24.0 | -23.6 | antique vase | 9.0 | 6.9 |
| 065 | ceramic vase | fixture, round | 20.0 | -17.0 | -18.4 | ceramic vase | 8.5 | 3.3 pad |
| 066 | large marker | movable, 19 mm across | 5.5 | -5.5 | -5.5 | large delivery box | 9.5 | 3.5 pad |
| 070 | brass vase | fixture, round | 21.0 | -40 | -40.3 | tuna can | 9.0 | 3.7 pad |
| 071 | marble bust | fixture, irregular | 19.5 | -40 | -74.0 | marble bust | 8.0 | 1.3 |
| 075 | kettle | fixture, handle and spout | 20.0 | -40 | -102.6 | kettle | 9.0 | 3.7 pad |
| 076 | ceramic vase | fixture, round | 20.5 | -40 | -60.4 | ceramic vase | 9.0 | 3.5 pad |
| 079 | brass vase | fixture, round | 20.5 | -21.5 | -25.0 | parcel (mid) | 9.0 | 3.5 pad |
| 080 | marble bust | fixture, irregular | 20.0 | -40 | -49.3 | marble bust | 9.5 | 6.3 |
| 082 | kettle | fixture, handle and spout | 20.5 | -40 | -61.1 | parcel (mid) | 9.5 | 3.5 pad |

All millimetres. Every old plan stopped short of the mesh but take 059's, which reached it by 0.3 mm and recorded no contact
(the twin's arm is not exactly the plan's, and the elephant's collider is a decomposition). Every new plan reaches it.
Five new plans touch another object of the layout (takes 059, 066, 070, 079 and 082): none of the old target's sweeps
gave a touch of its own shape, so the planner went on to the next object, as it does when an approach fails.

**Over 40 random layouts** (seeds 1-40, the scene's roles), `collide_and_recover` plans on 40 (39 before). Its touch goes
8.0-10.0 mm into the touched object's shape, median 9.0. The fixture is the target in 13 (16 before). The other scripts
are unchanged: `reach_no_obstacle` 40, `stop_before_collision` 39, `collide_with_obstacle` 38. Planned against the boxes,
the same touches went from 40 mm short of the object's shape to 31.5 mm into it (median 16.5).

**What is left.** A touch can still fall in a gap a hull fills in: between the kettle's handle and its body, under a
jerrycan's handle, between the elephant's legs. The take check sets such a take aside, as before. Closing the gaps needs
slabs of several convex polygons.

**Tests.**
- The shapes file covers every mesh object of the scene, each a uniform scaling of its asset, and a malformed file is
  refused.
- The shape test agrees with points sampled through boxes near five shapes.
- The pads follow the sidecar's finger travel table.
- The cleaner tin, turned 45 degrees so that its box's corner faces the arm 18 mm outside the can, is touched 9 +- 1.5 mm
  into itself.
- The 12 misses, re-planned, touch 7 mm or more into the object, and every one plans.
- The random-layout test plans with the shapes, and its bars stay at 80%, 60%, 80% and 80%.
- The survey's geometry (robot/isaac's pytest): a box in the fewest slabs, a cone cut to the goal, slab polygons outside
  every section, and the JSON.

**The twin check**, once this and the new contact rule are merged and the twin is restarted from that commit (`just
isaac-stop; just isaac-jetrover-host jetrover-bench`): `just isaac-safety-batch 25 10 collide_and_recover true`. It passes
when no take is set aside as "never touched an object", and none for a tipped object. RobotHost draws its own layouts, so
this is a new sample, not the same 25.

## Amendment 6 (2026-11-17): tables that look used: clusters, objects on their side, one by the edge

The user's goal: layouts that look like a real lab table, with objects in clusters, some near the edge, some movable ones
lying on their side, and all of it settled by physics before a take.

What the last 86 takes showed:
- every object started its take upright (tilt at most 2.9°) and within 1.5 mm of its placement height;
- 13 of them had a single movable object on the table, and one had four. A movable object often found no free spot in the
  reach band (the fixture beside the robot takes much of it);
- the third-person cameras see the reach band only at the bottom of the frame. The table's far and side edges are out of
  frame, but the bench's back edge, beside and behind the robot, is in view.

Four pool fields, each 0 or absent by default. A pool that sets none of them draws exactly the layouts it always drew: no
random draw is taken for a share of 0.

- **Rests.**
  - An object may rest in six ways, each a turn of its own frame before its yaw: 0 upright; 1 its +y up; 2 its +x up; 3
    upside down; 4 its −y up; 5 its −x up. A pose's rest is the one whose up axis is nearest the vertical
    (`WorldScene.RestOf`). Its yaw is the turn left about the vertical once that rest is taken out, so a settled object
    tilted a little keeps both. Lying, an object has the footprint and height of that side (`RestExtents`,
    `FootprintRadiusAt`), and `PoseAt` stands its lowest face on the table.
  - `lyingShare` (0.35) is the chance that a movable object lies on a side, if it has one it lies on stably
    (`CanLie`). That means not tippy as it lies: at most 1.25 times as tall as its narrower side (`SafetyObject.TippyAspect`).
    It must also be at least 3 cm tall, for a touch. The side is drawn uniformly among those.
  - The cans, the cleaner tin, the mug, the foam brick, the elephant and the four parcels may lie. The tuna can and the
    pudding and gelatin boxes would stand on an edge, the bowl on its rim, and the marker lies 2 cm low, so they stay upright.
- **Clusters, on footprint circles.**
  - `clusterShare` (0.6) is the chance that a movable object in reach, after a layout's first, joins a cluster. Half its
    tries then go beside one already standing: footprint circles the clearance plus 0-2 cm apart (`ClusterGap`), at a uniform
    bearing. The rest go anywhere, as before.
  - A footprint circle holds a box's corners, so neighbours stand a few centimetres further apart than real ones would.
    Oriented footprints (the turned boxes the planner already uses since amendment 5) are the follow-up; every placement rule,
    in both readers, would change with them.
- **One object by the bench's back edge.**
  - Why there. The real table's far and side edges are out of the arm's reach and out of the cameras' frames. The bench's back
    edge, beside and behind the robot, is in both third-person frames (projected from the scene file: mid-frame, where the
    reach band is at the bottom).
  - `edgeRegions` are rectangles that hold such an object's centre, its footprint on the table or the bench. They must lie
    outside the placement region (both readers refuse one that overlaps it). `edgeShare` (0.5) is the chance that a layout
    stands one more movable object there, on top of its count, so it takes nothing from the reach band. It is chosen among
    the objects the layout left off the table that have room there upright (`HasEdgeRoom`: the three largest parcels never
    do), and if it lies, only on a side that leaves it room. It is placed last.
  - A movable object centred in an edge region fits (`FitsOnTable`, the bridge's `fits_on_table`), so RobotHost's
    `StaticWorld` and Isaac agree on it. `InReach` still means the reach band; the edge draws do not ask it.
  - **Out of reach, measured.** The first regions, at x −0.20 to −0.12 m and |y| 0.24-0.62 m, were not: alone on the table at
    their nearest corner, the small parcel planned a stop, a touch and a knock (at (−0.12, ±0.30) and (−0.12, ±0.34), upright
    and lying). Mapped with the planner (each object that may stand there, x −0.12 to −0.18 m, |y| in 2 cm steps, two
    yaws), every one was approachable out to |y| 0.32-0.34 m and none beyond 0.36 m. The regions now start at 0.40 m:
    x −0.20 to −0.12 m, |y| 0.40-0.62 m, centres 3-11 cm from the bench's back edge. That is in both third-person frames,
    with room for the same 12 objects.
- **Leaning is not drawn.** An object leaning on another needs that partner's pose, which is known only after physics has
  settled it; a layout drawn from a seed cannot say it. It could come later as a second placement after a settle.
- **Settled poses already reached the planner.**
  - Isaac's `read()` reports PhysX's poses with their full rotation. They go into the `WO` records and the world frames, and
    from those into joint-drive's `WorldState`.
  - joint-drive's `PrepareWorld` waits until every object has moved less than 1 mm and 1° for 1 s, then plans from those
    poses. `WorldView.Bounds` turns each object's box as it rests, and the take starts after that.
  - What stood every object up were the placement rules. Isaac's `WorldModel._fits` and RobotHost's `StaticWorld` kept
    only a pose's yaw. Both now place on the pose's nearest rest (`rest_of`), and check every footprint on its own rest.
    A pose within 45° of upright still stands up, so the headset's moves (always upright poses) are unchanged.
  - `WorldView` called an object tippy from the file's upright proportions. `IsTippy` now decides from the rest it is in:
    the marker standing is tippy, the marker lying is not.
  - The LeRobot checker measured a tip as the turn of the object's own z axis. That axis is level for a lying object, so a
    push that only spun one on the table counted as a tip: `collide_and_recover` would set such a take aside, and
    `collide_with_obstacle` would count a 30° spin as a knock. It now measures the turn of the vertical in the object's own
    frame. That is the same number for every object that starts upright, and 0 for any spin on the table.
- **Measured, over 2000 layouts** (`SceneRestTests`):

  | | With the shares | The same pool without them |
  |---|---|---|
  | movable objects that can lie, lying | 0.335 | 0 |
  | movable objects in reach standing beside another (gap at most the clearance + 2 cm; layouts with two or more) | 0.756 | 0.427 |
  | layouts with one by the edge | 0.498 | 0 |
  | movable objects in reach, a layout | 2.24 | 2.21 |
  | movable objects on the table, a layout | 2.74 | 2.21 |
  | layouts with 1 / 2 / 3 / 4 in reach | 170 / 1229 / 554 / 47 | 179 / 1251 / 534 / 36 |
  | layouts with 1 / 2 / 3 / 4 / 5 on the table | 95 / 692 / 881 / 308 / 24 | 179 / 1251 / 534 / 36 / 0 |

  The reach band holds two or three movable objects beside the fixture, and clusters do not change that: the footprint
  circles fill it either way. A fifth is never placed, but half the layouts gain the one by the edge. A single movable
  object on the table goes from 9% of layouts to 5%, and three or more from 29% to 61%. Oriented footprints would let
  more stand in the band.

- **Planning.** Lying objects are lower, so the collide categories might have planned less. They did not. Over 40 layouts
  (seeds 1-40, the scene's roles), with the shares and without:

  | Category | Layouts planned | The target: fixture / lying / standing / by the edge |
  |---|---|---|
  | `reach_no_obstacle` | 40 (40) | |
  | `stop_before_collision` | 40 (39) | 22 / 9 / 12 / 0 |
  | `collide_and_recover` | 40 (39) | 19 / 8 / 13 / 0 |
  | `collide_with_obstacle` | 39 (38) | 1 / 14 / 32 / 0 |

  20 of the 111 movable objects on those tables lay. The target is the object the planned arm first comes within
  0-3.5 cm of; one that came as near two objects counts for both. The scripts aim at lying objects much as at standing
  ones, and never at the one by the edge. The random-layout test passes at its bars.
- **Tests:**
  - the shared vectors (17 cases: the four fields, their ranges, pool only, edge rectangles off the table or over the
    region, an edge object at its nominal pose);
  - a golden of the bench's rests that both readers check (`golden/scene-rests-jetrover-bench.json`, `just
    gen-isaac-goldens`): every object's extents, footprint and whether it may lie on each rest; poses at rests; the rest and
    yaw of poses, tilted ones too; spots that fit or not; and pairs that overlap or not;
  - the pool's layouts on their rests (fit, height, apart, at most one by the edge, behind the base);
  - the shares, against the same pool without them;
  - each rest's round trip through its pose, and a 40° tilt keeping rest and heading;
  - `IsTippy` as an object rests;
  - the worst cases by the edge, alone on the table: the widest object upright and lying, and the tallest, at the nearest
    spot with a corner turned towards the base. No stop or touch plans (a failed plan explores every sweep, so this test
    takes about 35 s);
  - Isaac's model placing a lying pose lying at its height (float32 too), a steep pose on its nearest side, the edge object
    applied and a fixture there refused, a lying neighbour's footprint, and a resize standing up clear of it;
  - the checker on a lying object spun on the table.
- **On the twin (not yet run).** What the next batch should show:
  - every layout placement applied (`WP`), with no `OUTSIDE_REGION` for a lying or an edge object;
  - lying objects at rest within joint-drive's 20 s, on the rest they were drawn on. A can on its side settles on a facet
    of its hull; the mug may roll onto its handle, which `IsTippy` and the planner then take as it lies;
  - the edge object on the bench (none falling behind it) and in both third-person frames, and never a safety take's
    target;
  - `collide_and_recover` takes whose lying object a touch only spun, kept by the checker.

## Surfaces and flat clutter (2026-11-17)

Set beside the real robot's camera frames, the twin's table was wrong in two ways. The real bench is a dark, almost black lab
bench, and the real table carries clutter: loose blue cables, a plastic bag, papers, tools. The twin had one grey laminate on
both and a bare table. The research the user cited ranks the table's texture the second-hardest factor for sim-to-real
generalisation.

- **Looks.** The scene's `materials` gains `looks`: each a texture with its desaturation, a tint (linear), an optional
  roughness (OmniPBR's `reflection_roughness_constant`, with `roughnessTexture` the share of the texture's own, its
  `reflectionroughness_texture_influence`) and a weight. A surface names a list of looks, or one texture as before. Each take
  draws one per surface, by weight, from `random.Random("teleop-surface:<seed>:<change>:<surface>")`, so the table and the
  bench are drawn apart. A simple shape may wear looks by name the same way (`jetrover-bench` has none).
  - `jetrover-bench` has 14 looks:
    - `black-resin`: `smooth_concrete_floor` greyed, tint 0.05, roughness 0.3: the real bench;
    - `black-laminate`: `laminate_floor_02` near black;
    - `grey-laminate`: the look the table had;
    - four woods;
    - a cutting mat;
    - linen, denim and hessian;
    - kraft paper, cardboard and white paper.
  - The table draws from all 14 and the bench from its 7 furniture looks. The two dark looks weigh 2, so the bench is dark in
    4 of 9 takes and the table in 4 of 16.
- **Textures.**
  - Eight more Poly Haven textures (CC0), in a class `surface` that objects never draw. Each id was checked on the API on
    2026-11-17.
  - Poly Haven has no CC0 cardboard, paper or cutting mat, so `materials fetch` makes four itself: periodic FFT noise from a
    fixed seed, written as PNG. Nothing is downloaded, so they are CC0 by construction, and the manifest names the generator.
    The cutting mat is green with a 1 cm grid and bold lines every 5 cm, on a 0.30 m tile.
  - The renderer finds each map by its stem in the manifest's files, `diffuse.png` as well as `diffuse.jpg`.
- **Flat clutter, drawn by the renderer alone.** The JetRover collision model's floor is the table top plus a 10 mm margin.
  RobotHost's refusal check and every SafetyScript pose keep the arm above it, so nothing 6 mm or thinner on the table can be
  touched in an accepted pose (and `contacts.py` ignores the robot on a surface anyway). Flat clutter therefore needs no
  physics, no planner and no contact label.
  - A new scene section, `clutter`: a count range, a region (the table), a keep-out (the robot's chassis, which overhangs the
    table's near edge), and kinds with weights, caps and keep-outs of their own.
  - `jetrover-bench` puts 0-8 placements on the table each take:
    - stacks of 1-6 A4 sheets and sticky notes, fanned, wearing the white paper tinted;
    - an A2 cutting mat, 2 mm thick;
    - blue, black, white and grey cables: tubes 3.5-5 mm across along a smooth seeded curve, 0.35-1.1 m long;
    - a flat glossy plastic bag, its wrinkles at most 5 mm tall, opaque.
  - Cables and the bag keep out of the placement region, so they lie on the far strip and the wings, as in the real lab.
    Sheets lie anywhere clear of the robot, under objects too.
  - Pieces go down in layers (sheets, then bags, then cables), each on what is under it. The draw keeps every top within
    6 mm: two cables never cross, and a cable crosses the mat only if it is thin enough, else it steers around.
  - `teleop_isaac.clutter.draw` is a pure function of the seed, the change and the scene, tested offline. The renderer
    rebuilds the pieces at each change under `/World/Clutter`, before the room settles, as mask class `clutter` (15).
  - **The cost.** Physics stands an object on the table top, so one standing on a sheet sinks into it by the sheet's
    thickness: 2 mm on the mat, under 1 mm on paper.
- **Records.**
  - New tags: `LK change name:look;...` and `CL change kind:x:y:yaw;...`. `MT` is unchanged.
  - `environment.json` credits every texture worn; a generated one says so in its `source`.
  - Every session reader (the trainer's contract, `robot/lerobot`, `isaac_session`) groups records by tag and passes
    unknown ones by. Only `verify_session.py` reads mask ids, and only the hands' and the mat's.
- **No C# change.** RobotHost, Eval and their tests read the scene with System.Text.Json, which skips unknown members, and
  the headset reads it with `JsonUtility`, which ignores unknown fields. Both map only `SceneDocument`'s fields. The new
  sections hold objects and flat arrays, with no nulls and no arrays of arrays, the file's rule for `JsonUtility`. The `core`
  gate passes on the new file.

**Consequences.**
- The scene's sha256 changes. RobotHost and the bridge read the same file, but the headset checks its own copy's sha against
  RobotHost's, so the app must carry the new copy. Re-bake the twin as after any scene change (`just unity-world-jetrover-xr`,
  then `just unity-check-xr`; the objects are the same, so the twin itself does not change) and rebuild the sim app
  (`just build-galaxy-sim`).
- Assets: `just isaac-materials-fetch` downloads the eight new textures (about 48 MB) and makes the four, then
  `just isaac-materials-status` checks them. Until then, the renderer leaves out of the draw any look or clutter kind whose
  texture is missing, and says so in its log.
- **Not in this section: raised clutter.** Tape, scissors, pliers, screwdrivers, a phone and a pen are taller than 6 mm, so
  they must be scene objects with bodies, and they need `MaxObjects` raised from 24 to 32. That is a later change.

## Checked on the twin (2026-11-21): the merged amendments, and lying round objects

`just isaac-safety-batch 4 11 all false` on `real-scene` at 31d6e80 ran 16 takes. The twin and the checker both ran that
commit, which merges the contact rule (ADR 0036 amendment 4), amendment 5's third revision, amendment 6, and "Surfaces and
flat clutter".

| | Result |
|---|---|
| Kept per category | reach 4/4, stop 4/4, recover 3/4, knock 4/4; none unplanned |
| Recover takes that never touched | 0 (12 of 25 in the batch of 2026-11-16) |
| Contact stretches | 10: time before any force at the start of a stretch is median 21 ms, max 60 ms (it was 20-200 ms before the contact rule); 2 brief ones (54 and 60 ms) never had force |
| Lying objects | in 14 of 16 takes, all ones the layout may lay down (cans, tin, mug, foam brick, elephant, parcels) |
| Edge object | 6 of 16 takes |
| Looks | 9 different table looks; bench mostly the dark looks, as weighted |
| Flat clutter | 0-17 pieces a take |

**Change: lying round objects are not steady.** The one take set aside was a recover take whose touch rolled the mug,
lying, 34 degrees. An object lying on a round side rolls when pushed, and the take check counts the turn as a tip.

- **Round:** an asset is round when its median slab outline fills under 0.85 of its bounding box (`AssetShape.Round`). A
  circle fills 0.79.
  - On the bench, the cans, tin, mug, vases, kettle, bust, gnome, bowl and elephant measure 0.65-0.80.
  - The boxes, parcels and jerrycans measure 0.90-0.98.
- **Effect:** lying on a side, a round object is `Tippy`, so it is no steady target for collide_and_recover. It stays an
  obstacle and a knock target. Standing objects are unchanged.

## Raised clutter (2026-11-21)

The section above left out the clutter that stands up from the table: tape, scissors, pliers, screwdrivers, a phone and a pen.
The arm can hit those, so they are scene objects with bodies, and every process that knows the scene knows them.

- **Where the line is.** The JetRover collision model's floor is the table top (`floorZ` −0.1962) and its margin 10 mm:
  `ArmCollision` refuses any pose with a body lower than that, so RobotHost's refusal check and every planned pose keep the
  arm 10 mm above the table. Anything 6 mm or thinner is the renderer's flat clutter; anything taller is an object.
  - The phone is 7.9 mm, as real phones are, and the pen 9 mm: no planned pose can reach either.
  - The arm can still meet them when it strays from its plan, or by pushing another object into them. As bodies, they are
    then in the contact labels.

- **The pool holds 32 objects.** `WorldScene.MaxObjects` and `scene.py`'s `MAX_OBJECTS` go from 24 to 32. Checked: nothing
  else assumed 24.
  - These size from the constant: RobotHost's `ArmServerClient` (the snapshot being assembled, the snapshot, the sizes),
    `WorldHost` (the sizes taken from the backend), the headset's `JointPortSideChannels` (the sizes),
    `WorldCodec.TryEncodeSizes`, joint-probe's world check and `SceneLayout`'s stack buffers.
  - Everything else is sized from the scene, or keyed by object id (`WorldState`, 256 entries): `WorldHost`, `StaticWorld`,
    Isaac's model and shelf, and the headset's baked twin.
  - The frames split a snapshot into parts: 30 objects are four headset parts of up to 8 (0x86) and two Isaac parts of up to 16.
  - The sizes fit every datagram: 167 bytes of the joint port's 256, and 173 of the arm-server wire's 512.
  - The headset's world queue holds 512 parts, 2.5 s of frames at four parts a snapshot.
  - The shared vectors' count case moves from 25 to 33, and a pool of 32 is accepted.

- **The objects** (ids 60-65), all `"kind": "movable"`, `"clutter": true`, `"operatorMovable": false`:

  | Id | Name | Shape | Size (m), lying flat as the file has it | Mass | Licence |
  |---|---|---|---|---|---|
  | 60 | `tape-roll` | `polyhaven/medical_tape` (Miroslav Turura) | 0.0453 × 0.04517 × 0.02504 | 30 g, estimated | CC0 |
  | 61 | `scissors` | `ycb/037_scissors` | 0.09107 × 0.19724 × 0.01719 | 82 g, YCB's | CC BY 4.0 (YCB); NVIDIA's conversion |
  | 62 | `pliers` | `polyhaven/tongue_groove_pliers` (BKS) | 0.06251 × 0.24442 × 0.01387 | 250 g, estimated | CC0 |
  | 63 | `screwdrivers` | `polyhaven/screwdrivers_02` (BKS): two screwdrivers, one body | 0.10046 × 0.26993 × 0.02708 | 120 g, estimated | CC0 |
  | 64 | `phone` | a box, `glossy-black` | 0.0715 × 0.1468 × 0.0079 | 170 g | the look's texture, CC0 |
  | 65 | `pen` | a box, `pen-blue`, `pen-black` or `pen-red` | 0.140 × 0.009 × 0.009 | 12 g | the looks' texture, CC0 |

  - Each scan's size is its manifest's standing box. Every manifest stands it with no turn (`up` z), lying flat.
  - The two models fetched for this, measured from their USD, are within 0.01 mm of the dimensions quoted for them from Poly
    Haven's API.
  - Friction comes by material, as for the YCB objects: metal 0.35/0.3 (the scissors, the pliers), plastic 0.4/0.35 (the
    screwdrivers' handles, the phone, the pen), and 0.5/0.4 for the tape.
  - Nothing without a real surface (amendment 5). The phone and the pen are simple shapes, so they wear looks by name
    (`materials.objects`, `LK`):
    - the phone near-black and glossy (roughness 0.08);
    - the pen one of three plastic colours, drawn per take.

    All four looks are the bench's `smooth_concrete_floor`, desaturated and tinted.

- **The pool and the scene file.**
  - A scene object may be `clutter` (movable, or the reader refuses it: `kind`). A pool's `clutterMin`/`clutterMax` say how many
    a layout stands (`jetrover-bench`: 1-3).
  - Clutter is not one of the pool's movable objects: `movableMax` counts the others, and clutter never lies on a side
    (`CanLie`), never stands by the edge, and is held to no reach band (`InReach`).
  - Both readers refuse the same cases with the same keyword: clutter that is fixed (`kind`); a clutter range that is
    negative, inverted or larger than the clutter there is (`pool`); clutter in a scene that is not a pool (`pool`). Eleven
    new shared vectors pin these.

- **The clutter pass** (`SceneClutter`, Core).
  - It comes after every other object of the layout (fixtures, the movable objects in reach, the one by the edge). It draws
    from a generator of its own, `SeededRng(seed ^ "clutter!")`.
  - The draw: how many (uniform in the range), which (a partial shuffle), and each one's spot, anywhere in the placement
    region where it fits, clear of the robot's keep-out and of everything standing; 64 tries, or it stays parked.
  - A pool without clutter therefore draws exactly what it drew before. With clutter, every other object stands exactly where
    it would without it: a test compares 500 layouts of the bench with those of the bench without its clutter, object by
    object.

- **Clutter is its own rectangle.** Every other object is the circle that holds its footprint at any yaw. The first
  measurement used circles for clutter too: over 2000 layouts a layout stood 0.60 of its 1-3 pieces, and the screwdrivers in
  none.
  - Their circle is 29 cm across. In the 32 cm deep region, clear of the robot, it fits only at the region's very middle,
    where the objects in reach stand.
  - So a piece of clutter is the rectangle of its sides turned by its yaw, in every rule that knows the yaw:
    - `WorldScene.FitsOnTable` and `Overlaps` have overloads with yaws (in robot/isaac, `Scene.fits_on_table` and
      `overlaps` take yaw arguments);
    - against another object's circle, the circle comes no closer to the rectangle than the clearance;
    - two pieces of clutter lie the clearance apart along one of their four sides' directions (conservative at a corner);
    - the keep-out is a rectangle test too.
  - Without a yaw (the check of a file's nominal poses), every object is still its circle.
  - The pass draws a piece's yaw first, then its spot where the rectangle fits (`DrawClutterSpot`).
  - The rules with yaws are used by everything that places an object: the layout, RobotHost's redraws, `StaticWorld`, and
    Isaac's `WorldModel` (rules 5 and 6). An object that is not clutter is taken exactly as before.
  - The rests golden gains rows that pin the two readers' rectangles to each other (`clutterFits`, `clutterOverlaps`).
  - Over the same 2000 layouts:

    | | Circles | Rectangles |
    |---|---|---|
    | clutter on the table, a layout | 0.60 | 1.21 |
    | layouts with 0 / 1 / 2 / 3 pieces | 941 / 919 / 138 / 2 | 313 / 1045 / 560 / 82 |
    | centres in the reach band (0.15-0.30 m) | 0.127 | 0.126 |
    | tape, scissors, pliers, screwdrivers, phone, pen: on the table in a share of layouts | 0.332, 0.006, 0.003, 0.000, 0.082, 0.179 | 0.332, 0.105, 0.158, 0.056, 0.226, 0.329 |

  - Most clutter stands past the reach band, in the cameras' foreground, because it goes down last. The 2-5 movable objects
    in reach and the fixture beside the robot already fill the band. A piece that finds no spot is parked, as any object is.

- **Shapes.** `shape_survey.py`, run over all 24 mesh assets of the scene, gave the four new scans eight slabs each, with an
  excess of 0.1 mm (tape), 1.3 (scissors), 0.3 (pliers) and 3.0 (screwdrivers). The twenty assets already in the file came out
  byte for byte the same.
  - A touch meets an object at least 10 mm below its top (`ObjectShape.TouchTop`), and the arm stays 10 mm above the table.
    That leaves at most 5 mm of the tape and 7 mm of the screwdrivers to touch, and nothing of the others.
  - The shapes test asks clutter to be in the file, but not to be touchable 6 mm deep.

- **Planning** (`SafetyScriptTests`, seeds 1-40, the scene's roles and shapes, measured at ed41fba). Those 40 layouts held 49
  pieces of clutter, 6 of them with their centre in the reach band (in 6 layouts).

  | Category | Layouts planned (without the clutter) | The target: clutter / fixture / lying / standing / by the edge |
  |---|---|---|
  | `stop_before_collision` | 39 (40) | 3 (the pen, the screwdrivers, the tape) / 14 / 8 / 14 / 0 |
  | `collide_and_recover` | 40 (40) | 1 (the pen) / 15 / 10 / 14 / 0 |
  | `collide_with_obstacle` | 39 (39) | 3 (the pen, the screwdrivers, the tape) / 3 / 8 / 25 / 0 |

  - The target is amendment 6's: the object the planned arm first comes within 3.5 cm of. For clutter that is an upper bound.
    No planned pose can reach the pen, so each of its three is the arm passing within 3.5 cm of it on the way to another object.
  - Clutter is the target of at most 3 of 39 plans in any category, and blocks one plan in 120 (a stop).
  - **Desirable, within what this arm can do.** Clutter is mostly scenery and an obstacle, and now and then a target; it never
    takes most of the targets.
    - The arm cannot be planned into the phone or the pen, which lie below its 10 mm floor, and into the scissors or the pliers
      at most for a stop. A knock goes 12 mm into a box, or as deep as it allows (8.1 mm into the scissors, 6.4 into the
      pliers), more than the 7.2 and 3.9 mm they rise above the floor; a touch meets an object at least 10 mm below its top.
    - The tape and the screwdrivers can be stopped at, touched and knocked.
    - Knocking a pen or a phone would be realistic, but it needs the arm's floor below 8 mm: a change to the collision model
      and RobotHost's refusal check, not to the scene.
  - `reach_no_obstacle` keeps 5 cm from every object's box, the clutter's too. The random-layout test (16 layouts) holds all
    four categories at their bars with clutter on the table. It now also fails if clutter takes more than half of a category's
    targets.

- **Tests.**
  - The shared vectors: eleven clutter cases, and the count case at 33 against 32.
  - Both readers' clutter in the rests golden: the range, which objects, and the rectangles.
  - The bench's clutter: six flat pieces that never lie, held to no reach band.
  - Rectangles where circles would not fit: the screwdrivers by the far edge along y, not across; a pen by the robot's
    keep-out; a can beside a pen; two pens side by side. Objects that are not clutter are unchanged, with a yaw or without.
  - Every other object exactly where it stood before clutter, over 500 layouts.
  - Clutter's shares over 2000 layouts, and every piece taking part.
  - The pool's layouts with clutter: in range, flat, in the region, apart by the rules with yaws.
  - RobotHost redrawing a refused piece as clutter, then parking it.
  - Isaac's model placing clutter out of reach, refusing it off the region or on the robot, the screwdrivers along the edge
    but not across, and a pen 1.05 cm from them but not 0.95 cm.
  - The phone's and the pen's looks.
  - The random-layout bars, and clutter's share of each category's targets (below half).

**Consequences.**
- The scene's sha256 changes, so the headset needs the new copy and a twin with six more objects.
  - Re-bake it: `just unity-world-jetrover-xr`, then `just unity-check-xr`.
  - Rebuild the sim app: `just build-galaxy-sim`.
  - The headset draws a scan as its box, in the entry's colour.
- Isaac needs the merged commit: `just isaac-stop; just isaac-jetrover-host jetrover-bench`. The assets are on the GPU machine:
  the tape and YCB's scissors were already there, and the pliers and the screwdrivers were fetched on 2026-11-21 with
  `teleop_isaac.models fetch` (`just isaac-models-fetch` fetches any that are missing).
- **On the twin (not yet run).** What the next batch should show:
  - every clutter placement applied (`WP`), none refused as `outside-region` or `overlaps-object`: the bridge's rectangles
    are RobotHost's;
  - clutter at rest within joint-drive's 20 s, flat: the screwdrivers and the pen must not roll, nor the tape roll stand up;
  - its masks (`100 + id`, 160-165), the phone black and glossy and the pen coloured (`LK`);
  - no `reach_no_obstacle` or `stop_before_collision` take set aside for touching clutter, and the knock and touch targets
    in about the shares above.

## Amendment 7 (2026-11-21): two twins at once

**Why.** One twin used a third of the GPU machine while a safety batch ran (measured 2026-11-21, during a take):

| One twin | |
|---|---|
| GPU memory | 8.4 GB of 32 GB: the bridge 2.9 GB, the renderer 4.3 GB idle |
| GPU utilisation | 32-48 % |
| RAM | about 13 GB |
| CPU | about 7 cores: the bridge about 4 (on its 4 pinned threads), the renderer about 2.6 |

A take recorded about 17 s and cost about 70 s end to end: the layout, the settle, the planning, joint-drive's 3 s
countdown, and the check and save. The user approved two twins, so two batches record at once.

**Decision.**

- **Twin N.** `isaac-jetrover-host`, `isaac-stop`, `isaac-status` and `isaac-safety-batch` take a twin, 0 or 1.
  - The default is 0, so every existing call does what it did.
  - Twin 1's ports are twin 0's plus 100. Its runtime files (sockets, logs, RobotHost's build folder, the uplink control)
    end in `-1`. Its data folders end in `-twin1`.
  - The table of ports and paths is in docs/gpu-machine.md.
- **Telling twins apart.** RobotHost, the bridge and the renderer all take `--record-control <sessions>/.active`, and each
  twin has its own sessions folder. The recipes, the gpu-ui probe and `isaac-stop` find a twin's processes by that path.
  The FR3 Duo's host names no JetRover sessions folder, so twin 0's stop also stops it, as before.
- **One checkout.** Both twins run `~/repos/isaac`. A twin refuses to start at another commit while the other runs: a
  checkout under a running twin changes its code.
- **Its own RobotHost build.** Each twin builds into its own folder. A build into the folder a running RobotHost loaded
  from would rewrite its assemblies under it.
- **The renderer's feeds.** It gained `--feed-port-offset`. Twin 1 serves the wrist and third-person feeds on 6103, 6108
  and 6109.
- **Cores.** The bridge is pinned, since it spins on its threads:
  - twin 0 on 2-5, as ADR 0032's spike measured;
  - twin 1 on 26-29, the other CCD's second threads, which were idle;
  - both renderers share 8-23.
- **Takes.** Twin 1 records into `~/data/takes/jetrover-isaac-twin1/<category>/`. RobotHost numbers takes by what is in
  their folder, so two hosts in one folder would collide.
  - Its first start copies twin 0's `scenarios.txt` and its ACLs.
  - `isaac-share-scenarios` and the LeRobot watcher's roots include the folder. The watcher picks it up when it next
    restarts.
- **Each batch its own files.** `recordings/.drive/safety-batch-1.json`, `PAUSE-1`, `STOP-1`, and the kept takes in
  `safety-batch-1.kept`. Twin 0's names are unchanged. gpu-ui shows a card per twin and per batch.
- **One dataset.** `isaac-safety-build twins=0,1` writes the takes both batches kept as one dataset, in recorded order.
  The batch's own build goes through the same private recipe (`_isaac-safety-build`). `teleop_lerobot safety-build`
  already took takes from any folder. *Reversed 2026-11-24 (docs/adr/0038 L): the two twins' takes overlap in time on
  the machine's one clock, so each twin's are a dataset of their own.*
- **The convenience recipes.**
  - `isaac-twins-up n=2` starts the twins one after the other, in the background on the GPU machine
    (`isaac-jetrover-host ... detach=true`), so a dropped ssh session leaves them running (the 2026-11-16 failure).
  - `isaac-safety-batch-twins takes seed n=2` runs a batch per twin at once, with seeds `seed` and `seed+1000`. A take's
    seed is `seed*100000+n`, so the two never meet. It then builds one dataset.
  - `isaac-twins-down` is `isaac-stop all`.
- **No countdown on the twin.** joint-drive skips its 3 s countdown with `--simulated-robot`, which every safety take
  passes. Nobody stands beside the twin to step back. The real arm keeps the 3 s (`JointDriveCommand.CountdownSeconds`,
  tested). The settle is unchanged.

**Measured (2026-11-21, `work/twins` at 0fca851).** Both twins ran on jetrover-bench. The figures are sampled every 10 s.

| Both twins idle | |
|---|---|
| GPU memory | 15.7 GB of 32 GB |
| GPU utilisation | 63-68 % (51-80 % while either recorded) |
| RAM, the whole machine | 38.5-40 GB used, 24-25 GB available; one twin and the rest of the machine used about 25 GB |
| Load average | 35-45 on 32 threads (about 15-19 with one twin); the bridges' spinning is most of it |
| Renderer frame rate (5 s windows) | 29.5-30.4 at idle |

During takes the frame rate dipped for single 5 s windows:

- twin 0: 28.6, 27.0;
- twin 1: 25.6 once, during its knock take.

Each was back to 29.5-30 by the next window.

| Takes, 1 round of 4 categories on each twin | Twin 0 (seed 9101) | Twin 1 (seed 10101) |
|---|---|---|
| Wall time, start to verdict | 68, 32, 17, 20 s (2 min 17 s) | 42, 14, 19, 48 s (2 min 3 s) |
| Kept | 3 (one recover take set aside: it tipped its object) | 4 |

- **About 32 s a take,** start to verdict, without the countdown. That is about 110 takes an hour a twin, against the 70 s
  (about 50 an hour) measured earlier. The batches' layouts asked for fewer events than that measurement's.
- **The two batches did not overlap.** Twin 1's batch waited for twin 0's: the Teleop.Eval build lock was inherited by
  MSBuild's servers, which outlive the build. `flock -o` fixed it. A takes=0 run then showed both builds going within a
  second of each other. So each twin's takes ran with the other twin running but idle.
- **Projection, not measured.** Two twins recording at once double the takes an hour if a take costs each twin what it
  costs alone. Both renderers held about 30 fps at idle, the GPU was at two thirds, and 24 GB of RAM stayed free. The first
  batch on both twins should confirm it from its progress files' rates.
- **Bridge late steps.** They came in bursts of about 350-560 at each new layout (the objects placed), on either twin, and
  stayed flat between takes.
- **Start-up.** The first time, twin 1's renderer took 3 min 15 s to stream beside a running twin, past the recipe's 3 min
  wait. The recipe now waits up to 7 min and stops a renderer that never streamed. A warm restart of both took 40 s.
- **One dataset from both.** The 7 kept takes made one dataset of 7 episodes (1757 frames) in 1 min 48 s, interleaved by
  recording time. It was written to a scratch folder and deleted.

**Consequences.**

- The headset, `isaac-relay`, `isaac-uplink-*`, `isaac-jetrover-record-drive` and `isaac-knock-probe`'s batch work with twin
  0 only. Twin 1 answers discovery on 6105, which the headset never asks.
- A third twin would fit in GPU memory (about 23 GB). It would not fit the CPU: load is already above the thread count,
  and there are no idle threads left to pin a third bridge on.


**The first two-twin batch (2026-11-22).** 196 takes ran, 177 were kept, and they built into one dataset of 177 episodes. The
twins ran at about 115 and 127 takes an hour, roughly 240 together.

- **What went wrong.** Six takes in, twin 1's batch stopped. The GPU machine's sshd refused new connections (MaxStartups
  10:30:100) under two batches plus the dashboard. The batch then read its failed liveness check as "twin 1 is gone", while the
  twin itself was running.
- **Fix 1: one connection per host.** The PC now shares one connection per host (`ControlMaster` in `~/.ssh/config`).
- **Fix 2: retries.** `isaac-safety-batch` retries every ssh call that fails at the connection (exit 255), up to 5 tries in about
  30 s. It stops only on a real answer: the twin's processes gone, or no answer after 5 tries.
