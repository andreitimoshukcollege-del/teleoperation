# ADR 0037: Varied scenes for the JetRover twin: real objects, walls and HDRI rooms

## Status

Accepted, 2026-11-15 (the user, on starting P1). P1 built 2026-11-15; P2 built 2026-11-15 (amendments 1 and 2); P3 built 2026-11-15 (amendment 3); P4 built 2026-11-15 (amendment 4). Real fixtures and materials 2026-11-16 (amendment 5).

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
