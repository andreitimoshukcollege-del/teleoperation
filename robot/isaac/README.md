# robot/isaac: the FR3 Duo in Isaac Sim

The Isaac bridge (docs/adr/0031) is the FR3 Duo with physics. It runs in Isaac Sim on the GPU machine,
behind a RobotHost there. The headset and the Eval verbs drive it exactly as they drive the simulated
host or, later, the real robot (docs/adr/0030). Its sessions say `"source": "isaac"`, and they are
never the robot's.

```
headset ──UDP joint port──> RobotHost (--backend arm-server) ──Unix datagram socket──> bridge ──> Isaac Sim (PhysX 1 kHz)
                                                                                         │ tap (copies, never blocks)
headset / lab screens <──UDP 6003 (head), 6007 (third person)── renderer (a second Kit, visual only, 30 fps) <──┘
```

## Running it

The GPU machine needs our own Isaac Sim, installed once: `just isaac-setup`, then `just isaac-check`.
Push the commit you want to run, then:

```bash
just isaac-host                       # Isaac and RobotHost on the GPU machine, with the bench scene; Enter stops both
just isaac-host fr3-duo-bench 7       # the same, layouts from seed 7 (just takes arguments by position)
just isaac-host none                  # the arms alone, no table or obstacles
just isaac-stop                       # stop them from any terminal (RobotHost closes its recording first)
just fr3-teleop-probe 0.08 3 franka-replay false <GPU machine>   # drive it headless from here
just push-galaxy-fr3-config <GPU machine>                        # point the headset at it
just fr3-world-probe <GPU machine> 7 true                        # the world through RobotHost: seed 7's poses, then NEW LAYOUT
just camera-probe 10 0 "" <GPU machine> 6003 6004                # the right wrist's feed (6007: the left wrist's; 6008, 6009: third-person left, right)
just isaac-host fr3-duo-bench auto false                         # without the renderer and its feeds
```

`isaac-host` refuses while a training run of ours, or another Isaac host of ours, is going. Other
people's GPU jobs no longer stop it (2026-11-02): it says how much of the GPU they hold, since sharing it
can make physics steps late. The bridge's log is `~/logs/isaac/bridge.log` on that machine (what lives where there:
docs/gpu-machine.md).

## Two JetRover twins at once (docs/adr/0037 amendment 7)

The JetRover's twin (`just isaac-jetrover-host`, docs/adr/0034) can run twice on the GPU machine, so two safety batches
record at once:

```bash
just isaac-twins-up 2                          # twins 0 and 1, one after the other, in the background there
just isaac-safety-batch-twins 25 1             # 25 rounds on each (seeds 1 and 1001) at once, then one dataset of both
just isaac-status                              # each twin: its processes, joint port, recording, renderer fps
just isaac-twins-down                          # both (just isaac-stop all)
```

Or one at a time: `just isaac-jetrover-host jetrover-bench auto 1` (a terminal; `... auto 1 true` in the background),
`just isaac-safety-batch 25 1001 all false 1-3 1`, `just isaac-stop 1`, and `just isaac-safety-build 0,1` for one dataset
from both batches' kept takes. Twin 1's ports are twin 0's plus 100 (joint 6102, feeds 6103, 6108, 6109; the renderer's
`--feed-port-offset 100`), and its takes go to `~/data/takes/jetrover-isaac-twin1`. What lives where:
docs/gpu-machine.md. Both twins run the same checkout, so starting one at another commit than the other refuses.

## The world (docs/adr/0031 amendment 2)

With a scene (`core/Scenes/<scene>.json`, default `fr3-duo-bench`), `world.py` puts the file's surfaces in
the stage as static colliders, its fixed obstacles as kinematic bodies and its movable ones as dynamic bodies
with their mass, which fall and which an arm can knock over (ADR 0031 amendment 4), on the session layer, so the
robot's USD is never changed. RobotHost decides where objects go, from seeded layouts, and asks with PLACE;
the bridge applies or refuses each one by RobotHost's own rules (`StaticWorld`), plus a PhysX overlap query
against the arms: `overlaps-arm`, and `arm-touching` for a move from the headset. After each Welcome it
sends WORLD_INFO, the scene file's sha256 (RobotHost runs only a world whose file is byte-identical to
its own), and every 20 ms WORLD, the objects' poses as PhysX has them.

- **The robot's fixed bodies ignore the surfaces.** The mount, both arms' `link0` and the head stand in
  the bench, and their contact with it fought the base's pin and threw both arms off their targets.
  Every moving link, the hands included, collides with the table, the bench and the obstacles.
- **Each arm joint pushes no harder than the profile's effort** (87 or 12 Nm; Isaac's import already
  had these), so an arm driven into an obstacle stalls against it, and what it measures (`QS`) falls
  short of what was commanded (`AO`).
- **Isaac does not report contacts yet** (`WC` is never written behind it).
- **A pool** (docs/adr/0037 decision 3, `jetrover-bench`): each layout puts a few of the file's objects on the table and
  sends the rest with PLACE cause `park`. The bridge stands those on its hidden shelf, the one the resize variants rest on
  (5 m below the floor, 17 m out, past every camera's far plane), and reports them with `FLAG_PARKED`; the renderer hides
  them too. A pool's objects start parked, and no rule counts a parked one as in the way. A pool with `sizeMin`/`sizeMax`
  also draws a size per resizable object per take (amendment 2), changed while it is parked.
- **Real objects** (docs/adr/0037 decision 4): `just isaac-ycb-fetch` (once) puts the 21 YCB objects NVIDIA converts in
  `~/isaacsim/assets/ycb/` (`teleop_isaac.ycb`), with each one's standing rotation and size, mass and friction. A scene
  object of shape `mesh` names one (`"asset": "ycb/025_mug"`); `world.py` references it, stood up and centred on its box,
  with a convex decomposition collider. Real objects keep their size: they get no resize bodies.
- **Fixtures and materials** (docs/adr/0037 amendment 5):
  - `just isaac-models-survey <categories>` measures Poly Haven's models and `just isaac-models-fetch` fetches the ones the
    scene names (`polyhaven/<id>`, into `~/isaacsim/assets/polyhaven/`, texture paths pointed at their own folder). The
    fixtures stand in the pool's `fixedRegions` (the table's wings).
  - `just isaac-materials-fetch` fetches the PBR textures the simple shapes wear per take and the table-top ones, and makes
    four Poly Haven lacks (a cutting mat, kraft paper, cardboard, white paper: `teleop_isaac.materials`, from a seed). Each
    shows at its real size (since 2026-11-23, docs/adr/0038 "A, appearance"): a textured box, the table and the bench become
    meshes with texture coordinates in metres (`renderer.box_mesh`), and a cylinder is projected in its own object space,
    which is metres. (A box projected in object space had been a unit cube: every side as many tiles as any other.)
- **Surfaces and flat clutter** (docs/adr/0037, its own section): the table and the bench each draw a look per take from the
  scene's `materials.looks` (`LK`), and `teleop_isaac.clutter` lays flat clutter on the table (papers, a cutting mat,
  cables, a plastic bag; `CL`, mask id 15). The renderer alone draws it: nothing is taller than 6 mm, which no accepted arm
  pose can touch, so the bridge, the planner and the contact labels never see it.
- **Raised clutter** (docs/adr/0037, its own section): desk clutter tall enough for the arm to hit (a roll of tape, scissors,
  pliers, two screwdrivers, a phone, a pen) is scene objects of `jetrover-bench` (ids 60-65, `"clutter": true`): movable
  bodies the bridge collides with and the contact labels see. RobotHost stands 1-3 a take, after every other object,
  anywhere in the placement region, each as its own rectangle at its heading, and `world.py`'s rules take it the same way.
  The tools are scans (Poly Haven's, by `just isaac-models-fetch`, and YCB's scissors); the phone and the pen are boxes that
  wear looks by name (`materials.objects`, `LK`).
- **Contacts and clearance, the JetRover twin** (docs/adr/0036 amendment 1, docs/adr/0038 decision 4): its bridge reports the
  arm's touches as `WC`, an object by its id and any surface (the table, the bench, the floor) as object 0, the table.
  `just isaac-clearance-geometry <scene>` (`teleop_isaac/clearance_geometry.py`, Isaac's Python without Kit) writes what PhysX
  collides with to `~/isaacsim/assets/clearance`: the links' boxes as the URDF importer made them, and each mesh asset as
  convex pieces (CoACD at PhysX's convex-decomposition budget). The safety build's per-frame clearance reads it
  (`teleop_lerobot.clearance`). Run it again after a scene names a new asset.

`just isaac-scene-check` (`scene_check.py`) runs the real bridge with the scene and talks to it as RobotHost
would. It checks WORLD_INFO, WORLD at 50 ± 5 Hz, placements applied and refused, and the left hand
driven into a post: it must stall more than 0.05 rad short, while the same motion without the post
tracks within 0.01 rad. It also checks `arm-touching` and `overlaps-arm` while the hand rests on the
post, and late steps of at most 1/s. On 2026-11-02 the hand stalled 0.256 rad short, the free motion
tracked exactly (0.000 rad), and there were no late steps in 16 s.

## The renderer and the feeds (docs/adr/0032 decisions 2 and 3)

`renderer.py` is a second, headless Kit process beside the bridge, on cores 8–23 while the bridge has 2–5. Its log is
`~/logs/isaac/renderer.log`. It works like this:
- **The stage.** It opens the bridge's converted robot USD with the `Physics` variant at `none`, so nothing steps.
- **The scene.** It builds the scene file's surfaces, objects and lights as visuals, through `world.py`'s
  `build(physics=False)`.
- **Posing.** The bridge copies every state and world message to it (`--tap`, never blocking; drops are counted in the
  bridge's last line). Each frame it poses the robot from the newest state (`kinematics.local_motion`) and the objects from
  the newest WORLD.
- **The cameras.** It renders the scene file's cameras at 30 fps: two third-person ones and one on each wrist (ADR 0032
  amendment 3), a wrist camera as a child of its hand's prim, so it moves with the hand. JPEG encoding runs on one thread per camera, and a
  frame is dropped rather than queued when its encoder is behind.
- **The feeds.** Each camera goes out on ADR 0014's wire (`camwire.py`, `camfeed.py`) to whoever subscribed: the right wrist
  on 6003, the left wrist on 6007 (the headset's two displays), the third-person cameras on 6008 and 6009. A frame is encoded only while someone is subscribed.
- **The capture stamp.** `captureTicks` is the sensed time of the state the frame shows, on RobotHost's CLOCK_MONOTONIC.
  The image read after posing frame k shows frame k − lag. The lag is measured at start-up with a cube on the table, with
  the objects hidden; on 2026-11-02 it was 2 frames in 100% of 87 samples.

- **The 5 s report** in its log gives the frame rate, then where each frame's time went (`tap`, `pose`, `render`, `read`,
  `frame`, and `change` when the room changed; median/longest in ms) and how often Python's garbage collector ran, with its
  longest pause. The heap as it stands once the scene is built is frozen out of later collections (`--no-gc-freeze` keeps
  it in).

If the renderer fails to start, `isaac-host` says so and runs the robot without feeds. `isaac-stop` stops it with the rest.
ADR 0032 amendment 1 holds the 2026-11-02 numbers: both feeds at 29.4–29.5 fps from the PC, no incomplete frames, and
0.05 late steps a second on the bridge.

## Backdrops and recording (docs/adr/0032 decisions 4–6)

```bash
just isaac-backdrops-fetch 2000      # once: Open Images V7 validation, CC BY 2.0 only, into ~/isaacsim/backdrops/
just isaac-backdrops-status          # how many, their licences, any whose sha256 no longer matches
just isaac-record-check              # against a running isaac-host: drive, record, check one session end to end
just isaac-edit-check                # against isaac-host fr3-duo-bench 7: headset moves applied and refused (an arm, off the table)
```

- **The backdrop.** The renderer puts a random image on a mat over the table top and another on a screen beyond the table.
  The pair is `backdrops.pick(images, seed, change)`: change 0 at start-up, then one more after each recorded session.
  - Downloads are untrusted data: a JPEG of at most 20 MB, decoded and at least 256 px a side, stored under its own sha256.
  - Without images, the table is plain and there is no screen.
- **The room** (docs/adr/0037 decision 2), for a scene with an `environment` (`jetrover-bench` since 2026-11-15, which has
  no backdrops): the dome light shows a Poly Haven indoor HDRI, a 360° photograph of a real room, so what the cameras see
  beyond the table and the light on it come from one photo.

  ```bash
  just isaac-environments-fetch      # once: every indoor HDRI (CC0, ~300, 2K, ~2 GB) into ~/isaacsim/assets/hdri/
  just isaac-environments-status     # how many, their licences, the spread of their brightness, any whose sha256 no longer matches
  ```

  - The room is `environments.pick(hdris, seed, change)`, at the same changes as the backdrop: every room once before any
    comes back, turned by a random yaw.
  - The dome's intensity starts at the scene's `meanRadiance` over the room's mean luminance (the rooms' own brightness
    spans 16x). That alone left a white studio twice as bright in the image as a dim store, so the renderer then meters
    the first still camera's image and corrects the dome toward `targetBrightness`, as a camera's auto-exposure would.
    Each other camera (the wrist's) then meters its own image and keeps that gain for the session: the dome sets the
    light, each camera its own exposure.
  - After a change the renderer re-renders the pose, unrecorded, until the image holds still (5-7 frames on the 5090),
    and again after each metering step.
  - The floor is not drawn (the room's own floor shows); it still catches a falling object in the bridge.
  - Each session records `EN` and `isaac/environment.json`, the attribution of every room shown.
  - Without rooms, the dome is plain.
- **Each take's look** (docs/adr/0038 "A, appearance"), for a scene with an `appearance` section (`teleop_isaac.appearance`;
  a scene without one renders exactly as before). At every change, before the room settles:
  - every scanned object's colour (brightness, desaturation, a slight tint), through OmniPBR's albedo inputs (YCB) or two
    MaterialX nodes in front of the base colour (Poly Haven); recorded as `OA`;
  - the light: the dome's colour temperature, the metering's target, and a key light the section adds, from a drawn
    direction, strength, size and colour (`LT`);
  - every camera moved by up to a few millimetres or centimetres and turned by up to a degree or two in its own frame
    (`CJ`); `CM` records the pose each camera then has.

  A session that opens while a change is under way waits for it, so its records describe its frames.
- **Recording.** While RobotHost's control file (`~/data/sessions/fr3_duo-isaac/.active`) names a session, the renderer writes
  into it `isaac.jlog` and `isaac/<camera>.<stream>.frames`:
  - head colour (JPEG 92), depth (16-bit PNG, mm) and masks (16-bit PNG, stable ids);
  - third-person colour;
  - `isaac/backdrops.json`, the attribution of every image shown (and `isaac/environment.json`, every room's).
  RobotHost waits up to 60 s for the trailer, checks the `FS` lines and hashes `isaac/`. The renderer records only when its
  start-up render lag agreed in ≥ 99.5% of samples, and stops below 20 GB free. The records are in
  `trainer/contract/SESSION_FORMAT.md`.
- **`isaac-record-check`** drives 40 s of seeded joint-space moves with `chain-drive --as-headset`, so that RobotHost
  records them. It waits for the session to complete, then runs `verify_session.py` on the GPU machine. ADR 0032
  amendment 2 holds the 2026-11-03 numbers: 100% of frames stamped with a `QS` state, depth p99 0.5 mm, and each hand's
  point in its mask in 100% of 61 frames.
- **Cost:** about 10 GB an hour of recording.

## Recording scenarios from the headset (docs/adr/0033)

For the person recording, on the headset:

1. **Choose the scenario** with `< PREV` and `NEXT >` on the bench's left wing, behind the power button. NEXT's label
   names the chosen one.
2. **Start a take** by pressing RECORD. It turns into `STOP ● mm:ss`, with the scenario and take number. Do the scenario,
   with the robot on.
3. **Save the take** by pressing STOP. A take that went wrong can be set aside with DISCARD LAST once it is stopped. It is
   moved, never deleted.

**Where the takes go:** on the GPU machine, under `/home/andrei/data/takes/fr3_duo-isaac/<scenario>/<UTC>-take<n>/`, one
folder per take.
- Each take holds:
  - `robothost.jlog`, the robot's truth and commands;
  - `isaac.jlog`, plus `isaac/` with the four cameras' colour, depth and masks;
  - `session.json`, which names the scenario and the take;
  - `files.sha256`. A take is complete once this is written, a few seconds after STOP.
- Another account reads them once `just isaac-share-scenarios <account>` has run (default `seaqueue-cheng`). That account
  can also edit the list, `/home/andrei/data/takes/fr3_duo-isaac/scenarios.txt`:
  - one scenario per line, `name | what to do`;
  - the headset shows the new list within a few seconds.
- `just isaac-scenarios` lists what has been recorded.

**A take stops by itself** when the headset is silent for 10 s, after 30 min, or when the disk is nearly full (below
21 GB). The button's label says so.

## The render check

`just isaac-render-check` (`render_check.py`, docs/adr/0032 phase P0) measures what the cameras' design
rests on, and prints one `RESULT` line:
1. **The bridge alone** for 20 s.
2. **A separate renderer with the bridge beside it,** the two pinned to separate cores. The renderer is a
   visual-only copy of the robot, posed from joint angles about its URDF axes, with the table, the
   backdrop surfaces and the head and third-person cameras.
3. **The head camera rendered inside the bridge's physics loop.**

It exits non-zero if any number is missing, or if the separate renderer misses a criterion:
- the bridge's late steps stay at most 2× its rate alone, or 1/s;
- at least 29.5 fps;
- a render lag that is stable in at least 99.5% of frames;
- depth p99 under 3 mm;
- projection p99 under 3 px.

The bridge runs from the pushed commit. Sample frames stay on that machine, in
`~/isaacsim/render-check/latest`. ADR 0032 holds the 2026-11-02 numbers.

## What is pinned, and how

| Copy | Of | Pinned by |
|---|---|---|
| `teleop_isaac/wire.py` | RobotHost's `ArmServer/ArmServerWire.cs` | `golden/arm-server-wire-v1.json`, written by the C# tests (`just gen-arm-server-golden`) |
| `teleop_isaac/shaper.py` | Core's `JointTrajectoryShaper` (via the trainer's copy, as text) | `golden/joint-shaper-v1.json`, the trainer's golden; an Eval test holds the two equal (`just gen-trainer-golden` refreshes both) |
| `teleop_isaac/scene.py` | Core's `WorldScene` reader, and its rests (docs/adr/0037 amendment 6) | `core/Scenes/testdata/vectors.json`, `golden/scene-fr3-duo-bench.json` and `golden/scene-rests-jetrover-bench.json` (`just gen-isaac-goldens`) |
| `teleop_isaac/kinematics.py` | Core's FK | `golden/fr3-duo-fk-v1.json` (`just gen-isaac-goldens`) |
| `teleop_isaac/world.py`'s rules | RobotHost's `StaticWorld` (a placement on the rest nearest its rotation) | `tests/test_world.py`, including Core's frozen seed-1 layout |
| `teleop_isaac/camwire.py`, `camfeed.py` | ADR 0014's `CameraChunkCodec`, `CameraSubscribeCodec`, `FramePacer`, `CameraStreamer` | `golden/camera-wire-v1.json`, written by `CameraWireGoldenTests` (`just gen-camera-wire-golden`) |

`just isaac-bridge-test` runs those checks. They are plain Python and need no Isaac. Change a copy
and its original together, or the tests fail.

## What stands in for the robot

These are all bridge arguments, and the Welcome's description records them in each session's log:
- **Position drives** of 16000 Nm/rad and 250 Nm·s/rad on the arm joints (about 20 Hz tracking),
  and 2000 N/m and 100 N·s/m on the fingers. They play Franka's internal controller.
- **No gravity on the arms' own links.** The real FR3 compensates its own weight. Only the robot's
  links: the scene's movable objects feel gravity.
- **PhysX at 1 kHz on the CPU,** with 8 solver iterations and no results written back to USD. That
  setting is what makes 1 kHz affordable (docs/adr/0031 amendment).
