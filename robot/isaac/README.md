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
just camera-probe 10 0 "" <GPU machine> 6003 6004                # the head camera's feed (6007 6014: the third person's)
just isaac-host fr3-duo-bench auto false                         # without the renderer and its feeds
```

`isaac-host` refuses while a training run of ours, or another Isaac host of ours, is going. Other
people's GPU jobs no longer stop it (2026-11-02): it says how much of the GPU they hold, since sharing it
can make physics steps late. The bridge's log is `~/isaacsim/bridge.log` on that machine.

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

`just isaac-scene-check` (`scene_check.py`) runs the real bridge with the scene and talks to it as RobotHost
would. It checks WORLD_INFO, WORLD at 50 ± 5 Hz, placements applied and refused, and the left hand
driven into a post: it must stall more than 0.05 rad short, while the same motion without the post
tracks within 0.01 rad. It also checks `arm-touching` and `overlaps-arm` while the hand rests on the
post, and late steps of at most 1/s. On 2026-11-02 the hand stalled 0.256 rad short, the free motion
tracked exactly (0.000 rad), and there were no late steps in 16 s.

## The renderer and the feeds (docs/adr/0032 decisions 2 and 3)

`renderer.py` is a second, headless Kit process beside the bridge, on cores 8–23 while the bridge has 2–5. Its log is
`~/isaacsim/renderer.log`. It works like this:
- **The stage.** It opens the bridge's converted robot USD with the `Physics` variant at `none`, so nothing steps.
- **The scene.** It builds the scene file's surfaces, objects and lights as visuals, through `world.py`'s
  `build(physics=False)`.
- **Posing.** The bridge copies every state and world message to it (`--tap`, never blocking; drops are counted in the
  bridge's last line). Each frame it poses the robot from the newest state (`kinematics.local_motion`) and the objects from
  the newest WORLD.
- **The cameras.** It renders the scene file's two cameras at 30 fps. JPEG encoding runs on one thread per camera, and a
  frame is dropped rather than queued when its encoder is behind.
- **The feeds.** Each camera goes out on ADR 0014's wire (`camwire.py`, `camfeed.py`) to whoever subscribed: the head camera
  on 6003, the third person on 6007. A frame is encoded only while someone is subscribed.
- **The capture stamp.** `captureTicks` is the sensed time of the state the frame shows, on RobotHost's CLOCK_MONOTONIC.
  The image read after posing frame k shows frame k − lag. The lag is measured at start-up with a cube on the table, with
  the objects hidden; on 2026-11-02 it was 2 frames in 100% of 87 samples.

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
- **Recording.** While RobotHost's control file (`~/isaacsim/recordings/.active`) names a session, the renderer writes
  into it `isaac.jlog` and `isaac/<camera>.<stream>.frames`:
  - head colour (JPEG 92), depth (16-bit PNG, mm) and masks (16-bit PNG, stable ids);
  - third-person colour;
  - `isaac/backdrops.json`, the attribution of every image shown.
  RobotHost waits up to 60 s for the trailer, checks the `FS` lines and hashes `isaac/`. The renderer records only when its
  start-up render lag agreed in ≥ 99.5% of samples, and stops below 20 GB free. The records are in
  `trainer/contract/SESSION_FORMAT.md`.
- **`isaac-record-check`** drives 40 s of seeded joint-space moves with `chain-drive --as-headset`, so that RobotHost
  records them. It waits for the session to complete, then runs `verify_session.py` on the GPU machine. ADR 0032
  amendment 2 holds the 2026-11-03 numbers: 100% of frames stamped with a `QS` state, depth p99 0.5 mm, and each hand's
  point in its mask in 100% of 61 frames.
- **Cost:** about 10 GB an hour of recording.

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
| `teleop_isaac/scene.py` | Core's `WorldScene` reader | `core/Scenes/testdata/vectors.json` and `golden/scene-fr3-duo-bench.json` (`just gen-isaac-goldens`) |
| `teleop_isaac/kinematics.py` | Core's FK | `golden/fr3-duo-fk-v1.json` (`just gen-isaac-goldens`) |
| `teleop_isaac/world.py`'s rules | RobotHost's `StaticWorld` | `tests/test_world.py`, including Core's frozen seed-1 layout |
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
