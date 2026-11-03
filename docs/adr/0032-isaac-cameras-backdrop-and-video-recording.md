# 32. Isaac cameras, a randomized backdrop and video recording for training data

## Status

Accepted (2026-11-02). The user merged it, with the spike's numbers (below), in PR #100.

**Why it exists.** The user wants to collect training data in the lab with the simulated FR3 Duo, for
models that will take video input. ADR 0031 decision 8 left "an Isaac camera to the headset's display"
and "Isaac as a source of training data" to an ADR of their own. This is that ADR. ADR 0031
amendment 2 covers the objects that go with it.

**The user's choices (2026-11-02):**
- **Recording camera:** one camera on the robot's head, between the arms, its pose recorded.
- **Third-person camera:** in front of the robot, about 45° to one side.
- **Live feeds:** both cameras shown live on the lab's screens, outside the headset's simulated network
  disturbance, as the JetRover's camera is.
- **Backdrop:** a random background on the work-table top and on a screen beyond the table, new each
  recording session, from a public image dataset.
- **Recorded per frame:** colour, depth and object masks.

## Context

**What exists.**
- **Isaac phase 1 (ADR 0031)** runs the arms at 1 kHz in a headless bridge that never renders. It
  turns physics results-to-USD off, because writing them took 1.5 ms of each 1 ms step.
- **The camera path (ADR 0014):** `Teleop.CameraHost` streams JPEG chunks over UDP, and
  `CameraFeedBridge` shows them on the lab's display. The FR3 scene has that display, captioned "none
  yet".

**The constraints.**
- **ADR 0031 decision 2:** "physics at 1 kHz, paced to the wall clock. It never silently runs slow,
  because a slow world corrupts every latency figure."
- **ADR 0031 rejected Isaac-rendered video as the operator's primary view.** The 3D world twin stays
  primary; these feeds are secondary screens.
- **ADR 0014 sends any consumer of frames other than the display to its own ADR.** Training data is
  such a consumer.

**The head camera's view.** The URDF's `head_camera_mounting_point` is on the head. Every joint from
the base to it is fixed, so the camera never moves. It sits at (0.0425, −0.0141, 0.3905) m in the
root frame, pitched 41.0° down. It mostly sees the work surface 0.25–1.1 m ahead, which is why the
backdrop covers the table top as well as a screen.

## Decision

1. **Two cameras,** defined in the scene file's Isaac-only `cameras` section, their poses and
   intrinsics recorded with every session.

   | | Head (recorded) | Third-person |
   |---|---|---|
   | Pose | on `head_camera_mounting_point` | at (1.45, 0.95, 0.85) m, looking at (0.15, 0, 0.20): in front of the robot, to its left and above, framing the robot and the table (amendment 1; the spike's pose cut the robot off) |
   | Resolution | 640×360 | 640×360 |
   | Field of view | 69° horizontal: fx = fy = 465.6 px, cx = 320, cy = 180, pixel centres at +0.5 | same |
   | Rate | 30 fps | 30 fps |
   | Clipping | 0.05–10 m | 0.05–10 m |
   | Model | pinhole, no distortion | pinhole, no distortion |
   | Recorded | colour, depth, masks | colour only |

2. **A separate render process,** `robot/isaac/teleop_isaac/renderer.py`, not the physics bridge.
   - **A second headless Kit process** on the GPU machine. It opens the bridge's converted robot USD
     with the importer's `Physics` variant set to `none`, so it is visual only and nothing steps.
   - **It poses the robot from the bridge's states.** Each moving link's local orientation is its
     authored rest orientation times a rotation about its URDF axis; a finger slides along its axis.
     The spike matched this against the URDF chain to 0.016 mm.
   - **The tap.** The bridge sends a copy of every datagram it sends RobotHost (states, world) to a
     second Unix socket, without blocking. When the renderer is absent or slow, the tap drops the
     datagram and counts it.
   - **Pinned cores:** `taskset`, with the bridge on 2–5 and the renderer on 8–23.
   - **Why not the bridge itself:** rendering in the physics loop makes the world run slow (spike
     below), which ADR 0031 decision 2 forbids. Beside the bridge, the renderer cost it nothing
     measurable.
   - **Frame association.** An image read after posing tick k shows tick k − 2. The spike found this
     render lag stable in every sample. So a frame is stamped with the sensed time of the state posed
     two ticks earlier. The renderer measures the lag at start-up with a test object and refuses to
     record when it is not stable in ≥ 99.5% of frames.

3. **Live feeds on ADR 0014's wire, unchanged** (ADR 0014 amendment 2).
   - **Ports:** the head camera on UDP 6003 and the third-person camera on 6007, on the GPU machine.
   - **Encoding:** JPEG quality 80, off the render thread, with Pillow (about 46 KB a frame, so about
     11 Mbit/s per feed at 30 fps).
   - **`captureTicks`** is the sensed time of the state shown, on CLOCK_MONOTONIC, the clock RobotHost
     reads.
   - **The Python port** of the codec and sender is pinned by a golden file the C# tests write.
   - **On the headset:** two lab displays, `HEAD CAM (REC)` and `3RD PERSON`. They never pass through
     the joint link's disturbance, and the headset decodes colour only.

4. **The backdrop.**
   - **Where:** a random image on the work-table top (a mat) and on a 2.4 × 1.0 m screen beyond the
     table's far edge. In the head camera's view the mat fills 86% of the frame and the screen the top
     12%.
   - **The images:** Open Images V7's validation split, **only images licensed CC BY 2.0**, with the
     author and source of each recorded. COCO was rejected: its per-image licences are mixed, many
     non-commercial, with no author field.
   - **Fetching them:** 2,000 images, once, onto the GPU machine (`just isaac-backdrops-fetch`), with a
     manifest of each file's sha256, licence and attribution. They are treated as untrusted data.
   - **When it changes:** a new seeded pair at each recording session's close and at start-up. A swap
     shows within 2 frames (the render lag), even for a 1.1 MB photo-sized image.
   - **Isaac-only.** The headset shows a plain table.

5. **What is recorded per frame,** for the head camera:
   - **colour:** JPEG quality 92;
   - **depth:** 16-bit PNG, in millimetres;
   - **instance masks:** 16-bit PNG of stable ids.
     - **Where the ids come from:** semantic labels on each link's visual prim and each scene object,
       mapped through a recorded table.
     - **Labels go on prims that don't nest,** because nested labels come out joined (for example
       `left_arm,left_hand`).

   **Masks and depth are privileged simulator labels.** They are never a model input on the robot.

6. **Recording follows RobotHost's sessions through its control file,**
   `~/isaacsim/recordings/.active`, as the JetRover driver follows it.
   - **What the renderer writes** into the session directory: `isaac.jlog` (ASCII, with the JSES header
     and `END` trailer) and `isaac/<camera>.<stream>.frames` sidecars.
   - **RobotHost's finalizer** waits for `isaac.jlog`'s trailer (`--auto-record-writer isaac:60`),
     checks each sidecar's hash, and lists the sidecars in `files.sha256`.
   - **Frames join the robot's truth exactly:** each frame's state stamp equals a `QS` record's.
   - **The tags and layouts** are defined in `trainer/contract/SESSION_FORMAT.md` by the change that
     builds them.

7. **What counts as evidence.**
   - **Frames are training data, never results.** Rendering on a GPU is not bit-reproducible, so no
     golden depends on pixels.
   - **What is pinned:** the camera wire, the forward kinematics and the mask-id table.

8. **Phases,** continuing ADR 0031 amendment 2's:
   - **P3:** the renderer and the two feeds.
   - **P4:** the backdrops and recording.

## Spike results

**The run.** `just isaac-render-check`, 2026-11-02, commit 4539771, Isaac Sim 6.2.0.0, on the RTX
5090 (32 cores). The renderer posed the arms through a scripted motion (joints 1, 2 and 4 swinging by
0.15 rad) and moved a 6 cm test cube to a new spot every frame. Its output was one `RESULT` line
with no field missing.

**Option (b), the separate renderer, passed every criterion.**

| Criterion | Gate | Measured |
|---|---|---|
| Bridge late steps, renderer beside it (cores pinned) | ≤ max(2× alone, 1/s) | 5 in 20,000 steps (0.25/s); alone 0 in 20,000 |
| Renderer frame rate | ≥ 29.5 fps | 30.0 fps; frame 16.4 ms p50, 22.0 ms p99 (render 11.2 / 15.4, posing 3.5, readback 1.6) |
| Render lag | stable in ≥ 99.5% of frames | 2 frames, in all 1,331 samples |
| Depth on the table plane | p99 < 3 mm | under 0.001 mm (float precision) |
| Projection (the cube's 8 corners against its mask, at lag 2) | p99 < 3 px | 0.45 px p50, 0.74 px p99 (the wrong lags: 103–275 px) |
| Masks tell the left arm from the right | yes | yes |

**What else it measured:**
- **Posing:** forward kinematics from the URDF matched the posed USD to within 0.016 mm.
- **The head camera:** it rendered exactly at the URDF mount, (0.0425, −0.0141, 0.3905) m. Its view
  was 86.4% mat, 11.7% screen and 1.9% test cube.
- **GPU memory:** the renderer used 3.3 GB and the bridge 3.0 GB.
- **Encoding with Pillow:** JPEG quality 80 took 0.39 ms and made 45.6 KB; quality 92 took 0.46 ms
  and made 75.6 KB.
- **16-bit PNGs:** 2.8 KB for depth and 5.6 KB for masks. These come from this simple scene; depth
  grows with objects in view.
- **Backdrop swaps:** a new texture showed in 2 frames, both for a 512×256 PNG and for a 1.1 MB,
  2048×1536 JPEG.

**Option (a), the head camera rendered inside the bridge's physics loop, failed.**
- **Late steps:** 152 a second while rendering, against 0.2 without.
- **Render stalls:** 4.7 ms p50, 6.6 ms p99 and 9.4 ms at most.
- **The 4 ms state stream:** its gaps went from 4.2 ms p99 to 8.2 ms p99, and 10.8 ms at most.
- **With PhysX writing to Fabric:** a separate run gave 243 late steps a second.

**What wasn't measured:** the renderer with the bridge's real states over the tap (P3 builds the
tap), and depth and mask sizes in a cluttered scene (P4 measures them on recorded sessions).

## Consequences

- **A second Kit process on the GPU machine:** about 3.3 GB more GPU memory, and about 5 s more to
  start.
- **Disk:** at the spike's sizes, about 17 GB an hour of recording at 30 fps.
  - Head colour: 8.2 GB/h. Third-person colour: 8.2 GB/h. Masks: 0.6 GB/h. Depth: 0.3 GB/h, growing
    with clutter.
  - Disk guards stop recording below 20 GB free.
- **Network:** two feeds at 30 fps take about 22 Mbit/s. The headset can ask for a lower rate.
- **The headset decodes two JPEG streams on its main thread.** The decode cost is logged per feed.
- **The Direction paragraph in root `CLAUDE.md`** gains a sentence: Isaac renders and records frames
  as training data, and nothing on the headset interprets them. A model that consumes them is a
  future ADR.

## Alternatives considered

- **Rendering inside the physics bridge.** Rejected on the spike's numbers: about 150 late steps a
  second with one colour camera, and 4 ms state gaps stretched to 12 ms.
- **The same, with PhysX writing to Fabric so the renderer sees every motion.** Later still: about 243
  late steps a second.
- **COCO for backdrops.** Its per-image licences are mixed.
- **Procedural textures.** Less realistic. The user chose a dataset.
- **One file per frame.** Tens of thousands of files per session. Sidecars indexed by the jlog keep a
  session to a few files.

## Amendment 1 (2026-11-02): the renderer and the feeds built (phase P3)

`teleop_isaac/renderer.py`, `camwire.py`, `camfeed.py` and the bridge's `--tap` are built as decisions 2 and 3 say. `just
isaac-host` starts the renderer beside the bridge (`feeds=false` leaves it out). The headset shows the two feeds on two lab
displays, `HEAD CAM (REC)` on the right and `3RD PERSON` on the left (`XrFr3WorldSetup`).

**Changed from the decision:**
- **The third-person camera** is at (1.45, 0.95, 0.85) m, looking at (0.15, 0, 0.20). At the spike's pose, aimed at the
  table's centre, it cut the robot off at the top right.
- **The scene's lights** are 2.5× dimmer: dome 300 and key 1000, not 800 and 2500. At the spike's values every colour came
  out washed towards white.
- **The render lag is measured on the table,** with the scene's objects hidden. A cube above them is outside the head
  camera's view, which looks 20–62° down.

**Measured on the 5090,** 2026-11-02, `just isaac-host fr3-duo-bench 7`, with the bridge on cores 2–5 and the renderer on
8–23:

| | Gate | Measured |
|---|---|---|
| Feeds, from the PC (`just camera-probe 10 0 "" <host> 6003 6004`, and 6007/6014) | ≥ 28 fps, no incomplete frames | head 29.4 fps, third-person 29.5 fps; 0 incomplete, 0 superseded |
| Renderer frame rate | 30 fps | 30.0 fps |
| Render lag at start-up | stable in ≥ 99.5% | 2 frames, in 100% of 87 samples |
| Bridge late steps with the renderer beside it | ≤ 1/s | 21 in 422,648 steps (0.05/s) |
| `fr3-teleop-probe` with the renderer running | PASS | PASS (tools 0.00 mm) |

**Other measurements:**
- **Frame size:** about 12–13 KB per frame at quality 80. That is smaller than the spike's 46 KB, because the scene has no
  backdrop yet (P4).
- **The tap:** it dropped 1.6% of its copies, all in the renderer's start-up seconds, before it drains them. The renderer
  took about 250 states and 50 world frames a second.
- **The newest state:** about 16 ms old when a frame is read.

**Not measured:** the headset's decode cost for two feeds, which is logged per feed (`[camera] HEAD CAM (REC) ...`).

## Amendment 2 (2026-11-03): backdrops and recording built (phase P4)

Decisions 4–6 are built.
- **The backdrop images** come from `just isaac-backdrops-fetch` (`teleop_isaac/backdrops.py`, standard library). On
  2026-11-03 it kept 2,000 images: 604 MB, all CC BY 2.0, from 1,790 authors.
- **The renderer swaps them** on a mat over the table top and a screen beyond it, and records while RobotHost's control
  file names a session (`teleop_isaac/isaac_session.py`).
- **RobotHost waits for the renderer** with `--auto-record-writer isaac:60`. It checks each `FS` line and lists `isaac/`
  in `files.sha256`.
- **The formats** are in `trainer/contract/SESSION_FORMAT.md` ("Isaac's rendered frames").

**How it differs from the decision:**
- **The mask ids** come from labels on each link's `<link>_visual` prim and on the head's meshes. The links themselves nest,
  and nested labels come out joined.
- **The recorded camera poses** are the USD cameras' (looking along −z, +y up), in the robot's base frame, as `CM` records.
- **The control file is read every 0.1 s.** At 0.5 s, 0.4% of a 73 s session's frames came from after RobotHost had closed
  it, and had no `QS`. Readers skip such frames.
- **`chain-drive --as-headset`** sends the headset's heartbeat, so `just isaac-record-check` can drive a session that
  RobotHost records. Joint-space moves are what bring the hands into the head camera's view: it looks 20–62° down, and at
  the start pose the hands are above its horizon.

**Measured on the 5090,** 2026-11-03, `just isaac-host fr3-duo-bench 7`, then `just isaac-record-check` (40 s of
`chain-drive` at ±0.6 rad, seed 3):

| Check (`verify_session.py`) | Gate | Measured |
|---|---|---|
| `files.sha256`, with the 4 sidecars | all verify | 10 files, all verify |
| `isaac.jlog` trailer and `FS` lines | all match | all match |
| Frames written + dropped vs duration × fps | ≥ 98% (≥ 95% written) | 2,190 of 2,190 expected over 73.0 s; 0 dropped |
| Frames stamped with a `QS` state | ≥ 99.5% | 100% |
| Depth on the mat against the table plane | p99 < 3 mm | p50 0.23 mm, p99 0.50 mm (4.2 M samples) |
| A point in each hand from the state's FK, projected, in its mask | ≥ 95% within 8 px, ≥ 20 samples | 100% of 61; p50 0.41 px, p95 0.59 px |

**Other measurements:**
- **Recording cost:** 209 MB for a 73 s session, or about 10 GB an hour, below the 17 GB/h this ADR estimated. The bridge
  had 0.05 late steps a second while recording.
- **The trainer:** `armpred check-session` accepts the session.
- **Another seed:** seed 5 at 60 s brought the hands into view in only 14 frames, all within 8 px. So the check names seed 3.

## Amendment 3 (2026-11-03): two third-person cameras and two wrist cameras, and a backdrop all around

The user: "we want 2 3rd person cameras and 2 at the wrists. no camera at the base anymore ... I want the background of the
scene to still be dynamic." Then they chose the poses: third-person cameras at front-left and front-right, and wrist
cameras mounted as an Intel RealSense D405 is on a Franka hand.

**The cameras** (the scene file's `cameras`; every one records colour, depth and masks):

| | `third-left` / `third-right` | `wrist-left` / `wrist-right` |
|---|---|---|
| Pose | at (1.20, ±1.10, 0.80) m, looking at (0.25, 0, 0.10): across the table, about 45° to each side and above, framing both arms and every object | on `<side>_fr3v2_hand`, at (0.06, 0, 0.04) m in the hand's frame (on its back, just above the flange), looking at (0, 0, 0.16) with the hand's +x up: along the fingers, tilted 27° towards them, the fingertips in the image's lower half |
| Field of view | 69° horizontal | 87° horizontal (a D405's) |
| Near clip | 0.05 m | 0.01 m |
| Feed port | 6008 / 6009 | 6007 (left) / 6003 (right) |

- **The head camera is gone.** The URDF's mount point is still there; nothing renders from it.
- **A mounted camera** is given in its link's frame (`mountPosition`, `mountLookAt`, `mountUp`). Without `mountLookAt`, it
  keeps the head mount's convention (along the link's +x, +z up).
- **The render lag** is measured with the first camera that stands still. That is `third-left`, so the lag cube on the table
  is in view.
- **What the recording says about a moving camera.**
  - Its `CM` pose is in the mount link's URDF frame. The renderer derives it from Isaac's rendered camera and the link's
    forward kinematics at rest.
  - A new `poseFrame` field says which frame the pose is in (`base` or `mount`).
  - A reader poses the camera at a frame by FK of the frame's `QS` state (`trainer/contract/SESSION_FORMAT.md`).
- **On the headset:** the two lab displays show the wrist cameras, `RIGHT WRIST (REC)` on the right and
  `LEFT WRIST (REC)` on the left.
  - Why the wrists: the twin already shows the arms from outside, while the wrist cameras show what each gripper is about to
    touch.
  - The third-person feeds stay on the GPU machine's ports: `just camera-probe` with 6008 and 6009.
  - The headset decodes two feeds, as before.

**The backdrop** (the scene file's `backdrops`):
- **The surfaces.** It is now a list of `mats` lying on surfaces and `screens` standing up. Each gets its own random image
  at every change (`backdrops.pick(images, seed, change, count)`):
  - `mat`: the work table's top, as before;
  - `floor`: the floor from x −1.0 to 1.4 m and y ±1.8 m;
  - four screens, 2.4 m high from the floor up, closing the scene in. `far` is beyond the table (x 1.4), `back` is behind
    the robot (x −1.0), and `left` and `right` are at y ±1.8.
- **Why:** the third-person cameras look back past the robot, where Isaac had nothing, and the wrist cameras look at the
  table and past it. With a random image on every surface either sees, the background stays something a model must learn
  to ignore.
- **Masks.** Each screen is in the `screen` mask class. The floor mat is in the `floor` class, and the table mat stays `mat`.
  The id table is unchanged.
- **`BK`** now lists `surface:sha256:imageId` for every surface.

**What `just isaac-record-check` (`verify_session.py`) checks now:**
- **Depth on the table mat,** for every camera with depth: p99 under 3 mm.
  - Every camera that stands still must see the mat.
  - So must at least one wrist camera. That checks the moving pose through FK.
- **The hand projection** uses the cameras that stand still.
  - With two of them, the hands are in view at most frames, not only when a joint-space move brings them down into the
    old head camera's view.

**Disk:** 12 streams instead of 4. A rough estimate is about 30–40 GB an hour, against the 10 GB measured with the head
camera; `--record-every 2` halves it. The renderer's frame rate with four cameras on a shared GPU is a risk, measured
below.

## Not decided here

- **Training a video model.** Its own ADR, once data exists.
- **The real FR3's head camera** (ADR 0030 decision 12).
- **Movable objects' rendering at their physics rate.**
- **Emulated impairment of the feeds** (ADR 0014).
