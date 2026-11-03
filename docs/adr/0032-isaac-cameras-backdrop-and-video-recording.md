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
   | Pose | on `head_camera_mounting_point` | at (1.15, 0.75, 0.60) m, looking at (0.45, 0, 0.05): in front of the robot, about 47° to its left, 28° above the workspace |
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

## Not decided here

- **Training a video model.** Its own ADR, once data exists.
- **The real FR3's head camera** (ADR 0030 decision 12).
- **Movable objects' rendering at their physics rate.**
- **Emulated impairment of the feeds** (ADR 0014).
