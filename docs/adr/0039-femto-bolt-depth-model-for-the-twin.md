# 39. A Femto Bolt depth model for the twin's cameras

## Status

Proposed (2026-11-25), branch `femto-depth`, from `v2-views` (c3503b9). It is built and tested without Isaac. It has not
run on a twin (see "The twin check").

**Why it exists.** The JetRover's real third-person cameras are two Orbbec Femto Bolts (`left` and `right`, docs/adr/0014
amendment 4). Their depth is not recorded yet: they need DC power, and the adapters are on order. Models trained on the
twin's depth should transfer to theirs. The twin records exact depth (ADR 0032 decision 5), and a time-of-flight camera
records something quite different: its own pose and lens, a hexagon of illumination, noise, smoothing, holes at edges and at
grazing angles, and millimetre steps. This ADR adds, opt-in, a recorded stream of that depth for chosen cameras, beside the
exact depth and without changing it. It is a new sensor model in the recorded data, so it needs an ADR (CLAUDE.md, "Where
new code goes"). It adds no `Contracts/` interface and no predictor: nothing on the headset or in Core reads it.

## Context

- **The device.** The Femto Bolt carries Microsoft's Azure Kinect 1-megapixel ToF depth module. It runs the same depth
  engine (on the host) and the same depth modes ("identical operating modes and performance": Orbbec's comparison page).
  So Azure Kinect evaluations apply to it. Its colour camera differs: 80° × 51° at 16:9, against the Azure Kinect's 90° × 59°.
- **The mode we will record.** `just femto-record` (robot/femto/femto_record.cpp) records NFOV unbinned, 640×576 Y16 in
  mm, raw (distorted), with Orbbec's calibration.json (intrinsics, distortion, `depth_to_color` extrinsic).
- **The twin.** `third-left` and `third-right` are the Femto Bolts' analogs: fixed in the room, 640×360, 69°, about 1 m
  from the work area. The safety datasets must not change, and every existing stream must stay byte-identical with the
  model off.
- **No real Femto depth exists yet.** This ADR works from published specifications, papers and a public dataset, and says
  how our own recordings replace each number.

## Evidence

### Published figures

| Figure | Value | Source |
|---|---|---|
| Depth modes | NFOV unbinned 640×576, 75°×65°, 0.5–3.86 m, 12.8 ms exposure; NFOV 2×2 binned (software) 320×288, 0.5–5.46 m; WFOV 2×2 binned 512×512, 120°×120°, 0.25–2.88 m; WFOV unbinned 1024×1024, 0.25–2.21 m, 20.3 ms, ≤15 fps (Femto ≤25); passive IR 1024×1024, 1.6 ms | [Microsoft, hardware specification][ms-hw]; [Orbbec, Femto Bolt hardware specifications][ob-hw] |
| Frame rates | 5, 15, 25, 30 fps (Femto Bolt; the Azure Kinect has no 25) | [ob-hw] |
| Range conditions | 15–95% reflectivity at 850 nm, 2.2 µW/cm²/nm ambient; "depth may be provided outside the operating range… depends on an object's reflectivity" | [ms-hw], [ob-hw] |
| Systematic error | typical < 11 mm + 0.1% of distance, excluding multipath (the mean over many frames minus ground truth) | [ms-hw], [Microsoft, depth camera][ms-depth] |
| Random error | standard deviation ≤ 17 mm (temporal, on a static scene) | [ms-hw], [ms-depth] |
| Invalid pixels | value 0. Causes: outside the active illumination mask (a hexagon in NFOV), saturated IR, low IR signal, filter outlier, multipath (corners, and mixed foreground/background at edges; more at edges in fast motion) | [ms-depth]; [Orbbec, depth camera][ob-depth] |
| Pixel, illuminators | 3.5 µm ToF pixel, per-pixel gain selection; two NIR lasers (NFOV, WFOV) | [ms-depth] |
| Depth vs colour camera | the depth camera is tilted 6° down from the colour camera; the NFOV illuminator is not tilted | [Microsoft, coordinate systems][ms-coord] |
| Factory extrinsic (one Azure Kinect) | depth → colour translation (−31.97, −1.94, 4.25) mm, rotation 6.03° about x | `TestCalibration.json` in the [Azure Kinect SDK][ms-sdk] (MIT) |
| Factory extrinsics (two more units) | (−32.29, −2.64, 4.37) mm, 5.1°; (−32.06, −2.19, 5.54) mm, 5.6° | HILO's `cam_params_arc*.json` ([HILO][hilo]) |
| Depth intrinsics (one unit, NFOV unbinned) | fx 515.99, fy 515.49, cx 326.75, cy 326.09 (the principal point 38 px below centre); radial k1 −0.3435, k2 0.2326 | [hilo] |
| Depth-to-colour alignment error | Femto Bolt max < 4 px, mean < 2 px (Azure Kinect < 15.05 / 7.8) | [Orbbec, comparison][ob-cmp] |
| Temporal noise, NFOV unbinned, white wall, centre | 0.61 mm at 0.8 m, 0.89 at 1.5 m, 1.78 at 3.0 m (binned: 0.50, 0.58, 0.98) | [Tölgyessy et al. 2021][tol], Table 2 |
| Warm-up | about +2 mm drift until stable; 40–60 min (Tölgyessy); 35 min ([Lopez Paredes et al. 2023][lp]) | [tol] §4.1, [lp] §IV-A |
| Accuracy and precision | error ≤ 3 mm, σ < 3 mm and rising linearly with distance (0.5–3 m) | [lp] §IV-B; [Song 2021][song] |
| Reflectivity | less reflective materials have higher σ; mirror-like foil reads farther and noisier; porous or translucent materials shift the distance | [tol] §4.4 |
| Flying pixels | present at the edges of a panel at 1.5 m; "inaccurately placed outside the actual object" | [tol] §4.7, [lp] §IV-E, [song] §5.6 |

### Sample data and what it shows

**The data.** [HILO][hilo] (Song, Li, Banerjee, Banerjee, 2025; zenodo 15263380; **CC BY 4.0**). It has 144 tabletop
scenes of 10 everyday objects on a turntable, from two wrist-mounted Azure Kinects at about 0.4–0.9 m. Its `depth_raw` is
the device's NFOV-unbinned output, unaltered: 640×576 16-bit mm, with the hexagon. 162 frames of 6 scenes, all 8 arcs, were
read out of the 39 GB zip by HTTP range requests (61 MB, with the object masks and calibration). They sit in
`~/scratch/agent-femto-depth/data/hilo/` on the GPU machine, treated as untrusted (only PNG and JSON were decoded, with
`python -I`). Also fetched there, CC BY on Zenodo: the PDFs of [lp] (7937972) and [song] (5665338). Microsoft's office sample
recordings were not used: their licence is not stated. No Femto Bolt recording with a licence was found.

**Measured** (`~/scratch/agent-femto-depth/tools/hilo_*.py`; repeated with the repo's `robot/isaac/depth_stats.py`):

| Quantity | HILO raw depth |
|---|---|
| Illuminated (valid in any frame) | 82.5% of the raw image: a hexagon whose top and bottom are the sensor's edges. In undistorted coordinates it spans x −0.72…0.69 and y −0.72…0.54 (about 70°×64°) |
| Invalid inside it | 1.7% a frame (p10 0.8%, p90 3.0%); the table 1.1%; object interiors 0.05% |
| Table invalid by incidence | ≤ 1.1% below 65°; 3.2% at 65–70°, 1.6% at 70–75°, 7.0% at 75–80°, 25% at 80–85° (a lower bound: wholly invalid patches drop out of the region measured) |
| Table invalid by signal cos i / Z² (m⁻²) | < 1.4% at ≥ 0.5; 4% at 0.25; 7.7% at 0.18; 17% at 0.125 |
| Near the object outline | table side, 1–4 px: 28, 21, 11, 7% (5% baseline further out); object side, 1 px: 3.7%. Below objects, where they stand on the table (no depth step): 9.5% at 1 px, against 29–36% at the sides and top. Left and right are alike: no visible illumination shadow |
| Steps (row and column transitions between stable pixels) | 40–100 mm: gap 5.8 px, 0.22 invalid and 3.4 flying px each; 100–300 mm: 6.3 px, 0.77 and 3.4; over 300 mm: 6.3 px, 3.3 and 0.7. Small steps are smoothed into ramps; big ones are invalidated |
| Per-pixel noise on the table (< 60°) | σ 1.08, 1.16 and 1.49 mm at 0.4–0.7, 0.7–1.0 and 1.0–1.3 m, spatially correlated: ρ 0.73, 0.46, 0.22, 0.05 at 1–4 px (a 4-px box gives 0.75, 0.5, 0.25, 0). The engine smooths over about 4 px |
| Quantisation | 1 mm (96% of integer values occupied) |
| Range | valid depth down to 0.20 m (0.1st percentile 202 mm, 1st 230 mm): NFOV unbinned gives depth well inside its 0.5 m "operating range" |
| PNG size | 121 KB a frame |

The spatial σ (≈1.1 mm at 0.8 m) is about twice Tölgyessy's temporal σ (0.55 mm with quantisation removed). So about 0.7–0.9
mm of a frame's roughness is static for a given view: an intensity-related bias that ToF sensors show, and texture. The
model keeps the two apart.

## Decision

1. **A depth model, `teleop_isaac/depth_model.py`** (pure numpy and OpenCV, no Isaac), with its parameters in a model file
   (`robot/isaac/depth_models/*.json`, format `teleop-depth-model` version 1). Opt-in: `renderer.py --depth-model FILE`,
   `just isaac-jetrover-host ... depth_model=femto-nfov`. Without it the renderer makes no extra camera, adds no stream,
   record or file, and keeps its encode pool as it was.
2. **The depth sensor's own view** (geometry `sensor`). For each named camera fixed in the room, Isaac renders a second
   camera at the depth sensor's pose: the colour camera's pose composed with the SDK's depth-to-colour extrinsic (the depth
   sensor at t, its axes R's columns; 32 mm to the colour camera's left, 6° down). It moves with the camera's per-take
   jitter.
   - **The render:** a centred pinhole at 0.86 × the raw focal length (443.8 px), 646×644, which covers the raw image's
     undistorted extent.
   - **The raw grid:** it is resampled onto the device's raw 640×576 grid through the rational distortion, nearest pixel,
     so no depth is ever blended.
   - **So `depth_femto` is the kind of image `femto-record` writes:** its intrinsics (OpenCV convention, pixel centres at
     integers), its lens distortion, its hexagon.
   - **Geometry `same`** applies the model to a camera's own exact depth and pinhole instead: no extra render, no offset or
     distortion. It is also offered for the view cameras (ADR 0038 V).
3. **The model, per pixel, in order:**
   1. **The engine's smoothing.** A centred 3-tap box (`blurPx` 3) over pixels of the same surface. It is blended out where
      the 5-px window spans more than `mixMm` 250 mm (logistic width 20 mm). Small steps become ramps of flying depths; big
      steps stay sharp.
   2. **Invalidation (0),** each cause an independent draw:
      - outside the illumination polygon (HILO's hexagon in undistorted coordinates), out of [0.2, 5.46] m, or no hit;
      - **steps:** a pixel whose window spans more than 40 mm is invalid with probability g(span) on the far side (`bg`
        1.0) and none on the near side (`fg` 0), g a logistic at 300 mm, 30 mm wide. Next to nothing (the room is an HDRI
        at infinity), the near side is kept;
      - **low signal:** a = ρ cos i / Z² through a logistic in ln a, 50% at a = 0.0036 (width 0.35). That is 5% at the
        spec's limit (ρ 0.15 at 3.86 m), so at the twin's 1 m only near-black surfaces at grazing incidence drop out,
        matching HILO's 0.05% on objects. ρ comes from the visible luminance, linear, mapped to 0.15–0.9 (a proxy: NIR
        reflectance is not colour);
      - **grazing:** a logistic at 86.2°, 3.4° wide (HILO's table);
      - **dropouts:** 0.05% of pixels.
      - Incidence comes from the depth's own normals, each tangent taken towards the neighbour nearer in depth, so a
        silhouette pixel gets its own surface's slope, not the step's.
   3. **Systematic error, per take and camera:** an offset (sd 2 mm: device spread and warm-up), a scale (sd 0.1%: the
      spec's 0.1% of distance) and a smooth field (4 slow waves, sd 1 mm).
   4. **Random error:**
      - **per frame:** σ² = σ0² + σ1² Z²/(ρ cos i), with σ0 0.117 and σ1 0.554 mm/m. That is Tölgyessy's NFOV-unbinned
        temporal σ, quantisation removed, least-squares at ρ 0.9;
      - **static per take:** 0.7 mm;
      - **both** are white noise box-filtered over 4×4 px and normalised: HILO's correlation.
   5. **Quantisation** to 1 mm.
   - **Determinism.** Every draw comes from `SeedSequence([tag, BK seed, change, sha256(camera)[:8], frame])`. A take's
     modelled depth is a pure function of its exact depth (the tests check it).
4. **The edge numbers are fitted** (`robot/isaac/depth_fit.py`), not guessed.
   - **The scenes:** HILO-like tabletops (boxes, upright cylinders, a floor 0.75 m below), ray-cast through the twin's own
     render-to-raw path.
   - **The fit:** a coordinate descent over `blurPx`, `windowPx`, `bg`, `fg`, `r50Mm`, `rWidthMm`, `mixMm` and
     `mixWidthMm`, fitted to HILO's step statistics and outline profile. Its loss fell from 98 (first guess) to 1.65.
   - **The bug it exposed:** central-difference normals at silhouettes invalidated 42% of objects' outline pixels, against
     HILO's 3.7%. It was fixed (item 3).
5. **Recorded** (trainer/contract/SESSION_FORMAT.md):
   - **the streams:** `<camera>.depth_femto`, a 16-bit PNG in mm with 0 invalid. Optionally `<camera>.depth_femto_aligned`,
     the same depth warped into the colour camera as the SDK's depth-to-colour transform does (a z-buffered 2×2 splat). With
     the model's `views`, also `view<k>.depth_femto`;
   - **`FM`,** one per stream in each preamble: the image's camera model with its distortion and pixel-centre convention,
     its pose in `CM`'s frame, the model's sha256, and the take's offset and scale;
   - **`isaac/depth_model.json`:** the model as read;
   - **an `FD` reason:** `<camera>.depth_femto-no-image`;
   - **`recordEvery`** (default 1; 2 is the Femto's 15 fps) thins the stream.
   - **What stays unchanged:** `CM`, the exact `depth`, every other stream and record. Every reader keys on `CM` cameras or
     named streams and passes over unknown tags. `test_depth_model_readers.py` builds the same LeRobot episode, camera
     sidecars and visibility from a take with and without the model.
6. **Shipped model files:**
   - `femto-nfov` (the default): NFOV unbinned behind `third-left` and `third-right`;
   - `femto-binned`: NFOV 2×2 binned, 320×288, a quarter of the render;
   - `femto-same`: no extra render, plus every view camera.
   Until our devices are recorded, the camera model and distortion are HILO unit A's, the extrinsic is Microsoft's test
   unit's (MIT), and the hexagon is HILO unit A's. `python -m teleop_isaac.depth_model --from-calibration calibration.json`
   writes a model file from `femto-record`'s calibration (Orbbec's K, k1–k6 and p1–p2, `depth_to_color`).

### The model against HILO

The same statistics (`depth_stats.summarise`), on HILO's 162 frames and on the model over 24 HILO-like synthetic scenes:

| | HILO raw | femto-nfov model | exact depth |
|---|---|---|---|
| Steps 40–100 mm: gap, invalid and flying px | 5.8, 0.26, 3.3 | 3.5, 0.45, 2.2 | 2.3, 0, 1.1 |
| Steps 100–300 mm | 6.3, 0.80, 3.3 | 5.9, 0.70, 4.0 | 2.6, 0, 0.7 |
| Steps over 300 mm | 6.2, 3.2, 0.66 | 6.1, 3.4, 0.87 | 2.1, 0, 0.1 |
| Per-pixel σ on the plane; ρ at lags 1–4 | 1.05 mm; 0.72, 0.43, 0.20, 0.04 | 1.17 mm; 0.69, 0.46, 0.23, 0.01 | 0.52 mm (rounding and resampling staircase) |
| Quantisation | 1 mm | 1 mm | 1 mm |
| Outline, table side 1/2/3 px, object side 1 px (minus HILO's 5% baseline) | 23, 16, 6; 3.7% | 18, 18, 7; 6.8% | — |
| Invalid in the lit region | 2.7% (p90 4.0%) | 9.1% (p10 1.1%, p90 27%: the synthetic scenes include a far floor at grazing incidence) | 1.5% (no hit) |
| PNG a frame | 121 KB | 166 KB | — |

**Where it falls short:**
- Small steps are narrower than HILO's (3.5 against 5.8 px). HILO's objects are curved, so part of its ramps is geometry.
- Grazing faces lose pixels one by one where a real engine probably loses patches.
- The 14% coarser render leaves a ~0.5 mm staircase on slanted surfaces, under the noise.

### Not modelled

- multipath at concave corners: HILO's contact lines show 5–8% extra invalid pixels within 2 px;
- saturation of specular or retroreflective surfaces (the `signal.saturation` slot is off);
- NIR reflectance apart from colour;
- motion widening edge invalidation;
- warm-up drift over a take;
- the room: the HDRI is at infinity, so it gives 0 where a real wall would give depth;
- WFOV modes;
- a camera on the arm with geometry `sensor` (refused).

## Cost (estimated offline; not measured on a twin)

- **GPU.** ADR 0038 V estimates about 4 ms of GPU per 640×360 camera per twin frame, about 12% of the 5090 at 30 fps.
  A `femto-nfov` depth render is 646×644, 1.8 times the pixels: about 7 ms, about 22% a sensor, about 43% for the two.
  - **One twin alone** (about 32–40% today) would run at about 75–83%: 30 fps is plausible.
  - **Both twins** (63–80% today) would go well over 100%. So `femto-nfov` runs on one twin with the other stopped.
  - **`femto-binned`:** 323×322, about 5% a sensor, about +11% a twin. Both twins at +22% would reach 85–102%:
    borderline.
  - **`femto-same`:** no GPU.
  - Render-product overheads may make the costs less than proportional to pixels; the twin check measures them.
- **CPU.** About 21 ms a camera-frame on one thread (Ryzen 9 9950X3D, numpy 2.2, OpenCV 4.12). It runs on the encode pool
  (4 threads), off the render thread, which sustained 60 two-camera frames a second: about 1.3 cores at 30 fps. On the
  render thread: two extra read-backs (646×644 float and RGB) and copies, an estimated 2–3 ms. PNG encoding, about 3 ms a
  frame.
- **Disk.** 120–170 KB a frame: 13–18 GB an hour a camera at 30 fps, so 26–36 GB an hour for two, on top of a twin's
  about 10 GB an hour. `recordEvery: 2` halves it. 166 GB were free on 2026-11-25.

## The twin check (needs the coordinator's slot and the user's go; the branch must be on GitHub)

1. **Frame rate, one twin, the other stopped.** About 2 minutes each of
   `just isaac-jetrover-host jetrover-mixed auto 0 true none -1 <model>` for `none`, `femto-same`, `femto-binned` and
   `femto-nfov`, each with a short take recorded.
   - Read `[render] N fps` and the render/read ms in the 5 s reports.
   - **Pass:** at least 29.5 fps and no `encoder-behind` `FD`.
   - Then, if `femto-binned` passes, twin 0 with it beside twin 1 without.
2. **A short check, 4 takes, `build=false`,** on that twin with `femto-nfov`. Then on the GPU machine:
   - `robot/isaac/verify_session.py <take>`: its checks pass unchanged, and `files.sha256` lists the new sidecars;
   - `python3 robot/isaac/depth_stats.py --take <take> --camera third-left` (and `third-right`). **Pass:**
     - `planeInBase` medianZ within 5 mm of the table top (−0.196 m on jetrover-mixed): the pose, extrinsic and distortion
       chain is right;
     - in-lit invalid share 1–10%;
     - per-pixel σ 0.8–1.6 mm;
     - quantisation 1 mm;
   - for the user, a contact sheet of exact and `depth_femto` frames.
3. **What to look for on the sheet:**
   - the hexagon, with its corners black;
   - thin black rims on the far side of every object against the table or floor, and none on the near side;
   - small steps (clutter, a lid) softened, not cut out;
   - the HDRI room black;
   - the table near-uniformly valid, with a few holes only where it is seen nearly edge-on;
   - a faint mottled texture of about 1 mm;
   - `depth_femto` shifted a few centimetres relative to the exact depth (the 32 mm baseline, the 6° tilt, the lens).

## Validating against our own Femto Bolts once they are powered

1. **Record.** `just femto-record` a still tabletop (the table, boxes, a cylinder, a dark object and a shiny one) at
   0.6–1.5 m, 10 s at 30 fps from both cameras, then 10 s with something moving.
2. **Their own geometry.** `python -m teleop_isaac.depth_model --from-calibration <rec>/left/calibration.json >
   depth_models/femto-left.json` (and right). This replaces K, the distortion and the extrinsic. The rest stays.
3. **Measure.** `python3 robot/isaac/depth_stats.py <rec>/left/depth --calibration <rec>/left/calibration.json --still`.
   It gives:
   - the temporal σ by depth: refit `sigma0Mm` and `sigma1MmPerM`;
   - the plane σ and ρ: `staticMm` is the spatial minus the temporal σ in quadrature, and `kernelPx` the correlation;
   - the step statistics: replace `depth_fit.TARGET` and rerun `depth_fit.py`;
   - the in-lit invalid share;
   - `litPolygonTan`: the new `illumination.polygonTan`.
4. **Compare.** Run the same statistics on a twin take recorded with the refitted model, and compare number for number.
   Once the Femto Bolts are calibrated to the robot's base (ADR 0014 amendment 4, "Not done"), place the twin's cameras
   there, so like views are compared.

## Alternatives considered

- **The model on the colour camera's own depth only (`same`).** It is free, but it has the wrong pose, field of view,
  lens and baseline: the occlusion rims a model learns from would be in the wrong places. It is kept as the cheap option.
- **An undistorted pinhole depth stream.** It is simpler for readers, but the engine works in sensor pixels and our real
  data will be raw. Recording raw lets the validation compare raw against raw, and readers can undistort with `FM`.
- **Per-pixel independent noise.** HILO's ρ at 1 px is 0.73; independent noise would look wrong at a glance.
- **Physically simulating the ToF signal** (phases, multipath, the actual filters). The depth engine is closed, and
  nothing here could validate it.

[ms-hw]: https://learn.microsoft.com/en-us/previous-versions/azure/kinect-dk/hardware-specification
[ms-depth]: https://learn.microsoft.com/en-us/previous-versions/azure/kinect-dk/depth-camera
[ms-coord]: https://learn.microsoft.com/en-us/previous-versions/azure/kinect-dk/coordinate-systems
[ms-sdk]: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/src/csharp/Tests/UnitTests/TestCalibration.json
[ob-hw]: https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Femto%20Bolt%20Hardware%20Specifications
[ob-depth]: https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Depth%20Camera%20(Femto%20Bolt)
[ob-cmp]: https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Femto%20Bolt%20Comparison%20with%20Azure%20Kinect%20DK
[tol]: https://doi.org/10.3390/s21020413
[lp]: https://doi.org/10.1109/JSEN.2023.3273165
[song]: https://doi.org/10.5281/zenodo.5665338
[hilo]: https://doi.org/10.5281/zenodo.15263380
