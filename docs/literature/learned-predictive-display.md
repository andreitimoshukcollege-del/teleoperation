# Learned and modern predictive display

**Field:** prediction of the *thing you cannot see yet* in a delayed teleoperation or streaming loop
— the remote robot's state, the operator's next command, or the remote camera's next frame — using
sequence models, learned dynamics, or generative video, as opposed to closed-form extrapolation.

Classical predictive display (Bejczy-style model-mediated display, Smith predictors, wireframe
overlays) is another researcher's file. This one starts where the model stops being written by hand.

**What this file is for.** The project's `Prediction/` axis has three classical predictors
(`none`, `const-vel`, `double-exp`) and one planned learned row (`seq-model`, routed through
`IInferenceBackend`). The question that decides whether that row is worth building is not "do
learned predictors win" — it is **what works, at what horizon, at what compute cost**. Every
section below is organised to answer that, and every source is reported with horizon, accuracy at
that horizon, inference latency and the hardware it was measured on, and model size, *wherever the
source gives them*. Where it does not give them, that absence is recorded, because a method whose
inference cost is unstated is not a candidate for a 90 Hz allocation-free frame path.

Per `docs/literature/CLAUDE.md`: nothing here is evidence about this system. Prior work reports X
under conditions Y; whether X holds on this platform's traces and profiles is a sweep, not a read.

---

## 1. The horizon question: where classical extrapolation of human hand motion degrades

This section exists because `docs/metrics.md` §4 mandates benchmarking every predictor at
Δ ∈ {50, 100, 200, 400} ms, and `docs/research-log/2026-09-09-intent-transmission-feasibility.md`
records a secondhand claim that classical hand-motion extrapolation "collapses past ~200 ms" from a
source that run could not retrieve. That source has now been retrieved in full. The claim is
roughly right, the number is somewhat wrong, and the shape is more interesting than either.

### 1.1 The retrieval

**Gamage, Ishtaweera, Weigel & Withana, "So Predictable! Continuous 3D Hand Trajectory Prediction
in Virtual Reality"**, UIST '21. DOI landing page
<https://dl.acm.org/doi/10.1145/3472749.3474753> — **HTTP 403, same as the earlier run** (accessed
2026-09-15). Retrieved instead as the author-hosted preprint at Honda Research Institute Europe,
<https://www.honda-ri.de/pubs/pdf/4814.pdf> (accessed 2026-09-15), full 12-page text, and confirmed
against the Semantic Scholar record
<https://api.semanticscholar.org/graph/v1/paper/DOI:10.1145/3472749.3474753> (accessed 2026-09-15),
which lists it as gold open access, CC-BY, DBLP key `conf/uist/GamageI0W21`. The earlier run's note
that ACM returns 403 unauthenticated is correct and worth keeping — but the paper itself is openly
available one hop away, and the search query that finds it is simply its title.

*Proposes:* a hybrid classical-plus-regressed kinematic model for continuous 3D hand-trajectory
prediction in VR. Displacement is `D(t) = [vt, at², jt³, st⁴, ct⁵] · α(t)` — i.e. a fifth-order
Taylor expansion (through crackle) whose coefficients, instead of being the fixed `1/2, 1/6, 1/24,
1/120` of the classical series, are **functions of the prediction interval** fitted by regression.
Below t = 0.16 s the model interpolates toward the classical coefficients; above it, toward the
regressed ones. Deliberately not a neural network: the authors argue the closed form is explainable
and cheap where "LSTM ... have a higher computational overhead, which is not well suited for
standalone VR systems with limited computation power."

*Claims:* RMSE 0.80 cm (SD 0.12), 0.85 cm (SD 0.14) and **3.15 cm** (SD 0.38) at 100 / 200 / 300 ms.
MAE 0.28 / 0.33 / 1.97 cm at the same horizons. 79.1 % and 78.1 % RMSE reduction versus the naive
baseline and versus the best classical model at 300 and 500 ms. Generalises to new users and new
activities without retraining. Recommended operating limit: "**best used for predicting ballistic
movements up to 340 ms**, as the model's R² score decreases below 0.9 beyond this prediction
interval."

*Conditions:* full text read. 20 participants (7 F, 13 M, mean age 22.4), all 18-39. **OptiTrack,
eight ceiling cameras, seven upper-body trackers, 100 Hz**; Oculus Quest headset and Touch
controllers additionally logged at 72 Hz. The predicted quantity is the *wrist* position (marker
3.5 cm above the wrist), position only — **no orientation, no wrist flexion**. Tasks: a structured
3D pointing/reaching task (targets 5 cm diameter, depths 20/40 cm, angular deviations 30°/60°, 160
trials per participant) plus three commercial VR games (Beat Saber, FitXR-Box, Eleven Table
Tennis). Movements are **ballistic** — aimed, voluntary, segmented as such. Horizons swept at 10 ms
steps from 10 to 500 ms. Error metric is RMSE on position, chosen over MAE deliberately because it
penalises large errors more. For scale, the paper notes commercial headset tracking error is
0.69 cm (Oculus Quest) and 1.69 cm (Galaxy S9) — so the 100 ms and 200 ms figures are *at or near
the tracker's own noise floor*, and the 300 ms figure is far above it.

### 1.2 What the paper actually says about the knee, which is not 200 ms

The 200 ms figure in this project's log is a paraphrase. The paper's own statements, quoted
precisely:

- The best classical model tested is the **fifth-order** Taylor expansion, `CM_k=5`. It "generated
  low average RMSEs until t = 0.16 s (mean = 0.5 cm, SD = 0.07 cm). However, **after t = 0.18 s**,
  data shows that the time varying nature of real hand movements are difficult to capture in these
  equations and it exponentially overestimate the movements."
- A Mann-Whitney test puts the first statistically significant separation at **t > 160 ms**: the
  fifth-order classical model's error (median 0.44) becomes significantly greater than the
  regressed model's (median 0.35), U = 2622, p = 0.024.
- The naive baseline they compare against — `S'(t₀+t) = S(t₀)` — is explicitly "the classical model
  with k = 0". **That is exactly this project's `none` predictor.**

So the documented knee for classical extrapolation of ballistic VR hand motion is **160-180 ms**,
not 200 ms, and it is a knee in an exponential, not a cliff. The project's log is directionally
right and numerically off by ~20 %.

### 1.3 The finding that actually matters: the derivative-order ordering inverts with horizon

The paper's Figure 4b result, stated in the text: "**each added derivative contributes to better
predictions at smaller prediction intervals, but at larger prediction intervals increasingly
contributes to the error**." The mechanism the authors give is error accumulation through the
integrative structure of the series — the fixed weights `1/2, 1/6, 1/24, 1/120` are correct for a
noiseless polynomial and badly wrong for a signal whose higher derivatives are themselves noisy
estimates.

This is a statement about the *shape of a tradeoff*, and it is the reason to read this source
rather than quote its headline. It predicts that on ballistic hand motion the ranking of `none`
(k=0), `const-vel` (k=1), `const-accel` (k=2) and higher orders is **not fixed** — it should
reverse somewhere in the low hundreds of ms, with `none` becoming *less bad* relative to
high-order extrapolators as Δ grows. `Prediction/CLAUDE.md` already half-anticipates this for
`const-accel` ("second-order; overshoots on direction reversal"). Whether the crossover exists on
this project's traces, and where, is a sweep — the horizon grid Δ ∈ {50, 100, 200, 400} ms is
exactly the instrument that would show it, and the crossing is only visible if the low-order rows
are run at the *high* horizons, which is an argument for keeping Δ = 400 ms in the grid rather than
dropping it.

### 1.4 Is half the mandated grid "measuring noise"?

Not quite, and the distinction is worth stating precisely. What prior work reports is:

- At **Δ = 50 and 100 ms**, classical predictors are at parity with the best alternatives and near
  the tracker noise floor. LaViola (below) tested *only* 50 and 100 ms; the UIST source has
  classical error at ~0.5 cm mean out to 160 ms.
- At **Δ = 200 ms**, classical extrapolation is past its documented knee but the learned/regressed
  model is still at 0.85 cm, barely worse than at 100 ms. This is the horizon where the *methods
  separate*, which makes it the most informative row in the grid, not the least.
- At **Δ = 300 ms** even the regressed hybrid degrades 3.7× (0.85 → 3.15 cm), and its authors stop
  recommending it at 340 ms.
- At **Δ = 400 ms**, no source found in this review reports usable *unconditioned* hand-motion
  extrapolation. Everything that works at that range does so by conditioning on something else —
  task structure, goal/target identity, or a learned motion prior over whole gestures (§3, §4).

The defensible reading for this platform: Δ = 400 ms is not noise, it is a **regime change**, and a
predictor that is not conditioned on task or goal is expected to lose to `none` there rather than
merely be imprecise. Reporting it is how that gets demonstrated. What would be wasteful is
reporting Δ = 400 ms *without* the `none` baseline in the same table — which the project's Gate 5
rules already forbid. Note also that all of these numbers are **hand** numbers; the project's
operator-side problem is predicting the *robot's* state, whose dynamics are smoother and more
model-able, and for which the knee should be expected elsewhere. `Prediction/CLAUDE.md`'s
insistence that the two problems be benchmarked separately is supported by everything in this file.

---

## 2. The classical baselines the learned work is measured against

Orienting note: every learned method below reports its win relative to one of three things — a
zero-order hold, a Taylor-series extrapolator, or a Kalman filter. Knowing what those baselines
actually achieve, and under what conditions, is what makes the learned numbers readable. Two of
these sources are also the provenance of predictors already in this project's `Prediction/` folder,
and both turn out to have been validated over a **narrower horizon range than the project sweeps**.

**LaViola, "Double Exponential Smoothing: An Alternative to Kalman Filter-Based Predictive
Tracking"**, Eurographics Workshop on Virtual Environments / IPT 2003.
<https://cs.brown.edu/people/jlaviola/pubs/kfvsexp_final_laviola.pdf> (accessed 2026-09-15).
*Proposes:* double exponential smoothing (DESP) on position and on quaternions as a drop-in
replacement for KF/EKF predictive tracking; one tunable parameter α.
*Claims:* "run approximately 135 times faster with equivalent prediction performance". Both DESP
and KF/EKF give 2-3× lower RMSE than no prediction. Mean RMSE difference between DESP and KF/EKF
was **0.0163 inch for position and 0.0709° for orientation** — negligible. Cost: DESP adds
**≈ 2 µs** per prediction, KF/EKF add **≈ 456 µs**.
*Conditions:* full text read. Six ~20-second datasets (three head, three hand) captured on an
**InterSense IS-900 in a CAVE**, not an HMD. Sampling rates **70 Hz and 180 Hz**. Prediction times
**50 ms and 100 ms only** — four scenarios total. "Ground truth" is the *lowpass-filtered* signal
(cutoff pairs between 1/3 and 6/8 Hz), so the reported RMSE excludes tracker noise by construction
and the evaluation is implicitly on a ≲ 1 Hz signal. Timings on an AMD Athlon XP 1800+, 512 MB RAM.
Per-dataset α is found by search per scenario; the paper names general-case parameter selection as
DESP's one weakness versus KF.
*Relevance here:* this is the source of `double-exp`. The 135× figure is real but was measured on
2003 hardware against a specific EKF implementation, and the accuracy parity claim is established
**only at 50 and 100 ms, on ≈ 1 Hz-bandlimited signals**. It is not evidence about parity at the
project's Δ = 200 or 400 ms rows, and the paper does not claim to be.

**Azuma, "Correcting for Dynamic Error"**, SIGGRAPH '97 Course Notes #30, Making Direct
Manipulation Work in Virtual Reality. <https://www.ronaldazuma.com/papers/sig97pred.pdf>
(accessed 2026-09-15). The companion tutorial to Azuma & Bishop's SIGGRAPH '95 frequency-domain
analysis. The '95 paper itself, <https://dl.acm.org/doi/10.1145/218380.218496>, returned
**HTTP 403** (accessed 2026-09-15), as did the UNC copy `cs.unc.edu/~azuma/s95paper.pdf`, while
`cs.unc.edu/~azuma/vrais95.pdf` returned **HTTP 404**. The 1997 course notes below are the
retrievable presentation of the same analysis and are what this entry reports.
*Proposes:* analysing predictors by transfer function instead of on motion datasets — a magnitude
ratio and phase difference per frequency, compared against the ideal (ratio 1, phase −ωp).
*Claims:* for the second-order predictor `x_pred = x + vp + 0.5ap²`, the magnitude ratio is
`sqrt(1 + (1/4)(ωp)⁴)`, which "grows roughly as the square of either p or ω". Consequences stated
explicitly: (i) **bandwidth × prediction interval is the invariant** — "halving the prediction
interval means that signal can double in frequency while maintaining the same prediction
performance"; (ii) the magnitude ratio above 1 *is* the mechanism of predictor "jitter" —
"prediction magnifies the high-frequency components of the original signal", making the predicted
image "shake rapidly"; (iii) "increasing the prediction interval from 50 ms to 500 ms does not make
the problem ten times harder; it makes the problem virtually intractable"; (iv) predictor execution
time adds directly to end-to-end delay, so "a complicated predictor that takes 50 ms longer to
execute than a simple predictor must be much more accurate ... just to break even".
*Conditions:* analytic, no dataset — that is the point of the method. Assumes a linear predictor and
superposition. The empirical context it quotes: head-motion energy "concentrated below 2 Hz",
system delays "typically fall between 50 - 250 ms", and for delays under 80 ms prediction reduced
average registration error by 2-3× without inertial sensors, 5-10× with them. The analysis is for
*head* motion; the notes explicitly warn hand motion "may be an order of magnitude faster".
*Relevance here:* this is the closest thing in the literature to a first-principles account of the
project's founding premise — see §7.

**Gül et al., "Kalman Filter-based Head Motion Prediction for Cloud-based Mixed Reality"**,
ACM Multimedia 2020. <https://arxiv.org/abs/2007.14084> (accessed 2026-09-15).
*Proposes:* KF and autoregressive head-pose prediction for remote-rendered MR, evaluated against
a no-prediction baseline for motion-to-photon compensation.
*Claims:* at a **60 ms look-ahead**, the KF predicts head orientation **0.5° more accurately** than
an autoregression baseline.
*Conditions:* **arXiv landing page only** — the full PDF was not read for this entry, so the capture
rate, trace count and error metric behind the 0.5° figure are unknown here and that number should
not be moved into any comparison without reading the paper. Application context is cloud-rendered
volumetric video to a mobile MR client. Listed because it is the standard KF comparison point that
later learned head-prediction work measures itself against, and because **60 ms is the horizon a
production-shaped remote-rendering system actually chose** — well inside the classical-safe range
of §1.

---

## 3. Generative video as predictive display — synthesise the frame the operator cannot see yet

Orienting note: this is the most-hyped branch and, on the evidence below, the one furthest from a
90 Hz frame path. The idea is to stop predicting *state* and instead predict *pixels*: feed the
delayed camera stream into a video model and render the frame the remote camera would be producing
now. It is attractive because it needs no state estimate, no plant model and no hand-authored
geometry. The measured inference cost in every source that reports it is between two and four
orders of magnitude above real time on a datacentre GPU, and the error grows monotonically along
the rollout. **This family is blocked on inference cost, not on accuracy.** Nothing here is a
candidate for `IInferenceBackend` on a Quest or a Jetson; the cheapest members (§3.3) are flow- or
interpolation-based, not diffusion. §3.4, added on a later pass, is the exception that fixes the
boundary: it is the only entry in this section that measures a per-frame cost and meets a
real-time bar, and it is neither generative nor learned — which is the point.

### 3.1 The benchmark that says the off-the-shelf models do not work yet

**Khalil & Kwon, "Towards Generative Predictive Display for Vision-Based Teleoperation: A Zero-Shot
Benchmark of Off-the-Shelf Video Models"**, arXiv:2605.09670 (10 May 2026).
<https://arxiv.org/abs/2605.09670> and full text at <https://arxiv.org/html/2605.09670>
(both accessed 2026-09-15).
*Proposes:* predictive display framed as rollout-based future-frame prediction; a unified zero-shot
benchmarking pipeline (no task-specific fine-tuning) over five public video models across two
resolutions and two conditioning regimes (multi-frame, single-frame). Scores prediction accuracy
(mean absolute difference), per-rollout latency, peak GPU memory, and **temporal error evolution
across the horizon** — that last axis is the one most video-prediction papers omit and the one
predictive display actually needs.
*Claims:* "no tested model simultaneously achieves low rollout error, non-divergent per-step error
behavior, and real-time inference at the source frame rate", and "increasing model scale or
resolution yields limited and, in some cases, inverted improvements".
*Conditions:* full text read. **CARLA simulator driving data**, not a robot arm and not real
imagery. 99 conditioning frames, 88 predicted frames at **15 FPS ≈ 587 ms of predicted video**;
source frame budget `T_frame = 66.7 ms`. Resolutions 256×160 and 512×320. Hardware: a single
**NVIDIA RTX 6000 Ada, 48 GB VRAM**. Models and measured cost:

| Model | Params | Rollout latency | Peak VRAM | MAD @256×160 / @512×320 |
|---|---|---|---|---|
| LTX-Video 13B | 13 B | 29.84-30.80 s | 46.68 GB | 22.34 / 19.57 |
| LTX-Video 2B (distilled) | 2 B | 24.96-25.36 s | 25.97 GB | 14.23 / 11.55 |
| Stable Video Diffusion 1.1 (I2V) | ~1.5 B | 10.48-12.32 s | 3.15-3.44 GB | 78.68 / 83.24 |
| Wan I2V 1.3B | 1.3 B | 21.20-23.36 s | 10.80 GB | 26.90 / 23.52 |
| Wan VACE 1.3B | 1.3 B | 20.32-30.56 s | 11.70-13.58 GB | 89.42 / 32.08 |

*Reading:* to produce ~587 ms of future video the cheapest model takes ~10.5 s on a 48 GB
workstation GPU — roughly **18× real time in the wrong direction**, and ~150-460× the 66.7 ms
per-frame budget. The scale inversion is the more interesting result: the 13B model is *worse* on
MAD than the distilled 2B, and SVD gets worse at higher resolution. Whatever the bottleneck is, it
is not capacity. The authors' own conclusion is that deployment needs "explicit short-horizon
temporal supervision, in-domain adaptation, or aggressive inference optimization" — i.e. the
off-the-shelf route is closed.

### 3.2 Flow-conditioned GAN frame prediction for ground-vehicle teleoperation

**Moniruzzaman, Rassau, Chai & Islam, "Long future frame prediction using optical flow-informed deep
neural networks for enhancement of robotic teleoperation in high latency environments"**, Journal of
Field Robotics 40(2):393-425, 2023. <https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22135>
(accessed 2026-09-15) — **HTTP 403, Wiley**, despite the record listing it as hybrid open access
CC-BY. The Edith Cowan University repository copy at
`ro.ecu.edu.au/cgi/viewcontent.cgi?article=2469&context=ecuworks2022-2026` also returned **HTTP
403**. Metadata and full abstract retrieved from
<https://api.semanticscholar.org/graph/v1/paper/DOI:10.1002/rob.22135> (accessed 2026-09-15), DBLP
key `journals/jfr/MoniruzzamanRCI23`.
*Proposes:* future-frame prediction as image-to-image translation with a **Pix2Pix conditional
GAN**, where synthetically generated optical-flow components "reflecting real-time control inputs"
are appended as two extra channels to the RGB input — a five-channel input. Notably the flow is
derived from the *operator's commands*, which makes this a command-conditioned video predictor
rather than a pure video model.
*Claims:* with three-channel RGB, **SSIM 0.60 and MS-SSIM 0.68**; with the five-channel
flow-augmented input, **SSIM 0.67 and MS-SSIM 0.74**. Human-rater agreement (Fleiss' κ) 0.40 → 0.55
on the same comparison. The authors claim to be "the first to attempt to reduce the impacts of
latency through future frame prediction using deep neural networks".
*Conditions:* **abstract only** — the full text could not be retrieved, so the following are from
the abstract and are all that is safe to state. Three datasets of **20,000 input images each**,
generated in the authors' own **custom teleoperation simulator**, with a **fixed 500 ms delay**
between input and target frame. So: one horizon, simulated imagery, no variable delay, no burst
loss. **No inference time, GPU or model size is stated in the abstract**, which for a Pix2Pix-class
cGAN is the single number that decides whether this is deployable — and its absence is why this
source generates a candidate rather than settling anything. The 170 ms and 1 s degradation
thresholds attributed to this paper in secondary sources were not verified against its text and are
not restated here as its findings.

A companion paper by the same group, **"Structure-Aware Image Translation-Based Long Future
Prediction for Enhancement of Ground Robotic Vehicle Teleoperation"**, Advanced Intelligent Systems,
<https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202200439>, returned **HTTP 403**
(accessed 2026-09-15) and no open copy was found; recorded as a known-unretrieved pointer, not as a
source.

### 3.3 The cheap end: learned warping rather than learned generation

**Chakraborty et al., "Towards Real-Time Generation of Delay-Compensated Video Feeds for Outdoor
Mobile Robot Teleoperation"**, arXiv:2409.09921 (Sep 2024, rev. Feb 2025).
<https://arxiv.org/abs/2409.09921> (accessed 2026-09-15).
*Proposes:* a modular learning-based vision pipeline that generates delay-compensated images in
real time for an agricultural-robot supervisor — explicitly modular rather than end-to-end
generative, which is what buys the runtime.
*Claims:* more accurate images than state-of-the-art alternatives in their setting; "one of the few
works to evaluate a delay-compensation method in outdoor field environments with complex terrain on
data from a real robot in real-time".
*Conditions:* **arXiv landing page only** — horizon, accuracy numbers, inference latency, GPU and
model size were not readable and are therefore not stated here. Domain is dense crop rows with
variable frame rate and a real robot. Worth a full read by anyone pursuing view synthesis, because
it is the one paper in this family that claims real-time on field hardware; that claim is exactly
the thing to check against §3.1's numbers.

### 3.4 The one method in this section that measures a per-frame budget — and it learns nothing

**Sharma, Calder & Rajamani, "Predictive Display for Teleoperation Based on Vector Fields Using
Lidar-Camera Fusion"**, International Journal of Computer Vision 133(12):8315-8331 (2025),
DOI 10.1007/s11263-025-02550-z. Both publisher copies —
`link.springer.com/content/pdf/10.1007/s11263-025-02550-z.pdf` and
`link.springer.com/article/10.1007/s11263-025-02550-z/fulltext.html` — return **HTTP 303 to a
Springer identity-provider cookie URL** (accessed 2026-09-16), which is what the earlier run
recorded. They do so **despite Crossref recording a `vor` licence of CC-BY 4.0** with zero embargo
(<https://api.crossref.org/works/10.1007/s11263-025-02550-z>, accessed 2026-09-16). The work is
NSF-funded (grant CNS 2321531), so the full text is in the **NSF Public Access Repository**:
landing page
<https://par.nsf.gov/biblio/10635373-predictive-display-teleoperation-based-vector-fields-using-lidar-camera-fusion>
and full text <https://par.nsf.gov/servlets/purl/10635373> (both accessed 2026-09-16, full 17-page
text read).

*Proposes:* predictive display with **no learned model and no pixel generation**. A **high-gain
observer** estimates the ego vehicle's position and yaw from **GNSS plus gyroscope only** (the
argument being that these are small enough to transmit far more often than imagery). Delayed Lidar
points are filtered to the camera frustum and projected into the delayed image; transforming them by
the observer's estimates at the delayed time t₁ and at now t₂ gives a **sparse vector field** of
per-pixel displacements. A **degree-two multivariate polynomial (k = 6 or 12 coefficients) fitted by
ridge regression** interpolates that sparse field to a dense one, and OpenCV `remap` warps the
delayed image into the predicted view. The paper explicitly rejects three richer interpolators on
cost and generalisation grounds: harmonic/biharmonic inpainting ("slow"), kernel regression ("quite
slow for real time applications"), and neural interpolation, which "requires training on large
datasets and significant training time ... it suffers from the problem of generalization". Its
stated motivation for avoiding generative video is the same one §3.1 measures: "the high inference
latency of diffusion models has limited their applicability in autonomous driving".

*Claims:* mean image metrics over the run, versus **showing the delayed image** and versus **NKSR**,
a neural 3D-reconstruction-plus-raycasting predictive display from the same group's earlier work:

| Dataset | Delay | PSNR delayed / NKSR / VF | SSIM delayed / NKSR / VF | MS-SSIM delayed / NKSR / VF | TOLM delayed / NKSR / VF |
|---|---|---|---|---|---|
| KITTI | 0.5 s | 10.39 / 10.92 / **12.38** | 0.25 / 0.27 / **0.32** | 0.26 / 0.36 / **0.37** | 0.25 / 0.26 / **0.41** |
| KITTI | 0.7 s | 9.84 / 10.54 / **11.23** | 0.23 / 0.25 / **0.30** | 0.23 / **0.33** / 0.30 | 0.17 / 0.18 / **0.26** |
| nuScenes | 0.5 s | 15.22 / 11.67 / **16.72** | 0.38 / 0.43 / **0.44** | 0.41 / **0.50** / 0.49 | 0.26 / 0.41 / **0.51** |
| nuScenes | 0.75 s | 14.56 / 11.40 / **16.00** | 0.36 / 0.39 / **0.41** | 0.38 / **0.46** / 0.45 | 0.19 / 0.30 / **0.42** |

Compute, Table 6: total time per synthesised image **0.02 s on KITTI** (image 375x1242, 126,995
Lidar points) and **0.044 s on nuScenes** (900x1600, 34,752 points); the high-gain observer
contributes "of the order of 5e−5 s". Complexity is `O(n·m + n_Lidar)` and is **dominated by image
size, not by Lidar point count**. Their conclusion: the method "can easily run at a rate of more than
20 Hz for both nuScenes and KITTI data making it reliable for real-time operations".

*Conditions:* full text read. **Delays are fixed and long** — 0.5 / 0.6 / 0.7 s on KITTI and
0.5 / 0.67 / 0.75 s on nuScenes; **no variable delay, no jitter, no loss**. Data is replayed from two
public AV datasets, not a live link. Sensor rates (their Table 1): KITTI GPS 10 Hz, IMU 10 Hz,
camera 10 FPS, Lidar 10 Hz; nuScenes GPS 100 Hz, IMU 100 Hz, camera 12 FPS, Lidar 12 Hz; GNSS
accuracy 10 cm, Lidar accuracy 2 cm. Hardware for the timings: **24-core Intel i9, 64 GB RAM, NVIDIA
RTX 4090 24 GB**, described as "typical for a remote teleoperation PC" — i.e. the operator station,
not the vehicle. The stated scope limit is severe and the authors say so: the method "is currently
limited to **static scenes** (where only the ego vehicle is moving)"; monocular front camera only;
"a sparse Lidar might degrade the system's performance". The regularisation parameter λ trades
warping against accuracy (KITTI, λ = 5 → PSNR 12.2 / SSIM 0.32 / TOLM 0.44; λ = 15 → 12.1 / 0.31 /
0.43). Delaying the GPS and IMU by 10 ms barely moves anything (nuScenes PSNR 16.73 → 16.71). The
observer-versus-EKF comparison is honest rather than one-sided: on KITTI the EKF has the lower x
RMSE (0.05 m versus the high-gain observer's 0.19 m), while the high-gain observer wins on yaw
(RMSE 2° versus 2.1°, max error 11.92° versus 13.74°) and on nuScenes wins on yaw by roughly 2×.
**TOLM is a metric the paper itself introduces** — YOLOv8 boxes on the undelayed image, SIFT feature
matching to find the corresponding box in the test image, mean IoU over boxes. It is reported here
as prior work's instrument; `docs/metrics.md` governs what this project measures and nothing here
proposes adding it.

*Why it belongs in this file:* it is the **only source in §3 that reports a measured per-frame cost
and meets its own real-time bar**, and it does so by replacing learning with a six-coefficient ridge
regression. Two cautions before that reads as encouraging. First, 0.02-0.044 s per image is 23-50 Hz
on an RTX 4090 workstation — **still 2-4× too slow for a 90 Hz frame path**, on hardware far above a
Jetson. Second, the horizon is 0.5-0.75 s, which §1 and §4 say is hopeless for unconditioned human
motion; it works here because the predicted signal is **vehicle ego-motion measured by GNSS and a
gyro**, about as smooth and observable as a teleoperation signal gets, and because the scene is
static. The structural analogue on this platform would be reprojecting a received frame using fresher
pose — which needs a camera, a depth source and a render target, lands in
`unity/TeleopVR/Assets/Teleop/Runtime/Bridge/`, and is therefore **blocked on human review here**,
not a Core row.

---

## 4. Learned prediction of the *operator's command* — the robot-side problem

Orienting note: `Prediction/CLAUDE.md` splits the problem in two and insists they be benchmarked
separately. The literature splits the same way, and the robot-side half is where the horizons get
long. The pattern across every source in this section is the same and it is the most transferable
finding in this file: **methods that beat the ~200 ms wall do it by conditioning on task structure,
not by extrapolating better.** A predictor that knows "this is a reach toward one of four known
targets" can run at 1.5 s; a predictor that only knows position, velocity and acceleration cannot
run at 300 ms. That is a statement about *what is being conditioned on*, not about model capacity —
and the strongest example below is not even a neural network.

**Penco, Mouret & Ivaldi, "Prescient teleoperation of humanoid robots"**, arXiv:2107.01281 (2 Jul
2021, rev. Mar 2022). <https://arxiv.org/abs/2107.01281>, full text read at
<https://ar5iv.labs.arxiv.org/html/2107.01281> (both accessed 2026-09-15).
*Proposes:* the robot **executes commands before it receives them**. It continuously predicts the
operator's future whole-body command by querying a model trained on past trajectories and
conditioned on the last received commands, so the visual feedback the operator sees appears
synchronised while the robot actually acted in the operator's past. The model is **Probabilistic
Movement Primitives (ProMPs)** — a distribution over whole trajectories in a basis-function space,
conditioned on the observed prefix. **Not a deep network.**
*Claims:* an operator successfully controlled a 32-DoF humanoid under **stochastic delays up to
2 seconds** across whole-body manipulation tasks (reaching targets, picking up a bottle, placing a
box). Cartesian trajectory error "around half a centimetre" after observing half of the motion.
*Conditions:* full text read. Robot is iCub (53 actuated DoF, 32 controlled — hands and eyes
excluded). Whole-body controller at **100 Hz**; motion capture received at 100 Hz, transmitted to
the robot controller at 50 Hz. Delay model: forward delay **N(750 ms, 100 ms)** plus 750 ms
backward, i.e. round-trip ≈ 1.5 s; also tested at 2 s, and at 3 s and 4 s where performance
degraded. Training sets are **tiny** — 12 whole-body demonstrations for bottle reaching, 42 for box
handling. **No inference time or model size is reported**, which for a ProMP query is plausibly
sub-millisecond but is not stated.
*Why it matters here:* this is the existence proof that the §1 wall is a property of *unconditioned
extrapolation*, not of human motion. 2 s is 5× the project's longest horizon row. The price is
explicit and large: the method needs a demonstration set per task, and works because whole-body
reaching is highly stereotyped. It is also **closed-loop by construction** — the operator sees
synchronised feedback and therefore behaves differently — which is the same objection
`Autonomy/CLAUDE.md` raises about scoring arbiters from recordings. A ProMP-style predictor could
not be honestly scored on this project's `.tlog` replays without acknowledging that the recorded
operator never experienced the prediction.

**Abubakar, Zweiri, Haddad, Yakubu, Alhammadi & Seneviratne, "Physics-Informed LSTM-Based Delay
Compensation Framework for Teleoperated UGVs"**, arXiv:2402.16587 (26 Feb 2024).
<https://arxiv.org/abs/2402.16587>, full text read at <https://arxiv.org/html/2402.16587v1>
(both accessed 2026-09-15).
*Proposes:* four LSTM predictors in a bilateral teleoperation loop — two compensating forward
delay, two backward — with physical constraints folded into the loss ("physics-informed"), replacing
the linear model-free predictors conventionally used in bilateral control.
*Claims:* **26.1 % improvement** in delay compensation over conventional model-free predictors for
large delays, open-loop. The concrete instance behind that figure: "average performance metric for
all variables (framework) for Operator #4 is 32.3 % for PiLSTM, in contrast to Conv's 58.4 %".
RMSE 2.90-72.3 × 10⁻³ across variables and test sets.
*Conditions:* full text read. **Simulated lunar UGV on soft terrain** — gravity 1.633 m/s², terrain
friction angle 0.50-0.95 rad, **maximum speed 0.1 m/s**. One-way delay 1.25 s, plus a real-network
variant of 1.0 s ± U(−0.25, 0.25). Sampling at **10 Hz**, 50-sample (5 s) history window, predicting
one step ahead. Architecture: dense layer 118-207 units, 2-4 LSTM layers, 75-188 hidden units.
**Inference time and hardware are not specified anywhere in the paper.**
*Reading:* a 10 Hz loop on a 0.1 m/s vehicle is about as far from a 90 Hz VR hand as a teleoperation
paper gets. The transferable part is the *structure* — separate forward and backward predictors,
physics as a constraint rather than as the model — not the 26.1 %.

**Zhang, Yang, Huang & Song, "A Delay Compensation Framework Based on Eye-Movement for Teleoperated
Ground Vehicles"**, arXiv:2309.07464 (14 Sep 2023). <https://arxiv.org/abs/2309.07464>
(accessed 2026-09-15).
*Proposes:* extract operator intent from **eye movement**, combine it with contextual constraints
into a guidance trajectory, and drive the vehicle from the trajectory — removing the operator from
the direct control loop entirely. "The delay can be compensated as long as the prediction horizon
exceeds the delay."
*Claims:* significant improvement in manoeuvrability and cognitive burden at delays **> 200 ms**,
by repeated-measures ANOVA; better than the same framework without the eye-movement feature.
*Conditions:* **arXiv landing page only.** Human-in-loop simulation platform, several delay levels;
no accuracy numbers, compute cost or hardware were readable. Listed because it is the clearest
statement of the **trajectory-transmission alternative** to extrapolation — the same idea this
project's `docs/research-log/2026-09-09-intent-transmission-feasibility.md` calls the `trajectory`
row — and because it sources intent from a signal (gaze) this platform does not currently capture.
Note that adding an eye-tracking input would land in `unity/` and is therefore blocked on human
review on this platform.

**Mompó Alepuz, Papageorgiou & Tolu, "Learning-based Delay Compensation for Enhanced Control of
Assistive Soft Robots"**, arXiv:2504.12428 (16 Apr 2025). <https://arxiv.org/abs/2504.12428>
(accessed 2026-09-15).
*Proposes:* learn an approximation of the **nonlinear Smith predictor** state predictor, using
Kernel Recursive Least Squares Tracker and a Legendre Delay Network, for a two-module soft robot
arm with short inherent input delay.
*Claims:* "significant improvement in tracking performance compared to a baseline model-based
non-linear controller" — no number given on the landing page.
*Conditions:* **arXiv landing page only.** Delay magnitude, horizon, accuracy figures, inference
cost and model size were all unreadable. Listed for one structural reason: the **Legendre Delay
Network** is a compact, fixed-size, linear-state-space way to hold a delayed signal history, which
is a plausible fit for an allocation-free `netstandard2.1` predictor in a way that an LSTM is not.
That is a candidate, not a result.

---

## 5. Sequence models on human hand and head pose — the family closest to `seq-model`

Orienting note: this is the family the planned `seq-model` row belongs to, and it is where the
compute question is sharpest, because these models are meant to run on the headset. The single most
useful pattern across this section, visible whenever a paper bothers to include a zero-velocity
baseline, is a **crossover**: below roughly 50 ms the learned model is at parity with or *worse
than* holding the last sample, and above roughly 200 ms it is decisively better. If that crossover
reproduces on this project's traces, the Δ ∈ {50, 100, 200, 400} ms grid is well placed to find it
and the `none` baseline is doing real work rather than being a formality.

**Li, Somarathne, Sarsenbayeva & Withana, "TA-GNN: Physics Inspired Time-Agnostic Graph Neural
Network for Finger Motion Prediction"**, arXiv:2503.13034 (17 Mar 2025).
<https://arxiv.org/abs/2503.13034>, full text read at <https://arxiv.org/html/2503.13034v1>
(both accessed 2026-09-15). Same lab as the UIST source in §1.
*Proposes:* a three-stage model — a kinematic feature extractor producing filtered velocity and
acceleration, a **physics-based encoder that follows linear kinematics**, and a graph decoder over
finger-joint topology. "Time-agnostic": the prediction interval is an input, so one model serves all
horizons rather than one model per horizon (contrast the UIST source's 4000 per-interval models).
*Claims:* per-horizon, against a **zero-velocity baseline (= this project's `none`)**:

| Horizon | VRHands MAE, TA-GNN | VRHands MAE, zero-velocity | Re:InterHand RMSE, TA-GNN | Re:InterHand RMSE, zero-velocity |
|---|---|---|---|---|
| 20 ms | 0.50° | **0.40°** (baseline wins) | — | — |
| 40 ms | 0.74° | 0.76° (2.7 % better) | 6.96 mm @44.4 ms | 8.38 mm |
| 100 ms | 1.01° | 1.73° | — | — |
| 200 ms | 1.09° | 3.06° (64.4 % better) | 10.23 mm @222 ms | 23.64 mm |
| 400 ms | 2.25° | 4.51° (50.1 % better) | 17.82 mm @444 ms | 29.64 mm |

Also reported: against Chan et al.'s classical Pehm model, 91.6-93.5 % lower MAE at 100/200/400 ms;
against Yang et al.'s LSTM-Attention, 59-82 % lower MSE. Ablation: removing the physics encoder
costs 18.0 % MAE on average, worst at 160 ms (+42.9 %).
*Conditions:* full text read. **VRHands**: Meta **Quest 2**, 7 participants, **100 Hz**, 14 finger
joints, 2+ hours. **Re:InterHand**: **90 Hz**, 21 joints per hand. The predicted quantity is
**finger-joint angles**, not wrist Cartesian position — a different and smoother signal than §1's.
Model size **1.49 M parameters** (9 K/joint extractor, 25 K/joint encoder, 37 K decoder). **No
runtime, latency or hardware is reported anywhere in the paper** — the authors assert only that the
model is "relatively simple and minimal in resources required." For a `seq-model` decision that
absence is the whole problem: 1.49 M parameters at 90 Hz is a very different proposition depending
on whether it is 0.2 ms or 20 ms per call, and Azuma's break-even argument (§2) says a predictor
that costs 20 ms has already spent a fifth of a 100 ms horizon.
*Two readings worth separating.* The favourable one: a small learned model degrades **roughly
linearly** out to 444 ms (6.96 → 17.82 mm, 2.6×) where §1's fifth-order classical extrapolator went
exponential past 180 ms. The unfavourable one: at 20 ms the zero-velocity baseline is
**better than the learned model**, and at 40 ms the learned model's margin is 2.7 % — i.e. inside
any plausible measurement noise. Prior work here reports no benefit from learning at short horizons.

**Zhong, Landfeldt, Alce & Caltenco, "Predictability-Aware Motion Prediction for Edge XR via
High-Order Error-State Kalman Filtering"**, arXiv:2507.13179 (Jul 2025).
<https://arxiv.org/abs/2507.13179>, full text read at <https://arxiv.org/html/2507.13179v2>
(both accessed 2026-09-15).
*Proposes:* a high-order error-state Kalman filter with a **motion classifier that categorises head
motion by how predictable it is**, and selects filter behaviour accordingly — an explicitly
predictability-aware, not just more-expressive, predictor.
*Claims:* at a **100 ms horizon**, position error (median, spanning easy to hard motion classes):
KF 2.061-38.645 mm, ESKF 1.943-35.693 mm, their p3o3 PseudoESKF **0.938-15.469 mm**. Orientation
error: KF 0.973-2.283°, p3o3 **0.424-1.172°**. "The performance gap widening as the prediction
horizon increases."
*Conditions:* full text read. Head motion captured from an **Oculus Quest 3 via ALVR at 100 Hz**.
The paper deliberately caps its "prediction horizon to less than 100 ms, consistent with prior
work" — worth noting as an independent vote on where the practical ceiling for head pose sits.
Implemented in **Python on an Apple M1 (8-core, 16 GB)**; **no inference latency reported**, and a
Python KF timing would not transfer to this project anyway. Note the 20× spread between easy and
hard motion classes: this is the same tail behaviour `docs/metrics.md` §4's "failure rate" metric is
there to expose, and it says a p50 comparison between predictors on mixed motion is close to
meaningless.

**Bao, Chen, Zeng, Li, Xu, Yuan & Kong, "Uncertainty-aware State Space Transformer for Egocentric 3D
Hand Trajectory Forecasting"**, ICCV 2023, arXiv:2307.08243.
<https://arxiv.org/abs/2307.08243>, full text read at <https://arxiv.org/html/2307.08243v2>
(both accessed 2026-09-15).
*Proposes:* USST — a state-space Transformer with **aleatoric uncertainty**, forecasting 3D hand
trajectory from first-person RGB video, with a velocity constraint and visual prompt tuning on a
large ViT backbone.
*Claims:* on H2O-PT, 3D ADE 0.031 m and 3D FDE 0.052 m; on EgoPAT3D-DT, 3D ADE 0.183 m (seen) /
0.120 m (unseen). Beats nine baselines including DKF, VRNN, SRNN, AgentFormer and ProTran.
*Conditions:* full text read, but the quantities this file cares about are **weakly specified in the
paper itself**. 64-frame clips with a 60 % observation ratio, sampled with step size 15 frames; the
paper does not consistently state the datasets' frame rates, so the horizon is only reconstructible
as **roughly 0.4-0.67 s** and should be treated as approximate. Latent dim 16, feature dim 256;
parameter count not given beyond "comparable model size to AgentFormer", and inference reported only
as "competitive speed to ProTran ... measured in milliseconds per video on RTX GPUs". **Input is
egocentric RGB video, not tracked pose** — so this predicts where a hand will go *from images*,
which is a different and much more expensive problem than the one `IPredictor` solves. Relevant
mainly for its uncertainty head: `Prediction/CLAUDE.md` requirement 5 asks for an uncertainty
estimate, and `Prediction/CLAUDE.md`'s planned `ekf` row wants covariance for the reconciler to use.
Prior work here provides one shape for that output.

**Hou, Zhang, Budagavi & Dey, "Head and Body Motion Prediction to Enable Mobile VR Experiences with
Low Latency"**, IEEE GLOBECOM 2019, DOI 10.1109/GLOBECOM38437.2019.9014097. IEEE Xplore
(<https://ieeexplore.ieee.org/document/9014097/>) returned **no retrievable body** (accessed
2026-09-15) and <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1109/GLOBECOM38437.2019.9014097>
(accessed 2026-09-15) records `openAccessPdf` as empty — but the **full 7-page camera-ready is hosted
by the authors' own lab**, UC San Diego's Mobile Systems Design Lab, at
<http://esdat.ucsd.edu/sites/default/files/publications/Globecom19_Xueshi_Hou.pdf> (accessed
2026-09-16), header "To appear in 2019 IEEE Global Communications Conference (GLOBECOM'19)". Reached
by fetching <http://esdat.ucsd.edu/publications> and reading its link list. DBLP key
`conf/globecom/HouZBD19`.
*Proposes:* predictive pre-rendering for edge/cloud **6DoF** VR. An encoder-decoder **LSTM** (two
layers of 60 units, then a 1-node fully-connected layer) and an **MLP** (two fully-connected layers,
60 and 1 nodes) each predict the *motion speed* of the next sample on six axes — body position
x, y, z and head pose pitch/yaw/roll — from a 60-sample window of past speeds, pre-filtered with a
Savitzky-Golay filter. The edge renders and caches the predicted view; it is streamed only if the
predicted position and direction are judged "correct", otherwise the true view is rendered and sent
at normal latency. Predict-then-*render*-ahead is the structural alternative to predicting state and
reconciling afterwards.
*Claims:* per-session tables against two classical baselines — **Lin-A** (linear acceleration from
the latest 3 points) and **Eql-A** (equal acceleration from the latest 2, valid because
"acceleration is approximately equal during a small time interval (e.g. 22 ms)"). Representative
rows, Virtual Museum session 1, RMSE / MAE: body-position x, Lin-A **0.139 / 0.068 mm**, Eql-A
0.079 / 0.037, MLP 0.083 / 0.051, **LSTM 0.061 / 0.035**; head pitch, Lin-A **0.64 / 0.34°**, Eql-A
0.47 / 0.29, MLP 0.51 / 0.35, **LSTM 0.44 / 0.28°**. The authors' own summary of the pattern: the
LSTM has the smallest RMSE for **body** motion in every session, but for **head** motion the MLP
wins in sessions 2 and 3 — the sessions with a large, fast, less-stereotyped range ("head motion can
be up to ±300°/s") — so "MLP is a more feasible model to do head motion prediction in general
cases". Second, downstream metric: **percentage of mismatched pixels**, the fraction of pixels whose
grayscale intensity differs between the user's actual view and the pre-rendered predicted view.
Lin-A is the worst model on it in every session; using the LSTM, "the average *percentage of
mismatched pixels* can be smaller than 1% in VM3 and RM3 sessions".
*Conditions:* **full text read.** Dataset: **over 20 users**, HTC Vive with wireless adaptor and
Lighthouse base stations, two 6DoF applications (Virtual Museum, Virtual Rome), three sessions each
varying guidance and whether a teleport controller was allowed, **840,000 samples**, walkable area
about 3 m x 3 m. Timestamps "appear each 11 ms (corresponding to 90 Hz, which is the refresh rate of
HTC Vive)" — and the model predicts **the next time point**, so **the prediction horizon is 11 ms**,
one 90 Hz sample. That is a fifth of the shortest row in this project's grid and this entry should
not be read as evidence about 50 ms or beyond. 80/20 train/test split with **test viewers disjoint
from training viewers**, MSE loss, 50 epochs. Window length 60 chosen empirically over 40/50/70/80/90.
Render server: Intel Core i7 quad-core with a **GeForce RTX 2060**, WiGig link box; software SteamVR
/ OpenVR SDK plus Unity, with the predictors in **Keras/Python**. **No inference latency is reported
anywhere in the paper**, and no model parameter count. The target it is written against is an
end-to-end budget of **< 20 ms**. The pre-rendering system itself was **not built**: "we will next
implement the full system and demonstrate the feasibility" — so there is **no measured latency
reduction and no user study**, and the "reduction in latency" of the abstract is a design argument,
not a measurement.
*Reading, and it is the reason this retrieval was worth the effort:* at an 11 ms horizon the absolute
gaps between all four models are **tens of micrometres and hundredths of a degree** — far below any
headset's own tracking noise, and on the raw error metric the learned models look like a rounding
difference. On the *rendered-view* metric computed from the same predictions, the same models
separate visibly, with the classical Lin-A worst everywhere. Prior work here reports that the
apparent size of a predictor difference depends strongly on **whether it is scored on state error or
on the rendered consequence of that error** — which is a caution about this file's own §9 table and
about any comparison in this project that scores predictors on position error alone. Note also that
neither baseline is a zero-order hold, so this source says nothing about the `none` row.

---

## 6. The same problem under a different vocabulary: cloud gaming and XR streaming

Orienting note: cloud gaming and remote-rendered XR solved a structurally identical problem — the
image on screen is a function of input the renderer has not received yet — a decade before the
current teleoperation literature, and they did it under production constraints. Two things carry
over. First, **their chosen horizons are short**: Outatime targets 120 ms, Gül et al. chose 60 ms,
and the XR compositor techniques operate inside a single frame. Second, and more usefully, **every
one of them pairs the predictor with an explicit correction mechanism and treats the correction's
visual cost as a first-class design problem** — which is exactly the pairing `docs/metrics.md` §5
enforces. This section is where the prior art on that pairing actually lives.

**Lee, Chu, Cuervo, Kopf, Degtyarev, Grizan, Wolman & Flinn, "Outatime: Using Speculation to Enable
Low-Latency Continuous Interaction for Mobile Cloud Gaming"**, MobiSys 2015.
<https://alecw.azurewebsites.net/work/papers/mobisys-2015-outatime.pdf> (accessed 2026-09-15,
full text read; the ACM DOI page <https://dl.acm.org/doi/10.1145/2742647.2742656> returned
**HTTP 403**, accessed 2026-09-15).
*Proposes:* the server renders **speculative frames of future possible outcomes** and delivers them
one full RTT early. Four mechanisms: (i) a **discrete-time Markov model of navigation**, one model
per component of the 6-D navigation vector; (ii) **supersampling** of input events (poll at ≥100 Hz
for touch, ≥125 Hz for mice, against a 32 ms game tick) purely to reduce sampling noise in the
predictor's input; (iii) **parallel speculation** over a subsampled state space for discrete impulse
events, with time-shifting of events onto the nearest speculated timeline; (iv) **checkpoint and
rollback** plus **image-based rendering** to repair mispredictions.
*Claims:* masks **up to 120 ms** of network latency; responsiveness comparable to a zero-latency
system at up to 120 ms RTT, and "acceptable" up to 250 ms RTT for less demanding games; in-game
skill did not drop as RTT rose to 120 ms. Bandwidth cost **1.97× at 128 ms RTT**, after a video
encoding scheme that recovers 40 % of the bitrate by exploiting spatial coherence between
speculated frames. Their own acceptability threshold for the navigation predictor: "prediction error
below 4° is under the threshold".
*Conditions:* full text read. Two games, **Doom 3** (twitch FPS) and **Fable 3** (action RPG), both
modified — de-randomised RNG, added checkpoint/rollback support. Server: quad-core Intel i7, 16 GB,
**Nvidia GTX 680 4 GB**. Game tick 32 ms (~30 fps); RTTs emulated at fixed values, evaluated
principally at 128 ms and 256 ms. User studies with **64 participants across three studies**, scored
by Mean Opinion Score (1-5), in-game skill proxy, and task completion time. Note the whole design
assumes an **authoritative deterministic simulator that can be rolled back** — this project has no
such thing on the operator side.
*The part that matters most here:* §4.4, "Shake Reduction with Kalman Filtering". The Markov
predictor was accurate, and *still* produced an artifact the authors named **"video shake"**: with
ground-truth yaw constant over three frames, prediction errors of +2°, −3°, +3° make the image jump
5° then 6°, and "the manifested shakiness was sufficiently noticeable so as to reduce playability."
Their fix was to add a Kalman filter *on top of* the predictor, weighting samples against an RTT's
worth of accumulated prediction error. That is a predictor-plus-reconciler pair, discovered
empirically, with the reconciler added because accuracy alone produced an unusable display.

**Application SpaceWarp (Meta Quest), as documented by Unity**, "Understand Application SpaceWarp",
Unity OpenXR Plugin 1.15.
<https://docs.unity3d.com/Packages/com.unity.xr.openxr@1.15/manual/features/spacewarp/spacewarp-overview.html>
(accessed 2026-09-15).
*Proposes:* "Application SpaceWarp synthesizes every other frame using data from previous frames" —
the application renders at half the display rate and the compositor extrapolates the intervening
frame from a **motion vector buffer and a depth buffer** supplied by the engine. This is predictive
display at the pixel level, made cheap by being handed the scene's true motion field instead of
inferring it.
*Claims:* "Meta estimates a 70% improvement in the best cases for Quest headsets." Does not work
below roughly 18 fps.
*Conditions:* vendor/engine documentation, not a measured study — no error metric, no user study, no
numbers beyond the 70 %. The **named failure modes are the valuable part**, because they are a
catalogue of what pixel-level extrapolation cannot do: "frame synthesis can introduce visual
artifacts such as distortion and stuttering"; objects without motion vectors "appear to stutter";
particle systems, unsupported shaders and composition layers are **not warped at all**; transparent
objects break because "the movement of objects behind a transparent object isn't captured"; and
fast-moving objects distort because "the algorithm distorts the background to fill any empty areas".
*Relevance:* this is the technique already running underneath any Quest application, including this
project's. Any operator-side predictor in `Prediction/` is composing with it, not replacing it, and
its extrapolation happens **after** the frame this project's Core produced. That interaction is not
something Core can measure headlessly, and reasoning about it would require `unity/` work — blocked
on human review here, and worth writing down as a known unmeasured confound in M2P rather than
pursued.

**Jacobellis, Ulhaq, Racapé, Choi & Yadwadkar, "DeDelayed: Deleting Remote Inference Delay via
On-Device Correction"**, arXiv:2510.13714 (Oct 2025, rev. Apr 2026).
<https://arxiv.org/abs/2510.13714> (accessed 2026-09-15).
*Proposes:* split the work. A **remote model runs on delayed frames and is trained to predict
anticipated future frames**; a **local model has the current frame** and fuses the remote
prediction into its own output. The two are jointly optimised with an autoencoder that caps the
downlink bitrate.
*Claims:* at **100 ms round-trip delay**, +6.4 mIoU over fully local inference and +9.8 mIoU over
remote inference on streaming video segmentation — "an equivalent improvement to using a model ten
times larger". Code and pretrained models released.
*Conditions:* **arXiv landing page only.** BDD100k driving dataset, one delay value (100 ms). No
inference latency, hardware or model size readable from the abstract. Listed because the
**architecture** is directly analogous to this platform's split: the remote side predicts into the
future, the local side holds fresher information and corrects — which is the reconciler's job stated
as a learning problem rather than a filtering one. If `Reconciliation/` is really the project's most
underestimated axis, this is the shape of prior art that argues so.

**Hirose, Kotoyori, Arunruangsirilert, Lin, Sun & Katto, "Real-time Video Prediction With Fast Video
Interpolation Model and Prediction Training"**, arXiv:2503.23185 (Mar 2025).
<https://arxiv.org/abs/2503.23185> (accessed 2026-09-15).
*Proposes:* IFRVP — video *prediction* built by retraining a **convolution-only frame interpolation**
network (IFRNet) with ELAN-based residual blocks, explicitly to escape the cost of
diffusion/transformer video prediction. "Most of the existing video prediction methods are
computationally expensive and impractical for real-time applications."
*Claims:* "the best trade-off between prediction accuracy and computational speed among the existing
video prediction methods."
*Conditions:* **arXiv landing page only** — the abstract states no latency figure, no hardware, no
model size, no dataset and no accuracy number, which is unfortunate given that speed is its entire
claim. Listed as the cheap-end counterpoint to §3.1: if generative predictive display ever becomes
viable at 90 Hz it will look more like this than like a 13B diffusion model.

---

## 7. Does prior work report an accuracy/correction-cost tradeoff, and is it monotonic?

This project asserts in four places, untested, that **"predicting harder necessarily costs more
correction."** `docs/metrics.md` §5 is built on it; `Prediction/CLAUDE.md` restates it. The question
asked of the literature was narrow: does anyone measure both sides, and if so what shape do they
find. The answer is that **yes, a small dedicated body of work measures both**, it is in HCI rather
than robotics, and **it does not report a single monotonic law — it reports two different
relationships that this project's one sentence conflates.**

The permitted form of the finding, per `docs/literature/CLAUDE.md`: prior work reports the
following under the following conditions. None of it says anything about this system, and settling
it here needs a sweep that holds the reconciler fixed and varies only the predictor.

### 7.1 Monotonic in the horizon, for a *fixed* predictor form — an analytic result

Azuma's transfer-function analysis (§2) gives the mechanism directly. For `x + vp + 0.5ap²` the
magnitude ratio is `sqrt(1 + (1/4)(ωp)⁴)`, which exceeds the ideal 1 for every ω > 0 and p > 0 and
"grows roughly as the square of either p or ω". The excess *is* the amplification of
high-frequency content, and Azuma names its perceptual consequence "jitter": the predicted signal
"appears to shake rapidly". So for **one predictor form, pushed to a longer horizon**, prior work
gives a derived, monotonic cost — and a functional form (quartic in ωp under the radical) far
steeper than linear. This is the strongest support the premise has, and it is analytic rather than
measured, on a linear predictor, for head motion below ~2 Hz.

Outatime (§6) is the empirical instance of the same shape: an accurate Markov predictor still
produced "video shake" severe enough to "reduce playability", and needed a Kalman filter layered on
top. The cost was not an artifact of a bad predictor; it appeared *because* the predictor was
acting, and it scaled with RTT — their filter accumulates error "over variable RTT time steps".

### 7.2 *Not* monotonic across predictor forms at a fixed horizon — a measured result

The UIST source from §1 is the one place found in this review where prediction accuracy and
correction-cost-like metrics are reported side by side, for two predictors, at the same horizon.
Using Nancel et al.'s side-effect metrics extended to 3D, Gamage et al. report for Lateness,
Over-anticipation and Wrong Orientation: **1.21 cm, 1.41 cm, 27.63° for their hybrid model versus
3.03 cm, 2.77 cm, 39.75° for the fifth-order classical model** — reductions of **60 %, 49 % and
30 %**. The hybrid model was simultaneously *more accurate* (79 % lower RMSE at 300 ms) and
*lower cost on all three side-effect metrics*.

That is a direct measured counterexample to "predicting harder necessarily costs more correction",
**under one specific reading of "harder"**. The reading it refutes is "a better predictor
necessarily corrects more". The reading it leaves untouched is "the same predictor asked for a
longer horizon corrects more", which is §7.1's. The distinction is exactly the one the project's
single sentence does not make, and the two readings recommend different experiments:

- If the premise means *horizon*, the falsifying sweep holds the predictor fixed and varies Δ,
  and expects correction cost to rise with Δ.
- If it means *predictor aggressiveness at fixed Δ*, the sweep varies predictor across the
  registry at one Δ, and prior work suggests the ordering may **not** track accuracy — because the
  fifth-order extrapolator's extra cost came from over-anticipation, which is a *fitting* failure,
  not an inherent price of accuracy.

**Both sweeps are cheap and neither has been run.** `docs/metrics.md` §5 already emits everything
needed (`correction_magnitude_mm`, `jerk_mm_s3`, `time_to_convergence_ms`) and
`DisplayedJerkEstimator` already forces a shared population across reconcilers, so the second sweep
is a table, not a feature. This file makes no claim about which way it comes out.

**Two later retrievals complicate the same point on a different axis, and are recorded here so §7.2
is not over-read.** Both are about *accuracy* ordering across predictor forms, not about correction
cost, so neither is evidence for or against the tradeoff §7 is actually asking about — but both show
that "more sophisticated predictor" and "better predictor" come apart at a fixed horizon, which is
the assumption the second reading of the premise rests on.

- §3.4's vector-field source reports that at a **fixed 0.5 s delay on nuScenes**, the *neural*
  predictive-display baseline (NKSR) scores **worse than simply showing the delayed image** on PSNR
  (11.67 versus 15.22) and no better on FSIM (0.65 versus 0.65), while beating it clearly on MS-SSIM
  (0.50 versus 0.41) and TOLM (0.41 versus 0.26). The authors attribute the PSNR collapse to black
  patches where the reconstruction has no surface. So under those conditions a more elaborate
  predictor is simultaneously better and worse than no prediction **depending on which metric is
  read** — and the delayed image is that paper's `none`.
- §5's newly retrieved GLOBECOM source reports that at a **fixed 11 ms horizon**, an encoder-decoder
  LSTM has the lowest RMSE for body position in every session, while a plain two-layer MLP beats it
  for head pose in the two sessions with the widest, fastest motion. Capacity ordering and accuracy
  ordering disagree, and they disagree **as a function of the motion regime rather than the horizon**
  — the same tail-versus-median distinction §5's ESKF entry raises.

Neither of these says anything about this platform. What they jointly recommend is that the
fixed-Δ, vary-predictor sweep in the second bullet above be read **per metric and per motion regime**
rather than as a single ordering, because prior work reports the ordering changing under both.

### 7.3 The metrics prior work uses for the cost side

**Nancel, Vogel, De Araujo, Jota & Casiez, "Next-Point Prediction Metrics for Perceived Spatial
Errors"**, UIST '16, pp. 271-285, DOI 10.1145/2984511.2984590. Project page
<https://gery.casiez.net/turbotouch/predictionmetrics/> and author page
<https://mathieu.nancel.net/> (both accessed 2026-09-15, both carrying the full abstract). The
Inria HAL PDF <https://inria.hal.science/hal-01420670/file/NextPointPredictionMetrics.pdf> and the
HAL landing page <https://hal.science/hal-01420670> both returned **HTTP 403 (Anubis
anti-scraping)**; the author redirect `gery.casiez.net/acm.php?id=N25028` returned **HTTP 404**.
*Proposes:* metrics quantifying the probability of **seven spatial-error "side-effects"** caused by
next-point prediction, computable **from input logs alone** — "These metrics enable practitioners to
compare next-point predictors using only input logs." The seven categories, as named in secondary
descriptions and partially confirmed by the UIST '21 source which extends three of them: *lateness,
over-anticipation, wrong distance, wrong orientation, jitter, jumps, spring effect*.
*Claims:* the metrics "correlate positively with the frequency of perceived side-effects."
*Conditions:* **abstract only — the full text could not be retrieved by any of the fifteen routes
now recorded in §11.2**, across two passes: ACM, HAL (PDF, landing page and `hal.inria.fr` mirror,
all Anubis), the dead author redirect, five author and lab pages, three aggregators, and the
Wayback Machine, which this environment cannot fetch at all. Semantic Scholar confirms the paper is
green OA deposited **only** in HAL, so there is no author copy to find. Known
from the abstract: 12 participants; drawing, dragging and panning tasks; **5 state-of-the-art
next-point predictors**; **2D touchscreen**, not 3D and not VR. Correlation coefficients, the
predictors' identities, the latency values and whether side-effects rose monotonically with
prediction distance are all **unknown from this review** and should not be assumed.
*Why it is the most important entry in this section:* it is a **log-computable** cost metric family,
which is precisely the constraint this platform operates under — everything is scored offline from
a `.tlog`. Three of the seven (lateness, over-anticipation, wrong orientation) have already been
extended to 3D by the UIST '21 source, with a worked comparison. This project's correction-cost
metrics are currently magnitude, rate, jerk and time-to-convergence; *over-anticipation* and *wrong
orientation* are not expressible in those four and are exactly the failure modes
`Prediction/CLAUDE.md` predicts for `const-accel` ("overshoots on direction reversal"). **Adding a
metric requires defining it in `docs/metrics.md` in the same change, which this survey run is not
permitted to do — recorded here as a proposal for a human, not as a change.**

**Wiese & Henze, "Predicting Mouse Positions Beyond a System's Latency Can Increase Throughput and
User Experience in Linear Steering Tasks"**, Mensch und Computer 2023, DOI 10.1145/3603555.3603556.
ACM <https://dl.acm.org/doi/fullHtml/10.1145/3603555.3603556> returned **HTTP 403**; verified via
the authors' replication repository
<https://github.com/ragor114/MUC23-Predicting-Mouse-Positions-Beyond-a-Systems-Latency>
(accessed 2026-09-15), which carries the title, venue, architecture, study design and results.
*Proposes:* deliberately over-predicting — predicting *past* the point where latency is merely
cancelled, into negative effective latency — and measuring whether users benefit.
*Claims:* an ensemble of fully-connected feed-forward ANNs trained with MAE loss, predicting
**100 ms ahead**; with effective latency swept from **+50 ms to −50 ms**, "decreasing latency beyond
the system's latency increases throughput up to −50ms", while "subjective measures improved up to
−16.67ms without negative effects on agency".
*Conditions:* **repository README and search metadata only; the paper itself was not retrievable.**
60 participants for data collection, 30 for the evaluation study. Task is a **2D linear Steering
Law task with a mouse** — not 3D, not VR, not a robot. Alternative architectures (RNN, Transformer)
were tried in hyperparameter search and the feed-forward ensemble won; no inference cost or model
size was readable.
*The shape, which is the point:* **objective performance and subjective cost have different optima**
— throughput kept improving to −50 ms while the subjective measures stopped improving at −16.67 ms.
Prior work here reports a tradeoff that is neither absent nor a single monotonic curve, but two
curves that peak in different places. If that shape reproduces on this platform it would be the
strongest possible argument for the project's standing rule that prediction error and correction
cost are never reported apart — and the opposite of a reason to pick one number.

---

## 8. Two more, at the long-horizon and cheap-inference ends

**Mondal & Wong, "User Head Movement-Predictive XR in Immersive H2M Collaborations over Future
Enterprise Networks"**, arXiv:2507.15254 (21 Jul 2025). <https://arxiv.org/abs/2507.15254>, full
text read at <https://arxiv.org/html/2507.15254v1> (both accessed 2026-09-15).
*Proposes:* predict the operator's head movement ahead with a **bidirectional LSTM** and use the
prediction to **orient the remote machine's camera in advance** — human-to-machine collaboration
over an enterprise network. Note this is prediction used to steer a physical remote actuator, not
to warp a local image.
*Claims:* **90 ms ahead** (6 samples), **normalised RMSE around 10 %** at a head-movement speed of
180°/s.
*Conditions:* full text read. BiLSTM with **3 layers, 200 hidden units per layer, dropout 0.3**;
parameter count not stated. Trained on ~10⁶ own samples plus ~30×10⁶ from an external dataset.
HMD packet inter-arrival modelled as Gamma(51.844, 0.273), mean **14.13 ms ⇒ ≈ 70 Hz**. Network
budget assumed **10-30 ms** end-to-end for 4K, ≤ 8 ms for ideal QoE. **Inference "a few
micro-seconds only" per sample — but on "Edge-AI servers", not on the headset**; training took 6-8
hours on an i7-1065G7 with a GTX 1650.
*Reading:* the only source in this review that gives both a per-sample inference figure and a
horizon, and it is favourable — microseconds against a 90 ms horizon, comfortably inside Azuma's
break-even. The caveat is that "a few microseconds" for a 3×200 BiLSTM is a server-GPU batched
number; the same model called once per frame on a mobile SoC is a different measurement, and the
paper does not make it.

**Li, Zhang, Liu & Wang, "Very Long Term Field of View Prediction for 360-degree Video Streaming"**,
arXiv:1902.01439 (4 Feb 2019). <https://arxiv.org/abs/1902.01439> (accessed 2026-09-15).
*Proposes:* FoV prediction as a sequence-learning problem at horizons of **more than seconds**,
predicting the target user's future field of view from their own past trajectory **and from other
users' future FoV locations** — i.e. escaping the single-trajectory information limit by borrowing
a population prior. Two representations: FoV-centre trajectories and equirectangular heatmaps.
*Claims:* significantly outperforms benchmark models; "other users' FoVs are very helpful for
improving long-term predictions."
*Conditions:* **arXiv landing page only** — horizons in seconds, per-horizon accuracy, baselines
and datasets ("two public datasets") were not readable. Included for one structural reason that
does transfer: at long horizons the useful information stopped coming from the signal's own
derivatives and started coming from **a prior over what people do**. That is the same move
Prescient teleoperation makes with demonstrations (§4) and the same move the UIST hybrid makes with
regressed coefficients (§1) — and it is the generalisation of §4's orienting note across three
otherwise unrelated fields.

---

## 9. What runs where — the compute separation, stated explicitly

The brief asked to keep datacentre-GPU methods and mobile-SoC methods explicitly separate. Collated
from the entries above; **every figure is the source's, measured on the source's hardware, and none
of it is evidence about this platform.** Blank means the source does not state it, which is itself
the finding for several rows.

| Method (§) | Horizon | Model size | Inference cost | Hardware measured on |
|---|---|---|---|---|
| Zero-order hold (`none` equiv.) | any | 0 | ~0 | — |
| Double exponential smoothing (§2) | 50, 100 ms | 2 params | **≈ 2 µs** | AMD Athlon XP 1800+, 2003 |
| KF / EKF (§2) | 50, 100 ms | — | **≈ 456 µs** | AMD Athlon XP 1800+, 2003 |
| High-order ESKF + classifier (§5) | < 100 ms | — | not stated | Apple M1, Python |
| Hybrid classical+regressed kinematics (§1) | ≤ 340 ms | 5 coefficient functions | not stated; "low computational overhead" | not stated |
| TA-GNN finger prediction (§5) | 40-400 ms | **1.49 M params** | **not stated anywhere** | not stated |
| BiLSTM head prediction (§8) | 90 ms | 3×200 BiLSTM | "a few micro-seconds" per sample | Edge-AI server (not headset) |
| ProMP whole-body command prediction (§4) | **1.5-2 s** | 12-42 demos | not stated | iCub control PC, 100 Hz loop |
| PiLSTM bilateral predictors (§4) | 100 ms step, 1.25 s delay | 2-4 LSTM layers, 75-188 units | **not stated** | not stated |
| USST egocentric hand forecasting (§5) | ~0.4-0.67 s | ViT-backed, ~AgentFormer size | "ms per video" | RTX GPUs |
| Pix2Pix cGAN frame prediction (§3) | 500 ms | cGAN | **not stated** | not stated |
| SVD 1.1 video rollout (§3) | 587 ms | ~1.5 B | **10.5-12.3 s** | RTX 6000 Ada 48 GB |
| LTX-Video 13B rollout (§3) | 587 ms | 13 B | **29.8-30.8 s** | RTX 6000 Ada 48 GB |
| Vector-field PD, Lidar+camera (§3.4) | 0.5-0.75 s | 6-12 polynomial coefficients | **0.02 s / 0.044 s** per image | 24-core Intel i9, RTX 4090 24 GB |
| Enc-dec LSTM / MLP 6DoF pre-render (§5) | **11 ms** (one 90 Hz sample) | 2x60 LSTM units; 2 FC layers | **not stated** | Keras/Python; render server i7 + RTX 2060 |

Four things fall out of the table that are worth stating plainly.

1. **The compute range spans nine orders of magnitude** — 2 µs to 30 s — for methods all described
   in their own papers as "predictive display". The phrase does not identify a cost class, so no
   argument of the form "predictive display works, therefore X" survives contact with the table.
2. **The learned methods that are plausibly affordable do not report their cost.** TA-GNN is the
   best-fitting candidate in this whole review for a `seq-model` row — right signal, right horizon
   grid, right baseline, 1.49 M parameters — and states **no runtime at all**. Given Azuma's
   break-even argument, a `seq-model` implementation should be treated as having an *unknown*
   budget until measured, which on this platform means measured through `IInferenceBackend` on the
   target, not on a workstation.
3. **Nothing in the generative-video family is within four orders of magnitude of a 90 Hz frame.**
   The cheapest measured rollout is ~10.5 s of compute for ~0.59 s of video. Any pursuit of §3 on
   this platform is a research programme about inference optimisation, not an algorithm row.
4. **The only pixel-domain method here that meets its own real-time bar does no learning at all.**
   §3.4 warps the delayed frame with a six-coefficient ridge regression in 0.02-0.044 s, and its
   authors reject kernel regression and neural interpolation explicitly on cost. Note what that
   still means for this platform: 0.02-0.044 s is 23-50 Hz **on an RTX 4090**, so even the cheap
   end of pixel-domain predictive display is 2-4x outside a 90 Hz budget before any mobile-SoC
   penalty. The row that is affordable in this table is the one predicting *state*, not pixels.

---

## 10. The two nearest neighbours — XR + robot arm + emulated network

Orienting note: these are the closest published systems to this platform's actual configuration
(headset, remote manipulator, emulated impairment). Both are worth reading for how they chose to
evaluate, and both are instructive for what they chose *not* to measure.

**Zhang, Liu & Kim, "Understanding and Mitigating Network Latency Effect on Teleoperated-Robot with
Extended Reality"**, arXiv:2506.01135 (1 Jun 2025, rev. 5 Jun). <https://arxiv.org/abs/2506.01135>,
full text read at <https://arxiv.org/html/2506.01135> (both accessed 2026-09-15).
*Proposes:* TeleXR — "the first end-to-end, fully open-sourced XR teleoperation framework that
decouples robot control and XR visualization from network dependencies", reconstructing delayed or
missing counterpart state from **local sensing** so both ends run concurrently with transmission.
Reconstruction is by **Extended Kalman Filter with an estimation window sized from the timestamp
difference** between the latest received pose and the current user pose — i.e. an explicitly
**adaptive horizon** driven by measured staleness, not a fixed Δ. Also: contention-aware GPU
scheduling and bandwidth-adaptive point-cloud scaling. They define **motion-to-motion (M2M)
latency** — user's latest motion to corresponding robot feedback.
*Claims:* significant reduction of network-induced error and mission time; see conditions.
*Conditions:* full text read, and the gaps are the story. Hardware: **Northstar Next XR headset and
a Kinova Gen3 manipulator**, with NVIDIA Jetson Xavier (XR side) and Orin Nano (robot host) as
embedded alternatives. Network conditions emulated: 5 GHz 802.11ac, 802.11b 2.4 GHz, 4G LTE, 5G,
and a cloud VPN path. But **no before/after M2M latency numbers in ms, no quantitative
teleoperation-error metric, no mission-completion-time measurements, and no human subjects** —
evaluation is hardware profiling plus network simulation. The one concrete compute figure is a
scheduling example, 27 → 14 time units.
*Why it is worth reading anyway:* (i) the adaptive-horizon EKF, keyed on measured staleness, is a
concrete design this project's `IPredictor.Predict(targetTicks)` signature already admits and no
implemented predictor uses; (ii) the hardware is a Jetson-class robot host, the same class as this
project's, which makes its (unstated) compute budget the right one to care about; (iii) it is a
reminder that "first fully open-sourced framework" and "measured result" are different claims.

**Deng & Yang, "Residual Reinforcement Learning for Robot Teleoperation under Stochastic Delays"**,
arXiv:2605.15480 (14 May 2026). <https://arxiv.org/abs/2605.15480>, full text read at
<https://arxiv.org/html/2605.15480v1> (both accessed 2026-09-15).
*Proposes:* an **LSTM state estimator** that reconstructs smooth continuous state from delayed
observations, feeding a **residual RL policy** that learns a torque correction on top of a nominal
controller. The framing is explicitly about the tradeoff: delayed observations cause
"high-frequency chattering", and the policy is meant to "balance tracking accuracy with velocity
smoothness".
*Claims:* under **high delay, high variance (total 90-290 ms)**, Cartesian endpoint error mean
0.037 m / max 0.063 m in simulation and mean 0.045 m / max 0.078 m on hardware; outperforms
state-of-the-art baselines.
*Conditions:* full text read. Three delay regimes, all uniform: low/low U(120,160) ms → 170-210 ms
total; high/low U(200,240) ms → 250-290 ms; high/high U(40,240) ms → 90-290 ms. **Uniform delay
only — no burst loss, no reordering, no heavy tail**, which is a much gentler impairment model than
this project's profiles. LSTM: hidden 256, 3 layers, input sequence 150 steps (600 ms),
autoregressive rollout horizon **245 steps ≈ 980 ms**. Control at **250 Hz (Δt = 4 ms)**.
**No inference cost or deployment hardware is reported** despite a 250 Hz loop. Robot: Franka
Panda. Crucially, evaluation is closed-loop but **not with a human** — "a desired reference
trajectory for the end-effector is commanded to the leader robot", i.e. a synthetic operator.
*The observation worth carrying:* the abstract sells an accuracy/smoothness balance, and **velocity
smoothness is never independently measured** — there is no smoothness metric, only reward weights
(λp, λv, λa) = (10, 5, 0.01) folded into training. A paper can name the tradeoff this project cares
about and still report only one side of it. That is an argument *for* `docs/metrics.md` §5's
existence and for Gate 5's rule that the two are always reported together, not merely a criticism
of one paper.

---

## 11. Negative results of the search itself

Recorded so the next person does not spend the window repeating them.

### 11.1 Searches that found nothing useful

- `"joint evaluation predictor and reconciliation smoothing policy teleoperation percentile distribution correction cost sweep"` —
  **no work found that evaluates a predictor and a reconciler/smoother as a factorial pair.** Every
  source in this file either fixes the smoother and varies the predictor, or (more often) bundles
  them into one method and reports one number. The coupled-axes discipline in this project's root
  `CLAUDE.md` has, as far as this review reached, **no prior art to borrow from**. That is a reason
  to run the sweep, and a reason not to expect published numbers to decompose.
- `"motion prediction VR report p95 p99 tail percentile error distribution instead of RMSE mean predictor evaluation"` —
  **no VR motion-prediction work found that reports tail percentiles.** The field reports RMSE, MAE
  and MSE, essentially always as means over a dataset. Two consequences: (i) the published numbers
  in this file are **means and are not comparable to this project's p50/p95/p99 by construction**,
  which is a stronger incomparability than the usual "different system" caveat; (ii) §5's observed
  20× spread between easy and hard head-motion classes is invisible in every headline number quoted
  above.
- `"on-device inference latency LSTM head pose prediction Quest mobile SoC milliseconds per inference"` —
  **nothing found measuring a motion predictor's inference cost on a headset-class SoC.** Results
  were generic mobile-inference benchmarking, not motion prediction. The one per-sample figure found
  anywhere (§8, "a few micro-seconds") was measured on an edge server.
- `"LSTM transformer head motion prediction VR remote rendering prediction horizon 100ms MAE comparison Kalman"` and
  `"LSTM head motion prediction 360 video streaming look-ahead time 100ms 500ms accuracy degradation deep learning"` —
  no source surfaced that tabulates learned versus Kalman versus double-exponential **at matched
  horizons on matched data**. TA-GNN (§5) is the closest thing found, and its classical comparison
  is to a 2007 model rather than to a Kalman filter or to double-exponential smoothing. **The direct
  comparison this project would most want from the literature does not appear to exist.**
- `"Next-Point Prediction Metrics for Perceived Spatial Errors" Nancel Vogel Casiez pdf`, plus
  per-co-author variants for Vogel (Waterloo), De Araujo (Toronto/INESC-ID) and Jota (Tactual Labs)
  — **no copy of this paper exists outside ACM and HAL.** Every result across five queries resolved
  to one of: the ACM DOI, the HAL record, the authors' project page, ResearchGate, Semantic Scholar,
  or the conference talk video. The technique that recovered the UIST '21 source in §1.1 — search the
  exact title, find a gold-OA author copy at a lab or corporate site — was applied here and **failed
  for a structural reason worth recording**: that paper is gold OA and this one is *green*, deposited
  only in HAL, and HAL is behind Anubis anti-scraping. Semantic Scholar confirms it:
  `isOpenAccess: true`, `openAccessPdf` = the HAL landing page, status `GREEN`. Five author and lab
  pages were then fetched directly (§11.2); the two that are still live list the DOI and nothing else.
  **Green OA in a single bot-walled repository is functionally unretrievable from this environment**,
  and that is a different failure from a paywall.

### 11.2 Sources that could not be retrieved

| Source | URL tried | Result |
|---|---|---|
| Gamage et al., UIST '21 | `dl.acm.org/doi/10.1145/3472749.3474753` | HTTP 403 — **but obtained elsewhere, see §1.1** |
| Azuma & Bishop, SIGGRAPH '95 | `cs.unc.edu/~azuma/vrais95.pdf` | HTTP 404 |
| Azuma & Bishop, SIGGRAPH '95 | `cs.unc.edu/~azuma/s95paper.pdf` | HTTP 403 |
| Moniruzzaman et al., JFR 2023 | `onlinelibrary.wiley.com/doi/full/10.1002/rob.22135` | HTTP 403 (listed as CC-BY hybrid OA) |
| Moniruzzaman et al., JFR 2023 | `ro.ecu.edu.au/cgi/viewcontent.cgi?article=2469&...` | HTTP 403 |
| Moniruzzaman et al., Adv. Intell. Syst. | `advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202200439` | HTTP 403 |
| Nancel et al., UIST '16 | `inria.hal.science/hal-01420670/file/NextPointPredictionMetrics.pdf` | HTTP 403, Anubis anti-scraping |
| Nancel et al., UIST '16 | `hal.science/hal-01420670` | HTTP 403, Anubis (error id `9e4edb5b6b850c41`) |
| Nancel et al., UIST '16 | `gery.casiez.net/acm.php?id=N25028` | HTTP 404 (reached via 301 from `cristal.univ-lille.fr`) |
| Wiese & Henze, MuC '23 | `dl.acm.org/doi/fullHtml/10.1145/3603555.3603556` | HTTP 403 |
| Hou et al., GLOBECOM '19 | `ieeexplore.ieee.org/document/9014097/` | 200 but **empty body**; no readable content — **but obtained from the authors' lab-hosted copy at `esdat.ucsd.edu`, see §5** |
| Sharma, Calder & Rajamani, IJCV 2025 | `link.springer.com/article/10.1007/s11263-025-02550-z` | HTTP 303 to an identity-provider auth URL |
| Sharma, Calder & Rajamani, IJCV 2025 | `link.springer.com/content/pdf/10.1007/s11263-025-02550-z.pdf` | HTTP 303 to the same identity provider, **despite Crossref recording a `vor` CC-BY 4.0 licence** — **but obtained in full from NSF-PAR, see §3.4** |
| Azuma & Bishop, SIGGRAPH '95 | `dl.acm.org/doi/10.1145/218380.218496` | HTTP 403 |
| Lee et al., Outatime (ACM copy) | `dl.acm.org/doi/10.1145/2742647.2742656` | HTTP 403 — **but obtained from the author copy, see §6** |
| Meta, mobile TimeWarp documentation | `developers.meta.com/horizon/documentation/native/android/mobile-timewarp/` | HTTP 404 |
| Meta, "Introducing Application SpaceWarp" blog | `developers.meta.com/horizon/blog/introducing-application-spacewarp/` | 200 but body was the site title only; JS-rendered, no readable text |
| Nancel et al., UIST '16 | `dl.acm.org/doi/pdf/10.1145/2984511.2984590` | HTTP 403 (accessed 2026-09-16) |
| Nancel et al., UIST '16 | `hal.inria.fr/hal-01420670/document` | HTTP 403, Anubis (accessed 2026-09-16) |
| Nancel et al., UIST '16 | `api.semanticscholar.org/graph/v1/paper/DOI:10.1145/2984511.2984590` | 200; `isOpenAccess: true` but `openAccessPdf` is the HAL landing page, status `GREEN` — **the index knows of no copy outside HAL** |
| Nancel et al., UIST '16 | `api.archives-ouvertes.fr/search/?q=halId_s:hal-01420670&fl=files_s,fileMain_s` | 200 — HAL's *API* is not behind Anubis, but every file URL it returns is |
| Nancel et al., UIST '16 | `mathieu.nancel.net` (first author) | 200; links only the dead `acm.php` redirect and the DOI, no local PDF |
| Nancel et al., UIST '16 | `loki.lille.inria.fr/publications.html` (the authors' lab) | 200; links ACM DOI, HAL and YouTube only |
| Nancel et al., UIST '16 | `gery.casiez.net/turbotouch/predictionmetrics/` (project page) | 200; links the ACM DOI and a participant-data + source-code ZIP, **no paper PDF** |
| Nancel et al., UIST '16 | `gery.casiez.net/` and `gery.casiez.net/publications/` | empty body / no readable content |
| Nancel et al., UIST '16 | `nonsequitoria.com/index.php` and `nonsequitoria.com/` (Vogel) | HTTP 404; root returns a one-word body |
| Nancel et al., UIST '16 | `hci.cs.uwaterloo.ca/publications` (Vogel's lab) | 200; **the entry is listed, with the DOI and no PDF** |
| Nancel et al., UIST '16 | `cs.toronto.edu/~brar/` → `bdearaujo.github.io` (De Araujo) | 301; paper not in the list. `bdearaujo.com/publications/` HTTP 404 |
| Nancel et al., UIST '16 | `core.ac.uk/search?q=...` | HTTP 403 |
| Nancel et al., UIST '16 | `base-search.net/Search/Results?lookfor=...` | HTTP 403, Anubis |
| Nancel et al., UIST '16 (and Hou) | `web.archive.org/web/2023/<dead url>` | **the fetch tool refuses `web.archive.org` outright** — not a server error; the Wayback route does not exist from this environment |
| Moniruzzaman et al., JFR 2023 | `onlinelibrary.wiley.com/doi/pdfdirect/10.1002/rob.22135` — the exact URL Semantic Scholar records as `openAccessPdf`, status `HYBRID` | HTTP 403 |
| Moniruzzaman et al., JFR 2023 | `scispace.com/pdf/long-future-frame-prediction-...-3gpdnj3g.pdf` and `scispace.com/papers/...` | 200 but empty body |
| Moniruzzaman et al., JFR 2023 | `typeset.io/papers/long-future-frame-prediction-...` | 301 → scispace, empty body |
| Moniruzzaman et al., JFR 2023 | `api.openaire.eu/search/publications?doi=10.1002/rob.22135` | 200; records **only** DBLP and Crossref instances — no repository full text anywhere in the OpenAIRE graph |
| Moniruzzaman et al., JFR 2023 | `ro.ecu.edu.au/do/search/?q=...` | HTTP 403 — the repository's browse pages (`/ecuworks2022-2026/index.NN.html`) fetch fine, its `cgi/` and `do/` paths do not |
| Moniruzzaman, PhD thesis, ECU 2023 | `ro.ecu.edu.au/theses/2644/` | 200, landing page readable, but **"Access to this thesis is restricted"**; the embargo notice on the page reads 12 April 2024 and the file is still not served |
| Moniruzzaman et al., JIRS 2022 (companion, CC-BY `vor` per Crossref) | `link.springer.com/content/pdf/10.1007/s10846-022-01749-3.pdf` | HTTP 303 to the Springer identity provider |
| Moniruzzaman et al., Adv. Intell. Syst. (companion, CC-BY `vor` per Crossref) | `advanced.onlinelibrary.wiley.com/doi/pdf/10.1002/aisy.202200439` | HTTP 403 |
| (general) dblp author pages | `dblp.org/pid/170/8513.html` | HTTP 403, Anubis — dblp is no longer fetchable either, though its keys remain correct where already recorded |

### 11.3 Process notes worth keeping

- **arXiv PDFs above ~10 MB exceed the fetch tool's content limit** (`arxiv.org/pdf/2107.01281` and
  `arxiv.org/pdf/2402.16587` both failed this way). The working route for full text is
  `arxiv.org/html/<id>v<n>` for recent papers and `ar5iv.labs.arxiv.org/html/<id>` for older ones.
  Both worked every time they were tried here.
- **The author-copy technique has a second, more durable form: the *lab* page, not the personal
  page.** §5's GLOBECOM source was recovered by fetching the advisor's group site root
  (`esdat.ucsd.edu`), then its `/publications` page, and reading the link list — the PDF sits under
  `/sites/default/files/publications/`. Every *personal* homepage tried this pass was either dead
  (`nonsequitoria.com/index.php` 404, `bdearaujo.com/publications/` 404, `gery.casiez.net/` empty)
  or listed DOIs only. Institutional group pages outlive their members' personal sites. Try them
  first.
- **NSF-funded work has a second home that publishers do not control.** §3.4's IJCV paper is CC-BY
  and Springer still served nothing but a cookie-auth redirect; the full text is on
  `par.nsf.gov`, found by searching `par.nsf.gov/search/term:"<exact title>"` and following the
  `biblio/<id>` landing page to `servlets/purl/<id>`. The cheap signal that predicted this route
  would exist was **Crossref's `funder` and `license` arrays**. The same route found nothing for
  §5's GLOBECOM source, which acknowledges UCSD's Center for Wireless Communications rather than a
  federal grant — so PAR is worth one call when a US funder is named and worth skipping otherwise.
- **`api.crossref.org/works/<doi>` is the cheapest decider before spending routes.** It says whether
  a `vor` CC-BY exists at all. It said yes for §3.4's paper (which was then recoverable) and yes for
  *both* Moniruzzaman companions (which were not). **An open licence does not imply an open server**,
  and that distinction is worth making explicitly in this file because "listed as CC-BY" appears
  three times in §11.2 next to an HTTP 403.
- **Several aggregators that a previous pass could have used are now bot-walled.** `dblp.org`,
  `base-search.net` and `core.ac.uk` all returned 403 this pass, the first two serving the same
  Anubis page HAL serves. The metadata routes still open are `api.crossref.org`,
  `api.semanticscholar.org`, `api.openaire.eu`, `api.archives-ouvertes.fr` and
  `export.arxiv.org/api/query` — all of which return identifiers and URLs, none of which return full
  text. Plan retrievals as "metadata API to find a URL, then fetch a *host* that is not a publisher
  or an aggregator".
- **Every identifier used this pass came from a fetched response, never from inference.** DOIs came
  from Crossref, Semantic Scholar, OpenAIRE or a page's own link list; the NSF-PAR id `10635373`
  came from PAR's own search results. This is the direct application of the previous pass's
  scuba-diving lesson below, and it cost nothing.
- **Do not guess DOIs.** A plausible-looking guess of `10.1145/2984511.2984519` for the Nancel UIST
  '16 paper resolved to a real but entirely different paper — an immersive scuba-diving simulator.
  The correct DOI (`10.1145/2984511.2984590`) came from the authors' own project page. A guessed
  identifier that resolves is worse than one that 404s, because it looks verified.

---

## 12. What this suggests for the open rows — candidates, not conclusions

Nothing below binds anything (`docs/literature/CLAUDE.md`: "Nothing here binds anything"). Each item
is a candidate experiment with the prior-work reason attached, ordered by how cheaply it could be
falsified.

1. **Sweep the existing three predictors across Δ ∈ {50, 100, 200, 400} ms and look for an order
   inversion.** Prior work (§1.3) reports that each added derivative helps at short horizons and
   increasingly hurts at long ones, and (§5) that a zero-velocity baseline *beats* a learned model at
   20 ms while losing by 64 % at 200 ms. If an inversion between `none`, `const-vel` and
   `double-exp` exists on this project's traces, it is visible with **zero new code** — the
   predictors, the grid and the metrics all exist. Falsifier: the ordering is horizon-invariant.
   This is the cheapest experiment in the file and it tests the most transferable published claim.

2. **Decide which reading of "predicting harder costs more correction" is being asserted, then run
   the corresponding sweep (§7).** Horizon-at-fixed-predictor and predictor-at-fixed-horizon are
   different claims; prior work supports the first analytically (§7.1) and provides a measured
   counterexample to the second (§7.2). Both sweeps use only existing metrics. Falsifier for the
   first: correction cost flat or falling in Δ. Falsifier for the second: correction cost tracks
   accuracy monotonically across predictor rows at fixed Δ.

3. **`const-accel` before `seq-model`.** §1.3 says a second-order term should *win* below ~160 ms and
   *lose badly* above it. That is a sharp, cheap, falsifiable prediction about a row already in
   `Prediction/CLAUDE.md`'s planned table, and it is the most informative classical row left.

4. **An adaptive-horizon predictor keyed on measured staleness (§10).** TeleXR sizes its EKF
   estimation window from the timestamp gap between the freshest received pose and now.
   `IPredictor.Predict(long targetTicks)` already permits this and no implemented predictor exploits
   it; under variable delay a fixed-Δ predictor is over- or under-extrapolating almost every frame.
   This is a `Prediction/` row, not an architecture change.

5. **If `seq-model` is built, TA-GNN (§5) is the closest published template** — right signal class,
   right horizon grid, explicit zero-velocity baseline, 1.49 M parameters, and a physics/kinematics
   encoder whose ablation is worth 18 % MAE. Two cautions carried from the source: it reports **no
   runtime at all**, and it gives **no benefit at 20-40 ms**. Any `seq-model` work should measure
   inference cost through `IInferenceBackend` **first** and treat accuracy as the second question —
   Azuma's break-even (§2) says a predictor costing a meaningful fraction of its own horizon has
   already lost.

6. **The Legendre Delay Network (§4) is the one learned-adjacent structure in this review that looks
   compatible with the Core invariants** — fixed-size linear state space, no allocation, no
   framework, deterministic. Worth a closer read before assuming `seq-model` must mean a neural
   network behind `IInferenceBackend`.

7. **Generative predictive display (§3) should be recorded as out of reach on cost, not untried.**
   The cheapest measured configuration is ~10.5 s of RTX 6000 Ada compute for ~587 ms of video. This
   is the kind of thing `Tried and rejected` sections exist for, except that nobody here tried it —
   so the honest place for it is here, as a reason not to start.
   A later retrieval sharpens this rather than softening it: §3.4's non-generative,
   non-learned warping method runs in 0.02-0.044 s per image, i.e. 23-50 Hz **on an RTX 4090**, and
   is the fastest pixel-domain predictive display in this file by two to three orders of magnitude.
   Even that is 2-4x outside a 90 Hz budget on hardware far above a Jetson. The cost gap in this
   family is not a diffusion problem; predicting pixels at all is the expensive choice.

### Two things this file believes should change in files it may not touch

Recorded as arguments for a human, per the instruction to write them here rather than act on them.

- **`docs/metrics.md` §5 arguably wants an over-anticipation / wrong-orientation metric.** The four
  current correction-cost metrics (magnitude, rate, jerk, time-to-convergence) cannot distinguish
  *late* from *overshooting*, and prior work (§7.3) defines a log-computable family that can, three
  of whose members have already been extended to 3D with a worked comparison (§1.2, §7.2). This is
  exactly the failure mode `Prediction/CLAUDE.md` predicts for `const-accel`, so the metric gap and
  the next planned predictor row are the same gap. **Adding a metric requires defining it in
  `docs/metrics.md` in the same change — out of scope for a survey run, and it must not be done by
  redefining an existing metric.**
- **`Prediction/CLAUDE.md`'s `double-exp` row deserves a provenance note.** Its source (§2)
  established accuracy parity with KF/EKF **only at 50 and 100 ms, on ≈ 1 Hz-bandlimited CAVE
  tracker data from 2003**. The row currently reads "strong baseline for head/hand pose" with no
  horizon qualification, and the project sweeps it to 400 ms. This is not a claim that the row is
  wrong — it is a claim that its published warrant does not extend to half the grid, and that the
  project's own sweep is the only thing that can extend it.

### The one-paragraph answer to "what works, at what horizon, at what compute cost"

Below ~100 ms, **nothing beats cheap classical extrapolation by enough to pay for itself** — double
exponential smoothing matched EKF at 135× lower cost in 2003 (§2), a zero-velocity hold beat a 1.49 M
parameter GNN at 20 ms in 2025 (§5), and the two production-shaped remote-rendering systems in this
review chose horizons of 60 ms and 120 ms (§2, §6). Between ~160 ms and ~340 ms, **unconditioned
extrapolation degrades exponentially while fitted and learned models degrade roughly linearly**, and
this is the band where learning earns its keep (§1, §5) — at a compute cost that the literature
mostly declines to state (§9). Beyond ~400 ms, **no method in this review predicts unconditioned
human motion usefully**; everything that works at 1-2 s works by conditioning on a prior over what
the operator is *doing* — demonstrations, task structure, goal identity, or other users' behaviour
(§4, §8) — which is a different algorithm with a different failure mode, and in at least one case
(§4) one that cannot be honestly scored from a recording at all.
