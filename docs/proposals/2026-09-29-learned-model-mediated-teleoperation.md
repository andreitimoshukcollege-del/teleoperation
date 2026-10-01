# Learned model-mediated teleoperation

**Proposal, drafted 2026-09-29. Not an ADR and not a result.** Nothing described here has been
built or measured, and nothing here binds code. The first step of the plan is the ADR that would.

A world model on a slow loop estimates what is around the arm. The existing 90 Hz predictors use
that estimate to foresee contact and payload effects that the delayed operator cannot see yet.

## What we propose

Today the operator sees the arm where a predictor extrapolates it from delayed robot poses. Near an
obstacle or after a grasp, that prediction fails in a foreseeable way. It carries the arm through
the surface and ignores the load, then corrects when the truth arrives. We propose a second, slow
loop that watches the workspace through a fixed third-person camera and publishes a small
description of the environment. The fast loop in Core applies that description to every frame's
prediction.

```text
 SLOW LOOP · robot site · RTX 4090 · about 1 Hz · output is seconds old
 camera ─> environment model ─────────────> environment state
                                            surfaces, held object, payload,
                                            confidence, source-frame time
                                                   │
                                                   │ small message over the impaired link,
                                                   │ same emulation as the pose stream
                                                   ▼
 delayed pose ─> command-aware predictor ─> constraint step ─> reconciler ─> display
 FAST LOOP · operator side · Core · every 11.1 ms frame
```

Exists today: the delayed pose downlink (`Pipeline/RobotStateFrame.cs`, 57 bytes), the
reconcilers, and the Quest display. Everything else in the diagram is to be built.

The slow loop owns only what changes more slowly than its output ages. A surface or a payload is
still where it was two seconds ago. The arm's motion depends on what the operator does next, so it
stays in the fast loop. The world model reports the environment, and the fast loop computes the
correction.

## What we have

- A deterministic, headless Core in C# (`core/Teleop.Core/`), built by both .NET and Unity, with
  gates for replay determinism (`verify`) and architectural invariants (`audit`).
- Operator-side predictors in `Prediction/`: `none` (passthrough), `const-vel`, `double-exp`.
- Five reconcilers in `Reconciliation/`, scored on correction magnitude, rate, jerk and time to
  convergence (`docs/metrics.md` §5).
- Playout buffering in `Buffering/`, and a composable impairment pipeline in
  `Transport/Impairments/`: fixed delay, uniform jitter, Gilbert-Elliott burst loss, reordering,
  recorded delay traces (ADR 0013).
- A real JetRover arm (four joints plus gripper) driven through `core/Teleop.RobotHost/`, with
  cross-machine clock sync (ADR 0008), and a Quest operator in Unity.
- A 210-source literature review in `docs/literature/`, an H200 for training and an RTX 4090 for
  inference.

## What is missing

- Environment sensing of any kind. The robot sends a 57-byte pose and nothing else.
- The camera stream, its timestamps on the shared clock, and camera-to-base calibration.
- A predictor that uses the commands already sent. `IPredictor<Pose>` observes only stamped robot
  poses.
- An `IInferenceBackend` implementation (the contract exists, with no implementations and no
  consumers), a plant that can make contact (`Plant/RigidBodyPlant.cs` is kinematic by design), and
  metrics windowed around contact and grasp events.
- Physical motion-to-photon calibration (Gate 4 in `docs/setup.md`, ADR 0003). The comparisons
  proposed here are relative and do not depend on it. Absolute latency claims would.

## Design choices and why

| Choice | Reason |
|---|---|
| The world model outputs environment state, never corrections. | A correction depends on the operator's next move, which a seconds-old model has not seen. Environment state survives the delay. |
| The fast loop stays analytic, inside Core. | The frame budget is 11.1 ms. Cosmos 3 Edge forward dynamics takes 3.91 s on an H100 (NVIDIA model card, 30 steps, undistilled). Core keeps its determinism and allocation rules. |
| The model runs at the robot site, next to the camera. | Video never crosses the delayed link. Only the small message does, through the same impairment emulation as the pose stream. |
| Model outputs are recorded as observations. | Replay stays bit-identical, and replaying a session needs no GPU. |
| A command-aware predictor comes first. | The operator side knows every command in flight. Without this baseline, some of the world model's measured gain would really be command information. |
| An oracle comes before Cosmos. | Ground-truth environment state, delivered at a swept update rate and age, bounds the benefit and sets the rate and age a real model must reach, before any training is paid for. |
| Constraints are confidence-gated, and false ones are counted. | A missed surface falls back to today's behaviour. A phantom surface stops the displayed arm while the real one keeps moving, which is worse. |
| Display only, at first. | Using constraints to limit commands belongs to the Autonomy axis, which needs closed-loop evaluation (`Autonomy/CLAUDE.md`) and is safety-relevant on hardware. |

## Plan

1. **ADR** for the environment-state message, its Core contract, where the model runs, recording,
   and the new metrics. This is a new question under root `CLAUDE.md`'s "Where new code goes", so
   the ADR comes before any code.
2. **Command-aware predictor**, benchmarked against `const-vel` and `double-exp`.
3. **Camera, timestamps and calibration.** Record sessions with a soft obstacle and with grasps.
4. **Oracle sweep** over update rate and age. **Go/no-go:** stop here if a perfect oracle at an
   achievable rate and age gives no benefit.
5. **Head-to-head in the slow slot:** oracle ceiling, classical perception (depth, plane fitting,
   payload identification from joint data), and Cosmos 3 Edge fine-tuned on the H200 and served on
   the 4090.

The new metrics (error windowed around contact and grasp events, false-constraint rate) get their
definitions in `docs/metrics.md` in the same PR that first emits them.

## Risks

- The classical baseline may match Cosmos on surfaces and payloads. The learned model would then
  have to earn its place on pushed objects, deformables and occlusion.
- Calibration error lands directly in constraint positions, and the tolerances here are
  single-digit millimetres.
- Each world-model update moves the displayed arm. That is a model jump, and the reconcilers have
  never been tested against one.
- Contact data means driving the JetRover into things. Use a soft obstacle, a `just` recipe, and a
  person watching (root `CLAUDE.md`, "Testing the real robot").

## Prior work it builds on

Each source below has a full entry, including the conditions it was measured under, in the
`docs/literature/` file named after it.

- **Mitra & Niemeyer, "Model-mediated Telemanipulation", IJRR 2008**
  <https://api.crossref.org/works/10.1177/0278364907084590>. The remote side abstracts sensing into
  a simple environment model that the operator side renders without lag. Shown on 1 DoF at a 4 s
  round trip. (`predictive-display.md` §6.4)
- **Park, "Improving Teleoperation with Models and Tasks", Stanford dissertation, 2009**
  <https://stacks.stanford.edu/file/druid:ph465qm8083/dissertation-augmented.pdf>. MMT depends on
  the environment model changing slowly, and each update is where delay re-enters.
  (`predictive-display.md` §6.4)
- **Willaert, Van Brussel & Niemeyer, "Stability of Model-Mediated Teleoperation", EuroHaptics
  2012** <https://lirias.kuleuven.be/retrieve/1ad073f5-f797-4e9a-bb01-529b8d0126d3>. Model jumps,
  introduced gradually (500 ms in, 300 ms out) behind a 4 mm deadband. 1 DoF, constant 150 ms.
  (`predictive-display.md` §6.4)
- **Bejczy, Kim & Venema, "The phantom robot", ICRA 1990**
  <https://ntrs.nasa.gov/citations/19910050529>. A command-driven arm model overlaid on the camera
  view. Its reported benefit is in free motion. (`predictive-display.md` §6.2)
- **Black et al., "Visual-Haptic Model Mediated Teleoperation for Remote Ultrasound", 2025**
  <https://arxiv.org/abs/2502.07922>. Adding a visual model to MMT compensated round trips up to
  1000 ms for 15 operators (abstract only). (`predictive-display.md` §6.4)
- **Genie Envisioner / GE-Act, 2025** <https://arxiv.org/abs/2508.05635> and **DSWAM, 2026**
  <https://arxiv.org/abs/2607.04927>. Dual-rate systems: a 5 Hz world model driving 30 Hz actions,
  and a slow planner over a fast executor. (`world-models.md`, family 4)
- **AHEAD, 2026** <https://arxiv.org/abs/2606.02486>. A small latent world model with an
  uncertainty-gated horizon compensates a robot's own inference latency. (`world-models.md`,
  family 4)
- **Khalil & Kwon, 2026** <https://arxiv.org/abs/2605.09670>. No off-the-shelf video model meets
  real-time predictive display: 1.3-3.9 s per frame on an RTX 6000 Ada. (`world-models.md`,
  family 6)
- **MiraBench, 2026** <https://arxiv.org/abs/2605.29360>. Optimism bias is widespread across 12
  world-model configurations. (`world-models.md`, surveys and benchmarks)
- **NVIDIA, Cosmos3-Edge model card** <https://huggingface.co/nvidia/Cosmos3-Edge> (accessed
  2026-09-29). Forward dynamics takes 3.91 s on an H100 SXM 80GB at 480p, with 30 denoising steps,
  batch 1, undistilled. Not yet in `docs/literature/`. The card was read through a summarising
  fetch, so confirm the figure against it before reusing it. The Cosmos 3 technical report
  (<https://research.nvidia.com/labs/cosmos-lab/cosmos3/technical-report.pdf>) was not read.

## Where it would extend prior work

- In the 210 sources reviewed, MMT's environment models are analytic, and no source uses a learned
  model of the scene for operator-facing delay compensation. World-model papers that mention
  teleoperation almost always mean collecting training data.
- MMT has been shown on 1 DoF, with haptics, at constant delay. This would be visual, in VR, on a
  multi-joint arm, under jitter, burst loss and reordering.
- The phantom robot helps in free motion. Contact is the case a command-driven model cannot
  foresee, and it is the case targeted here.
- MMT assumes the environment changes slowly, but no source reports how the benefit decays with
  update rate and age. The oracle sweep measures that curve.
- Willaert et al. state that longer lag enlarges model jumps without measuring it. This platform
  reports jump size and jerk as distributions.
- AHEAD-style compensation serves the robot's own compute, with no person in the loop. Here the
  delay is the network and the viewer is a person.
- A false-constraint rate would put a number on optimism bias as a display failure.

---

Figures from prior work were measured on other systems. They motivate the experiments proposed
here and are not evidence about this platform (`docs/literature/CLAUDE.md`).
