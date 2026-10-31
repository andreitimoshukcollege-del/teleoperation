# 31. Isaac Sim: the FR3 Duo with physics and objects, behind RobotHost

## Status

Proposed (2026-10-31). The user asked to connect Isaac Sim, which is installed on the lab's RTX 5090
machine. For the first version they chose **physics, with objects shown in VR**, over arm physics
alone or a camera feed.

**This is the first part of the system that models the environment.** Root `CLAUDE.md`'s Direction
paragraph says that is an architecture change, and that the ADR comes first. No code is written
against this until it is accepted.

**It builds on ADR 0030** (proposed, PR #95): the arm-server wire protocol, and RobotHost beside the
process that owns the arm. If ADR 0030 changes in review, this one follows it.

## Context

**What exists is kinematic.**
- **The simulated host** (ADR 0027) is Core's trajectory shaper plus a second-order lag. It has no
  mass, no contact and no objects.
- **Core's `Plant/`** is kinematic by deliberate choice.
- **What the system knows of the world:** only the robot's own body. The headset draws the arms and a
  lab that nothing outside it knows about.

**What Isaac Sim offers.** It is NVIDIA's robot simulator on Omniverse and PhysX. It provides:
- articulated robots imported from a URDF, with joint drives;
- contact, friction and grasping;
- rigid objects;
- rendered cameras.

It runs headless, scripted from Python (`SimulationApp`).

**The 5090 machine, as inspected on 2026-10-31** (read-only, over the lab VPN):
- **Hardware and OS:** Ubuntu 24.04.5, 32 cores, 62 GB of RAM, an RTX 5090 on driver 580, 484 GB of
  free disk.
- **Already installed:** the .NET 8 SDK and ROS 2 Jazzy.
- **Shared.** Two other accounts use it, and the GPU is shared with training (`just gpu-train`
  refuses when more than 2 GB of the GPU is in use).
- **The existing Isaac Sim install is not ours to use.** It is not in our account or in any shared
  directory. The other home directories are private, and only another user is in the `docker` group,
  so it is probably theirs.
- **A pip install fits.** The system Python is 3.12, with `venv`, and that is what Isaac Sim 6.x's
  pip wheels are built for. NVIDIA's index offers 6.0.0.1 to 6.2.0.0; 6.2.0.0 is the newest.

**What latency research needs from a world.** The object poses the operator sees must cross the same
network as the robot's state:
- through the same impairments, clock sync and playout;
- and later, through prediction.

A world drawn from a side channel, or simulated on the headset, would leave the network out of
exactly the part of the task that is new. This is also the gap the user named in a lab mate's Isaac
setup: it "completely removes the lag/network disturbance aspect".

## Decision

1. **Isaac Sim is a third serial-chain backend, on the 5090 machine.**
   - **The layout.** RobotHost runs there too, on its .NET 8, beside an *Isaac bridge*, over a Unix
     datagram socket. This is ADR 0030 decision 4's layout, with `--backend arm-server` pointed at the
     bridge.
   - **The headset** reaches that RobotHost as it reaches the simulated host: the joint port,
     `ClockSync`, and discovery (ADR 0022). Nothing changes on the headset for the arms.
   - **The simulated host** stays on the PC (`just fr3-sim-host`) for quick work without the GPU.

2. **The Isaac bridge** (`robot/isaac/`) is a standalone Isaac Sim Python application, headless by
   default.
   - **Its environment.** It has its own Python environment on the 5090 with a pinned Isaac Sim
     version. It reads the repo's contract files (the robot profile, the scene file, the goldens) and
     imports no other repo code: not the trainer's, not Core's.
   - **The arms** are imported from the URDF the profile came from (`franka_description`'s `fr3_duo`).
     At load the bridge checks the import against `core/RobotProfiles/fr3-duo.json`: joint names,
     limits, and the tool poses of a few seeded joint sets. It refuses to start on a mismatch.
   - **The shaper.** A Python copy of `JointTrajectoryShaper` and its finger gate is pinned to Core's
     golden file, as ADR 0030's C++ copy is (decision 3 there). It is stepped once per physics step.
   - **The tracking.** The shaper's output becomes PhysX joint position-drive targets. The drives'
     stiffness and damping play the part of Franka's internal controller. The gains are parameters,
     recorded with each session, and are fitted to the real robot once it exists (ADR 0030 decision
     11).
   - **Physics at 1 kHz, paced to the wall clock.** If the simulation falls behind, the bridge counts
     the lag and reports it. It never silently runs slow, because a slow world corrupts every latency
     figure.
   - **The hands.** Each Franka Hand's fingers are prismatic joints. The finger gate moves them, under
     a force limit, so a grasp holds by friction (*verify*: the limit against the real hand's
     continuous force).
   - **States.** Each arm's state goes to RobotHost every 4 ms, in ADR 0030's message.

3. **The world: one scene file, shared by Isaac and the headset** (`core/Scenes/*.json`, read the way
   the robot profiles are).
   - **What it holds:**
     - the table, aligned with the lab bench the FR3 stands on;
     - each object's id, name, shape (box, cylinder or sphere; meshes later), size, mass, friction,
       colour and starting pose;
     - all poses in the robot's root frame (ROS convention: right-handed, Z-up, X-forward).
   - **Who reads it:**
     - **Isaac** builds the scene's physics from it.
     - **The headset** builds matching visuals from it. Unity simulates nothing.
     - **The FR3 scene check** confirms the Unity bench matches the file's table.
   - **Repeatable trials.** A seed jitters the starting poses inside ranges the file states.

4. **The world state travels like the robot state.** This is the new path:
   - **Bridge to RobotHost.** The bridge sends object poses to RobotHost as a *world* message: an
     amendment to ADR 0030's protocol, made before either side is built.
   - **RobotHost to the headset.** RobotHost forwards them on the joint port as a new frame kind: the
     sensed time, then each object's id, position and orientation.
     - **Size and rate:** up to 8 objects per datagram under the joint port's 256-byte budget (ADR
       0026 decision 6), with more objects spanning datagrams; sent at 50 Hz.
     - **Same route as the joint states:** the same transport and the same headset-side disturbance,
       `ClockSync` and playout.
   - **On the headset.** A *world twin* poses each object at its reported pose.
     - **Version 1** shows the newest received pose, after the joint link's playout.
     - **Predicting world poses** is a new research question (a `Contracts/` axis). It gets its own
       ADR, which can use the data this one records.
   - **Code layout.** The Core types and the codec live where the joint codecs do
     (`Teleop.RobotArm/Wire/`). Bridge code (`unity/Teleop.Bridge/`) receives and converts, and the
     handedness conversion stays in `CoordConversion.cs` only.

5. **Recording.** Isaac sessions are session format v2 with `"source": "isaac"`:
   - **`session.json`** also names the scene file (its sha256), the seed, the Isaac version and the
     drive gains;
   - **`robothost.jlog`** gains world records: the object poses at their sensed times, and contacts
     between a hand and an object. They are defined in `trainer/contract/SESSION_FORMAT.md` by the
     change that builds them.

   These are the system's first recordings with a ground-truth world: the data the Direction
   paragraph's world-model prediction will need. **An Isaac session is never the robot's,** and is
   reported apart, as `sim-host` sessions are.

6. **What counts as evidence.**
   - **Goldens.** PhysX on a GPU is not bit-reproducible, so no golden test or Core test depends on
     Isaac. The shaper's Python copy is golden-pinned; the physics is not.
   - **Headless.** Isaac runs headless on the 5090. A recorded Isaac session replays through
     `Teleop.Eval` like any other, world records included. That meets root `CLAUDE.md`'s rule that an
     algorithm must be evaluable without a headset.

7. **Recipes.** Everything goes through `just`, from the PC over SSH, as `gpu-train` does:
   - **`isaac-setup`**, run once: installs the pinned Isaac Sim (`isaac_version`, 6.2.0.0) into our
     own account, in user space with no `sudo`. It makes a venv of the machine's Python 3.12 at
     `~/isaacsim/venv` and installs NVIDIA's wheels into it, detached, with a log. NVIDIA's licence is
     accepted at Isaac's first launch. The user accepts it; no recipe accepts it on their behalf.
     `isaac-status` shows how the install is going.
   - **`isaac-host`**: starts the bridge and RobotHost on the 5090, streams their logs, and stops both
     on Enter.
     - **GPU sharing:** it refuses when another account's processes hold more than a threshold of
       the GPU, and says whose they are.
     - **The headset** finds RobotHost through discovery, or through `just push-galaxy-fr3-config
       host=<5090>`.
   - **`isaac-status`**: the bridge's real-time factor, physics lag and GPU use.

8. **Phases.** Each one is usable on its own:
   1. **The arms in Isaac** behind RobotHost: the protocol, the bridge, the shaper copy and the
      drives. The current headset app drives them unchanged.
   2. **The scene and the world twin:** the scene file, world frames, the world twin and the scene
      check.
   3. **Recording and replay** of world records.

   Later, each with its own ADR:
   - predicting world poses on the headset;
   - an Isaac camera to the headset's display (ADR 0014's path);
   - Isaac as a source of training data.

## Consequences

- **The environment enters the system.** Root `CLAUDE.md`'s "nothing in this system models the
  environment" stops being true when phase 2 lands, and that paragraph is updated in the same change.
  `Plant/` stays kinematic: the physics is Isaac's, behind RobotHost.
- **A third copy of the shaper** (Python, in the bridge), beside the trainer's and ADR 0030's C++
  copy. Each is pinned by the same golden file. The trainer's isolation rule forbids sharing its
  copy.
- **A second Python environment on the 5090** (Isaac's), apart from the trainer's.
- **The 5090 becomes a host as well as a trainer.** An Isaac session and a training run compete for
  one GPU, so `isaac-host` and `gpu-train` each refuse while the other runs.
- **No change to the JetRover,** or to the simulated host on the PC.

## Alternatives considered

- **Isaac's ROS 2 bridge, through a ROS node, to RobotHost.** ROS 2 Jazzy is installed, but this
  puts ROS between two processes this repo owns. ADR 0030's protocol already carries exactly these
  messages, and ROS stays possible later.
- **Isaac renders the operator's view and streams video** (the lab mate's way). Not the primary
  path. The user chose 3D objects in VR, and this platform's prediction and reconciliation work on
  poses rendered on the headset, not on frames rendered remotely. A camera feed can come later
  (decision 8).
- **Physics for the objects in Unity, on the headset.** Rejected. Contact has to be computed where
  the robot's true state is, on the far side of the network, or the network drops out of the
  manipulation task.
- **MuJoCo.** Lighter, and deterministic on a CPU. Not chosen, because the user asked for Isaac and
  has it installed. The trainer's simulator remains the analytic model the user chose for training
  (ADR 0028).
- **RobotHost on the PC, with the bridge over UDP.** Rejected for ADR 0030's reasons: two clocks to
  sync, and a network hop between targets and the shaper.
- **Using the existing Isaac install in another account.** It would need that user's Docker group
  or their directories, both outside our account. A copy in our own account keeps the two setups
  from interfering.

## Open questions

- **Isaac Sim 6.2.0.0 on this machine:** whether it runs on the RTX 5090 with driver 580. The first
  headless launch will show it.
- **The FR3 Duo's URDF in Isaac:** whether `fr3_duo` imports cleanly, with the hands' meshes and
  their mimic joints.
- **Starting values:** drive gains (start from Isaac's own Franka example's), the hand's force limit,
  and object friction.
- **The real-time factor** at 1 kHz physics with a few objects, on a shared GPU.
- **World playout:** whether world frames share the joint link's playout or get their own. Version 1
  shares it.
