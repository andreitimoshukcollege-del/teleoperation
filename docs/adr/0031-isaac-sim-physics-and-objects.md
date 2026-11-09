# 31. Isaac Sim: the FR3 Duo with physics and objects, behind RobotHost

## Status

Accepted (2026-10-31). The user asked to connect Isaac Sim, which is installed on the lab's RTX 5090
machine. For the first version they chose **physics, with objects shown in VR**, over arm physics
alone or a camera feed.

**This is the first part of the system that models the environment.** Root `CLAUDE.md`'s Direction
paragraph says that is an architecture change, and that the ADR comes first. It was accepted before
any bridge code was written.

**It builds on ADR 0030**, accepted the same day: the arm-server wire protocol, and RobotHost beside
the process that owns the arm.

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
     on Enter. `isaac-stop` does the same from any terminal (2026-11-02): SIGTERM to RobotHost, which
     takes it as Enter.
     - **GPU sharing:** it says how much of the GPU other accounts' processes hold. It refused above
       2 GB until 2026-11-02, when the user removed that limit: the 5090 has 32 GB and Isaac needs
       about 4 GB, so the two share its compute. Late steps in the bridge's log can come from that.
     - **The headset** finds RobotHost through discovery, or through `just push-galaxy-fr3-config
       <the 5090's address>`.
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

## Amendment (2026-10-31): phase 1 built and run

The arms run in Isaac behind RobotHost. Two things on top of the decisions above came out of building
it.

**Built:**
- **The wire.** `ArmServerWire` (RobotHost) and `robot/isaac/teleop_isaac/wire.py` are pinned to
  each other byte for byte by `robot/isaac/golden/arm-server-wire-v1.json`. The C# tests write that
  file (`just gen-arm-server-golden`).
- **The backend.** `ArmServerClient` is ADR 0030's `FciArmServerClient`, renamed because it serves
  Isaac as well.
  - It refuses a server whose motors are not the profile's, in the profile's order, or whose clock
    is more than 50 ms from RobotHost's.
  - It starts from the pose the server reports.
  - Sessions take the server's kind as their `source`.
- **The bridge.** `robot/isaac/teleop_isaac/`. Its shaper is the trainer's copy, taken as text and
  pinned to the same golden: an Eval test holds the two golden files equal.
- **The recipes:** `just isaac-host`, `just isaac-bridge-test` and `just gen-arm-server-golden`.

**What Isaac 6.2 needed, found on the 5090:**
- **The PhysX variant.** The URDF importer writes the physics as a `Physics` variant set (`mujoco`,
  `physics`, `physx`) and selects none. The bridge selects `physx`.
- **Drive gains.** The importer leaves the joint drives at zero gain. The bridge sets 16000 Nm/rad
  and 250 Nm·s/rad on the arm joints (about 20 Hz tracking), and 2000 N/m and 100 N·s/m on the
  fingers.
- **No results back to USD.** Writing physics results back to USD every step took 1.5 ms of each
  1 ms step. With it off, a step that sets new targets costs 0.38 ms, 2.6 times real time.
- **Robot gravity off,** as decided. With gravity on and no gains, the arms fell.

**Measured end to end** on 2026-10-31. `just fr3-teleop-probe` ran from the PC against `just isaac-host`
on the 5090, over the lab VPN (RTT 12.5 ms):
- **The left tool** ended 0.07 mm and 0.01° from its target, with the gripper closed.
- **The right tool** ended 0.07 mm from its target, with the gripper open.
- **The twin (`franka-replay`)** was within 0.35° of each reading, at p95 over 2632 readings. That is
  against 0.04° on the simulated host: `franka-replay` models the shaper exactly but not the PhysX
  drives' lag.
- **The bridge** kept real time (1.0×) for 56028 steps, with 13 late.

**Not yet:** the forward-kinematics check against the profile, which decision 2 asks for. Only joint
names and limits are checked so far. Phases 2 and 3 have not started.

## Amendment 2 (2026-11-02): phase 2, the work table, fixed obstacles, layouts and moves

The user wants to collect training data in the lab with the simulated FR3 Duo, and chose, for the objects:
- **fixed obstacles first**, movable objects later;
- **layouts from the scene file, and moves from the headset**;
- **layouts that change only when asked**: at host start, with NEW LAYOUT, or by moving an object;
- objects shown **in 3D in VR** (decision 4's world twin) **and on two camera feeds** (ADR 0032).

This amendment fixes what decisions 3–5 left open, before either side is built.

1. **A work table in front of the robot.** The VR lab's bench ends 0.2 m in front of the FR3, short of
   its workspace.
   - **The scene file** holds the bench and a work table that abuts the bench's far edge, flush with
     its top, spanning x = 0.20–0.90 m in the root frame.
   - **The FR3 scene** builds the table from the file, and the scene check compares the two within
     1 mm.
   - **The file:** `core/Scenes/fr3-duo-bench.json`.

2. **Fixed obstacles first.**
   - **`kind: fixed`** is a kinematic rigid body in Isaac: the arms collide with it, and only a
     placement moves it.
   - **`movable`** is reserved, and version 1 of the file refuses it.

3. **The headset's IK and RobotHost's refusal check stay unaware of objects.**
   - **Why:** the user wants the arms to collide in Isaac, so the contact shows as command against
     measured. That divergence is the signal the Direction paragraph's world model is meant to learn.
   - **What follows:** a command into an obstacle is accepted, and the arm stops against it. At
     contact, the commanded ghost and the predicted arm pass through the obstacle while the measured
     arm stops.
   - **The drives:** each arm joint's force is limited to the profile's effort, so an arm stalls
     against an obstacle instead of overpowering the solver.
   - **A test pins it:** `WithAScene_ACommandIntoAnObstacleIsAccepted`.

4. **Layouts are drawn by RobotHost** (Core's `SceneLayout.Draw(scene, seed)`, on `SeededRng`,
   allocation-free).
   - **When:** at host start (`--scene-seed n|auto`, recorded), and on NEW LAYOUT from the headset (a
     bench button, only while motion is off). They never change on their own.
   - **Isaac checks each placement:** it refuses a pose that overlaps an arm or another object (a
     PhysX overlap query). A refused object is redrawn up to 8 times, and otherwise stays where it was.
   - **The same code serves the simulated host:** `fr3-sim-host` runs it against a static world, so
     the path is tested end to end without the GPU machine.

5. **Moves from the headset come last** (phase P5).
   - **EDIT OBJECTS** is a bench button, active only while motion is off.
   - **The gesture:** a held curl grabs the nearest movable object's twin. Dragging moves it in x and
     y, and palm yaw turns it, clamped to the table. Opening the hand sends one request.
   - **The solid twin moves only when world frames show the new pose,** so a move visibly crosses the
     network.

6. **The wire.** Quaternions are w, x, y, z throughout, as `RigidTransform` and Isaac order them.

   **Arm-server wire, version 1, new kinds 7–11**, in `ArmServerWire.cs`, `wire.py` and the golden
   file. An older RobotHost counts them as rejected, and an older bridge ignores them.

   | Kind | Direction | Body |
   |---|---|---|
   | 7 `WORLD_INFO` | bridge → host | `32s sceneSha256, u8 n, n × u8 id`. Sent after every WELCOME and on every scene change |
   | 8 `WORLD` | bridge → host | `i64 sensedNs, u32 snapshot, u32 revision, u8 part, u8 parts, u8 n ≤ 16, n × (u8 id, u8 flags, 3 × f32 position, 4 × f32 quaternion)`. 50 Hz, poses from PhysX, not USD |
   | 9 `PLACE` | host → bridge | `i64 hostTicks, u32 request, u8 cause (1 layout, 2 headset, 3 reset), u8 id, 3 × f32, 4 × f32` |
   | 10 `PLACED` | bridge → host | `u32 request, u8 id, u8 outcome, u32 revision, 3 × f32, 4 × f32`: the pose the object has now |
   | 11 `CONTACTS` | bridge → host | `i64 sensedNs, u8 n, n × (u8 arm, u8 link, u8 object (0 = table), f32 normal force N)`. Optional: sent if PhysX's contact reports work |

   **Outcomes:**

   | Code | Meaning |
   |---|---|
   | 0 | applied |
   | 1 | overlaps an arm |
   | 2 | overlaps an object |
   | 3 | outside the placement region |
   | 4 | unknown object |
   | 5 | not movable |
   | 6 | an arm touches it now |
   | 7 | busy |
   | 8 | invalid pose |
   | 9 | timed out |

   **Joint port, three new tags.** They share the joint states' transport, disturbance, `ClockSync`
   and playout. That answers this ADR's open question: world frames share the joint link's playout.

   | Tag | Direction | Layout |
   |---|---|---|
   | `0x85` edit request | headset → host | 44 B: `u8 tag, u8 version, u8 op (1 move, 2 new layout, 3 reset object, 4 reset layout), u8 id, u32 sequence, i64 operatorSendTicks, 3 × f32, 4 × f32`. Re-sent every 0.25 s until echoed, abandoned after 2 s |
   | `0x86` world frame | host → headset | ≤ 249 B: `u8 tag, u8 version, u32 snapshot, i64 sensedTicks, u8 part, u8 parts, u8 n ≤ 8, n × (u8 id, 3 × f32, 4 × f32)`. Each part is applied as it arrives |
   | `0x87` world status | host → headset | 64 B: `u8 tag, u8 version, u8 flags (world running, physics rather than static, edits allowed), u8 object count, 32 B scene sha256, u32 layout number, u64 layout seed, u32 echoed edit sequence, u8 outcome, u8 object id, u16 reserved, i64 robotSendTicks`. Once a second and on change |

   **The headset checks the scene:** it compares the status's scene sha256 with its own copy of the
   file, and shows SCENE MISMATCH instead of a wrong world.

7. **Recording.**
   - **`robothost.jlog`** gains these records, defined in `trainer/contract/SESSION_FORMAT.md` by the
     change that builds them:

     | Tag | Records |
     |---|---|
     | `WL` | a layout |
     | `WP` | a placement and its outcome |
     | `WO` | an object-pose snapshot, on change |
     | `WF` | a world frame sent to the headset |
     | `WC` | contacts |
     | `WE` | an edit from the headset |

   - **`session.json`** gains a `scene` block (name, sha256, seed) and the arm server's description
     (the Isaac version and drive gains, which decision 5 asks for).
   - **The scene file** is copied into the session as `scene.json` and hashed.

8. **The scene file, version 1.**
   - **Readers:**
     - RobotHost reads it with System.Text.Json and the headset with `JsonUtility`. Both read the same
       `[Serializable]` document in `Teleop.RobotArm`, which validates it and does no I/O.
     - Isaac reads it with Python's `json`.
   - **Shared test vectors** in `core/Scenes/testdata/` hold the readers to one meaning.
   - **What it holds:**
     - surfaces (bench, table, floor), each marked whether the headset shows it and whether it
       collides;
     - objects: id 1–254, name, shape, size, kind, whether the operator may move it, friction, colour,
       a pose that rests on the table, and jitter in x, y and yaw;
     - the placement region and clearance;
     - Isaac-only `backdrops`, `cameras` and `lights` (ADR 0032).
   - **Format constraints:** no nulls and no nested arrays, so `JsonUtility` can read it.

9. **Phases.** Decision 8's phase 2 becomes two:
   - **P1:** the scene file, its readers in Core and Isaac, and the work table in the VR lab.
   - **P2:** objects in Isaac, world frames, the headset's world twin, NEW LAYOUT and world records.
     The world twin moved here from P1 because nothing can pose it before world frames exist. The root `CLAUDE.md` Direction
   paragraph is updated in P2. Moves from the headset are P5, after ADR 0032's feeds (P3) and recording
   (P4), the order the user chose.

## Amendment 3 (2026-11-03): moving objects by hand from the headset (phase P5)

Amendment 2's moves are built on the headset.
- **EDIT OBJECTS** is a bench button right of RECENTRE. It works only while the robot is off, with absolute hands and a
  running world whose scene file matches. Switching motion on turns it off.
- **While it is on**, the arms let go and don't follow the hands.
- **The gesture** is Core's `ObjectEditGesture` (`Teleop.RobotArm/World`).
  - **The cursor** is each hand's mapped tool pose, where the gripper would go.
  - **A held curl** (the grab's curl, not the double-curl lock) grabs the nearest operator-movable object within 6 cm of its
    footprint and at most 10 cm above its top.
  - **Dragging** moves it in x and y and turns it with the hand's heading, kept on the table and in the placement region.
  - **Opening the hand** asks once for that pose. A drag under 5 mm and 2° asks nothing, and a lost hand cancels.
  - **One object at a time,** by one hand.
- **The request** is a `0x85` move. The joint link re-sends it until RobotHost echoes it. RobotHost places it with
  `PlaceCause.Headset`, and Isaac refuses an overlap with an arm or another object, or an object an arm is touching.
- **The twin** shows a see-through ghost where the hand puts the object. The solid object moves only when world frames show
  the move, so the operator sees it cross the network, or be refused. The button's label says RobotHost's answer.

**Verified:**
- **Tests:**
  - `ObjectEditGestureTests`: grab, drag, turn, clamp, a single request, tiny drags, lost tracking, one hand at a time,
    unposed and fixed objects, and no allocation.
  - `WorldHostTests`: a re-sent move is placed once, refused while motion is on, and times out when the backend never
    answers.
- **`just isaac-edit-check` on the 5090,** 2026-11-03, `isaac-host fr3-duo-bench 7`:
  - the can moved to a free spot: applied, and the frames show it there to 0.000 mm and 0.000°;
  - a spot off the table: `outside-region`;
  - the left post moved aside: applied;
  - with the left arm held forward and down, the post back under that hand: `overlaps-arm`.
- **The same moves beside the simulated arm** (`fr3-sim-host`): applied, `outside-region`, `overlaps-object`.

**How the arm was lowered:** `chain-drive --hold 1:0.6,2:0.6`. Folding the elbow (`--hold 4:-0.6`) is refused by
RobotHost's collision model, with link2 against link5.

**Not verified:** whether the gesture feels right on the headset: the grab reach, the clamp, and the ghost's look.

## Amendment 4 (2026-11-03): movable objects, and the robot just ahead of the operator

The user, after trying amendment 3 on the headset: "we want the objects to not be static, so the arm should be able to knock
it down, and the arm feels far away, we want the arm and the objects setup to be close to the user". They chose the layout
"just ahead, over its shoulder".

**Movable objects (scene file version 2).**
- **The format.** Version 2 adds `kind: "movable"` with a `mass` in kg, more than 0 and at most 20. A fixed object has no
  mass. Version 1 still reads, and still refuses `movable`. Readers take versions 1 to 2.
  - Core's `WorldScene` and Python's `scene.py` apply the same rules with the same keywords (`kind`, `mass`, `version`).
  - The shared vectors gained seven cases.
  - The canonical-parse golden now carries `movable` and `mass`.
- **The shipped scene.** All four objects are movable:
  - the posts, 0.4 kg each;
  - the wall, 0.6 kg;
  - the can, 0.35 kg.

  The floor now collides, so whatever falls off the table lands on it.
- **Isaac.**
  - A movable object is a dynamic rigid body: `kinematicEnabled` is false, and `MassAPI` carries its mass. It feels gravity;
    only the robot's links have gravity off.
  - It is reported with WORLD flag 0. A fixed object keeps `FLAG_KINEMATIC`.
  - Each WORLD read takes PhysX's pose as where the object is now (`WorldModel.sense`). The placement rules use that pose:
    overlap with another object, and arm touching. A pushed object is not a new revision; revisions count placements.
  - A placement stands a movable object upright where it is put, and zeroes its velocities, wherever it had fallen. So
    NEW LAYOUT, or a move by hand, is how a knocked-over object comes back.
- **RobotHost's `StaticWorld`** (no physics, `fr3-sim-host`) is unchanged: there, every object stays where it was placed.
- **Unchanged:**
  - The headset's IK and RobotHost's refusal check still know nothing of objects (amendment 2).
  - The twin already poses each object with its full orientation from world frames, so a fallen object is drawn lying
    down.
- **`just isaac-scene-check`** runs the bridge on a copy of the scene with post-left made fixed, because checks 4 and 5 need
  an object that cannot move. It gains a sixth check: post-right under the left hand, then the same elbow motion.
  - It passes when the post moves more than 2 cm or tips more than 15°, and the arm falls short of its command by less
    than 0.05 rad.
  - Placed again, the post stands within 3 mm of where it was put.
  - A movable object at rest must stay within 3 mm of its nominal pose.

**The robot just ahead of the operator.**
- **The scene file's bench** is 0.6 m deep: x from −0.4 to 0.2. The robot's base is now 0.4 m in from the bench's near
  edge, about 0.5 m ahead of the operator's eye; it was 1.1 m in. The work table still starts at the bench's far edge, so
  the objects are about 0.9–1.1 m away.
- **On the headset,** `XrFr3SceneBuilder.BenchDepth`/`RobotDepth` are 0.6/0.4, held to the file within 1 mm by
  `XrFr3SceneCheck`.
- **The controls** are built by the JetRover's code as before, then moved out of the arms' way (`ArrangeControls`):
  - Power, pause, the twin's switches and the added-delay slider stay on the front-left.
  - The predictor row runs along the front, right of the switches.
  - The three network sliders are on the front-right.
  - RECENTRE, NEW LAYOUT and EDIT OBJECTS stand one behind another on the right wing.
  - The console's panel stands off the bench's right side, turned to the operator, so it never hides an arm.
- **Why there:** by Core's FK at the start pose, the elbows are 0.5–0.6 m either side of the base, at its depth, 0.35–0.5 m
  up. The bench's front 0.25 m is clear of them.
- **Not changed:** the hands still map to the tools about the neutral pose (`BodyAnchoredMapping`). Only where the operator
  sees the robot moved, not how it is driven.

**Verified:**
- **Tests:** `dotnet test` with the version-2 vectors, the golden and `AVersion1File_StillReads_AndItsObjectsAreFixed`;
  `just isaac-bridge-test` (42 tests, including a sensed movable pose).
- **`just isaac-scene-check` on the 5090,** 2026-11-03 (a8fab7a): all 20 checks pass.

  | Check | Measured |
  |---|---|
  | The fixed post stalls the hand | 0.256 rad short; the post moved 0.00 mm |
  | The movable post under the same motion | moved 86 mm, tipped 31.8°; the arm 0.0004 rad short |
  | Placed again | applied, and at rest 0.0 mm from where it was put |
  | Movable objects at rest at their nominal poses | within 0.0001 mm |
  | Late steps | 4 in 25.5 s (0.16/s) |
- **The headset scene:** rebuilt, `just unity-check-fr3-xr` passes, and the previews show the robot just below the
  operator's eye, the controls on the front strip.

**Not verified:** how it looks and feels in the headset: how close the robot seems, whether the arms crowd the controls,
and how a knocked object looks at 50 Hz world frames.

## Amendment 5 (2026-11-06): pointing at buttons and objects while the robot is off

The user: "add the ability to point and click on a button while the robot is off and to point and drag the isaac sim
objects around".

- **The ray.** While the robot is off, each tracked hand casts a ray from about its shoulder through its pinch point
  (`Fr3HandPointer`), as Android XR's hand rays and the JetRover's display mover do. A thumb–index pinch below 2 cm
  clicks; the pinch releases above 3.5 cm.
- **On a button.** A pinch whose ray is on a button presses it, exactly as a fingertip does. This covers every cap a
  `FingertipPress` watches: power, the console, the switches, RECENTRE, NEW LAYOUT, EDIT OBJECTS and the RECORD group.
  `FingertipPress.PointerClick` marks the cap, and its next update reports a press, under the same cooldown. Caps are
  small, so the ray may pass 2 cm beside one.
- **On an object.** A pinch whose ray is on an object grabs it. The object follows the ray's point on the level through
  its centre, kept on the table and in the placement region. Releasing asks RobotHost to place it there, through the same
  move request and refusals as EDIT OBJECTS.
  - The logic is Core's `PointerDrag` (`Teleop.RobotArm/World`), headless-tested and allocation-free.
  - An object counts as hit within its bounding sphere, so a fallen object is caught too.
  - A click on a fallen object, even one on the floor, asks to stand it upright on the table.
  - The twin's see-through ghost shows the drag. The solid object moves when the world frames show it.
- **Only while the robot is off,** and not while EDIT OBJECTS is on. So a pinch never does two things: with the robot on,
  the hands drive the arms, the rays are hidden, and fingertips still press buttons. A ray whose hit is nearer than 15 cm
  is hidden too, because that close the fingertip is the way. Pointing at power can switch the robot on; the rays then
  disappear.
- **Clamped drags stay 1 mm inside the region.** Found while testing: a drag clamped exactly to the region's edge goes
  out as a float32 pose whose rounding can land it a hair outside, and RobotHost would refuse it. EDIT OBJECTS had the
  same edge case and now keeps the same margin.
- **The JetRover scene is unchanged:** it has no `Fr3HandPointer`, so its buttons still take fingertips only.
