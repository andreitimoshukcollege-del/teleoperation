# 30. The real FR3: a real-time arm server behind RobotHost

## Status

Accepted (2026-10-31), before the hardware. This is phase P8 of the FR3 Duo plan. At the start of the
FR3 work the user chose to design the real robot's side ahead of the robot, so the day it arrives is
spent testing, not designing.

It decides what ADR 0027 decision 3 left to "its own ADR":
- libfranka or franka_ros2;
- one FCI connection per arm;
- the real-time PC's network and recipes;
- parking as stop-and-brake;
- the socket protocol between RobotHost and the real-time server.

It also turns on what ADR 0027 decision 5 left off: arm power, park and relax.

**Nothing here is verified against a robot.** Every item marked *verify* is an assumption about the
delivered hardware or about Franka's software for its system version. It must be checked, and this
ADR amended, before code depends on it.

## Context

**What the FR3's control interface (FCI) demands** (ADR 0027):
- a joint command every millisecond, from a real-time callback;
- every command within the velocity, acceleration and jerk limits, or the robot stops with a reflex
  error;
- no missed cycle, or the motion aborts.

The Franka Hand is a separate device. Its moves are blocking calls, and it cannot take a stream.

**What exists:**
- **The seam.** `IArmBackend` (`Teleop.RobotHost/Backend/`): `Send` targets, `ReadState`,
  `SetEnabled`.
- **The plant.** `SerialChainPlant`, upstream of the seam in radians, owns staleness, limit clamping
  and the reported state.
- **The shaper.** One trajectory shaper in Core (`JointTrajectoryShaper`, 1 ms steps, a
  `limitRate`-style tracker with a newest-wins finger gate).
- **The simulated backend.** `Fr3SimBackend` runs that shaper in-process.
- **The prior.** `franka-replay` is exact because the backend and the prior run the same shaper.

**What RobotHost is.** RobotHost runs on .NET, with a garbage collector, in a 5 ms loop. It can do
everything around the arm (the headset's joint port and `ClockSync`, the collision model, power,
recording, discovery), but it cannot be the 1 kHz loop.

**The JetRover precedent** (ADR 0019):
- RobotHost runs on the robot's own computer.
- A separate process owns the hardware: the ROS relay, reached over a Unix datagram socket.
- Its wire format is changed on both sides together (`RelayProtocol.cs` with `relay_protocol.py`).

**What is assumed about the hardware** (*verify*):
- **Two independent arms.** The FR3 Duo is two FR3 arms on one mount, each its own FCI robot with
  its own controller and network address. `franka_description`'s `fr3_duo` describes it that way, and
  `core/RobotProfiles/fr3-duo.json` was imported from it.
- **The hands and the head.** Each arm has a Franka Hand. The head is a fixed link.
- **The user stop.** Each arm has its own, and stopping is always in reach of whoever is at the
  robot.

## Decision

1. **libfranka directly, in a small C++ arm server; not franka_ros2.** The server needs exactly two
   things: the C++ copy of the shaper inside the real-time callback, and a socket to RobotHost.
   - **franka_ros2 would add, around the same C++ shaper:** DDS, executors, launch files, a
     `ros2_control` controller plugin to hold the shaper, and a ROS bridge to RobotHost.
   - **libfranka is Franka's primary interface,** and franka_ros2 is built on it.
   - **Why the JetRover differs.** It runs ROS only because its vendor's servo driver is a ROS node.

2. **One FCI connection per arm, each in its own real-time thread.** There are two `franka::Robot`
   instances.
   - **Each arm's loop** runs in a `SCHED_FIFO` thread pinned to an isolated core.
   - **Each hand** has its own `franka::Gripper` in an ordinary thread, because its calls block.
   - **The two loops are not synchronized to the millisecond,** and nothing needs them to be:
     - targets are per motor;
     - the collision check runs on RobotHost's targets (decision 4);
     - each state report carries its own arm's time.

3. **The arm server**, in `robot/fr3/arm_server/`, built with CMake on the real-time PC, next to
   `robot/ros/` and independent of the .NET and Unity builds:
   - **The shaper.** It is a header-only library with no libfranka dependency: a line-by-line port of
     `JointTrajectoryShaper` and its finger gate, in double precision, in the same operation order.
     - **Its tests** replay a golden file that Core writes, as the trainer's copies do, and must match
       exactly. A mismatch is a bug in whichever side moved.
     - **The golden file** is the shaper half of `gen-trainer-golden`'s output, written a second time
       into `robot/fr3/arm_server/golden/`. An Eval test holds the two copies byte-identical, so
       `robot/` reads nothing from `trainer/`.
   - **The callback.** It does no locking, no allocation and no system calls. Each millisecond:
     1. read the newest targets from a lock-free single-slot exchange that the socket thread fills;
     2. step the shaper once;
     3. return `franka::JointPositions`.
   - **libfranka's own filtering:**
     - **The low-pass filter is off** (cutoff at libfranka's maximum). The robot then gets exactly
       what the shaper computed, and `franka-replay` stays exact.
     - **The rate limiter stays on, as a safety net, and is counted.** The server also runs
       libfranka's public `limitRate` on its own output and counts every step it would have changed.
       That count must stay at zero; a nonzero count is reported as a defect, and the prior is no
       longer exact while it lasts.
   - **The hand.** The finger gate decides when a move starts: the hand is idle and the target is
     beyond the deadband. The gripper thread then calls `move(width, speed)`. Grasping with force
     (`grasp`) is out of scope until a task needs it.
   - **Contact backstop.** Franka's collision thresholds (`setCollisionBehavior`) are set
     conservatively. Any contact the capsule model missed then ends in a reflex stop.
   - **A simulate mode** (`--simulate`). It runs the same loop and socket against an in-process
     model: `Fr3SimBackend`'s tracking lag, ported. RobotHost's client, the protocol and the recipes
     can then run end to end on any Linux machine before the robot arrives.
   - **It never moves on its own.** At start it reads each arm's measured pose and starts the shaper
     there. It only moves toward targets RobotHost sent while enabled.

4. **RobotHost runs on the real-time PC, beside the server, over a Unix datagram socket.** The new
   backend is `ArmServerClient : IArmBackend` (`--backend arm-server --arm-server-socket <path>`). It
   was called `FciArmServerClient` here until it was built for Isaac first (ADR 0031).
   - **Same machine.** There is no network between the targets and the shaper. A garbage-collection
     pause in RobotHost only delays a target, and the shaper's gap policy absorbs that (ADR 0027
     decision 2).
   - **One clock.** The server stamps its readings with the clock RobotHost's `Stopwatch` reads on
     Linux, `CLOCK_MONOTONIC` at 1 GHz (*verify* for the .NET version deployed). The stamps are then
     on RobotHost's timebase without any synchronization.
   - **RobotHost keeps the rest:**
     - the joint port and `ClockSync`;
     - the limits in `SerialChainPlant`;
     - the collision model (ADR 0026, 0029). It refuses colliding targets, as the simulated host
       does;
     - power, recording and discovery.
   - **The simulated host stays on the PC** (`just fr3-sim-host`), unchanged.

5. **The protocol: arm-server wire v1.** Datagrams are little-endian, with a kind byte, a version
   and a sequence number, in the relay's shape.
   - **To the server:**
     - **targets**: host time, then for each motor its id and value, in radians for a joint or the
       0..1 opening for a hand;
     - **power**: hold, park, stop, recover (decisions 6 and 7).
   - **From the server**, every 4 ms per arm (the cadence of the `QS` records), each arm's state:
     - the host time of the reading, and libfranka's own robot time;
     - measured positions and velocities, and the shaped command;
     - finger width;
     - robot mode (`franka::RobotMode`) and error flags;
     - loop health: missed and late cycles, and the limiter count.
   - **Codecs on both sides, changed together.** The C# codec lives in RobotHost and the C++ codec
     in the server. Golden byte files written by C# are read by the C++ tests, as with
     `RelayProtocol.cs` and `relay_protocol.py`.
   - **No libfranka in the protocol.** The server's simulate mode speaks it, and so could another
     backend later (decision 12).

6. **Power: on the FR3, park is stop-and-brake.** The headset keeps its three motion modes and its
   messages (ADR 0023 decision 5: `ArmPowerRequest`, `ArmPowerState`), so the operator's buttons do
   not change. On the FR3 they mean:
   - **On (hold).** Unlock the brakes if they are locked. Start each arm's loop with the shaper at the
     measured pose, then forward targets. The headset's IK restarts from the measured pose, as after a
     JetRover relax.
   - **Paused.** No targets are sent. The loops hold the last shaped target and the brakes stay open.
   - **Off (relax).** Park, end the loops, then lock the brakes.
     - **The park path** goes to a rest pose along a joint path checked against the collision model.
       It is sampled and refused on any collision, as the JetRover's park is.
     - **The state.** On the FR3, `ArmPowerState.Relaxed` means "parked, brakes locked".
     - **No limp arm.** There is no torque-off as on the JetRover: an FR3 holds itself while its loop
       runs, and its brakes hold it after.
   - **The brakes** are locked and unlocked through Desk's interface, as Desk's own buttons do it
     (*verify*: the endpoints and the control token for the delivered system version). Until that is
     verified, the operator locks and unlocks them in Desk, and a park ends with the loops stopped
     and the arm holding.
   - **The rest pose** is measured on the robot, as the JetRover's was. Until it is, every park is
     refused, as on a JetRover started without `--rest-pose-deg`.
   - **The user stop** belongs to the person at the robot. It works independently of all of this, and
     the repo's rules for moving real hardware apply (`CLAUDE.md`, "Testing the real robot").

7. **Errors are never recovered automatically.**
   - **A reflex** (a `franka::ControlException`) ends that arm's loop. The server reports it, and
     RobotHost passes it to the headset as the reason the arm stopped.
   - **Recovery** (`automaticErrorRecovery`) happens only on an explicit request: the operator's hold
     after the reason is shown, or `just fr3-recover`. The shaper then restarts from the measured
     pose.
   - **If RobotHost goes away,** the server keeps holding the last target and never parks or moves on
     its own. A RobotHost that starts again re-attaches.

8. **The real-time PC and its network** (*verify* each against the delivered system):
   - **The PC.** Ubuntu LTS with a `PREEMPT_RT` kernel, fixed CPU frequency, and two cores isolated
     for the arm threads.
   - **The arm links.** One network card per arm, each on a direct cable to that arm's controller,
     with static addresses and nothing else on the link. That is Franka's recommendation for 1 kHz
     control.
   - **The lab link.** A third card on the lab network, over DHCP. The headset, the PC and discovery
     reach RobotHost there (ADR 0022: mDNS and Tailscale names, never a hard-coded address).
   - **The bar before any motion.** libfranka's `communication_test` passes for each arm at its
     stated success rate.

9. **Recipes.** Real hardware is driven only through `just` (`CLAUDE.md`):
   - **`fr3-rt-check`**, read-only: the kernel, the core isolation, the links, and
     `communication_test` for each arm;
   - **`deploy-fr3`**: RobotHost and the arm server to the real-time PC, as services;
   - **`fr3-host`**: RobotHost with the arm server, the real robot's `fr3-sim-host`;
   - **`fr3-state`**, read-only: streams both arms' states and compares the measured tool poses with
     Core's forward kinematics. This is the profile's first real check;
   - **`fr3-park`, `fr3-unlock`, `fr3-lock` and `fr3-recover`**: each says what will move and asks for
     the workspace to be confirmed clear, as `move-arm` does;
   - **`fr3-arm-server-test`**: the shaper and protocol goldens, on any Linux machine, with no robot.

10. **Bring-up order.** Each step is gated on the one before:
    1. `fr3-rt-check` passes.
    2. **Read-only states.** The tool pose from the measured joints matches Franka's own tool pose
       within a stated tolerance. This checks the profile and the mount angles taken from the URDF.
    3. **The joint-limited capsule pairs** (ADR 0029 amendment, item 3) are checked on the robot.
       Each pair's closest poses inside the limits are approached slowly, and the real gap is looked
       at.
    4. **The simulate mode against the real RobotHost** on the real-time PC.
    5. **First motion.** Small moves at a velocity scale of 0.1, with the workspace confirmed clear.
       Then a park-and-brake cycle.
    6. **Headset teleoperation at a reduced scale,** then at the default 0.9.

11. **After the first real sessions: system identification, then fine-tuning.**
    - **Refit the simulator.** Fit the trainer's FR3 simulator (`armpred/sim/fr3.py`: each joint's
      tracking lag, the gravity and payload bias, the reading noise) to real pairs of targets sent
      (`AO`) and states measured (`QS`). The fit replaces ADR 0027's guessed parameters (20 Hz,
      damping 1).
    - **Measure the prior.** Measure `franka-replay`'s error on real sessions. It should be the
      tracking lag plus the network.
    - **Retrain.** Re-run the simulated pretraining with the fitted simulator, then fine-tune on real
      sessions under a pre-registered split (ADR 0028's gate), as was done for the JetRover (ADR 0021,
      0025).

12. **Not decided here: a physics backend.** The user has asked about Isaac Sim.
    - **What the seam allows.** An Isaac Sim process, on the Linux GPU machine for example, could sit
      behind the same wire protocol as another backend. It would give the contacts and payloads that
      the kinematic `Plant/` and the simulated backend do not.
    - **Why it needs its own ADR.** It would be the first part of the system that models the
      environment (`CLAUDE.md`'s Direction paragraph), which makes it an architecture change.
    - **What this ADR does.** It only keeps the protocol free of libfranka, so that such a bridge can
      speak it. The FR3's head camera (ADR 0014's path) is also left to its own change.

## Consequences

- **RobotHost moves to the real-time PC for the real robot.** The PC keeps the simulated host.
- **C++ joins the robot path,** in this repository as the ROS code is, and built apart from .NET and
  Unity. The shaper copy is proved correct by a golden test before it drives an arm (ADR 0027
  consequence 2).
- **`franka-replay` stays exact on the real robot only while two things hold:** the limiter count is
  zero, and the low-pass filter is off. Both are reported, and a session that breaks either says so.
- **Real sessions are session v2 with `"source": "robot"`.** Their truth is the server's measured
  states. The server's loop health and errors become new records, defined in
  `trainer/contract/SESSION_FORMAT.md` by the change that builds them.
- **Purchases:** a real-time PC with three network cards (*verify* the count against the Duo's
  wiring).
- **The JetRover is untouched:** its relay, recipes and sessions.

## Alternatives considered

- **franka_ros2 with a `ros2_control` controller holding the shaper.** Rejected: it is the same C++
  shaper with more around it (decision 1). Worth revisiting only if MoveIt or RViz tooling becomes
  necessary.
- **RobotHost on another machine, talking to the server over UDP.** Rejected for now. A network hop
  between the targets and the shaper buys nothing, and one clock is simpler than two that need
  syncing. The protocol uses datagrams, so it can move to UDP if that ever changes.
- **Streaming the shaper's 1 kHz output from RobotHost.** Rejected. .NET with a garbage collector
  cannot meet a 1 ms deadline over a socket: a late packet is a missed cycle and an aborted motion.
- **A C++ copy of the capsule collision check in the server, as a second line of defence.** Deferred.
  RobotHost already refuses colliding targets, and Franka's contact reflex stops any contact the
  model missed (decision 3). Revisit if bring-up shows a gap.
- **Torque-off for relax, as on the JetRover.** It does not fit the FR3, which holds itself or is held
  by its brakes. Off becomes park and brake (decision 6).

## Open questions to verify on delivery

- **The arms and the wiring:** whether each arm is a separate FCI robot with its own address, and
  whether the Duo's wiring allows one network card per arm or ships with its own switch.
- **Software versions:** the robot's system version, the libfranka version it requires, and FCI
  activation in Desk.
- **Desk's interface:** its brake and FCI calls, and the control token, for that version. Otherwise
  the brakes are handled by hand in Desk.
- **The user stop:** how it behaves (stop category, brakes) and how the server sees it
  (`kUserStopped`).
- **The mount:** whether the delivered mount matches the URDF's mount angles (decision 10, step 2).
- **The capsule pairs:** the joint-limited pairs of ADR 0029.
- **The rest pose,** measured on the robot.
