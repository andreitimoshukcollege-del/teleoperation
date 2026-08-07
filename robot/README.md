# robot/

Documentation only — no colcon workspace lives here. The root `CLAUDE.md` directory table's
promise that `robot/` "does not interact with the above builds" is kept literally: the actual
ROS 2 code for the physical JetRover lives in its own repo, built and run on the Jetson, never
referenced by `core/`'s or `unity/`'s build.

## Where the real code lives

**[`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros)** —
a ROS 2 (Foxy) colcon workspace with one package, `jetrover_arm_control`, wrapping the Hiwonder
JetRover's STM32 board serial protocol and exposing its 4-DOF arm + gripper as ROS topics. Ported
and trimmed from `SINRG-Lab/industryxr-robot`'s `sinrg_robot_sdk` — see that repo's README for
what was kept and what was left behind (camera/perception code, `ros_tcp_endpoint`, etc.).

Runs on the JetRover's Jetson Nano, reachable over Tailscale.

## How it connects to this repo

`IRobotPlant` (`core/Teleop.Core/Contracts/IRobotPlant.cs`) is Core's contract for "the robot
being commanded." A real hardware implementation, `JetRoverPlant`, lives in a new sibling .NET
project, `core/Teleop.RobotHost/` — **not** in `unity/`'s `Bridge/` and **not** in this
directory. See that project's own docs (once it exists) and
`docs/adr/0007-jetrover-plant-and-robot-host.md` for the full architecture and why.

```
Teleop.RobotHost (on the Jetson)          jetrover-teleop-ros (on the Jetson)
  UdpTransport : ITransport   <-- real UDP over Tailscale, from the operator side
  RobotEndpoint (Core, unmodified)
  JetRoverPlant : IRobotPlant
      |
      | Unix domain socket, local only, tiny fixed struct (Relay/RelayProtocol.cs /
      | teleop_relay/relay_protocol.py -- must match exactly, kept in sync by hand)
      v
  teleop_relay (ROS 2 node) ──────────────────────────────────────────>  jetrover_arm_control
                                                                          (base/lower/middle/upper
                                                                           joints + gripper)
```

`teleop_relay` is a real, working ROS 2 package in `jetrover-teleop-ros` (not planned anymore) --
a thin node with no staleness/sequencing/gap-policy logic of its own, translating the local wire
protocol above into calls against `jetrover_arm_control`'s existing topics.

## Status

- **Phase 1 (arm base servo only) confirmed working fully end-to-end on the physical robot**: a
  real `CommandFrame`, sent over genuine UDP through Tailscale from a dev-machine test harness,
  decoded by `Teleop.RobotHost`'s `UdpTransport`, applied by Core's unmodified `RobotEndpoint` to
  a real `JetRoverPlant`, forwarded over a Unix domain socket to `teleop_relay`, published to
  `jetrover_arm_control`'s `/arm/servo/base` topic, and physically moving the real servo --
  visually confirmed. Feedback round-trips correctly back through the same chain into the
  `RobotStateFrame` reply.
- **Phase 2 (real inverse kinematics, all four position-affecting joints, gripper) also confirmed
  working on the physical robot.** `JetRoverPlant.Command` now runs
  `Kinematics/FourDofArmKinematics.cs` against `CommandFrame.Pose` instead of Phase 1's stand-in,
  and denormalizes `CommandFrame.Gripper`. A commanded Cartesian target moved the arm to
  approximately the right position -- X/Y matched the intended target closely; Z was
  off by the amount explained below (not a sign or math error -- confirmed by hand-deriving the
  IK for the test target and cross-checking against a passing round-trip test suite).
- **Real, hardware-level finding: this JetRover's middle-arm servo (ID 3) never responds to
  position-read requests**, confirmed independently of ROS by calling the board SDK directly,
  repeatedly, with nothing else running. Writes to it work fine (it visibly moves); only reads
  never succeed. This is not a software bug to fix -- `JetRoverPlant.State` already degrades
  honestly for exactly this case (`IsFullySensed` reports false; the affected joint's contribution
  falls back to this plant's last-commanded target rather than a stale sensed value or a fixed
  default). It is the reason Phase 2's real-hardware Z result didn't exactly match the commanded
  target: the middle joint's actual contribution to `State` is an estimate, not a measurement,
  until/unless this servo's read path is fixed at the hardware or firmware level.
- Three real, pre-existing bugs in the ported `jetrover_arm_control` SDK were found and fixed
  along the way (all in `jetrover-teleop-ros`, not here): an uncaught `queue.Empty` on a
  board-read timeout that crashed the whole ROS node; a single-slot response queue with no
  request/response correlation that silently dropped fresh servo-position responses whenever a
  previous request's response had arrived late; and (found during Phase 2) the same
  ROS-side node's `pulseToDeg` uses an assumed 180-degree servo range where Hiwonder's own
  published docs confirm the real range is 240 degrees -- `JetRoverPlantConfig` accounts for this
  explicitly (`PulsePerRadian` vs `PulsePerDegreeAssumed180`) rather than treating it as fixed.
- A real bug was also found and fixed in `JetRoverPlant` itself during Phase 2's hardware testing:
  when a single command's required delta exceeded the per-call direction clamp, the plant
  credited its own belief with the full, unclamped target instead of the amount actually applied
  -- silently stalling large moves forever instead of closing the remaining distance over
  repeated commands. See `JetRoverPlant`'s own doc comment and its regression test.
- Mecanum base motion is explicitly out of scope for now — no wheel odometry exists anywhere in
  the board SDK, so a base plant could only ever be dead-reckoned from commanded velocity, never
  sensed.
- Real cross-machine `ClockSync` validation (Phase 3) hasn't been done yet -- Phases 1-2's tests
  proved the pipe (and now the kinematics) work, not that the latency numbers crossing it are
  trustworthy yet.
