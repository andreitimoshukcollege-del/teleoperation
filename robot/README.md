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
                                                                          (/arm/servo/base today)
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
- Two real, pre-existing bugs in the ported `jetrover_arm_control` SDK were found and fixed along
  the way (both in `jetrover-teleop-ros`, not here): an uncaught `queue.Empty` on a board-read
  timeout that crashed the whole ROS node, and a single-slot response queue with no
  request/response correlation that silently dropped fresh servo-position responses whenever a
  previous request's response had arrived late -- see that repo's commit history for details.
- `CommandFrame.Pose` isn't converted through real inverse kinematics yet (Phase 2) -- Phase 1
  uses a documented, temporary stand-in (`JetRoverPlant`'s own doc comment) to prove the pipe
  works. Gripper and the lower/middle/upper joints are not wired up yet either.
- Mecanum base motion is explicitly out of scope for now — no wheel odometry exists anywhere in
  the board SDK, so a base plant could only ever be dead-reckoned from commanded velocity, never
  sensed.
- Real cross-machine `ClockSync` validation (Phase 3) hasn't been done yet -- today's test proved
  the pipe works, not that its latency numbers are trustworthy yet.
