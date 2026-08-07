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
  RobotEndpoint (Core)
  JetRoverPlant : IRobotPlant
      |
      | Unix domain socket, local only, tiny fixed struct (see "Local relay protocol" below)
      v
  relay node (planned)  ─────────────────────────────────────────────>  jetrover_arm_control
                                                                          (existing /arm/servo/* topics)
```

The "relay" package bridging `Teleop.RobotHost` to `jetrover_arm_control`'s topics doesn't exist
yet — it's planned for `jetrover-teleop-ros`, not here. This file will be updated with the local
relay protocol's exact wire format once that phase lands.

## Status

- Arm + gripper control confirmed working end-to-end from `jetrover-teleop-ros`, via real ROS 2
  topics, on the physical robot (supervised hardware test).
- Mecanum base motion is explicitly out of scope for now — no wheel odometry exists anywhere in
  the board SDK, so a base plant could only ever be dead-reckoned from commanded velocity, never
  sensed.
- `Teleop.RobotHost`, `JetRoverPlant`, and the relay node are not yet built.
