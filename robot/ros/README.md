# jetrover-teleop-ros

ROS 2 (Foxy) workspace for controlling a Hiwonder JetRover's 4-DOF arm and gripper, for use as
the real-hardware side of [teleoperation](https://github.com/andreitimoshukcollege-del/teleoperation)'s
`IRobotPlant` integration.

Ported and trimmed from [`SINRG-Lab/industryxr-robot`](https://github.com/SINRG-Lab/industryxr-robot)'s
`sinrg_robot_sdk` package: the camera/perception code, `ros_tcp_endpoint`, and other pieces
unrelated to arm/gripper control were left behind, and the package was renamed to
`jetrover_arm_control`.

## Packages

- **`jetrover_arm_control`** — wraps the Hiwonder STM32 board's serial bus-servo protocol
  (`/dev/ttyACM0`) and exposes the arm's 4 joints + 2-part gripper as ROS topics
  (`/arm/servo/{base,joint/lower,joint/middle,joint/upper,gripper/base,gripper/main}`), plus
  battery/IMU. See that package's own code for topic names and message types.

A "relay" package bridging this to `teleoperation`'s `Teleop.RobotHost` (a small .NET process
that is the real `IRobotPlant`/`ITransport` endpoint) will be added here in a later phase — see
`teleoperation`'s `robot/README.md` for the up-to-date protocol/architecture pointer.

## Build

```bash
colcon build
source install/setup.bash
ros2 launch jetrover_arm_control jetrover_arm.launch.py
```

### System requirements

- ROS 2 Foxy
- Python 3's `pyserial` (`serial` import) for the board's `/dev/ttyACM0` connection

## License

MIT (see `LICENSE`), matching the upstream `industryxr-robot` project this was ported from.
