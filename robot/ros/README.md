# robot/ros — the JetRover's ROS 2 workspace

The ROS 2 (Foxy) colcon workspace that runs on the JetRover's Jetson and drives the Hiwonder board.
It moved here from [`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros)
on 2026-10-15, with its history (docs/adr/0019). That repository is left as it was; this one is the
source of truth. `../README.md` is the architecture and the incident log.

It was ported and trimmed from [`SINRG-Lab/industryxr-robot`](https://github.com/SINRG-Lab/industryxr-robot)'s
`sinrg_robot_sdk`. The camera and perception code, `ros_tcp_endpoint`, and other pieces unrelated
to arm control were left behind (the camera is now served by `core/Teleop.CameraHost`).

## Packages

- **`jetrover_arm_control`** drives the Hiwonder STM32 board over its serial bus-servo protocol
  (`/dev/ttyACM0`):
  - it takes per-motor targets on `/arm/servo/command_array`;
  - it publishes servo feedback on `/unity/robot/servo/feedback_array` and the battery level on
    `/robot/battery` (millivolts).
  - **`FEEDBACK_MOTOR_IDS`** in `robot_controller_node.py` lists the servos it reads back. That is
    1–5. Motor 10, the gripper, is left out: it never answers, and each read of it used to stall
    the loop for a second.
  - `board_manager/robot_controller_sdk.py` is Hiwonder's board library. `just robot-board-commands`
    lists what the board accepts.
- **`teleop_relay`** bridges RobotHost to the driver over a Unix socket
  (`/tmp/jetrover_relay.sock`):
  - `relay_protocol.py` is the wire format. It must match `core/Teleop.RobotHost/Relay/RelayProtocol.cs`
    exactly, so change both in the same PR, and deploy both together. Version 5 (2026-10-26) adds a
    kind byte and a torque message, which the relay publishes on the driver's `/arm/servo/set/torque`
    (docs/adr/0023 decision 5).
  - `stale_tracked_value.py` stops a servo that stopped answering from being reported as measured
    forever.

## Build, test, deploy

From the repository root, never by hand on the Jetson:

```bash
just deploy-ros false            # copy to the Jetson, build it there, run the Python tests
just deploy-ros                  # ...and switch the two ROS services to it (someone at the robot)
just robot-ros-diff              # read-only: is the Jetson running exactly what is here?
```

The plain-Python tests (`test_relay_protocol.py`, `test_stale_tracked_value.py`,
`test_servo_controller.py`) need only `pytest`. The ament lint tests (`test_flake8.py`,
`test_pep257.py`, `test_copyright.py`) need ROS and are not run by the deploy.

Requirements on the Jetson: ROS 2 Foxy, and Python 3's `pyserial`.

## License

MIT (see `LICENSE`), matching the upstream `industryxr-robot` project this was ported from.
