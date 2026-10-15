# 19. The robot's ROS 2 code lives in this repository

## Status

Accepted (2026-10-15). The user asked for the robot side to be brought in, and for the old
repository to be left as it is, not archived.

## Context

The JetRover runs two kinds of software:
- **From this repository:** `Teleop.RobotHost` and `Teleop.CameraHost` (.NET). `just deploy-robothost`
  and `just deploy-camerahost` build them here and copy them over.
- **From [`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros):**
  a ROS 2 Foxy colcon workspace with two packages. `jetrover_arm_control` drives the Hiwonder
  board and its servo bus. `teleop_relay` bridges it to RobotHost over a Unix socket. The Jetson
  ran it from its own git checkout, `~/projects/jetrover-teleop-ros`, edited and rebuilt in place.

The two halves share a wire format by hand: `RelayProtocol.cs` here, and `relay_protocol.py`
there, must match exactly (`robot/README.md`). Keeping them in two repositories let them drift:
- The ROS side of ADR 0011 (per-joint, motor-id-keyed relay messages for six joints) was never
  pushed. GitHub's newest relay still speaks the four-joint format. The only working copy was on
  the Jetson.
- PR #3 there (relay feedback validity latching) was never merged, and branch
  `absolute-joint-targets` (ADR 0010's ROS side) has no PR.
- The servo-feedback ids (`FEEDBACK_MOTOR_IDS`, motors 5 and 10 added 2026-10-12) were changed on
  the Jetson only.

A change to either half could not be reviewed, tested or rolled back together with the other.

## Decision

1. **The workspace moves to `robot/ros/`** (`robot/ros/src/jetrover_arm_control`,
   `robot/ros/src/teleop_relay`). It is imported from the Jetson's own checkout (`just
   robot-ros-fetch`), with its full git history and its uncommitted changes, rather than from
   GitHub, because the Jetson's copy is the one that runs.
2. **`robot/` is no longer documentation only.** The root CLAUDE.md row "colcon (ROS 2);
   independent; does not interact with the above builds" still holds: nothing in `core/` or
   `unity/` builds or references `robot/ros/`. The coupling is the relay wire format, as before,
   now in one repository.
3. **`just deploy-ros` deploys it like RobotHost.**
   - It copies `robot/ros/` to a new `~/teleop_ros_deploy_<time>/` on the Jetson and builds it
     there.
   - It points `~/teleop_ros_current` at it, installs `robot/systemd`'s two ROS units (they now
     source `~/teleop_ros_current/install/setup.bash`), and restarts them.
   - `activate=false` only builds.
   - Rollback is repointing the link. The Jetson needs no git checkout, and never clones this
     repository (its Unity assets are large and LFS).
4. **The old repository stays as it is.** It is not archived and not pushed to. It is history; this
   repository is the source of truth.
5. **The Hiwonder board library** (`robot_controller_sdk.py`) comes along. All three repositories
   involved (this one, `jetrover-teleop-ros`, and `SINRG-Lab/industryxr-robot` it was trimmed from)
   are public and MIT-licensed, so nothing is published that was not already.

## Consequences

- **A relay protocol change is one PR:** `RelayProtocol.cs` and `relay_protocol.py` change
  together, and the reviewer sees both.
- **No CI coverage for the ROS code.** CI builds .NET only. Building the ROS code needs ROS 2 Foxy
  on Ubuntu 20.04 (the Jetson's). Its pure-Python tests (the relay protocol, the stale-value
  tracker) can run under plain `pytest` and could be added to CI later. Until then, `just
  deploy-ros activate=false` is the build check, and it runs on the Jetson.
- **Settings once changed on the Jetson by hand, such as `FEEDBACK_MOTOR_IDS`,** are now edits here
  followed by `just deploy-ros`. `just robot-set-feedback-ids`, which edited the Jetson's checkout
  in place, is removed.
- **The read-only recipes look in the new place.** `ros-check`, `robot-feedback-ids` and
  `robot-battery` read `~/teleop_ros_current/src`, falling back to the old checkout on a Jetson
  that has not had `deploy-ros` yet.
