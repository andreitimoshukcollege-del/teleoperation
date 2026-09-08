# Jetson systemd units

Three services that make the robot side come up on its own after any Jetson boot/reboot, so it's
already reachable by the time you open Unity or run `just move-arm`/etc. — no manual SSH launch
step required (see root `README.md`'s JetRover section for the full picture).

| Unit | Runs |
|---|---|
| `teleop-robothost.service` | `Teleop.RobotHost` via `/home/jetson/robothost-run.sh` (a stable wrapper `just deploy-robothost` regenerates on every deploy — see that recipe) |
| `jetrover-relay.service` | `teleop_relay`'s `relay_node` |
| `jetrover-arm-control.service` | `jetrover_arm_control`'s `robot_controller_manager` |

All three: `Restart=on-failure`, `RestartSec=2`, `WantedBy=multi-user.target` (enabled = starts on
boot). The two ROS units explicitly `source` both required setup scripts inside `ExecStart`, since
systemd bypasses `.bashrc` entirely — do not rely on the Jetson's own `.bashrc` here, it currently
sources a different, stale path from an unrelated earlier project.

## Installing (one-time per Jetson, or after a fresh flash)

```bash
scp robot/systemd/*.service jetson@<jetson-ip>:/tmp/
ssh jetson@<jetson-ip>
sudo cp /tmp/teleop-robothost.service /tmp/jetrover-relay.service /tmp/jetrover-arm-control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now teleop-robothost jetrover-relay jetrover-arm-control
systemctl status teleop-robothost jetrover-relay jetrover-arm-control --no-pager
```

`teleop-robothost.service` won't start cleanly the very first time until `/home/jetson/robothost-run.sh`
exists — run `just deploy-robothost` once first (it creates/overwrites that script on every deploy
regardless of whether the service unit is installed yet).

Or from a dev machine in one shot: `just robot-status` (checks all three) after running the above.

## Why this replaced manual `nohup`/SSH launches

Every prior JetRover hardware session in this repo's history started these three processes by
hand over SSH after every reboot — easy to forget a step, and easy for one to silently not be
running without anyone noticing until something didn't move. `Restart=on-failure` also means a
crash (not just a reboot) self-heals without a human present.
