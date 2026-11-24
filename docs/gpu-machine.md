# The GPU machine's account

This is the account `andrei@` on the lab's RTX 5090 box. It is reached over the lab VPN. The machine is on DHCP, so its
address moves; the justfile's `gpu_host` holds the current one.

Everything the recipes run there lives in this account, laid out as below since 2026-11-15. The recipes create what
they need. Nothing here is edited by hand, except a take list (`scenarios.txt`).

```
~/
├── isaacsim/                        the twin's software (a virtualenv breaks if moved: these stay put)
│   ├── venv/                        Isaac Sim (just isaac-setup); VERSION names the install
│   ├── lerobot-venv/                the LeRobot converter's environment (just lerobot-setup)
│   ├── robothost/                   RobotHost, as the Isaac hosts build it (robothost-1/: twin 1's, ADR 0037 amendment 7)
│   ├── work/                        the URDFs converted to USD
│   ├── backdrops/                   the Open Images backdrops (just isaac-backdrops-fetch; the scene files name this path)
│   ├── assets/hdri/polyhaven-indoor/ the HDRI rooms around the JetRover twin (ADR 0037; just isaac-environments-fetch; jetrover-bench names this path)
│   └── jetrover-uplink              the uplink impairment's control file (just isaac-uplink-*; twin 0's: twin 1 has jetrover-uplink-1)
├── data/                            everything recorded, and what is built from it
│   ├── sessions/<robot>-<source>/   automatic sessions: jetrover-isaac (isaac-jetrover-host), fr3_duo-isaac (isaac-host);
│   │                                jetrover-isaac-twin1 (the JetRover's second twin, ADR 0037 amendment 7)
│   ├── takes/<robot>-<source>/      the RECORD button's takes, <scenario>/<UTC>-take<n>/, set-asides in _discarded/:
│   │                                jetrover-isaac and fr3_duo-isaac (the twins), jetrover-isaac-twin1 (the JetRover's
│   │                                second twin), jetrover-real (copied from the robot)
│   ├── lerobot/<robot>-<source>/    the LeRobot datasets (ADR 0036); jetrover-isaac-safety/ holds the safety batches;
│   │                                robot-hosts (the robot's address) and skipped.jsonl (takes not converted)
│   └── network/                     the public network datasets (ADR 0035 N2)
├── analysis/                        studies' outputs: render-quality/, motion/ (the human-teleop motion scorer),
│                                    simfit/ (just jetrover-sim-fit), render-check/
├── logs/
│   ├── isaac/                       the bridges', renderer's, RobotHost build's and Isaac setup's logs; twin 1's are
│   │                                suffixed -1 (jetrover-bridge-1.log, renderer-1.log, robothost-build-1.log), and a twin
│   │                                started with detach=true logs to jetrover-host[-1].log
│   ├── lerobot/                     the watcher's log and pid file, and its cron keeper's log
│   └── training/                    train-sim-<run>.log and the network datasets' fetch log, with their pid files
├── repos/                           checkouts the recipes keep, each at the commit it was last asked to run:
│                                    isaac (the Isaac hosts), lerobot (the watcher), safety (the safety batch's checker),
│                                    safety-check (isaac-safety-build-check, isaac-safety-rebuild), datasets, simfit,
│                                    lerobot-test
└── teleoperation/                   the trainer's checkout (just gpu-train): its trainer/.venv and trainer/runs/<run>/
```

The labmate's account reads parts of `data/` once `just isaac-share-scenarios` has run. It can list the home folder,
`data/` and `data/takes/` (an "enter only" entry there made `ls` fail, 2026-11-17); what is private keeps its own mode
(`.ssh`, `.config`, `.cache`, `.local` are 700):

- `data/takes/fr3_duo-isaac`, `data/takes/jetrover-isaac` and `data/takes/jetrover-isaac-twin1`: read, and edit their
  `scenarios.txt`;
- `data/takes/jetrover-real` and `data/lerobot`: read only.

What runs there now, how far along it is, and buttons to stop or pause it: `just gpu-ui` (robot/gpu).

## Two JetRover twins (ADR 0037 amendment 7)

Twin 1 runs beside twin 0 from the same checkout (`repos/isaac`), so both run one commit. Its ports are twin 0's plus 100,
its runtime files carry `-1` and its data folders `-twin1`:

| | Twin 0 | Twin 1 |
|---|---|---|
| RobotHost's ports: headset pose, reply, joint, discovery (UDP) | 6000, 6001, 6002, 6005 | 6100, 6101, 6102, 6105 |
| Camera feeds: wrist, third-left, third-right (UDP) | 6003, 6008, 6009 | 6103, 6108, 6109 |
| Relay and world sockets | `/tmp/teleop-jetrover-relay-andrei.sock`, `.world`, `.robothost` | `/tmp/teleop-jetrover-relay-andrei-1.sock`, ... |
| Renderer's tap socket | `/tmp/teleop-isaac-tap-andrei.sock` | `/tmp/teleop-isaac-tap-andrei-1.sock` |
| Sessions (and the record control file `.active`) | `data/sessions/jetrover-isaac` | `data/sessions/jetrover-isaac-twin1` |
| Takes | `data/takes/jetrover-isaac` | `data/takes/jetrover-isaac-twin1` |
| Uplink control | `isaacsim/jetrover-uplink` | `isaacsim/jetrover-uplink-1` |
| RobotHost build | `isaacsim/robothost` | `isaacsim/robothost-1` |
| Logs | `jetrover-bridge.log`, `renderer.log`, `robothost-build.log` | `jetrover-bridge-1.log`, `renderer-1.log`, `robothost-build-1.log` |
| Cores: bridge, renderer | 2-5, 8-23 | 26-29, 8-23 (shared) |

Each twin's processes are told apart by the record control file they all name (`--record-control .../.active`). The
headset, `isaac-relay` and the `isaac-uplink-*` recipes work with twin 0 only. Only twin 0 answers ADR 0022's discovery on
the usual port.

Disk: about 10 GB an hour of a twin's recording (three or four cameras' colour, depth and masks). RobotHost stops a take
below 21 GB free, and the renderer stops recording below 20 GB.

## Moved on 2026-11-15

| Before | Now |
|---|---|
| `isaacsim/recordings` | `data/sessions/fr3_duo-isaac` |
| `isaacsim/jetrover-recordings` | `data/sessions/jetrover-isaac` |
| `isaacsim/scenarios` | `data/takes/fr3_duo-isaac` |
| `isaacsim/scenarios-jetrover` | `data/takes/jetrover-isaac` |
| `isaacsim/scenarios-real` | `data/takes/jetrover-real` |
| `isaacsim/lerobot` | `data/lerobot` (its logs: `logs/lerobot`) |
| `datasets/network` | `data/network` |
| `isaacsim/render-quality`, `motion-analyst`, `simfit` | `analysis/render-quality`, `motion`, `simfit` |
| `isaacsim/*.log` | `logs/isaac/` |
| `train-sim-*.log`, `network-datasets.log` and their pid files | `logs/training/` |
| `teleoperation-isaac`, `-lerobot`, `-safety`, `-datasets`, `-simfit`, `-lerobot-test` | `repos/isaac`, `lerobot`, `safety`, `datasets`, `simfit`, `lerobot-test` |

The LeRobot datasets' ledgers (`meta/teleop/takes.jsonl`) and `skipped.jsonl` name each take by its path. They were
rewritten to the new paths in the same move. The watcher matches a take by its path and its `files.sha256`, and would
otherwise have rebuilt every dataset.
