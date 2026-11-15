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
│   ├── robothost/                   RobotHost, as the Isaac hosts build it
│   ├── work/                        the URDFs converted to USD
│   ├── backdrops/                   the Open Images backdrops (just isaac-backdrops-fetch; the scene files name this path)
│   └── jetrover-uplink              the uplink impairment's control file (just isaac-uplink-*)
├── data/                            everything recorded, and what is built from it
│   ├── sessions/<robot>-<source>/   automatic sessions: jetrover-isaac (isaac-jetrover-host), fr3_duo-isaac (isaac-host)
│   ├── takes/<robot>-<source>/      the RECORD button's takes, <scenario>/<UTC>-take<n>/, set-asides in _discarded/:
│   │                                jetrover-isaac and fr3_duo-isaac (the twins), jetrover-real (copied from the robot)
│   ├── lerobot/<robot>-<source>/    the LeRobot datasets (ADR 0036); jetrover-isaac-safety/ holds the safety batches;
│   │                                robot-hosts (the robot's address) and skipped.jsonl (takes not converted)
│   └── network/                     the public network datasets (ADR 0035 N2)
├── analysis/                        studies' outputs: render-quality/, motion/ (the human-teleop motion scorer),
│                                    simfit/ (just jetrover-sim-fit), render-check/
├── logs/
│   ├── isaac/                       the bridges', renderer's, RobotHost build's and Isaac setup's logs
│   ├── lerobot/                     the watcher's log and pid file, and its cron keeper's log
│   └── training/                    train-sim-<run>.log and the network datasets' fetch log, with their pid files
├── repos/                           checkouts the recipes keep, each at the commit it was last asked to run:
│                                    isaac (the Isaac hosts), lerobot (the watcher), safety (the safety batch's checker),
│                                    datasets, simfit, lerobot-test
└── teleoperation/                   the trainer's checkout (just gpu-train): its trainer/.venv and trainer/runs/<run>/
```

The labmate's account reads parts of `data/` once `just isaac-share-scenarios` has run:

- `data/takes/fr3_duo-isaac` and `data/takes/jetrover-isaac`: read, and edit their `scenarios.txt`;
- `data/takes/jetrover-real` and `data/lerobot`: read only.

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
