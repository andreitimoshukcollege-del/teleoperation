# Session format, versions 1 and 2

What the robot records (docs/adr/0021 decision 3), and what the simulator writes, so that a simulated
session and a recorded one travel the same path. Version 1 is the JetRover's and everything below
describes it; version 2 is a serial-chain robot's (the FR3 Duo, docs/adr/0027 decision 6) and is
described in its own section at the end, as what differs from version 1. Written by RobotHost (C#), by the ROS driver
(Python) and by `armpred/sim`; read by `armpred/contract/session.py`; checked by
`armpred check-session` (`just recording-check`).

## A session is a directory

`recordings/jetrover/<UTC>-<label>/` (gitignored; archived to a GitHub Release). A session RobotHost
opened itself (docs/adr/0023 decision 4) is `<UTC>-auto/`, or `<UTC>-auto-<n>/` when one second holds
two; its `label` is still `auto`.

| File | Written by | Required |
|---|---|---|
| `session.json` | `just record-start`, completed by `just record-stop`; for an automatic session, RobotHost (both steps) | yes |
| `robothost.jlog` | RobotHost | yes |
| `driver.jlog` | the ROS driver (`jetrover_arm_control/session_log.py`) | yes |
| `robothost.<n>.jlog`, `driver.<n>.jlog` | the same writer, after it restarted during the session (n = 2, 3, ...) | no; not read |
| `marks.jlog` | `just record-mark` | no |
| `operator.jlog` | `joint-drive` (later, the Bridge) | no |
| `latents.jlog` | the simulator only | sim only |
| `files.sha256` | `just record-stop`, or RobotHost for an automatic session (`sha256sum` format) | yes, for a packed session |

**A log that exists is never truncated.** A writer that restarts while the control file still names
the session leaves `<writer>.jlog` as it was (cut off, without a trailer: the session fails the check)
and writes a complete file of its own, `<writer>.2.jlog`. Readers read only `<writer>.jlog`; the
segments keep what was recorded after the restart, for a person to look at.

**Automatic sessions** are marked on the robot, not in the session: while `~/recordings/.active-auto`
holds the same directory as the control file `~/recordings/.active`, RobotHost opened that session
and closes and completes it. In the session they are recognised by `label` and `motion` `auto`.

## `session.json`

```json
{
  "format": "jetrover-session",
  "version": 1,
  "id": "20261021T153000Z-teleop-2p5-none",
  "source": "robot",
  "label": "teleop-2p5-none",
  "startedUtc": "2026-10-21T15:30:00Z",
  "stoppedUtc": "2026-10-21T15:40:03Z",
  "conditions": {
    "speedSliderRadPerSec": 2.5,
    "load": { "kind": "none", "payloadGrams": 0 },
    "network": { "uplinkProfile": "lan", "uplinkSeed": 0, "addedDelayMs": 0 },
    "motion": "teleop",
    "operator": "A"
  },
  "builds": {
    "robothost": { "sha": "<git sha>", "dirty": false },
    "driver": { "sha": "<git sha>", "dirty": false }
  },
  "robotProfileSha256": "<sha256 of the jetrover.json RobotHost loaded>",
  "notes": ""
}
```

- `source` is `robot` or `sim`.
- `conditions.speedSliderRadPerSec` is a number, or `"varied"` when it was moved during the session,
  or `"unknown"` in an automatic session whose commands carried no speed (readers treat it as
  `varied`: not a number).
  Since 2026-10-25 `just record-stop` takes it from the speed the session's joint commands carried
  (the headset's slider rides in each command's speed field), not from what `record-start` was told;
  a difference is noted in `notes`. `just recording-set-slider` corrects a session stopped before then.
- `conditions.load.kind` is `none`, `payload` or `hand`, or `unknown` in an automatic session, whose
  `payloadGrams` is then `null`. `armpred sysid` uses only `none`.
- `conditions.network.uplinkProfile` is a profile name from Core's catalog. `lan` means the robot
  impaired nothing. `schedule` comes with `uplinkSchedule` (docs/adr/0021 decision 13). `varied`, in
  an automatic session, means RobotHost's own impairment changed during it; its `UP` records say
  when. `addedDelayMs` (the headset's added delay) is `null` in an automatic session: the robot does
  not know it.
- `conditions.motion` is `teleop`, `joint-drive:<script>`, or `auto` (an automatic session).
- `conditions.operator` is a name, `joint-drive@robot`, or, in an automatic session, `headset <ip:port>`
  (the joint-command sender, which only a headset can be since 2026-10-26: RobotHost records only a
  sender that sends the headset heartbeat, docs/adr/0023 decision 4). Automatic sessions recorded
  before then say `auto <ip:port>`.
- These values were added without a version change: they are new values of existing fields, and
  every reader already handles them (`trainer/tests/test_contract.py`).
- `conditions` describe the session for stratification and splits. **A model never sees them.**

## Lines

Every `.jlog` file is ASCII text with LF line endings, one record per line, fields separated by `|`.

- **Header, the first line:** `JSES|1|<writer>|<ticksPerSecond>|<clock>|<sessionId>`.
  - `writer` is `robothost`, `driver`, `marks`, `operator` or `latents`.
  - `ticksPerSecond` is the unit of every timestamp in the file: 1000000000 on the Jetson.
  - `clock` is `CLOCK_MONOTONIC` for everything written on the Jetson, which RobotHost (`Stopwatch`)
    and the driver (`time.monotonic_ns`) share. `operator.jlog` uses the operator's own clock and
    says so.
- **Trailer, the last line:** `END|<records>|<dropped>|<sha256>`.
  - `records` counts the record lines between the header and the trailer.
  - `dropped` counts records the writer discarded because its ring was full. A session with any
    drop fails the check.
  - `sha256` is 64 lowercase hex digits: SHA-256 over every byte of the file before the trailer
    line, the header included, each line's `\n` included. (SHA-256, not a hand-rolled hash: the
    driver writes in Python on a Jetson Nano, and `hashlib` hashes in C outside the GIL, so logging
    cannot steal time from the driver thread it records.)
  - A file with no trailer was cut off (the writer died) and fails the check.
- **Inside a field,** a list is `;`-separated items whose parts are `:`-separated, e.g.
  `1:0.123:2.5;2:-0.4:2.5`.
- **Numbers:** integers in decimal; floats in round-trip form, invariant culture (`.` decimal
  point): C# `float.ToString("R")` / `double.ToString("R")`, Python `repr(float)`. An empty field
  is absent.
- Records in a file are in the order they were written, which is time order per writer. `JC` is
  written when the plant takes the command, so it is in `tDeliver` order; its `t` can go back in time
  when a robot-side uplink impairment reorders commands.

## RobotHost records (`robothost.jlog`)

All times are RobotHost `Stopwatch` ticks on the Jetson (`CLOCK_MONOTONIC`, ns).

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `KA` | `t` `opTicks` `opTps` | A keepalive arrived at `t`. `opTicks`/`opTps` are the operator stamp and rate it carried |
| `JC` | `t` `tDeliver` `seq` `captureTicks` `outcome` `targets` | A joint command arrived at the socket at `t` and reached the plant at `tDeliver` (equal unless the uplink is impaired on the robot). `captureTicks` is on the sender's clock. `outcome` is `A` accepted, `S` stale (capture not newer than the last accepted), `R` refused (the pose would collide, ADR 0020), `P` ignored because the arm was parking or relaxed (ADR 0023 decision 5, since 2026-10-26; it reached no plant). `targets` is `motor:angleRad:speedRadPerSec;...` as decoded |
| `RO` | `t` `targets` | Targets sent to the relay: `motor:pulse:pulsesPerSecond;...` (the pulse as sent, a float) |
| `RI` | `t` `entries` | A relay feedback datagram taken by the plant: `motor:valid:pulse;...`, `valid` 0 or 1 |
| `FR` | `t` `seq` `robotSendTicks` `sensedTicks` `echoOperatorTicks` `echoRobotRecvTicks` `joints` | A joint-state frame sent, decoded from the bytes that went out. `joints` is `motor:flags:value;...` |
| `SS` | `t` `endpoint` | The joint-state stream's subscriber: written at the start of every session while one is subscribed, then when it changes |
| `JS` | `t` `endpoint` | The joint-command sender: written at the start of every session while one is commanding, or at the session's first command, then when it changes (the staleness check restarted) |
| `UP` | `t` `profile` `seed` | The robot-side uplink impairment, at the start of every session when it is not `none` (since 2026-10-26), then whenever it changes (`profile` `none` is off): from then on, `JC`'s `tDeliver` is when that Core profile released the command, `t` when the socket was read. Keepalives go through it too. A schedule (ADR 0021 decision 13) writes one `UP` per change; a simulated session's trainer-only segments name themselves (`param:...`, `queue:...`, `outage:...`) and its `latents.jlog` `LP uplink_schedule` holds their parameters |
| `OM` | `t` `kind` `mode` `endpoint` | The operator message's sender (docs/adr/0018 §9): its client `kind` (`headset`, or `tool`) and motion `mode` (`on`, `paused`, `off`; the headset's ROBOT buttons), written at the start of every session while one is heard, then when any of the three changes. Since 2026-10-26 |
| `PQ` | `t` `seq` `request` `opTicks` | An arm-power request arrived (docs/adr/0023 decision 5): `request` `relax` (park, then torque off) or `hold` (torque on), `?<n>` for an unknown one; `opTicks` on the sender's clock. Heartbeats (no request) are not written |
| `PS` | `t` `state` `reason` | The arm-power state after a change, `holding`, `parking` or `relaxed`, and why: `none`, `no-rest-pose`, `path-collides`, `park-timed-out`, `board-not-answering`, `no-collision-model`, `unknown-request` or `hold-requested`. A refused relax is `holding` with its reason. From `parking` to `relaxed` or `holding`, no joint command reached the plant: those that arrived are `JC` with outcome `P` |
| `PK` | `t` `targets` | A park waypoint given to the plant (its relay output is the `RO` that follows): `motor:angleRad:speedRadPerSec;...`, the arm joints only |
| `TQ` | `t` `on` | A torque switch sent to the relay for every servo: `1` on, `0` off. Nothing confirms it; with the servo board silent it reaches no servo |
| `TK` | `t` `course` `round` `task` `phase` `purpose` `level` `score` `disturbed` | The headset's training course (docs/adr/0024), written when its label changes and at the start of every session while a round is going. `course` is the course's seed, `round` counts from 1 within it. `task` `reach`, `trace`, `shuttle` or `none`; `phase` `idle` (announced, or stopped), `active`, `suspended` (the robot not ready: nothing counts) or `done` (the final `score`); `purpose` `train` or `test` (held out: never train on it); `level` the round's speed in rad/s, or `varied`; `score` targets reached, touches, or percent of time on the trace's bead; `disturbed` `1` when the headset's network disturbance or added delay was on during the round (not a real-network measurement) |

| `HD` | `t` `lagMs` `jitterMs` `lossPercent` `addedDelayMs` `seed`, then optionally `handMinCutoffHz` `handPositionBeta` `handRotationBeta` `handDerivativeCutoffHz` | The headset's own network disturbance on the joint link (both ways; `lagMs` `NaN` for a named profile or trace) and its added delay on the robot's state reports (its view only). Written when they change, and at the start of every session while anything is on or the hand filter is reported. Any non-zero (or `NaN`) `lagMs`, `jitterMs`, `lossPercent` or `addedDelayMs` makes the session not a real-network measurement: `armpred check-session` fails it, so it is never packed for training. RobotHost also sets `conditions.network.headsetDisturbed` in `session.json` when it completes an automatic session. The four hand fields, added 2026-11-01 (docs/adr/0029 amendment 7), are the One Euro filter the FR3 headset puts on the palms before they drive the arms: its cutoff for a still hand (Hz), how fast the cutoff rises with the hand's speed (Hz per m/s) and turning (Hz per rad/s), and its speed estimate's cutoff (Hz). `handMinCutoffHz` 0 means raw hands. Without these fields, the headset did not report a hand filter (the JetRover's, or an older build). They are not a disturbance. Absent in sessions recorded before 2026-10-27 or with an older headset |

Readers ignore the tags they do not use; `OM`, `PQ`, `PS`, `PK`, `TQ`, `TK` and `HD` were added without a version
change, and a session with them reads and checks as before (`trainer/tests/test_contract.py`).

## Driver records (`driver.jlog`)

All times are `time.monotonic_ns()` on the Jetson.

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `RD` | `tReq` `tResp` `motor` `pulse` | One position read. `tReq` is just before the bus request, after the write-settle wait; `tResp` is when it returned. `pulse` is empty when the servo did not answer (a miss; `tResp` is then the end of the timeout) |
| `WR` | `t` `durationMs` `positions` | One board move packet, written at `t`. `durationMs` is the encoded `int(duration * 1000)`. `positions` is `motor:pulse;...` |
| `CI` | `t` `targets` | A command array reached the gate: `motor:pulse:pulsesPerSecond;...` (the pulse after `int()` and the 0..1000 clamp) |
| `GP` | `t` `motor` `pulse` | The gate kept a target pending: the servo was in its cooldown |
| `GD` | `t` `motor` `pulse` | The gate dropped a target: within the deadband of the last one written |
| `CY` | `t` `tEnd` | One feedback cycle (all reads) ran from `t` to `tEnd` |
| `BV` | `t` `millivolts` | A battery report |
| `TQ` | `t` `on` | The torque topic switched every servo (1, 2, 3, 4, 5 and 10): `1` on, `0` off, written after the board packets went out (docs/adr/0023 decision 5, since 2026-10-26). Pending targets were dropped with it |

## Operator records (`operator.jlog`)

On the operator's own clock (the header says `OPERATOR`). Written by the simulator, and by
`joint-drive` and (later) the Bridge.

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `OC` | `t` `seq` `captureTicks` `targets` | The headset sent joint command `seq` at `t`, stamped `captureTicks`, with `targets` `motor:angleRad:speedRadPerSec;...`: every command, whether or not it reached the robot. A `JC` with the same `seq` is its arrival |

## Marks (`marks.jlog`)

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `MK` | `t` `text` | A marker from `just record-mark`, on the Jetson's `CLOCK_MONOTONIC`. `text` has no `\|` |

## Latents (`latents.jlog`, simulated sessions only)

The simulator's hidden values. **Never a model input**: they stratify results and feed oracles.

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `LP` | `key` `value` | A session-wide parameter (e.g. `uplinkBaseDelayNs`, `batteryStartMv`) |
| `LJ` | `t` `motor` `speedScale` `lagSeconds` `loadNm` `stalled` | One joint's hidden state from `t` on, written when it changes |
| `EV` | `t` `kind` `detail` | An event: `slider` (detail: rad/s), `payload` (grams), `hand-on`, `hand-off`, `uplink` (profile), `board-silent`, `board-back` |

## The truth

The truth for motor `m` is its successful `RD` reads, never the frames:
- **value:** `(pulse - zeroPulse) / pulsesPerRadian - zeroOffsetRadians(m)`, with the robot profile's
  numbers (on the JetRover: `zeroPulse` 500, `pulsesPerRadian` 1000 per 240°, and the profile's
  per-joint `zeroOffsetRadians`). For the gripper (motor 10, read since 2026-10-25): its open..closed
  fraction `(pulse - gripperOpenPulse) / (gripperClosedPulse - gripperOpenPulse)`, clamped to 0..1, as
  RobotHost reports it (on the JetRover: 116 and 645);
- **time:** `(tReq + tResp) / 2`.

## Version 2: a serial-chain robot (the FR3 Duo)

Written by the serial-chain RobotHost (docs/adr/0027 decision 5), today only against the simulated arm
(`just fr3-sim-host`, `just fr3-sim-record`), and by the trainer's FR3 simulator (`armpred/sim/fr3.py`,
`source: sim`). `armpred/contract/session.py` reads it (docs/adr/0028): `QS` becomes the truth reads, at
the profile's width (16 on the FR3 Duo), and the view synthesis takes their values as they are.

**The directory** is `recordings/fr3-sim/<UTC>-<label>/` for the simulated arm (`<UTC>-auto/` for a
session RobotHost opened itself). Its files:

| File | Written by | Required |
|---|---|---|
| `session.json` | `just fr3-sim-record`, or RobotHost for an automatic session | yes |
| `robot-profile.json` | the same: a copy of the serial-chain profile RobotHost ran | yes |
| `robothost.jlog` | RobotHost | yes |
| `operator.jlog` | `chain-drive` (`just fr3-sim-record`); later the Bridge | no |
| `files.sha256` | the same as `session.json` | yes, for a packed session |

There is no `driver.jlog`: there is no driver. The arm's state comes from the backend into RobotHost,
which records it (`QS` below). A future real-time arm server hands RobotHost the same state over its
socket, so the session layout does not change when the robot arrives.

**`session.json`** is version 1's, except:
- `"format": "teleop-session"` and `"version": 2`;
- `source` is `robot`, `sim-host` (the simulated arm, `Fr3SimBackend`), `isaac` (the FR3 Duo in
  Isaac Sim behind RobotHost, docs/adr/0031), `arm-server-sim` (the real arm server's simulate mode,
  docs/adr/0030) or `sim` (the trainer's simulator). In a `sim-host` or `isaac` session the operator's
  commands are real and the arm is not, so it is never reported as the robot's. Readers treat any
  source but `robot` as not the robot;
- `robotProfile` names the profile (`fr3-duo`), and `robotProfileSha256` hashes `robot-profile.json`;
- `builds` has no `driver`.

**Lines** are version 1's (header, trailer, fields, numbers). The header's `clock` is the host's
monotonic clock (`Stopwatch`; `ticksPerSecond` 10000000 on Windows, 1000000000 on Linux).

**`robothost.jlog`** keeps every robot-agnostic record of version 1 with the same fields: `KA`, `JC`,
`FR`, `SS`, `JS`, `UP`, `OM`, `TK`, `HD`, and `PQ`, which is recorded and refused, because a serial-chain
robot has no arm power yet. `JC`'s `outcome` is `A` or `S`. Its targets are `motor:value:speed`:
radians for an arm joint, the 0..1 opening fraction for a hand. The speed is ignored, because the
shaper's limits set the speed. The relay records (`RO`, `RI`, `TQ`, `PK`, `PS`) do not occur. Added:

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `AO` | `t` `targets` | The targets an accepted command sent the arm backend, after the soft-limit clamp: `motor:value;...` |
| `QS` | `t` `sensed` `joints` | The arm's measured state: `motor:value;...` for every motor (radians; a hand's 0..1 fraction), measured at `sensed`, written at `t`. Every 4 ms of the backend's own 1 ms steps, so `sensed` is evenly spaced even when `t` is not. From the simulated arm it is the tracked position without the reading noise |

**The truth** for motor `m` is its `QS` values at their `sensed` times, never the frames. No unit
conversion and no zero offsets: the values are the profile's joint coordinates.

### The world beside the arms (docs/adr/0031 amendment 2)

When RobotHost runs with `--scene` (`just isaac-host`, `just fr3-sim-host`, both by default with
`core/Scenes/fr3-duo-bench.json`), there are objects beside the arms: a work table and obstacles
from the scene file, placed by RobotHost from seeded layouts. In Isaac they collide with the arms, and a
movable one (scene file version 2, ADR 0031 amendment 4) is a dynamic body an arm can push or knock over,
so between placements `WO` follows it wherever physics takes it; beside the simulated arm (`sim-host`)
nothing collides and nothing moves. Simulated sessions only: the real robot has no world.
Such a session has in addition:

- **`scene.json`**: a byte-for-byte copy of the scene file RobotHost ran, hashed in `files.sha256`.
- **`session.json`'s `scene`**: `{"name", "file": "scene.json", "sha256", "seed"}`. The sha256 is the
  file's, and the seed is the one layout 1 was drawn from (layout *n* uses `seed + n − 1`).
- **`session.json`'s `armServer`** (any session behind an arm server, scene or not): `{"kind",
  "description"}`, the server's own account of itself: Isaac's version, the physics rate, drive gains and
  efforts, and the scene it built.

**`robothost.jlog`** gains these records. Poses are `x:y:z:qw:qx:qy:qz` in the robot's base frame (metres,
Z up, the profile's frame); `sensed` is on the host's clock, like `QS.sensed`.

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `WL` | `t` `layout` `seed` | A new layout was drawn: at start, on NEW LAYOUT, or on a reset. Also written once when a session opens, for the layout in force |
| `WP` | `t` `request` `cause` `id` `pose` `outcome` | A placement settled. `cause`: `layout`, `headset` or `reset`. `outcome`: `applied`, `overlaps-arm`, `overlaps-object`, `outside-region`, `unknown-object`, `not-movable`, `arm-touching`, `busy`, `invalid-pose` or `timed-out`. `pose` is where the object is now, which for anything but `applied` is where it already was |
| `WO` | `t` `sensed` `revision` `objects` | Where every object was: `id:pose;...`. Written when a session opens, when any object moves more than 0.1 mm or 0.05°, and otherwise once a second. `revision` counts the placements the backend has applied |
| `WF` | `t` `snapshot` `sensed` `parts` | A world frame sent to the headset (50 Hz): the snapshot it carried and when that was sensed |
| `WE` | `t` `sequence` `op` `id` `pose` `outcome` | An edit from the headset. `op`: `move`, `new-layout`, `reset-object` or `reset-layout`. Written `pending` when it arrives, then again with its outcome |
| `WC` | `t` `sensed` `contacts` | Contacts between the arms and objects, `arm:link:object:forceN;...` (object 0 is the table); empty when they end. Only when the arm server reports them; Isaac does not yet |

**Objects are not the truth for the arm.** An arm stopped by an obstacle shows up as `QS` falling short
of `AO`, never in the joint records themselves.

### Isaac's rendered frames (docs/adr/0032 decisions 5 and 6)

Behind `just isaac-host`, Isaac's renderer follows the same control file and writes the session's frames. RobotHost waits
for its trailer (`--auto-record-writer isaac:60`), checks its `FS` lines, and lists every file under `isaac/` in
`files.sha256`, so `sha256sum -c` covers the frames. A session from it has, in addition:

| File | What |
|---|---|
| `isaac.jlog` | the renderer's log: a `JSES` header (writer `isaac`, CLOCK_MONOTONIC nanoseconds, RobotHost's clock), the records below, an `FS` line per file under `isaac/`, an `END` trailer (records, drops, sha256 of every line before it). A second renderer in the same session continues in `isaac.2.jlog`, with `.2` sidecars |
| `isaac/<camera>.<stream>.frames` | the frames: `TFRM`, uint16 version 1, uint16 0, then records, each a uint32 little-endian length and that many bytes |
| `isaac/backdrops.json` | every backdrop image the session showed: sha256, Open Images id, licence (CC BY 2.0), author, title, pages |

The streams are the scene file's `cameras[].record`. For `fr3-duo-bench`, they are `head.rgb`, `head.depth`, `head.mask`
and `third.rgb`:
- **`rgb`** is a JPEG at the camera's `recordQuality` (92).
- **`depth`** is a 16-bit PNG of the distance to the image plane in millimetres, with 0 where nothing was hit.
- **`mask`** is a 16-bit PNG of stable ids (`ID`).

**Depth and masks are privileged simulator labels:** never a model input that a real robot could supply.

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `IV` | `isaacVersion` `fps` `recordEvery` `renderLag` `lagAgreement` `lagSamples` | The renderer: its rate, every how many frames it records, and its render lag in frames as measured at start-up (it records only when the lag agreed in ≥ 99.5% of samples) |
| `SC` | `scene` `sha256` | The scene file it built (the same file as `scene.json`) |
| `CM` | `camera` `width` `height` `fx` `fy` `cx` `cy` `near` `far` `x` `y` `z` `qw` `qx` `qy` `qz` `mount` `streams` | One per camera. A pinhole with no distortion, pixel centres at +0.5. The pose is the USD camera's in the robot's base frame: it looks along its −z with +y up, so a point projects to `u = cx + fx·x_c/(−z_c)`, `v = cy − fy·y_c/(−z_c)`. `mount` is the URDF link it is fixed to, or empty for a camera fixed in the room |
| `ID` | `id:label;...` | The masks' stable ids: 1 `left_arm`, 2 `left_hand`, 3 `right_arm`, 4 `right_hand`, 5 `head`, 6 `mount`, 10 `table`, 11 `bench`, 12 `floor`, 13 `mat`, 14 `screen`, and `100 + n` for scene object `n`. 0 is anything unlabelled |
| `BK` | `t` `change` `seed` `matSha256` `matImageId` `screenSha256` `screenImageId` | The backdrop shown from now on: change 0 at the renderer's start, then one per session it closes; the pair is `backdrops.pick(images, seed, change)` |
| `OL` | `sensed` `revision` `objects` | Where the objects are, `id:x:y:z:qw:qx:qy:qz;...`, at the start and whenever the world's revision changes |
| `VF` | `t` `frame` `stateSensed` `flags` `parts` | One recorded frame. `stateSensed` is the `sensed` of the `QS` state the frame shows, so frames join the arm's truth exactly. `flags` holds `B` while a new backdrop may still be settling (the render lag after a swap). `parts` lists `stream:offset:length;...` into the sidecars, the offset being the payload's first byte |
| `FD` | `t` `frame` `reason` | A frame not recorded (`writer-behind`, `encoder-behind`, `<camera>-no-image`) |
| `RH` | `t` `fps` `stateAgeMs` `headSkipped` `thirdSkipped` `recordSkipped` | Renderer health, every 5 s |
| `ER` | `t` `reason` | Why the session's frames ended early (a nearly full disk: below 20 GB free, an I/O error) |
| `FS` | `file` `bytes` `sha256` | One per file under `isaac/`, written before the trailer |

**A frame whose `stateSensed` has no `QS`** comes from the session's last tenth of a second or so. That is after RobotHost
closed the session and before the renderer noticed; skip it. `just isaac-record-check` measures how many there are (0.4%
of the frames of a 73 s session on 2026-11-03, with the control file read every 0.5 s; it is read every 0.1 s since).

## Versioning

Readers reject an unknown version. A change adds a version, and the reader keeps reading every
version a recording still uses.
