# Session format, version 1

What the robot records (docs/adr/0021 decision 3), and what the simulator writes, so that a simulated
session and a recorded one travel the same path. Written by RobotHost (C#), by the ROS driver
(Python) and by `armpred/sim`; read by `armpred/contract/session.py`; checked by
`armpred check-session` (`just recording-check`).

## A session is a directory

`recordings/jetrover/<UTC>-<label>/` (gitignored; archived to a GitHub Release):

| File | Written by | Required |
|---|---|---|
| `session.json` | `just record-start`, completed by `just record-stop` | yes |
| `robothost.jlog` | RobotHost | yes |
| `driver.jlog` | the ROS driver (`jetrover_arm_control/session_log.py`) | yes |
| `marks.jlog` | `just record-mark` | no |
| `operator.jlog` | `joint-drive` (later, the Bridge) | no |
| `latents.jlog` | the simulator only | sim only |
| `files.sha256` | `just record-stop` (`sha256sum` format) | yes, for a packed session |

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
- `conditions.speedSliderRadPerSec` is a number, or `"varied"` when it was moved during the session.
  Since 2026-10-25 `just record-stop` takes it from the speed the session's joint commands carried
  (the headset's slider rides in each command's speed field), not from what `record-start` was told;
  a difference is noted in `notes`. `just recording-set-slider` corrects a session stopped before then.
- `conditions.load.kind` is `none`, `payload` or `hand`.
- `conditions.network.uplinkProfile` is a profile name from Core's catalog. `lan` means the robot
  impaired nothing.
- `conditions.motion` is `teleop`, or `joint-drive:<script>`.
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
| `JC` | `t` `tDeliver` `seq` `captureTicks` `outcome` `targets` | A joint command arrived at the socket at `t` and reached the plant at `tDeliver` (equal unless the uplink is impaired on the robot). `captureTicks` is on the sender's clock. `outcome` is `A` accepted, `S` stale (capture not newer than the last accepted), `R` refused (the pose would collide, ADR 0020). `targets` is `motor:angleRad:speedRadPerSec;...` as decoded |
| `RO` | `t` `targets` | Targets sent to the relay: `motor:pulse:pulsesPerSecond;...` (the pulse as sent, a float) |
| `RI` | `t` `entries` | A relay feedback datagram taken by the plant: `motor:valid:pulse;...`, `valid` 0 or 1 |
| `FR` | `t` `seq` `robotSendTicks` `sensedTicks` `echoOperatorTicks` `echoRobotRecvTicks` `joints` | A joint-state frame sent, decoded from the bytes that went out. `joints` is `motor:flags:value;...` |
| `SS` | `t` `endpoint` | The joint-state stream's subscriber changed |
| `JS` | `t` `endpoint` | The joint-command sender changed, so the staleness check restarted |
| `UP` | `t` `profile` `seed` | The robot-side uplink impairment changed (`profile` `none` is off): from then on, `JC`'s `tDeliver` is when that Core profile released the command, `t` when the socket was read. Keepalives go through it too. A schedule (ADR 0021 decision 13) writes one `UP` per change; a simulated session's trainer-only segments name themselves (`param:...`, `queue:...`, `outage:...`) and its `latents.jlog` `LP uplink_schedule` holds their parameters |

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

## Versioning

Readers reject an unknown version. A change adds a version, and the reader keeps reading every
version a recording still uses.
