# View format, version 3

A *view* is one session replayed through one network schedule and one seed: exactly the calls the
headset's predictor would have received, plus the truth. Version 2 (docs/adr/0021 decision 13) adds
what a network forecast needs: every command the headset sent (lost ones too), each command's true
uplink delay, the headset's estimate of each frame's send time, every keepalive round trip, enough in
`meta_json` to rebuild both directions' true conditions, and optionally one forecast stream.
Version 3 (2026-10-25) changes only what `tr_value` means for the gripper (below). Readers keep reading
versions 1 and 2: version 1's version-2 arrays take the defaults below, and a version-1 or -2 view's
gripper truth is dropped, since it went through the arm's angle formula. The trainer writes views
(`armpred make-views`). The repo's `joint-score` (built only after the trainer gate, ADR 0021
decision 8) replays them through every C# joint predictor, so both sides score identical inputs.

## File

An uncompressed NumPy `.npz` (`numpy.savez`): a zip of `.npy` arrays, readable from C# with
`System.IO.Compression` and the NPY header. All integers are little-endian.

| Array | dtype | Shape | Meaning |
|---|---|---|---|
| `meta_json` | uint8 | [bytes] | UTF-8 JSON, below |
| `cmd_t` | int64 | [n] | When `ObserveCommand` was called (the send), operator ticks |
| `cmd_eff` | int64 | [n] | The `effectiveTicks` passed: send + `OffsetUncertaintyTicks` once synced |
| `cmd_speed` | float32 | [n] | The `speedRadPerSec` passed: the largest of the command's target speeds |
| `cmd_count` | int16 | [n] | Targets in the command |
| `cmd_motor` | int16 | [n, 13] | Motor ids (0 past `cmd_count`) |
| `cmd_value` | float32 | [n, 13] | Target values (rad; the gripper a 0..1 fraction) |
| `st_t` | int64 | [m] | When `ObserveState` was called (the display tick that released the frame) |
| `st_sensed` | int64 | [m] | The `sensedTicks` passed: the frame's sensed (or send) time, converted by `ClockSync` |
| `st_count`, `st_motor`, `st_value`, `st_flags` | | [m], [m, 13] | The frame's joints; `st_flags` bit 0 = sensed |
| `tr_t` | int64 | [r] | Truth: each successful driver read's midpoint, on the operator's clock |
| `tr_motor` | int16 | [r] | Its motor |
| `tr_value` | float64 | [r] | Its angle (rad), by the session format's truth conversion; for the gripper (motor 10), its 0..1 open..closed fraction, clamped, as RobotHost reports it (version 3) |
| `frame_send`, `frame_delivered`, `frame_arrival`, `frame_segment` | | [f] | What the downlink did to every frame sent |
| `sync_t`, `sync_offset`, `sync_uncertainty`, `sync_accepted` | | [s] | Every round trip offered to `ClockSync`, and its state after |
| `events_t`, `events_kind`, `events_detail` | int64, U32, U64 | [e] | What metrics may align on: `network` switches, `uplink` changes (the robot side's `UP` records, and the view's slow shift as `shift:<ms>`), and the session's own events |
| `cmd_lost` | bool | [n] | v2. The command never reached the robot (default false) |
| `cmd_reconstructed` | bool | [n] | v2. Not in the recording: filled in from a gap in the command sequence numbers (default false) |
| `cmd_true_uplink` | int64 | [n] | v2. Ticks from the send to the plant; -1 if lost (default -1) |
| `frame_send_est` | int64 | [f] | v2. The frame's send time as the headset's ClockSync placed it when it arrived; -1 before sync |
| `sync_rtt`, `sync_frame`, `sync_min_rtt`, `sync_synced` | int64, int64, int64, bool | [s] | v2. Each round trip's RTT (accepted or not), the frame that echoed it, ClockSync's window minimum and whether it trusted itself after it |
| `fc_t`, `fc_q_ms`, `fc_loss`, `fc_floor_ms`, `fc_outage`, `fc_valid_q`, `fc_valid_loss`, `fc_valid_outage` | | | v2, optional. One network forecast stream (`FORECAST_FORMAT.md`) |

Calls are in call order. When a command and a state share a tick, the command came first; a forecast
issued at that tick comes after both.

## `meta_json`

```json
{
  "format": "jetrover-view", "version": 3,
  "session": "<session id>", "source": "robot" | "sim",
  "schedule": "lan@0;150ms-20j-0.5loss@60", "namedOnly": true,
  "seed": 3,
  "operatorTicksPerSecond": 1000000000, "robotTicksPerSecond": 1000000000,
  "operatorUptimeNs": 123456789, "operatorDriftPpm": -12.3,
  "conditions": { "...": "copied from session.json" },
  "t0RobotNs": 3600000000000,
  "downlinkSchedule": "<the schedule's JSON, armpred/data/schedule.py>",
  "uplink": { "schedule": "<JSON>", "originRobotNs": 3600000000000, "lanBaseMs": 2.1, "lanJitterMs": 0.9 },
  "commandsLost": 12, "commandsReconstructed": 0,
  "forecastLevel": "good"
}
```

`t0RobotNs`, `downlinkSchedule` and `uplink` (v2) rebuild the true conditions, by send time, of
the downlink this view synthesised and of the uplink the robot side physically applied
(`armpred.data.views.view_truths`). `forecastLevel` is present only when the view carries a stream.

**`namedOnly` must be true for any view used in a reported number.** Only then is every downlink
segment a profile from Core's catalog, through the exact copies the golden files pin. False marks
the trainer-only schedules (random parametric segments, ramps, hold-back), which are for training
only.

## How a view is made (armpred/data/views.py)

- **Clocks.** A fresh virtual headset clock per view: its own uptime (hours to days), drifting
  ±50 ppm. The original headset's clock is related to the robot's exactly for a simulated session,
  and from the lower envelope of command arrivals for a recorded one. The envelope cannot see the
  minimum one-way delay, so it assumes 0.5 ms; that misplaces each send by the real minimum delay
  minus 0.5 ms (a couple of milliseconds on the LAN), the same for every command.
- **Uplink.** The recorded robot-side arrivals stay; the virtual headset sent each command (and each
  keepalive, whose stamps the frames echo) earlier by the schedule's uplink shift. Every command the
  headset sent is a call: from `operator.jlog` when the session has one (exact), otherwise from the
  robot's `JC` records plus one per short gap in their sequence numbers, interpolated and marked
  `cmd_reconstructed`.
- **Downlink.** Each frame goes through the link of the segment active when it was sent, one link
  per segment, seeded from (seed, segment).
- **Headset.** As `JetRoverJointStateLink`: ticks at about 90 Hz; ClockSync on echo changes with the
  true arrival; frames decoded before sync never reach the predictor; the hold-back; effective time
  = send + RTT/2.
