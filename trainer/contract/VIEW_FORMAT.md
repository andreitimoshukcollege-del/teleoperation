# View format, version 1

A *view* is one session replayed through one network schedule and one seed: exactly the calls the
headset's predictor would have received, plus the truth. The trainer writes views
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
| `tr_value` | float64 | [r] | Its angle (rad), by the session format's truth conversion |
| `frame_send`, `frame_delivered`, `frame_arrival`, `frame_segment` | | [f] | What the downlink did to every frame sent |
| `sync_t`, `sync_offset`, `sync_uncertainty`, `sync_accepted` | | [s] | Every round trip offered to `ClockSync`, and its state after |
| `events_t`, `events_kind`, `events_detail` | int64, U32, U64 | [e] | What metrics may align on: `network` switches, `uplink` changes, and the session's own events |

Calls are in call order. When a command and a state share a tick, the command came first.

## `meta_json`

```json
{
  "format": "jetrover-view", "version": 1,
  "session": "<session id>", "source": "robot" | "sim",
  "schedule": "lan@0;150ms-20j-0.5loss@60", "namedOnly": true,
  "seed": 3,
  "operatorTicksPerSecond": 1000000000, "robotTicksPerSecond": 1000000000,
  "operatorUptimeNs": 123456789, "operatorDriftPpm": -12.3,
  "conditions": { "...": "copied from session.json" }
}
```

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
  keepalive, whose stamps the frames echo) earlier by the schedule's uplink shift.
- **Downlink.** Each frame goes through the link of the segment active when it was sent, one link
  per segment, seeded from (seed, segment).
- **Headset.** As `JetRoverJointStateLink`: ticks at about 90 Hz; ClockSync on echo changes with the
  true arrival; frames decoded before sync never reach the predictor; the hold-back; effective time
  = send + RTT/2.
