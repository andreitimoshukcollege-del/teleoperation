# Link trace, version 1 (docs/adr/0035 N1)

The real network as the headset saw it: the raw stamps of every joint-state frame and keepalive echo the JetRover link
received, recorded by the headset (`unity/Teleop.Bridge/Runtime/LinkTraceRecorder.cs`, records by Core's
`Teleop.RobotArm.Wire.LinkTraceCodec`) to `persistentDataPath/linktraces/<UTC>.ltrace`, one file per app run.
`just pull-galaxy-traces` (the APK) and `just pull-editor-traces` (Direct Preview) bring them to the PC;
`armpred/net/linktrace.py` reads and summarises them. `golden/linktrace-v1.json` pins the reader to the writer.

## File

1. **One header line**, JSON, then `\n`:

   | Field | Meaning |
   |---|---|
   | `format`, `version` | `"teleop-link-trace"`, `1` |
   | `headsetTps` | the headset's clock rate: every headset time below is in these ticks |
   | `remoteHost` | the address the headset talked to (the robot, Isaac's GPU machine, or the PC running `just isaac-relay`) |
   | `startedUtc` | when the run began |
   | `app`, `device`, `editor` | which build, which device; `editor` true for Play mode (Direct Preview: the network is the PC's, not the headset's) |
   | `networkForecast` | the forecast level the learned model was told (docs/adr/0035 N3: `persistence`, `rtt-half` or `absent`); missing in traces from before it |
   | `recordBytes` | `48` |

2. **Records**, 48 bytes each, little-endian: `u8 kind | u8[3] 0 | u32 sequence | i64 a | i64 b | i64 c | i64 d | i64 e`.
   A torn last record (the app stopped mid-write) is dropped by readers.

| kind | What | `sequence` | a | b | c | d | e |
|---|---|---|---|---|---|---|---|
| 1 | a joint-state frame arrived | the frame's | robot send (robot ticks) | robot tps | arrival (headset ticks) | the send on the headset's clock by `ClockSync`, or `-2^63` before it synced | the reading's sensed time (robot ticks), 0 none |
| 2 | a keepalive echo arrived (what `ClockSync` takes) | 0 | the headset's keepalive send | the robot's receipt of it (robot ticks) | the robot's send (robot ticks) | robot tps | arrival (headset ticks) |
| 3 | the headset's own disturbance changed (the console's sliders, ADDED DELAY) | 0 | lag (µs) | jitter (µs) | loss (ppm) | added delay (µs) | when (headset ticks) |

## Rules

- **Frames while any kind-3 value is non-zero are not the real network**: the headset delayed or dropped them itself.
  Readers leave them out of anything called a network measurement.
- **Times are raw.** Nothing is corrected: a reader redoes `ClockSync` from the kind-2 records, or uses kind 1's `d`.
  A one-way delay from `d` carries `ClockSync`'s offset error; jitter computed from `a` and `c` (RFC 3550, docs/metrics.md
  section 3) does not.
- **Downlink only.** What happened to the headset's commands on the way up is in RobotHost's session (`JC` records:
  arrival and the headset's send stamp).
- **A recording in Play mode (`editor: true`) measures the PC's path**, not the headset's Wi-Fi.
