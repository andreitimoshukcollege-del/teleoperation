# ADR 0035: A learned predictor that knows the network it is on

## Status

Accepted, 2026-11-08 (direction): the user, "start with N1, record the wifi traces". Each phase is one PR, settled by
amendments here; N0's standards text is still to be copied in before N2 uses it.

## Context

Driving the JetRover in Isaac (ADR 0034) from home, through `just isaac-relay`, both predictors felt jittery, the learned
one (ADR 0025, `finetune-pilot2-early7`) most. Measured that evening:

- **The uplink was fine.** RobotHost's `JC` records of four sessions: commands sent every 27.8 ms (two frames of the
  headset's 72 Hz), RFC 3550 jitter 3–9 ms, per-command |D| p99 27–30 ms. Inside the training mix.
- **The last Wi-Fi hop was not.** Round trips PC ↔ headset: min 3, median 83, p90 312, max 631 ms; PC ↔ GPU machine
  over the VPN: 10 / 13 / 16 / 54 ms. A router buffering frames for a dozing headset and releasing them in bursts.
- **The model cannot tell.** Its input has a network block (ADR 0021 decision 12: 101 values, a forecast of delay, jitter
  and loss), but the headset never fills it: at inference it is the `absent` level, all zeros. It was trained with
  `absent` in 10% of views; the rest saw a forecast.
- **The training mix rarely shows this.** `RandomScheduleConfig`: named profiles `lan`, `50ms-5j`, `150ms-20j-0.5loss`
  (20%), random parametric (40%), added delay (20%), a filling queue (8%), an outage then a burst (4%), the rest bursty
  congestion and sines. Recorded sessions only ever shift the uplink slowly (`MAX_UPLINK_SLOPE`), so the fine-tune saw no
  fast uplink at all.

The user: "the learned model should be able to compensate for all types of lag including this one", and train on "standard
data sets of networks ... for good, poor, and very bad connections".

What "compensate" can mean has a floor: through a 600 ms blackout no predictor knows where the arm is beyond what the
commands imply. The aim is a twin that stays smooth through a stall and needs a small correction after it — low
`joint_correction_deg` and `joint_display_jitter_deg` (docs/metrics.md section 4) at a given prediction error — not a
perfect pose.

## Decision

1. **The headset fills the network block.** Core gains the forecast's causal levels, `persistence` and `rtt-half`
   (`trainer/armpred/net/forecast.py`: built only from what the headset observes — frame arrivals, `ClockSync` round trips,
   sequence gaps), as a C# copy pinned to the Python by a golden file, as `LearnedJointWindow` is. `LiveLinkMeter`
   (ADR 0034 amendment 8) is its first consumer's cousin; the forecast is the model's input, the meter the operator's
   display. The model is then told, frame by frame, the link it is on.

2. **Network conditions come in three tiers, from standards and public measurements, never invented.**

   | Tier | Means | Sources |
   |---|---|---|
   | **good** | wired or a clean managed network | ITU-T Y.1541 class 0 bounds (mean ≤ 100 ms, delay variation ≤ 50 ms, loss ≤ 10⁻³); ITU-T G.1050's *well-managed* scenarios; RIPE Atlas pings between wired probes |
   | **poor** | home broadband, busy Wi-Fi, 4G | Y.1541 classes 1/3 (delay ≤ 400 ms); G.1050 *partially managed*; FCC Measuring Broadband America UDP latency and loss; Puffer's per-connection RTT (`tcpi_rtt`) |
   | **very bad** | congested cellular, power-saving Wi-Fi, outages | G.1050 *unmanaged*; Mahimahi's cellular capacity traces (Saturator, NSDI 2013) replayed through a queue at the joint stream's bitrate; our own recorded Wi-Fi traces (decision 3) |

   - **Each dataset is fetched by a recipe** into `~/datasets/network/<name>/` on the GPU machine, with a
     `manifest.json` of its source URL, licence, terms, sha256 and what was kept, as the backdrops manifest records its
     images (ADR 0032). Downloaded files are untrusted data: parsed, never run.
   - **They become traces**, Core's `.trace` format (`TRACE|1|<tps>`, one delay per line; loss as a marker), which the
     trainer already replays (`armpred.net.impairments.TraceDelay`) and `Teleop.Eval` already reads. A converter per
     source, tested on a fixture of it. RTT-only sources (RIPE Atlas, Puffer, FCC) give one-way delay as half the RTT plus
     their own spread; that assumption is written into each trace's manifest.
   - **G.1050 and Y.1541 are parametric.** Their scenario tables (G.1050 clause 7) become parametric profiles. The
     standard is bought or read at the ITU, and its numbers are copied into this ADR's amendment with the clause they
     come from, before they are used.
   - **A split by source, not only by session.** Some traces of each tier are held out of training entirely, and the
     reported numbers are on those.

3. **Our own Wi-Fi is recorded.** The headset writes a downlink trace — each joint frame's one-way delay by `ClockSync`,
   and its sequence gaps — to `persistentDataPath` during any session (`just pull-galaxy-traces`). A few evenings of home
   Wi-Fi, the lab's SINRG Wi-Fi and the relay become very-bad and poor traces, and a test set nobody can say is
   synthetic.

4. **Fast uplink comes from sessions the arm really lived through** (ADR 0021 decision 13). Isaac, now fitted to the real
   arm (ADR 0034 amendment 11), is driven from the headset with the console's network sliders set from the tiers'
   profiles, through `JointLinkDisturbance`, so commands really arrive late and lost at the arm. Those sessions
   (`source: isaac`) are the uplink half of the training set. Real-robot sessions stay the fine-tune's ground truth.

5. **The training mix is re-weighted by tier.** `RandomScheduleConfig` gains the dataset traces and the standards'
   profiles, and the share of stalls and bursts rises from 4% to a share chosen from the recorded Wi-Fi's own statistics.
   The `absent` forecast level stays in the mix, so a headset without the forecast still works.

6. **Judged on held-out traces of every tier, against today's model and `servo 10-25`**, with docs/metrics.md section 4's
   prediction error, `joint_correction_deg` and `joint_display_jitter_deg`, per tier. A model that wins on poor and very
   bad but loses on good is not accepted.

## Phases

| Phase | What | Needs |
|---|---|---|
| N0 | This ADR accepted; G.1050 / Y.1541 numbers copied in with their clauses | the standards' text |
| N1 | `just pull-galaxy-traces`: the headset records downlink traces; a week of home, lab and relay Wi-Fi | the headset |
| N2 | `just network-datasets-fetch` and converters to `.trace` for RIPE Atlas, FCC MBA, Puffer, Mahimahi; manifests, licences, fixtures; the tier split | the GPU machine's disk |
| N3 | Core: the forecast's causal levels in C#, golden-pinned to `forecast.py`; the headset fills the network block | an ADR-level Core change (this one) |
| N4 | Isaac uplink sessions: the console's sliders driven through the tier profiles while recording | Isaac, the headset |
| N5 | Retrain: sim pre-train with the new mix, fine-tune on real sessions plus the Isaac uplink sessions; evaluate per tier | the 5090 |
| N6 | Ship the winner on the headset; a session at home over the relay and one in the lab, scored | the headset |

## Consequences

- The model's input no longer depends on the headset's network being like the lab's.
- New datasets come with licences and terms (RIPE NCC's terms of service; Puffer asks for one download per user; the FCC's
  is public). Each manifest records them, and nothing from them is redistributed in the repo: only converters, fixtures
  of a few lines, and our own traces.
- Training data grows; the 5090's disk and the trainer's shard budget are checked in N2.

## Alternatives considered

- **Only re-weight the existing synthetic mix.** Cheaper, but the user asked for standard conditions, and synthetic
  schedules are exactly where the very-bad tier was thin.
- **A smoothing filter on the twin.** Hides jitter by adding lag; a research axis of its own (Reconciliation), not this.
- **Wired or Direct Preview only.** The right way to drive Isaac remotely today, and done (`just unity-robot-host
  10.188.58.133 false`), but it does not make the model robust, which is the point.

## Not decided here

- The exact shares of the new mix (N5, from the recorded traces).
- A network model that forecasts beyond the causal levels (ADR 0021's `DEGRADED` levels need knowledge the headset lacks).
- The real FR3's network.

## Amendment 1 (2026-11-08): N1, the headset records its link

- **What is recorded.** Every joint-state frame the JetRover link receives (its sequence, the robot's send stamp and tick
  rate, the arrival, the send on the headset's clock by `ClockSync`, the reading's sensed time), every keepalive echo as
  `ClockSync` takes it, and every change of the headset's own disturbance (the console's sliders, ADDED DELAY), so frames
  it delayed itself can be left out. Raw stamps only: a reader can redo `ClockSync`.
- **Format.** `trainer/contract/LINK_TRACE_FORMAT.md`: a JSON header line, then 48-byte records from Core's
  `LinkTraceCodec`; `trainer/contract/golden/linktrace-v1.json` (written by `LinkTraceCodecTests`) pins the trainer's reader,
  `armpred/net/linktrace.py`, to it.
- **On the headset.** `LinkTraceRecorder` (Bridge) writes `persistentDataPath/linktraces/<UTC>.ltrace`, one file a run,
  about 2.5 KB/s, buffered and written once a second (nothing allocates per frame); the newest 200 runs are kept.
  `JetRoverArmConfig.RecordLinkTrace` (default on) turns it off. A write failure stops the trace, never the operator.
- **Recipes.** `just pull-galaxy-traces` (the APK's files) and `just pull-editor-traces` (Play mode, whose header says
  `editor: true`: its network is the PC's, not the headset's Wi-Fi) into `recordings/linktraces/`;
  `just link-trace-summary` gives per trace the one-way delay, round trips, RFC 3550 jitter, loss by sequence, stalls over
  100/250/500 ms and the bursts after them.
- **Downlink only.** The uplink is already recorded by RobotHost (`JC`: arrival and the headset's send stamp).
- **First trace (2026-11-09, Galaxy XR on home Wi-Fi, through `just isaac-relay` to Isaac).** 162 s, 7989 frames (49 Hz),
  399 KB (2.5 KB/s, as designed). One-way delay p50/p95/p99/max 14 / 25 / 54 / 163 ms, round trips p50/p95/max
  28 / 41 / 213 ms (half the RTT agrees with the one-way delay, so the stamps and `ClockSync` agree), RFC 3550 jitter 7 ms,
  loss 0.013%, 18 stalls over 100 ms and none over 250 ms, each followed by a burst of about 7 frames. A calmer evening than
  the one in Context (RTT p50 83, p90 312 ms): one trace is not the tier. A week of them, at different times of day, is N1's
  remaining work.
