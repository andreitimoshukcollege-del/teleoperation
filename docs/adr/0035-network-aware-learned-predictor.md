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
| N0 | This ADR accepted; G.1050 / Y.1541 numbers copied in with their clauses (amendment 2) | the standards' text (both free from the ITU) |
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

## Amendment 2 (2026-11-09): N0, the standards' numbers, and the tiers are G.1050's profiles

Both Recommendations are free from the ITU ("Freely available items"); the numbers below were copied from their text,
fetched 2026-11-09, and are in `trainer/armpred/net/datasets/standards.py`, which a test holds to them.

**ITU-T Y.1541 (12/2011), clause 5.3, Table 1** "IP network QoS class definitions and network performance objectives":

| Class | Mean IPTD (upper bound) | IPDV: 1 − 10⁻³ quantile of IPTD minus its minimum | IPLR |
|---|---|---|---|
| 0 | 100 ms | 50 ms | 1 × 10⁻³ |
| 1 | 400 ms | 50 ms | 1 × 10⁻³ |
| 2 | 100 ms | U | 1 × 10⁻³ |
| 3 | 400 ms | U | 1 × 10⁻³ |
| 4 | 1 s | U | 1 × 10⁻³ |
| 5 | U | U | U |

"U" is unspecified. The general notes: "An evaluation interval of 1 minute is suggested for IPTD, IPDV, and IPLR ...
Any minute observed should meet these objectives."

**ITU-T G.1050 (07/2016), clause 7.1, Table 6** "Impairment levels per service test profile" (ranges, min to max):

| | Profile A, well-managed | Profile B, partially-managed | Profile C, unmanaged |
|---|---|---|---|
| One-way latency | 20–100 ms regional, 90–300 ms intercontinental | 20–100 ms regional, 90–400 ms intercontinental | 20–500 ms |
| Jitter (peak-to-peak) | 0–50 ms | 0–150 ms | 0–500 ms |
| Sequential packet loss | random loss only | 40–200 ms | 40–10 000 ms |
| Rate of sequential loss | random loss only | < 10⁻³ /s | < 10⁻¹ /s |
| Random packet loss | 0–0.05 % | 0–2 % | 0–20 % |
| Reordered packets | 0–0.001 % | 0–0.01 % | 0–0.1 % |

"The total packet loss shown is the sum of the sequential packet loss and random packet loss." Clause 7.2 labels the
standard test cases `<access><profile><severity>`: D (DSL) or G (GPON), w / p / u, a higher digit a harder case.

**The tiers are G.1050's three profiles**: good = A, poor = B, very bad = C (and anything past C). This refines decision 2,
whose table assigned tiers by source: a source spans tiers (the FCC's panel has DSL and fibre; RIPE Atlas has data
centres and LTE), so each trace is assessed on its own (`tiering.py`):

- per minute (Y.1541's interval; a trace under a minute is one window): the mean one-way delay, and the jitter as Y.1540's
  2-point IPDV (the 1 − 10⁻³ quantile minus the minimum). G.1050 says "peak-to-peak" without saying over which packets;
  the quantile keeps one late datagram in thousands from deciding the tier;
- over the trace: a run of lost datagrams lasting at least 40 ms (Table 6's shortest burst) is sequential loss, a shorter
  one random loss; a datagram that arrives before the one sent before it is reordered;
- a trace's profile is the least severe one whose every bound holds in every minute. The Y.1541 class is recorded too.

Measured this way, the headset's first trace (amendment 1) is **poor**: profile B, Y.1541 class 2 (worst-minute IPDV
140 ms, mean 16 ms, loss 0.013 %, no bursts).

G.1050's attachment (Annex H, a zip of about 780 MB, also free) holds the Recommendation's own simulator output per test
case (Annex F): per-packet delay and loss for the managed flows of profiles A and B (`$FLOW.out`, clause 6.6 Table 3),
and per-bin delay statistics of the unmanaged iPerf flow (`bw-*.csv`, Table 4). These are the standard's tiers as traces,
and N2 uses them.

## Amendment 3 (2026-11-09): N2, the datasets

`trainer/armpred/net/datasets/`, run on the GPU machine by `just network-datasets-fetch` (detached; this commit, from its
own clone `~/teleoperation-datasets`, with the trainer's venv) and `just network-datasets-status`; our own traces locally by
`just network-traces-ours`.

- **`.trace` version 2** (`datasets/trace.py`): `TRACE|2|<tps>|<sendIntervalTicks>`, then a delay in ticks or `L` (lost)
  per datagram, datagram i sent at i × the interval. Only the trainer reads it; `Teleop.Eval`, the Bridge, the trainer's v1
  reader and `analysis/playout_bounds.py` refuse it by its header (the last now checks the version).
- **Per dataset**, under `~/datasets/network/<name>/`: `download/` (untrusted: HTTPS from the dataset's own hosts only,
  size-capped, under names chosen here, parsed and never run), `manifest.json` (URL, sha256, bytes, when, licence, terms,
  citation, what was kept), `traces/*.trace` and `index.json` (per trace: group, sha256, tier and the measured tier,
  the assessment, what the conversion assumed and its seed). `trainer/contract/NETWORK_DATASETS.md` has the formats.
- **The split** (`split.json`): a group (a G.1050 test case, a Mahimahi link, a Puffer session, an FCC unit, a RIPE probe,
  one of our recordings) is held out when hash(salt, dataset, group) < 0.2; a (dataset, tier) with two or more groups and
  none held out holds out its lowest-hash one. Adding data never moves a group.

| Dataset | What | Becomes | Licence / terms (as recorded in its manifest) |
|---|---|---|---|
| `g1050-2016` | the attachment zip, read in place (Annex H describes an ISO image; the zip as published holds the folders `Software/{D,G}/out.{bidi,dwn,up}/<case>-t/`) | A and B: VoIP flows (20 ms) per packet as they are; C: iPerf bins as a normal per datagram from the bin's mean and √(m2 − mean²) | ITU, freely available; its `COPYRIGHT.txt`: the simulator output is "free from copyright restriction"; not redistributed |
| `mahimahi` | 8 cellular links × down/up (Saturator, NSDI 2013) | the joint stream (284-byte datagrams every 20 ms) through a FIFO in front of the link's delivery opportunities; a datagram not delivered in 3 s is lost (the FCC MBA's UDP timeout), where an unbounded queue made a driving trace's dead zones into delays of a minute | GPL-3.0 (the repository's); none stated for the traces |
| `puffer` | one day's `video_sent` (2026-11-07) | per stream ≥ 120 s: half the smoothed `tcpi_rtt`, interpolated between ~2 s chunks; no packet-scale jitter | **none stated**; Puffer asks for one download, which is all the fetch does |
| `fcc-mba` | July 2023 raw archive (the last), unit profile | per unit, idle hours (60 s) and loaded speed tests (10 s): summary traces | unverified (fcc.gov answered 403); US federal works are generally public domain. **data.fcc.gov refuses automated downloads (HTTP 403), and that is respected**: left out of a plain fetch; the two files can be put in `download/` by hand and `fetch fcc-mba` records them |
| `ripe-atlas` | measurement 1001 (k-root), probes by tag (data centre, home fibre/cable/DSL/Wi-Fi, LTE, 5G, Starlink, satellite), 2026-11-01 + 2 days | per probe and 4-hour block: a 60 s summary trace | RIPE Atlas terms 4.5: research allowed, commercial needs permission; raw data not republished |
| `ours-wifi` | the headset's `.ltrace` files | per piece: the one-way delay by `ClockSync`, losses by sequence, the arrival order the headset saw kept | ours |

- **Summary traces** (`synth.py`; FCC, RIPE): one-way delay half the round trip; a shifted gamma above half the minimum
  with half the mean and half the standard deviation, drawn per datagram independently (a summary says nothing of
  correlation in time); Bernoulli loss at the measured rate; delivered first in, first out, so draws never invent
  reordering. Seeded (`numpy` PCG64), the seed in the index. These are statistical stand-ins at the right level and
  spread, not recordings, and the index says which kind (`per-packet`, `replayed`, `level`, `summary`, `statistical`,
  `recorded`) every trace is.
- **A trace's tier is what it measures, for G.1050 too**; the test case's profile is kept as `info.caseTier`. On the
  attachment, 65 of 224 traces measured better than their case: profile B's VoIP flows are managed (QoS-prioritised;
  worst-minute IPDV about 1 ms), and clause 7.2's cases run "mild to severe within each service test profile", so the
  mildest unmanaged cases (Du1, Gu1) measure good. A profile describes a network, not each flow in it, and the headset's
  stream has no priority.
- **The split holds a group out across tiers** (a G.1050 case whose flows measure differently is on one side in both).

**The first fetch** (GPU machine, 2026-11-09, commit `5797365`; 2.6 GB, 375 GB left):

| Dataset | Traces | good | poor | very bad | held out: traces / groups (groups counted per tier) |
|---|---|---|---|---|---|
| `g1050-2016` | 224 | 155 | 26 | 43 | 20 / 5 of 42 cases |
| `mahimahi` | 16 | 0 | 0 | 16 | 6 / 3 of 8 links |
| `puffer` | 400 | 224 | 122 | 54 | 78 / 71 of 342 sessions |
| `ripe-atlas` | 1276 | 1003 | 120 | 153 | 312 / 45 of 108 probes |
| `fcc-mba` | — | | | | refused (HTTP 403) |
| `ours-wifi` | 1 (local) | 0 | 1 | 0 | — |

- No group straddles the split (checked on the GPU machine: 0 of 500).
- Mahimahi, fetched and converted on the PC too: every driving trace loses 1.8–8.6 % in bursts; the short ones queue to
  1.4–1.7 s and lose nothing.
- **What N5 must weigh**: RIPE Atlas is three quarters of the good tier and its traces are summaries (no correlation in
  time); Puffer's are slow levels (no packet-scale jitter); only G.1050's per-packet flows, Mahimahi and ours are packet
  timing. N5 samples by tier and by kind, not by count.

## Amendment 4 (2026-11-09): N3, the headset tells the model its link

Decision 1, built. The deployed model (`finetune-pilot2-early7`) was pre-trained and fine-tuned on decision 12's mixture,
so 15 % of its views carried a `persistence` forecast and 10 % `rtt-half`; until now the headset always sent `absent`.

- **Core.** `Types/NetworkForecast` (format v1); `Contracts/INetworkForecaster` with `Forecast/PersistenceForecaster`
  and `RttHalfForecaster` (`Registries.NetworkForecasters`, enforced by `audit`); `Forecast/NetworkForecastBlock` (the
  network block, for layouts v3 and v4) and `ForecastEffectiveTimes`; `ClockSync.RoundTripTicks` (the round trip
  `AddRoundTrip` computes, for every round trip offered, accepted or not). `Contracts/INetworkForecastTaker`: `learned`
  takes the forecast; both learned windows carry it and its age in their network block, zeros without one, as before.
- **The trainer's rules, kept.** `JointPredictorBank` gives a forecast taker each command at send + the forecast's uplink
  median at h = 0 (FIFO-clamped), and every other preset send + RTT/2, as `evaluate.py` gives a predictor that uses the
  forecast and one that does not. A round trip's downlink delay and a frame's send estimate are taken after ClockSync has
  seen that frame's round trip, as `views.py` builds them.
- **Pinned.** `armpred golden-forecast` (`just gen-forecast-golden`) writes `core/testdata/learned/forecast-golden-v1.json`:
  30 s of made-up observations (frames before sync, a 450 ms stall released as a burst, losses, an overtaken frame, a
  negative and a stalled round trip), every forecast of both levels at 100 ms, the encoded block of every seventh, and
  the effective times of 1000 commands. `NetworkForecasterTests` replays it.
- **The headset.** `JetRoverJointStateLink` feeds the forecaster each frame and round trip, issues every 100 ms, and passes
  each forecast to the bank; `JetRoverArmConfig.NetworkForecast` (`persistence` by default, `rtt-half`, or `absent` for
  the old behaviour). The link trace's header records the level (`networkForecast`).
- **Not done here.** The FR3's link (`SerialChainJointLink`) does not forecast yet. No `forecast.jlog` is recorded.
  Whether `persistence` beats `absent` for this model on the real robot is a measurement for N6, not a claim: the model
  saw it in only 15 % of training, and N5 retrains on the new mix.
- **First replay** (`just headset-replay`, `Teleop.Eval headset-replay`): the 2026-11-09 Isaac session
  `20261109T221225Z-auto` (55 s over home Wi-Fi through the relay) with its link trace, through the headset's presets.
  Arm joints, degrees, error p50 / p95, correction p95, display jitter p95: `learned` with `persistence` 0.25 / 7.27,
  1.14, 1.88; without a forecast 0.27 / 7.37, 1.16, 1.84; `hold-last` 0.24 / 1.68, 1.68, 1.44; `servo 10-25`
  0.24 / 4.20, 1.97, 2.21. The forecast changes almost nothing here: its uplink median (14.6 ms) was RTT/2's (14.4 ms).
  What dominates is that Isaac's arm did not follow the commands: against the command sent 150 ms before each reading,
  30–90 % of readings were more than 5° off on the worst joint (joint 1 commanded to 40.6° but never past 25.2°; joint 4
  reaching −80° where no command went below −38.7°), so every predictor that replays commands loses to `hold-last`.
  ADR 0034 amendment 12 found why: not the sim (it follows the real arm's commands as the real arm did), but the driving,
  twice the motion and the reversals of any real session (the wrist's −80° was the park at the end). N6 judges the forecast
  on sessions driven as the real arm is.
