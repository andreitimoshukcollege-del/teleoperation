# Network forecast format, version 1

What a network-prediction model hands the arm model (docs/adr/0021 decisions 11 and 12). The network
model is built separately, later, and by someone else. This file is its output contract. Until that
model exists, `armpred/net/forecast.py` writes forecasts in this format at fixed quality levels, so
the arm model can be trained and measured against forecast quality first.

The arm model shows **where the arm is now**. A forecast of *future* lag cannot move that. What a
forecast can change is how late the commands already sent reach the robot. So the format describes
packets already sent as well as packets about to be sent.

## What a forecast is

A forecast is issued at time `t` on the headset's clock, at least 10 Hz. It holds the following, for
each direction (uplink = joint commands headset → robot, downlink = joint-state frames robot →
headset):

| Field | Per | Meaning |
|---|---|---|
| `q[h][p]` | horizon `h`, quantile `p` | the one-way delay, in ms, of a datagram **sent** at `t + h`: its quantile `p` |
| `loss[h]` | horizon `h` | the probability that a datagram sent at `t + h` is lost |
| `floor` | — | the base one-way delay in ms: what the path takes with empty queues |
| `outage` | — | the probability of an outage, nothing delivered for at least 250 ms, starting within [t, t + 1 s) |
| `valid_q`, `valid_loss`, `valid_outage` | — | whether the forecaster knows that field. **An unknown field is marked invalid, never zero-filled** |

- **Horizons** `h` (ms, on send time): −1000, −500, −250, −100, 0, +100, +250, +500, +1000.
  - Negative `h` covers datagrams already sent: in flight, or just delivered.
  - Positive `h` is the look-ahead. "There will be lag in 500 ms" is a step in `q` at `h = +500`.
- **Quantiles** `p`: 0.10, 0.50, 0.90, 0.99.
- **Units:** ms; probabilities in [0, 1].
- **Clocks:** the headset's own. A downlink send time is the robot's send time as the headset's
  clock sync places it.
- **Scope:** the network only, socket to socket. The robot's own pipeline after its socket
  (RobotHost's loop, the relay's poll, the driver's cycle) is not part of it; `relay-replay-servo`
  models that pipeline separately.

### How each impairment appears

| Impairment | Where it shows |
|---|---|
| lag | the level of `q[h][0.50]` |
| jitter | the spread `q[h][0.90] − q[h][0.10]` |
| buffering / queueing (bufferbloat) | `q[h][0.50]` above `floor`, and its rise across `h` |
| loss and loss bursts | `loss[h]` across `h` (a burst is a run of high values) |
| outage (a Wi-Fi roam, say) | `outage`, and `q` rising to the outage's end for datagrams sent inside it |
| reordering | an optional field (below); otherwise jitter wider than the send interval |

### Optional fields

These are defined now so the format need not change when a network model can provide them. Arm model
layout v2 does not read them.
- per direction: `reorder` (probability), `burst` (expected loss-burst length, in datagrams),
  `bandwidth` (available bit/s);
- downlink only: `buffer_target` (the receiver's jitter-buffer target, ms).

### Why this shape

The survey behind it is `docs/literature/networking.md` §16 (2026-10-21). Every predictor there
that drives a controller reports a distribution indexed by horizon: quantiles
(NetEQ's target delay, Sprout's cautious forecast, a quantile-regression RTT forecaster) or bins
(Fugu's transmission-time predictor), because a controller acts on the risk of being late, not on
the mean. Separating a floor from a queue is the BBR/LEDBAT path model. And a 3GPP-style notice,
"this threshold is crossed in this window, with this confidence", is a special case of a quantile
grid over horizons.

## Recorded: `forecast.jlog`

The line format of `SESSION_FORMAT.md`: header `JSES|1|forecast|<tps>|OPERATOR|<sessionId>`, then
one record per direction per forecast, with the trailer as in that file.

| Tag | Fields after the tag | Meaning |
|---|---|---|
| `FC` | `t` `dir` `valid` `floorMs` `pOutage` `values` | `dir` is `up` or `down`; `valid` is three characters `q`, `l`, `o` or `-` for the three flags; `values` is `;`-separated, one item per horizon in the order above, each `p10:p50:p90:p99:pLoss` (an invalid field is empty) |
| `FX` | `t` `dir` `key` `value` | an optional field: `reorder`, `burst`, `bandwidth`, `bufferTarget` |

## In a view

A view (`VIEW_FORMAT.md`, version 2) can carry one forecast stream as the arrays `fc_t`, `fc_q_ms`
[n, 2, 9, 4], `fc_loss` [n, 2, 9], `fc_floor_ms` [n, 2], `fc_outage` [n, 2], `fc_valid_q`,
`fc_valid_loss` and `fc_valid_outage` [n, 2]. Its `meta_json` names the level (`forecastLevel`).
Direction index 0 is the uplink, 1 the downlink.

## What the host does with it

- **The forecast reaches the arm model as a call of its own**, issued when it arrives. The model's
  window carries the newest one and its age (`MODEL_FORMAT.md`, layout v2).
- **A command's effective time** becomes its send time plus `q[0][0.50]` of the uplink, from the
  newest forecast at the send, instead of the clock sync's RTT/2. It never falls before the send, and
  never before the previous command's: a FIFO path cannot deliver a later datagram first, and the
  predictor would reject it.
- **With no forecast, or with `valid_q` false,** effective times stay RTT/2, as today.

## Quality levels (training and evaluation only)

`armpred/net/forecast.py` makes forecasts from a view's true network schedule. ADR 0021 decision 12
fixes the level values:

| Level | Knows a change after | Announces in advance | Lead | Error on the delay above the floor | False alarms | Spread × |
|---|---|---|---|---|---|---|
| `oracle` | 0 | every change | the whole look-ahead | 0 | none | 1 |
| `good` | 100 ms | 70 % | U(0.1, 1.0) s | lognormal σ 0.15 | 1 per 300 s | U(0.8, 1.25) |
| `fair` | 300 ms | 40 % | U(0.1, 1.0) s | σ 0.30 | 1 per 60 s | U(0.6, 1.6) |
| `poor` | 1000 ms | 10 % | U(0.1, 1.0) s | σ 0.50 | 1 per 30 s | U(0.5, 2.0) |

Three more use only what the headset observes today:
- `persistence`: downlink delays from frame stamps; uplink delays from keepalive round trips minus
  the downlink estimate; loss from frame-sequence gaps; no look-ahead;
- `rtt-half`: the clock sync's RTT/2 in both directions, with no spread, loss or outage;
- `absent`: no forecast.

**The oracle's quantiles are those of the true delay distribution at each send time,** not the
realised delay of each datagram: a perfect forecaster of network conditions, not of individual
packets.

## Versioning

Readers reject an unknown version. A change bumps the version, changes this file and both sides in
the same commit, and keeps the old version readable while anything still uses it.
