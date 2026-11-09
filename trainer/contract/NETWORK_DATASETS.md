# Network datasets (docs/adr/0035 amendments 2 and 3)

Public network measurements and our own Wi-Fi, as delay traces the trainer replays, each with its tier by the standards.
Written by `armpred/net/datasets/` (`python -m armpred.net.datasets`), under `~/datasets/network/` on the GPU machine
(`TELEOP_NETWORK_DATASETS` moves it). Nothing here is committed except converters and test fixtures of a few lines.

## `.trace` version 2

```
TRACE|2|<ticksPerSecond>|<sendIntervalTicks>
<delay ticks>      one datagram that arrived, this long after it was sent
L                  one datagram that was lost
```

Datagram i is sent at `i * sendIntervalTicks`. Version 1 (docs/adr/0004: `TRACE|1|<tps>`, delays only) is still what
Core, `Teleop.Eval`, the Bridge and `analysis/` read; they refuse version 2 by its header. `datasets/trace.py` reads both.

## A dataset's folder: `<root>/<name>/`

- `download/`: the files as fetched. Untrusted data: fetched over HTTPS from the dataset's own hosts, size-capped, under
  names chosen by the fetcher, parsed and never run.
- `manifest.json`: `name`, `title`, `hosts`, `licence`, `termsUrl`, `citation`, `notes`, `kept`, `files` (each `path`,
  `url`, `bytes`, `sha256`, `fetched_utc`), plus the dataset's own parameters (a day, a window).
- `traces/<id>.trace` (version 2) and `index.json`:

```json
{"format": "teleop-network-dataset-index", "version": 1, "dataset": "mahimahi", "convertedUtc": "...",
 "traces": [{"id": "Verizon-LTE-short-down", "group": "Verizon-LTE-short", "file": "traces/Verizon-LTE-short-down.trace",
             "sha256": "...", "tier": "very-bad", "tierFrom": "measured", "measuredTier": "very-bad",
             "assessment": {...}, "info": {"kind": "replayed", "assumption": "...", ...}}]}
```

`tier` is `good`, `poor` or `very-bad`, what the trace measures (`tierFrom: measured`; `index.emit` can take a source's
own tier instead, `tierFrom: source`, and no dataset does today). G.1050's traces keep their test case's profile as
`info.caseTier`. `info.kind` says what the trace is: `per-packet` (a recording of every datagram), `replayed` (a
capacity trace with the joint stream run through it), `level` (a measured slow delay level), `summary` or `statistical`
(drawn from measured statistics, seeded: `info.seed`), `recorded` (ours).

## The assessment (`tiering.py`)

| Field | Meaning |
|---|---|
| `seconds`, `sent` | the trace's length, datagrams sent |
| `loss_ratio`, `random_loss_ratio` | all lost; lost in runs shorter than 40 ms (G.1050 Table 6's shortest burst) |
| `burst_count`, `longest_burst_ms`, `burst_rate_per_s` | runs of loss lasting 40 ms or more |
| `reorder_ratio` | datagrams arriving before the one sent before them, over those that arrived |
| `delay_ms` | min, p50, p95, p99, max of the one-way delay |
| `worst_minute` | the worst minute's mean delay, IPDV (Y.1540: 1 − 10⁻³ quantile minus minimum) and loss |
| `y1541_class` | the best Y.1541 (12/2011) Table 1 class every minute meets |
| `g1050_profile`, `tier` | the least severe G.1050 (07/2016) Table 6 profile every bound of which holds: A good, B poor, C (or `beyond-C`) very bad |

## The split: `<root>/split.json`

`{"format": "teleop-network-split", "version": 1, "salt", "fraction", "madeUtc", "counts": {dataset: {tier: {train, test,
groups, groupsTest}}}, "traces": {"<dataset>/<id>": "train" | "test"}}`. A whole group is on one side.
