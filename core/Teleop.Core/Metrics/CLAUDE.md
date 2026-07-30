# Metrics

`IMetricSink` implementations that can live in Core: a null sink and an in-memory tracker.
Definitions for every metric name ever passed to `Record` live in `docs/metrics.md`, not here.

## Implemented

| Name | File | Notes |
|---|---|---|
| — | `NullMetricSink.cs` | discards everything; inject where a sink is required but unused |
| — | `InMemoryMetricTracker.cs` | fixed-capacity ring buffer; test/inspection aid, not a full-run historian |

**`CsvMetricSink` deliberately does not live here.** Writing a `metrics.csv` is I/O, and I/O is
not Core's — per `Contracts/IMetricSink.cs`'s own doc comment, that implementation lives in the
hosts (`core/Teleop.Eval/Metrics/CsvMetricSink.cs`).

## Requirements

1. Allocation-free `Record`. It is called several times per frame by hot-path components
   (a reconciler emitting correction cost every step, for example).
2. Never reorder or drop a sample silently. `InMemoryMetricTracker`'s bounded-capacity overwrite
   is a documented, deliberate policy — it is not "dropping," since the caller knows the
   capacity it configured.
3. `Reset()` fully restores as-constructed state, with a test proving it.
