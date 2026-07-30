# 2. `LatencyTrace`: a per-command record of the timestamp table

## Status

Accepted.

## Context

`docs/metrics.md` §1 defines the timestamp table every latency figure is built from
(`t_capture`, `t_send`, `t_recv`, `t_playout`, `t_render`, `t_photon`), and §2 defines OWD,
M2P, and C2A as differences between those stamps. Nothing in Core currently has a place to
hold them. `Stamped<T>` carries exactly one `CaptureTicks` alongside one value — it is the
generic "state at a time" wrapper used on the hot path for poses, velocities, and other
per-frame samples, and every consumer of it needs that shape to be minimal and allocation-free.
`LatencyTrace` is a different thing: a telemetry record for one round trip, built up
incrementally as that round trip crosses the operator/robot boundary twice, with most of its
fields unknown at the point it is created. Loading that onto `Stamped<T>` would force every
per-frame consumer to carry seven fields of telemetry it never reads, and would still not solve
the incremental-construction problem, since `Stamped<T>` is constructed once with everything
known. This ADR defines `LatencyTrace` as its own type instead.

The type is being added now, ahead of `Pipeline/` and `Recording/RecordFormat`, because
`Recording/` must eventually serialize these stamps into the `.tlog` format, and nailing the
schema down before a serializer exists is cheaper than migrating a recorded format after the
fact. It is deliberately not wired into anything: no producer sets these fields yet, no
`IMetricSink` reads them yet. That wiring is `Pipeline/`'s job once it exists.

## Decision

### Correlate by `CommandFrame.Sequence`, not `CaptureTicks`

A round trip needs one stable key that identifies "this specific command" across both
directions of the wire, survives duplication and reordering, and does not depend on which
sender's clock produced it. `CaptureTicks` fails all three: it is a timestamp, not an
identifier, it lives in the sender's own clock domain, and a duplicate or retransmitted frame
carries the same `CaptureTicks` as the original with no way to tell them apart. `Sequence` is
already the field the project uses for exactly this kind of accounting — `CommandFrame` uses
it for loss/reordering and as the delta-codec key — so `LatencyTrace` reuses it rather than
inventing a second identifier.

**Downlink direction.** The robot's state update, for the uplink command that produced it,
echoes that command's `Sequence` (the same pattern `CommandFrame.AckSequence` already uses to
carry the peer's last-seen sequence). This gives the operator side an unambiguous key to close
the loop: the downlink message carrying `t_recv`/`t_playout`/`t_render`/`t_photon` for a given
command is matched against the `LatencyTrace` opened for that command's `Sequence`, not against
any timestamp. Without this, the two halves of a round trip have no way to find each other once
more than one command is in flight, which is always, given any nonzero network delay.

### Canonical domain: the operator's monotonic clock

Motion-to-photon is defined at the operator's display (docs/metrics.md §2), so every field on
`LatencyTrace` is in ticks on the **operator's** `ITimeAuthority` timebase. Stamps produced on
the robot (`t_recv` of the uplink command, `t_send` of the resulting state) arrive in the
robot's own timebase and must be offset-corrected into operator time before they are stored —
that conversion happens once, at the point `ClockSync` has the sample, and never again. A
`LatencyTrace` with mixed domains would make every downstream subtraction wrong in a way that
looks exactly like network jitter or prediction error, which is the failure mode
docs/metrics.md's `t_recv` warning is already guarding against for the frame-vs-network-thread
case; this is the same class of bug for clock domain instead of thread.

`ClockOffsetTicks` and `ClockOffsetUncertaintyTicks` are carried on the trace alongside the
converted stamps rather than discarded after conversion, because the conversion is lossy in a
way that matters for reporting: sync uncertainty puts a floor under how precisely a converted
OWD can be trusted, and a figure reported tighter than that floor is false precision. Keeping
the offset and its uncertainty on the record is what lets analysis enforce that floor instead of
silently reporting through it.

### `TryGet` accessors; no field is trusted implicitly

Most fields on a freshly opened `LatencyTrace` are unknown — the round trip hasn't happened
yet — and some are unknown for the entire lifetime of a given run. In particular, `t_render`
and the `t_photon` derived from it can only be produced by the host: Core has no compositor, no
frame loop, and (per the one law) no `UnityEngine`, so it cannot know when a frame was
submitted. A headless `Teleop.Eval` run over a `.tlog` has no host to supply that field at all,
by design. If these fields were plain `long` ticks, an unset value (`0`, or whatever sentinel)
is indistinguishable from a legitimate tick and will eventually be differenced against another
stamp by code that forgot to check. `LatencyTrace` stores a private sentinel
(`Unset = long.MinValue`) internally, but the only public way to read a field is a
`TryGetXTicks(out long ticks)` method returning whether it was ever set. There is no plain
getter. This makes "was this stamp ever produced" a compile-time-visible question at every call
site instead of a runtime footgun.

### Immutable `With*` construction

`LatencyTrace` is a `readonly struct` built by a chain of `WithXTicks(...)` calls, each
returning a new value with one more field populated, mirroring how the round trip actually
fills it in — uplink capture and send at the operator, uplink receive at the robot, downlink
send at the robot, downlink receive/playout/render/photon back at the operator. This keeps the
type allocation-free and keeps every intermediate state (e.g. "uplink stamps known, downlink not
yet") a valid, inspectable value rather than a partially-mutated object. `ForSequence(sequence)`
is the only way to obtain the all-unset starting value; there is no public parameterless
constructor use, because a `LatencyTrace` without a `Sequence` cannot be correlated to anything
and should not type-check as usable.

## Consequences

- `LatencyTrace` has no producer or consumer yet. `Pipeline/` wires it up; `IMetricSink` and
  `Recording/RecordFormat` will need to be extended to accept it once they exist.
- Domain-conversion behavior (the operator/robot offset correction itself) is untested by this
  change, because `Time/ClockSync.cs` does not exist yet. The tests added here cover the
  `With*` chain and `TryGet`-on-unset behavior only; a follow-up PR that introduces `ClockSync`
  must add tests proving robot-domain stamps convert correctly before anything writes to those
  fields for real.
- Because `t_actuation` is referenced by the C2A formula in docs/metrics.md §2 but is not one
  of the stamps defined in §1's table, it is intentionally omitted from `LatencyTrace` rather
  than invented here. Adding it requires first adding its definition to docs/metrics.md, per
  that document's own rule.
