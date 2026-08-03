# 3. `DisplayOffset`: storage, computation, and the physical calibration procedure

## Status

Accepted.

## Context

`docs/metrics.md` §1 defines `t_photon = t_render + DisplayOffset; estimated light emission`,
and Gate 4 (`docs/setup.md`) requires "`DisplayOffset` calibrated for this headset and refresh
rate," validated against a physical rig. Before this ADR, `DisplayOffset` existed only as a
phrase in three doc comments (`Types/LatencyTrace.cs`, this project's ADR 0002, `docs/metrics.md`)
and one sentence in `unity/TeleopVR/Assets/Teleop/CLAUDE.md` ("comes from OpenXR
`predictedDisplayTime` where available, otherwise a per-headset calibrated constant measured
with the photodiode rig") — no field, no config schema, no storage format, no procedure. This
ADR fills that gap as part of closing Gate 4's Bridge-layer build.

## Decision

### Storage and loading are host-side; no new Core type

`t_render` can only be produced by the host (ADR 0002: "Core has no compositor and no frame
loop"). `t_photon = t_render + DisplayOffset` is exactly as host-only, so `DisplayOffset` never
enters Core: `Bridge/TeleopOperatorBridge.cs` computes `t_render + DisplayOffset` directly and
calls `LatencyTrace.WithPhotonTicks` with the result. Adding a Core-side type for a single host
constant would be ceremony implying Core has a use for it, which it does not.

- `Bridge/DisplayCalibrationConfig.cs` — a plain `[Serializable]` class (`DisplayOffsetMilliseconds`,
  `HeadsetModel`, `RefreshRateHz`), JSON-round-trippable via `UnityEngine.JsonUtility` rather than
  a `ScriptableObject`, so calibrating on-device never requires re-authoring a binary/YAML asset.
- `Bridge/ConfigLoader.cs` — loads it the way Teleop/CLAUDE.md's Quest constraints require: an
  override at `Application.persistentDataPath/display_calibration.json` (pushed with `adb push`,
  no rebuild) wins over a `Resources` default shipped in the build. Never throws; a missing or
  malformed file falls back to a hardcoded placeholder rather than blocking Play mode.
- `Bridge/XrDisplayTimeProvider.cs` — the single call site that turns the config into ticks.
  Only the calibrated-constant path is implemented; live OpenXR `predictedDisplayTime`
  integration is a deliberate follow-up (see the file's own doc comment for why: the exact API
  surface needs verification against the installed package version in an actual Editor, which
  the agent that wrote this did not have).

### The shipped default is a placeholder, not a measurement

`Resources/display_calibration.json` ships `DisplayOffsetMilliseconds: 20.0` — an invented
round number, not a measurement. Any M2P figure recorded before the procedure below has been run
on the real headset should not be cited; `DisplayCalibrationConfig`'s `HeadsetModel` field
defaults to `"uncalibrated"` specifically so a recorded `.tlog`'s session metadata makes this
obvious rather than silent.

## Physical measurement procedure

Per `docs/metrics.md` §2's existing validation language ("LED plus photodiode, or a high-speed
camera on a spinning marker"):

1. **LED + photodiode.** Wire an LED to trigger at the same moment `TeleopOperatorBridge`
   captures a pose (e.g. a button press read by both the LED driver and `SubmitCommand`'s input
   path). Aim a photodiode at the screen region the ghost/ground-truth robot renders into.
   Timestamp LED-on and photodiode-detected-light with an oscilloscope or logic analyzer; their
   difference is a physical M2P measurement independent of the software estimate.
2. **High-speed camera.** Film, at ≥240 fps, both a physical spinning marker and the in-headset
   display (via a mirror rig, a beam splitter, or screen capture from the compositor). Frame-count
   the delay between the marker's real position and its displayed position.
3. **Compare and calibrate.** Run several trials; take the median physical M2P. Compare it to the
   software-estimated M2P the HUD/`.tlog` reports for the same session. The difference, after
   accounting for every other stamp already in `LatencyTrace`, is `DisplayOffset`. Write the
   measured value into `DisplayOffsetMilliseconds`, either as a `persistentDataPath` override
   (fastest, no rebuild) or by updating the shipped `Resources` default and rebuilding, and record
   the real `HeadsetModel`/`RefreshRateHz` alongside it.
4. **Re-validate whenever the render path changes** (docs/metrics.md §2) — a different XR plugin
   version, a different refresh rate, or an editor upgrade can all shift the real offset without
   any change to this code.

## Consequences

- This ADR defines the mechanism, not a number. Gate 4's `DisplayOffset` checkbox does not close
  until the procedure above is actually run against the real headset — that step needs a human
  with the hardware, not an agent.
- OpenXR `predictedDisplayTime` remains unimplemented. Until it lands, `DisplayOffset` is a
  single constant per headset/refresh-rate pair, not a per-frame-varying estimate; this is a
  documented simplification, not a silently missing feature.
