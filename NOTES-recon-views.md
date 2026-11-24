# Handoff: random camera viewpoints for scene reconstruction (WIP, paused 2026-11-24)

Task: an ADR 0038 amendment plus an implementation that lets the JetRover twin record takes from N extra "view" cameras at
seeded random viewpoints, with colour, metric depth, instance masks, and each camera's intrinsics and pose. No headset change.
Paused at the user's request before any code was written. **This commit holds only this note.** No process was started
on the GPU machine. The only contact was one read-only ssh (uptime, nvidia-smi and a grep of Isaac's CameraSensor source).

## Done
- Read CLAUDE.md, docs/agent-operations.md, ADR 0032, ADR 0038 (decision, A appearance, masks), SESSION_FORMAT's Isaac
  section, renderer.py (camera creation, jitter, change, preamble, loop, writer), lerobot takes.py/sidecars/visibility,
  the justfile's isaac-jetrover-host.
- Checked the readers. Every reader of `isaac.jlog` keys on `CM` camera ids and ignores the `VF` parts of other streams:
  takes.py, sidecars.py, visibility.py, appearance_check.py and verify_session.py. New tags are skipped.

## Design decided so far (to write as the ADR 0038 amendment)
1. **Opt-in, the default unchanged.** There is a new renderer flag `--views FILE` (plus `--views-count N`), and
   `isaac-jetrover-host` gets a `views=` option. A `views` scene section is also accepted, but no shipped scene gets one,
   so no scene sha, golden or headset changes. Without them, no code path runs, and the records and bytes are identical.
   The shipped spec would be `robot/isaac/views/recon.json`.
2. **Views are kept out of `CM`.** That way takes.py, the safety datasets and the jitter/mask checks never see them. The
   new records (in SESSION_FORMAT.md):
   - `VS` `change seed sha256`: the spec in force. The spec itself is copied to `isaac/views.json` through final_files.
   - `VW`, one per view per change and in the preamble: `change camera width height fx fy cx cy near far x y z qw qx qy qz
     streams mode draw gain motion`. Here `draw` is `hfovDeg:azimuthDeg:elevationDeg:distanceM:rollDeg:tx:ty:tz:attempts`.
   - `VP` `frame stateSensed poses`, before each `VF`, only for moving views: `view:x:y:z:qw:qx:qy:qz;...`.
   - Streams are `view<k>.rgb|depth|mask`. Mask ids follow the existing `ID` table.
3. **Resolution is fixed per slot**, because a render product cannot be resized per take: slot k uses
   `slots[k % len]`, 640x360 (DaBai-like) and 1280x720 (Femto Bolt colour, 80°H x 51°V per Orbbec's docs). The DaBai FOV
   was not found and is unverified. The FOV is drawn per take (about 60-90°) by setting the camera's focalLength attribute.
4. **Sampling**, seeded and pure, using `random.Random("teleop-view:<seed>:<change>:<view>")`:
   - a look-at target in a box over the reach region (x 0.12-0.45, y ±0.30, z from the table top -0.196 to 0.0);
   - azimuth 0-360, elevation 8-80°, distance 0.6-2.0 m, roll ±4°.
   Rejection rules (all from the spec plus the scene's surfaces):
   - the camera must lie outside:
     - the robot body box, URDF base_link collisions x[-0.21,0.14] y[-0.13,0.13] z[-0.20,0.09];
     - the arm sphere, centre (0,0,0.035), r 0.55;
     - each collidable surface's footprint +0.10 m, up to 0.75 m above its top;
   - floor +0.3 m ≤ z ≤ floor +2.6 m, and horizontal radius ≤ 2.2 m;
   - workspace points (a 5x5 grid on the reach region of the table top, plus a few arm points): ≥ 80% in frame (2% border),
     ≥ 70% unoccluded by the body and slab boxes, and their projected bounding box ≥ 12% of the image;
   - at least 15° between views in one take. After 200 attempts, the best candidate is kept and flagged.
5. **Static and moving.** Each view is static for the take with probability 0.7. Otherwise it moves on a closed periodic
   loop around its centre pose:
   - az + A sin(ωt), el + A sin(ωt+φ), r + A sin(2ωt+φ), target drift;
   - a period of 8-30 s, checked against every rule at 48 phases.
   - t counts render frames since the change, divided by fps.
   - Timing: a camera pose set at loop iteration k shows in the image read at k+lag-1 (as for the riding cameras). So VP
     for frame m is the pose of iteration m-lag+1. The twin check must confirm this: back-project the table depth through
     VP, and through VP shifted ±1 frame.
6. **Renderer plumbing:**
   - view sensors are created before `play`, with no feed;
   - each view meters its own gain in settle_room;
   - the preamble waits for the change when views are on;
   - the `sight` radius covers the views (target + far + 2.0 + 0.5, still clear of the 14 m shelf);
   - encoding runs in a thread pool when views are on, because one writer thread already spends about 18 ms a frame on the
     three base cameras, and a 720p view adds about 24 ms.
7. **LeRobot:** safety datasets do not carry views (unchanged). Reconstruction reads the takes. A new read-only
   `robot/isaac/views_check.py` would check them: the records match the draws, the depth through VW/VP lands on the table
   top with p99 < 3 mm, the ±1-frame association holds, and `--sheet` makes a contact sheet. A lerobot test would check
   that a take with VW/VP and view parts builds the same.

## Cost estimate (not measured)
Two twins now use 63-80% of the GPU with three 640x360 cameras each. That is roughly 3.9 ms of GPU per 360p camera per
frame per twin, about 12% of the GPU per twin. A 720p view is about 4x that.
- Two twins: about one 360p view each at most.
- One twin: about two 360p views, or one 720p view.
Provisional default: N=2 at 640x360 on one twin. The 720p slot is opt-in. This must be measured: N ∈ {0,1,2,3} and 720p,
from the renderer's 5 s `fps` and `render`/`read` ms. That needs the user's go via the coordinator.

## Next
1. Write the amendment (design above) in docs/adr/0038 and commit it.
2. `robot/isaac/teleop_isaac/views.py` (spec, draw, rules, trajectory, record encoding) and
   `robot/isaac/tests/test_views.py`: determinism, bounds, each rejection rule, moving loop, records, shelf out of sight.
3. Wire renderer.py, the justfile `views=` option, the SESSION_FORMAT rows, views_check.py and the lerobot ignore test.
   Run `scripts/gpu/gate.sh isaac lerobot`.
4. Report the twin check wanted: a fps sweep, then a short views batch on one twin, then views_check and a contact sheet.

## Pending decisions
- The default N and whether 720p is on by default: needs the measurement.
- The moving share (0.3), and whether moving views should also vary FOV during a take (proposed: no).
