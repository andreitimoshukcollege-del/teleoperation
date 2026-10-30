# Unity — TeleopXR (Samsung Galaxy XR, Unity 6000.6.4f1, URP, hand tracking)

The Galaxy XR operator project (docs/adr/0016). It shares Core, RobotArm and Bridge with
`unity/TeleopVR` (Quest) by `file:` path and adds only what Android XR and hand tracking need. Rules
shared by every Unity project are in `unity/CLAUDE.md`, and Bridge's are in
`unity/Teleop.Bridge/CLAUDE.md`.

## Pinned stack

The versions are the ones Unity 6000.6.4f1 recommends, pinned in `Packages/manifest.json`:

| Package | Version | Why |
|---|---|---|
| URP | 17.6.0 | Google's recommendation for Android XR; HDR off, no post-processing |
| OpenXR | 1.18.0 | |
| Unity OpenXR: Android XR (`com.unity.xr.androidxr-openxr`) | 1.4.0 | Android XR Support; adds the `HAND_TRACKING` permission to the manifest |
| Android XR Extensions for Unity (`com.google.xr.extensions`) | git tag v1.4.0 | Direct Preview (the Android XR Streaming feature). Apache-2.0 |
| XR Hands | 1.9.0 | the Hand Tracking Subsystem |
| XR Interaction Toolkit | 3.6.1 | the hands rig and pinch interactions |
| AR Foundation / XR Management / Input System | 6.6.2 / 4.7.0 / 1.20.0 | dependencies |

There is no Meta SDK and no Quest feature group. Quest builds come from TeleopVR.

## Project configuration is code

`Assets/Teleop/Editor/XrProjectSetup.cs` (**Teleop → XR → Configure Project**, or
`just unity-setup-xr`) sets everything: the samples, URP, the Android Player settings, and OpenXR
for Android and for Windows (Direct Preview). Its doc comment gives the reason for each value.

**To change a setting, change it there and rerun.** A value clicked into an Inspector is undone
by the next run, and nobody can review it.

## Fresh clone

1. Install Unity **6000.6.4f1** with Android Build Support (OpenJDK, Android SDK & NDK).
2. Run `just unity-setup-xr` **twice**. The first run imports XRI's Starter Assets and Hands
   Interaction Demo samples and XR Hands' HandVisualizer, which compile only after it ends. The
   second run confirms the rest.
3. `Assets/Samples/` is **gitignored**: the samples are regenerated from the pinned packages, and
   their GUIDs ship inside the packages, so the committed scene's references resolve. Never edit
   anything under `Assets/Samples`; the next import overwrites it.
4. `just unity-check-xr`. The scene, `Assets/Scenes/JetRoverLabXR.unity`, is committed.

## The scene (`JetRoverLabXR`) and its safety settings

`XrLabSceneBuilder` (**Teleop → XR → Build Lab Scene**, or `just unity-scene-xr`) made it, and it is
create-only, like TeleopVR's lab. It dresses the lab as it builds (see *The lab* below). From now on
the scene is edited by hand in the editor. `rebuild=true` replaces it and discards those edits.
Contents:

- **XRI's `XR Origin Hands (XR Rig)`.**
  - Its modality manager has **no controllers**, so the scene is hands only.
  - Tracking origin is **Floor**.
  - Its **One Euro hand-smoothing filter (`HandsOneEuroFilterPostProcessor`) is disabled.** It
    filters every hand joint, including the pinch pose that becomes the robot's target.
- **The arm rig, `JetRoverOperatorBridge` and `CameraFeedBridge`.** Values are identical to
  TeleopVR's `JetRoverControl`, so the IK sees the same arm in both projects.
- **`HandTrackingSession`.** It asks for the `HAND_TRACKING` runtime permission and logs the session's
  input modality once.
- **Clutch control and the gripper (docs/adr/0017), from `XrControlSetup`** (**Teleop → XR → Set Up
  Clutch Control**; a lab rebuild reruns it):
  - **`HandClutchDriver`** on `JetRoverOperator`.
    - The **right hand's pinch is the clutch**: pinch anywhere and the arm's wrist follows your
      hand's motion from there, scaled by `ClutchScale` (0.5). Release and the arm holds, so you
      can re-pinch somewhere comfortable.
    - **The left hand acts only while the right hand is pinched**, like the arm. Releasing the
      right hand holds the gripper and the roll.
      - Its **pinch is the gripper** (motor 10): thumb and index 8 cm apart or more is open,
        1.5 cm or less is closed, linear in between.
      - Its **twist is the wrist roll** (motor 5), relative: the wrist turns by how far the left
        hand twists after the right hand pinches, from wherever it was. It is clamped to ±90°.
        `GripperRollSign` flips the direction.
      - Relative because XR Hands' palm "up" is not where an absolute mapping assumed: a flat hand
        read −90° and pinned the gripper vertical.
    - It reads hand joints from XR Hands directly. The mapping is Core's (`Teleop.RobotArm`'s
      `ClutchMapping` and `GripperMapping`), reached through Bridge's `JetRoverClutch`.
    - Feedback: the wrist LED ring turns cyan while clutched. **Orange** means the IK tilted the
      wrist to keep clear of the robot, and **red** means the target is out of reach or blocked by
      the robot's body, so the arm has stopped short (docs/adr/0020). The display's caption line
      reads `CLUTCH ON · gripper N%`.
  - **`DragTarget`** is no longer grabbable. It is a 2 cm marker showing the commanded wrist
    position, and the bridge still reads it.
  - **`TwinGripper`** on the wrist closes the twin's jaws by the gripper command and rolls the
    gripper by the wrist roll. It shows what is sent; the real gripper has no feedback.
  - **`MotionToggleButton`, the robot's power and pause buttons, on the bench's front-left corner**
    (docs/adr/0023 decision 5). Press them down with a fingertip. Only a fingertip counts; no ray or
    far pinch can press them. The robot is **off at every launch**.
    - **Power (the big one): red `ROBOT OFF`, green `ROBOT ON`.** On switches the arm's torque on, and
      no command goes until RobotHost reports the arm holding (`STARTING`). Off parks the arm at its
      rest pose and then switches torque off (`PARKING N°`, then `relaxed`). If RobotHost refuses the
      park, the label says why, and the arm keeps its torque where it stopped.
    - **PAUSE (the small one to its left), amber when paused.** It works only while the robot is on.
      Paused sends nothing and keeps torque on, so the arm holds where it is; press PAUSE or power to
      go on again. Use it instead of power when you do not want the arm to relax.
    - **Taking the headset off is ROBOT OFF** (`HeadsetRemovalGuard`, the user's choice, 2026-10-27):
      the arm parks and relaxes. When Android pauses the app or it loses focus, the relax request goes
      out three times before the app stops; where the runtime reports user presence, it also turns off
      after 0.5 s without you. Putting the headset back on does not turn it on again, so the arm never
      jumps to wherever your hands are. The power label then says why it is off. Builds only for
      pause and focus: the editor loses focus whenever another window is clicked.
    - Off and paused both send nothing while the virtual arm still follows your hands, and both
      offer the camera display for moving. Both buttons show LOCKED when `ConfirmHardwareMotion` is
      off in the config.
  - Tune these with `just push-galaxy-config`, no rebuild: `ClutchScale`, `ClutchMinHeightMeters`
    (the wrist never goes lower, default −0.05 m below the base pivot), `GripperPinchOpenMeters` and
    `GripperPinchClosedMeters`. Edit them in `unity/Teleop.Bridge/Runtime/Resources/jetrover_connection.json`
    first; the recipe pushes that file.

- **The twin's three arms (docs/adr/0018), built by the same `XrControlSetup`:**
  - **Predicted, solid** (`PredictedArm`): where the robot most likely is right now. It is the
    newest joint-state frame from the robot, rolled forward by the joint predictor through the
    commands sent since. Which predictor does it is chosen on the operator console (below), from
    the config's `JointPredictorPresets`; `JointPredictorActive` names the one drawing at launch:
    **`learned`, the trained model** (docs/adr/0025; `finetune-pilot2-early7`, shipped in Bridge's
    `Resources/models/`), with the conventional presets beside it on the console. Its gripper is not
    learned yet (worse than `servo 10-25`'s). `just push-galaxy-model <run>` puts a newer model on the
    headset without a rebuild; name it in a preset's `Model` and `just push-galaxy-config`.
    `just unity-check-xr` (and so `build-galaxy`) fails if a `learned` preset's model does not load and run.
  - **Commanded, ghost**: the clutch's arm, where you told it to go.
  - **Measured, faint amber** (`MeasuredArm`): the newest frame exactly as received, about one
    network trip old.
  - **Colour code** (`XrControlSetup.TwinColors`): each arm, its switch and its wrist LED share one
    colour.
    - **Commanded, blue:** the target marker, the ghost arm, and the clutch cue.
    - **Predicted, green:** the robot's own green; the arm itself stays solid.
    - **Measured, amber.**
    - The sliders are neutral white.
  - **Switches.** Three fingertip switches next to the motion button (PREDICTED, COMMANDED,
    MEASURED) show or hide each arm. Each is lit in its colour when the arm is shown, and dim when
    it is hidden. The line above them says
    whether the robot's state is arriving: `NO ROBOT STATE` means RobotHost is too old for the
    stream; otherwise it shows `live · rtt N ms`.
  - **Mechanics.** `TwinArmsDriver` (on `JetRoverOperator`) drives the predicted and measured
    arms in `onBeforeRender` from Bridge's `JetRoverJointStateLink`. That link sends the
    keepalive on the joint socket, receives the frames, runs its own `ClockSync`, and feeds the
    Core predictor. The arms are hidden until the first frame arrives, and the predicted one
    until the clocks are synced.
  - **See-through only on device.** `ArmAppearance` makes an arm see-through at runtime only, so
    editor previews show all three solid. The predicted and measured arms start inactive, so
    previews show only the commanded arm.
  - **Finding the right arm.** Editor tools find the commanded arm through the bridge's `armRig`
    (`XrControlSetup.CommandedRig`), never "any rig": the scene holds three.

- **The operator console (docs/adr/0023)**, on the bench's left behind the motion button, built by
  `XrControlSetup.BuildConsole`:
  - **Front row, predictors.** One slot per `JointPredictorPresets` entry (up to six; labels come from
    the config at runtime). Press a slot's cap to draw the twin with that predictor; it lights green.
    The small dot beside it turns the predictor on or off. Every predictor that is on runs side by side
    on the same stream (Core's `JointPredictorBank`), so switching is instant and keeps history; one
    turned on again starts from nothing and shows `warming up` until its first reading. The drawing
    one cannot be turned off.
  - **Behind it, three amber network sliders**: LAG (one-way, 0–1000 ms), JITTER (0–200 ms) and LOSS
    (0–30%), applied together on the headset to the joint link in both directions
    (`JointLinkDisturbance`) once they stop moving for 0.4 s; all at zero is off. While one is on,
    the panel and the twin's status line say `NETWORK DISTURBED`, and each change is logged with its
    seed. Anything recorded with one on is not a real-network measurement: the headset reports the
    settings to RobotHost, which writes `HD` records and marks the session, and the trainer refuses it
    (docs/adr/0023 amendment). Selecting TRAIN on the training course resets them, and ADDED DELAY, to
    zero, and a running course holds them there (`· COURSE`). The bridge's caps
    (`DisturbanceMaxDelayMs`, `DisturbanceMaxLossPercent`) bound them.
  - **Servo board silent.** When frames keep arriving but the servo readings stop advancing for 3 s,
    the twin's status line and the panel say `SERVO BOARD NOT ANSWERING · power-cycle the robot`
    (`JetRoverJointStateLink.BoardSilent`; robot/README.md's incident log has the fault).
  - **The panel**, raised above the sliders: each preset's one-line `Description` from the config, and for every predictor that is on, error p50/p95, correction p95 and
    jitter p95 (degrees, arm joints) and the gripper's error p95 (servo degrees), over the last 30 s,
    four times a second (Core's `LiveJointScorer`, docs/metrics.md §4's live variants). Diagnostics,
    never results.
  - New presets need no rebuild: edit `JointPredictorPresets` in the Resources JSON and
    `just push-galaxy-config`. `just unity-check-xr` checks the console's wiring (not a safety
    setting: it moves nothing).
- **The training course (docs/adr/0024)**, on the bench's right, built by `XrControlSetup.BuildTrainingCourse`
  (`TrainingCourseView`; the logic is `Teleop.RobotArm`'s `TrainingCourse`, reached through Bridge's
  `JetRoverTrainingCourse`). Something to do while the robot records data for the learned predictor:
  - **START/STOP** runs rounds in turn: **REACH** (eight magenta targets, one at a time; hold the
    marker on each for 0.4 s, and some ask for the gripper OPEN or CLOSE), **TRACE** (keep the marker
    on a bead moving round a closed curve, 45 s) and **SHUTTLE** (touch two targets in turn, fast, 30 s).
    Each is announced for 3 s and its score shown for 3 s.
  - **The course sets the arm speed** for each round (1.0, 2.5, 3.8, 6.0 rad/s, or varied), the level
    with the fewest minutes so far; the speed slider follows it and reads `· COURSE`. The minutes per
    level are kept on the headset (`PlayerPrefs`) and shown on the panel.
  - **TRAIN/TEST** labels the next rounds' data; TEST (amber) is held out, for sessions recorded after
    a model is frozen.
  - A round's clock runs only while the robot is on and holding; paused or off, it waits
    (`suspended`). What scores is the commanded wrist, not the robot.
  - Every target and every point of a trace is one the headset's IK reaches from home, above the
    clutch's floor. A round is drawn when it is announced: up to about 14 ms for a trace on the PC,
    likely a few times that on the headset, so watch for a dropped frame then.
  - The headset sends the course state to RobotHost, which writes `TK` records into the session
    (`trainer/contract/SESSION_FORMAT.md`). That needs a RobotHost from this change
    (`just deploy-robothost`); an older one ignores it.
  - Wiring is checked by `just unity-check-xr` (not a safety setting: it moves nothing the clutch
    does not). Preview 15 of `just unity-lab-xr` shows the caps and panel.
- **The added-delay slider** (orange, beside the speed slider): 0–400 ms held back on the robot's
  state reports before they become the measured arm and reach the predictor. Commands are not
  delayed, and clock sync sees the real arrival.
  - **Why:** on the LAN the robot answers in about 2 ms (`just joint-probe`), so there is almost
    nothing for prediction to hide. With, say, 200 ms added, the measured arm visibly trails the
    command and the predicted arm should keep up with it.
  - **Recordings:** anything recorded with a delay set is not a real-network measurement.
- **What the robot measures:** `just joint-probe` prints which motors have servo feedback.
  - **Motors 1–5 are measured** since `just robot-set-feedback-ids` (2026-10-12). Before that the
    relay only read 1–4.
  - **Motor 10 (the gripper) never is.** Commanding it works (it closes and opens), but its servo
    does not answer position reads, so its value is only what RobotHost last commanded.
  - **On the measured arm:** it hides its gripper while the roll is unmeasured.
  - **On the predicted arm:** the predictor keeps its own estimate of an unmeasured joint,
    moving at servo speed, instead of copying the belief. The gripper takes its command.
- **The arm-speed slider** (`SpeedSlider`) on the bench's left, left of the motion button.
  - Slide an index fingertip along it: near you is slow (0.5 rad/s), far is fast (6). It starts at
    3.8.
  - It sets `JetRoverOperatorBridge.SpeedRadPerSec`, which rides in every joint command's `Speed`
    field.
  - RobotHost turns it into the servo speed for both command paths, never faster than its deployed
    `pulses_per_second` (default 2200, about 9 rad/s).
  - The predictor replays each command at that same speed, capped at `JointModelSpeedRadPerSec` (6,
    the servo-limit guess).
- **Moving the camera display, only while robot motion is off.**
  - Point at the display with your right hand. The ray runs from about your shoulder through
    your pinch point, as Android XR's hand rays do. When it is on the display, a yellow line
    appears and the bar on top turns yellow.
  - Pinch to grab, and the bar turns green. Move your hand to sweep the display around you. It
    slides over the floor and keeps facing you.
  - Where a pinch starts decides what it holds: the display under the ray, or else the clutch.
  - With motion on, the display is never offered, and switching motion on mid-move lets go of
    it. A pinch therefore can never drive a live robot while you meant to move the display, or
    the reverse.
  - The display's stand and bezel are dynamic, lit by light probes, so they move with it. Its
    position is not saved between sessions.

**These are safety settings.** `just unity-check-xr` fails, and `just build-galaxy` refuses to
build, if any of them changes:

| Setting | Value | Because |
|---|---|---|
| `DragTarget` interactable | none | a pinch could grab it and move the robot outside the clutch's scale and workspace limits |
| `HandClutchDriver` | present, wired to the bridge, rig and XR Origin, enabled | nothing else drives the arm |
| Clutch engage / release distance | engage below release (1.5 / 3 cm) | otherwise the clutch chatters on and off |
| `ClutchScale` | in (0, 1] | above 1 the arm moves further than the hand |
| Gripper pinch range | open distance > closed distance | otherwise the mapping is inverted |
| `JetRoverOperatorBridge.motionEnabledAtStart` | off | the robot must not move at launch, before anyone presses the button |
| `JetRoverOperatorBridge.snapTargetToReachable` | on | when the IK stops at the robot's body, the marker moves to where the arm stopped, so it never sits inside the robot and the clutch re-engages from the arm (docs/adr/0020) |
| `MotionToggleButton` | one Power and one Pause, each present, wired, enabled | power is the only way to start, and stop, the robot from the headset |
| `HeadsetRemovalGuard` | present, wired to the bridge, enabled | taking the headset off must turn the robot off: nobody is operating it then |
| `TwinArmsDriver` | present, wired, not driving the commanded arm | the robot's state must never overwrite the display of the command |
| Hand smoothing filter | inactive | it filters the hand joints the clutch reads; intent is sent raw (ADR 0016 §5) |
| Modality manager controllers | none | hands only |
| Active Input Handling (Player Settings) | Both | XRI's hand interactions read the Input System |
| Android XR Support, Hand Tracking (OpenXR, Android) | on | without them the APK launches as a 2D panel and crashes |

The clutch is safe by construction, not just by settings:
- **Releasing, or losing tracking of the clutch hand,** disengages, and the arm holds.
- **The target is projected onto the reachable workspace every frame,** above
  `ClutchMinHeightMeters`. Pushing past the edge leaves the arm at the edge, and moving back moves
  it immediately.
- **The IK never asks for a pose that hits the robot** (docs/adr/0020). It knows the robot's body
  and joint ranges from a collision model generated from the CAD (`just robot-collision-model`).
  - It tilts the wrist first to reach the target exactly.
  - Failing that, it stops the arm at the body, and the marker follows.
  - RobotHost enforces the same model and refuses any colliding pose, so a bug or another tool
    can't drive the arm into the chassis either.
- **Losing tracking of the gripper hand holds the gripper.** It never opens on its own.

What the check cannot see:

- **The gripper's pulse range is uncalibrated** on the robot (`GenericArmPlantConfig.GripperOpenPulse`
  and `GripperClosedPulse`). Measure it with `just move-arm 0.15 0 0.08 <gripper 0..1>` while you
  watch the robot, before the headset drives the gripper.
- **Whether the palm's pitch feels right for the wrist** is a headset question. `followPitch` on
  `HandClutchDriver` turns it off.
- **Hand jitter and dropouts reach the robot**, scaled by `ClutchScale`, through the same IK and
  48 Hz command stream. Filtering them would be a Core research axis with its own ADR, never a
  smoothing checkbox here.
- The base rig also carries locomotion providers (move, turn, teleport, climb, grab-move). All of
  them are driven by controller input actions, so with no controllers they do nothing.

## The FR3 Duo scene (`Fr3DuoXR`, docs/adr/0029)

A second scene for the simulated FR3 Duo, beside the JetRover's lab, which it leaves alone. It is built by
`XrFr3SceneBuilder` (**Teleop → XR → Build FR3 Duo Scene**, or `just unity-scene-fr3-xr`). Like the lab
builder, it only creates the scene; pass `rebuild=true` to replace it.

**Trying it, from the editor (Direct Preview):**

1. Start the simulated robot on this PC with `just fr3-sim-host`. It listens on the joint port, 6002, and
   records every session the headset commands into `recordings/fr3-sim/`.
2. Open `Assets/Scenes/Fr3DuoXR.unity` and press Play. `fr3_connection.json`'s `RemoteHost` is 127.0.0.1,
   which is this PC, so no push is needed.
3. For the APK instead: `just build-galaxy-fr3` builds `Builds/TeleopXR-FR3.apk`, and installing it
   replaces the JetRover app. Then run `just push-galaxy-fr3-config` to point the app at this PC's LAN
   address.

**Driving it.** Each hand drives its own arm: the left hand the left arm, the right hand the right arm.

- **Grab** by curling your middle, ring and little fingers into the palm. That arm then follows your
  palm, position and rotation, from where both were when you grabbed. Open the hand, or lose tracking
  of it, and the arm holds. Grab again anywhere and it carries on from there.
- **While a hand is grabbed, its thumb-index gap is that arm's gripper:** 8 cm or more is open, 1.5 cm
  or less is closed. Letting go holds the gripper.
- **The status line over the robot** shows `HELD`, `OUT OF REACH` (orange) or `BLOCKED` (red) for each
  arm, with its grip. Blocked means the step would hit the other arm or the body, so the arm stays put.
  The markers show where each hand is asking the tool to be, in the same colours.
- **The motion button** is on the table's front-left. Press it with a fingertip. The scene starts
  `ROBOT OFF`, and while it is off the markers still follow your hands. Switching it on takes the
  robot's pose as the command first, so nothing jumps.
- **The twin** shows the active predictor's estimate: `franka-replay` at launch, with `hold-last` and
  `learned (sim)` also in `fr3_connection.json`. It is built from the URDF's collision meshes, a few
  hundred triangles per link. The visual CAD is 59 MB.

**Safety settings.** `just unity-check-fr3-xr` fails, and `just build-galaxy-fr3` refuses to build, if
any of these changes:

| Setting | Value | Because |
|---|---|---|
| `SerialChainOperatorBridge.motionEnabledAtStart` | off | nothing moves before someone presses the button |
| `DuoHandDriver`, `Fr3MotionButton` | exactly one each, wired, enabled | nothing else drives the arms or starts them |
| Grab engage / release curl | engage below release (1.25 / 1.55) | otherwise the clutch chatters |
| Gripper pinch range | open > closed | otherwise the gripper is inverted |
| `PositionScale` / `MaxStepRadians` | (0, 2] / (0, 0.2] | the arm moves no further than twice the hand, and never far between checked poses |
| Hand smoothing, controllers | inactive, none | as in the JetRover scene |
| The twin | its tool frames within 1 mm / 0.1° of Core's FK at 25 poses per arm | a twin that is not the robot looks exactly like prediction error |

**Before the headset: `just fr3-teleop-probe`** runs this scene's link and teleop with scripted hands
against a running `just fr3-sim-host`, over the real socket. It checks the whole path except the
hands and Unity: sync, the first command, clutch, IK, collision, sending and the twin's prediction.
`just unity-preview-fr3-xr` renders the scene, with the twin at the ready pose, to `%TEMP%\teleop-fr3\`.

**What the check cannot see:** whether the grab thresholds suit your hands on this headset's
tracking, whether the clutch's rotation feels right, and the frame time on the device. All three
thresholds and both limits can be changed without a rebuild, with `just push-galaxy-fr3-config` after
editing `unity/Teleop.Bridge/Runtime/Resources/fr3_connection.json`.

## Testing on the Galaxy XR

**Once per headset: developer mode and USB debugging.** The menu names below are from ManageXR's
Galaxy XR guide and may differ slightly between headset software versions.

1. **Settings → About device**: tap **Build number** seven times and enter the PIN.
2. **Settings → System → Developer options**: turn on **USB debugging**.
3. Connect it with a data-capable USB-C cable. In the headset, allow USB debugging for this
   computer, and tick *always allow*.
4. `just _adb devices -l` should list the headset as `device`. If it shows `unauthorized`, the
   prompt in step 3 is still waiting.

**Direct Preview** (Play mode streamed to the headset, the Link equivalent). It needs Windows 11
and a GPU with Vulkan video encoding (this PC: RTX 4060 Laptop).

1. **Once per PC:** install Google's **Android XR Engine Hub**. Close the Unity editor and open the
   Engine Hub. Pick the headset in the device drop-down, click **Install Stream Client**, then
   click **Set** next to **Active OpenXR Runtime**. That runtime is a setting of the machine, not
   the project, which is why the setup code doesn't make it.
2. **Once per clone:** `just unity-setup-xr` (or **Teleop → XR → Configure Project**) sets the
   editor up the way Android XR Streaming requires: Vulkan only on Windows, multi-pass, 24-bit
   depth, legacy foveation. The editor changes graphics API only when it restarts, so reopen it;
   its title bar then ends in `<Vulkan>`. **Project Settings → XR Plug-in Management → Project
   Validation**, Standalone tab, should list no `[OpenXR]` or `[Android XR Streaming]` errors.
3. `just unity-robot-host` points Play mode at the robot's current address.
4. Connect the headset over USB-C and press Play. The stream starts by itself.

**APK** (the headset alone, over Wi-Fi):

1. `just build-galaxy` produces `Builds/TeleopXR.apk`. It runs the scene check first and refuses
   to build if the check fails.
2. `just install-galaxy` installs it over USB.
3. `just push-galaxy-config` points the app at the robot's current LAN address without a
   rebuild. Use `just push-galaxy-config auto false` the first time you try the controls: the
   virtual arm moves and the real one doesn't. It finds the robot itself (`just robot-ip`); pass an address to override. It pushes a
   full `jetrover_connection.json` override to the app's files directory; restart the app
   afterwards.
4. The headset must be on the robot's network, or run Tailscale. **It finds the robot itself at
   start** (docs/adr/0022): it asks through the robot's Tailscale address for its LAN addresses and
   uses the LAN when the robot answers there, or asks for `jetrover-sinrg.local` over mDNS when
   Tailscale is not running on the headset, so a new DHCP lease needs no config push. The pushed `RemoteHost` is the
   fallback when nothing answers. The Console logs each `[discovery]` step. `DiscoverRobot: false`
   in the config turns it off. RobotHost must answer (`--discovery-port`, `just deploy-robothost`);
   `just robot-discover` runs the same lookup from the PC. Accept the hand-tracking permission on the
   first launch.

**Watching the headset from the PC:** `just mirror-galaxy` mirrors what the wearer sees into a
window on this PC over USB, with scrcpy (`winget install Genymobile.scrcpy` once). Extra flags pass
through to scrcpy, e.g. `just mirror-galaxy --record=session.mp4`.

**The robot must reply to the headset.** This applies to the APK; in Direct Preview the PC is the
sender. Make sure the Jetson runs the reply-to-sender RobotHost: `just deploy-robothost`. The
camera sender already answers whoever sends the keepalive.

**M2P on Galaxy XR is relative only** until its `DisplayOffset` has been measured with the photodiode
rig and pushed as a per-device `display_calibration.json` (ADR 0016 §6).

## The lab: room, twin and display are generated

The night-shift lab around the arm is built by code in `Assets/Teleop/Editor/Lab/`:

- the room: sign wall, window skyline, desks, racks, shelving, a taped workcell with a status ring,
  a mobile-robot test area, motion-capture cameras;
- the JetRover twin, from the robot's CAD (below), with the arm parts on the rig's
  pivots;
- the 1.2 × 0.9 m camera display on a stand, with a live status strip and, under it, the group's
  nameplate (SPATIAL INTELLIGENCE RESEARCH GROUP, in red).

It goes into the scene under one `LabDressing` root. The meshes, textures, materials and lighting
settings it generates go to `Assets/Teleop/Lab/Generated/`.

- **Review it headless:** `just unity-lab-xr`, or the **Teleop → XR** menu: *Rebuild Lab Dressing*,
  *Bake Lab Lighting*, *Capture Lab Previews*. It rebuilds the dressing and bakes the lighting (the
  bake takes under a minute on the RTX 4060; the whole run takes a few minutes). It then renders nine
  fixed views and a contact sheet to `%TEMP%\teleop-lab\<time>\`, with a `report.txt` of triangles,
  renderers, materials and lightmaps. Look there before putting on a headset. `bake=false` skips
  the bake. Close the editor first.
- **Regenerated, never hand-edited.**
  - A rebuild deletes and recreates `LabDressing` and the twin's `TwinVisual` children. It also
    deletes the builder's original placeholder roots, `Room`, `CameraPanel` and `ConnectionStatus`.
  - It rewrites the generated assets in place, so their GUIDs stay stable.
  - It resets the directional light, the scene's lighting settings and the wiring below.
  - Hand edits anywhere else in the scene survive. To change the look, change the code and rerun.
- **It rewires three things, and `just unity-check-xr` guards both safety-relevant ones:**
  - `CameraFeedBridge.panel` becomes the display's screen, with `armRig` left empty. With an arm
    rig set, the bridge would move the screen out of the display at start.
  - `JetRoverArmRig.reachWarningRenderer` becomes the LED ring on the twin's wrist. The old segment
    cubes are hidden, not deleted, and the check fails if the warning renderer isn't drawn.
  - `CameraStatusBar` drives the display's strip, and `LinkStatusLamp` the workcell ring. These
    are display-only and reference Bridge only.
- **Lighting is baked.**
  - Subtractive mode: area lights under and over the pendant fixtures, cove lines, light probes
    where the arm moves, and one box-projected reflection probe.
  - The single Mixed directional light gives the moving arm a real-time shadow on the bench, which
    tells the operator how high the gripper is.
  - The cove lines and fixture diffusers have HDR colours. The screen clamps them, but the
    reflection probe keeps the full value, so the glossy floor reflects them.
  - Bake output (`Assets/Scenes/JetRoverLabXR/`) and the mesh library are LFS-tracked.
- **The twin is the robot's own CAD** (`JetRoverCad`): one FBX per URDF link in
  `Assets/Teleop/Lab/RobotModel/` (Body, Base, LowerArm, CenterArm, UpperArm, Gripper). The user
  modelled it and it is free to share; it is LFS-tracked.
  - The files come from Blender at 4.4435 units per metre, with the arm standing straight up and the
    robot facing −z. Base.fbx lacks the 270° X rotation the other parts have. `JetRoverCad` corrects
    all three and snaps each arm part onto its rig pivot, so the IK's 13 cm links are what moves.
    The CAD's links are 13.04 and 12.91 cm.
  - The CAD puts the shoulder on the yaw axis, so the rebuild zeroes the rig's old 14 mm sideways
    shoulder offset. That offset was only visual; the IK never used it.
  - The bench is set from the CAD: the wheel bottoms are 19.62 cm under the yaw pivot.
  - The gripper keeps the URDF's joints. Each jaw link's origin is its hinge, so `TwinGripper` turns
    `l_link`/`r_link` by up to 37° (the tips meet) and their fingers back by the same, which keeps
    the pads parallel, as the real linkage does.
  - **The meshes are dense:** about 170k vertices per arm, three arms, plus 33k for the chassis.
    If the headset drops frames, decimate them in Blender first.
  - `RobotModelInspector.Run` reports how they imported, and `Render` draws each part alone, to
    `%TEMP%`. Previews 11 and 12 of `just unity-lab-xr` show the gripper open and closed.
  - Without the files, the rebuild falls back to `JetRoverTwin`'s procedural model.
- **TextMesh Pro's essential resources are committed** in `Assets/TextMesh Pro/`; the lab's labels
  need them. Batch mode cannot import them, because the import is asynchronous and `-quit` ends the
  run first. `just unity-setup-xr` only checks that they are there.

## Not here yet

- Controllers, passthrough and eye tracking (ADR 0016 *Out of scope*).
- Robot discovery by name (ADR 0015).
