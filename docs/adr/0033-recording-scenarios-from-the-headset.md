# ADR 0033: Recording scenarios from the headset

## Status

Accepted, 2026-11-06. Built on branch `sim-scenario-record`.

## Context

A labmate will record training data with the simulated FR3 Duo (ADR 0031, ADR 0032 amendment 3). He works on the GPU
machine, under his own account (`seaqueue-cheng`), and does not use the repo. What exists today:

- **Automatic sessions** (ADR 0023 decision 4). RobotHost records whenever a headset commands the arms, into
  `~/isaacsim/recordings`. They are named by time alone.
- **Hand-made sessions** with `just record-start` and `record-stop`, from a PC with the repo.

Neither says what the operator was doing, and neither is reachable from his account.

The user's choices (2026-11-03):
- a RECORD button in the headset, not a desktop app;
- the scenario picked from a short list on the bench, with a REC light and a timer;
- the data delivered through a shared folder on the GPU machine.

## Decision

1. **A take is a session.** RobotHost opens and completes it exactly as an automatic session (`SessionRecorder`).
   - The Isaac renderer follows the control file into it, so the take has the four cameras' frames.
   - RobotHost waits for the renderer's trailer, checks the `FS` lines, and writes `files.sha256`.
   - **Where it goes:** `<scenarios>/<scenario>/<UTC>-take<n>/`. `n` counts up per scenario, past any take set aside.
   - **`session.json`:** `label` `scenario-<name>`, `conditions.motion` `scenario:<name>`, and a `scenario` object
     (`name`, `take`, `description`).
2. **The list is a text file the labmate edits:** `<scenarios>/scenarios.txt`.
   - **Format:** one scenario per line, `name | what to do`. `#` starts a comment.
   - **Names:** 1–24 characters from letters, digits, `.`, `_` and `-`, not starting with `_` or `.`, because a name names
     a folder. At most 16 scenarios.
   - **Bad lines:** a line that breaks a rule is skipped, and RobotHost's log says why. An empty file means `free-play`.
   - **Template:** RobotHost writes one when the file is missing.
   - **Changes:** the file is re-read when it changes (checked every 2 s), and the headset shows the new list within a few
     seconds.
3. **Three new joint-port datagrams** (`Teleop.RobotArm/Wire/RecordCodec.cs`). They are control, not robot state, so the
   headset applies them as they arrive, outside its network disturbance and playout.

   | Tag | Direction | What |
   |---|---|---|
   | `0x88` request | headset → RobotHost | start / stop / discard last, the scenario's index and the list revision it indexes; re-sent every 0.25 s until echoed, given up after 2 s |
   | `0x89` status | RobotHost → headset | idle or recording, the last request's echo and outcome, the scenario, elapsed time, take number, takes kept, free disk; once a second and on change |
   | `0x8A` list | RobotHost → headset | the names, at most 8 per part, with the list's revision; every 3 s and on change |

   - A start names the list revision its index is into. If the list changed meanwhile, the answer is "unknown scenario",
     never another scenario's take.
4. **RobotHost's `ScenarioRecorder`** (`--scenarios <dir>`, which needs `--auto-record`).
   - **One take at a time.** While it records, the automatic recorder is held off: an open automatic session is closed
     first, and none opens until the usual idle gap after the take.
   - **A take stops by itself** when:
     - the headset that started it is silent for 10 s;
     - it reaches 30 min;
     - the disk falls below 21 GB free.
   - **A take starts** only with 25 GB free. The renderer stops writing frames below 20 GB.
   - **Discard** moves the last take this RobotHost recorded into `<scenarios>/_discarded/<scenario>/`, once it is
     complete. Nothing is deleted.
5. **On the headset,** four bench buttons on the left wing, behind the power row, work whether the robot is on or off:
   - **`< PREV`** and **`NEXT >`** choose the scenario. NEXT's label names it, with its place in the list.
   - **RECORD** starts a take. While recording it reads `STOP ● mm:ss` with the scenario and take number, and pressing it
     saves the take.
   - **DISCARD LAST** sets the last take aside.
   - Every label carries RobotHost's last answer: saved, set aside, low disk, the list changed, and so on.
6. **The shared folder.** `just isaac-host` passes `--scenarios ~/isaacsim/scenarios`. `just isaac-share-scenarios`
   (default user `seaqueue-cheng`) sets ACLs once, with no sudo:
   - The labmate can enter our home and `~/isaacsim` but not list them.
   - He can read everything under the scenarios folder, and every take written there later (a default ACL).
   - He can edit `scenarios.txt`.
   - Takes stay read-only to him.

   `just isaac-scenarios` lists what has been recorded.

## Consequences

- **The takes are training data with a label** saying what the operator meant to do. That is useful for splits and for
  later task-conditioned models. The label is metadata: `conditions` are never a model input (SESSION_FORMAT).
- **Two kinds of headset session now land in two folders:** automatic ones in `recordings/`, takes in `scenarios/`.
  `armpred check-session` reads both: the takes add new values of existing fields, and one new object.
- **The labmate needs nothing but the headset and a file manager.** He adds a scenario by editing a text file.
- **What is still not tested on the headset:** whether the buttons' place on the left wing is comfortable, and whether
  the labels read well.

## Alternatives considered

- **A desktop app on the GPU machine:** the user preferred the headset. The person recording is the person operating.
- **Copying each finished take into his home:** it stores the data twice, and he would have had to make his folder
  writable for us.
- **An export recipe:** a manual step for someone who does not use the repo.
- **Scenarios in the headset's config** (`fr3_connection.json`): changing the list would then need adb and the repo. A
  file on the GPU machine is his to edit.

## Amendment 1 (2026-11-12): RECORD on the real JetRover; disk thresholds as options

ADR 0036 wants the real robot's takes too.

1. **`deploy-robothost` passes `--scenarios /home/jetson/recordings/scenarios`.** The headset's buttons already work
   with the JetRover (`OperatorSideControls`): with this RobotHost they answer instead of reading "RobotHost records no
   scenarios". The labmate's list is `scenarios.txt` in that folder; `just robot-scenarios` lists the takes.
2. **The disk thresholds become options:** `--scenario-start-free-gb` and `--scenario-stop-free-gb`.
   - The defaults stay 25 and 21 GB, which suit the GPU machine.
   - The Jetson's disk is smaller, so `deploy-robothost` passes thresholds sized to it. A take with the camera costs
     about 4.5 GB an hour (ADR 0014 amendment 3).
3. **Real takes reach the GPU machine.** ADR 0036's watcher copies complete ones there, and that machine is where they
   are converted and shared.
