# Operating notes for agents

How work gets done in this repo day to day, learned the hard way (mostly November 2026). Root `CLAUDE.md` is binding and
comes first. This page adds the practice around it: where to work, how to check work, how to treat the GPU machine, the
twins, the robot and the user's own changes. Every agent in `.claude/agents/` reads it before starting.

## 1. Where to work

- **Work in your own clone on WSL's own disk, not on `/mnt/c`.**
  - WSL's view of the Windows drive (`/mnt/c`, 9p drvfs) drops with "Input/output error" at random (§7). Anything working
    there stalls, and a subagent whose working directory is there dies.
  - Make a clone at `~/work/<task>` and keep every command inside it (`cd ~/work/<task> && ...`):
    ```
    GIT_LFS_SKIP_SMUDGE=1 gh repo clone andreitimoshukcollege-del/teleoperation ~/work/<task> -- --branch <base> --quiet
    git -C ~/work/<task> checkout -b work/<task>
    ```
    or clone from an existing clone (`git clone ~/work/<other> ~/work/<task>`). `GIT_LFS_SKIP_SMUDGE=1` leaves the Unity
    LFS files as pointers, which nothing outside Unity needs.
- **The user's checkout is `C:\Users\andre\Projects\teleoperation`** (`/mnt/c/...` from WSL), where Unity and the headset
  builds run. It holds the user's own uncommitted changes:
  - `justfile` hunks: `mirror-galaxy`, `gpu-mount`/`gpu-unmount`, Galaxy help lines;
  - `.claude/settings.json`;
  - TeleopXR scripts and materials.
  - **Never stage, commit, revert or reformat them.** To commit your own change to a file that also holds theirs, stage
    only your hunks: write HEAD's version plus your edit to a temp file, `git hash-object -w`, then
    `git update-index --cacheinfo`.
- **Updating that checkout** to a newer branch head when a file the user changed also changes upstream:
  1. back up `git diff` to `~/work/tools/`;
  2. `git checkout -- <file>`;
  3. `git reset --keep <new head>` (a fast-forward that refuses to touch local changes; never `--hard`);
  4. `git apply --3way <their patch>`;
  5. `git restore --staged <file>`, because `--3way` stages what it applies;
  6. check that their diff is byte-for-byte what it was.
- **Commit after every meaningful step**, with explicit paths. End each message with
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
  - Never `--amend`, rebase, `reset --hard` or force-push.
  - Push only when the user asks, or when an approved plan includes the push. The twins run what GitHub has, so a twin
    check needs the branch on GitHub.
- **Never touch:**
  - `results/` (append-only), `unity/*/Library/`, `build/`;
  - `core/testdata/` (edits there are denied: if a golden there would change, stop and report).
- **Deleting:** never `rm -rf`. Use `find <path> -delete`, and only for files you made.

## 2. Checking work

- **The three Core gates** (`dotnet test`, `verify`, `audit`, i.e. `just core-check`) plus `just bridge-check` when a
  Core/RobotArm signature changes. The Python suites: `isaac`, `lerobot`, and `gpu` (`robot/gpu`). Report verbatim
  pass/fail lines.
- **From a WSL clone,** run them on the GPU machine with `scripts/gpu/gate.sh <gate>...`. It pushes your committed branch to
  a scratch repo there, so commit first. There is no local dotnet or numpy in WSL, and CLAUDE.md forbids installing the
  Linux .NET SDK on this machine.
  - `core`/`test` cost about 10 minutes of the machine's CPU. If the user has said the machine is busy, run only the light
    gates (`isaac`, `lerobot`, `gpu`, `test=<filter>`), or ask.
  - The alternative is a git worktree of the C: checkout with the Windows dotnet (`just core-check`); it is exposed to §7.
- **Unity:** `just unity-check-xr` and `just build-galaxy-sim`/`build-galaxy` run on the laptop, with that project's editor
  closed. A change under `unity/Teleop.Bridge/` wants human review even when it compiles.

## 3. The GPU machine (RTX 5090)

- **Access:** `ssh andrei@10.188.58.133`, over the lab VPN, with key login. The address comes from DHCP; recipes take
  `GPU_HOST=user@addr`. `~/.ssh/config` shares one connection for all ssh calls to it (`ControlMaster`). Its sshd drops new
  connections when more than ~10 are being opened at once, which once stopped a batch. Keep it that way, and don't open
  bursts of parallel ssh calls.
- **It is shared.** It is the labmate's desktop and runs his work. Before anything heavy (a batch, a dataset build, Isaac,
  `dotnet test`, video encoding of many takes), check `nvidia-smi` and the load (`uptime`), and follow the user's latest word.
  "Don't run anything intensive" means only light reads, pytest and single short ffmpeg jobs.
- **Layout:** `docs/gpu-machine.md`.
  - `~/data/sessions/`, `~/data/takes/`, `~/data/lerobot/`;
  - `~/repos/isaac` (the twins' checkout), `~/repos/safety`;
  - `~/logs/isaac/`, `~/isaacsim/` (venvs, assets).
  - Your scratch is `/tmp/agent-<name>/` and `~/scratch/agent-<name>/`.
  - Disk was 234 GB free on 2026-11-22. A twin records about 10 GB an hour.
- **Never delete takes, sessions or datasets** unless the user explicitly asks for exactly that set. List first, then confirm.
- **Data is stored on the Explorer cluster** (docs/explorer-cluster.md "Data from the GPU machine", the user, 2026-11-25):
  `/projects/world-models/<user>/teleop-data`.
  - The GPU machine's `~/data` is the working set. Archive with `just cluster-archive`; safety batches do it themselves.
  - Free space with `just cluster-prune <days>`: the dry run first, and the list goes to the user before `true`.
- **The labmate** (account `seaqueue-cheng`):
  - He reads our takes and datasets through ACLs (`just isaac-share-scenarios`). A named ACL entry overrides a folder's
    "other" bits: give `r-x`, not `--x`, where he must list.
  - **Don't touch anything in his home or folders** (the user, 2026-11-25), including the sim-data folder our account
    can write to, unless the user explicitly asks for that delivery. He reads our data from the cluster archive.
  - Keep his and the user's personal details out of anything sent outside.
- **Copying files:** `scp` is denied. Fetch a small result with `ssh host 'cat <file>' > <local>`, as the take tools do.

- **Northeastern's Explorer cluster** (H200s, for trainer grids): `ssh explorer`, with the key RC issued (own keys are
  refused there). Use the `just explorer-*` recipes. Never run work on its login nodes, and keep runs out of `/scratch`,
  which RC empties every month. Access, limits and layout: `docs/explorer-cluster.md`.

## 4. The twins

- **Recipes** (run from a clone whose HEAD is on GitHub; they check):
  - `just isaac-twins-up 2` / `just isaac-twins-down`.
  - `just isaac-jetrover-host jetrover-bench auto <twin> [true=detached]`.
  - `just isaac-stop <twin|all>`, `just isaac-status`.
  - `just isaac-safety-batch <takes> <seed> <categories> <build> <events> <twin>`, where `takes` is rounds of the four
    categories.
  - `just isaac-safety-batch-twins <takes> <seed>`: one batch per twin (seeds `seed` and `seed+1000`), then one dataset
    per twin (`<name>_twin0`, `<name>_twin1`).
  - `just isaac-safety-build <twins> [name] [out]`: one dataset per twin from the batches' `.kept` lists. Two twins record
    at once on one clock, so one dataset of both would hold takes that overlap in time, and the builder refuses that
    (docs/adr/0038 L). `just isaac-safety-rebuild <dataset>` rebuilds one built that way before 2026-11-24.
  - Assets: `isaac-materials-fetch`/`status`, `isaac-models-fetch <scene>`.
- **Twin 1** uses ports +100 (6100–6109), `-1` sockets and logs, `~/data/sessions/jetrover-isaac-twin1` and
  `~/data/takes/jetrover-isaac-twin1`. Both twins run one commit from `~/repos/isaac`, and a twin refuses to start at
  another commit while the other runs.
- **Ready means fresh.** `renderer.log` keeps the previous run's lines until the new renderer truncates it. "Ready" means
  all of: the renderer process exists, the bridge log has `[jetrover] contacts:`, and `just isaac-status` shows that twin's
  `[render] ~30 fps`. A cold first start can take 3+ minutes.
- **The dashboard** (`just gpu-ui`) reads a batch's progress from `<repo>/recordings/.drive/` of the clone the batch was
  started from. Start it from that clone (`--repo`), or the user sees nothing.
- **Seeds:** take n's seed is `seed·100000+n`. To resume a stopped batch, use a fresh seed (e.g. +1000) and save its
  `.kept` list first, or takes repeat and lists are overwritten.
- **Throughput** (2026-11-22, two twins): about 115–127 takes an hour each, about 240 together. The dataset build is serial,
  about 15 s an episode.
- **Before trusting a batch,** run a short check (`takes=4`, `build=false`) on a new commit, then
  `scripts/gpu/take_report.py` and `take_timing.py`, and show the user a contact sheet and one video.

## 5. Takes, records, datasets

- **A take:**
  - `~/data/takes/<root>/<category>/<UTC>-take<n>/`, or `_discarded/<category>/` when the checker set it aside;
  - holds `robothost.jlog`, `isaac.jlog`, `isaac/<camera>.<rgb|depth|mask>.frames`, `scene.json` and `safety-check.json`;
  - is complete once `files.sha256` exists.
- **Records** (`trainer/contract/SESSION_FORMAT.md` is the authority):
  - in `robothost.jlog`:
    - `WC` contacts (`arm:link:object:forceN`), `WO` object poses (`id:x:y:z:qw:qx:qy:qz`), `WL` layout (`ids`),
      `WP` placement outcomes;
    - `SE` planned events (times are seconds after its `t`), `RO`/`RI`/`JC` joints and commands;
  - in `isaac.jlog`: `VF` frames (with `stateSensed`), `VQ` joint values per frame, `LK` looks, `CL` flat clutter,
    `EN` room, `MT` materials.
- **Categories and the checker** (`robot/lerobot/teleop_lerobot/safety.py`):
  - reach and stop: no contact;
  - recover: every touch backs off, nothing tips, not touching at the end;
  - knock: at least one object tipped or moved.
  - A touch is a contact point within 1 mm, or an impulse (ADR 0036 amendment 4).
  - Takes draw 1–3 events anywhere in the take (amendment 5).
- **Datasets:**
  - `~/data/lerobot/jetrover-isaac-safety/jetrover_<UTC>_<id>/`, with a `.build-N.log` beside it;
  - in the labmate's format: `manual_safety_labels.json` (ranges per episode), `meta/safety_labels.parquet`,
    `safety_events.jsonl`; `observation.images.front` is the wrist camera.

## 6. Scenes, goldens, the headset

- **The scene and its copy.** `core/Scenes/jetrover-bench.json` and `unity/Teleop.Bridge/Runtime/Resources/jetrover_scene.json`
  must stay byte-identical (`cmp`).
  - Shapes: `core/Scenes/jetrover-bench.shapes.json` (`robot/isaac/shape_survey.py`, one entry per mesh asset).
  - Shared reader vectors: `core/Scenes/testdata/vectors.json`, hand-kept with indent 1, read by both the C# and the
    Python reader.
  - `WorldScene.MaxObjects` and `scene.py` `MAX_OBJECTS` are 32.
- **The rests golden** `robot/isaac/golden/scene-rests-jetrover-bench.json` pins the scene's sha256. After any scene edit,
  regenerate it with `just gen-isaac-goldens`, or remotely:
  1. run `TELEOP_WRITE_GOLDEN=1 dotnet test core/Teleop.RobotArm.Tests --filter "FullyQualifiedName~SceneRestTests.TheRestsGoldenFile"`
     in your gate scratch checkout;
  2. bring the change back as `git diff --binary` over ssh;
  3. `git apply` it.
  Regenerate another golden only when you know why it changed.
- **The headset after a scene change:** `just unity-world-jetrover-xr`, then `just unity-check-xr`, then
  `just build-galaxy-sim` (and `build-galaxy` for the robot app). Leave the generated Unity changes for the user to review.

## 7. When `/mnt/c` drops

- **Symptom:** every path under `/mnt/c` returns "Input/output error", and every `just` recipe run from the C: checkout fails.
- **Cause:** a race in Windows' own Plan 9 server (`vp9fs.dll`, microsoft/WSL#41702). It triggers when a process is killed
  with file requests open on `/mnt/c`, which tool timeouts do. Avoid wrapping `/mnt/c` work in `timeout`.
- **Recovery:** the `mnt-c-watchdog` service remounts within about 5 s (`journalctl -u mnt-c-watchdog`). A shell sitting in
  the old mount needs `cd "$PWD"`. If the watchdog is not installed, tell the user to run
  `sudo umount -l /mnt/c && sudo mount -t drvfs C: /mnt/c -o uid=1000,gid=1000`, or `wsl --shutdown`.

## 8. The real JetRover

- **Only through `just` recipes,** never raw ssh to the robot for anything that changes state. Read-only diagnostics are
  fine: `robot-status`, `robot-net-status`, `robot-camera-check`, `camera-logs`.
- **`just` takes recipe arguments by position:** `just robot-camera-check /dev/video1`, not `device=/dev/video1`.
- **The robot's address** comes from `just _robot-host` (mDNS through Windows' resolver, then Tailscale), and it moves. Never
  hard-code it.
- **Nothing moves without the user's confirmation.** That covers the arm, the servo driver and deploys: say what will move
  and how far, and wait.
- **Never publish `/robot/buzzer`:** it crashes the arm driver.
- **The robot's cameras** (docs/adr/0014 amendment 4).
  - **Placement.** Since 2026-11-24 they are placed for data collection. The exception is the DaBai (`front`), which is
    temporarily in a third-person spot.
  - **The Femto Bolts.** USB `2bc5:066b`: port 1.4.3 is `left` (6008) and 1.4.2 is `right` (6009). They stream colour
    through V4L2. Depth, IR and the sync hub need Orbbec's SDK: `just femto-record`, untested.
  - **Calibration.** `just camera-calib-record` records all three feeds for calibration, and
    `scripts/gpu/charuco_calibrate.py` calibrates them. The calibration is valid only until a camera moves.

## 9. Licences, privacy, outside requests

- **Assets:** CC0 or CC-BY only, recorded in the manifests. Poly Haven is CC0; YCB is CC BY 4.0. Not ABO, HOPE or
  BEHAVIOR-1K.
- **Outside requests:** send User-Agent `teleop-research-renderer/1.0` and no personal details. `curl` is denied; use
  WebFetch or Python.

## 10. Reporting

- **Measure, don't assert.** Give before/after numbers, counts out of totals, and which commit ran.
- **Say what was not checked,** especially anything not yet run on the twin.
- **Design changes** go in an ADR amendment with the measured figures, the way docs/adr/0036 and 0037 do it.
- **Show the user** a contact sheet and a video in `/mnt/c/Users/andre/Downloads/` for anything visual.
