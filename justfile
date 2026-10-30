# Teleop Research Platform -- convenience commands.
# Install `just`: https://github.com/casey/just
# `just` with no arguments lists everything below.

default:
    @just --list

# ---- core/ (dotnet: algorithms, unit tests, sweep) ----

# Unit + allocation tests for Teleop.Core
[group('checks')]
core-test:
    cd core && dotnet test

# Replay the golden .tlog twice, assert identical (determinism gate)
[group('checks')]
core-verify:
    cd core && dotnet run --project Teleop.Eval -- verify

# Invariant + registry-completeness check over the built assembly
[group('checks')]
core-audit:
    cd core && dotnet run --project Teleop.Eval -- audit

# Run all three core/ gates -- "Verify your work" in root CLAUDE.md
[group('checks')]
core-check: core-test core-verify core-audit

# Compile Bridge/ against Core headlessly -- catches a Core API change silently breaking unity/. NOT a substitute for opening the editor; see unity/BridgeCheck/README.md
[group('checks')]
bridge-check:
    cd unity/BridgeCheck && dotnet build --nologo

# Copy core/testdata/traces/*.trace where Unity's Editor can read them (Application.persistentDataPath), for NetworkImpairmentController's trace mode. On device, `adb push` them to the same place instead.
[group('experiments')]
install-traces:
    #!/usr/bin/env bash
    set -euo pipefail
    # Deliberately a copy at the moment of use rather than a tracked duplicate under unity/:
    # core/testdata/traces/ is the one source, and a second committed copy would drift from it
    # silently. Re-run this after regenerating a trace with `Teleop.Eval gen-trace`.
    #
    # Editor persistentDataPath on Windows is %USERPROFILE%\AppData\LocalLow\<company>\<product>,
    # read here from ProjectSettings.asset rather than hardcoded so renaming the product in Unity
    # doesn't leave this recipe silently writing to a directory nothing reads.
    settings=unity/TeleopVR/ProjectSettings/ProjectSettings.asset
    company=$(grep -m1 'companyName:' "$settings" | sed 's/.*companyName: *//' | tr -d '\r')
    product=$(grep -m1 'productName:' "$settings" | sed 's/.*productName: *//' | tr -d '\r')
    dest="/mnt/c/Users/$(cmd.exe /c 'echo %USERNAME%' 2>/dev/null | tr -d '\r\n')/AppData/LocalLow/${company}/${product}"
    if [ ! -d "$dest" ]; then
        echo "Creating $dest (Unity has not run yet, or the product name changed)" >&2
        mkdir -p "$dest"
    fi
    count=0
    for f in core/testdata/traces/*.trace; do
        [ -e "$f" ] || { echo "No .trace files in core/testdata/traces/" >&2; exit 1; }
        cp "$f" "$dest/"
        echo "installed $(basename "$f") -> $dest"
        count=$((count + 1))
    done
    echo "$count trace(s) installed."

# ---- unity/ headless runs (docs/adr/0016: TeleopVR = Quest on 2022.3, TeleopXR = Galaxy XR on 6.6) ----

# Internal: run one Unity project's editor headless with -executeMethod; picks the editor version the project pins, refuses while that project is open in an editor, prints the log's verdict lines. graphics=true keeps the GPU (baking, screenshots)
_unity-batch project method extra="" graphics="false":
    #!/usr/bin/env bash
    set -euo pipefail
    proj="unity/{{project}}"
    ver=$(sed -n 's/^m_EditorVersion: //p' "$proj/ProjectSettings/ProjectVersion.txt" | tr -d '\r')
    exe="/mnt/c/Program Files/Unity/Hub/Editor/$ver/Editor/Unity.exe"
    if [ ! -x "$exe" ]; then
        echo "error: $proj pins Unity $ver, which is not installed ($exe). Install it from Unity Hub." >&2
        exit 1
    fi
    winproj=$(wslpath -w "$proj")
    # An open editor holds the project lock and batch mode would fail on it, so check first. Match
    # on the command line: the asset-import workers of *another* project must not block this one.
    open=$(powershell.exe -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"name='Unity.exe'\" | Where-Object { \$_.CommandLine -like '*{{project}}*' -and \$_.CommandLine -notlike '*AssetImportWorker*' } | Measure-Object | Select-Object -ExpandProperty Count" | tr -d '\r')
    if [ "${open:-0}" != "0" ]; then
        echo "error: a Unity editor has $proj open. Close it first: batch mode needs the project lock." >&2
        exit 1
    fi
    tmp=$(wslpath -u "$(cmd.exe /c 'echo %TEMP%' 2>/dev/null | tr -d '\r\n')")/teleop-unity
    mkdir -p "$tmp"
    log="$tmp/{{project}}-$(date +%Y%m%d-%H%M%S).log"
    echo "Unity $ver, headless: {{project}} -> {{method}} (log: $log)" >&2
    set +e
    "$exe" -batchmode {{ if graphics == "true" { "" } else { "-nographics" } }} -quit -projectPath "$winproj" -executeMethod {{method}} \
        -logFile "$(wslpath -w "$log")" {{extra}}
    code=$?
    set -e
    grep -a -E '^\[(scene-check|xr-setup|xr-scene|build|lab)\]|error CS[0-9]+' "$log" | sed 's/\r$//' | tail -40 || true
    if [ "$code" -ne 0 ]; then
        echo "Unity exited $code. Full log: $log" >&2
    fi
    exit "$code"

# Headless check of unity/TeleopVR (Unity 2022.3): compiles, and every JetRover scene opens with no missing scripts and its Bridge wiring intact -- moves nothing; close that project's editor first
[group('quest (parked)')]
unity-check-vr:
    just _unity-batch TeleopVR Teleop.Editor.SceneIntegrityCheck.Run

# Configure unity/TeleopXR (Galaxy XR, Unity 6.6) from code: XRI/XR Hands samples, URP, Android player settings, OpenXR for Android and Direct Preview. Idempotent; the first run imports samples, so run it twice on a fresh clone. Close that project's editor first
[group('headset (Galaxy XR)')]
unity-setup-xr:
    just _unity-batch TeleopXR Teleop.XR.Editor.XrProjectSetup.Run

# Build unity/TeleopXR's Galaxy XR scene (JetRoverLabXR) from code: hands rig, arm, pinch-grab target, camera panel. Create-only; rebuild=true replaces it, discarding hand edits. Run unity-setup-xr first
[group('headset (Galaxy XR)')]
unity-scene-xr rebuild="false":
    just _unity-batch TeleopXR Teleop.XR.Editor.XrLabSceneBuilder.Run {{ if rebuild == "true" { "-teleopRebuildScene" } else { "" } }}

# Build the arm's collision model from the robot's CAD (docs/adr/0020) and write core/Teleop.RobotArm/Collision/JetRoverCollisionModel.g.cs: fits capsules to each link and boxes to the body, decides which joint-local pairs to skip, and validates (every surface sample covered, Core's frames match the Unity rig) before writing anything. Report and preview images go to %TEMP%\teleop-collision. Rerun whenever the CAD changes. Needs the lab built with the CAD; close TeleopXR's editor first
[group('headset (Galaxy XR)')]
robot-collision-model:
    just _unity-batch TeleopXR Teleop.XR.Editor.RobotCollisionExtractor.Run "" true

# Rebuild unity/TeleopXR's lab (night-shift room, JetRover twin, camera display, lighting), bake the lighting, and render preview screenshots to the Windows temp folder; prints the contact sheet's path. bake=false skips the multi-minute bake. Close that project's editor first
[group('headset (Galaxy XR)')]
unity-lab-xr bake="true":
    just _unity-batch TeleopXR Teleop.XR.Editor.LabDressing.Run "-labCapture {{ if bake == "true" { "-labBake" } else { "" } }}" true

# Headless check of unity/TeleopXR's scene: no missing scripts, Bridge wiring intact, pinch-grab settings safe for a real arm (no pull-to-hand, no snap, no smoothing, no throw), hands only
[group('headset (Galaxy XR)')]
unity-check-xr:
    just _unity-batch TeleopXR Teleop.XR.Editor.XrSceneCheck.Run

# Build the Galaxy XR APK (unity/TeleopXR, Unity 6.6) after its scene check -> unity/TeleopXR/Builds/TeleopXR.apk. Close that project's editor first
[group('headset (Galaxy XR)')]
build-galaxy:
    just _unity-batch TeleopXR Teleop.XR.Editor.XrBuild.Run "-buildTarget Android"

# Build the Quest APK (unity/TeleopVR, Unity 2022.3) from its Build Settings scenes after its scene check -> unity/TeleopVR/Builds/TeleopVR-Quest.apk. The first Android build re-imports assets; close that project's editor first
[group('quest (parked)')]
build-quest:
    just _unity-batch TeleopVR Teleop.Editor.QuestBuild.Run "-buildTarget Android"

# Internal: the adb that ships with the Unity editor unity/TeleopXR pins (or $ADB), run on the Windows side. Arguments are re-split by the shell, so pass Windows paths as `wslpath -m` (forward slashes): backslashes would be eaten
_adb *args:
    #!/usr/bin/env bash
    set -euo pipefail
    ver=$(sed -n 's/^m_EditorVersion: //p' unity/TeleopXR/ProjectSettings/ProjectVersion.txt | tr -d '\r')
    adb="${ADB:-/mnt/c/Program Files/Unity/Hub/Editor/$ver/Editor/Data/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb.exe}"
    [ -x "$adb" ] || { echo "error: adb not found at $adb (install Unity $ver's Android SDK module, or set ADB)" >&2; exit 1; }
    "$adb" {{args}}

# Mirror the Galaxy XR's view into a window on this PC over USB, with scrcpy (install it once: `winget install Genymobile.scrcpy`). Uses Unity's adb, as every recipe here does, so two adb versions never fight over the server. Extra flags pass through, e.g. `just mirror-galaxy --record=session.mp4`. Close the window to stop
[group('headset (Galaxy XR)')]
mirror-galaxy *args:
    #!/usr/bin/env bash
    set -euo pipefail
    ver=$(sed -n 's/^m_EditorVersion: //p' unity/TeleopXR/ProjectSettings/ProjectVersion.txt | tr -d '\r')
    adb="${ADB:-/mnt/c/Program Files/Unity/Hub/Editor/$ver/Editor/Data/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb.exe}"
    [ -x "$adb" ] || { echo "error: adb not found at $adb (install Unity $ver's Android SDK module, or set ADB)" >&2; exit 1; }
    scrcpy=$(ls -d /mnt/c/Users/*/AppData/Local/Microsoft/WinGet/Packages/Genymobile.scrcpy_*/scrcpy-win64-*/scrcpy.exe 2>/dev/null | sort -V | tail -1 || true)
    [ -n "$scrcpy" ] || { echo "error: scrcpy not found; install it with: winget install Genymobile.scrcpy" >&2; exit 1; }
    # ADB reaches the Windows process through WSLENV, as the Windows path scrcpy expects.
    ADB="$(wslpath -w "$adb")" WSLENV="${WSLENV:+$WSLENV:}ADB" "$scrcpy" --no-audio --max-size 1920 --window-title=GalaxyXR {{args}}

# Install the Galaxy XR APK on the headset over USB (developer mode and USB debugging on; accept the prompt in the headset)
[group('headset (Galaxy XR)')]
install-galaxy:
    #!/usr/bin/env bash
    set -euo pipefail
    apk=unity/TeleopXR/Builds/TeleopXR.apk
    [ -f "$apk" ] || { echo "error: $apk not found; run just build-galaxy first" >&2; exit 1; }
    just _adb devices -l
    just _adb install -r "$(wslpath -m "$apk")"

# Point the Galaxy XR app at the robot without a rebuild: pushes a full jetrover_connection.json override (the Bridge default with RemoteHost replaced) to the app's files directory. Finds the robot itself (`just robot-ip`); pass an address to override. motion=false sends nothing to the arm (ConfirmHardwareMotion off): only the virtual arm moves, for a first try of the controls
[group('headset (Galaxy XR)')]
push-galaxy-config host=default_jetrover_host motion="true":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{motion}}" in true|false) ;; *) echo "error: motion must be true or false, got '{{motion}}'" >&2; exit 1 ;; esac
    host=$(just _robot-host "{{host}}")
    # The headset has no Tailscale. When this PC can only reach the robot through it (it is on
    # another network), ask the robot for its own lab-network address and give the headset that.
    case "$host" in 100.*)
        lan=$(ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 "jetson@$host"             "ip -4 -o addr show scope global | grep -v -E ' (tailscale0|docker0|lo) ' | awk '{print \$4}' | cut -d/ -f1 | head -1" || true)
        if [ -z "$lan" ]; then
            echo "error: the robot is only reachable here over Tailscale ($host), which the headset cannot use, and it reported no LAN address. Pass the robot's LAN address: just push-galaxy-config <address>" >&2
            exit 1
        fi
        echo "robot: $host is its Tailscale address; giving the headset its LAN address, $lan, instead" >&2
        host=$lan ;;
    esac
    tmp=$(mktemp --suffix=.json)
    trap 'rm -f "$tmp"' EXIT
    python3 - "$tmp" "$host" "{{motion}}" <<'PY'
    import json, sys
    cfg = json.load(open("unity/Teleop.Bridge/Runtime/Resources/jetrover_connection.json", encoding="utf-8"))
    cfg["RemoteHost"] = sys.argv[2]
    cfg["ConfirmHardwareMotion"] = sys.argv[3] == "true"
    json.dump(cfg, open(sys.argv[1], "w"), indent=2)
    PY
    dest=/sdcard/Android/data/com.teleop.jetroverxr/files
    just _adb shell mkdir -p "$dest"
    just _adb push "$(wslpath -m "$tmp")" "$dest/jetrover_connection.json"
    echo "Galaxy XR app now targets $host, hardware motion {{ if motion == "true" { "ON" } else { "OFF (virtual arm only)" } }}; restart the app to pick it up."

# Put a learned joint model on the Galaxy XR without a rebuild (docs/adr/0025): exports trainer/runs/<run>'s model.onnx as weights (checked against it on the golden's real windows) and pushes them to the app's files/models/<run>.weights, which the `learned` preset named <run> loads in place of the build's Resources copy. Point a preset at it with "Model": "<run>" (just push-galaxy-config pushes the presets); restart the app
[group('headset (Galaxy XR)')]
push-galaxy-model run:
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{run}}" in *[!A-Za-z0-9._-]*|"") echo "error: run is a trainer/runs name" >&2; exit 1 ;; esac
    [ -f "trainer/runs/{{run}}/model.onnx" ] || { echo "error: no trainer/runs/{{run}}/model.onnx (just gpu-fetch it)" >&2; exit 1; }
    mkdir -p dist
    ( cd trainer && .venv-win-cuda/Scripts/armpred.exe export-weights --run "runs/{{run}}" --golden ../core/testdata/learned/golden-v3.json --out "../dist/{{run}}.weights" )
    dest=/sdcard/Android/data/com.teleop.jetroverxr/files/models
    just _adb shell mkdir -p "$dest"
    just _adb push "dist/{{run}}.weights" "$dest/{{run}}.weights"
    echo "the headset has model {{run}}; a preset with \"Model\": \"{{run}}\" runs it after the app restarts"

# Run an experiment sweep, e.g. `just sweep experiments/exp-001-predictor-baseline.yaml`
[group('experiments')]
sweep config:
    cd core && dotnet run --project Teleop.Eval -- sweep ../{{config}}

# ---- JetRover hardware (a human must be watching anything that moves the arm) ----

# Where the JetRover and this operator machine are reachable from here.
# The robot defaults to "auto": every recipe that talks to it looks it up when it runs (`just
# robot-ip`), because its DHCP address can change. Export JETROVER_HOST to pin an address instead
# (`just` parameters are positional, so `remote_host=` on the command line does not work).
# OPERATOR_HOST is where RobotHost replies before its first command arrives; after that it replies
# to the command's source (#57).
# The ssh/scp recipes accept IPv6 literals, but WSL's default NAT networking has no IPv6 route and
# the UDP transports in Teleop.RobotHost and Teleop.Eval are IPv4-only, so use IPv4 for now.
default_jetrover_host := env_var_or_default("JETROVER_HOST", "auto")
default_operator_host := env_var_or_default("OPERATOR_HOST", "100.82.140.80")
# The robot's hostname, which it announces over mDNS as <name>.local (`just robot-set-hostname`).
robot_mdns_name := env_var_or_default("JETROVER_NAME", "jetrover-sinrg")
# The GPU machine for trainer runs (just gpu-train, gpu-status, gpu-fetch): an account with this repo's SSH key.
gpu_host := env_var_or_default("GPU_HOST", "andrei@10.188.58.164")
# Where ROBOT OFF parks the arm before its torque goes off (docs/adr/0023 decision 5), passed by
# deploy-robothost as --rest-pose-deg: degrees, motors 1-5 (base yaw, shoulder, elbow, wrist pitch,
# wrist roll; the gripper is left as it is), in RobotHost's joint angles as `just joint-probe` prints
# them (the profile's zero offsets applied). Placed by the user and measured with joint-probe on
# 2026-10-26 (the second placing): -1.20, 91.16, -78.48, -114.00, 5.76, with the wrist pulled in to
# -112.0, 0.7 deg inside its measured range (-112.7 to 117.0, docs/adr/0020); at -114 every relax would
# be refused as outside it. To re-measure (robot/README.md, "Park, then relax"): place the arm by hand on its
# support, or drive it there slowly with the headset; run `just joint-probe` with the headset's motion
# off and copy the five sensed angles here (or export JETROVER_REST_POSE_DEG), then
# `just deploy-robothost`. Empty refuses every relax.
# RobotHost's own collision check still decides each park, and a pose outside a joint's range is
# refused. This pose is clear by the model and parks from home with the strict check, so
# JETROVER_PARK_CONTACT_DEG stays empty. That option is for a rest pose that lies on the body (the
# first 2026-10-26 pose had the gripper on Body.7): it allows that contact within so many degrees of
# the rest pose and nothing else.
jetrover_rest_pose_deg := env_var_or_default("JETROVER_REST_POSE_DEG", "-1.20,91.16,-78.48,-112.0,5.76")
jetrover_park_contact_deg := env_var_or_default("JETROVER_PARK_CONTACT_DEG", "")

# Print the robot's current IPv4 address and how it was found -- moves nothing. Tries its mDNS name (jetrover-sinrg.local, on the lab LAN), then Tailscale (the direct LAN path if it has one, else the 100.x address). Every robot recipe does this lookup itself unless JETROVER_HOST is set
[group('robot: checks and diagnostics (nothing moves)')]
robot-ip:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -n "${JETROVER_HOST:-}" ]; then
        echo "note: JETROVER_HOST=$JETROVER_HOST is set, so recipes use that instead of the address below" >&2
    fi
    just _robot-host auto

# Find the robot as the Galaxy XR headset does at start (docs/adr/0022): ask through its Tailscale address for its LAN addresses and try those, else jetrover-sinrg.local over mDNS -- sends only discovery and mDNS queries, moves nothing. Says which address the headset would use and why. Needs RobotHost deployed with --discovery-port (just deploy-robothost); tailscale=none skips Tailscale, as on a headset without it
[group('robot: checks and diagnostics (nothing moves)')]
robot-discover tailscale="100.112.90.72" configured="":
    cd core && dotnet run --project Teleop.Eval -- discover-robot --tailscale {{tailscale}} {{ if configured != "" { "--configured " + configured } else { "" } }}

# Internal: resolve a robot address. "auto" looks the robot up; anything else is printed back unchanged
_robot-host host="auto":
    #!/usr/bin/env bash
    set -euo pipefail
    if [ "{{host}}" != "auto" ]; then echo "{{host}}"; exit 0; fi
    name="{{robot_mdns_name}}"
    # The robot's own Docker bridge (172.17.x) and link-local addresses are never the way to reach it.
    first_usable() { while read -r a; do case "$a" in 127.*|169.254.*|172.17.*|"") ;; *.*.*.*) echo "$a"; return ;; esac; done; }
    is_private() { case "$1" in 10.*|192.168.*|172.1[6-9].*|172.2[0-9].*|172.3[01].*) return 0 ;; *) return 1 ;; esac; }

    # 1. mDNS, through the OS resolver. Under WSL that has to be Windows' resolver: multicast does not
    #    cross WSL's NAT, so Linux-side lookups of .local names never get an answer.
    ip=""
    if command -v powershell.exe >/dev/null 2>&1; then
        ip=$(powershell.exe -NoProfile -Command "try { [System.Net.Dns]::GetHostAddresses('$name.local') | Where-Object { \$_.AddressFamily -eq 'InterNetwork' } | ForEach-Object { \$_.IPAddressToString } } catch { }" 2>/dev/null | tr -d '\r' | first_usable || true)
    elif command -v getent >/dev/null 2>&1; then
        ip=$(getent ahostsv4 "$name.local" 2>/dev/null | awk '{print $1}' | first_usable || true)
    fi
    if [ -n "$ip" ]; then
        echo "robot: $ip (mDNS $name.local)" >&2
        echo "$ip"
        exit 0
    fi

    # 2. Tailscale. A ping makes it establish (and report) its best path: "via <lan-ip>:<port>" when
    #    the two machines can reach each other directly, "via DERP(...)" when only a relay works.
    ts=$(command -v tailscale.exe || command -v tailscale || true)
    if [ -n "$ts" ]; then
        pong=$("$ts" ping -c 1 --timeout 3s "$name" 2>/dev/null | tr -d '\r' | grep -m1 '^pong' || true)
        tsip=$(printf '%s' "$pong" | sed -n 's/^pong from [^ ]* (\([0-9.]*\)).*/\1/p')
        via=$(printf '%s' "$pong" | sed -n 's/.* via \([0-9.]*\):[0-9]* .*/\1/p')
        if [ -n "$via" ] && is_private "$via"; then
            echo "robot: $via (Tailscale's direct path to $name; mDNS did not answer)" >&2
            echo "$via"
            exit 0
        fi
        if [ -n "$tsip" ]; then
            echo "robot: $tsip (Tailscale address of $name: no direct LAN path, so traffic may be relayed)" >&2
            echo "$tsip"
            exit 0
        fi
    fi

    echo "error: could not find the robot. Nothing answered for $name.local on this network, and Tailscale could not reach '$name'. Check it is powered and online (cat /sys/class/net/eth0/carrier, tailscale status on the robot), or set JETROVER_HOST to an address." >&2
    exit 1

# Point both Unity editors' Play mode at the robot's current address: updates RemoteHost in each project's per-machine jetrover_connection.json override (created from the Bridge default if missing) -- no rebuild. Windows side only. The Galaxy headset gets the same with `just push-galaxy-config`
[group('headset (Galaxy XR)')]
unity-robot-host host=default_jetrover_host:
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{host}}")
    winuser=$(cmd.exe /c 'echo %USERNAME%' 2>/dev/null | tr -d '\r\n')
    for proj in TeleopVR TeleopXR; do
        settings=unity/$proj/ProjectSettings/ProjectSettings.asset
        company=$(grep -m1 'companyName:' "$settings" | sed 's/.*companyName: *//' | tr -d '\r')
        product=$(grep -m1 'productName:' "$settings" | sed 's/.*productName: *//' | tr -d '\r')
        dir="/mnt/c/Users/$winuser/AppData/LocalLow/$company/$product"
        mkdir -p "$dir"
        python3 - "$dir/jetrover_connection.json" "$host" "$proj" <<'PY'
    import json, os, sys
    path, host, proj = sys.argv[1:4]
    # The override replaces the whole config, so keep an existing one's other fields.
    src = path if os.path.exists(path) else "unity/Teleop.Bridge/Runtime/Resources/jetrover_connection.json"
    cfg = json.load(open(src, encoding="utf-8"))
    old = cfg.get("RemoteHost")
    cfg["RemoteHost"] = host
    json.dump(cfg, open(path, "w", encoding="utf-8"), indent=2)
    print(f"{proj}: RemoteHost {old} -> {host}  ({path})")
    PY
    done

# Move the real JetRover arm to a Cartesian target (wrist frame, meters) and hold, e.g. `just move-arm 0.15 0 0.08` -- requires Teleop.RobotHost running on the Jetson and a human watching the hardware
[group('robot: moves the arm (someone watching)')]
move-arm x y z gripper="0" remote_host=default_jetrover_host remote_port="6000" local_port="6001":
    cd core && dotnet run --project Teleop.Eval -- move-arm \
        --x {{x}} --y {{y}} --z {{z}} --gripper {{gripper}} \
        --remote-host $(just _robot-host "{{remote_host}}") --remote-port {{remote_port}} --local-port {{local_port}} \
        --confirm-hardware-motion

# Drive the real arm through a scripted session of joint moves, the way the headset does but with no headset (docs/adr/0021's joint-drive), e.g. `just joint-drive steps 6 120`: script waypoints (smooth, teleop-like moves with pauses) or steps (one joint at a time jumps and holds: what sysid needs to see the servo's speed limit and lag), at speed rad/s (the headset slider's range, up to 6), for seconds. Limits: the IK's joint ranges less margin_deg at each end, and every pose is checked against the collision model before anything moves; it starts by moving slowly from wherever the arm is to the model's home, and ends there. It holds the arm where it is and exits non-zero when you press Enter (Ctrl+C does not reach the Windows dotnet under WSL), on a stall, or when the joint-state stream has been missing or thin for 3 s of any 10 s (a lost robot, or the headset app on the port too); a shorter Wi-Fi stall only pauses it. Close the headset app first: two senders on the joint port fight. To record it, use just record-drive. Someone watches the arm, with the space around it clear (the model knows only the robot's own body and the floor)
[group('robot: moves the arm (someone watching)')]
joint-drive script speed seconds="300" seed="1" margin_deg="15" remote_host=default_jetrover_host:
    cd core && dotnet run --project Teleop.Eval -- joint-drive --script {{script}} --speed {{speed}} --seconds {{seconds}} \
        --seed {{seed}} --margin-deg {{margin_deg}} --remote-host $(just _robot-host "{{remote_host}}") --confirm-hardware-motion

# What just joint-drive would do, planned from the model's home pose -- sends nothing, needs no robot: the envelope (the IK's joint ranges less margin_deg), the moves, the range of targets and the fastest commanded motion
[group('robot: checks and diagnostics (nothing moves)')]
joint-drive-preview script speed seconds="300" seed="1" margin_deg="15":
    cd core && dotnet run --project Teleop.Eval -- joint-drive --script {{script}} --speed {{speed}} --seconds {{seconds}} \
        --seed {{seed}} --margin-deg {{margin_deg}} --dry-run

# Interactively author a new RobotArmProfile JSON (docs/adr/0011); answers prompts from the terminal, e.g. `just build-profile`
[group('robot: profiles')]
build-profile output="" force="false":
    cd core && dotnet run --project Teleop.Eval -- build-profile \
        {{ if output != "" { "--output " + output } else { "" } }} \
        {{ if force == "true" { "--force" } else { "" } }}

# Regenerate core/RobotProfiles/fr3-duo.json (and the trainer's byte-identical copy, trainer/contract/profiles/) and its collision model fr3-duo.collision.json (fit-capsules: one capsule per link from the URDF's collision meshes, docs/adr/0026 amendment) from the vendored URDF (docs/adr/0026 decision 3) -- never edit that file by hand. Ids: left arm 1-7 + hand 8, right arm 11-17 + hand 18. The URDF has no acceleration or jerk limits, so Franka's published FR3 figures are given here (10 rad/s^2, 5000 rad/s^3); Teleop.Eval.Tests' ImportUrdfCommandTests repeats these arguments and requires the committed bytes, so change both together
[group('robot: profiles')]
import-fr3-profile:
    cd core && dotnet run --project Teleop.Eval -- import-urdf \
        --urdf ../robot/franka/fr3_duo/urdf/fr3_duo.urdf --out RobotProfiles/fr3-duo.json --name fr3-duo \
        --arm left:left_fr3v2_hand_tcp:1 --arm right:right_fr3v2_hand_tcp:11 \
        --gripper left:left_fr3v2_finger_joint1:8 --gripper right:right_fr3v2_finger_joint1:18 \
        --acceleration 10 --jerk 5000 \
        --upstream frankarobotics/franka_description@7aeeddc449edf8d62b594f9e36a81da53e7796f9
    cp core/RobotProfiles/fr3-duo.json trainer/contract/profiles/fr3-duo.json
    cd core && dotnet run --project Teleop.Eval -- fit-capsules --urdf ../robot/franka/fr3_duo/urdf/fr3_duo.urdf \
        --profile RobotProfiles/fr3-duo.json --out RobotProfiles/fr3-duo.collision.json --margin 0.01

# ---- the simulated FR3 Duo (docs/adr/0026, 0027): nothing real moves ----

fr3_sim_recordings := "recordings/fr3-sim"

# RobotHost on this PC against a SIMULATED FR3 Duo (docs/adr/0027): core/RobotProfiles/fr3-duo.json, Core's trajectory shaper at 1 kHz, a second-order tracking lag of track_hz, readings with noise_rad of seeded noise. Joint port 6002 and discovery 6005, as on the robot, so the headset and the Eval tools talk to it as they would to a robot (just fr3-sim-drive, just joint-probe remote_host=127.0.0.1). Sessions the headset commands are recorded automatically into recordings/fr3-sim (teleop-session v2, source sim-host); the uplink control file is recordings/fr3-sim/.uplink. Enter stops it. Moves nothing real
[group('fr3 (simulated)')]
fr3-sim-host track_hz="20" noise_rad="0" seed="1" port="6002" discovery_port="6005":
    mkdir -p {{fr3_sim_recordings}}
    cd core && dotnet run --project Teleop.RobotHost -- --profile-path RobotProfiles/fr3-duo.json --joint-local-port {{port}} --backend sim \
        --sim-track-hz {{track_hz}} --sim-noise-rad {{noise_rad}} --sim-seed {{seed}} --discovery-port {{discovery_port}} \
        --collision-model RobotProfiles/fr3-duo.collision.json \
        --record-control ../{{fr3_sim_recordings}}/.active --auto-record --auto-record-root ../{{fr3_sim_recordings}} \
        --uplink-control ../{{fr3_sim_recordings}}/.uplink

# Drive the simulated FR3 Duo of a running just fr3-sim-host (in another terminal) through seeded joint-space waypoints at 48 Hz (Teleop.Eval chain-drive): every joint within +-amplitude rad of where it starts, the hands toggling; Enter stops it, back at the start. Simulated robots only: chain-drive refuses a real one until the serial-chain collision model exists (docs/adr/0026 decision 5)
[group('fr3 (simulated)')]
fr3-sim-drive seconds="60" seed="1" amplitude="0.5" remote_host="127.0.0.1":
    cd core && dotnet run --project Teleop.Eval -- chain-drive --simulated-robot --remote-host {{remote_host}} \
        --seconds {{seconds}} --seed {{seed}} --amplitude-rad {{amplitude}}

# Record one scripted session on the simulated FR3 Duo (a running just fr3-sim-host): opens recordings/fr3-sim/<UTC>-<label>/ with a teleop-session v2 session.json (source sim-host) and the profile, runs chain-drive into its operator.jlog, then closes it (stoppedUtc, files.sha256). Its arm is simulated and is never the robot's (docs/adr/0027 decision 6)
[group('fr3 (simulated)')]
fr3-sim-record label seconds="60" seed="1" amplitude="0.5" network="lan":
    #!/usr/bin/env bash
    set -euo pipefail
    root={{fr3_sim_recordings}}
    mkdir -p "$root"
    [ -s "$root/.active" ] && { echo "error: a session is already recording ($(cat "$root/.active"))" >&2; exit 1; }
    id="$(date -u +%Y%m%dT%H%M%SZ)-{{label}}"
    dir="$root/$id"
    mkdir -p "$dir"
    cp core/RobotProfiles/fr3-duo.json "$dir/robot-profile.json"
    python3 - "$dir/session.json" "$id" "{{label}}" "{{network}}" "{{seed}}" "{{amplitude}}" "$(sha256sum core/RobotProfiles/fr3-duo.json | cut -d' ' -f1)" "$(git rev-parse HEAD)" "$([ -n "$(git status --porcelain -- core)" ] && echo true || echo false)" <<'PY'
    import datetime, json, sys
    path, sid, label, network, seed, amplitude, profile_sha, sha, dirty = sys.argv[1:]
    json.dump({
        "format": "teleop-session", "version": 2, "id": sid, "source": "sim-host", "label": label,
        "startedUtc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "conditions": {
            "speedSliderRadPerSec": "none",
            "load": {"kind": "none", "payloadGrams": 0},
            "network": {"uplinkProfile": network, "uplinkSeed": 0, "addedDelayMs": 0},
            "motion": "chain-drive:waypoints", "operator": "chain-drive seed " + seed + " amplitude " + amplitude,
        },
        "builds": {"robothost": {"sha": sha, "dirty": dirty == "true"}},
        "robotProfile": "fr3-duo", "robotProfileSha256": profile_sha,
        "notes": "the simulated FR3 Duo (Fr3SimBackend, just fr3-sim-host); its arm is simulated and is never the robot's",
    }, open(path, "w"), indent=2)
    PY
    if [ "{{network}}" != "lan" ]; then printf '%s 1\n' "{{network}}" > "$root/.uplink"; sleep 1; fi
    printf '%s\n' "../$dir" > "$root/.active"
    sleep 1
    status=0
    (cd core && dotnet run --project Teleop.Eval -- chain-drive --simulated-robot --seconds {{seconds}} --seed {{seed}} \
        --amplitude-rad {{amplitude}} --operator-log "../$dir/operator.jlog" --session-id "$id") || status=$?
    rm -f "$root/.active" "$root/.uplink"
    for i in $(seq 1 40); do tail -n 1 "$dir/robothost.jlog" 2>/dev/null | grep -q '^END|' && break; sleep 0.25; done
    python3 - "$dir/session.json" <<'PY'
    import datetime, json, sys
    meta = json.load(open(sys.argv[1]))
    meta["stoppedUtc"] = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    json.dump(meta, open(sys.argv[1], "w"), indent=2)
    PY
    (cd "$dir" && sha256sum session.json robot-profile.json *.jlog > files.sha256)
    for f in "$dir"/*.jlog; do printf '%-16s %8s lines  %s\n' "$(basename "$f")" "$(wc -l < "$f")" "$(tail -n 1 "$f" | cut -c1-60)"; done
    echo "recorded $dir (chain-drive exit $status)"
    exit $status

# Phase-3 cross-machine ClockSync diagnostic against an already-running Teleop.RobotHost -- moves the real arm once and holds; a human must be watching the hardware
[group('robot: moves the arm (someone watching)')]
clocksync-check remote_host=default_jetrover_host remote_port="6000" local_port="6001" rate_hz="20" duration_seconds="20":
    cd core && dotnet run --project Teleop.Eval -- clocksync-check \
        --remote-host $(just _robot-host "{{remote_host}}") --remote-port {{remote_port}} --local-port {{local_port}} \
        --rate-hz {{rate_hz}} --duration-seconds {{duration_seconds}} \
        --confirm-hardware-motion

# Redeploy everything on the robot from this checkout, in one go, and check it came up: Core's gates (checks=false skips them), robot/ros (built and tested on the Jetson before its two services restart), RobotHost (after the ROS services, so it connects to the fresh relay), the camera sender, then robot-ros-diff, joint-probe and robot-status. Stops at the first failure. Restarts the servo driver, so someone must be at the robot
[group('robot: deploy and restart (someone at the robot)')]
deploy-robot checks="true":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{checks}}" in true|false) ;; *) echo "error: checks must be true or false, got '{{checks}}'" >&2; exit 1 ;; esac
    step() { echo; echo "=== $*"; }
    if [ "{{checks}}" = "true" ]; then step "1/5 Core's gates (test, verify, audit)"; just core-check; fi
    # Resolve the robot once, so every step talks to the same address.
    export JETROVER_HOST="$(just _robot-host "${JETROVER_HOST:-auto}")"
    step "2/5 ROS workspace (robot/ros): build and test on the Jetson, then switch and restart"; just deploy-ros
    step "3/5 Teleop.RobotHost"; just deploy-robothost
    step "4/5 Teleop.CameraHost"; just deploy-camerahost
    step "5/5 Checks"; just robot-ros-diff; just joint-probe; just robot-status
    echo; echo "Robot redeployed from $(git rev-parse --short HEAD) at $JETROVER_HOST."

# Publish Teleop.RobotHost for the Jetson (linux-arm64), copy it over, and restart its systemd service -- needs passwordless ssh/scp; does NOT touch the Jetson's ROS 2 nodes, see robot/systemd/README.md. It runs recording whenever the headset commands the robot (--auto-record, docs/adr/0023; robot/README.md) unless auto_record=false, and parks then relaxes the arm on the headset's ROBOT OFF at rest_pose_deg (default jetrover_rest_pose_deg, docs/adr/0023 decision 5). Relay wire v5: deploy the ROS side too (just deploy-ros, or just deploy-robot for both)
[group('robot: deploy and restart (someone at the robot)')]
deploy-robothost remote_host=default_jetrover_host remote_user="jetson" operator_host=default_operator_host max_direction_magnitude="" profile_path="" pulses_per_second="" auto_record="true" rest_pose_deg=jetrover_rest_pose_deg park_contact_deg=jetrover_park_contact_deg:
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{auto_record}}" in true|false) ;; *) echo "error: auto_record must be true or false, got '{{auto_record}}'" >&2; exit 1 ;; esac
    host=$(just _robot-host "{{remote_host}}")
    cd core
    stamp=$(date +%Y%m%d-%H%M%S)
    remote_dir="robothost_deploy_${stamp}"
    # A relative path, not mktemp -d's /tmp/... -- dotnet here is the Windows SDK (see root
    # CLAUDE.md's Environment section), which doesn't resolve a WSL-native absolute path passed as
    # an argument (only the CWD gets translated), so -o would silently publish somewhere under
    # C:\tmp instead and leave this script tarring up an empty directory.
    publish_dir="_scratch_publish"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.RobotHost for linux-arm64..." >&2
    dotnet publish Teleop.RobotHost -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir"
    # The build every recorded session's session.json names (docs/adr/0021): ~/robothost_current/BUILD_INFO.
    printf '{"sha": "%s", "dirty": %s}\n' "$(git rev-parse HEAD)" "$([ -n "$(git status --porcelain --untracked-files=no -- .)" ] && echo true || echo false)" > "$publish_dir/BUILD_INFO"
    # The profile every automatic session copies (docs/adr/0023 decision 4), as record-start copies
    # core/RobotProfiles/jetrover.json: ~/robothost_current/robot-profile.json.
    cp RobotProfiles/jetrover.json "$publish_dir/robot-profile.json"
    tar czf "$archive" -C "$publish_dir" .
    echo "Copying to {{remote_user}}@$host:~/${remote_dir}..." >&2
    # scp, unlike ssh, needs an IPv6 literal in brackets to tell the address from the path.
    case "$host" in *:*) scp_dest="{{remote_user}}@[$host]" ;; *) scp_dest="{{remote_user}}@$host" ;; esac
    scp -o StrictHostKeyChecking=accept-new "$archive" "${scp_dest}:/tmp/${remote_dir}.tar.gz"
    extra_args=""
    if [ -n "{{max_direction_magnitude}}" ]; then extra_args="$extra_args --max-direction-magnitude {{max_direction_magnitude}}"; fi
    if [ -n "{{profile_path}}" ]; then extra_args="$extra_args --profile-path {{profile_path}}"; fi
    # The fastest the servos may be told to move (pulses/s; the code default is 2200, about 9 rad/s).
    # The headset's speed slider asks for slower moves with every joint command, and RobotHost caps
    # those requests at this. Lower it to cap the slider; leave it empty to keep the default.
    if [ -n "{{pulses_per_second}}" ]; then extra_args="$extra_args --pulses-per-second {{pulses_per_second}}"; fi
    # Recording whenever the headset commands the robot (docs/adr/0023 decision 4; robot/README.md): RobotHost
    # opens a session on the first headset joint command after 30 s without one and closes it after 30 s
    # without; only a sender whose headset heartbeat is at most 5 s old counts, so joint-drive, move-arm
    # and other tools are never recorded automatically; manual sessions (record-start, the onboard pilot)
    # win. The thresholds keep their defaults (--auto-record-idle-seconds 30, --auto-record-max-minutes 60,
    # --auto-record-stop-free-gb 1, --auto-record-start-free-gb 2, --auto-record-driver-wait-seconds 15,
    # --auto-record-min-split-seconds 5, --auto-record-headset-seconds 5).
    # auto_record=false turns it off. With profile_path, the sessions copy that profile instead.
    # Park, then relax (docs/adr/0023 decision 5): ROBOT OFF on the headset parks the arm at the rest pose
    # (jetrover_rest_pose_deg above) at 0.5 rad/s, waits until every reading is within 3 degrees, then
    # switches torque off; --park-speed-rad-per-sec, --park-tolerance-deg and --park-timeout-seconds (20)
    # keep their defaults. An empty rest pose refuses every relax.
    rest_pose='{{rest_pose_deg}}'
    case "$rest_pose" in *[!0-9.,-]*) echo "error: rest_pose_deg must be comma-separated degrees, got '$rest_pose'" >&2; exit 1 ;; esac
    if [ -n "$rest_pose" ]; then extra_args="$extra_args --rest-pose-deg $rest_pose"; fi
    if [ -n "{{park_contact_deg}}" ]; then extra_args="$extra_args --park-contact-deg {{park_contact_deg}}"; fi
    if [ "{{auto_record}}" = "true" ]; then
        extra_args="$extra_args --auto-record --build-info-robothost /home/jetson/robothost_current/BUILD_INFO --build-info-driver /home/jetson/teleop_ros_current/BUILD_INFO"
        if [ -z "{{profile_path}}" ]; then extra_args="$extra_args --auto-record-profile /home/jetson/robothost_current/robot-profile.json"; fi
    fi
    # robothost-run.sh is the stable script teleop-robothost.service's ExecStart points at
    # (robot/systemd/teleop-robothost.service) -- regenerated on every deploy with this run's exact
    # args (including any optional extra_args), so redeploying never needs `systemctl
    # daemon-reload` or touching the unit file itself, only `systemctl restart`. This also sidesteps
    # the earlier pkill+nohup approach's self-inflicted-signal bug entirely (systemd owns the
    # process lifecycle instead of a hand-rolled pkill matching its own ancestor shell).
    # --remote-host is only where replies go before the first command arrives: Teleop.RobotHost
    # defaults to --reply-to sender and then replies to wherever each command came from, so a
    # changed operator address no longer needs a redeploy. Add `--reply-to fixed` to pin it.
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" "cat > /home/jetson/robothost-run.sh" <<EOF
    #!/bin/bash
    exec /home/jetson/.dotnet/dotnet /home/jetson/robothost_current/Teleop.RobotHost.dll \
        --local-port 6000 --remote-host {{operator_host}} --remote-port 6001 \
        --relay-socket /tmp/jetrover_relay.sock --local-relay-socket /tmp/teleop_robot_host.sock \
        --joint-local-port 6002 --record-control /home/jetson/recordings/.active \
        --uplink-control /home/jetson/recordings/.uplink --discovery-port 6005 ${extra_args}
    EOF
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" "
        set -e
        mkdir -p ~/${remote_dir}
        tar xzf /tmp/${remote_dir}.tar.gz -C ~/${remote_dir}
        chmod +x ~/${remote_dir}/Teleop.RobotHost
        ln -sfn ~/${remote_dir} ~/robothost_current
        chmod +x ~/robothost-run.sh
        sudo systemctl restart teleop-robothost
        sleep 2
        echo '--- teleop-robothost.service status ---'
        systemctl status teleop-robothost --no-pager -n 10
    "

# Read-only: the camera sender's recent journal (teleop-camerahost.service) with its per-10 s [stats] lines, the video devices present, and the kernel's recent USB/camera events. Use it when the headset's feed freezes or cuts out
[group('robot: checks and diagnostics (nothing moves)')]
camera-logs lines="200" remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "systemctl status teleop-camerahost --no-pager -n 0; journalctl -u teleop-camerahost --no-pager -n {{lines}}; \
         echo '--- video devices'; ls -l /dev/video* 2>&1; ls -l /dev/v4l/by-id/ 2>&1; v4l2-ctl --list-devices 2>&1 | head -20; \
         echo '--- kernel USB/camera events (newest last)'; journalctl -k --no-pager -n 3000 | grep -iE 'usb|uvc|video' | tail -40"

# Read-only: RobotHost's recent journal (teleop-robothost.service): who it streams joint state to, the commands it receives and acts on, and anything it rejects. Use it when the headset connects but the arm doesn't move
[group('robot: checks and diagnostics (nothing moves)')]
robothost-logs lines="200" remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "systemctl status teleop-robothost --no-pager -n 0; journalctl -u teleop-robothost --no-pager -n {{lines}}"

# Read-only: the ROS side's recent journals, the servo bus driver (jetrover-arm-control) and the relay between it and RobotHost (jetrover-relay). Python buffers their stdout, so a quiet log is not proof of a quiet driver
[group('robot: checks and diagnostics (nothing moves)')]
arm-logs lines="100" remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "for u in jetrover-arm-control jetrover-relay; do echo \"=== \$u\"; journalctl -u \$u --no-pager -n {{lines}}; done"

# Read-only: the ROS link between the relay and the servo driver. The topics each side publishes and subscribes to, the services' environment (the CLI runs with ROS_LOCALHOST_ONLY=1 as they do; a mismatch makes it see no messages), the live topic list, and for a few seconds each how often messages actually flow. Use it when RobotHost logs commands but the arm doesn't move
[group('robot: checks and diagnostics (nothing moves)')]
ros-check seconds="4" remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "r=/home/jetson/teleop_ros_current; [ -d \$r/src ] || r=/home/jetson/projects/jetrover-teleop-ros; src=\$r/src; echo \"--- topics in the source (\$r)\"; grep -rn -E 'create_publisher|create_subscription' \$src --include=*.py | sed \"s|\$src/||\" | head -30; \
         echo '--- service environment'; for u in jetrover-arm-control jetrover-relay; do echo \"\$u: \$(systemctl show -p Environment --value \$u)\"; grep -h -E 'ExecStart|Environment' /etc/systemd/system/\$u.service; done; \
         source /opt/ros/foxy/setup.bash; export ROS_LOCALHOST_ONLY=1; echo \"--- this CLI (as the services): ROS_LOCALHOST_ONLY=\$ROS_LOCALHOST_ONLY ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-unset}\"; \
         echo '--- live topics'; timeout 10 ros2 topic list 2>&1; \
         for t in \$(timeout 10 ros2 topic list 2>/dev/null | grep -v -E '^/(rosout|parameter_events)\$'); do echo \"--- \$t\"; timeout {{seconds}} ros2 topic hz \$t 2>&1 | tail -2; done; true"

# Read-only: the servo controller board's serial link. Lists the serial devices present, the ones jetrover-arm-control holds open (a "(deleted)" one means the board re-enumerated after a reset or brown-out and the driver still holds the dead port), and the kernel's recent serial USB events. Use it when commands arrive but the arm doesn't move
[group('robot: checks and diagnostics (nothing moves)')]
robot-serial remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "echo '--- serial devices'; ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* /dev/serial/by-id/ 2>&1; \
         echo '--- serial ports held by jetrover-arm-control and its children'; for p in \$(systemctl show -p MainPID --value jetrover-arm-control) \$(pgrep -f robot_controller_manager); do ls -l /proc/\$p/fd 2>/dev/null | grep -E 'tty|serial' | sed \"s/^/pid \$p: /\"; done; \
         echo '--- kernel serial USB events (newest last)'; journalctl -k --no-pager -n 5000 | grep -iE 'ch34|cdc_acm|ttyUSB|ttyACM|cp210|ftdi' | grep -v 'Modules linked in' | tail -30"

# Read-only: when the board has stopped answering (just joint-probe shows every motor sensed NO), find out which side is dead, without opening the serial port (opening it can reset the board and erase the evidence). Shows: the robot's uptime; the kernel's power and USB alarms since boot; the driver's threads and what each waits on; whether any bytes reach the driver over a few seconds; how often the board's own battery report and the servo feedback still arrive; and any receive errors in the driver's log. Run it while the fault is present, before any restart; then just robot-board-usb for the USB link below the driver
[group('robot: checks and diagnostics (nothing moves)')]
robot-board-diagnose seconds="6" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "SECONDS_TO_WATCH={{seconds}} bash -s" <<'REMOTE'
    echo "--- uptime: $(uptime)"
    echo '--- kernel: power, thermal and USB alarms since boot (newest last)'
    journalctl -k -b --no-pager | grep -iE 'soctherm|oc alarm|under.?volt|brown|throttl|over.?current|usb [0-9.-]+: (reset|disconnect|device descriptor|cannot|error)|cdc_acm.*(error|-71|-110|-32)|ttyACM' | grep -v -E 'Modules linked in|trip temperature|thermtrip|throttrip|parse_throttle|cooling device registered|probe successful|failed to register sensor' | tail -25
    echo '--- Jetson input power rails (ina3221, if present)'
    for f in /sys/bus/i2c/drivers/ina3221*/*/iio:device*/in_voltage*_input /sys/bus/i2c/drivers/ina3221*/*/iio:device*/rail_name_*; do [ -r "$f" ] && echo "$(basename "$f"): $(cat "$f")"; done 2>/dev/null | head -12
    echo '--- every start of the servo driver over the last three boots (each start reopens the serial port)'
    for b in -2 -1 0; do journalctl -b "$b" -u jetrover-arm-control --no-pager -o short 2>/dev/null | grep -E 'Started|-- Logs begin' | grep Started | sed "s/^/boot $b: /"; done | tail -30
    pid=$(pgrep -f 'lib/jetrover_arm_control/robot_controller_manager' | head -1)
    if [ -z "$pid" ]; then echo '--- the driver is not running'; exit 0; fi
    echo "--- driver pid $pid, started $(ps -o lstart= -p "$pid")"
    echo '--- its threads: name, state, kernel wait'
    for t in /proc/$pid/task/*; do printf '%-8s %-16s %s %s\n' "$(basename "$t")" "$(cat "$t/comm")" "$(awk '{print $3}' "$t/stat")" "$(cat "$t/wchan" 2>/dev/null)"; done
    echo "--- bytes the driver read and wrote over ${SECONDS_TO_WATCH} s (all its fds: serial and ROS sockets together)"
    read -r r1 w1 c1 < <(awk '/^rchar|^wchar|^syscr/{printf "%s ", $2}' /proc/$pid/io); sleep "$SECONDS_TO_WATCH"
    read -r r2 w2 c2 < <(awk '/^rchar|^wchar|^syscr/{printf "%s ", $2}' /proc/$pid/io)
    echo "read $((r2 - r1)) bytes, wrote $((w2 - w1)) bytes, $((c2 - c1)) read calls"
    source /opt/ros/foxy/setup.bash; export ROS_LOCALHOST_ONLY=1
    echo "--- messages in ${SECONDS_TO_WATCH} s: servo feedback, then the board's battery report (every 5 s when the board answers)"
    timeout "$SECONDS_TO_WATCH" ros2 topic echo /unity/robot/servo/feedback_array 2>/dev/null | grep -c '^data:' || true
    timeout "$SECONDS_TO_WATCH" ros2 topic echo /robot/battery 2>/dev/null | grep '^data:' | tail -2 || true
    echo '--- receive errors in the driver log since it started'
    journalctl -u jetrover-arm-control --no-pager --since "$(date -d "$(ps -o lstart= -p "$pid")" '+%Y-%m-%d %H:%M:%S')" | grep -iE 'verificatio|traceback|error|exception|END\.\.\.' | tail -15
    echo '--- py-spy stack of every driver thread (only if py-spy is installed)'
    command -v py-spy >/dev/null && py-spy dump --pid "$pid" 2>&1 | head -60 || echo 'py-spy not installed'
    REMOTE

# Read-only, for a board that has stopped answering (run it while the fault is present, after just robot-board-diagnose): looks below the driver, at the USB link to the board, without opening the serial port. Shows: when the fault began and what the whole robot logged in the 90 s before it, the battery and read times leading up to it; every process that has the board's port open (a second reader steals its bytes); ModemManager, brltty and serial gettys, which probe new ttyACM ports; the board's USB identity, speed and power management (autosuspend); its URB count over a few seconds; and, with usbmon, what actually crosses the wire: whether the board still accepts the driver's requests (bulk OUT completing) and whether it sends anything back (bulk IN). Needs passwordless sudo on the Jetson for the last two
[group('robot: checks and diagnostics (nothing moves)')]
robot-board-usb seconds="5" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "SECONDS_TO_WATCH={{seconds}} bash -s" <<'REMOTE'
    tty=/dev/ttyACM0
    echo "--- uptime: $(uptime)"
    echo '--- the fault: the first driver [stats] line of this boot where motor 1 read nothing'
    stats=$(journalctl -b -u jetrover-arm-control --no-pager -o short-iso | grep 'robot_controller_node.*\[stats')
    onset_line=$(echo "$stats" | grep -m1 'reads ok/missed/slowest 1:0/')
    if [ -z "$onset_line" ]; then
        echo 'none: every stats line this boot has motor 1 answering (the fault is not present, or began before the driver logged)'
    else
        onset=$(echo "$onset_line" | awk '{print $1}')
        echo "$onset_line" | cut -c1-260
        echo '--- the 12 stats lines before it: cycle rate, slowest reads, battery'
        echo "$stats" | grep -B12 -m1 'reads ok/missed/slowest 1:0/' | head -12 | \
            sed -E 's/.*\] \[stats ([0-9.]+s)\] cycles [0-9]+ \(([0-9.]+ Hz)\).*reads ok\/missed\/slowest (.*); commands ([0-9]+).*battery ([0-9]+ mV).*/\1 \2 | \3 | cmds \4 | \5/' | sed "s/^/  /"
        echo '--- every unit and the kernel, 90 s before the onset to 15 s after (driver and relay stats lines left out)'
        journalctl -b --no-pager -o short-iso --since "$(date -d "$onset - 90 sec" '+%Y-%m-%d %H:%M:%S')" --until "$(date -d "$onset + 15 sec" '+%Y-%m-%d %H:%M:%S')" | \
            grep -v -E '\[stats [0-9.]+s\]' | cut -c1-220 | tail -60
    fi
    echo "--- processes with $tty open (two readers on a tty split its bytes between them)"
    sudo -n true 2>/dev/null && SUDO='sudo -n' || { SUDO=''; echo '(no passwordless sudo: only this user'"'"'s processes are visible)'; }
    $SUDO find /proc/[0-9]*/fd -lname "$tty" 2>/dev/null | cut -d/ -f3 | sort -u | while read -r p; do echo "  pid $p: $(tr '\0' ' ' < /proc/$p/cmdline | cut -c1-150)"; done
    echo '--- services that probe or claim serial ports'
    for u in ModemManager brltty serial-getty@ttyACM0 nvgetty; do printf '  %-24s %s\n' "$u" "$(systemctl is-active "$u" 2>/dev/null)"; done
    echo "--- the board's USB device"
    intf=$(readlink -f /sys/class/tty/ttyACM0/device 2>/dev/null)
    if [ -z "$intf" ]; then echo "  $tty has no device in sysfs: it is not enumerated"; exit 0; fi
    dev=$(dirname "$intf")
    for f in idVendor idProduct manufacturer product serial speed busnum devnum version bMaxPower; do printf '  %-14s %s\n' "$f" "$(cat "$dev/$f" 2>/dev/null)"; done
    for f in control runtime_status autosuspend_delay_ms runtime_active_time runtime_suspended_time; do printf '  power/%-22s %s\n' "$f" "$(cat "$dev/power/$f" 2>/dev/null)"; done
    echo "  usbcore autosuspend default: $(cat /sys/module/usbcore/parameters/autosuspend 2>/dev/null) s"
    hub=$(dirname "$dev"); echo "  parent: $(basename "$hub") $(cat "$hub/product" 2>/dev/null), power/control $(cat "$hub/power/control" 2>/dev/null), runtime_status $(cat "$hub/power/runtime_status" 2>/dev/null)"
    echo "  configuration bmAttributes $(cat "$dev/bmAttributes" 2>/dev/null) (0x80: powered from USB, 0xc0: self-powered)"
    hubdesc=$(sudo -n lsusb -v -s "$(cat "$hub/busnum"):$(cat "$hub/devnum")" 2>/dev/null | grep -iE 'power switching|Ganged|Per-port|No power switching|wHubCharacteristic' | tr -s ' ' | tr '\n' ';')
    echo "  hub power switching: ${hubdesc:-unknown (no lsusb, or no sudo)}"
    echo "  siblings on that hub: $(for d in "$hub"/*-*; do [ -f "$d/idVendor" ] && printf '%s(%s) ' "$(basename "$d")" "$(cat "$d/product" 2>/dev/null)"; done)"
    u1=$(cat "$dev/urbnum"); sleep "$SECONDS_TO_WATCH"; u2=$(cat "$dev/urbnum")
    echo "  URBs submitted to it in ${SECONDS_TO_WATCH} s: $((u2 - u1)) (the driver writes ~120 requests/s when the board answers, ~1/s when every read times out)"
    echo "--- usbmon: ${SECONDS_TO_WATCH} s of the wire to bus $(cat "$dev/busnum") device $(cat "$dev/devnum")"
    if [ -z "$SUDO" ]; then echo '  skipped: needs sudo'; exit 0; fi
    $SUDO modprobe usbmon 2>/dev/null
    mountpoint -q /sys/kernel/debug || $SUDO mount -t debugfs none /sys/kernel/debug 2>/dev/null
    mon=/sys/kernel/debug/usb/usbmon/$(cat "$dev/busnum")u
    if ! $SUDO test -r "$mon"; then echo "  skipped: $mon is not there (usbmon not in this kernel?)"; exit 0; fi
    addr=$(printf ':%d:%03d:' "$(cat "$dev/busnum")" "$(cat "$dev/devnum")") # usbmon's text: Bo:1:005:2
    cap=$(mktemp); $SUDO timeout "$SECONDS_TO_WATCH" cat "$mon" > "$cap" 2>/dev/null
    grep -F "$addr" "$cap" > "$cap.dev"
    echo "  lines for this device: $(wc -l < "$cap.dev") (of $(wc -l < "$cap") on the bus)"
    awk '{ split($4, a, ":"); k = $3 " " a[1]; n[k]++; if ($3 == "C") { st[a[1] " status " $5]++; if ($5 == 0) bytes[a[1]] += $6 } }
         END { for (k in n) printf "  %-6s %d\n", k, n[k]; for (k in st) printf "  %s: %d\n", k, st[k]; for (k in bytes) printf "  %s bytes completed: %d\n", k, bytes[k] }' "$cap.dev" | sort
    echo '  (S = submitted, C = completed; Bo = to the board, Bi = from it, Ci/Co/Ii = control/interrupt. A Bi submitted and never completed means the board sends nothing; Bo never completing means it no longer accepts data)'
    echo '  first 12 lines:'; head -12 "$cap.dev" | cut -c1-140 | sed 's/^/    /'
    rm -f "$cap" "$cap.dev"
    REMOTE

# Read-only: the servo driver's [stats] lines (every 5 s: cycle rate, reads ok/missed/slowest per motor, commands, battery) and its starts, for each of the last `boots` boots, oldest first, each line prefixed by its boot offset. The journal persists across reboots (just robot-journal-persist), so this is the history to look for a pattern in the board going silent: uptime, read count, commands, battery at each onset
[group('robot: checks and diagnostics (nothing moves)')]
robot-driver-stats boots="6" remote_host=default_jetrover_host remote_user="jetson":
    @ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        'for b in $(seq -$(( {{boots}} - 1 )) 0); do journalctl -b "$b" --no-pager -o short-iso -u jetrover-arm-control 2>/dev/null | \
            grep -E "\[stats|Started|Stopping|Stopped" | sed "s/^/boot $b /"; journalctl -b "$b" --no-pager -o short-iso -n 1 2>/dev/null | sed "s/^/boot $b LAST /"; done'

# Read-only: everything the robot logged (every unit and the kernel) from `before` seconds before to `after` seconds after a moment of a past boot -- `at` is the robot's local time as journalctl prints it (e.g. "2026-10-25 22:58:18"), `boot` the boot offset (0 this boot, -1 the one before; just robot-driver-stats prints both). The driver's and relay's 5 s stats lines are left out unless stats=true
[group('robot: checks and diagnostics (nothing moves)')]
robot-journal-around at boot="0" before="120" after="20" stats="false" remote_host=default_jetrover_host remote_user="jetson":
    @ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        'journalctl -b {{boot}} --no-pager -o short-iso --since "@$(( $(date -d "{{at}}" +%s) - {{before}} ))" --until "@$(( $(date -d "{{at}}" +%s) + {{after}} ))" | \
            { if [ "{{stats}}" = true ]; then cat; else grep -v -E "\[stats [0-9.]+s\]"; fi; } | cut -c1-240'

# Nearly read-only: the board's serial line as its USB chip reports it, from the kernel's cdc_acm counters (TIOCGICOUNT) twice, `seconds` apart: framing, parity and overrun errors and breaks the chip has signalled since it enumerated, and the modem lines. A board whose transmit line is held low (unpowered, or a crashed controller) shows breaks and framing errors; an idle-high line shows none. Opening the port a second time asserts DTR and RTS, which this puts back (cleared) straight away; it sends nothing and does not disturb the driver's own open
[group('robot: checks and diagnostics (nothing moves)')]
robot-board-line seconds="5" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "sudo python3 - {{seconds}}" <<'PY'
    import fcntl, os, struct, sys, time
    TIOCMGET, TIOCMBIC, TIOCGICOUNT = 0x5415, 0x5417, 0x545D
    TIOCM = {"DTR": 0x002, "RTS": 0x004, "CTS": 0x020, "CD": 0x040, "RI": 0x080, "DSR": 0x100}
    NAMES = ("cts", "dsr", "rng", "dcd", "rx", "tx", "frame", "overrun", "parity", "brk", "buf_overrun")

    def icount(fd):
        raw = fcntl.ioctl(fd, TIOCGICOUNT, bytes(80))
        return dict(zip(NAMES, struct.unpack("11i", raw[:44])))

    def lines(fd):
        bits = struct.unpack("i", fcntl.ioctl(fd, TIOCMGET, bytes(4)))[0]
        return " ".join(f"{k} {int(bool(bits & v))}" for k, v in TIOCM.items())

    fd = os.open("/dev/ttyACM0", os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    try:
        print(f"modem lines just after opening: {lines(fd)}  (cdc_acm always reports CTS set)")
        fcntl.ioctl(fd, TIOCMBIC, struct.pack("i", TIOCM["DTR"] | TIOCM["RTS"]))
        print(f"after clearing DTR and RTS again: {lines(fd)}")
        try:
            first = icount(fd)
        except OSError as e:
            print(f"TIOCGICOUNT: {e} (this kernel's cdc_acm does not count line events)")
            sys.exit(0)
        print("counters since the chip enumerated: " + ", ".join(f"{k} {v}" for k, v in first.items()))
        time.sleep(float(sys.argv[1]))
        second = icount(fd)
        print(f"change over {sys.argv[1]} s: " + (", ".join(f"{k} +{second[k] - first[k]}" for k in NAMES if second[k] != first[k]) or "none"))
    finally:
        os.close(fd)
    PY

# Read-only: what the robot is made of, for hardware debugging -- the Jetson model and kernel, the USB tree, sound capture devices, where each network interface sits on the buses (a Wi-Fi behind the board's USB hub drops with it), how the servo driver's service is started (its environment: without PYTHONUNBUFFERED the board SDK's own prints, such as "Board Verificatio Failed", sit in a buffer and may never reach the journal), and every receive error, traceback or thread exit the driver logged over the last `boots` boots
[group('robot: checks and diagnostics (nothing moves)')]
robot-hw-inventory boots="10" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "BOOTS={{boots}} bash -s" <<'REMOTE'
    echo "--- model: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null); kernel $(uname -r); $(head -1 /etc/nv_tegra_release 2>/dev/null)"
    echo '--- USB tree'; lsusb -t 2>/dev/null
    echo '--- USB devices'; lsusb 2>/dev/null
    echo '--- sound capture devices'; arecord -l 2>/dev/null || echo '(arecord not installed)'; cat /proc/asound/cards 2>/dev/null
    echo '--- sound playback devices'; aplay -l 2>/dev/null | grep -E '^card' || echo '(aplay not installed)'
    echo "--- python3 numpy: $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo missing)"
    echo '--- serial ports: by-id and by-path names, the USB device behind each, who has it open'
    ls -l /dev/serial/by-id/ /dev/serial/by-path/ 2>/dev/null | grep -- '->' | sed 's/.* \([^ ]* -> .*\)/  \1/'
    for t in /dev/ttyTHS*; do
        users=$(sudo -n fuser "$t" 2>/dev/null | tr -s ' ')
        [ -n "$users" ] && for p in $users; do echo "  $t open by pid $p ($(systemctl status "$p" --no-pager 2>/dev/null | head -1 | cut -c1-80)): $(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | cut -c1-160)"; done
    done
    for t in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$t" ] || continue
        d=$(readlink -f "/sys/class/tty/$(basename "$t")/device/.." 2>/dev/null); [ -f "$d/idVendor" ] || d=$(dirname "$d")
        users=$(sudo -n fuser "$t" 2>/dev/null | tr -s ' ')
        echo "  $t: usb $(basename "$d") $(cat "$d/idVendor" 2>/dev/null):$(cat "$d/idProduct" 2>/dev/null) '$(cat "$d/product" 2>/dev/null)' serial '$(cat "$d/serial" 2>/dev/null)'; open by:${users:- nobody}"
        for p in $users; do echo "      pid $p: $(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | cut -c1-120)"; done
    done
    echo '--- the vendor image: robot configuration (machine and lidar type) and what its launch files open'
    grep -hsE '^\s*export\s+(MACHINE_TYPE|LIDAR_TYPE|DEPTH_CAMERA_TYPE|CAMERA_TYPE|ASR_|LIDAR)' ~/.typerc ~/.bashrc ~/.zshrc /etc/environment 2>/dev/null | sed 's/^/  /'
    for ws in ~/ros2_ws ~/ros_ws ~/jetrover_ws; do [ -d "$ws/src" ] && grep -rhoE "/dev/(ttyUSB[0-9]|ttyACM[0-9]|ttyTHS[0-9]|[a-z_]*lidar[a-z_]*|rrc[a-z_]*)" "$ws/src" --include='*.py' --include='*.launch*' --include='*.yaml' 2>/dev/null | sort | uniq -c | sed "s|^|  $ws: |"; done
    echo "  home: $(ls ~ | tr '\n' ' ' | cut -c1-300)"
    grep -lsE 'LIDAR_TYPE|MACHINE_TYPE' ~/.[a-z]*rc ~/.profile /etc/profile.d/* 2>/dev/null | sed 's/^/  set in: /'
    echo '  kernel, this boot:'; journalctl -k -b --no-pager | grep -E 'ch341|ttyUSB' | tail -6 | sed 's/^/    /'
    echo '--- udev rules that name serial devices'
    grep -hsE 'ttyUSB|ttyACM|1a86|SYMLINK' /etc/udev/rules.d/*.rules 2>/dev/null | grep -v '^#' | cut -c1-200 | head -20
    echo '--- network interfaces and where they sit'
    for n in /sys/class/net/*; do printf '  %-12s %s\n' "$(basename "$n")" "$(readlink -f "$n/device" 2>/dev/null || echo virtual)"; done
    echo '--- the servo driver service'
    systemctl cat jetrover-arm-control --no-pager 2>/dev/null | grep -E '^(ExecStart|Environment|User|WorkingDirectory|StandardOutput)'
    pid=$(pgrep -f 'lib/jetrover_arm_control/robot_controller_manager' | head -1)
    [ -n "$pid" ] && echo "  running pid $pid, its PYTHONUNBUFFERED: $(sudo -n cat /proc/$pid/environ 2>/dev/null | tr '\0' '\n' | grep -E '^PYTHONUNBUFFERED=' || echo unset)"
    echo "--- receive errors, tracebacks and thread exits the driver logged, last $BOOTS boots"
    for b in $(seq -$((BOOTS - 1)) 0); do
        journalctl -b "$b" -u jetrover-arm-control --no-pager -o short-iso 2>/dev/null | grep -iE 'verificatio|traceback|exception in thread|END\.\.\.|serialexception|error' | sed "s/^/boot $b: /" | cut -c1-200 | tail -8
    done
    echo '(end)'
    REMOTE

# Nearly read-only: listen on one of the Jetson's serial ports (`dev`, e.g. /dev/ttyUSB0) at each of `bauds` for `seconds` each and show what arrives: byte count, the first bytes in hex, and the text when it is mostly printable -- to tell a lidar (a steady binary stream), a board's debug console (text) and a silent port apart. Sends nothing. Opening a port makes Linux assert DTR and RTS; this releases RTS first and then DTR, the order in which Hiwonder's auto-download circuit (RTS -> BOOT0, RTS asserted with DTR released -> reset) never resets the controller. Refuses a port another process has open; stop_driver=true stops the servo driver for the listen (to hear the board's own port) and starts it again, stop_service=<unit> another holder (nvgetty on ttyTHS1)
[group('robot: checks and diagnostics (nothing moves)')]
robot-serial-sniff dev bauds="115200,230400" seconds="2" stop_driver="false" stop_service="" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    # stop_driver=true frees the servo board's port (ttyACM0) for the listen and starts the driver again after;
    # stop_service=<unit> does the same for another holder (nvgetty holds the 40-pin header's ttyTHS1).
    stop="$([ "{{stop_driver}}" = true ] && echo jetrover-arm-control) {{stop_service}}"
    [ -n "${stop// /}" ] && ssh "$target" "sudo systemctl stop $stop"
    trap '[ -n "${stop// /}" ] && ssh "$target" "sudo systemctl start $stop"' EXIT
    ssh -o StrictHostKeyChecking=accept-new "$target" "sudo python3 - '{{dev}}' '{{bauds}}' '{{seconds}}'" <<'PY'
    import fcntl, os, struct, subprocess, sys, termios, time
    dev, bauds, seconds = sys.argv[1], [int(b) for b in sys.argv[2].split(",")], float(sys.argv[3])
    users = subprocess.run(["fuser", dev], capture_output=True, text=True).stdout.split()
    if users:
        sys.exit(f"{dev} is open by pid {' '.join(users)}: not touching it")
    TIOCMBIC, DTR, RTS = 0x5417, 0x002, 0x004
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    fcntl.ioctl(fd, TIOCMBIC, struct.pack("i", RTS))  # BOOT0 low first ...
    fcntl.ioctl(fd, TIOCMBIC, struct.pack("i", DTR))  # ... then DTR: no reset on the way
    try:
        for baud in bauds:
            attrs = termios.tcgetattr(fd)
            speed = getattr(termios, f"B{baud}")
            attrs[0] = 0                                    # iflag: raw
            attrs[1] = 0                                    # oflag
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0                                    # lflag: no echo, not canonical
            attrs[4] = attrs[5] = speed
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIFLUSH)
            got, end = b"", time.time() + seconds
            while time.time() < end:
                try:
                    got += os.read(fd, 4096)
                except BlockingIOError:
                    time.sleep(0.01)
            printable = sum(32 <= b < 127 or b in (9, 10, 13) for b in got) / max(1, len(got))
            print(f"{dev} at {baud} 8N1, {seconds:g} s: {len(got)} bytes ({len(got) / seconds:.0f}/s), {printable:.0%} printable")
            if got:
                print(f"  first bytes: {got[:48].hex(' ')}")
                if printable > 0.8:
                    print("  text: " + got[:400].decode("ascii", "replace").replace("\r", "").replace("\n", " | "))
    finally:
        os.close(fd)
    PY

# Read-only: every USB connect, disconnect, reset and over-current the kernel logged over the last `boots` boots, oldest first, each line prefixed by its boot offset -- to line up USB trouble (the camera's hub dropping off, a device re-enumerating) against the times the servo board went silent (just robot-driver-stats)
[group('robot: checks and diagnostics (nothing moves)')]
robot-usb-events boots="10" remote_host=default_jetrover_host remote_user="jetson":
    @ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        'for b in $(seq -$(( {{boots}} - 1 )) 0); do journalctl -b "$b" -k --no-pager -o short-iso 2>/dev/null | \
            grep -E "USB disconnect|new (full|high|low|super)-speed USB device|reset (full|high|low|super)-speed USB device|over-current|overcurrent|cannot enable|device descriptor read|unable to enumerate|Under-voltage|throttl" | \
            sed "s/^/boot $b /" | cut -c1-170; done'

# Read-only: Bluetooth devices advertising right next to the robot (signal at least `min_rssi` dBm, so roughly within a metre; nearby people's devices are only counted, never named) -- to find a Bluetooth module on the servo board, which its firmware's Bluetooth task (UART2) talks through independently of the USB link. Runs the Jetson's own discovery (btmgmt find, LE and classic) for about 10 s
[group('robot: checks and diagnostics (nothing moves)')]
robot-ble-scan min_rssi="-75" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "MIN_RSSI={{min_rssi}} bash -s" <<'REMOTE'
    echo "--- adapter: $(hciconfig 2>/dev/null | head -3 | tr -s ' \t' ' ' | tr '\n' ' ')"
    systemctl is-active bluetooth >/dev/null 2>&1 || echo '(bluetoothd is not running)'
    command -v btmgmt >/dev/null || { echo 'btmgmt not installed'; exit 0; }
    sudo -n btmgmt power on >/dev/null 2>&1
    # btmgmt prints nothing without a terminal on this BlueZ: give it one (script), and name the adapter.
    out=$(sudo -n script -qfc 'timeout -s INT 14 btmgmt --index 0 find' /dev/null < /dev/null 2>&1 | tr -d '\r')  # stdin: this script
    echo "$out" | python3 -c '
    import os, re, sys
    min_rssi = int(os.environ["MIN_RSSI"])
    near, far, cur = [], 0, None
    for line in sys.stdin:
        m = re.search(r"dev_found: (\S+) type (.+?) rssi (-?\d+)", line)
        if m:
            cur = {"addr": m.group(1), "type": m.group(2), "rssi": int(m.group(3)), "name": ""}
            (near.append(cur) if cur["rssi"] >= min_rssi else None)
            far += cur["rssi"] < min_rssi
            continue
        m = re.search(r"name (.+)$", line.strip())
        if m and cur is not None:
            cur["name"] = m.group(1)
    print(f"--- within {min_rssi} dBm: {len(near)} device(s); farther away: {far}")
    if not near and not far:
        print("  (btmgmt reported nothing it found; its output began:)")
    module = re.compile(r"(?i)hiwonder|^bt|ble|hc-?0|jdy|^hm-?1|dx-?bt|spp|cc41|mlt|esp|robot|rrc|^xfm")
    named = [d for d in near if module.search(d["name"])]
    unnamed = sum(1 for d in near if not d["name"])
    print("  of those, named like a hardware module (others are counted, not shown): {}; unnamed: {}".format(len(named), unnamed))
    for d in sorted(named, key=lambda d: -d["rssi"]):
        print("  {}  {:<16} rssi {:>4}  {}".format(d["addr"], d["type"], d["rssi"], d["name"]))
    strongest = sorted(d["rssi"] for d in near)[-3:]
    print(f"  strongest signals within range: {strongest}")
    '
    echo "$out" | grep -q dev_found || echo "$out" | head -8 | sed 's/^/    /'
    REMOTE

# Read-only: everything the kernel knows about one USB device (default: the servo board's chip on hub 1-2 port 2) -- its descriptors as sysfs has them, every interface with its class and bound driver, the tty it created if any, and lsusb's full descriptor dump
[group('robot: checks and diagnostics (nothing moves)')]
robot-usb-device path="1-2.2" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s {{path}}" <<'REMOTE'
    d=/sys/bus/usb/devices/$1
    [ -e "$d/idVendor" ] || { echo "no USB device at $1"; exit 1; }
    for f in idVendor idProduct bcdDevice product serial speed devnum bNumConfigurations bConfigurationValue bNumInterfaces bmAttributes authorized; do printf '  %-20s %s\n' "$f" "$(cat "$d/$f" 2>/dev/null)"; done
    for i in "$d"/$1:*; do
        [ -d "$i" ] || continue
        printf '  interface %-10s class %s/%s/%s, driver %s, tty %s\n' "$(basename "$i")" "$(cat "$i/bInterfaceClass")" "$(cat "$i/bInterfaceSubClass")" "$(cat "$i/bInterfaceProtocol")" \
            "$(basename "$(readlink -f "$i/driver" 2>/dev/null)" 2>/dev/null)" "$(ls "$i/tty" 2>/dev/null | tr '\n' ' ')"
    done
    echo '--- lsusb -v'
    sudo -n lsusb -v -s "$(cat "$d/busnum"):$(cat "$d/devnum")" 2>/dev/null | grep -E 'bcdUSB|bDeviceClass|bInterfaceClass|bInterfaceNumber|bEndpointAddress|wMaxPacketSize|iInterface|bmAttributes|MaxPower|CDC|bcdCDC|Call Management|ACM' | sed 's/^/  /' | head -40
    REMOTE

# The board-silence experiment (robot/README.md, "The servo board goes silent"; servo_controller/idle_keepalive.py): what the servo driver does while nobody commands the arm. `write` flashes the board's LED once a second after 5 s idle (the theory that the board needs writes); `slow-poll` reads at 10 Hz instead of 50 after 30 s idle, back to full rate on the first command (the theory that each read risks a wedge); `off` is neither. No argument shows the current mode. Nothing moves; the driver picks the change up within 5 s, no restart (it needs a driver deployed with the experiment: just deploy-ros). Compare the time to silence per mode with just robot-driver-stats
[group('robot: checks and diagnostics (nothing moves)')]
robot-keepalive mode="" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{mode}}" in ""|off|write|slow-poll) ;; *) echo "error: mode is off, write or slow-poll, got '{{mode}}'" >&2; exit 1 ;; esac
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "MODE='{{mode}}' bash -s" <<'REMOTE'
    set -euo pipefail
    f=~/.teleop/arm-keepalive
    if [ -n "$MODE" ]; then
        mkdir -p ~/.teleop
        was=$(cat "$f" 2>/dev/null || echo off)
        echo "$MODE" > "$f"
        echo "idle keep-alive: $was -> $MODE ($(date -Is)); the driver switches within 5 s"
    else
        echo "idle keep-alive: $(cat "$f" 2>/dev/null || echo 'off (no file)')"
    fi
    journalctl -u jetrover-arm-control --since "-15 s" --no-pager -o cat 2>/dev/null | grep -o "keep-alive [a-z-]* (idle [^)]*)" | tail -1 || true
    REMOTE

# Read-only: what fills the Jetson's disk -- free space, the largest things in the home directory, how many old deploy directories each component has left behind (every deploy writes a new one and repoints a *_current link) and their size, the recordings, the journal, and the usual system caches. Deletes nothing
[group('robot: checks and diagnostics (nothing moves)')]
robot-disk remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    echo "--- the root file system: $(df -h / | tail -1 | awk '{print $3 " used of " $2 ", " $4 " free (" $5 ")"}')"
    echo '--- the largest things in the home directory'
    du -sh ~/* ~/.cache ~/.local ~/.ros ~/.nuget ~/.dotnet 2>/dev/null | sort -rh | head -20 | sed 's/^/  /'
    echo '--- deploy directories (the *_current link points at the live one)'
    for link in ~/*_current; do
        [ -L "$link" ] || continue
        live=$(basename "$(readlink -f "$link")"); prefix=${live%_*}; prefix=${prefix%_*}
        n=$(ls -d ~/"${prefix}"_* 2>/dev/null | grep -v _current | wc -l)
        size=$(du -sch $(ls -d ~/"${prefix}"_* | grep -v _current) 2>/dev/null | tail -1 | cut -f1)
        echo "  $(basename "$link") -> $live; $n directories named ${prefix}_*, $size in all"
    done
    echo "--- recordings: $(du -sh ~/recordings 2>/dev/null | cut -f1) in $(ls ~/recordings/jetrover 2>/dev/null | wc -l) sessions"
    echo "--- the journal: $(journalctl --disk-usage 2>/dev/null | grep -oE '[0-9.]+[KMGT]')"
    echo '--- system caches'
    sudo -n du -sh /var/cache/apt /var/log /tmp /var/crash 2>/dev/null | sed 's/^/  /'
    command -v docker >/dev/null && sudo -n docker system df 2>/dev/null | sed 's/^/  /'
    echo '--- the whole disk, two levels deep (this file system only)'
    sudo -n du -xh --max-depth=2 / 2>/dev/null | sort -rh | head -30 | sed 's/^/  /'
    echo '--- the largest installed packages (MB)'
    dpkg-query -Wf '${Installed-Size}\t${Package}\n' 2>/dev/null | sort -rn | head -15 | awk '{printf "  %6.0f  %s\n", $1/1024, $2}'
    echo "--- swap files: $(swapon --show=NAME,SIZE --noheadings 2>/dev/null | tr '\n' ' ')"
    REMOTE

# Free space on the Jetson (its 26 GB card ran 95% full on 2026-10-26). Dry run unless apply=true: prints what would go and apt's own simulation of the package removals, then changes nothing. What it removes: the apt cache; root's pip cache; the CUDA and VisionWorks offline installer repos (the toolkits themselves stay installed); CUDA/TensorRT/VisionWorks samples and the CUDA documentation; the CUDA/cuDNN/TensorRT/VisionWorks -dev packages (headers and static libraries, needed only to compile CUDA code on the robot, which nothing of ours does); Chromium, Firefox and Thunderbird; and every old deploy directory except the live one and the two newest before it. It never runs apt autoremove (with the CUDA metapackages gone, that would take the CUDA runtime libraries too), and never touches recordings (just robot-prune-recordings) or the journal
[group('robot: deploy and restart (someone at the robot)')]
robot-disk-cleanup apply="false" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "APPLY={{apply}} bash -s" <<'REMOTE'
    set -uo pipefail
    echo "--- before: $(df -h / | tail -1 | awk '{print $4 " free of " $2 " (" $5 " used)"}')"
    installed() { dpkg-query -Wf '${Package}\n' 2>/dev/null | grep -E "$1" | sort -u; }
    mb() { dpkg-query -Wf '${Installed-Size}\n' "$@" 2>/dev/null | awk '{s+=$1} END {printf "%.0f MB", s/1024}'; }
    repos=$( { installed '^cuda-repo-l4t-.*-local$'; dpkg -S /var/visionworks-repo /var/visionworks-sfm-repo 2>/dev/null | cut -d: -f1 | tr ', ' '\n'; } | grep -v '^$' | sort -u)
    samples=$(installed '^(cuda-samples-|cuda-documentation-|libnvinfer-samples|libvisionworks-samples|libvisionworks-sfm-samples|libvisionworks-tracking-samples|vpi.*-samples)')
    dev=$(installed '^(libcudnn[0-9]*-dev|libnvinfer(-plugin)?-dev|libnvonnxparsers-dev|libnvparsers-dev|python3-libnvinfer-dev|libvisionworks(-sfm|-tracking)?-dev|vpi[0-9]*-dev|cuda-[a-z0-9-]*-dev-10-2|libcu[a-z0-9]*-dev-10-2|libnpp[a-z0-9]*-dev-10-2)$')
    apps=$(installed '^(chromium-browser|chromium-browser-l10n|chromium-codecs-ffmpeg-extra|firefox|firefox-locale-.*|thunderbird|thunderbird-locale-.*|thunderbird-gnome-support)$')
    for group in repos samples dev apps; do
        list=${!group}
        echo "--- $group ($(echo $list | wc -w) packages, $( [ -n "$list" ] && mb $list || echo '0 MB')): $(echo $list | tr '\n' ' ')"
    done
    all=$(echo $repos $samples $dev $apps)
    echo '--- apt would remove (simulation; any line here not in the lists above is pulled in by them):'
    [ -n "$all" ] && sudo -n apt-get -s purge $all 2>/dev/null | grep -E '^Purg|^Remv' | awk '{print "  " $2}' | sort | tr '\n' ' ' | fold -w 160 -s; echo
    echo "--- caches: apt $(sudo -n du -sh /var/cache/apt 2>/dev/null | cut -f1), root's pip $(sudo -n du -sh /root/.cache/pip 2>/dev/null | cut -f1)"
    old=()
    for link in ~/*_current; do
        [ -L "$link" ] || continue
        live=$(readlink -f "$link"); prefix=$(basename "$live"); prefix=${prefix%_*}; prefix=${prefix%_*}
        keep=$(ls -d ~/"${prefix}"_* 2>/dev/null | grep -v _current | grep -vx "$live" | sort | tail -2)
        for d in $(ls -d ~/"${prefix}"_* 2>/dev/null | grep -v _current | grep -vx "$live" | sort); do
            echo "$keep" | grep -qx "$d" || old+=("$d")
        done
    done
    echo "--- old deploy directories to delete: ${#old[@]} ($( [ ${#old[@]} -gt 0 ] && du -sch "${old[@]}" | tail -1 | cut -f1 || echo 0))"
    if [ "$APPLY" != true ]; then echo; echo "dry run: nothing changed. just robot-disk-cleanup true applies it"; exit 0; fi
    echo '--- applying'
    [ -n "$all" ] && sudo -n DEBIAN_FRONTEND=noninteractive apt-get purge -y -q $all 2>&1 | grep -E '^(Removing|Purging)|^E:' | sed 's/^/  /' | tail -60
    sudo -n apt-get clean
    sudo -n rm -rf /root/.cache/pip
    [ ${#old[@]} -gt 0 ] && rm -rf "${old[@]}"
    echo "--- after: $(df -h / | tail -1 | awk '{print $4 " free of " $2 " (" $5 " used)"}')"
    REMOTE

# Which recordings the headset's network disturbance altered (LAG/JITTER/LOSS act on the uplink before the real send, so RobotHost's JC records show the reordering, loss and delay; trainer/armpred/contract/disturbance.py; HD records too, since 2026-10-27), over recordings/jetrover/ here. apply=true sets them aside: moves each into recordings/disturbed/ here and ~/recordings/disturbed/ on the robot (never the session recording now), so nothing fetches, packs or trains on them; nothing is deleted. extra="id1 id2" sets aside sessions too short to judge. Fetch first (just fetch-recordings) so every session is judged
[group('robot: recording (docs/adr/0021)')]
recordings-disturbed apply="false" extra="" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{apply}}" in true|false) ;; *) echo "error: apply must be true or false" >&2; exit 1 ;; esac
    cd trainer
    PY=$([ -f .venv-win/Scripts/python.exe ] && echo .venv-win/Scripts/python.exe || echo .venv/bin/python)
    "$PY" -m armpred check-disturbance "../recordings/jetrover/*"
    ids=$("$PY" -m armpred check-disturbance --list "../recordings/jetrover/*" | tr -d '\r')
    for e in {{extra}}; do
        case "$e" in *[!A-Za-z0-9._-]*) echo "error: '$e' is not a session id" >&2; exit 1 ;; esac
        ids="$ids $e"
    done
    ids=$(echo $ids | tr ' ' '\n' | sort -u | tr '\n' ' ')
    if [ -z "${ids// /}" ]; then echo "no altered sessions"; exit 0; fi
    echo; echo "altered (or named in extra): $ids"
    if [ "{{apply}}" != "true" ]; then echo "dry run: just recordings-disturbed true to set them aside"; exit 0; fi
    cd ..
    mkdir -p recordings/disturbed
    for id in $ids; do
        if [ -d "recordings/jetrover/$id" ]; then mv "recordings/jetrover/$id" "recordings/disturbed/$id" && echo "here: $id -> recordings/disturbed/"; fi
    done
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "IDS='$ids' bash -s" <<'REMOTE'
    set -euo pipefail
    active=$(cat ~/recordings/.active 2>/dev/null || true)
    mkdir -p ~/recordings/disturbed
    for id in $IDS; do
        d=~/recordings/jetrover/$id
        [ -d "$d" ] || { echo "robot: $id not there"; continue; }
        case "$active" in *"$id"*) echo "robot: $id is recording now; left alone"; continue ;; esac
        mv "$d" ~/recordings/disturbed/ && echo "robot: $id -> ~/recordings/disturbed/"
    done
    REMOTE

# Delete recordings from the robot that this machine already holds, verified: a session goes only when every file in it on the robot has a copy under recordings/jetrover/<id>/ here with the same sha256. Never the session that is recording (named in ~/recordings/.active) or one whose logs are not closed yet (no END trailer). Dry run unless apply=true. Run just fetch-recordings first to copy what is missing
[group('robot: recording (docs/adr/0021)')]
robot-prune-recordings apply="false" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    remote=$(ssh -o StrictHostKeyChecking=accept-new "$target" 'active=$(cat ~/recordings/.active 2>/dev/null); cd ~/recordings/jetrover 2>/dev/null || exit 0
        for d in */; do d=${d%/}
            [ -n "$active" ] && [ "$(basename "$active")" = "$d" ] && { echo "SKIP $d recording now"; continue; }
            open=0; for j in "$d"/*.jlog; do [ -f "$j" ] && { tail -c 300 "$j" | grep -q "END|" || open=1; }; done
            [ $open = 1 ] && { echo "SKIP $d a log is not closed"; continue; }
            find "$d" -type f -print0 | xargs -0 sha256sum | sed "s|^|FILE |"
        done')
    echo "$remote" | grep '^SKIP' | sed 's/^SKIP /  keeping /'
    verified=$(echo "$remote" | grep '^FILE ' | python3 -c '
    import hashlib, os, sys, collections
    files = collections.defaultdict(list)
    for line in sys.stdin:
        _, sha, path = line.rstrip("\n").split(None, 2)
        files[path.split("/", 1)[0]].append((path, sha))
    for sid, entries in sorted(files.items()):
        bad = []
        for path, sha in entries:
            local = os.path.join("recordings", "jetrover", path)
            if not os.path.isfile(local):
                bad.append(f"{path} missing here")
                continue
            with open(local, "rb") as f:
                if hashlib.sha256(f.read()).hexdigest() != sha:
                    bad.append(f"{path} differs")
        if bad:
            print(f"keeping {sid}: " + "; ".join(bad[:3]), file=sys.stderr)
        else:
            print(sid)
    ')
    n=$(echo "$verified" | grep -c . || true)
    echo "--- verified on this machine and deletable on the robot: $n session(s)"
    echo "$verified" | grep . | sed 's/^/  /'
    if [ "{{apply}}" != true ] || [ "$n" = 0 ]; then [ "{{apply}}" != true ] && echo "dry run: nothing deleted. just robot-prune-recordings true deletes them"; exit 0; fi
    ids=$(echo "$verified" | grep -E '^[0-9]{8}T[0-9]{6}Z-[A-Za-z0-9._-]+$' | tr '\n' ' ')
    ssh "$target" "cd ~/recordings/jetrover && for d in $ids; do rm -rf -- \"\$d\" && echo \"  deleted \$d\"; done; echo \"--- free now: \$(df -h / | tail -1 | awk '{print \$4}')\""
    REMOTE_DONE=1

# Read-only: subscribe to RobotHost's joint-state stream (docs/adr/0018) for a few seconds and print the frame rate, round trip, and per motor whether it is sensed (servo feedback) or only a commanded belief. Sends a keepalive, never a command: nothing moves
[group('robot: checks and diagnostics (nothing moves)')]
joint-probe seconds="3" remote_host=default_jetrover_host:
    cd core && dotnet run --project Teleop.Eval -- joint-probe --remote-host $(just _robot-host "{{remote_host}}") --seconds {{seconds}}

# Read-only: which servo ids the Jetson's ROS relay reads position feedback from (FEEDBACK_MOTOR_IDS in robot/ros's robot_controller_node.py, docs/adr/0011; change it there and run just deploy-ros), the loop that reads them (a servo that does not answer is silently skipped), and where relay_node.py tracks them. A motor missing or silent here is reported as a commanded belief, never measured (just joint-probe shows the effect)
[group('robot: checks and diagnostics (nothing moves)')]
robot-feedback-ids remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "systemctl cat jetrover-arm-control --no-pager 2>/dev/null | grep -E '^(ExecStart|WorkingDirectory)=' ; \
         echo '--- FEEDBACK_MOTOR_IDS (file:line):'; \
         grep -rn --include='*.py' -E 'FEEDBACK_MOTOR_IDS *=' /home/jetson 2>/dev/null | grep -v -E '/(build|install|\.cache)/' ; \
         echo '--- where the list is used:'; \
         grep -rn --include='*.py' 'FEEDBACK_MOTOR_IDS' /home/jetson 2>/dev/null | grep -v -E '/(build|install|\.cache)/' | grep -v -E 'FEEDBACK_MOTOR_IDS *=' | head -10; \
         echo '--- the feedback loop:'; r=/home/jetson/teleop_ros_current; [ -d \$r/src ] || r=/home/jetson/projects/jetrover-teleop-ros; f=\$r/src/jetrover_arm_control/jetrover_arm_control/robot_controller_node.py; \
         n=\$(grep -n 'for motor_id in FEEDBACK_MOTOR_IDS' \$f | head -1 | cut -d: -f1); [ -n \"\$n\" ] && sed -n \"\$((n-12)),\$((n+25))p\" \$f; \
         echo '--- relay_node.py, where feedback ids are tracked:'; \\
         grep -n -E 'StaleTracked|motor_id|FEEDBACK|_ids|MOTOR' \$(ls \$r/src/*/*/relay_node.py 2>/dev/null | head -1) | head -30"

# Read-only: the Jetson's git checkout of the ROS code (~/projects/jetrover-teleop-ros): its branch and remote, how it compares with GitHub, uncommitted changes in full, and untracked files. Fetches, never pulls or resets
[group('robot: checks and diagnostics (nothing moves)')]
robot-ros-status remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "cd /home/jetson/projects/jetrover-teleop-ros && git remote -v | head -2 && git status -sb | head -1 && \
         (timeout 20 git fetch -q 2>&1 || echo 'fetch failed (offline?)') && echo '--- vs its upstream (behind / ahead):' && (git rev-list --left-right --count @{u}...HEAD 2>/dev/null || echo 'no upstream: this branch was never pushed') && \
         echo '--- last commits:' && git log --oneline -5 && echo '--- uncommitted:' && git status --short && git diff && \
         echo '--- untracked, outside build/install/log:' && git ls-files --others --exclude-standard | grep -v -E '^(build|install|log)/' | head -30"

# Read-only: is the ROS code on the Jetson the same as robot/ros/? Compares a checksum of every source file under robot/ros/src (build output, caches and .bak files left out) with the Jetson's active copy (~/teleop_ros_current, or the old checkout before the first deploy-ros). Prints only the files that differ or exist on one side, then a verdict
[group('robot: checks and diagnostics (nothing moves)')]
robot-ros-diff remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    filter='/(build|install|log|__pycache__|\.pytest_cache)/|\.pyc$|\.bak$'
    here=$(cd robot/ros && find src -type f | grep -v -E "$filter" | sort | xargs sha256sum)
    there=$(ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" \
        "r=/home/jetson/teleop_ros_current; [ -d \$r/src ] || r=/home/jetson/projects/jetrover-teleop-ros; echo \"# \$r\" >&2; \
         cd \$r && find src -type f | grep -v -E '$filter' | sort | xargs sha256sum")
    d=$(diff <(echo "$here") <(echo "$there") || true)
    if [ -z "$d" ]; then echo "robot/ros/src matches the Jetson: $(echo "$here" | wc -l) files"; else echo "$d" | sed 's/^</  repo  /; s/^>/  robot /' | grep -E '^  (repo|robot)'; echo "DIFFERS"; exit 1; fi

# Copy everything the Jetson's ROS checkout holds that GitHub may not, into <dest>: a git bundle of all its refs (commits never pushed included), the uncommitted diff against HEAD, and a tarball of its untracked source files (build/, install/ and log/ left out). How robot/ros/ was first imported (docs/adr/0019); also a backup. Writes only /tmp on the robot; changes nothing there
[group('robot: checks and diagnostics (nothing moves)')]
robot-ros-fetch dest remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    mkdir -p "{{dest}}"
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" "
        set -e; cd /home/jetson/projects/jetrover-teleop-ros
        echo \"branch: \$(git rev-parse --abbrev-ref HEAD)  HEAD: \$(git rev-parse HEAD)\"
        git bundle create /tmp/ros-import.bundle --all 2>&1 | tail -1
        git diff HEAD > /tmp/ros-import-worktree.diff
        git ls-files --others --exclude-standard | grep -v -E '^(build|install|log)/' > /tmp/ros-import-untracked.txt || true
        tar czf /tmp/ros-import-untracked.tgz -T /tmp/ros-import-untracked.txt
        echo \"uncommitted diff: \$(wc -l < /tmp/ros-import-worktree.diff) lines; untracked files: \$(wc -l < /tmp/ros-import-untracked.txt)\"
    "
    scp -q -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host:/tmp/ros-import.bundle" "{{remote_user}}@$host:/tmp/ros-import-worktree.diff" \
        "{{remote_user}}@$host:/tmp/ros-import-untracked.txt" "{{remote_user}}@$host:/tmp/ros-import-untracked.tgz" "{{dest}}/"
    ls -la "{{dest}}"

# Read-only: what the Hiwonder controller board can be told to do. Finds the board library jetrover-arm-control uses (its set_buzzer, board commands and function codes) and lists every function, plus anything about the buzzer, battery or alarm. Nothing is sent to the board
[group('robot: checks and diagnostics (nothing moves)')]
robot-board-commands remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "files=\$(grep -rl --include=*.py -E 'def set_buzzer|PACKET_FUNC|BOARD_' /home/jetson /opt 2>/dev/null | grep -v -E '/build/|/install/|site-packages/(numpy|scipy)' | sort -u | head -8); \
         for f in \$files; do echo \"=== \$f\"; grep -n -E '^ *def |PACKET_FUNC|FUNC_|= *0x' \$f | head -60; echo '--- buzzer / battery / alarm:'; grep -n -i -E 'buzz|batt|alarm|volt|warn|beep' \$f | head -30; done"

# Make the robot's journal survive reboots, once: without it a power-cycle (the only cure when the servo board goes silent, robot/README.md) wipes the driver's statistics that show when it went silent and at what voltage. Creates /var/log/journal, caps the journal at 200 MB (a drop-in in /etc/systemd/journald.conf.d/, so it cannot fill the card), and flushes the running journal to disk. It does not restart systemd-journald, which on older systemd can cut the running services' log streams. The cap applies from the next boot. Safe to run again. Afterwards `journalctl --list-boots` on the robot lists earlier boots
[group('robot: deploy and restart (someone at the robot)')]
robot-journal-persist remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    set -euo pipefail
    echo "--- systemd $(systemctl --version | head -1 | awk '{print $2}'); Storage setting: $(grep -hE '^\s*Storage=' /etc/systemd/journald.conf /etc/systemd/journald.conf.d/*.conf 2>/dev/null | tail -1 || true)"
    echo "--- free space on /var: $(df -h /var | awk 'NR==2 {print $4 " of " $2}')"
    if grep -hqE '^\s*Storage=(volatile|none)' /etc/systemd/journald.conf /etc/systemd/journald.conf.d/*.conf 2>/dev/null; then
        echo "journald is set to keep nothing on disk (Storage=volatile or none). Changing that needs a journald restart; not doing it here." >&2
        exit 1
    fi
    sudo mkdir -p /var/log/journal /etc/systemd/journald.conf.d
    sudo systemd-tmpfiles --create --prefix /var/log/journal
    printf '[Journal]\nStorage=persistent\nSystemMaxUse=200M\n' | sudo tee /etc/systemd/journald.conf.d/teleop-persistent.conf >/dev/null
    sudo journalctl --flush
    echo "--- on disk now:"; sudo du -sh /var/log/journal
    echo "--- boots the journal knows:"; journalctl --list-boots --no-pager | tail -3
    REMOTE

# Measure how far joints really go, by hand: stops the servo driver, switches off torque on the given servos (default 2, the shoulder, and 4, the wrist pitch), then for `seconds` prints their angles live while someone moves each one by hand to where it physically touches, and records the furthest each reached. Angles are the IK's (RobotHost's conversion: (pulse - 500) / 238.7 per radian, minus jetrover.json's zeroOffsetRadians). At the end each servo is commanded to hold where it is (so nothing snaps back), and the driver starts again. With the shoulder's torque off the whole arm falls: someone must hold it the entire time. docs/adr/0020: these numbers, less a margin, replace the CAD's joint ranges
[group('robot: deploy and restart (someone at the robot)')]
robot-measure-joint-limits ids="2 4" seconds="90" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "IDS='{{ids}}' SECONDS_TO_MEASURE={{seconds}} bash -s" <<'REMOTE'
    sudo systemctl stop jetrover-arm-control
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONUNBUFFERED=1 PYTHONPATH="$sdk" python3 - <<'PY'
    import math, os, time
    from jetrover_arm_control.board_manager.robot_controller_sdk import Board, PacketFunction

    ids = [int(i) for i in os.environ["IDS"].split()]
    seconds = float(os.environ["SECONDS_TO_MEASURE"])
    names = {1: "base yaw", 2: "shoulder", 3: "elbow", 4: "wrist pitch", 5: "wrist roll"}
    zero_offset = {2: 0.12217305}  # core/RobotProfiles/jetrover.json
    pulse_per_radian = 1000.0 / math.radians(240.0)

    def angle(servo, pulse):
        return math.degrees((pulse - 500) / pulse_per_radian - zero_offset.get(servo, 0.0))

    def read(board, servo):
        got = board.bus_servo_read_position(servo)
        return got[0] if got else None

    def torque(i):
        got = board.bus_servo_read_torque_state(i)
        return got[0] if got else None

    board = Board()
    board.enable_reception()
    time.sleep(0.5)
    lo, hi, last = {}, {}, {}
    try:
        for n in (5, 4, 3, 2, 1):
            print(f"torque off on {', '.join(names.get(i, str(i)) for i in ids)} in {n} s: hold the arm", flush=True)
            time.sleep(1)
        for i in ids:
            # Torque off is sub-command 0x0B on this board (the opposite of Hiwonder's published SDK,
            # measured 2026-10-21). Sent raw, so it does not depend on which SDK version is deployed.
            board.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [0x0B, i])
            time.sleep(0.02)
            state = torque(i)
            print(f"  {names.get(i, i)} (motor {i}): torque now {state} (0 = off)", flush=True)
            if state != 0:
                raise RuntimeError(f"motor {i} did not switch its torque off; stopping")
        print(f"TORQUE OFF. Move each joint slowly to where it touches, both ways. {seconds:.0f} s.", flush=True)
        end, shown = time.time() + seconds, 0.0
        while time.time() < end:
            for i in ids:
                p = read(board, i)
                if p is not None:
                    a = angle(i, p)
                    last[i] = p
                    lo[i] = min(lo.get(i, a), a)
                    hi[i] = max(hi.get(i, a), a)
            if time.time() - shown >= 0.5:
                shown = time.time()
                print("  " + "   ".join(
                    f"{names.get(i, i)} {angle(i, last[i]):7.1f} deg (seen {lo[i]:6.1f} to {hi[i]:6.1f})" if i in last else f"{names.get(i, i)} no reading"
                    for i in ids) + f"   {end - time.time():3.0f} s left", flush=True)
        for n in (5, 4, 3, 2, 1):
            print(f"holding in {n} s: put the arm where it should stay", flush=True)
            time.sleep(1)
    finally:
        # Command each servo to where it is now, which turns its torque back on there.
        for i in ids:
            p = read(board, i)
            if p is None:
                p = last.get(i)
            if p is not None:
                board.bus_servo_set_position(1.0, [[i, int(p)]])
            print(f"  {names.get(i, i)} (motor {i}): holding at pulse {p}, torque {torque(i)}", flush=True)
        print("TORQUE ON (holding).", flush=True)
        lines = [f"  {names.get(i, i)} (motor {i}): {lo[i]:.1f} to {hi[i]:.1f} deg" if i in lo else f"  {names.get(i, i)} (motor {i}): no reading"
                 for i in ids]
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.expanduser(f"~/joint-limits-{stamp}.txt")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        print(f"--- measured, in the IK's angles (also saved to {path}):")
        print("\n".join(lines))
    PY
    sudo systemctl start jetrover-arm-control
    sleep 5
    systemctl is-active jetrover-arm-control
    REMOTE

# Find where the gripper (motor 10) physically stops, by driving it there slowly (moves the gripper only; someone watches it, and keeps it empty: anything in it reads as a stop). It cannot be measured by hand: with its torque off it still does not move (2026-10-25, its gearing holds it). Stops the servo driver, then steps the gripper `step` pulses at a time toward open and reads where it is after each step; once it has stopped moving (under 3 pulses of progress over two steps, lagging its target by more than `lag`; or lagging by over 60) it has reached the stop, which is recorded, and the gripper is backed off `margin` pulses at once so it never sits straining there. Then the same toward closed. Suggests GripperOpenPulse/GripperClosedPulse `margin` inside the stops (on 2026-10-25 it sat at pulse 176 against an open target of 167, at 72 C). Ends with the gripper open, just inside its stop, and the driver started again
[group('robot: deploy and restart (someone at the robot)')]
robot-measure-gripper step="10" lag="20" margin="10" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "STEP={{step}} LAG={{lag}} MARGIN={{margin}} bash -s" <<'REMOTE'
    sudo systemctl stop jetrover-arm-control
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONUNBUFFERED=1 PYTHONPATH="$sdk" python3 - <<'PY'
    import os, time
    from jetrover_arm_control.board_manager.robot_controller_sdk import Board

    GRIPPER = 10
    step, lag_limit, margin = int(os.environ["STEP"]), int(os.environ["LAG"]), int(os.environ["MARGIN"])
    STEP_SECONDS = 0.12
    STILL, MAX_LAG = 3, 60
    # GenericArmPlantConfig.Default, core/Teleop.RobotHost/Plant/GenericArmPlantConfig.cs: open is the smaller pulse.
    current_open, current_closed = 116, 645

    def read(board):
        got = board.bus_servo_read_position(GRIPPER)
        return got[0] if got else None

    def move(board, pulse, seconds=STEP_SECONDS):
        board.bus_servo_set_position(seconds, [[GRIPPER, int(max(0, min(1000, pulse)))]])

    def find_stop(board, start, direction, name):
        """Steps from `start` in `direction` (-1 toward open, +1 toward closed) until the gripper stops moving.

        Lag alone is not a stop: it follows its target 10-20 pulses behind all the way (2026-10-25), and
        is slow to start. A stop is where it no longer moves: under `STILL` pulses of progress over two
        steps while its target went on, with it lagging by more than `lag`. `MAX_LAG` is a backstop.
        """
        target, history = start, [start]
        while 0 < target < 1000:
            target += direction * step
            move(board, target)
            time.sleep(STEP_SECONDS + 0.03)
            p = read(board)
            if p is None:
                continue
            history.append(p)
            lag = (target - p) * direction
            progress = (p - history[-3]) * direction if len(history) >= 3 else step
            print(f"  {name}: target {target:4d}  at {p:4d}  lag {lag:4d}  moved {progress:4d} over two steps", flush=True)
            if (lag > lag_limit and progress < STILL) or lag > MAX_LAG:
                # The stop: back off at once, so it does not strain there.
                move(board, p - direction * margin, 0.3)
                time.sleep(0.4)
                return p
        return history[-1]

    board = Board()
    board.enable_reception()
    time.sleep(0.5)
    start = read(board)
    if start is None:
        raise SystemExit("the gripper did not answer a position read; nothing moved")
    print(f"gripper at pulse {start}, temperature {(board.bus_servo_read_temp(GRIPPER) or ['?'])[0]} C", flush=True)
    for n in (3, 2, 1):
        print(f"the gripper moves in {n} s: keep it empty and keep fingers out", flush=True)
        time.sleep(1)
    open_stop = closed_stop = None
    try:
        # Take hold where it is first, so the first step is a small one from here.
        move(board, start, 0.2)
        time.sleep(0.3)
        open_stop = find_stop(board, start, -1, "open")
        here = read(board) or open_stop + margin
        closed_stop = find_stop(board, here, +1, "close")
    finally:
        rest = (open_stop + margin) if open_stop is not None else start
        move(board, rest, 1.0)
        # Wait until it is there (up to 4 s), not a fixed time: it may move slower than it is told.
        for _ in range(40):
            time.sleep(0.1)
            p = read(board)
            if p is not None and abs(p - rest) <= 5:
                break
        print(f"gripper left open at pulse {read(board)} (just inside the open stop), temperature {(board.bus_servo_read_temp(GRIPPER) or ['?'])[0]} C", flush=True)
    lines = [
        f"stops: open {open_stop}, closed {closed_stop} ({closed_stop - open_stop} pulses apart)",
        f"current config: open {current_open:.0f}, closed {current_closed:.0f}",
        f"suggested ({margin} inside each stop): GripperOpenPulse {open_stop + margin}, GripperClosedPulse {closed_stop - margin}",
    ]
    path = os.path.expanduser(time.strftime("~/gripper-stops-%Y%m%d-%H%M%S.txt"))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("--- measured (also saved to " + path + "):")
    print("\n".join(lines))
    PY
    sudo systemctl start jetrover-arm-control
    sleep 5
    echo "--- driver: $(systemctl is-active jetrover-arm-control)"
    REMOTE

# Ask servos what they will report, one question at a time -- sends no move and no torque command, but stops the servo driver for a few seconds (restarting it has made the shoulder sag: someone holds the arm). For each id: its id, position, input voltage and temperature, each read on its own with a 2 s wait, saying whether the servo answered with a value, answered with an error code, or did not answer, and how long it took; the position five times. The board library folds an error and silence into the same None, so the driver cannot tell them apart; this can. Then it starts the driver again and checks it came back
[group('robot: deploy and restart (someone at the robot)')]
robot-servo-probe ids="1 5 10" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "IDS='{{ids}}' bash -s" <<'REMOTE'
    sudo systemctl stop jetrover-arm-control
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONUNBUFFERED=1 PYTHONPATH="$sdk" python3 - <<'PY'
    import os, queue, struct, time
    from jetrover_arm_control.board_manager.robot_controller_sdk import Board, PacketFunction

    QUESTIONS = [("id", 0x12, "<BBbB"), ("position", 0x05, "<BBbh"), ("voltage mV", 0x07, "<BBbH"), ("temperature C", 0x09, "<BBbB")]

    def ask(board, servo, cmd, unpack, wait=2.0):
        # As Board.bus_servo_read_and_unpack, but keeping what it throws away: the error code, and silence apart from it.
        with board.servo_read_lock:
            try:
                board.bus_servo_queue.get_nowait()
            except queue.Empty:
                pass
            start = time.monotonic()
            board.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo])
            try:
                data = board.bus_servo_queue.get(block=True, timeout=wait)
            except queue.Empty:
                return f"NO ANSWER in {wait:.0f} s"
            ms = (time.monotonic() - start) * 1000
            try:
                sid, rcmd, success, *info = struct.unpack(unpack, data)
            except struct.error:
                return f"answered {ms:.0f} ms, unreadable reply {bytes(data).hex()}"
            if rcmd != cmd or sid != servo:
                return f"answered {ms:.0f} ms, but for servo {sid} question {rcmd:#x}: {bytes(data).hex()}"
            if success != 0:
                return f"answered {ms:.0f} ms with ERROR code {success}: {bytes(data).hex()}"
            return f"{info[0] if len(info) == 1 else info} ({ms:.0f} ms)"

    board = Board()
    board.enable_reception()
    time.sleep(0.5)
    for servo in [int(i) for i in os.environ["IDS"].split()]:
        print(f"--- servo {servo}", flush=True)
        for name, cmd, unpack in QUESTIONS:
            print(f"  {name:14} {ask(board, servo, cmd, unpack)}", flush=True)
            time.sleep(0.05)
        for n in range(4):
            print(f"  {'position':14} {ask(board, servo, 0x05, '<BBbh')}", flush=True)
            time.sleep(0.05)
    PY
    sudo systemctl start jetrover-arm-control
    sleep 5
    echo "--- driver: $(systemctl is-active jetrover-arm-control)"
    REMOTE

# Experiment, for a board that has stopped answering (just robot-board-diagnose: 0 bytes read): try to bring it back from the Jetson instead of the power switch. Stops the servo driver, opens the board's serial port itself, and asks servo 1 for its position: with both control lines released, then after a 100 ms DTR pulse, then after a 100 ms RTS pulse, stopping at the first that gets an answer. It prints which one worked, then starts the driver again and checks the feedback. First run, 2026-10-20: none did, so these lines do not reset the board (only the power switch did). A pulse that resets the board's controller can drop the servos' torque, so someone must hold the arm
[group('robot: deploy and restart (someone at the robot)')]
robot-board-reset remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    sudo systemctl stop jetrover-arm-control
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONPATH="$sdk" python3 - <<'PY'
    import time
    import serial
    from jetrover_arm_control.board_manager.robot_controller_sdk import checksum_crc8

    def request():
        buf = [0xAA, 0x55, 5, 2, 0x05, 1]  # bus servo (5): read position (0x05) of servo 1
        return bytes(buf + [checksum_crc8(bytes(buf[2:]))])

    def answer(port, seconds):
        port.reset_input_buffer()
        port.write(request())
        got, end = b"", time.time() + seconds
        while time.time() < end:
            got += port.read(64)
        return got

    port = serial.Serial(None, 1000000, timeout=0.1)
    port.rts = False
    port.dtr = False
    port.port = "/dev/ttyACM0"
    port.open()
    time.sleep(0.5)
    got = answer(port, 1.5)
    print(f"lines released: {len(got)} bytes back {got[:24].hex()}")
    for line in ("dtr", "rts"):
        if got:
            break
        setattr(port, line, True)
        time.sleep(0.1)
        setattr(port, line, False)
        time.sleep(1.5)  # the controller boots
        got = answer(port, 2.0)
        print(f"after a {line.upper()} pulse: {len(got)} bytes back {got[:24].hex()}")
    port.close()
    print("RESULT: the board answers" if got else "RESULT: still silent; only the power switch is left")
    PY
    sudo systemctl start jetrover-arm-control
    sleep 6
    REMOTE
    just joint-probe 3

# Experiment, for a board that has stopped answering (just robot-board-usb: bulk OUT completes, nothing comes back): reset only the board's USB-serial chip (the CH9102, 1a86:55d4) at the USB level, as unplugging and replugging it would, without cutting its power or the robot's. Stops the servo driver, issues USBDEVFS_RESET to the chip, waits for /dev/ttyACM0 to come back, asks servo 1 for its position as robot-board-reset does, then starts the driver and shows its first stats. A driver restart already pulses DTR (which seems to reset the board's controller) and never revived a silent board (2026-10-20), so an answer here means the chip's USB side was what was stuck. The shoulder may sag while the driver is down, so someone must be at the robot
[group('robot: deploy and restart (someone at the robot)')]
robot-board-usb-reset remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    dev=$(dirname "$(readlink -f /sys/class/tty/ttyACM0/device)")
    node=$(printf '/dev/bus/usb/%03d/%03d' "$(cat "$dev/busnum")" "$(cat "$dev/devnum")")
    echo "--- the board's chip: $(basename "$dev") $(cat "$dev/idVendor"):$(cat "$dev/idProduct") $(cat "$dev/product"), $node"
    sudo systemctl stop jetrover-arm-control
    since=$(date '+%Y-%m-%d %H:%M:%S')
    sudo python3 - "$node" <<'PY'
    import fcntl, os, sys
    USBDEVFS_RESET = 0x5514  # _IO('U', 20)
    fd = os.open(sys.argv[1], os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        print("USBDEVFS_RESET: done")
    except OSError as e:
        print(f"USBDEVFS_RESET failed: {e}")
    finally:
        os.close(fd)
    PY
    for i in $(seq 1 40); do [ -e /dev/ttyACM0 ] && break; sleep 0.25; done
    sleep 1
    echo "--- kernel since the reset"; journalctl -k --no-pager --since "$since" | tail -12
    echo "--- serial ports now: $(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONPATH="$sdk" python3 - <<'PY'
    import time
    import serial
    from jetrover_arm_control.board_manager.robot_controller_sdk import checksum_crc8
    buf = [0xAA, 0x55, 5, 2, 0x05, 1]  # bus servo (5): read position (0x05) of servo 1
    request = bytes(buf + [checksum_crc8(bytes(buf[2:]))])
    port = serial.Serial(None, 1000000, timeout=0.1)
    port.rts = False
    port.dtr = False
    port.port = "/dev/ttyACM0"
    port.open()
    time.sleep(1.5)  # opening pulses DTR; give the controller time to boot if that reset it
    got = port.read(4096)
    print(f"unprompted in 1.5 s (a battery report comes every 5 s): {len(got)} bytes {got[:24].hex()}")
    port.write(request)
    end = time.time() + 2.0
    while time.time() < end:
        got += port.read(64)
    port.close()
    print("RESULT: the board answers after a USB reset of its chip" if got else "RESULT: still silent after a USB reset of its chip")
    PY
    sudo systemctl start jetrover-arm-control
    sleep 12
    echo "--- the driver's first stats"; journalctl -u jetrover-arm-control --no-pager --since "$since" | grep '\[stats' | tail -2 | cut -c1-300
    REMOTE

# Experiment, for a board that has stopped answering and survived robot-board-usb-reset: is the board's controller alive, and can the Jetson reset it? Stops the servo driver, then with the port open: listens 8 s after a DTR pulse, an RTS pulse and a pulse of both (asking servo 1 for its position half way), and tries both wirings of the usual auto-boot circuit to start the STM32's ROM bootloader (BOOT0 held, reset pulsed, then 0x7F at 115200 8E1, which a live STM32 answers with 0x79 and GET with its command list). Nothing is written to flash; it ends with a plain reset into the firmware, listens again and starts the driver. Each step prints the robot's clock, so a beep from the board can be matched to it. A reset of the controller can drop the servos' torque, so someone must hold the arm
[group('robot: deploy and restart (someone at the robot)')]
robot-board-boot-probe remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    sudo systemctl stop jetrover-arm-control
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONUNBUFFERED=1 PYTHONPATH="$sdk" python3 - <<'PY'
    import time
    import serial
    from jetrover_arm_control.board_manager.robot_controller_sdk import checksum_crc8

    buf = [0xAA, 0x55, 5, 2, 0x05, 1]  # bus servo (5): read position (0x05) of servo 1
    REQUEST = bytes(buf + [checksum_crc8(bytes(buf[2:]))])

    def say(text):
        print(f"{time.strftime('%H:%M:%S')} {text}")

    def modem(port):
        return f"CTS {int(port.cts)} DSR {int(port.dsr)} RI {int(port.ri)} CD {int(port.cd)}"

    def listen(port, seconds, ask_at=None):
        got, start, asked = b"", time.time(), False
        while time.time() - start < seconds:
            if ask_at is not None and not asked and time.time() - start >= ask_at:
                port.write(REQUEST)
                asked = True
            got += port.read(256)
        return got

    def firmware_mode(port):
        port.baudrate, port.parity = 1000000, serial.PARITY_NONE

    def pulse(port, lines):
        for line in lines:
            setattr(port, line, True)
        time.sleep(0.1)
        for line in lines:
            setattr(port, line, False)

    port = serial.Serial(None, 1000000, timeout=0.05)
    port.rts = False
    port.dtr = False
    port.port = "/dev/ttyACM0"
    port.open()
    say(f"port open, lines released; inputs: {modem(port)}")
    got = listen(port, 6, ask_at=1)
    say(f"no pulse, 6 s: {len(got)} bytes {got[:24].hex()}")
    for lines in (["dtr"], ["rts"], ["dtr", "rts"]):
        say(f"pulse {'+'.join(l.upper() for l in lines)} 100 ms")
        pulse(port, lines)
        got = listen(port, 8, ask_at=4)
        say(f"  then 8 s: {len(got)} bytes {got[:24].hex()}; inputs: {modem(port)}")
    for boot0, reset in (("rts", "dtr"), ("dtr", "rts")):
        say(f"bootloader try: {boot0.upper()} held as BOOT0, {reset.upper()} pulsed as reset")
        setattr(port, boot0, True)
        time.sleep(0.05)
        pulse(port, [reset])
        time.sleep(0.3)
        port.baudrate, port.parity = 115200, serial.PARITY_EVEN
        port.reset_input_buffer()
        port.write(b"\x7f")
        ack = listen(port, 1)
        line = f"  0x7F -> {ack[:16].hex() or 'nothing'}"
        if ack[-1:] == b"\x79" or ack[-1:] == b"\x1f":
            port.write(b"\x00\xff")  # GET: version and supported commands
            line += f"; GET -> {listen(port, 1)[:20].hex() or 'nothing'}"
        say(line)
        setattr(port, boot0, False)
        firmware_mode(port)
        if ack:
            break
    say("plain reset into the firmware (DTR pulse, then RTS pulse)")
    pulse(port, ["dtr"])
    time.sleep(0.5)
    pulse(port, ["rts"])
    got = listen(port, 10, ask_at=5)
    say(f"  then 10 s: {len(got)} bytes {got[:24].hex()}")
    port.close()
    PY
    sudo systemctl start jetrover-arm-control
    sleep 12
    echo "--- the driver's first stats"; journalctl -u jetrover-arm-control --no-pager -n 40 | grep '\[stats' | tail -1 | cut -c1-300
    REMOTE

# Experiment, for a board that has stopped answering and survived robot-board-usb-reset: cut the USB port power of only the board's USB-serial chip for `off_s` seconds and restore it -- a real power reset of the chip (it is powered from USB, bmAttributes 0xa0), while the board's controller and the servos stay on battery. Uses the hub's per-port power switching (ClearPortFeature/SetPortFeature PORT_POWER), as uhubctl does. Runs detached on the Jetson, so the port comes back on even if the hub turns out to switch every port together and the Wi-Fi drops with it; the log is ~/board-port-power.log. Stops the servo driver first and starts it again after, so someone must be at the robot
[group('robot: deploy and restart (someone at the robot)')]
robot-board-usb-power-cycle off_s="3" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    ssh -o StrictHostKeyChecking=accept-new "$target" "cat > /tmp/board-port-power.sh" <<'REMOTE'
    set -u
    off_s=$1
    dev=$(dirname "$(readlink -f /sys/class/tty/ttyACM0/device)")
    name=$(basename "$dev"); hub=$(dirname "$dev"); port=${name##*.}
    hubnode=$(printf '/dev/bus/usb/%03d/%03d' "$(cat "$hub/busnum")" "$(cat "$hub/devnum")")
    echo "$(date +%T) chip $name ($(cat "$dev/idVendor"):$(cat "$dev/idProduct")) on port $port of hub $(basename "$hub") ($hubnode)"
    for n in /sys/class/net/*; do p=$(readlink -f "$n/device" 2>/dev/null); case "$p" in *"/$(basename "$hub")/"*|*"/$(basename "$hub").") echo "$(date +%T) note: $(basename "$n") is behind the same hub ($p)";; esac; done
    systemctl stop jetrover-arm-control
    feature() {  # $1: 1 = ClearPortFeature, 3 = SetPortFeature; PORT_POWER = 8
        python3 - "$hubnode" "$1" "$port" <<'PY'
    import ctypes, fcntl, os, sys
    class Ctrl(ctypes.Structure):
        _fields_ = [("bRequestType", ctypes.c_uint8), ("bRequest", ctypes.c_uint8), ("wValue", ctypes.c_uint16),
                    ("wIndex", ctypes.c_uint16), ("wLength", ctypes.c_uint16), ("timeout", ctypes.c_uint32), ("data", ctypes.c_void_p)]
    USBDEVFS_CONTROL = (3 << 30) | (ctypes.sizeof(Ctrl) << 16) | (ord("U") << 8) | 0
    c = Ctrl(0x23, int(sys.argv[2]), 8, int(sys.argv[3]), 0, 1000, None)
    fd = os.open(sys.argv[1], os.O_RDWR)
    try:
        fcntl.ioctl(fd, USBDEVFS_CONTROL, c)
        print("ok")
    except OSError as e:
        print(f"failed: {e}")
    finally:
        os.close(fd)
    PY
    }
    echo "$(date +%T) port power off: $(feature 1)"
    sleep "$off_s"
    echo "$(date +%T) after ${off_s} s: chip in sysfs: $([ -e "$dev/idVendor" ] && echo yes || echo no); ttyACM: $(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
    echo "$(date +%T) port power on: $(feature 3)"
    for i in $(seq 1 40); do [ -e /dev/ttyACM0 ] && break; sleep 0.25; done
    sleep 1
    echo "$(date +%T) ttyACM now: $(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
    journalctl -k --no-pager --since "-$((off_s + 15)) s" | grep -E "usb 1-|cdc_acm|ttyACM" | tail -8
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONPATH="$sdk" python3 - <<'PY'
    import time
    import serial
    from jetrover_arm_control.board_manager.robot_controller_sdk import checksum_crc8
    buf = [0xAA, 0x55, 5, 2, 0x05, 1]
    request = bytes(buf + [checksum_crc8(bytes(buf[2:]))])
    port = serial.Serial(None, 1000000, timeout=0.1)
    port.rts = False
    port.dtr = False
    port.port = "/dev/ttyACM0"
    port.open()
    got = b""
    end = time.time() + 6.0
    while time.time() < end:
        got += port.read(256)
    unprompted = len(got)
    port.write(request)
    end = time.time() + 2.0
    while time.time() < end:
        got += port.read(256)
    port.close()
    print(f"{time.strftime('%T')} unprompted in 6 s: {unprompted} bytes; with the request: {len(got)} bytes {got[:24].hex()}")
    print(f"{time.strftime('%T')} RESULT: " + ("the board answers after a power reset of its USB chip" if got else "still silent after a power reset of its USB chip"))
    PY
    systemctl start jetrover-arm-control
    sleep 12
    journalctl -u jetrover-arm-control --no-pager -n 40 | grep '\[stats' | tail -1 | cut -c1-300
    echo "$(date +%T) done"
    REMOTE
    ssh "$target" "rm -f ~/board-port-power.log; sudo setsid nohup bash /tmp/board-port-power.sh {{off_s}} > ~/board-port-power.log 2>&1 < /dev/null &"
    for i in $(seq 1 30); do
        sleep 2
        if out=$(ssh -o ConnectTimeout=5 "$target" 'cat ~/board-port-power.log' 2>/dev/null) && grep -q " done$" <<<"$out"; then echo "$out"; exit 0; fi
    done
    echo "no 'done' in ~/board-port-power.log after 60 s; what it has so far:"; ssh -o ConnectTimeout=5 "$target" 'cat ~/board-port-power.log' || echo "(the robot is unreachable: the Wi-Fi may be behind that hub; it powers the port back on by itself)"
    exit 1

# Is the board's controller still receiving and executing commands, whatever it sends back? Stops the servo driver, then three times, 4 s apart: three short beeps (2 kHz) and a fast blink of the board's LED, sent with the SDK's own set_buzzer/set_led straight to the serial port (not the /robot/buzzer topic, which crashes the driver), and prints any bytes the board sends meanwhile. Someone at the robot listens: beeps mean the controller runs and receives, so a silent board's fault is in its transmit path. Starts the driver again after, so someone must be at the robot
[group('robot: deploy and restart (someone at the robot)')]
robot-board-beep remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    sudo systemctl stop jetrover-arm-control
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONUNBUFFERED=1 PYTHONPATH="$sdk" python3 - <<'PY'
    import time
    from jetrover_arm_control.board_manager.robot_controller_sdk import Board
    board = Board(device="/dev/ttyACM0", timeout=0.05)
    got = b""
    for n in range(3):
        print(f"{time.strftime('%T')} beep {n + 1}/3: three short beeps, LED blinking")
        board.set_buzzer(2000, 0.12, 0.12, 3)
        board.set_led(0.1, 0.1, 10)
        end = time.time() + 4
        while time.time() < end:
            got += board.port.read(256)
    board.port.close()
    print(f"{time.strftime('%T')} the board sent {len(got)} bytes meanwhile {got[:24].hex()}")
    PY
    sudo systemctl start jetrover-arm-control
    REMOTE

# Moves ONE servo, the base (motor 1) or the gripper (motor 10): is a silent board's controller still executing commands? Writes the given pulses straight to the board's serial port with the SDK's bus_servo_set_position, one every 3 s, each over 1.5 s, beside the running driver (no driver restart, so the shoulder cannot sag), and prints the robot's clock at each so camera frames (just camera-view) can be matched to it. The camera sees the base turn the whole arm; it cannot see the gripper. Limits: the base within 440..560 (+-14 degrees of its centre, 500), the gripper within its open..closed 116..645. Nothing else moves. If the servo moves while the board sends nothing back, its controller receives and runs and only its transmit path is dead. Confirm the arm is clear before running
[group('robot: deploy and restart (someone at the robot)')]
robot-board-servo-test motor pulses remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    case "{{motor}}" in 1) lo=440 hi=560 ;; 10) lo=116 hi=645 ;; *) echo "error: motor {{motor}}: only the base (1) or the gripper (10)" >&2; exit 2 ;; esac
    pulses=$(echo "{{pulses}}" | tr ',' ' ')
    for p in $pulses; do
        if ! [[ "$p" =~ ^[0-9]+$ ]] || [ "$p" -lt $lo ] || [ "$p" -gt $hi ]; then echo "error: pulse $p for motor {{motor}} is outside $lo..$hi" >&2; exit 2; fi
    done
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<REMOTE
    sdk=\$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONUNBUFFERED=1 PYTHONPATH="\$sdk" python3 - {{motor}} $pulses <<'PY'
    import sys, time
    from jetrover_arm_control.board_manager.robot_controller_sdk import Board
    board = Board(device="/dev/ttyACM0", timeout=0.05)
    motor, got = int(sys.argv[1]), b""
    for pulse in map(int, sys.argv[2:]):
        print(f"{time.strftime('%T')} motor {motor} to pulse {pulse} over 1.5 s")
        board.bus_servo_set_position(1.5, [[motor, pulse]])
        end = time.time() + 3
        while time.time() < end:
            got += board.port.read(256)
    board.port.close()
    print(f"{time.strftime('%T')} the board sent {len(got)} bytes meanwhile {got[:24].hex()}")
    PY
    REMOTE

# Experiment, for a silent board: start the STM32's ROM bootloader through the board's USB-serial port with the auto-download sequence of Hiwonder's circuit (RTS asserted -> BOOT0 high; RTS asserted with DTR released -> reset; from the RRC Lite schematic; wiring=swapped exchanges DTR and RTS), and if it answers, read -- never write or erase -- the controller's chip id, its reset-cause flags (RCC_CSR) and the live levels of its IMU I2C pins (PB10 SCL, PB11 SDA) and bus-servo pins (PC7 RX, PG6 TX, PE7), then reset it back into its firmware and listen for it. Readout Unprotect, Write and Erase are never sent. Stops the servo driver for the duration and starts it again; the controller's reset may drop the servos' torque, so confirm the arm may sag before running
[group('robot: deploy and restart (someone at the robot)')]
robot-board-bootloader dev="/dev/ttyACM0" wiring="hiwonder" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    sudo systemctl stop jetrover-arm-control
    sudo python3 - "{{dev}}" "{{wiring}}" <<'PY'
    import fcntl, os, struct, sys, termios, time
    dev, wiring = sys.argv[1], sys.argv[2]
    if wiring not in ("hiwonder", "swapped"):
        sys.exit(f"wiring must be hiwonder (RTS -> BOOT0, RTS with DTR released -> reset) or swapped (DTR and RTS exchanged), not {wiring}")
    TIOCMSET, DTR, RTS = 0x5418, 0x002, 0x004
    ACK, NACK = 0x79, 0x1F

    def say(text):
        print(f"{time.strftime('%T')} {text}", flush=True)

    def lines(dtr, rts):  # both at once, so no half-step resets the controller by accident
        if wiring == "swapped":
            dtr, rts = rts, dtr
        fcntl.ioctl(fd, TIOCMSET, struct.pack("i", (DTR if dtr else 0) | (RTS if rts else 0)))

    def mode(baud, even):
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | (termios.PARENB if even else 0)
        a[4] = a[5] = getattr(termios, f"B{baud}")
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIOFLUSH)

    def read(n, seconds=1.0):
        got, end = b"", time.time() + seconds
        while len(got) < n and time.time() < end:
            try:
                got += os.read(fd, n - len(got))
            except BlockingIOError:
                time.sleep(0.002)
        return got

    def command(code):
        os.write(fd, bytes([code, code ^ 0xFF]))
        return read(1)

    def read_word(address):
        if command(0x11) != bytes([ACK]):
            return None, "READ MEMORY refused (read protection?)"
        a = struct.pack(">I", address)
        os.write(fd, a + bytes([a[0] ^ a[1] ^ a[2] ^ a[3]]))
        if read(1) != bytes([ACK]):
            return None, "address refused"
        os.write(fd, bytes([3, 3 ^ 0xFF]))
        if read(1) != bytes([ACK]):
            return None, "length refused"
        data = read(4)
        return (struct.unpack("<I", data)[0], "") if len(data) == 4 else (None, f"short read {data.hex()}")

    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)  # opening asserts DTR and RTS: BOOT0 high, no reset
    try:
        say("entry: reset with BOOT0 high (RTS, DTR released), then release the reset (DTR) and wait")
        lines(dtr=False, rts=True)
        time.sleep(0.1)
        lines(dtr=True, rts=True)
        time.sleep(0.3)
        mode(115200, even=True)
        os.write(fd, b"\x7f")
        first = read(1)
        say(f"0x7F -> {first.hex() or 'nothing'}" + ("  (ACK: the ROM bootloader is running)" if first == bytes([ACK]) else ""))
        if first in (bytes([ACK]), bytes([NACK])):
            if command(0x00) == bytes([ACK]):
                n = read(1)
                body = read(n[0] + 1) if n else b""
                read(1)
                say(f"GET: bootloader version {body[:1].hex()}, commands {body[1:].hex(' ')}")
            if command(0x02) == bytes([ACK]):
                n = read(1)
                pid = read(n[0] + 1) if n else b""
                read(1)
                say(f"GET ID: product 0x{pid.hex()} (0x0413 is the STM32F405/407)")
            names = [("RCC_CSR", 0x40023874), ("GPIOB_IDR", 0x40020410), ("GPIOC_IDR", 0x40020810),
                     ("GPIOD_IDR", 0x40020C10), ("GPIOE_IDR", 0x40021010), ("GPIOG_IDR", 0x40021810)]
            words = {}
            for name, address in names:
                value, why = read_word(address)
                words[name] = value
                say(f"{name} @0x{address:08x}: " + (f"0x{value:08x}" if value is not None else why))
                if value is None:
                    break
            if words.get("RCC_CSR") is not None:
                flags = {31: "low-power", 30: "window watchdog", 29: "independent watchdog", 28: "software",
                         27: "power-on/down", 26: "reset pin", 25: "brown-out"}
                csr = words["RCC_CSR"]
                say("reset causes recorded since they were last cleared: " + ", ".join(v for b, v in flags.items() if csr >> b & 1))
            bit = lambda name, n: None if words.get(name) is None else (words[name] >> n) & 1
            if bit("GPIOB_IDR", 10) is not None:
                say(f"IMU I2C: SCL (PB10) {bit('GPIOB_IDR', 10)}, SDA (PB11) {bit('GPIOB_IDR', 11)}  (idle both 1; SDA 0 = a sensor holding the bus)")
            if bit("GPIOC_IDR", 7) is not None:
                say(f"bus servo: RX (PC7) {bit('GPIOC_IDR', 7)}, TX (PG6) {bit('GPIOG_IDR', 6)}, PE7 {bit('GPIOE_IDR', 7)}  (an idle bus reads 1)")
            if bit("GPIOD_IDR", 9) is not None:
                say(f"host UART3 pins: PD8 {bit('GPIOD_IDR', 8)}, PD9 {bit('GPIOD_IDR', 9)}")
        say("exit: reset with BOOT0 low -- reset (RTS, DTR released), then release both")
        lines(dtr=False, rts=True)
        time.sleep(0.1)
        lines(dtr=False, rts=False)
        mode(1000000, even=False)
        got = read(4096, 8.0)
        say(f"firmware, 8 s after the reset: {len(got)} bytes {got[:24].hex()}")
    finally:
        lines(dtr=False, rts=False)
        os.close(fd)
    PY
    sudo systemctl start jetrover-arm-control
    sleep 12
    journalctl -u jetrover-arm-control --no-pager -n 40 | grep '\[stats' | tail -1 | cut -c1-260
    REMOTE

# Moves nothing, makes noise: does a silent board still execute commands that do not touch the servos? Records 15 s from the robot's microphone array (iFlytek) while it plays two control tones on the robot's speaker (1 kHz at 1 s, 1.5 kHz at 12.5 s: proves the microphones hear the robot's surroundings) and sends the board buzzer commands (2 kHz x3 at 3 s, 3 kHz x3 at 6 s, a UART break then 2 kHz x3 at 9 s) on its serial port beside the running driver, then prints, per 100 ms, which tone bands rose above the noise. The audio is analysed on the robot and deleted; only band levels come back. Opens the port without toggling the reset lines (both released at once)
[group('robot: checks and diagnostics (nothing moves)')]
robot-board-audio-test dev="/dev/ttyACM0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "bash -s" <<'REMOTE'
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    # OPENBLAS_CORETYPE: numpy's OpenBLAS dies with SIGILL on the Nano's Cortex-A57 without it.
    sudo env PYTHONPATH="$sdk" PYTHONUNBUFFERED=1 OPENBLAS_CORETYPE=ARMV8 python3 - "{{dev}}" <<'PY'
    import fcntl, math, os, struct, subprocess, sys, termios, time, wave
    import numpy as np
    from jetrover_arm_control.board_manager.robot_controller_sdk import checksum_crc8
    dev = sys.argv[1]
    MICS = {"iflytek": "plughw:CARD=XFMDPV0018,DEV=0"}  # the C-Media dongle (the speaker) has no microphone
    SPEAKER = "plughw:CARD=Device,DEV=0"
    RATE, SECONDS = 16000, 15

    def tone_file(path, freq, secs, amp=0.25, rate=48000):
        with wave.open(path, "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
            n = int(secs * rate)
            w.writeframes(b"".join(struct.pack("<h", int(amp * 32767 * math.sin(2 * math.pi * freq * i / rate) * min(1, i / 480, (n - i) / 480))) for i in range(n)))

    def frame(func, data):
        body = [func, len(data)] + list(data)
        return bytes([0xAA, 0x55] + body + [checksum_crc8(bytes(body))])

    def buzzer(fd, freq, on_ms=300, off_ms=200, repeat=3):
        os.write(fd, frame(2, struct.pack("<HHHH", freq, on_ms, off_ms, repeat)))

    tone_file("/tmp/at_1k.wav", 1000, 0.4); tone_file("/tmp/at_1k5.wav", 1500, 0.4)
    recs = {name: subprocess.Popen(["arecord", "-q", "-D", d, "-f", "S16_LE", "-r", str(RATE), "-c", "1", "-d", str(SECONDS), f"/tmp/at_{name}.wav"],
                                   stderr=subprocess.PIPE) for name, d in MICS.items()}
    t0 = time.time()
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    fcntl.ioctl(fd, 0x5418, struct.pack("i", 0))  # TIOCMSET: DTR and RTS released together, so no reset
    events = [(1.0, "speaker 1 kHz (control)", lambda: subprocess.Popen(["aplay", "-q", "-D", SPEAKER, "/tmp/at_1k.wav"])),
              (3.0, "board buzzer 2 kHz x3", lambda: buzzer(fd, 2000)),
              (6.0, "board buzzer 3 kHz x3", lambda: buzzer(fd, 3000)),
              (9.0, "UART break", lambda: termios.tcsendbreak(fd, 0)),
              (9.7, "board buzzer 2 kHz x3", lambda: buzzer(fd, 2000)),
              (12.5, "speaker 1.5 kHz (control)", lambda: subprocess.Popen(["aplay", "-q", "-D", SPEAKER, "/tmp/at_1k5.wav"]))]
    for at, label, act in events:
        time.sleep(max(0, at - (time.time() - t0)))
        try:
            act(); print(f"t={time.time() - t0:5.2f} s  {label}")
        except Exception as e:
            print(f"t={time.time() - t0:5.2f} s  {label}: FAILED {e}")
    os.close(fd)
    for name, r in recs.items():
        r.wait()
        err = r.stderr.read().decode().strip()
        if err: print(f"{name} recording: {err[:200]}")
    bands = {"1k": 1000, "1k5": 1500, "2k": 2000, "3k": 3000, "2.4-3.2k": None}
    for name in MICS:
        path = f"/tmp/at_{name}.wav"
        if not os.path.exists(path) or os.path.getsize(path) < 1000:
            print(f"--- {name}: no recording"); continue
        with wave.open(path) as w:
            x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768
        os.remove(path)
        n, hop = 2048, 1600  # 0.128 s windows every 0.1 s
        win = np.hanning(n)
        f = np.fft.rfftfreq(n, 1 / RATE)
        frames = [np.abs(np.fft.rfft(x[i:i + n] * win)) ** 2 for i in range(0, len(x) - n, hop)]
        S = np.array(frames)
        def band(c):
            m = (f > 2400) & (f < 3200) if c is None else (f > c - 60) & (f < c + 60)
            return 10 * np.log10(S[:, m].sum(axis=1) + 1e-12)
        print(f"--- {name}: {len(x) / RATE:.1f} s, rms {20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-9):.0f} dBFS; tone bands more than 12 dB above their median:")
        hits = []
        for label, c in bands.items():
            e = band(c); floor = np.median(e)
            for i in np.where(e > floor + 12)[0]:
                hits.append((i * hop / RATE, label, e[i] - floor))
        if not hits:
            print("    none")
        for t, label, db in sorted(hits):
            print(f"    t={t:5.1f} s  {label:<9} +{db:4.0f} dB")
    PY
    sudo rm -f /tmp/at_1k.wav /tmp/at_1k5.wav
    REMOTE

# Experiment: cut one port of the Jetson's USB hub (1-2, the Nano devkit's RTS5411) for `off_s` seconds with the port claimed first (USBDEVFS_CLAIM_PORT, as uhubctl does), so the kernel's hub driver cannot power it back on or re-enumerate it while it is off. Reports, every half second while off, the hub's view of the port (connected / power) and whether the device is still in sysfs: a device that stays "connected" with its port unpowered is powered from somewhere else. Port 2 is the servo board's USB chip: the driver is stopped first, the board is asked for a servo position after power returns, and the driver is started again. The Wi-Fi is on PCIe, so no hub port carries the connection
[group('robot: checks and diagnostics (nothing moves)')]
robot-hub-port-test port="4" off_s="10" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "sudo bash -s {{port}} {{off_s}}" <<'REMOTE'
    port=$1; off_s=$2; hub=/sys/bus/usb/devices/1-2; dev=$hub/1-2.$port
    [ -e "$dev/idVendor" ] || { echo "nothing on hub 1-2 port $port"; exit 2; }
    for n in /sys/class/net/*; do case "$(readlink -f "$n/device" 2>/dev/null)" in */1-2/*|*/1-2.*) echo "refusing: $(basename "$n") is behind hub 1-2"; exit 2;; esac; done
    # The board's chip by its USB id, not by ttyACM0, which is missing whenever cdc_acm is not bound.
    board=""; for d in $hub/1-2.*; do [ "$(cat "$d/idVendor" 2>/dev/null):$(cat "$d/idProduct" 2>/dev/null)" = "1a86:55d4" ] && board=$(basename "$d"); done
    echo "$(date +%T) port $port: $(cat "$dev/idVendor"):$(cat "$dev/idProduct") $(cat "$dev/product" 2>/dev/null), bmAttributes $(cat "$dev/bmAttributes")$([ "1-2.$port" = "$board" ] && echo ' -- the servo board')"
    [ "1-2.$port" = "$board" ] && systemctl stop jetrover-arm-control
    hubnode=$(printf '/dev/bus/usb/%03d/%03d' "$(cat "$hub/busnum")" "$(cat "$hub/devnum")")
    since=$(date '+%Y-%m-%d %H:%M:%S')
    python3 - "$hubnode" "$port" "$off_s" "$dev" <<'PY'
    import ctypes, fcntl, os, sys, time
    hubnode, port, off_s, dev = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    class Ctrl(ctypes.Structure):
        _fields_ = [("bRequestType", ctypes.c_uint8), ("bRequest", ctypes.c_uint8), ("wValue", ctypes.c_uint16),
                    ("wIndex", ctypes.c_uint16), ("wLength", ctypes.c_uint16), ("timeout", ctypes.c_uint32), ("data", ctypes.c_void_p)]
    CONTROL = (3 << 30) | (ctypes.sizeof(Ctrl) << 16) | (ord("U") << 8) | 0
    CLAIM_PORT, RELEASE_PORT = (2 << 30) | (4 << 16) | (ord("U") << 8) | 24, (2 << 30) | (4 << 16) | (ord("U") << 8) | 25
    fd = os.open(hubnode, os.O_RDWR)
    t0 = time.time()
    def say(text): print(f"{time.strftime('%T')} +{time.time() - t0:5.2f} s  {text}", flush=True)
    def status():
        buf = (ctypes.c_uint8 * 4)()
        fcntl.ioctl(fd, CONTROL, Ctrl(0xA3, 0, 0, port, 4, 1000, ctypes.cast(buf, ctypes.c_void_p)))
        st = buf[0] | buf[1] << 8
        return f"hub says connected {st & 1}, power {st >> 8 & 1}; device in sysfs: {'yes' if os.path.exists(dev + '/idVendor') else 'no'}"
    def power(on):
        fcntl.ioctl(fd, CONTROL, Ctrl(0x23, 3 if on else 1, 8, port, 0, 1000, None))  # Set/ClearPortFeature(PORT_POWER)
    claimed = False
    try:
        fcntl.ioctl(fd, CLAIM_PORT, ctypes.c_uint(port)); claimed = True
        say(f"claimed port {port}; {status()}")
        power(False)
        say("power OFF")
        end = time.time() + off_s
        while time.time() < end:
            time.sleep(0.5)
            say(status())
        power(True)
        say("power ON")
        time.sleep(0.5)
        say(status())
    finally:
        if claimed:
            fcntl.ioctl(fd, RELEASE_PORT, ctypes.c_uint(port))
        os.close(fd)
    say("port released to the kernel")
    PY
    for i in $(seq 1 40); do sleep 0.25; [ -e "$dev/idVendor" ] && break; done
    sleep 1
    # A device that enumerates while the port is still claimed is left unconfigured (the kernel does not
    # configure a device user space owns): configure it, so its drivers (cdc_acm, usbhid) bind again.
    if [ -e "$dev/idVendor" ] && [ -z "$(cat "$dev/bConfigurationValue" 2>/dev/null)" ]; then
        echo 1 > "$dev/bConfigurationValue" && echo "$(date +%T) configured it (it came up while the port was claimed)"
        for i in $(seq 1 20); do sleep 0.25; [ "1-2.$port" != "$board" ] || [ -e /dev/ttyACM0 ] && break; done
    fi
    echo "$(date +%T) after: device in sysfs: $([ -e "$dev/idVendor" ] && echo yes || echo no); interfaces: $(cat "$dev/bNumInterfaces" 2>/dev/null)$([ "1-2.$port" = "$board" ] && echo "; ttyACM0: $([ -e /dev/ttyACM0 ] && echo yes || echo no)")"
    journalctl -k --no-pager --since "$since" | grep -E "usb 1-2\.$port[: ]" | cut -c1-110
    if [ "1-2.$port" = "$board" ]; then
        sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
        PYTHONPATH="$sdk" python3 - <<'PY'
    import fcntl, os, struct, time
    from jetrover_arm_control.board_manager.robot_controller_sdk import checksum_crc8
    import termios
    fd = os.open("/dev/ttyACM0", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    fcntl.ioctl(fd, 0x5418, struct.pack("i", 0))
    a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B1000000
    termios.tcsetattr(fd, termios.TCSANOW, a)
    got, end = b"", time.time() + 6
    while time.time() < end:
        try: got += os.read(fd, 4096)
        except BlockingIOError: time.sleep(0.01)
    body = [5, 2, 0x05, 1]
    os.write(fd, bytes([0xAA, 0x55] + body + [checksum_crc8(bytes(body))]))
    end = time.time() + 2
    while time.time() < end:
        try: got += os.read(fd, 4096)
        except BlockingIOError: time.sleep(0.01)
    os.close(fd)
    print(f"{time.strftime('%T')} board: {len(got)} bytes in 8 s {got[:24].hex()}  ->  " + ("ANSWERS" if got else "still silent"))
    PY
        systemctl start jetrover-arm-control
    fi
    REMOTE

# Restart the servo bus driver (jetrover-arm-control), which reopens its serial link to the Hiwonder board. For when the board stopped answering (just ros-check shows no feedback_array or battery messages) after a brown-out or a reset. It interrupts the arm, and the shoulder may sag while the driver comes back (seen 2026-10-12), so someone must be at the robot. Afterwards the feedback should flow again
[group('robot: deploy and restart (someone at the robot)')]
restart-arm-driver remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "sudo systemctl restart jetrover-arm-control; sleep 5; systemctl status jetrover-arm-control --no-pager -n 8"
    just ros-check 4

# Switch off as much of the robot as software can, then halt the Jetson (the arm goes limp: someone holds it and lowers it). Stops an onboard pilot run and every service (RobotHost, CameraHost, relay, servo driver) so nothing commands the arm, switches off torque on every arm servo (1-5 and the gripper, 10; 0x0B, read back) and stops the wheel motors, then `sudo poweroff`, so the SD card is not cut mid-write. Nothing in the board's protocol powers the board itself off, so the board, the servos' own electronics and its low-battery alarm stay on battery until someone flips the power switch; with torque off they draw little. If the servo board has gone silent, run just robot-board-diagnose first (it needs the fault present): torque cannot be switched off then, and the shutdown says so and halts the Jetson anyway
[group('robot: deploy and restart (someone at the robot)')]
robot-shutdown remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -uo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    echo "This switches off torque on every arm servo: the ARM FALLS. Someone must hold it and lower it."
    read -r -p "Someone is holding the arm? Enter to shut down, Ctrl+C to abort " _
    ssh -o StrictHostKeyChecking=accept-new "$target" "bash -s" <<'REMOTE'
    touch ~/recordings/.pilot-stop 2>/dev/null
    p=$(cat ~/recordings/.pilot-onboard.pid 2>/dev/null)
    for i in $(seq 1 20); do { [ -n "$p" ] && kill -0 "$p" 2>/dev/null; } || break; sleep 0.5; done
    rm -f ~/recordings/.pilot-stop
    sudo systemctl stop teleop-robothost teleop-camerahost jetrover-relay jetrover-arm-control
    sdk=$(ls -d /home/jetson/teleop_ros_current/install/jetrover_arm_control/lib/python3*/site-packages | head -1)
    PYTHONUNBUFFERED=1 PYTHONPATH="$sdk" timeout 30 python3 - <<'PY'
    import time
    from jetrover_arm_control.board_manager.robot_controller_sdk import Board, PacketFunction

    names = {1: "base yaw", 2: "shoulder", 3: "elbow", 4: "wrist pitch", 5: "wrist roll", 10: "gripper"}
    board = Board()
    board.enable_reception()
    time.sleep(0.5)
    board.set_motor_speed([[m, 0.0] for m in (1, 2, 3, 4)])
    for n in (5, 4, 3, 2, 1):
        print(f"torque off in {n} s: hold the arm", flush=True)
        time.sleep(1)
    still_on = []
    for i in names:
        # Torque off is sub-command 0x0B on this board (the opposite of Hiwonder's published SDK,
        # measured 2026-10-21). Sent raw, so it does not depend on which SDK version is deployed.
        board.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [0x0B, i])
        time.sleep(0.02)
        got = board.bus_servo_read_torque_state(i)
        state = got[0] if got else None
        print(f"  {names[i]} (motor {i}): torque {'off' if state == 0 else 'NO ANSWER' if state is None else 'STILL ON'}", flush=True)
        if state != 0:
            still_on.append(names[i])
    print("TORQUE OFF on every arm servo." if not still_on else
          f"NOT CONFIRMED OFF: {', '.join(still_on)} (board silent? just robot-board-diagnose before the switch)", flush=True)
    PY
    echo "--- halting the Jetson"
    sudo poweroff
    REMOTE
    echo "Jetson halting (allow ~20 s). The board, the servos' electronics and the low-battery alarm are still on battery: flip the power switch."

# Read-only: the battery reading jetrover-arm-control publishes (millivolts, from the Hiwonder board), plus the Jetson's own supply rails. A continuous beep from the robot is usually the board's low-battery alarm: check this first
[group('robot: checks and diagnostics (nothing moves)')]
robot-battery remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "r=/home/jetson/teleop_ros_current; [ -d \$r/src ] || r=/home/jetson/projects/jetrover-teleop-ros; f=\$r/src/jetrover_arm_control/jetrover_arm_control/robot_controller_node.py; \
         topic=\$(grep -o -E \"create_publisher\\(Int32, *['\\\"][^'\\\"]+\" \$f | head -1 | sed -E \"s/.*['\\\"]//\"); \
         echo \"battery topic: \${topic:-not found}\"; grep -n -E 'create_timer|def getBattery|getBattery\\(' \$f \$r/src/jetrover_arm_control/jetrover_arm_control/*.py 2>/dev/null | grep -i -E 'batter|timer' | head -6; \
         source /opt/ros/foxy/setup.bash; export ROS_LOCALHOST_ONLY=1; \
         echo '--- battery messages for 6 s (published every 5 s; none means the board did not answer, or ROS discovery was too slow):'; [ -n \"\$topic\" ] && (timeout 6 ros2 topic echo \"\$topic\" 2>&1 | head -4); \
         echo '--- Jetson supply (mV, mA):'; for d in /sys/bus/i2c/drivers/ina3221x/*/iio:device*; do \
           for c in 0 1 2; do n=\$(cat \$d/rail_name_\$c 2>/dev/null) && echo \"\$n: \$(cat \$d/in_voltage\${c}_input 2>/dev/null) mV, \$(cat \$d/in_current\${c}_input 2>/dev/null) mA\"; done; done; true"

# Build the ROS 2 workspace in robot/ros/ on the Jetson and switch the two ROS services to it (docs/adr/0019): copies the sources to a new ~/teleop_ros_deploy_<time>/, builds it there with colcon, runs its plain-Python tests there, then points ~/teleop_ros_current at it, installs robot/systemd's two ROS units (they source ~/teleop_ros_current) and restarts them. `just deploy-ros false` only uploads and builds, touching nothing that runs. Restarting interrupts the arm (the shoulder may sag), so someone must be at the robot. Roll back by pointing ~/teleop_ros_current at an earlier deploy and restarting the two services
[group('robot: deploy and restart (someone at the robot)')]
deploy-ros activate="true" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{activate}}" in true|false) ;; *) echo "error: activate must be true or false, got '{{activate}}'" >&2; exit 1 ;; esac
    host=$(just _robot-host "{{remote_host}}")
    target="{{remote_user}}@$host"
    case "$host" in *:*) scp_dest="{{remote_user}}@[$host]" ;; *) scp_dest="$target" ;; esac
    remote_dir="teleop_ros_deploy_$(date +%Y%m%d-%H%M%S)"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"' EXIT
    tar czf "$archive" -C robot/ros --exclude=build --exclude=install --exclude=log --exclude='__pycache__' .
    scp -q -o StrictHostKeyChecking=accept-new "$archive" "${scp_dest}:/tmp/${remote_dir}.tar.gz"
    scp -q -o StrictHostKeyChecking=accept-new robot/systemd/jetrover-relay.service robot/systemd/jetrover-arm-control.service "${scp_dest}:/tmp/"
    echo "Building in ~/${remote_dir} (a few minutes on the Jetson)..." >&2
    ssh -o StrictHostKeyChecking=accept-new "$target" "
        set -e
        mkdir -p ~/${remote_dir} && tar xzf /tmp/${remote_dir}.tar.gz -C ~/${remote_dir} && rm -f /tmp/${remote_dir}.tar.gz
        cd ~/${remote_dir} && source /opt/ros/foxy/setup.bash && colcon build 2>&1 | tail -4
        test -f ~/${remote_dir}/install/setup.bash
        echo '--- the plain-Python tests (relay protocol, stale tracking, servo controller, read backoff, session log):'
        PYTHONPATH=src/teleop_relay:src/jetrover_arm_control python3 -m pytest -q -p no:cacheprovider \
            src/teleop_relay/test/test_relay_protocol.py src/teleop_relay/test/test_stale_tracked_value.py \
            src/jetrover_arm_control/test/test_servo_controller.py src/jetrover_arm_control/test/test_read_backoff.py \
            src/jetrover_arm_control/test/test_session_log.py 2>&1 | tail -6
        exit \${PIPESTATUS[0]}
    "
    # The build every recorded session's session.json names (docs/adr/0021): ~/teleop_ros_current/BUILD_INFO.
    printf '{"sha": "%s", "dirty": %s}\n' "$(git rev-parse HEAD)" "$([ -n "$(git status --porcelain --untracked-files=no -- robot/ros)" ] && echo true || echo false)" \
        | ssh -o StrictHostKeyChecking=accept-new "$target" "cat > ~/${remote_dir}/BUILD_INFO"
    if [ "{{activate}}" != "true" ]; then
        echo "Built ~/${remote_dir}; nothing running was changed. Activate it with: just deploy-ros"
        exit 0
    fi
    ssh -o StrictHostKeyChecking=accept-new "$target" "
        set -e
        ln -sfn ~/${remote_dir} ~/teleop_ros_current
        sudo cp /tmp/jetrover-relay.service /tmp/jetrover-arm-control.service /etc/systemd/system/
        rm -f /tmp/jetrover-relay.service /tmp/jetrover-arm-control.service
        sudo systemctl daemon-reload
        sudo systemctl restart jetrover-arm-control jetrover-relay
        sleep 6
        systemctl status jetrover-arm-control jetrover-relay --no-pager -n 4
    "
    just ros-check 4

# Print systemctl status for the Jetson-side services (teleop-robothost, jetrover-relay, jetrover-arm-control) in one shot -- see robot/systemd/README.md
[group('robot: checks and diagnostics (nothing moves)')]
robot-status remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "systemctl status teleop-robothost jetrover-relay jetrover-arm-control --no-pager -n 5"

# Read-only: the Jetson's Wi-Fi profiles, visible networks, IPv4/IPv6 addresses, routes and Tailscale state -- moves nothing; run before and after `robot-wifi-join`
[group('robot: checks and diagnostics (nothing moves)')]
robot-net-status remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" 'bash -s' <<'REMOTE'
    echo "== $(hostname): network profiles (name:type:device:autoconnect:priority)"
    nmcli -t -f NAME,TYPE,DEVICE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show 2>&1 || echo "nmcli unavailable"
    echo "== visible Wi-Fi (* = in use; in-use:ssid:signal:security)"
    nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY device wifi list 2>&1 | head -20 || true
    echo "== IPv4"; ip -4 -brief addr show
    echo "== IPv6"; ip -6 -brief addr show
    echo "== default routes"; ip route show default; ip -6 route show default
    echo "== tailscale"; tailscale status 2>&1 | head -5 || true
    echo "== services that may manage Wi-Fi"
    systemctl list-units --type=service --no-pager --plain 2>/dev/null | grep -i -E 'wifi|hotspot|hostapd|wpa_supplicant|NetworkManager|tailscale' || true
    REMOTE

# Read-only: list the Jetson's cameras, grab one frame from `device`, copy it here and print where it went -- moves nothing; exits non-zero if no frame was captured
[group('robot: checks and diagnostics (nothing moves)')]
robot-camera-check device="/dev/video0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    target="{{remote_user}}@$host"
    case "$host" in *:*) scp_src="{{remote_user}}@[$host]" ;; *) scp_src="$target" ;; esac
    remote_file=/tmp/jetrover-camera-check.jpg
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$target" "bash -s -- '{{device}}' '$remote_file'" <<'REMOTE'
    dev="$1"; out="$2"
    rm -f "$out"
    echo "== cameras on $(hostname)"
    if command -v v4l2-ctl >/dev/null 2>&1; then v4l2-ctl --list-devices 2>&1; else ls -l /dev/video* 2>&1 || true; fi
    echo "== USB devices"; lsusb 2>&1 || true
    if [ ! -e "$dev" ]; then
        echo "error: $dev does not exist; pass device=/dev/videoN from the list above" >&2
        exit 1
    fi
    echo "== formats $dev offers (pixel format, sizes, frame rates)"
    if command -v v4l2-ctl >/dev/null 2>&1; then v4l2-ctl -d "$dev" --list-formats-ext 2>&1; else echo "v4l2-ctl missing (apt install v4l-utils)"; fi
    if command -v fuser >/dev/null 2>&1 && fuser "$dev" >/dev/null 2>&1; then
        echo "warning: $dev is already open by another process (a ROS camera node?), so capture may fail:" >&2
        fuser -v "$dev" 2>&1 || true
    fi
    echo "== capturing one frame from $dev"
    # Every capture command reads from /dev/null: this script itself arrives on stdin (`bash -s`), and
    # a tool that reads stdin (ffmpeg does by default) would swallow the rest of it.
    # A CSI camera sits behind the Jetson ISP and needs nvarguscamerasrc; USB (UVC) cameras take plain V4L2.
    if command -v v4l2-ctl >/dev/null 2>&1 && v4l2-ctl -d "$dev" --info 2>/dev/null | grep -qi 'vi-output'; then
        gst-launch-1.0 -q nvarguscamerasrc num-buffers=1 ! 'video/x-raw(memory:NVMM),width=1280,height=720' ! nvjpegenc ! filesink location="$out" </dev/null 2>&1 || true
    fi
    if [ ! -s "$out" ] && command -v ffmpeg >/dev/null 2>&1; then
        ffmpeg -nostdin -hide_banner -loglevel error -f v4l2 -i "$dev" -frames:v 1 -y "$out" 2>&1 || true
    fi
    if [ ! -s "$out" ] && command -v gst-launch-1.0 >/dev/null 2>&1; then
        gst-launch-1.0 -q v4l2src device="$dev" num-buffers=1 ! videoconvert ! jpegenc ! filesink location="$out" </dev/null 2>&1 || true
    fi
    if [ ! -s "$out" ] && python3 -c 'import cv2' >/dev/null 2>&1; then
        python3 - "$dev" "$out" <<'PY' 2>&1 || true
    import sys, cv2
    cap = cv2.VideoCapture(sys.argv[1])
    ok, frame = False, None
    for _ in range(10):  # let auto-exposure settle before keeping a frame
        ok, frame = cap.read()
    cap.release()
    if ok:
        cv2.imwrite(sys.argv[2], frame)
    PY
    fi
    if [ ! -s "$out" ]; then
        echo "error: no frame captured from $dev (tried whichever of nvarguscamerasrc, ffmpeg, GStreamer and OpenCV exist)" >&2
        exit 1
    fi
    ls -l "$out"
    REMOTE
    local_file="${TMPDIR:-/tmp}/jetrover-camera-$(date +%Y%m%d-%H%M%S).jpg"
    scp -q -o StrictHostKeyChecking=accept-new "${scp_src}:${remote_file}" "$local_file"
    echo "frame saved to $local_file"
    if command -v wslpath >/dev/null 2>&1; then echo "Windows path: $(wslpath -w "$local_file")"; fi

# Read-only: publish Teleop.CameraHost, run its timestamp-spike on the Jetson (docs/adr/0014, resolved question 1) and delete it again -- uses the camera for a few seconds, touches no service, moves nothing; exits with the spike's code (0 pass, 1 fail, 2 setup error)
[group('robot: checks and diagnostics (nothing moves)')]
camera-timestamp-spike frames="150" fps="30" device="/dev/video0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    cd core
    # Relative output path for the Windows dotnet, as in deploy-robothost.
    publish_dir="_scratch_camspike"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.CameraHost for linux-arm64..." >&2
    dotnet publish Teleop.CameraHost -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir" --nologo -v quiet
    tar czf "$archive" -C "$publish_dir" .
    target="{{remote_user}}@$host"
    case "$host" in *:*) scp_dest="{{remote_user}}@[$host]" ;; *) scp_dest="$target" ;; esac
    scp -q -o StrictHostKeyChecking=accept-new "$archive" "${scp_dest}:/tmp/camspike.tar.gz"
    set +e
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$target" "
        rm -rf /tmp/camspike && mkdir -p /tmp/camspike && tar xzf /tmp/camspike.tar.gz -C /tmp/camspike &&
        /home/jetson/.dotnet/dotnet /tmp/camspike/Teleop.CameraHost.dll timestamp-spike \
            --device {{device}} --fps {{fps}} --frames {{frames}}
        status=\$?; rm -rf /tmp/camspike /tmp/camspike.tar.gz; exit \$status"
    status=$?
    set -e
    exit "$status"

# Run the camera streaming sender (docs/adr/0014) on the Jetson in the foreground for `seconds`, then delete it again -- uses the camera, opens UDP :6003, touches no service, moves nothing. Pair with `just camera-probe` from this machine. Not a systemd service yet
[group('robot: checks and diagnostics (nothing moves)')]
camera-serve seconds="30" fps="30" device="/dev/video0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    cd core
    publish_dir="_scratch_camserve"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.CameraHost for linux-arm64..." >&2
    dotnet publish Teleop.CameraHost -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir" --nologo -v quiet
    tar czf "$archive" -C "$publish_dir" .
    target="{{remote_user}}@$host"
    case "$host" in *:*) scp_dest="{{remote_user}}@[$host]" ;; *) scp_dest="$target" ;; esac
    scp -q -o StrictHostKeyChecking=accept-new "$archive" "${scp_dest}:/tmp/camserve.tar.gz"
    set +e
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$target" "
        rm -rf /tmp/camserve && mkdir -p /tmp/camserve && tar xzf /tmp/camserve.tar.gz -C /tmp/camserve &&
        /home/jetson/.dotnet/dotnet /tmp/camserve/Teleop.CameraHost.dll serve \
            --device {{device}} --fps {{fps}} --seconds {{seconds}}
        status=\$?; rm -rf /tmp/camserve /tmp/camserve.tar.gz; exit \$status"
    status=$?
    set -e
    exit "$status"

# Subscribe to the camera sender from this machine for `seconds` and report what arrives (frames, rate, sizes, reassembly drops); `save` writes the last frame to a path the (Windows) dotnet can resolve. Exit 1 if no frame arrived -- usually the sender is not running or this machine's firewall blocks inbound UDP on 6004
[group('robot: checks and diagnostics (nothing moves)')]
camera-probe seconds="10" max_fps="0" save="" remote_host=default_jetrover_host:
    cd core && dotnet run --project Teleop.CameraHost -c Release -- probe --host $(just _robot-host "{{remote_host}}") \
        --seconds {{seconds}} --max-fps {{max_fps}} {{ if save != "" { "--save '" + save + "'" } else { "" } }}

# Show the robot's camera live in a window on this machine until Enter -- moves nothing, touches no service: subscribes as the headset does and serves the frames to a browser window at http://localhost:<http_port>/ (Edge's app window when it can). The sender streams to its newest subscriber only, so while this runs the headset loses the feed (close one of the two). Needs the camera sender running (just camera-logs) and inbound UDP on 6004 allowed by this machine's firewall. seconds=0 runs until Enter
[group('robot: checks and diagnostics (nothing moves)')]
camera-view seconds="0" max_fps="0" http_port="8090" remote_host=default_jetrover_host:
    cd core && dotnet run --project Teleop.CameraHost -c Release -- view --host $(just _robot-host "{{remote_host}}") \
        --seconds {{seconds}} --max-fps {{max_fps}} --http-port {{http_port}}

# Publish Teleop.CameraHost for the Jetson, copy it over, write ~/camerahost-run.sh and restart teleop-camerahost if it is installed (see robot/systemd/README.md) -- moves nothing; does not touch teleop-robothost or the ROS nodes. size is the capture size, WIDTHxHEIGHT, e.g. `just deploy-camerahost 1280x720`. The default 640x360 is the camera's whole width (its 4:3 sizes, 640x480 included, are the middle 76% of its 16:9 sensor; docs/adr/0014, 2026-10-25). The camera offers MJPEG at 320x180, 320x240, 640x360, 640x480, 1280x720, 1280x960, 1440x1080 and 1920x1080; the receivers take frames up to 256 KB
[group('robot: deploy and restart (someone at the robot)')]
deploy-camerahost size="640x360" fps="30" device="auto" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{size}}" in [1-9]*x[1-9]*) ;; *) echo "error: size is WIDTHxHEIGHT, e.g. 1280x720 (got '{{size}}')" >&2; exit 1 ;; esac
    width="{{size}}"; width="${width%x*}"; height="{{size}}"; height="${height#*x}"
    case "$width$height" in *[!0-9]*) echo "error: size is WIDTHxHEIGHT, e.g. 1280x720 (got '{{size}}')" >&2; exit 1 ;; esac
    host=$(just _robot-host "{{remote_host}}")
    cd core
    stamp=$(date +%Y%m%d-%H%M%S)
    remote_dir="camerahost_deploy_${stamp}"
    publish_dir="_scratch_camerahost"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.CameraHost for linux-arm64..." >&2
    dotnet publish Teleop.CameraHost -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir" --nologo -v quiet
    tar czf "$archive" -C "$publish_dir" .
    target="{{remote_user}}@$host"
    case "$host" in *:*) scp_dest="{{remote_user}}@[$host]" ;; *) scp_dest="$target" ;; esac
    scp -q -o StrictHostKeyChecking=accept-new "$archive" "${scp_dest}:/tmp/${remote_dir}.tar.gz"
    # Stable wrapper the unit's ExecStart points at, rewritten on every deploy (same pattern as
    # deploy-robothost), so a redeploy needs only a restart. `exec` so SIGTERM reaches the sender.
    # device=auto: the sender finds the DaBai's RGB camera by its stable /dev/v4l/by-id name, and
    # when the camera drops off the USB bus (it does, now and then, coming back as another
    # /dev/videoN) it waits for it and reopens it, instead of exiting until systemd gives up
    # (2026-10-12). Pass a /dev/videoN to pin a device.
    ssh -o StrictHostKeyChecking=accept-new "$target" "cat > /home/jetson/camerahost-run.sh" <<EOF
    #!/bin/bash
    exec /home/jetson/.dotnet/dotnet /home/jetson/camerahost_current/Teleop.CameraHost.dll serve \
        --device {{device}} --width $width --height $height --fps {{fps}} --port 6003
    EOF
    ssh -o StrictHostKeyChecking=accept-new "$target" "
        set -e
        mkdir -p ~/${remote_dir}
        tar xzf /tmp/${remote_dir}.tar.gz -C ~/${remote_dir}
        rm -f /tmp/${remote_dir}.tar.gz
        ln -sfn ~/${remote_dir} ~/camerahost_current
        chmod +x ~/camerahost-run.sh
        if systemctl list-unit-files teleop-camerahost.service | grep -q teleop-camerahost; then
            sudo systemctl restart teleop-camerahost
            sleep 2
            systemctl status teleop-camerahost --no-pager -n 5
        else
            echo 'Deployed, but teleop-camerahost.service is not installed yet: run just install-camerahost-service.'
        fi
    "

# One-time: copy robot/systemd/teleop-camerahost.service to the Jetson, enable it at boot and start it -- run `just deploy-camerahost` first so the wrapper script exists
[group('robot: deploy and restart (someone at the robot)')]
install-camerahost-service remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    target="{{remote_user}}@$host"
    case "$host" in *:*) scp_dest="{{remote_user}}@[$host]" ;; *) scp_dest="$target" ;; esac
    scp -q -o StrictHostKeyChecking=accept-new robot/systemd/teleop-camerahost.service "${scp_dest}:/tmp/teleop-camerahost.service"
    ssh -o StrictHostKeyChecking=accept-new "$target" "
        set -e
        test -x ~/camerahost-run.sh || { echo 'error: ~/camerahost-run.sh missing; run just deploy-camerahost first' >&2; exit 1; }
        sudo cp /tmp/teleop-camerahost.service /etc/systemd/system/teleop-camerahost.service
        rm -f /tmp/teleop-camerahost.service
        sudo systemctl daemon-reload
        sudo systemctl enable --now teleop-camerahost
        sleep 2
        systemctl status teleop-camerahost --no-pager -n 5
    "

# NOT YET RUN AGAINST THE ROBOT. Join the Jetson to another Wi-Fi network, e.g. `just robot-wifi-join "SINRG WIFI"`. The password comes from JETROVER_WIFI_PASSWORD or a prompt, never the command line. The current network stays as an automatic fallback in `revert_minutes` unless `just robot-wifi-confirm` runs first. Changes the robot's network: have someone near the robot
[group('robot: network setup')]
robot-wifi-join ssid revert_minutes="5" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    password="${JETROVER_WIFI_PASSWORD:-}"
    if [ -z "$password" ]; then
        read -r -s -p "Wi-Fi password for '{{ssid}}': " password
        echo
    fi
    target="{{remote_user}}@$host"
    # The script holds no secret. The password reaches it on stdin and is written into a root-only
    # NetworkManager keyfile through `sudo tee`, so it never appears in an argv, `ps`, or sudo's log.
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$target" 'cat > /tmp/jetrover-wifi-join.sh' <<'REMOTE'
    set -euo pipefail
    ssid="$1"; revert_minutes="$2"
    IFS= read -r psk || true
    if [ -z "$psk" ]; then echo "error: no password received" >&2; exit 1; fi
    if ! sudo -n true 2>/dev/null; then echo "error: this needs passwordless sudo on $(hostname)" >&2; exit 1; fi
    if ! nmcli -t general status >/dev/null 2>&1; then
        echo "error: NetworkManager is not running on $(hostname); not switching" >&2
        exit 1
    fi
    dev=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" { print $1; exit }')
    if [ -z "$dev" ]; then echo "error: no Wi-Fi device on $(hostname)" >&2; exit 1; fi
    if [ "$(nmcli -t -f DEVICE,STATE device status | awk -F: -v d="$dev" '$1 == d { print $2; exit }')" = "unmanaged" ]; then
        echo "error: NetworkManager does not manage $dev on $(hostname) (another Wi-Fi manager owns it); not switching" >&2
        exit 1
    fi
    current=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: -v d="$dev" '$2 == d { print $1; exit }')
    if [ "$current" = "$ssid" ]; then echo "$(hostname) is already on '$ssid'"; exit 0; fi
    sudo nmcli device wifi rescan ifname "$dev" >/dev/null 2>&1 || true
    sleep 3
    if ! nmcli -t -f SSID device wifi list ifname "$dev" | grep -Fxq "$ssid"; then
        echo "error: '$ssid' is not visible from $(hostname) right now; not switching" >&2
        exit 1
    fi
    if nmcli -t -f NAME connection show | grep -Fxq "$ssid"; then sudo nmcli connection delete id "$ssid" >/dev/null; fi
    file="/etc/NetworkManager/system-connections/${ssid}.nmconnection"
    sudo tee "$file" >/dev/null <<KEYFILE
    [connection]
    id=${ssid}
    uuid=$(cat /proc/sys/kernel/random/uuid)
    type=wifi
    interface-name=${dev}
    autoconnect=true
    autoconnect-priority=10

    [wifi]
    mode=infrastructure
    ssid=${ssid}

    [wifi-security]
    key-mgmt=wpa-psk
    psk=${psk}

    [ipv4]
    method=auto

    [ipv6]
    method=auto
    KEYFILE
    sudo chown root:root "$file"
    sudo chmod 600 "$file"
    sudo nmcli connection reload
    # Both steps run as transient systemd units so they finish after this ssh session drops.
    sudo systemctl stop jetrover-wifi-revert.timer >/dev/null 2>&1 || true
    sudo systemctl reset-failed jetrover-wifi-revert.timer jetrover-wifi-revert.service jetrover-wifi-join.service >/dev/null 2>&1 || true
    if [ -n "$current" ]; then
        sudo systemd-run --unit=jetrover-wifi-revert --on-active="${revert_minutes}min" /usr/bin/nmcli connection up id "$current" >/dev/null
        echo "fallback: $(hostname) goes back to '$current' in ${revert_minutes} min unless 'just robot-wifi-confirm' runs first"
    fi
    sudo systemd-run --unit=jetrover-wifi-join /usr/bin/nmcli connection up id "$ssid" ifname "$dev" >/dev/null
    echo "switching $(hostname) from '${current:-nothing}' to '$ssid' now; this ssh session will drop"
    REMOTE
    set +e
    printf '%s\n' "$password" | ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$target" \
        "bash /tmp/jetrover-wifi-join.sh $(printf '%q' "{{ssid}}") {{revert_minutes}}; status=\$?; rm -f /tmp/jetrover-wifi-join.sh; exit \$status"
    status=$?
    set -e
    if [ "$status" -eq 255 ]; then
        echo "(ssh dropped, which is expected once the robot leaves its old network)" >&2
    elif [ "$status" -ne 0 ]; then
        exit "$status"
    fi
    echo "Next: wait ~30 s, run 'just robot-net-status' (over Tailscale) to read the robot's new address, then 'JETROVER_HOST=<new address> just robot-wifi-confirm' to keep it." >&2

# NOT YET RUN AGAINST THE ROBOT. Keep the Wi-Fi network `robot-wifi-join` switched to, cancelling its scheduled fallback -- run it over the new path once that path works, e.g. `JETROVER_HOST=<new address> just robot-wifi-confirm`
[group('robot: network setup')]
robot-wifi-confirm remote_host=default_jetrover_host remote_user="jetson":
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "if systemctl is-active --quiet jetrover-wifi-revert.timer; then sudo systemctl stop jetrover-wifi-revert.timer && echo 'fallback cancelled; staying on this network'; else echo 'no fallback pending'; fi"

# Pin the Jetson's wired IPv4 address instead of taking it from DHCP, e.g. `just robot-static-ip 10.188.57.2/21 10.188.56.1`. Keeps the DNS servers DHCP gave it. Reverts to DHCP in `revert_minutes` unless `just robot-static-ip-confirm` runs first. Only use an address the network admin has approved; DHCP does not know about it
[group('robot: network setup')]
robot-static-ip address gateway connection="Wired connection 1" revert_minutes="5" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" \
        "bash -s -- $(printf '%q' '{{connection}}') {{address}} {{gateway}} {{revert_minutes}}" <<'REMOTE'
    set -euo pipefail
    con="$1"; address="$2"; gateway="$3"; revert_minutes="$4"
    if ! sudo -n true 2>/dev/null; then echo "error: this needs passwordless sudo on $(hostname)" >&2; exit 1; fi
    if ! nmcli -t -f NAME connection show | grep -Fxq "$con"; then
        echo "error: no NetworkManager profile named '$con' on $(hostname)" >&2
        exit 1
    fi
    dev=$(nmcli -g connection.interface-name connection show "$con")
    if [ -z "$dev" ]; then dev=$(nmcli -g GENERAL.DEVICES connection show "$con" 2>/dev/null || true); fi
    if [ -z "$dev" ]; then echo "error: '$con' is not bound to a device; not changing it" >&2; exit 1; fi
    dns=$(nmcli -g IP4.DNS device show "$dev" | tr '|' ' ' | xargs || true)
    echo "$(hostname): '$con' on $dev is currently ipv4.method=$(nmcli -g ipv4.method connection show "$con"), DNS: ${dns:-none}"
    # Fallback first, as a transient timer, so it fires even if the change below cuts this session off.
    sudo systemctl stop jetrover-static-ip-revert.timer >/dev/null 2>&1 || true
    sudo systemctl reset-failed jetrover-static-ip-revert.timer jetrover-static-ip-revert.service jetrover-static-ip-apply.service >/dev/null 2>&1 || true
    sudo systemd-run --unit=jetrover-static-ip-revert --on-active="${revert_minutes}min" /bin/sh -c \
        "nmcli connection modify '$con' ipv4.method auto ipv4.addresses '' ipv4.gateway '' ipv4.dns '' && nmcli connection up '$con'" >/dev/null
    sudo nmcli connection modify "$con" ipv4.method manual ipv4.addresses "$address" ipv4.gateway "$gateway" ipv4.dns "$dns"
    sudo systemd-run --unit=jetrover-static-ip-apply /usr/bin/nmcli connection up "$con" >/dev/null
    echo "applying $address via $gateway on $dev now; back to DHCP in ${revert_minutes} min unless 'just robot-static-ip-confirm' runs first"
    REMOTE

# Keep the static address `robot-static-ip` set, cancelling its scheduled return to DHCP -- run it once the robot answers on that address
[group('robot: network setup')]
robot-static-ip-confirm remote_host=default_jetrover_host remote_user="jetson":
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "if systemctl is-active --quiet jetrover-static-ip-revert.timer; then sudo systemctl stop jetrover-static-ip-revert.timer && echo 'fallback cancelled; static address kept'; else echo 'no fallback pending'; fi; nmcli -g ipv4.method,ipv4.addresses connection show 'Wired connection 1'"

# Rename the Jetson (hostname, /etc/hosts, its mDNS name <name>.local, and its Tailscale machine name) -- moves nothing. docs/adr/0015 finds the robot by this name, so it must be unique on the LAN
[group('robot: network setup')]
robot-set-hostname name remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    if ! printf '%s' '{{name}}' | grep -Eq '^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$'; then
        echo "error: '{{name}}' is not a valid hostname (lowercase letters, digits, inner hyphens, at most 63)" >&2
        exit 1
    fi
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" "bash -s -- {{name}}" <<'REMOTE'
    set -euo pipefail
    new="$1"; old=$(hostname)
    if ! sudo -n true 2>/dev/null; then echo "error: this needs passwordless sudo on $old" >&2; exit 1; fi
    if [ "$old" = "$new" ]; then echo "already named $new"; else
        sudo hostnamectl set-hostname "$new"
        # sudo resolves its own hostname through /etc/hosts; a stale 127.0.1.1 line makes every sudo warn.
        if grep -q '^127\.0\.1\.1[[:space:]]' /etc/hosts; then
            sudo sed -i "s/^127\.0\.1\.1[[:space:]].*/127.0.1.1\t$new/" /etc/hosts
        else
            printf '127.0.1.1\t%s\n' "$new" | sudo tee -a /etc/hosts >/dev/null
        fi
        echo "renamed $old -> $new"
    fi
    if systemctl is-active --quiet avahi-daemon; then
        sudo systemctl restart avahi-daemon
        echo "mDNS: avahi-daemon restarted, now announcing $(avahi-resolve -4 -n "$new.local" 2>/dev/null || echo "$new.local")"
    else
        echo "warning: avahi-daemon is not running, so $new.local will not resolve on the LAN" >&2
    fi
    if command -v tailscale >/dev/null; then
        sudo tailscale set --hostname="$new" 2>/dev/null && echo "Tailscale machine name: $new" \
            || echo "note: this tailscale cannot 'set --hostname'; rename the machine in the admin console" >&2
    fi
    REMOTE

# ---- analysis/ (python: figures, percentile tables) ----

# Internal: create analysis/.venv if it doesn't exist yet (fast no-op otherwise). `.venv/` is
# gitignored, so a fresh clone has none -- this makes `test`/`experiment-gui`/`report` below work
# on the first try instead of failing with "python.exe: not found" and telling you to run setup
# yourself. Safe to depend on from every recipe below; it won't touch an existing venv.
_analysis-venv:
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    # A venv whose interpreter exists but has no pip is a real failure mode, not a fresh clone:
    # on Debian/Ubuntu `python3 -m venv` half-succeeds without the python3-venv package, leaving
    # .venv/bin/python in place with no ensurepip. Checking for the interpreter alone made this
    # recipe no-op and pushed a confusing "No module named pip" into whatever ran next.
    for py in .venv/Scripts/python.exe .venv/bin/python; do
        if [ -x "$py" ] && "$py" -c 'import pip' >/dev/null 2>&1; then
            exit 0
        fi
    done
    if [ -e .venv ]; then
        echo "analysis/.venv exists but is unusable (no pip) -- delete it and re-run" >&2
        exit 1
    fi
    echo "analysis/.venv not found -- running one-time setup..." >&2
    just analysis-setup

# One-time analysis/ venv setup (host python auto-detected per box -- see analysis/CLAUDE.md).
# Also called automatically by test/experiment-gui/report if analysis/.venv doesn't exist yet.
[group('checks')]
analysis-setup:
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    # The host python differs per box (see root CLAUDE.md "Environment"): the Windows box builds
    # a Scripts/ venv through WSL interop, the Linux box uses its own python3. Detect by which
    # interpreter actually exists rather than by uname, so neither box needs a flag.
    WIN_PY=/mnt/c/Users/andre/AppData/Local/Microsoft/WindowsApps/python.exe
    if [ -f "$WIN_PY" ]; then
        "$WIN_PY" -m venv .venv
        PY=./.venv/Scripts/python.exe
    else
        python3 -m venv .venv || {
            echo "python3 -m venv failed. On Debian/Ubuntu the stdlib venv/ensurepip module is" >&2
            echo "packaged separately: sudo apt install python3-venv" >&2
            exit 1
        }
        PY=./.venv/bin/python
    fi
    "$PY" -m pip install -r requirements-dev.txt
    "$PY" -m pip install -e . --no-build-isolation

# Run the analysis/ pytest suite -- scriptable, use this for CI/agent verification
[group('checks')]
test: _analysis-venv
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    PY=$([ -f .venv/Scripts/python.exe ] && echo .venv/Scripts/python.exe || echo .venv/bin/python)
    "$PY" -m pytest -v

# Opens a GUI window to configure and run a sweep, then view its figures (needs a real display)
[group('experiments')]
experiment-gui: _analysis-venv
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    PY=$([ -f .venv/Scripts/python.exe ] && echo .venv/Scripts/python.exe || echo .venv/bin/python)
    "$PY" run_tests.py

# Generate figures + summary table for a run, e.g. `just report results/exp-001-predictor-baseline/20260804-020431Z`
[group('experiments')]
report run_dir: _analysis-venv
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    PY=$([ -f .venv/Scripts/python.exe ] && echo .venv/Scripts/python.exe || echo .venv/bin/python)
    "$PY" -m teleop_analysis.cli ../{{run_dir}}

# ---- trainer/ (docs/adr/0021: the learned joint predictor's trainer, packed to one file) ----

# Internal: the trainer's local python (CPU), same host detection as analysis-setup
_trainer-python:
    #!/usr/bin/env bash
    set -euo pipefail
    cd trainer
    for py in .venv-win/Scripts/python.exe .venv/bin/python; do
        if [ -f "$py" ] && "$py" -c "import numpy, pytest" 2>/dev/null; then echo "trainer/$py"; exit 0; fi
    done
    echo "error: no trainer venv -- run 'just trainer-setup' (add torch=true to also train and export locally)" >&2
    exit 1

# Simulated pretraining on this machine's GPU, straight from the repo (Linux; docs/adr/0021): the first time, trainer/setup.sh builds trainer/.venv with the torch build this NVIDIA driver needs and runs the selftest; then armpred sim-pretrain. config: smoke, medium or sim-pretrain. The run goes to trainer/runs/<name> (gitignored); re-running the same name reuses what it already simulated and built. Extra args pass through, e.g. `just train-sim sim-pretrain sim1 --skip-eval`. The full sim-pretrain wants about 32 GB of RAM and 25 GB of disk, and takes 3-4 h (mostly CPU); run it under tmux or nohup so a dropped SSH session does not stop it
[group('trainer')]
train-sim config="sim-pretrain" name="" *args:
    trainer/train-sim.sh "{{config}}" "{{name}}" {{args}}

# The FR3 Duo's simulated pretraining on this machine (docs/adr/0028): simulate the FR3 session matrix (session v2), build layout v4, train on top of franka-replay, export (weights format 2), evaluate, all into trainer/runs/<name>. On Linux it runs trainer/train-fr3-sim.sh (trainer/.venv, built the first time); under WSL it runs the Windows CUDA venv trainer/.venv-win-cuda. config: fr3-smoke (minutes) or fr3-sim-pretrain (hours). SIMULATED: the report says so, and it is never a gate verdict
[group('trainer')]
train-fr3-sim config="fr3-sim-pretrain" name="" *args:
    #!/usr/bin/env bash
    set -euo pipefail
    name="{{name}}"; [ -n "$name" ] || name="{{config}}-$(git rev-parse --short HEAD)"
    # The Windows CUDA venv: this checkout's, or (in a git worktree) the main checkout's.
    py=""
    for v in trainer/.venv-win-cuda "$(dirname "$(git rev-parse --path-format=absolute --git-common-dir)")/trainer/.venv-win-cuda"; do
        [ -x "$v/Scripts/python.exe" ] && { py="$(cd "$v" && pwd)/Scripts/python.exe"; break; }
    done
    if [ -n "$py" ] && grep -qi microsoft /proc/version 2>/dev/null; then
        cd trainer && "$py" -m armpred fr3-sim-pretrain --config "configs/{{config}}.toml" --out "runs/$name" {{args}}
    else
        trainer/train-fr3-sim.sh "{{config}}" "$name" {{args}}
    fi

# Start `just train-sim <config>` on the GPU machine ($GPU_HOST, default andrei@10.188.58.164), detached, so this machine and the SSH session can go (docs/adr/0021). Clones the repo into ~/teleoperation there the first time and pulls main every time; refuses while a run of ours is going there, or while the GPU is busy (over 2 GB in use: someone else's job). The run is trainer/runs/<config>-<commit> there; the log ~/train-sim-<config>.log. Watch it with just gpu-status; bring it back with just gpu-fetch
[group('trainer')]
gpu-train config="sim-pretrain-pilot1":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{config}}" in *[!A-Za-z0-9._-]*) echo "error: config is a trainer/configs name" >&2; exit 1 ;; esac
    [ -f "trainer/configs/{{config}}.toml" ] || { echo "error: no trainer/configs/{{config}}.toml here" >&2; exit 1; }
    ssh -o ConnectTimeout=10 "{{gpu_host}}" "CONFIG='{{config}}' bash -s" <<'REMOTE'
    set -euo pipefail
    if [ -f ~/train-sim.pid ] && kill -0 "$(cat ~/train-sim.pid)" 2>/dev/null; then
        echo "error: a run is already going (pid $(cat ~/train-sim.pid)): just gpu-status" >&2; exit 1
    fi
    used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1 | tr -d ' ')
    if [ "${used:-0}" -gt 2048 ]; then
        echo "error: the GPU has ${used} MiB in use: someone else's job. Not starting." >&2
        nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv >&2 || true
        exit 1
    fi
    [ -d ~/teleoperation ] || git clone -q https://github.com/andreitimoshukcollege-del/teleoperation.git ~/teleoperation
    cd ~/teleoperation && git checkout -q main && git pull -q --ff-only
    echo "repo at $(git log --oneline -1)"
    log=~/train-sim-$CONFIG.log
    # The script, not `just train-sim`: a GPU machine's `just` may be too old for this justfile.
    nohup setsid trainer/train-sim.sh "$CONFIG" > "$log" 2>&1 < /dev/null &
    echo $! > ~/train-sim.pid
    echo "started (pid $!), run trainer/runs/$CONFIG-$(git rev-parse --short HEAD), log $log"
    sleep 15; tail -5 "$log"
    REMOTE

# Fine-tune a GPU run on a real data bundle there (ADR 0021 step D3), detached: `just gpu-finetune <bundle .tar> <split> <config> <init run> <out>`, e.g. `just gpu-finetune dist/jetrover-data-20261026T211226Z.tar split-heldout-early7 real-finetune-pilot2 sim-pretrain-pilot2-69a8261 finetune-pilot2-early7`. The init run's checkpoint never leaves that machine, so the fine-tune runs there: this copies the bundle (the robot's recordings; the user allowed them on that machine on 2026-10-27) and the two trainer/configs files up into ~/finetune-inputs (the configs need not be on main; anything they name, such as a prior, must be), pulls main, and starts `armpred finetune`, which scores the split's test sessions itself. Same guards as gpu-train (one run at a time, the GPU not busy). Watch with `just gpu-status <out>`; fetch with `just gpu-fetch <out>`
[group('trainer')]
gpu-finetune bundle split config init out:
    #!/usr/bin/env bash
    set -euo pipefail
    for v in "{{split}}" "{{config}}" "{{init}}" "{{out}}"; do
        case "$v" in *[!A-Za-z0-9._-]*|"") echo "error: '$v': split and config are trainer/configs names, init and out trainer/runs names" >&2; exit 1 ;; esac
    done
    [ -f "{{bundle}}" ] || { echo "error: no bundle {{bundle}} here (just pack-dataset)" >&2; exit 1; }
    for f in "trainer/configs/{{split}}.toml" "trainer/configs/{{config}}.toml"; do [ -f "$f" ] || { echo "error: no $f" >&2; exit 1; }; done
    name=$(basename "{{bundle}}")
    case "$name" in *[!A-Za-z0-9._-]*) echo "error: the bundle's file name must be plain" >&2; exit 1 ;; esac
    ssh -o ConnectTimeout=10 "{{gpu_host}}" "INIT='{{init}}' OUT='{{out}}' bash -s" <<'REMOTE'
    set -euo pipefail
    if [ -f ~/train-sim.pid ] && kill -0 "$(cat ~/train-sim.pid)" 2>/dev/null; then
        echo "error: a run is already going (pid $(cat ~/train-sim.pid)): just gpu-status" >&2; exit 1
    fi
    used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1 | tr -d ' ')
    if [ "${used:-0}" -gt 2048 ]; then
        echo "error: the GPU has ${used} MiB in use: someone else's job. Not starting." >&2
        nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv >&2 || true
        exit 1
    fi
    [ -f ~/teleoperation/trainer/runs/"$INIT"/checkpoints/best.pt ] || { echo "error: no runs/$INIT/checkpoints/best.pt there" >&2; exit 1; }
    [ ! -e ~/teleoperation/trainer/runs/"$OUT" ] || { echo "error: runs/$OUT already exists there" >&2; exit 1; }
    mkdir -p ~/finetune-inputs/"$OUT"
    chmod 700 ~/finetune-inputs
    REMOTE
    scp -q "{{bundle}}" "{{gpu_host}}:finetune-inputs/$name"
    scp -q "trainer/configs/{{split}}.toml" "{{gpu_host}}:finetune-inputs/{{out}}/split.toml"
    scp -q "trainer/configs/{{config}}.toml" "{{gpu_host}}:finetune-inputs/{{out}}/config.toml"
    ssh -o ConnectTimeout=10 "{{gpu_host}}" "INIT='{{init}}' OUT='{{out}}' NAME='$name' bash -s" <<'REMOTE'
    set -euo pipefail
    cd ~/teleoperation && git checkout -q main && git pull -q --ff-only
    echo "repo at $(git log --oneline -1)"
    cd trainer
    unset PYTHONPATH  # as train-sim.sh: a sourced ROS setup.bash breaks the venv
    log=~/train-sim-$OUT.log
    nohup setsid .venv/bin/armpred finetune --data ~/finetune-inputs/"$NAME" --split ~/finetune-inputs/"$OUT"/split.toml \
        --config ~/finetune-inputs/"$OUT"/config.toml --init runs/"$INIT" --out runs/"$OUT" > "$log" 2>&1 < /dev/null &
    echo $! > ~/train-sim.pid
    echo "started (pid $!), run trainer/runs/$OUT, log $log"
    sleep 15; tail -5 "$log"
    REMOTE

# The GPU machine's run -- touches nothing: whether ours is still going, the GPU, and the end of its log
[group('trainer')]
gpu-status config="sim-pretrain-pilot1" lines="8":
    @ssh -o ConnectTimeout=10 "{{gpu_host}}" 'p=$(cat ~/train-sim.pid 2>/dev/null); \
        if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then echo "run: going (pid $p)"; else echo "run: not going"; fi; \
        nvidia-smi --query-gpu=name,memory.used,memory.total,utilization.gpu --format=csv,noheader; \
        echo "its clock: $(date +%H:%M:%S); load: $(cut -d" " -f1-3 /proc/loadavg) on $(nproc) cores; trainer processes: $(pgrep -fc "armpred" || true)"; \
        echo; tail -n {{lines}} ~/train-sim-{{config}}.log 2>/dev/null || echo "(no log ~/train-sim-{{config}}.log)"'

# Bring a GPU run back here, e.g. `just gpu-fetch sim-pretrain-pilot1-24901ef`: packs trainer/runs/<run> there without its checkpoints or training data, copies it to dist/, and unpacks it into trainer/runs/<run> (refusing to overwrite one already here)
[group('trainer')]
gpu-fetch run:
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{run}}" in *[!A-Za-z0-9._-]*|"") echo "error: run is a trainer/runs name" >&2; exit 1 ;; esac
    [ ! -e "trainer/runs/{{run}}" ] || { echo "error: trainer/runs/{{run}} is already here" >&2; exit 1; }
    ssh -o ConnectTimeout=10 "{{gpu_host}}" "cd ~/teleoperation/trainer/runs && [ -d '{{run}}' ] && tar czf ~/'{{run}}'.tgz --exclude='{{run}}/checkpoints' --exclude='{{run}}/data' '{{run}}' && ls -la ~/'{{run}}'.tgz"
    mkdir -p dist trainer/runs
    scp -q "{{gpu_host}}:{{run}}.tgz" "dist/{{run}}.tgz"
    tar xzf "dist/{{run}}.tgz" -C trainer/runs
    echo "trainer/runs/{{run}}:"; ls trainer/runs/{{run}}
    [ -f "trainer/runs/{{run}}/eval/report.md" ] && sed -n '1,20p' "trainer/runs/{{run}}/eval/report.md" || true

# One-time local trainer venv, CPU only: the contract, the checker, the simulator and the tests (torch=true adds the CPU torch build, for smoke training and export). GPU machines use trainer/setup.sh from the packed file instead
[group('trainer')]
trainer-setup torch="false":
    #!/usr/bin/env bash
    set -euo pipefail
    cd trainer
    WIN_PY=/mnt/c/Users/andre/AppData/Local/Microsoft/WindowsApps/python.exe
    if [ -f "$WIN_PY" ]; then
        "$WIN_PY" -m venv .venv-win
        PY=./.venv-win/Scripts/python.exe
    else
        python3 -m venv .venv || { echo "python3 -m venv failed: sudo apt install python3-venv" >&2; exit 1; }
        PY=./.venv/bin/python
    fi
    "$PY" -m pip install --quiet -r requirements.txt
    if [ "{{torch}}" = "true" ]; then
        "$PY" -m pip install --quiet torch==2.14.1 --index-url https://download.pytorch.org/whl/cpu
    fi
    "$PY" -c "import numpy; print('trainer venv ready, numpy', numpy.__version__)"

# The trainer's tests (the golden-file checks of its copies of Core included). Needs `just trainer-setup`
[group('checks')]
trainer-test:
    #!/usr/bin/env bash
    set -euo pipefail
    PY=$(just _trainer-python)
    cd trainer
    "../$PY" -m pytest -q -p no:cacheprovider

# Regenerate trainer/contract/golden from Core (after changing SeededRng, the impairments, the profile catalog, ClockSync or relay-replay); then fix the trainer's copies until `just trainer-test` passes
[group('trainer')]
gen-trainer-golden:
    cd core && dotnet run --project Teleop.Eval -- gen-trainer-golden

# Pack trainer/ into the one file the GPU machine needs: dist/armpred-<sha>.tar.gz, with a VERSION file naming the commit (and whether the tree was dirty). Unpack it there and run ./setup.sh (trainer/README.md)
[group('trainer')]
pack-trainer:
    #!/usr/bin/env bash
    set -euo pipefail
    sha=$(git rev-parse HEAD)
    dirty=false
    if [ -n "$(git status --porcelain -- trainer)" ]; then dirty=true; echo "warning: trainer/ has uncommitted changes; VERSION records dirty=true" >&2; fi
    stage=$(mktemp -d)
    trap 'rm -rf "$stage"' EXIT
    mkdir -p "$stage/armpred" dist
    (cd trainer && tar cf - --exclude='__pycache__' --exclude='*.pyc' \
        armpred contract configs tests slurm pyproject.toml requirements.txt setup.sh README.md CLAUDE.md) \
        | (cd "$stage/armpred" && tar xf -)
    printf '{"sha": "%s", "dirty": %s, "source": "pack-trainer", "packedUtc": "%s"}\n' "$sha" "$dirty" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" > "$stage/armpred/VERSION"
    suffix=""; if [ "$dirty" = true ]; then suffix="-dirty"; fi
    out="dist/armpred-${sha:0:12}${suffix}.tar.gz"
    tar czf "$out" -C "$stage" armpred
    echo "$out ($(du -h "$out" | cut -f1))"

# ---- recording sessions for the trainer (docs/adr/0021; trainer/contract/SESSION_FORMAT.md) ----

robot_recordings := "/home/jetson/recordings"

# Internal: if the robot is recording an automatic session (RobotHost opened it: ~/recordings/.active-auto names the same directory as .active, docs/adr/0023 decision 4), close it and wait until RobotHost has completed it (END trailers, session.json, files.sha256). A manual session, or none, is left alone. Same steps as robot/pilot/onboard.py's close_auto_session (keep the two the same). Moves nothing
_close-auto-session host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{host}}" 'bash -s' <<'SH'
    set -euo pipefail
    r={{robot_recordings}}
    dir=$(cat "$r/.active" 2>/dev/null || true)
    [ -n "$dir" ] || exit 0
    [ "$(cat "$r/.active-auto" 2>/dev/null || true)" = "$dir" ] || exit 0
    echo "closing the automatic session $(basename "$dir") first (manual sessions win)" >&2
    rm -f "$r/.active"
    for i in $(seq 1 120); do [ -f "$dir/files.sha256" ] && break; sleep 0.25; done
    [ -f "$dir/files.sha256" ] || { echo "error: RobotHost did not complete $(basename "$dir") within 30 s -- just robothost-logs" >&2; exit 1; }
    for f in "$dir"/*.jlog; do printf '  %-16s %s\n' "$(basename "$f")" "$(tail -n 1 "$f")" >&2; done
    SH

# Start recording on the robot -- moves nothing. Closes an automatic session RobotHost opened first, if one is open (manual sessions win, docs/adr/0023). Writes session.json (the conditions describe the session for splits and stratification, never the model's inputs) and the control file RobotHost and the driver both watch, then checks both logs are growing. After the label and the slider (rad/s, or varied), options as key=value, e.g. `just record-start payload-100g 2.5 load=payload payload_g=100`: load (none, payload or hand), payload_g, network (the robot-side uplink profile: lan for none, or "schedule:<profile>@<s>;..." for one that changes mid-session, quoted for the ;), added_delay_ms, motion (teleop, or joint-drive:<script>), operator, notes, remote_host, remote_user
[group('robot: recording (docs/adr/0021)')]
[positional-arguments]
record-start label slider *options:
    #!/usr/bin/env bash
    set -euo pipefail
    load=none payload_g=0 network=lan added_delay_ms=0 motion=teleop operator=A notes="" remote_host="{{default_jetrover_host}}" remote_user=jetson
    # just has no keyword arguments: `load=payload` would arrive as the next positional value, so options are parsed here.
    for opt in "${@:3}"; do
        case "$opt" in
            load=*|payload_g=*|network=*|added_delay_ms=*|motion=*|operator=*|notes=*|remote_host=*|remote_user=*) printf -v "${opt%%=*}" '%s' "${opt#*=}" ;;
            *) echo "error: unknown option '$opt' (load= payload_g= network= added_delay_ms= motion= operator= notes= remote_host= remote_user=)" >&2; exit 1 ;;
        esac
    done
    case "$load" in none|payload|hand) ;; *) echo "error: load must be none, payload or hand" >&2; exit 1 ;; esac
    case "{{label}}" in *[!A-Za-z0-9._-]*) echo "error: label may hold only letters, digits, . _ -" >&2; exit 1 ;; esac
    case "$notes" in *"|"*) echo "error: notes may not hold |" >&2; exit 1 ;; esac
    host=$(just _robot-host "$remote_host")
    target="$remote_user@$host"
    id="$(date -u +%Y%m%dT%H%M%SZ)-{{label}}"
    dir="{{robot_recordings}}/jetrover/$id"
    # Manual sessions win (docs/adr/0023 decision 4): an automatic one RobotHost opened is closed first.
    just _close-auto-session "$host" "$remote_user"
    if ssh -o StrictHostKeyChecking=accept-new "$target" "test -s {{robot_recordings}}/.active"; then
        echo "error: a session is already recording ($(ssh "$target" cat {{robot_recordings}}/.active)) -- just record-stop first" >&2
        exit 1
    fi
    builds=$(ssh "$target" "cat ~/robothost_current/BUILD_INFO 2>/dev/null || echo '{}'; cat ~/teleop_ros_current/BUILD_INFO 2>/dev/null || echo '{}'")
    meta=$(BUILDS="$builds" python3 - "$id" "{{label}}" "{{slider}}" "$load" "$payload_g" "$network" "$added_delay_ms" "$motion" "$operator" "$notes" "$(sha256sum core/RobotProfiles/jetrover.json | cut -d' ' -f1)" <<'PY'
    import datetime, json, os, sys
    sid, label, slider, load, grams, network, added, motion, operator, notes, profile_sha = sys.argv[1:]
    lines = [l for l in os.environ["BUILDS"].splitlines() if l.strip()]
    builds = [json.loads(l) for l in lines] + [{}, {}]
    uplink = {"uplinkProfile": network, "uplinkSeed": 0, "addedDelayMs": float(added)}
    if network.startswith("schedule:"):
        uplink.update({"uplinkProfile": "schedule", "uplinkSchedule": network[len("schedule:"):], "uplinkSeed": 1})
    print(json.dumps({
        "format": "jetrover-session", "version": 1, "id": sid, "source": "robot", "label": label,
        "startedUtc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "conditions": {
            "speedSliderRadPerSec": slider if slider == "varied" else float(slider),
            "load": {"kind": load, "payloadGrams": float(grams)},
            "network": uplink,
            "motion": motion, "operator": operator,
        },
        "builds": {"robothost": builds[0], "driver": builds[1]},
        "robotProfileSha256": profile_sha, "notes": notes,
    }, indent=2))
    PY
    )
    ssh "$target" "mkdir -p $dir && cat > $dir/session.json" <<< "$meta"
    scp -q core/RobotProfiles/jetrover.json "$target:$dir/robot-profile.json"
    # The control file is created only if it does not exist (a hard link, which fails if it does):
    # RobotHost may have opened an automatic session since the check above, and then it is closed again.
    opened=false
    for attempt in 1 2 3; do
        if ssh "$target" "[ -s {{robot_recordings}}/.active ] || rm -f {{robot_recordings}}/.active; printf '%s\n' $dir > {{robot_recordings}}/.active.start && ln {{robot_recordings}}/.active.start {{robot_recordings}}/.active; s=\$?; rm -f {{robot_recordings}}/.active.start; exit \$s" 2>/dev/null; then
            opened=true; break
        fi
        just _close-auto-session "$host" "$remote_user"
    done
    if ! $opened; then
        echo "error: could not take the control file ($(ssh "$target" cat {{robot_recordings}}/.active 2>/dev/null)) -- just record-stop first" >&2
        exit 1
    fi
    sleep 2.5
    sizes=$(ssh "$target" "stat -c '%n %s' $dir/robothost.jlog $dir/driver.jlog 2>&1 || true")
    echo "$sizes"
    ok=true
    case "$sizes" in *"robothost.jlog"*" 0"|*"robothost.jlog'"*|*"No such file"*robothost*) echo "warning: RobotHost is not recording -- redeploy it (just deploy-robothost) so it runs with --record-control" >&2; ok=false ;; esac
    case "$sizes" in *"driver.jlog'"*|*"No such file"*driver*) echo "warning: the driver is not recording -- just deploy-ros (someone watching: it restarts the servo driver)" >&2; ok=false ;; esac
    $ok || exit 1
    if [ "${network#schedule:}" != "$network" ]; then
        # A schedule starts once the logs are open, so robothost.jlog records every change (UP).
        just uplink-schedule "${network#schedule:}" 1 "$host" "$remote_user"
    elif [ "$network" != "lan" ] && [ "$network" != "none" ]; then
        # The uplink changes once the logs are open, so robothost.jlog records the change (UP).
        just uplink-impair "$network" 1 "$host" "$remote_user"
    fi
    echo "recording $id"

# Stop the robot's recording -- moves nothing. An automatic session (docs/adr/0023) is only closed: RobotHost completes it. Otherwise removes the control file, waits for both logs' END trailers, stamps stoppedUtc, sets the session's slider from the speed its joint commands actually carried (warning when that is not what record-start was told), and writes files.sha256
[group('robot: recording (docs/adr/0021)')]
record-stop remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    ssh "$target" 'bash -s' <<'SH'
    set -euo pipefail
    control={{robot_recordings}}/.active
    [ -s "$control" ] || { echo "nothing is recording" >&2; exit 1; }
    if [ "$(cat {{robot_recordings}}/.active-auto 2>/dev/null || true)" = "$(cat "$control")" ]; then
        # An automatic session (docs/adr/0023): RobotHost completes it itself, and leaves its own uplink alone.
        dir=$(cat "$control"); rm -f "$control"
        echo "stopping the automatic session $(basename "$dir"); RobotHost completes it (the next one opens on a command after 30 s without one)"
        for i in $(seq 1 120); do [ -f "$dir/files.sha256" ] && break; sleep 0.25; done
        [ -f "$dir/files.sha256" ] || { echo "error: RobotHost did not complete it within 30 s -- just robothost-logs" >&2; exit 1; }
        for f in "$dir"/*.jlog; do printf '%-16s %s\n' "$(basename "$f")" "$(tail -n 1 "$f")"; done
        echo "stopped $(basename "$dir")"
        exit 0
    fi
    if [ -e {{robot_recordings}}/.uplink ]; then rm -f {{robot_recordings}}/.uplink; sleep 1; fi
    dir=$(cat "$control"); rm -f "$control"
    for i in $(seq 1 60); do
        done=true
        for f in robothost driver; do tail -n 1 "$dir/$f.jlog" 2>/dev/null | grep -q '^END|' || done=false; done
        $done && break; sleep 0.25
    done
    python3 - "$dir" <<'PY'
    import datetime, json, sys
    path = sys.argv[1] + "/session.json"
    meta = json.load(open(path))
    meta["stoppedUtc"] = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    # The slider is what the commands carried, not what record-start was told: the headset's slider
    # rides in every joint command's speed field (motor 10, the gripper, aside; 0 means none was sent).
    # Same reading as robot/pilot/onboard.py and just recording-set-slider.
    speeds = set()
    with open(sys.argv[1] + "/robothost.jlog") as f:
        for line in f:
            if line.startswith("JC|"):
                for target in line.rstrip("\n").split("|")[6].split(";"):
                    motor, _, speed = target.split(":")
                    if motor != "10" and float(speed) > 0:
                        speeds.add(round(float(speed), 2))
    asked = meta["conditions"]["speedSliderRadPerSec"]
    if speeds:
        measured = next(iter(speeds)) if len(speeds) == 1 else "varied"
        meta["conditions"]["speedSliderRadPerSec"] = measured
        if measured != asked:
            note = f"slider from the commands: {measured} ({', '.join(str(s) for s in sorted(speeds))} rad/s); record-start was told {asked}"
            meta["notes"] = (meta["notes"] + "; " if meta.get("notes") else "") + note
            print(f"WARNING: the slider was {measured}, not {asked}: session.json says {measured}")
        else:
            print(f"slider {measured} rad/s, as asked")
    else:
        print(f"no joint commands: the slider stays {asked}, as record-start was told")
    json.dump(meta, open(path, "w"), indent=2)
    PY
    cd "$dir" && sha256sum session.json robot-profile.json *.jlog > files.sha256
    for f in *.jlog; do printf '%-16s %s\n' "$f" "$(tail -n 1 "$f")"; done
    echo "stopped $(basename "$dir")"
    SH

# Record one session the arm drives itself (moves the arm; someone watching), e.g. `just record-drive steps-6 steps 6 seconds=300`: just record-start with motion=joint-drive:<script>, then just joint-drive, then its operator.jlog (every command it sent) into the session, then just record-stop. Press Enter to stop the arm where it is. The session is stopped and closed even when the drive stops early, and the recipe then exits with the drive's code. Options as key=value: seconds (300), seed (1), margin_deg (15), and any of record-start's (load, payload_g, network, notes, remote_host, remote_user). Close the headset app first
[group('robot: recording (docs/adr/0021)')]
[positional-arguments]
record-drive label script speed *options:
    #!/usr/bin/env bash
    set -euo pipefail
    seconds=300 seed=1 margin_deg=15 remote_host="{{default_jetrover_host}}" remote_user=jetson
    pass=()
    for opt in "${@:4}"; do
        case "$opt" in
            seconds=*|seed=*|margin_deg=*) printf -v "${opt%%=*}" '%s' "${opt#*=}" ;;
            remote_host=*|remote_user=*) printf -v "${opt%%=*}" '%s' "${opt#*=}" ;;
            motion=*|operator=*) echo "error: record-drive sets $opt itself (motion=joint-drive:<script>)" >&2; exit 1 ;;
            *) pass+=("$opt") ;;
        esac
    done
    case "{{script}}" in waypoints|steps) ;; *) echo "error: script must be waypoints or steps" >&2; exit 1 ;; esac
    host=$(just _robot-host "$remote_host")
    target="$remote_user@$host"
    # Built before the session opens, so the recording does not start with a compile.
    (cd core && dotnet build Teleop.Eval -nologo -v q > /dev/null)
    just record-start "{{label}}" "{{speed}}" "${pass[@]}" motion="joint-drive:{{script}}" operator=joint-drive remote_host="$host" remote_user="$remote_user"
    id=$(ssh "$target" 'basename "$(cat {{robot_recordings}}/.active)"')
    mkdir -p "recordings/.drive/$id"
    # Enter stops joint-drive, which holds the arm and exits; this script then closes the session. A Ctrl+C
    # that kills this script instead is caught here too, so the session is still closed.
    trap 'echo "interrupted: closing the session"' INT
    status=0
    (cd core && dotnet run --no-build --project Teleop.Eval -- joint-drive --script "{{script}}" --speed "{{speed}}" --seconds "$seconds" \
        --seed "$seed" --margin-deg "$margin_deg" --remote-host "$host" --confirm-hardware-motion \
        --operator-log "../recordings/.drive/$id/operator.jlog" --session-id "$id") || status=$?
    trap - INT
    if [ -s "recordings/.drive/$id/operator.jlog" ]; then
        scp -q "recordings/.drive/$id/operator.jlog" "$target:{{robot_recordings}}/jetrover/$id/operator.jlog"
    fi
    just record-stop "$host" "$remote_user"
    [ "$status" -eq 0 ] || echo "joint-drive exited $status; the session is recorded up to there" >&2
    exit "$status"

# The battery voltage (mV) in the newest recorded session's driver log -- moves nothing. Empty when the log has no battery report
[group('robot: recording (docs/adr/0021)')]
recording-battery remote_host=default_jetrover_host remote_user="jetson":
    @ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        'd=$(ls -td {{robot_recordings}}/jetrover/*/ 2>/dev/null | head -1); [ -n "$d" ] && grep "^BV|" "$d/driver.jlog" 2>/dev/null | tail -1 | cut -d"|" -f3 || true'

# Record a list of sessions one after another (moves the arm; someone watching the whole time): robot/pilot/pilot-v1.txt by default, the ADR 0021 pilot hour. For each it says what a person does first (headset on, a slider value, a payload, close the app), waits for Enter, and records it with record-start/record-stop (teleop, timed) or record-drive (the arm drives itself). It stops below 11.0 V and ends with just fetch-recordings. Options as key=value: plan=<file>, from=<label> to resume there, only=teleop|drive (only=teleop, then just pilot-on-robot for the drive sessions without the laptop), remote_host
[group('robot: recording (docs/adr/0021)')]
[positional-arguments]
pilot *options:
    #!/usr/bin/env bash
    set -euo pipefail
    plan=robot/pilot/pilot-v1.txt from="" only=all remote_host="{{default_jetrover_host}}"
    for opt in "$@"; do
        case "$opt" in
            plan=*|from=*|only=*|remote_host=*) printf -v "${opt%%=*}" '%s' "${opt#*=}" ;;
            *) echo "error: unknown option '$opt' (plan= from= only= remote_host=)" >&2; exit 1 ;;
        esac
    done
    python3 robot/pilot/pilot.py "$plan" --from "$from" --only "$only" --host "$remote_host"

# Publish joint-drive (Teleop.Eval, linux-arm64) to the Jetson as ~/jointdrive_current, for just pilot-on-robot -- moves nothing and restarts nothing: RobotHost and the driver keep running. Ends with a dry run on the robot to show it starts
[group('robot: deploy and restart (someone at the robot)')]
deploy-joint-drive remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    target="{{remote_user}}@$host"
    cd core
    remote_dir="jointdrive_deploy_$(date +%Y%m%d-%H%M%S)"
    # Relative, as deploy-robothost's: the Windows dotnet does not resolve a WSL absolute path.
    publish_dir="_scratch_publish"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.Eval (joint-drive) for linux-arm64..." >&2
    dotnet publish Teleop.Eval -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir" -nologo -v q
    printf '{"sha": "%s", "dirty": %s}\n' "$(git rev-parse HEAD)" "$([ -n "$(git status --porcelain --untracked-files=no -- .)" ] && echo true || echo false)" > "$publish_dir/BUILD_INFO"
    tar czf "$archive" -C "$publish_dir" .
    scp -q -o StrictHostKeyChecking=accept-new "$archive" "$target:/tmp/$remote_dir.tar.gz"
    ssh "$target" "mkdir -p ~/$remote_dir && tar xzf /tmp/$remote_dir.tar.gz -C ~/$remote_dir && rm /tmp/$remote_dir.tar.gz && ln -sfn ~/$remote_dir ~/jointdrive_current && \
        echo \"jointdrive_current -> $remote_dir: \$(cat ~/jointdrive_current/BUILD_INFO)\" && \
        ~/.dotnet/dotnet ~/jointdrive_current/Teleop.Eval.dll joint-drive --script steps --speed 1 --seconds 10 --dry-run | tail -1"

# Record a plan's drive sessions on the robot itself, so the laptop can close (moves the arm; someone watches it the WHOLE time): copies robot/pilot (and the robot profile record-start uses) to the Jetson and starts robot/pilot/onboard.py there, detached. It runs every drive: session back to back, joint-drive talking to RobotHost over loopback (so no Wi-Fi in these sessions; their notes say so), rests rest_s between sessions, stops below 11.0 V and after any session that ends early. Teleop sessions need the laptop: just pilot only=teleop. Stop: just pilot-stop from any machine, or the robot's power switch. Watch: just pilot-status. Needs just deploy-joint-drive first. Options as key=value: plan=<file>, from=<label>, rest_s (60), start_delay_s (30), remote_host
[group('robot: recording (docs/adr/0021)')]
[positional-arguments]
pilot-on-robot *options:
    #!/usr/bin/env bash
    set -euo pipefail
    plan=robot/pilot/pilot-v1.txt from="" rest_s=60 start_delay_s=30 remote_host="{{default_jetrover_host}}"
    for opt in "$@"; do
        case "$opt" in
            plan=*|from=*|rest_s=*|start_delay_s=*|remote_host=*) printf -v "${opt%%=*}" '%s' "${opt#*=}" ;;
            *) echo "error: unknown option '$opt' (plan= from= rest_s= start_delay_s= remote_host=)" >&2; exit 1 ;;
        esac
    done
    host=$(just _robot-host "$remote_host")
    target="jetson@$host"
    built=$(ssh "$target" 'cat ~/jointdrive_current/BUILD_INFO 2>/dev/null' || true)
    [ -n "$built" ] || { echo "error: joint-drive is not on the robot: just deploy-joint-drive" >&2; exit 1; }
    case "$built" in *"$(git rev-parse HEAD)"*) ;; *) echo "warning: the robot's joint-drive ($built) is not this checkout's $(git rev-parse --short HEAD): just deploy-joint-drive to update it" >&2 ;; esac
    if ssh "$target" 'p=$(cat ~/recordings/.pilot-onboard.pid 2>/dev/null) && kill -0 "$p" 2>/dev/null'; then
        echo "error: an onboard pilot is already running: just pilot-status, or just pilot-stop" >&2; exit 1
    fi
    dest="pilot_$(date +%Y%m%d-%H%M%S)"
    ssh "$target" "mkdir -p ~/$dest ~/recordings && rm -f ~/recordings/.pilot-stop"
    scp -q robot/pilot/pilot.py robot/pilot/onboard.py "$plan" core/RobotProfiles/jetrover.json "$target:$dest/"
    ssh "$target" "cd ~/$dest && setsid nohup python3 -u onboard.py '$(basename "$plan")' --profile jetrover.json --from '$from' \
        --rest '$rest_s' --start-delay '$start_delay_s' > ~/recordings/pilot-onboard.log 2>&1 < /dev/null & echo \$! > ~/recordings/.pilot-onboard.pid"
    sleep 3
    ssh "$target" 'cat ~/recordings/pilot-onboard.log'
    echo
    echo "Running on the robot ($host), detached: the laptop can close. Someone stays with the arm."
    echo "  watch: just pilot-status     stop: just pilot-stop (or the power switch)     afterwards: just fetch-recordings"

# What the onboard pilot is doing -- moves nothing: whether it runs, the end of its log, and what is recording
[group('robot: recording (docs/adr/0021)')]
pilot-status lines="25" remote_host=default_jetrover_host:
    @ssh "jetson@$(just _robot-host "{{remote_host}}")" 'p=$(cat ~/recordings/.pilot-onboard.pid 2>/dev/null); \
        if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then echo "onboard pilot: running (pid $p)"; else echo "onboard pilot: not running"; fi; \
        echo "recording now: $(cat ~/recordings/.active 2>/dev/null || echo nothing)"; echo; tail -n {{lines}} ~/recordings/pilot-onboard.log 2>/dev/null || echo "(no log yet)"'

# Stop the onboard pilot: joint-drive holds the arm where it is at once, the session is closed, and no other starts. Waits up to 30 s for it to finish, then shows its log
[group('robot: recording (docs/adr/0021)')]
pilot-stop remote_host=default_jetrover_host:
    @ssh "jetson@$(just _robot-host "{{remote_host}}")" 'touch ~/recordings/.pilot-stop; p=$(cat ~/recordings/.pilot-onboard.pid 2>/dev/null); \
        for i in $(seq 1 60); do { [ -n "$p" ] && kill -0 "$p" 2>/dev/null; } || break; sleep 0.5; done; \
        if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then echo "still running after 30 s: just pilot-status"; else echo "stopped"; fi; \
        tail -n 8 ~/recordings/pilot-onboard.log 2>/dev/null; rm -f ~/recordings/.pilot-stop'

# Impair the robot's uplink: joint commands reach the arm through a Core network profile (e.g. `just uplink-impair 150ms-20j-0.5loss`), or `none` to stop. The arm then really receives late and lossy commands, so someone watches. RobotHost must have been deployed with --uplink-control (just deploy-robothost)
[group('robot: recording (docs/adr/0021)')]
uplink-impair profile seed="1" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    if [ "{{profile}}" = "none" ] || [ "{{profile}}" = "lan" ]; then
        ssh "$target" "rm -f {{robot_recordings}}/.uplink"
    else
        ssh "$target" "mkdir -p {{robot_recordings}} && printf '%s %s\n' '{{profile}}' '{{seed}}' > {{robot_recordings}}/.uplink"
    fi
    sleep 1
    ssh "$target" "journalctl -u teleop-robothost --no-pager -n 20 | grep '\[uplink\]' | tail -2" || true

# Impair the robot's uplink on a schedule that changes by itself (docs/adr/0021 decision 13), e.g. `just uplink-schedule "lan@0;150ms-20j-0.5loss@45;lan@90;50ms-5j@135"`: Core profiles from offsets in seconds, switched by RobotHost on time (what a profile held still arrives), each switch logged as UP. The arm then really receives late and lossy commands, so someone watches. `just uplink-impair none` stops it. RobotHost must have been deployed with --uplink-control (just deploy-robothost)
[group('robot: recording (docs/adr/0021)')]
uplink-schedule spec seed="1" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    case '{{spec}}' in *[!A-Za-z0-9.@\;_-]*|"") echo "error: spec is <profile>@<seconds>;... (named Core profiles)" >&2; exit 1 ;; esac
    case '{{seed}}' in *[!0-9]*|"") echo "error: seed is a whole number" >&2; exit 1 ;; esac
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    ssh "$target" "mkdir -p {{robot_recordings}} && printf '%s %s %s\n' schedule '{{seed}}' '{{spec}}' > {{robot_recordings}}/.uplink"
    sleep 1
    ssh "$target" "journalctl -u teleop-robothost --no-pager -n 20 | grep '\[uplink\]' | tail -3" || true

# Mark a moment in the running session (e.g. `just record-mark "payload on"`) -- moves nothing. Written over SSH on the Jetson's own clock: anything sending on UDP 6002 would steal the joint-state stream
[group('robot: recording (docs/adr/0021)')]
record-mark text remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    ssh "$target" "python3 - '{{text}}'" <<'PY'
    import hashlib, os, sys, time
    text = sys.argv[1].replace("|", "/").replace("\n", " ")
    now = time.monotonic_ns()
    control = "{{robot_recordings}}/.active"
    directory = open(control).read().strip() if os.path.exists(control) else ""
    if not directory:
        sys.exit("nothing is recording")
    path = os.path.join(directory, "marks.jlog")
    lines = []
    if os.path.exists(path):
        lines = open(path).read().splitlines()[:-1]  # drop the trailer; it is rewritten below
    else:
        lines = ["JSES|1|marks|1000000000|CLOCK_MONOTONIC|" + os.path.basename(directory.rstrip("/"))]
    lines.append("MK|%d|%s" % (now, text))
    body = "".join(l + "\n" for l in lines).encode("ascii")
    with open(path + ".tmp", "wb") as f:
        f.write(body + ("END|%d|0|%s\n" % (len(lines) - 1, hashlib.sha256(body).hexdigest())).encode("ascii"))
    os.replace(path + ".tmp", path)
    print("marked:", text)
    PY

# List the sessions on the robot, their sizes, and whether each log was closed -- moves nothing
[group('robot: recording (docs/adr/0021)')]
robot-recordings remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        'echo "recording now: $(cat {{robot_recordings}}/.active 2>/dev/null || echo nothing)"; \
         for d in {{robot_recordings}}/jetrover/*/; do [ -d "$d" ] || continue; \
           closed=$(for f in "$d"*.jlog; do tail -n 1 "$f" | grep -q "^END|" && echo -n "+" || echo -n "-"; done); \
           printf "%-60s %6s  logs closed: %s\n" "$(basename "$d")" "$(du -sh "$d" | cut -f1)" "$closed"; done; \
         df -h {{robot_recordings}} | tail -1'

# Copy new sessions from the robot into recordings/jetrover/ (gitignored; archive them to a GitHub Release), never overwriting one already here, then check each new one
[group('robot: recording (docs/adr/0021)')]
fetch-recordings remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    host=$(just _robot-host "{{remote_host}}")
    mkdir -p recordings/jetrover
    before=$(ls recordings/jetrover)
    # Not the session recording now, nor an automatic one RobotHost has closed but not yet completed
    # (docs/adr/0023): --ignore-existing would keep their half-written files here for good.
    skip=$(ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$host" 'r={{robot_recordings}}; a=$(cat $r/.active 2>/dev/null); [ -n "$a" ] && basename "$a";
        for d in $r/jetrover/*/; do [ -d "$d" ] || continue; [ -f "$d/files.sha256" ] || ! grep -q "\"label\": \"auto\"" "$d/session.json" 2>/dev/null || basename "$d"; done; true' | sort -u)
    excludes=()
    for s in $skip; do excludes+=(--exclude "/$s/"); echo "skipping $s: recording, or not completed yet" >&2; done
    rsync -a --ignore-existing ${excludes[@]+"${excludes[@]}"} -e "ssh -o StrictHostKeyChecking=accept-new" "{{remote_user}}@$host:{{robot_recordings}}/jetrover/" recordings/jetrover/
    new=$(comm -13 <(echo "$before" | sort) <(ls recordings/jetrover | sort) || true)
    if [ -z "$new" ]; then echo "nothing new"; exit 0; fi
    echo "fetched: $new"
    just recording-check $(for s in $new; do echo "recordings/jetrover/$s"; done)

# Is a recorded session fit to train or test on? Exits non-zero on drops, a missing trailer, clock-causality failures or read gaps over 500 ms (allow_gaps=true keeps gaps, annotated). Writes check.json into each session
[group('robot: recording (docs/adr/0021)')]
recording-check +sessions:
    #!/usr/bin/env bash
    set -euo pipefail
    PY=$(just _trainer-python)
    cd trainer
    args=()
    for s in {{sessions}}; do case "$s" in allow_gaps=true) args+=(--allow-read-gaps) ;; /*) args+=("$s") ;; *) args+=("../$s") ;; esac; done
    "../$PY" -m armpred check-session --json "${args[@]}"

# Prove RobotHost and the driver stamp their logs on one clock -- moves nothing. RobotHost (--clock-echo) answers Stopwatch readings that must fall between CLOCK_MONOTONIC reads taken just before and after each question
[group('robot: recording (docs/adr/0021)')]
jetson-clock-check remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" "python3 -" <<'PY'
    import subprocess, sys, time
    p = subprocess.Popen(["/home/jetson/.dotnet/dotnet", "/home/jetson/robothost_current/Teleop.RobotHost.dll", "--clock-echo"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True, bufsize=1)
    print("python:", time.get_clock_info("monotonic"))
    worst, bad = 0, 0
    for i in range(50):
        before = time.monotonic_ns(); p.stdin.write("t\n"); p.stdin.flush()
        ticks, freq = (int(x) for x in p.stdout.readline().split()); after = time.monotonic_ns()
        ns = ticks * 1000000000 // freq
        if not before <= ns <= after: bad += 1
        worst = max(worst, after - before)
    p.stdin.close(); p.wait()
    print("Stopwatch frequency %d; %d of 50 readings outside their bracket; widest bracket %.3f ms" % (freq, bad, worst / 1e6))
    sys.exit("FAIL: RobotHost and the driver are not on the same clock" if bad else 0)
    PY

# Correct a fetched session's slider label (its conditions.speedSliderRadPerSec), e.g. `just recording-set-slider recordings/jetrover/20261025T200935Z-teleop-2p5 3.8` -- the local copy only; the robot's copy keeps the old label. Checks the value against the speed every joint command in the session actually carried (the headset's slider rides in each command's speed field) and refuses if they disagree, unless the value is `varied` and they did vary. Notes the change in session.json, rewrites files.sha256, and reruns just recording-check. The model never sees the label; splits and stratification do
[group('robot: recording (docs/adr/0021)')]
recording-set-slider session value:
    #!/usr/bin/env bash
    set -euo pipefail
    python3 -I - "{{session}}" "{{value}}" <<'PY'
    import datetime, hashlib, json, os, sys
    d, value = sys.argv[1], sys.argv[2]
    speeds = set()
    with open(os.path.join(d, "robothost.jlog")) as f:
        for line in f:
            if line.startswith("JC|"):
                for target in line.rstrip("\n").split("|")[6].split(";"):
                    motor, _, speed = target.split(":")
                    if motor != "10":
                        speeds.add(round(float(speed), 2))
    if not speeds:
        sys.exit(f"{d}: no joint commands, so nothing to check the label against")
    if value == "varied":
        if len(speeds) < 2:
            sys.exit(f"{d}: every command carried {speeds.pop()} rad/s; it did not vary")
        new = "varied"
    else:
        new = float(value)
        if speeds != {round(new, 2)}:
            sys.exit(f"{d}: the commands carried {sorted(speeds)} rad/s, not only {new}; pass one of those, or varied")
    path = os.path.join(d, "session.json")
    meta = json.load(open(path))
    old = meta["conditions"]["speedSliderRadPerSec"]
    if old == new:
        print(f"{d}: already {new}")
        sys.exit(0)
    meta["conditions"]["speedSliderRadPerSec"] = new
    note = (f"slider relabelled {old} -> {new} on {datetime.date.today().isoformat()}: the joint commands carried "
            f"{', '.join(str(s) for s in sorted(speeds))} rad/s")
    meta["notes"] = (meta.get("notes") + "; " if meta.get("notes") else "") + note
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)
    sums = os.path.join(d, "files.sha256")
    lines = []
    for line in open(sums):
        digest, name = line.rstrip("\n").split("  ", 1)
        if name == "session.json":
            digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
        lines.append(f"{digest}  {name}")
    open(sums, "w").write("\n".join(lines) + "\n")
    print(f"{d}: {note}")
    PY
    just recording-check "{{session}}" | tail -3

# Pack checked sessions into the one file the trainer needs: dist/jetrover-data-<UTC>.tar, with an index, the robot profile and this commit (the format: trainer/armpred/contract/bundle.py). Refuses a session whose check.json is missing or failed (run just recording-check first)
[group('robot: recording (docs/adr/0021)')]
pack-dataset +sessions:
    #!/usr/bin/env bash
    set -euo pipefail
    PY=$(just _trainer-python)
    mkdir -p dist
    out="dist/jetrover-data-$(date -u +%Y%m%dT%H%M%SZ).tar"
    args=()
    for s in {{sessions}}; do case "$s" in /*) args+=("$s") ;; *) args+=("../$s") ;; esac; done
    cd trainer
    "../$PY" -m armpred pack-dataset --out "../$out" --profile ../core/RobotProfiles/jetrover.json --repo-sha "$(git rev-parse HEAD)" "${args[@]}"

# ---- everything ----

# Every verification gate in the repo: core/ dotnet tests + Bridge/-vs-Core compile + analysis/ python tests
[group('checks')]
check: core-check bridge-check test
