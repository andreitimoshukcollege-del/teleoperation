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

# Print the robot's current IPv4 address and how it was found -- moves nothing. Tries its mDNS name (jetrover-sinrg.local, on the lab LAN), then Tailscale (the direct LAN path if it has one, else the 100.x address). Every robot recipe does this lookup itself unless JETROVER_HOST is set
[group('robot: checks and diagnostics (nothing moves)')]
robot-ip:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -n "${JETROVER_HOST:-}" ]; then
        echo "note: JETROVER_HOST=$JETROVER_HOST is set, so recipes use that instead of the address below" >&2
    fi
    just _robot-host auto

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

# Interactively author a new RobotArmProfile JSON (docs/adr/0011); answers prompts from the terminal, e.g. `just build-profile`
[group('robot: profiles')]
build-profile output="" force="false":
    cd core && dotnet run --project Teleop.Eval -- build-profile \
        {{ if output != "" { "--output " + output } else { "" } }} \
        {{ if force == "true" { "--force" } else { "" } }}

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

# Publish Teleop.RobotHost for the Jetson (linux-arm64), copy it over, and restart its systemd service -- needs passwordless ssh/scp; does NOT touch the Jetson's ROS 2 nodes, see robot/systemd/README.md
[group('robot: deploy and restart (someone at the robot)')]
deploy-robothost remote_host=default_jetrover_host remote_user="jetson" operator_host=default_operator_host max_direction_magnitude="" profile_path="" pulses_per_second="":
    #!/usr/bin/env bash
    set -euo pipefail
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
        --uplink-control /home/jetson/recordings/.uplink ${extra_args}
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

# Read-only: when the board has stopped answering (just joint-probe shows every motor sensed NO), find out which side is dead, without opening the serial port (opening it can reset the board and erase the evidence). Shows: the robot's uptime; the kernel's power and USB alarms since boot; the driver's threads and what each waits on; whether any bytes reach the driver over a few seconds; how often the board's own battery report and the servo feedback still arrive; and any receive errors in the driver's log. Run it while the fault is present, before any restart
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

# Restart the servo bus driver (jetrover-arm-control), which reopens its serial link to the Hiwonder board. For when the board stopped answering (just ros-check shows no feedback_array or battery messages) after a brown-out or a reset. It interrupts the arm, and the shoulder may sag while the driver comes back (seen 2026-10-12), so someone must be at the robot. Afterwards the feedback should flow again
[group('robot: deploy and restart (someone at the robot)')]
restart-arm-driver remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@$(just _robot-host "{{remote_host}}")" \
        "sudo systemctl restart jetrover-arm-control; sleep 5; systemctl status jetrover-arm-control --no-pager -n 8"
    just ros-check 4

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
        echo '--- the plain-Python tests (relay protocol, stale tracking, servo controller, session log):'
        PYTHONPATH=src/teleop_relay:src/jetrover_arm_control python3 -m pytest -q -p no:cacheprovider \
            src/teleop_relay/test/test_relay_protocol.py src/teleop_relay/test/test_stale_tracked_value.py \
            src/jetrover_arm_control/test/test_servo_controller.py \
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

# Publish Teleop.CameraHost for the Jetson, copy it over, write ~/camerahost-run.sh and restart teleop-camerahost if it is installed (see robot/systemd/README.md) -- moves nothing; does not touch teleop-robothost or the ROS nodes
[group('robot: deploy and restart (someone at the robot)')]
deploy-camerahost fps="30" device="auto" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
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
        --device {{device}} --fps {{fps}} --port 6003
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

# Start recording on the robot -- moves nothing. Writes session.json (the conditions describe the session for splits and stratification, never the model's inputs) and the control file RobotHost and the driver both watch, then checks both logs are growing. load: none, payload or hand; slider: rad/s or varied; network: the robot-side uplink profile, lan for none, or schedule:<profile>@<s>;... for one that changes mid-session (just uplink-schedule)
[group('robot: recording (docs/adr/0021)')]
record-start label slider load="none" payload_g="0" network="lan" added_delay_ms="0" motion="teleop" operator="A" notes="" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{load}}" in none|payload|hand) ;; *) echo "error: load must be none, payload or hand" >&2; exit 1 ;; esac
    case "{{label}}" in *[!A-Za-z0-9._-]*) echo "error: label may hold only letters, digits, . _ -" >&2; exit 1 ;; esac
    host=$(just _robot-host "{{remote_host}}")
    target="{{remote_user}}@$host"
    id="$(date -u +%Y%m%dT%H%M%SZ)-{{label}}"
    dir="{{robot_recordings}}/jetrover/$id"
    if ssh -o StrictHostKeyChecking=accept-new "$target" "test -s {{robot_recordings}}/.active"; then
        echo "error: a session is already recording ($(ssh "$target" cat {{robot_recordings}}/.active)) -- just record-stop first" >&2
        exit 1
    fi
    builds=$(ssh "$target" "cat ~/robothost_current/BUILD_INFO 2>/dev/null || echo '{}'; cat ~/teleop_ros_current/BUILD_INFO 2>/dev/null || echo '{}'")
    meta=$(BUILDS="$builds" python3 - "$id" "{{label}}" "{{slider}}" "{{load}}" "{{payload_g}}" "{{network}}" "{{added_delay_ms}}" "{{motion}}" "{{operator}}" "{{notes}}" "$(sha256sum core/RobotProfiles/jetrover.json | cut -d' ' -f1)" <<'PY'
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
    ssh "$target" "printf '%s\n' $dir > {{robot_recordings}}/.active"
    sleep 2.5
    sizes=$(ssh "$target" "stat -c '%n %s' $dir/robothost.jlog $dir/driver.jlog 2>&1 || true")
    echo "$sizes"
    ok=true
    case "$sizes" in *"robothost.jlog"*" 0"|*"robothost.jlog'"*|*"No such file"*robothost*) echo "warning: RobotHost is not recording -- redeploy it (just deploy-robothost) so it runs with --record-control" >&2; ok=false ;; esac
    case "$sizes" in *"driver.jlog'"*|*"No such file"*driver*) echo "warning: the driver is not recording -- just deploy-ros (someone watching: it restarts the servo driver)" >&2; ok=false ;; esac
    $ok || exit 1
    network='{{network}}'
    if [ "${network#schedule:}" != "$network" ]; then
        # A schedule starts once the logs are open, so robothost.jlog records every change (UP).
        just uplink-schedule "${network#schedule:}" 1 "{{remote_host}}" "{{remote_user}}"
    elif [ "$network" != "lan" ] && [ "$network" != "none" ]; then
        # The uplink changes once the logs are open, so robothost.jlog records the change (UP).
        just uplink-impair "$network" 1 "{{remote_host}}" "{{remote_user}}"
    fi
    echo "recording $id"

# Stop the robot's recording -- moves nothing. Removes the control file, waits for both logs' END trailers, stamps stoppedUtc and writes files.sha256
[group('robot: recording (docs/adr/0021)')]
record-stop remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    target="{{remote_user}}@$(just _robot-host "{{remote_host}}")"
    ssh "$target" 'bash -s' <<'SH'
    set -euo pipefail
    control={{robot_recordings}}/.active
    [ -s "$control" ] || { echo "nothing is recording" >&2; exit 1; }
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
    json.dump(meta, open(path, "w"), indent=2)
    PY
    cd "$dir" && sha256sum session.json robot-profile.json *.jlog > files.sha256
    for f in *.jlog; do printf '%-16s %s\n' "$f" "$(tail -n 1 "$f")"; done
    echo "stopped $(basename "$dir")"
    SH

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
    rsync -a --ignore-existing -e "ssh -o StrictHostKeyChecking=accept-new" "{{remote_user}}@$host:{{robot_recordings}}/jetrover/" recordings/jetrover/
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
