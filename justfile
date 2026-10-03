# Teleop Research Platform -- convenience commands.
# Install `just`: https://github.com/casey/just
# `just` with no arguments lists everything below.

default:
    @just --list

# ---- core/ (dotnet: algorithms, unit tests, sweep) ----

# Unit + allocation tests for Teleop.Core
core-test:
    cd core && dotnet test

# Replay the golden .tlog twice, assert identical (determinism gate)
core-verify:
    cd core && dotnet run --project Teleop.Eval -- verify

# Invariant + registry-completeness check over the built assembly
core-audit:
    cd core && dotnet run --project Teleop.Eval -- audit

# Run all three core/ gates -- "Verify your work" in root CLAUDE.md
core-check: core-test core-verify core-audit

# Compile Bridge/ against Core headlessly -- catches a Core API change silently breaking unity/. NOT a substitute for opening the editor; see unity/BridgeCheck/README.md
bridge-check:
    cd unity/BridgeCheck && dotnet build --nologo

# Copy core/testdata/traces/*.trace where Unity's Editor can read them (Application.persistentDataPath), for NetworkImpairmentController's trace mode. On device, `adb push` them to the same place instead.
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

# Internal: run one Unity project's editor headless with -executeMethod; picks the editor version the project pins, refuses while that project is open in an editor, prints the log's verdict lines
_unity-batch project method extra="":
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
    "$exe" -batchmode -nographics -quit -projectPath "$winproj" -executeMethod {{method}} \
        -logFile "$(wslpath -w "$log")" {{extra}}
    code=$?
    set -e
    grep -a -E '^\[(scene-check|xr-setup|xr-scene|build)\]|error CS[0-9]+' "$log" | sed 's/\r$//' | tail -40 || true
    if [ "$code" -ne 0 ]; then
        echo "Unity exited $code. Full log: $log" >&2
    fi
    exit "$code"

# Headless check of unity/TeleopVR (Unity 2022.3): compiles, and every JetRover scene opens with no missing scripts and its Bridge wiring intact -- moves nothing; close that project's editor first
unity-check-vr:
    just _unity-batch TeleopVR Teleop.Editor.SceneIntegrityCheck.Run

# Configure unity/TeleopXR (Galaxy XR, Unity 6.6) from code: XRI/XR Hands samples, URP, Android player settings, OpenXR for Android and Direct Preview. Idempotent; the first run imports samples, so run it twice on a fresh clone. Close that project's editor first
unity-setup-xr:
    just _unity-batch TeleopXR Teleop.XR.Editor.XrProjectSetup.Run

# Build unity/TeleopXR's Galaxy XR scene (JetRoverLabXR) from code: hands rig, arm, pinch-grab target, camera panel. Create-only; rebuild=true replaces it, discarding hand edits. Run unity-setup-xr first
unity-scene-xr rebuild="false":
    just _unity-batch TeleopXR Teleop.XR.Editor.XrLabSceneBuilder.Run {{ if rebuild == "true" { "-teleopRebuildScene" } else { "" } }}

# Headless check of unity/TeleopXR's scene: no missing scripts, Bridge wiring intact, pinch-grab settings safe for a real arm (no pull-to-hand, no snap, no smoothing, no throw), hands only
unity-check-xr:
    just _unity-batch TeleopXR Teleop.XR.Editor.XrSceneCheck.Run

# Build the Galaxy XR APK (unity/TeleopXR, Unity 6.6) after its scene check -> unity/TeleopXR/Builds/TeleopXR.apk. Close that project's editor first
build-galaxy:
    just _unity-batch TeleopXR Teleop.XR.Editor.XrBuild.Run "-buildTarget Android"

# Build the Quest APK (unity/TeleopVR, Unity 2022.3) from its Build Settings scenes after its scene check -> unity/TeleopVR/Builds/TeleopVR-Quest.apk. The first Android build re-imports assets; close that project's editor first
build-quest:
    just _unity-batch TeleopVR Teleop.Editor.QuestBuild.Run "-buildTarget Android"

# Internal: the adb that ships with the Unity editor unity/TeleopXR pins (or $ADB), run on the Windows side
_adb *args:
    #!/usr/bin/env bash
    set -euo pipefail
    ver=$(sed -n 's/^m_EditorVersion: //p' unity/TeleopXR/ProjectSettings/ProjectVersion.txt | tr -d '\r')
    adb="${ADB:-/mnt/c/Program Files/Unity/Hub/Editor/$ver/Editor/Data/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb.exe}"
    [ -x "$adb" ] || { echo "error: adb not found at $adb (install Unity $ver's Android SDK module, or set ADB)" >&2; exit 1; }
    "$adb" {{args}}

# Install the Galaxy XR APK on the headset over USB (developer mode and USB debugging on; accept the prompt in the headset)
install-galaxy:
    #!/usr/bin/env bash
    set -euo pipefail
    apk=unity/TeleopXR/Builds/TeleopXR.apk
    [ -f "$apk" ] || { echo "error: $apk not found; run just build-galaxy first" >&2; exit 1; }
    just _adb devices -l
    just _adb install -r "$(wslpath -w "$apk")"

# Point the Galaxy XR app at the robot without a rebuild: pushes a full jetrover_connection.json override (the Bridge default with RemoteHost replaced) to the app's files directory. On SINRG use the robot's LAN address, e.g. JETROVER_HOST=10.188.57.2
push-galaxy-config host=default_jetrover_host:
    #!/usr/bin/env bash
    set -euo pipefail
    tmp=$(mktemp --suffix=.json)
    trap 'rm -f "$tmp"' EXIT
    python3 - "$tmp" "{{host}}" <<'PY'
    import json, sys
    cfg = json.load(open("unity/Teleop.Bridge/Runtime/Resources/jetrover_connection.json", encoding="utf-8"))
    cfg["RemoteHost"] = sys.argv[2]
    json.dump(cfg, open(sys.argv[1], "w"), indent=2)
    PY
    dest=/sdcard/Android/data/com.teleop.jetroverxr/files
    just _adb shell mkdir -p "$dest"
    just _adb push "$(wslpath -w "$tmp")" "$dest/jetrover_connection.json"
    echo "Galaxy XR app now targets {{host}}; restart the app to pick it up."

# Run an experiment sweep, e.g. `just sweep experiments/exp-001-predictor-baseline.yaml`
sweep config:
    cd core && dotnet run --project Teleop.Eval -- sweep ../{{config}}

# ---- JetRover hardware (a human must be watching anything that moves the arm) ----

# Where the JetRover and this operator machine are reachable from here. The defaults are their
# Tailscale addresses. For a direct LAN path (both machines on the same Wi-Fi), export
# JETROVER_HOST and OPERATOR_HOST in your shell (`just` parameters are positional, so `remote_host=`
# on the command line does not work).
# The ssh/scp recipes accept IPv6 literals, but WSL's default NAT networking has no IPv6 route and
# the UDP transports in Teleop.RobotHost and Teleop.Eval are IPv4-only, so use IPv4 for now.
default_jetrover_host := env_var_or_default("JETROVER_HOST", "100.112.90.72")
default_operator_host := env_var_or_default("OPERATOR_HOST", "100.82.140.80")

# Move the real JetRover arm to a Cartesian target (wrist frame, meters) and hold, e.g. `just move-arm 0.15 0 0.08` -- requires Teleop.RobotHost running on the Jetson and a human watching the hardware
move-arm x y z gripper="0" remote_host=default_jetrover_host remote_port="6000" local_port="6001":
    cd core && dotnet run --project Teleop.Eval -- move-arm \
        --x {{x}} --y {{y}} --z {{z}} --gripper {{gripper}} \
        --remote-host {{remote_host}} --remote-port {{remote_port}} --local-port {{local_port}} \
        --confirm-hardware-motion

# Interactively author a new RobotArmProfile JSON (docs/adr/0011); answers prompts from the terminal, e.g. `just build-profile`
build-profile output="" force="false":
    cd core && dotnet run --project Teleop.Eval -- build-profile \
        {{ if output != "" { "--output " + output } else { "" } }} \
        {{ if force == "true" { "--force" } else { "" } }}

# Phase-3 cross-machine ClockSync diagnostic against an already-running Teleop.RobotHost -- moves the real arm once and holds; a human must be watching the hardware
clocksync-check remote_host=default_jetrover_host remote_port="6000" local_port="6001" rate_hz="20" duration_seconds="20":
    cd core && dotnet run --project Teleop.Eval -- clocksync-check \
        --remote-host {{remote_host}} --remote-port {{remote_port}} --local-port {{local_port}} \
        --rate-hz {{rate_hz}} --duration-seconds {{duration_seconds}} \
        --confirm-hardware-motion

# Publish Teleop.RobotHost for the Jetson (linux-arm64), copy it over, and restart its systemd service -- needs passwordless ssh/scp; does NOT touch the Jetson's ROS 2 nodes, see robot/systemd/README.md
deploy-robothost remote_host=default_jetrover_host remote_user="jetson" operator_host=default_operator_host max_direction_magnitude="" profile_path="":
    #!/usr/bin/env bash
    set -euo pipefail
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
    tar czf "$archive" -C "$publish_dir" .
    echo "Copying to {{remote_user}}@{{remote_host}}:~/${remote_dir}..." >&2
    # scp, unlike ssh, needs an IPv6 literal in brackets to tell the address from the path.
    case "{{remote_host}}" in *:*) scp_dest="{{remote_user}}@[{{remote_host}}]" ;; *) scp_dest="{{remote_user}}@{{remote_host}}" ;; esac
    scp -o StrictHostKeyChecking=accept-new "$archive" "${scp_dest}:/tmp/${remote_dir}.tar.gz"
    extra_args=""
    if [ -n "{{max_direction_magnitude}}" ]; then extra_args="$extra_args --max-direction-magnitude {{max_direction_magnitude}}"; fi
    if [ -n "{{profile_path}}" ]; then extra_args="$extra_args --profile-path {{profile_path}}"; fi
    # robothost-run.sh is the stable script teleop-robothost.service's ExecStart points at
    # (robot/systemd/teleop-robothost.service) -- regenerated on every deploy with this run's exact
    # args (including any optional extra_args), so redeploying never needs `systemctl
    # daemon-reload` or touching the unit file itself, only `systemctl restart`. This also sidesteps
    # the earlier pkill+nohup approach's self-inflicted-signal bug entirely (systemd owns the
    # process lifecycle instead of a hand-rolled pkill matching its own ancestor shell).
    # --remote-host is only where replies go before the first command arrives: Teleop.RobotHost
    # defaults to --reply-to sender and then replies to wherever each command came from, so a
    # changed operator address no longer needs a redeploy. Add `--reply-to fixed` to pin it.
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" "cat > /home/jetson/robothost-run.sh" <<EOF
    #!/bin/bash
    exec /home/jetson/.dotnet/dotnet /home/jetson/robothost_current/Teleop.RobotHost.dll \
        --local-port 6000 --remote-host {{operator_host}} --remote-port 6001 \
        --relay-socket /tmp/jetrover_relay.sock --local-relay-socket /tmp/teleop_robot_host.sock \
        --joint-local-port 6002 ${extra_args}
    EOF
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" "
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

# SSH in and print systemctl status for all three Jetson-side services (teleop-robothost, jetrover-relay, jetrover-arm-control) in one shot -- see robot/systemd/README.md
robot-status remote_host=default_jetrover_host remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" \
        "systemctl status teleop-robothost jetrover-relay jetrover-arm-control --no-pager -n 5"

# Read-only: the Jetson's Wi-Fi profiles, visible networks, IPv4/IPv6 addresses, routes and Tailscale state -- moves nothing; run before and after `robot-wifi-join`
robot-net-status remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" 'bash -s' <<'REMOTE'
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
robot-camera-check device="/dev/video0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    target="{{remote_user}}@{{remote_host}}"
    case "{{remote_host}}" in *:*) scp_src="{{remote_user}}@[{{remote_host}}]" ;; *) scp_src="$target" ;; esac
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
camera-timestamp-spike frames="150" fps="30" device="/dev/video0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    cd core
    # Relative output path for the Windows dotnet, as in deploy-robothost.
    publish_dir="_scratch_camspike"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.CameraHost for linux-arm64..." >&2
    dotnet publish Teleop.CameraHost -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir" --nologo -v quiet
    tar czf "$archive" -C "$publish_dir" .
    target="{{remote_user}}@{{remote_host}}"
    case "{{remote_host}}" in *:*) scp_dest="{{remote_user}}@[{{remote_host}}]" ;; *) scp_dest="$target" ;; esac
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
camera-serve seconds="30" fps="30" device="/dev/video0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    cd core
    publish_dir="_scratch_camserve"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.CameraHost for linux-arm64..." >&2
    dotnet publish Teleop.CameraHost -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir" --nologo -v quiet
    tar czf "$archive" -C "$publish_dir" .
    target="{{remote_user}}@{{remote_host}}"
    case "{{remote_host}}" in *:*) scp_dest="{{remote_user}}@[{{remote_host}}]" ;; *) scp_dest="$target" ;; esac
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
camera-probe seconds="10" max_fps="0" save="" remote_host=default_jetrover_host:
    cd core && dotnet run --project Teleop.CameraHost -c Release -- probe --host {{remote_host}} \
        --seconds {{seconds}} --max-fps {{max_fps}} {{ if save != "" { "--save '" + save + "'" } else { "" } }}

# Publish Teleop.CameraHost for the Jetson, copy it over, write ~/camerahost-run.sh and restart teleop-camerahost if it is installed (see robot/systemd/README.md) -- moves nothing; does not touch teleop-robothost or the ROS nodes
deploy-camerahost fps="30" device="/dev/video0" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
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
    target="{{remote_user}}@{{remote_host}}"
    case "{{remote_host}}" in *:*) scp_dest="{{remote_user}}@[{{remote_host}}]" ;; *) scp_dest="$target" ;; esac
    scp -q -o StrictHostKeyChecking=accept-new "$archive" "${scp_dest}:/tmp/${remote_dir}.tar.gz"
    # Stable wrapper the unit's ExecStart points at, rewritten on every deploy (same pattern as
    # deploy-robothost), so a redeploy needs only a restart. `exec` so SIGTERM reaches the sender.
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
install-camerahost-service remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    target="{{remote_user}}@{{remote_host}}"
    case "{{remote_host}}" in *:*) scp_dest="{{remote_user}}@[{{remote_host}}]" ;; *) scp_dest="$target" ;; esac
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
robot-wifi-join ssid revert_minutes="5" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    password="${JETROVER_WIFI_PASSWORD:-}"
    if [ -z "$password" ]; then
        read -r -s -p "Wi-Fi password for '{{ssid}}': " password
        echo
    fi
    target="{{remote_user}}@{{remote_host}}"
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
robot-wifi-confirm remote_host=default_jetrover_host remote_user="jetson":
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" \
        "if systemctl is-active --quiet jetrover-wifi-revert.timer; then sudo systemctl stop jetrover-wifi-revert.timer && echo 'fallback cancelled; staying on this network'; else echo 'no fallback pending'; fi"

# Pin the Jetson's wired IPv4 address instead of taking it from DHCP, e.g. `just robot-static-ip 10.188.57.2/21 10.188.56.1`. Keeps the DNS servers DHCP gave it. Reverts to DHCP in `revert_minutes` unless `just robot-static-ip-confirm` runs first. Only use an address the network admin has approved; DHCP does not know about it
robot-static-ip address gateway connection="Wired connection 1" revert_minutes="5" remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" \
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
robot-static-ip-confirm remote_host=default_jetrover_host remote_user="jetson":
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" \
        "if systemctl is-active --quiet jetrover-static-ip-revert.timer; then sudo systemctl stop jetrover-static-ip-revert.timer && echo 'fallback cancelled; static address kept'; else echo 'no fallback pending'; fi; nmcli -g ipv4.method,ipv4.addresses connection show 'Wired connection 1'"

# Rename the Jetson (hostname, /etc/hosts, its mDNS name <name>.local, and its Tailscale machine name) -- moves nothing. docs/adr/0015 finds the robot by this name, so it must be unique on the LAN
robot-set-hostname name remote_host=default_jetrover_host remote_user="jetson":
    #!/usr/bin/env bash
    set -euo pipefail
    if ! printf '%s' '{{name}}' | grep -Eq '^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$'; then
        echo "error: '{{name}}' is not a valid hostname (lowercase letters, digits, inner hyphens, at most 63)" >&2
        exit 1
    fi
    ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" "bash -s -- {{name}}" <<'REMOTE'
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
test: _analysis-venv
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    PY=$([ -f .venv/Scripts/python.exe ] && echo .venv/Scripts/python.exe || echo .venv/bin/python)
    "$PY" -m pytest -v

# Opens a GUI window to configure and run a sweep, then view its figures (needs a real display)
experiment-gui: _analysis-venv
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    PY=$([ -f .venv/Scripts/python.exe ] && echo .venv/Scripts/python.exe || echo .venv/bin/python)
    "$PY" run_tests.py

# Generate figures + summary table for a run, e.g. `just report results/exp-001-predictor-baseline/20260804-020431Z`
report run_dir: _analysis-venv
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    PY=$([ -f .venv/Scripts/python.exe ] && echo .venv/Scripts/python.exe || echo .venv/bin/python)
    "$PY" -m teleop_analysis.cli ../{{run_dir}}

# ---- everything ----

# Every verification gate in the repo: core/ dotnet tests + Bridge/-vs-Core compile + analysis/ python tests
check: core-check bridge-check test
