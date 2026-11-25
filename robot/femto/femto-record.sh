#!/usr/bin/env bash
# Record depth + colour (+ IR) from the robot's Orbbec Femto Bolts through Orbbec SDK v2, with each
# device's factory calibration, then pull the recording to this machine. Moves nothing: it touches
# only the two Femto Bolt camera senders (teleop-camerahost-left/-right), which it stops for the
# capture and ALWAYS starts again (trap), never teleop-camerahost (the DaBai), RobotHost or ROS.
# Run through `just femto-record` (see the justfile); the recorder is robot/femto/femto_record.cpp.
#
# usage: femto-record.sh <host> <label> <seconds> <fps> <ir_every> <local_root> [probe]
set -euo pipefail
host=$1 label=$2 seconds=$3 fps=$4 ir_every=$5 local_root=$6 mode=${7:-record}
here=$(cd "$(dirname "$0")" && pwd)
sdk_ver=v2.10.6
sdk_tgz=OrbbecSDK_v2.10.6_202609291004_575347c_linux_arm64.tar.gz
# USB port -> twin name (ADR 0014 amendments): 1.4.3 is left, 1.4.2 is right.
ssh_opts=(-o StrictHostKeyChecking=accept-new -o ServerAliveInterval=15)
r="jetson@$host"
stamp=$(date -u +%Y%m%dT%H%M%SZ)
name="$stamp-$label"

echo "--- SDK and recorder on the robot (user space: /home/jetson/orbbec)"
ssh "${ssh_opts[@]}" "$r" "mkdir -p ~/orbbec/src && cat > ~/orbbec/src/femto_record.cpp.new" < "$here/femto_record.cpp"
ssh "${ssh_opts[@]}" "$r" "SDK_VER=$sdk_ver SDK_TGZ=$sdk_tgz bash -s" <<'REMOTE'
set -euo pipefail
cd ~/orbbec
if [ ! -d "$SDK_VER" ]; then
    curl -sSL -o sdk.tar.gz "https://github.com/orbbec/OrbbecSDK_v2/releases/download/$SDK_VER/$SDK_TGZ"
    mkdir -p "$SDK_VER" && tar xzf sdk.tar.gz -C "$SDK_VER" && rm sdk.tar.gz
fi
SDK=$(ls -d ~/orbbec/$SDK_VER/OrbbecSDK_*)
if ! cmp -s src/femto_record.cpp.new src/femto_record.cpp || [ ! -x femto_record ]; then
    mv src/femto_record.cpp.new src/femto_record.cpp
    echo "building femto_record"
    g++ -O2 -std=c++14 -I"$SDK/include" src/femto_record.cpp -o femto_record -L"$SDK/lib" -lOrbbecSDK -lpng -lpthread -Wl,-rpath,"$SDK/lib"
else
    rm -f src/femto_record.cpp.new
fi
REMOTE

echo "--- capture ($mode): the left/right camera senders stop now and start again afterwards"
ssh "${ssh_opts[@]}" "$r" "NAME=$name SECONDS_=$seconds FPS=$fps IR_EVERY=$ir_every MODE=$mode SDK_VER=$sdk_ver bash -s" <<'REMOTE'
set -uo pipefail
svcs="teleop-camerahost-left teleop-camerahost-right"
restore() {
    sudo systemctl start $svcs
    sleep 4
    for p in 1.4.2 1.4.3; do
        if [ ! -e "/dev/v4l/by-path/platform-70090000.xusb-usb-0:$p:1.0-video-index0" ]; then
            # The SDK's libuvc backend detaches uvcvideo; re-enumerate the port to give it back.
            echo "re-enumerating usb 2-$p (its V4L2 node is missing)"
            echo 0 | sudo tee /sys/bus/usb/devices/2-$p/authorized > /dev/null; sleep 1
            echo 1 | sudo tee /sys/bus/usb/devices/2-$p/authorized > /dev/null; sleep 4
            sudo systemctl restart $svcs
        fi
    done
    sleep 3
    for s in $svcs; do echo "$s: $(systemctl is-active $s)"; done
    ls -l /dev/v4l/by-path/ | grep -E '1\.4\.[23]:1\.0' || true
}
trap restore EXIT
SDK=$(ls -d ~/orbbec/$SDK_VER/OrbbecSDK_*)
labels="$(cat /sys/bus/usb/devices/2-1.4.3/serial)=left,$(cat /sys/bus/usb/devices/2-1.4.2/serial)=right"
echo "labels: $labels"
free_gb=$(df -BG --output=avail /home/jetson | tail -1 | tr -dc 0-9)
echo "free on /home/jetson: ${free_gb} GB"
if [ "$MODE" = record ] && [ "$free_gb" -lt 4 ]; then echo "error: under 4 GB free, not recording" >&2; exit 1; fi
sudo systemctl stop $svcs
sleep 2
out=/home/jetson/recordings/depth/$NAME
mkdir -p "$out"
cd "$SDK/lib"   # the SDK reads OrbbecSDKConfig.xml and its extensions (depth engine) from here
if [ "$MODE" = probe ]; then
    sudo "$HOME/orbbec/femto_record" --probe --labels "$labels"
    rc=$?
    rmdir "$out" 2>/dev/null
else
    t0=$(date +%s)
    ( while sleep 10; do top -b -n 1 | head -15 | grep -E 'femto_rec|^%Cpu|load average' ; done ) > "$out/cpu.log" 2>&1 &
    mon=$!
    sudo "$HOME/orbbec/femto_record" --out "$out" --seconds "$SECONDS_" --fps "$FPS" --ir-every "$IR_EVERY" --labels "$labels" 2>&1 | tee "$out/record.log"
    rc=${PIPESTATUS[0]}
    kill $mon 2>/dev/null
    sudo chown -R jetson:jetson "$out"
    echo "recording took $(( $(date +%s) - t0 )) s, rc=$rc, size $(du -sh "$out" | cut -f1)"
fi
exit $rc
REMOTE
rc=$?
[ "$mode" = probe ] && exit $rc

echo "--- pull to $local_root/$name"
mkdir -p "$local_root/$name"
ssh "${ssh_opts[@]}" "$r" "tar -C /home/jetson/recordings/depth/$name -cf - ." | tar -xf - -C "$local_root/$name"
remote_n=$(ssh "${ssh_opts[@]}" "$r" "find /home/jetson/recordings/depth/$name -type f | wc -l")
local_n=$(find "$local_root/$name" -type f | wc -l)
echo "files: robot $remote_n, here $local_n"
if [ "$remote_n" = "$local_n" ]; then
    ssh "${ssh_opts[@]}" "$r" "rm -rf /home/jetson/recordings/depth/$name"
    echo "removed the robot's copy"
else
    echo "error: file counts differ; the robot's copy is kept at /home/jetson/recordings/depth/$name" >&2
    exit 1
fi
exit $rc
