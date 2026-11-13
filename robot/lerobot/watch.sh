#!/usr/bin/env bash
# The LeRobot converter on the GPU machine (docs/adr/0036). Run by the justfile's lerobot-* recipes, by isaac-host /
# isaac-jetrover-host (ensure), and by cron (keep); never by hand on a PC.
#
#   start <commit>   (re)start the watcher at <commit>, from its own checkout ~/teleoperation-lerobot (so an Isaac host
#                    started at another commit never changes the code under a running watcher), and install the keeper
#   ensure [commit]  start it if it is not running: at <commit>, or at the checkout's current commit
#   keep             cron's every-5-minutes and at-boot call: ensure, silent unless it had to start it
#   stop             remove the keeper, then SIGTERM, which the watcher honours between takes (up to 10 min for the
#                    take it is writing), then SIGKILL
#   status           running or not, the keeper, the log's tail, every dataset with its episodes
#
# The keeper is two lines in our crontab, tagged with $tag: the GPU machine's account has no lingering systemd user
# session (enabling one needs root), and cron runs without anyone logged in. It brings the watcher back within 5
# minutes of a reboot or a crash. A real take meanwhile waits on the robot, which keeps it until it is copied.
set -euo pipefail
cmd=${1:-status}
commit=${2:-}
repo=$HOME/teleoperation-lerobot
venv=$HOME/isaacsim/lerobot-venv
out=$HOME/isaacsim/lerobot
pidfile=$out/watch.pid
log=$out/watch.log
lock=$out/.watch.lock
tag="# teleop-lerobot-keeper (docs/adr/0036; just lerobot-watch-stop removes it)"
# The scenario roots: the FR3 Duo's and the JetRover's Isaac takes, and the real robot's (copied in by ADR 0036
# decision 7). A root that does not exist yet is skipped each pass.
roots=(--root "$HOME/isaacsim/scenarios" --root "$HOME/isaacsim/scenarios-jetrover" --root "$HOME/isaacsim/scenarios-real")
# The real robot's complete takes are copied into scenarios-real (pull.py), with the key and address that
# just lerobot-robot-key sets up; until then the watcher says once that it has no key.
robot=(--pull-robot "$HOME/isaacsim/scenarios-real" --robot-hosts-file "$out/robot-hosts" --robot-key "$HOME/.ssh/id_ed25519_jetrover")

mkdir -p "$out"
running() { [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; }

keeper_on() {
    local self="$repo/robot/lerobot/watch.sh"
    { crontab -l 2>/dev/null | grep -vF "$tag" || true
      echo "@reboot bash $self keep >> $out/keeper.log 2>&1 $tag"
      echo "*/5 * * * * bash $self keep >> $out/keeper.log 2>&1 $tag"
    } | crontab -
}

keeper_off() {
    if crontab -l 2>/dev/null | grep -qF "$tag"; then
        { crontab -l 2>/dev/null | grep -vF "$tag" || true; } | crontab -
        echo "[lerobot] keeper removed: the watcher stays stopped until just lerobot-watch-start"
    fi
}

stop() {
    running || { echo "[lerobot] the watcher is not running"; return 0; }
    local pid; pid=$(cat "$pidfile")
    kill "$pid"
    for _ in $(seq 600); do kill -0 "$pid" 2>/dev/null || { echo "[lerobot] stopped (pid $pid)"; rm -f "$pidfile"; return 0; }; sleep 1; done
    kill -9 "$pid" 2>/dev/null || true
    rm -f "$pidfile"
    echo "[lerobot] killed (pid $pid) after 10 min: the dataset it was writing is rebuilt on the next start"
}

launch() {
    [ -x "$venv/bin/python" ] || { echo "error: no converter venv at $venv: just lerobot-setup" >&2; return 1; }
    [ -d "$repo" ] || git clone -q https://github.com/andreitimoshukcollege-del/teleoperation.git "$repo"
    if [ -n "$commit" ]; then (cd "$repo" && git fetch -q origin && git checkout -q "$commit"); fi
    cd "$repo/robot/lerobot"
    # 9>&- : the watcher must not inherit the lock, or every later start would wait on it for good.
    nohup setsid "$venv/bin/python" -m teleop_lerobot watch "${roots[@]}" "${robot[@]}" --out "$out" >> "$log" 2>&1 < /dev/null 9>&- &
    echo $! > "$pidfile"
    echo "[lerobot] $(date -u +%FT%TZ) watcher started at $(cd "$repo" && git log --oneline -1) (pid $!), log $log"
}

# One start at a time: the keeper, an Isaac host and a recipe may all try at once.
exec 9>"$lock"
case "$cmd" in
    start)
        [ -n "$commit" ] || { echo "error: start needs a commit" >&2; exit 1; }
        flock 9; stop; launch; keeper_on
        echo "[lerobot] keeper installed: cron starts the watcher again within 5 min of a reboot or a crash" ;;
    ensure)
        flock 9
        if running; then echo "[lerobot] watcher running (pid $(cat "$pidfile"))"; else launch; fi ;;
    keep)
        flock -n 9 || exit 0
        running || launch ;;
    stop) flock 9; keeper_off; stop ;;
    status)
        if running; then echo "[lerobot] watcher running (pid $(cat "$pidfile")), at $(cd "$repo" && git log --oneline -1)"
        else echo "[lerobot] watcher NOT running"; fi
        if crontab -l 2>/dev/null | grep -qF "$tag"; then echo "[lerobot] keeper on: cron restarts it within 5 min (at boot too)"
        else echo "[lerobot] keeper off: nothing restarts it after a reboot (just lerobot-watch-start)"; fi
        if [ -f "$log" ]; then tail -8 "$log"; fi
        for ledger in "$out"/*/*/meta/teleop/takes.jsonl; do
            [ -f "$ledger" ] || continue
            ds=${ledger%/meta/teleop/takes.jsonl}
            echo "  ${ds#$out/}: $(wc -l < "$ledger") episode(s), $(du -sh "$ds" | cut -f1)"
        done
        if [ -f "$out/skipped.jsonl" ]; then echo "  skipped takes: $(wc -l < "$out/skipped.jsonl") (see $out/skipped.jsonl)"; fi
        ;;
    *) echo "usage: watch.sh start <commit> | ensure [commit] | keep | stop | status" >&2; exit 2 ;;
esac
