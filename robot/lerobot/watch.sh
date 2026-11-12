#!/usr/bin/env bash
# The LeRobot converter on the GPU machine (docs/adr/0036): `watch.sh start|ensure <commit>`, `watch.sh stop|status`.
# Run by the justfile's lerobot-* recipes and by isaac-host / isaac-jetrover-host (ensure), never by hand on a PC.
#
# - start: (re)start the watcher at <commit>, from its own checkout ~/teleoperation-lerobot, so an Isaac host started
#   at another commit never changes the code under a running watcher;
# - ensure: start it at <commit> only if it is not running;
# - stop: SIGTERM, which it honours between takes (up to 10 min for the take it is writing), then SIGKILL;
# - status: running or not, the log's tail, and every dataset with its episodes.
set -euo pipefail
cmd=${1:-status}
commit=${2:-}
repo=$HOME/teleoperation-lerobot
venv=$HOME/isaacsim/lerobot-venv
out=$HOME/isaacsim/lerobot
pidfile=$out/watch.pid
log=$out/watch.log
# The scenario roots: the FR3 Duo's and the JetRover's Isaac takes, and the real robot's (copied in by ADR 0036
# decision 7). A root that does not exist yet is skipped each pass.
roots=(--root "$HOME/isaacsim/scenarios" --root "$HOME/isaacsim/scenarios-jetrover" --root "$HOME/isaacsim/scenarios-real")

running() { [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; }

stop() {
    running || { echo "[lerobot] the watcher is not running"; return 0; }
    local pid; pid=$(cat "$pidfile")
    kill "$pid"
    for _ in $(seq 600); do kill -0 "$pid" 2>/dev/null || { echo "[lerobot] stopped (pid $pid)"; rm -f "$pidfile"; return 0; }; sleep 1; done
    kill -9 "$pid" 2>/dev/null || true
    rm -f "$pidfile"
    echo "[lerobot] killed (pid $pid) after 10 min: the dataset it was writing is rebuilt on the next start"
}

start() {
    [ -n "$commit" ] || { echo "error: start needs a commit" >&2; exit 1; }
    [ -x "$venv/bin/python" ] || { echo "error: no converter venv at $venv: just lerobot-setup" >&2; exit 1; }
    [ -d "$repo" ] || git clone -q https://github.com/andreitimoshukcollege-del/teleoperation.git "$repo"
    (cd "$repo" && git fetch -q origin && git checkout -q "$commit")
    mkdir -p "$out"
    cd "$repo/robot/lerobot"
    nohup setsid "$venv/bin/python" -m teleop_lerobot watch "${roots[@]}" --out "$out" >> "$log" 2>&1 < /dev/null &
    echo $! > "$pidfile"
    echo "[lerobot] watcher started at $(cd "$repo" && git log --oneline -1) (pid $!), log $log"
}

case "$cmd" in
    start) stop; start ;;
    ensure) if running; then echo "[lerobot] watcher running (pid $(cat "$pidfile"))"; else start; fi ;;
    stop) stop ;;
    status)
        if running; then echo "[lerobot] watcher running (pid $(cat "$pidfile")), at $(cd "$repo" && git log --oneline -1)"
        else echo "[lerobot] watcher NOT running"; fi
        [ -f "$log" ] && tail -8 "$log"
        for ledger in "$out"/*/*/meta/teleop/takes.jsonl; do
            [ -f "$ledger" ] || continue
            ds=${ledger%/meta/teleop/takes.jsonl}
            echo "  ${ds#$out/}: $(wc -l < "$ledger") episode(s), $(du -sh "$ds" | cut -f1)"
        done
        [ -f "$out/skipped.jsonl" ] && echo "  skipped takes: $(wc -l < "$out/skipped.jsonl") (see $out/skipped.jsonl)"
        ;;
    *) echo "usage: watch.sh start|ensure <commit> | stop | status" >&2; exit 2 ;;
esac
