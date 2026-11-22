#!/usr/bin/env bash
# Run the repo's gates on the GPU machine for the clone you are in. A WSL clone has no local dotnet or numpy, and a gate
# there needs neither; see docs/agent-operations.md.
#
#   scripts/gpu/gate.sh <gate>...          (from anywhere inside a clone)
#
# The clone's current branch is pushed to a scratch bare repo on the GPU machine (~/scratch/agent-git/teleoperation.git) and
# checked out in ~/scratch/agent-<name>/teleoperation, never the twins' checkout. <name> is the clone folder's name; GATE_NAME
# overrides it. GPU_HOST overrides the machine, as it does for the justfile. Only committed work is pushed: commit first.
#
# Gates:
#   core          dotnet test, verify and audit (just core-check)        heavy: about 10 min of the machine's CPU
#   test          dotnet test (all of core/Teleop.sln)                    heavy
#   test=FILTER   dotnet test --filter FILTER (e.g. test=FullyQualifiedName~SafetyScript)
#   verify        dotnet run --project Teleop.Eval -- verify
#   audit         dotnet run --project Teleop.Eval -- audit
#   bridge        unity/BridgeCheck build (just bridge-check)
#   isaac         robot/isaac pytest (just isaac-bridge-test)             light
#   lerobot       robot/lerobot pytest                                    light
#   gpu           robot/gpu pytest (the dashboard)                        light
# Ask before a heavy gate when the user has said the machine is busy. Exit status is non-zero if any gate failed.
set -euo pipefail
[ $# -ge 1 ] || { sed -n 2,24p "$0"; exit 2; }
repo=$(git rev-parse --show-toplevel)
name=${GATE_NAME:-$(basename "$repo")}
case "$name" in *[!A-Za-z0-9_-]*|"") echo "error: the clone's name '$name' is not a plain word; set GATE_NAME" >&2; exit 2 ;; esac
host=${GPU_HOST:-andrei@10.188.58.133}
branch=$(git -C "$repo" branch --show-current)
[ -n "$branch" ] || { echo "error: $repo is not on a branch" >&2; exit 2; }
ssh -o ConnectTimeout=10 "$host" 'mkdir -p ~/scratch/agent-git && { [ -d ~/scratch/agent-git/teleoperation.git ] || git init -q --bare ~/scratch/agent-git/teleoperation.git; }'
git -C "$repo" push -q --no-verify "ssh://$host/home/${host%@*}/scratch/agent-git/teleoperation.git" "$branch:refs/heads/$branch"
ssh -o ConnectTimeout=10 "$host" "NAME='$name' BRANCH='$branch' GATES='$*' bash -s" <<'REMOTE'
set -uo pipefail
export GIT_LFS_SKIP_SMUDGE=1 DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1
d=$HOME/scratch/agent-$NAME/teleoperation
if [ ! -d "$d/.git" ]; then
    mkdir -p "$HOME/scratch/agent-$NAME"
    git clone -q -c filter.lfs.process="git-lfs filter-process --skip" -c filter.lfs.smudge="git-lfs smudge --skip -- %f" \
        "$HOME/scratch/agent-git/teleoperation.git" "$d" || exit 3
fi
cd "$d" && git fetch -q origin && git checkout -q -f -B "$BRANCH" "origin/$BRANCH" || exit 3
echo "GPU machine at $(git log --oneline -1)"
py=$HOME/isaacsim/lerobot-venv/bin/python
log=/tmp/agent-$NAME-gate.log
rc=0
run() {
    local what=$1; shift
    if "$@" >"$log" 2>&1; then
        echo "== $what: PASS"; grep -E "Passed!|Failed!|passed|failed|[0-9]+ Warning" "$log" | tail -n 12
    else
        echo "== $what: FAIL"; tail -n 60 "$log"; rc=1
    fi
}
for g in $GATES; do
    case "$g" in
        core)    run "dotnet test" bash -c 'cd core && dotnet test'
                 run verify bash -c 'cd core && dotnet run --project Teleop.Eval -- verify'
                 run audit bash -c 'cd core && dotnet run --project Teleop.Eval -- audit' ;;
        test)    run "dotnet test" bash -c 'cd core && dotnet test' ;;
        test=*)  f=${g#test=}; run "dotnet test --filter $f" bash -c "cd core && dotnet test --filter '$f'" ;;
        verify)  run verify bash -c 'cd core && dotnet run --project Teleop.Eval -- verify' ;;
        audit)   run audit bash -c 'cd core && dotnet run --project Teleop.Eval -- audit' ;;
        bridge)  run bridge-check bash -c 'cd unity/BridgeCheck && dotnet build --nologo' ;;
        isaac)   run "isaac pytest" bash -c "cd robot/isaac && '$py' -m pytest -q -p no:cacheprovider tests" ;;
        lerobot) run "lerobot pytest" bash -c "cd robot/lerobot && '$py' -m pytest -q -p no:cacheprovider tests" ;;
        gpu)     run "gpu pytest" bash -c "cd robot/gpu && '$py' -m pytest -q -p no:cacheprovider tests" ;;
        *)       echo "== unknown gate '$g'"; rc=2 ;;
    esac
done
exit $rc
REMOTE
