#!/usr/bin/env bash
# Keep the GPU machine's recorded data on the Explorer cluster's lab space (docs/explorer-cluster.md "Data from the GPU
# machine"). Runs ON the GPU machine; the justfile's cluster-* recipes stream it there (ssh ... 'bash -s' < this file).
#
#   cluster_archive.sh archive [path under ~/data ...]   copy to the cluster (default: every folder of ~/data), then
#                                                       check sizes and times match; exit 1 if anything differs
#   cluster_archive.sh prune <days> [apply]             list what is older than <days> days AND verified identical on the
#                                                       cluster (checksums), take by take and session by session; with
#                                                       `apply`, delete exactly those from the GPU machine
#   cluster_archive.sh status                           sizes here, and what the cluster holds
#
# Only this account's data: ~/data here and its own folder in the lab space there. Nothing in another account's home
# (the labmate's) and nothing of other lab members in /projects/world-models is read, written or deleted.
#
# The cluster copy is never deleted or overwritten with older files by this script (rsync --update, no --delete): it is
# the archive. Datasets (data/lerobot) are archived but never pruned here: a dataset is deleted by hand, by its path, once
# the user says so. .active, *.part and *.tmp are skipped; a take still being written is copied as far as it got and
# completed by the next run (prune only ever takes folders untouched for <days> days).
set -euo pipefail

XFER=${CLUSTER_XFER:-explorer-xfer}                       # ~/.ssh/config there: xfer.discovery.neu.edu, the RC key
DEST=${CLUSTER_DATA:-}                                     # default: /projects/world-models/<cluster user>/teleop-data
SRC=${DATA_ROOT:-$HOME/data}
LOGDIR=${CLUSTER_LOGS:-$HOME/logs/cluster}
mkdir -p "$LOGDIR"
if [ -z "$DEST" ]; then
    cu=$(ssh -n -o BatchMode=yes -o ConnectTimeout=20 "$XFER" 'echo $USER') || { echo "error: cannot reach $XFER (just gpu-cluster-key installs the key)" >&2; exit 2; }
    DEST=/projects/world-models/$cu/teleop-data
fi
case "$DEST" in /projects/*) ;; *) echo "error: CLUSTER_DATA must be under /projects (got $DEST)" >&2; exit 2 ;; esac

# Group-readable and -writable for world-models (the setgid lab space gives the group), nothing for others.
RS=(rsync -rlt --update --chmod=Dug=rwx,Do=,Fug=rw,Fo= --partial --exclude=.active --exclude='*.part' --exclude='*.tmp'
    -e "ssh -o BatchMode=yes -o ConnectTimeout=20")

log() { echo "[cluster] $*"; }

remote() { ssh -n -o BatchMode=yes -o ConnectTimeout=20 "$XFER" "$@"; }

archive() {
    local paths=("$@")
    if [ "${#paths[@]}" = 0 ]; then mapfile -t paths < <(cd "$SRC" && find . -mindepth 1 -maxdepth 1 -type d -printf '%P\n' | sort); fi
    remote "mkdir -p '$DEST'"
    local rc=0 p
    for p in "${paths[@]}"; do
        case "$p" in /*|*..*) echo "error: '$p' must be a path under $SRC" >&2; return 2 ;; esac
        [ -e "$SRC/$p" ] || { echo "error: $SRC/$p does not exist" >&2; rc=1; continue; }
        log "copying $p ($(du -sh "$SRC/$p" | cut -f1))"
        remote "mkdir -p '$DEST/$(dirname "$p")'"
        if [ -d "$SRC/$p" ]; then
            "${RS[@]}" "$SRC/$p/" "$XFER:$DEST/$p/" || rc=1
        else
            "${RS[@]}" "$SRC/$p" "$XFER:$DEST/$p" || rc=1
        fi
        # Quick check: what a second pass would still send (by size and time). Nothing = the cluster has it all.
        local left
        left=$("${RS[@]}" --dry-run --itemize-changes "$SRC/$p$([ -d "$SRC/$p" ] && echo /)" "$XFER:$DEST/$p$([ -d "$SRC/$p" ] && echo /)" | grep -c '^<f' || true)
        if [ "$left" = 0 ]; then log "$p: on the cluster at $DEST/$p"; else log "$p: $left file(s) still differ"; rc=1; fi
    done
    return $rc
}

# Take and session folders, the units prune works in.
units() {
    find "$SRC/takes" -mindepth 3 -maxdepth 4 -type d -name '*-take*' 2>/dev/null
    find "$SRC/sessions" -mindepth 2 -maxdepth 2 -type d ! -name '.*' 2>/dev/null
}

prune() {
    local days=$1 apply=${2:-}
    case "$days" in ''|*[!0-9]*) echo "error: days must be a whole number" >&2; return 2 ;; esac
    [ "$days" -ge 1 ] || { echo "error: days must be at least 1" >&2; return 2; }
    local ok=() missing=0 bytes=0 u rel diff
    while read -r u; do
        [ -n "$u" ] || continue
        # Older than <days> by its newest file, so a take still growing is never a candidate.
        [ -z "$(find "$u" -newermt "-$days days" -print -quit)" ] || continue
        # Only this account's own folders: anything holding another account's file (the labmate's) is never deleted.
        if [ -n "$(find "$u" ! -user "$(id -un)" -print -quit)" ]; then log "kept (holds another account's files): ${u#"$SRC"/}"; continue; fi
        rel=${u#"$SRC"/}
        diff=$(rsync -rlt --checksum --dry-run --itemize-changes -e "ssh -o BatchMode=yes -o ConnectTimeout=20" \
               --exclude=.active "$u/" "$XFER:$DEST/$rel/" 2>/dev/null | grep -c '^<f' || true)
        if remote "test -d '$DEST/$rel'" && [ "$diff" = 0 ]; then
            ok+=("$u"); bytes=$((bytes + $(du -sb "$u" | cut -f1)))
        else
            missing=$((missing + 1)); log "not on the cluster yet (kept): $rel"
        fi
    done < <(units | sort)
    log "${#ok[@]} folder(s) older than $days days and identical on the cluster: $(numfmt --to=iec "$bytes")"
    log "$missing older folder(s) not (fully) on the cluster: kept; run archive first"
    local list="$LOGDIR/prune-$(date -u +%Y%m%dT%H%M%SZ).txt"
    printf '%s\n' "${ok[@]}" > "$list"
    log "the list: $list"
    if [ "$apply" != apply ]; then
        printf '  %s\n' "${ok[@]:0:20}"; [ "${#ok[@]}" -le 20 ] || log "... and $(( ${#ok[@]} - 20 )) more (all in the list)"
        log "dry run: nothing deleted (pass apply to delete these)"
        return 0
    fi
    for u in "${ok[@]}"; do
        case "$u" in "$SRC"/takes/*/*|"$SRC"/sessions/*/*) ;; *) echo "refusing to delete $u" >&2; continue ;; esac
        [ -z "$(find "$u" ! -user "$(id -un)" -print -quit)" ] || { echo "refusing to delete $u: another account's files" >&2; continue; }
        rm -r -- "$u"
    done
    log "deleted ${#ok[@]} folder(s) from the GPU machine; df now: $(df -h "$SRC" | awk 'NR==2 {print $4 " free of " $2}')"
}

status() {
    log "GPU machine: $(df -h "$SRC" | awk 'NR==2 {print $4 " free of " $2 " (" $5 " used)"}')"
    (cd "$SRC" && du -sh -- */ 2>/dev/null | sed 's/^/  here    /')
    log "cluster: $DEST"
    remote "df -h /projects/world-models | tail -1 | awk '{print \"  lab space: \" \$4 \" free of \" \$2}'; cd '$DEST' 2>/dev/null && for d in */; do printf '  cluster %s %s files\n' \"\$d\" \"\$(find \"\$d\" -type f | wc -l)\"; done || echo '  (nothing archived yet)'"
}

cmd=${1:-}; shift || true
case "$cmd" in
    archive) archive "$@" ;;
    prune) prune "$@" ;;
    status) status ;;
    *) echo "usage: cluster_archive.sh archive|prune|status ..." >&2; exit 2 ;;
esac
