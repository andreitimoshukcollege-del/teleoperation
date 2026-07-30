#!/usr/bin/env bash
# Fast invariant guard for core/Teleop.Core, run after every edit.
#
# Deliberately ignores hook stdin and just greps the whole Core tree: it is small, grep is
# instant, and this way the hook cannot break when the hook input schema changes.
#
# Exit 2 surfaces the message to Claude as a blocking error. Exit 0 = clean.

set -uo pipefail
CORE="core/Teleop.Core"
[ -d "$CORE" ] || exit 0

FAIL=0
report() { echo "CORE INVARIANT VIOLATION — $1" >&2; FAIL=1; }

hit() { grep -rn --include='*.cs' -E "$1" "$CORE" 2>/dev/null | grep -v '^\s*//'; }

# 1. Unity leakage
out=$(hit 'using[[:space:]]+UnityEngine|UnityEngine\.')            && [ -n "$out" ] && report "UnityEngine referenced in Core:"$'\n'"$out"
# 2. Wall clock — the highest-consequence violation; produces plausible but wrong latency data
out=$(hit 'DateTime\.(Now|UtcNow)|Environment\.TickCount|new[[:space:]]+Stopwatch|Time\.time') && [ -n "$out" ] && report "wall-clock read in Core (use ITimeAuthority):"$'\n'"$out"
# 3. I/O and concurrency
out=$(hit 'using[[:space:]]+System\.IO|using[[:space:]]+System\.Net|new[[:space:]]+Thread|Task\.Run') && [ -n "$out" ] && report "I/O or threading in Core (belongs in the host):"$'\n'"$out"
# 4. Unseeded randomness
out=$(hit 'new[[:space:]]+Random\(|Guid\.NewGuid')                 && [ -n "$out" ] && report "unseeded randomness in Core:"$'\n'"$out"
# 5. Reflection — silently stripped by IL2CPP, fails on device only
out=$(hit 'Activator\.CreateInstance|Reflection\.Emit|Expression\.Compile') && [ -n "$out" ] && report "reflection in Core (breaks IL2CPP AOT):"$'\n'"$out"
# 6. Build output inside the UPM package — Unity would import it as a duplicate plugin
{ [ -d "$CORE/bin" ] || [ -d "$CORE/obj" ]; } && report "bin/ or obj/ inside $CORE — Unity will import a stray DLL and duplicate every type. Check Directory.Build.props."

exit $((FAIL * 2))
