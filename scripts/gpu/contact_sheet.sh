#!/usr/bin/env bash
# Up to 16 twin takes on one image: one frame of each, 4 by 4. Runs on the GPU machine; see docs/agent-operations.md.
#
#   ssh andrei@10.188.58.133 'bash -s <out.jpg> <camera> <fraction> <take_dir>...' < scripts/gpu/contact_sheet.sh
#   ssh andrei@10.188.58.133 'cat <out.jpg>' > <local file>
#
# <camera> is third-left, third-right or wrist. <fraction> is how far into each take the frame is taken: 0.15 shows the
# settled layout, about 0.45 often catches the arm at work. The takes are drawn in the order given, and it prints the index.
set -euo pipefail
out="$1"; cam="$2"; frac="$3"; shift 3
w=$(mktemp -d)
n=0
for d in "$@"; do
    [ "$n" -lt 16 ] || break
    python3 - "$d" "$w/$(printf %02d "$n").jpg" "$cam" "$frac" <<'PY'
import os, sys
d, out, cam, frac = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
vf = [l for l in open(os.path.join(d, "isaac.jlog"), "rb") if l.startswith(b"VF|")]
line = vf[min(len(vf) - 1, int(len(vf) * frac))].decode().rstrip("\n").split("|")[-1]
for it in line.split(";"):
    name, off, ln = it.rsplit(":", 2)
    if name == cam + ".rgb":
        f = open(os.path.join(d, "isaac", cam + ".rgb.frames"), "rb"); f.seek(int(off)); open(out, "wb").write(f.read(int(ln)))
PY
    echo "$(printf %02d "$n") $(basename "$(dirname "$d")") $(basename "$d")"
    n=$((n + 1))
done
ffmpeg -nostdin -loglevel error -y -framerate 1 -i "$w/%02d.jpg" -vf "scale=480:-2,tile=4x4:padding=4" -frames:v 1 "$out"
find "$w" -type f -delete; find "$w" -depth -type d -delete
ls -la "$out"
