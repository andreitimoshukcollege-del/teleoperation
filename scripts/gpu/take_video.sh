#!/usr/bin/env bash
# One twin take as a video: its cameras side by side, from the recorded frames (isaac.jlog VF records and the
# isaac/<camera>.rgb.frames sidecars). Runs on the GPU machine; see docs/agent-operations.md.
#
#   ssh andrei@10.188.58.133 'bash -s <take_dir> <out.mp4> [cameras]' < scripts/gpu/take_video.sh
#   ssh andrei@10.188.58.133 'cat <out.mp4>' > <local file>        # then copy it to the user's Downloads
#
# [cameras] is a comma list, default third-left,third-right,wrist. Light work: a few seconds of ffmpeg.
set -euo pipefail
d="$1"; out="$2"; cams="${3:-third-left,third-right,wrist}"
w=$(mktemp -d)
python3 - "$d" "$w" "$cams" <<'PY'
import os, sys
d, w, cams = sys.argv[1], sys.argv[2], sys.argv[3].split(",")
files = {c: open(os.path.join(d, "isaac", c + ".rgb.frames"), "rb") for c in cams}
n = 0
for line in open(os.path.join(d, "isaac.jlog"), "rb"):
    if not line.startswith(b"VF|"):
        continue
    got = {}
    for it in line.decode().rstrip("\n").split("|")[-1].split(";"):
        name, off, ln = it.rsplit(":", 2)
        cam, kind = name.rsplit(".", 1)
        if kind == "rgb" and cam in files:
            f = files[cam]; f.seek(int(off)); got[cam] = f.read(int(ln))
    if len(got) != len(cams):
        continue
    for c in cams:
        os.makedirs(os.path.join(w, c), exist_ok=True)
        open(os.path.join(w, c, f"{n:05d}.jpg"), "wb").write(got[c])
    n += 1
print(n, "frames")
PY
IFS=, read -r -a list <<<"$cams"
inputs=(); labels=""
for i in "${!list[@]}"; do inputs+=(-framerate 30 -i "$w/${list[$i]}/%05d.jpg"); labels+="[$i]"; done
if [ "${#list[@]}" -gt 1 ]; then filter="${labels}hstack=inputs=${#list[@]},scale=1440:-2"; else filter="scale=960:-2"; fi
ffmpeg -nostdin -loglevel error -y "${inputs[@]}" -filter_complex "$filter" -c:v libx264 -pix_fmt yuv420p -crf 23 "$out"
find "$w" -type f -delete; find "$w" -depth -type d -delete
ls -la "$out"
