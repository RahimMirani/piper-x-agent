#!/usr/bin/env bash
# Stitch the scene frames the Pi saved during one episode into an MP4.
#
# This shows the arm at every decision point, not continuously: frames exist
# only where the model called observe. It is a review aid, not a substitute for
# an external camera recording the whole episode.
set -euo pipefail
[[ $# -eq 2 ]] || { echo "Usage: scripts/make-episode-video.sh <pi-run-directory> <out.mp4>" >&2; exit 2; }
RUN="$1"; OUT="$2"
command -v ffmpeg >/dev/null || { echo "ffmpeg is required" >&2; exit 1; }
COUNT=$(find "$RUN" -name '*-scene.jpg' | wc -l | tr -d ' ')
[[ "$COUNT" -gt 0 ]] || { echo "No scene frames in $RUN" >&2; exit 1; }
ffmpeg -y -framerate 2 -pattern_type glob -i "$RUN/*-scene.jpg" \
  -c:v libx264 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" "$OUT"
echo "Wrote $OUT from $COUNT observation frames."
