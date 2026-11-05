#!/bin/bash
# Record the desktop (Gazebo, RViz, the teleop window ...) to an MP4 until q or Ctrl-C.
#
#   scripts/record_screen.sh                  # -> runs/videos/falcon_<time>.mp4
#   scripts/record_screen.sh my_run.mp4
#   FPS=60 scripts/record_screen.sh
#
# Run it on the host (not in the container), in its own terminal.  Needs ffmpeg
# (sudo apt install ffmpeg).  Encodes on the NVIDIA GPU when it can.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-${SCRIPT_DIR}/../runs/videos/falcon_$(date +%Y%m%d_%H%M%S).mp4}"
DISP="${DISPLAY:-:0}"
FPS="${FPS:-30}"
mkdir -p "$(dirname "${OUT}")"

SIZE=$(xdpyinfo -display "${DISP}" 2>/dev/null | awk '/dimensions:/ {print $2}')
if [ -z "${SIZE}" ]; then
    echo "Cannot read the size of display ${DISP}; is DISPLAY set?" >&2
    exit 1
fi

if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_nvenc && nvidia-smi &>/dev/null; then
    ENC=(-c:v h264_nvenc -preset p4 -cq 23)
else
    ENC=(-c:v libx264 -preset veryfast -crf 23)
fi

echo "Recording ${DISP} (${SIZE}, ${FPS} fps) to ${OUT}; press q or Ctrl-C to stop."
ffmpeg -hide_banner -loglevel warning -f x11grab -framerate "${FPS}" -video_size "${SIZE}" \
    -i "${DISP}" "${ENC[@]}" -pix_fmt yuv420p "${OUT}"
echo "Saved ${OUT}"
