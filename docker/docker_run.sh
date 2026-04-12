#!/bin/bash
set -e
# Run script for self-contained Falcon Autonomy container
# Ensures X11 UI applications (Foxglove, Gazebo, etc.) render properly on host.

# Allow local X11 connections to display GUI from within Docker
xhost +local:root >/dev/null 2>&1 || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[docker_run] Starting Falcon Autonomy Container..."
docker run -it --rm \
    --net host \
    --ipc host \
    --privileged \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --gpus all \
    --volume="${WS_ROOT}:/workspace:rw" \
    --workdir="/workspace" \
    --name falcon_container \
    --entrypoint /ros_entrypoint.sh \
    falcon_autonomy:latest "${@:-bash}"
