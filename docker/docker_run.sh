#!/bin/bash
set -e
# Run script for self-contained Falcon Autonomy container
# Ensures X11 UI applications (Foxglove, Gazebo, etc.) render properly on host.

# Allow local X11 connections to display GUI from within Docker
xhost +local:root >/dev/null 2>&1 || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Check if nvidia-smi exists to determine if we should enable GPUs
if command -v nvidia-smi &> /dev/null; then
    echo "[docker_run] NVIDIA GPU detected. Enabling GPU support."
    GPU_ARGS="--gpus all"
else
    echo "[docker_run] No NVIDIA GPU detected. Running in standard CPU mode."
    GPU_ARGS=""
fi

echo "[docker_run] Starting Falcon Autonomy Container..."
docker run -it \
    --net host \
    --ipc host \
    --privileged \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ${GPU_ARGS} \
    --volume="${WS_ROOT}/falcon_ws/src:/workspace/falcon_ws/src:rw" \
    --workdir="/workspace" \
    --name falcon_container \
    --entrypoint /ros_entrypoint.sh \
    falcon_autonomy:latest "${@:-bash}"
