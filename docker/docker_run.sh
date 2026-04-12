#!/bin/bash
set -e
# Run script for self-contained Falcon Autonomy container
# Ensures X11 UI applications (Foxglove, Gazebo, etc.) render properly on host.

# Allow local X11 connections to display GUI from within Docker
xhost +local:root >/dev/null 2>&1 || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Enable --gpus only when the driver actually responds (nvidia-smi may be
# installed while the kernel module is missing, Secure Boot blocks loading, etc.)
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "[docker_run] NVIDIA driver responding. Enabling GPU support."
    GPU_ARGS="--gpus all"
elif command -v nvidia-smi &> /dev/null; then
    echo "[docker_run] nvidia-smi is present but the driver is not responding; running without --gpus."
    echo "[docker_run] Fix the host driver (reinstall, reboot, disable Secure Boot if it blocks modules), then install nvidia-container-toolkit for Docker GPU."
    GPU_ARGS=""
else
    echo "[docker_run] No NVIDIA tools detected. Running in standard CPU mode."
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
