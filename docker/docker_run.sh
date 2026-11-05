#!/bin/bash
set -e
# Run script for self-contained Falcon Autonomy container
# Ensures X11 UI applications (Foxglove, Gazebo, etc.) render properly on host.
#
#   ./docker/docker_run.sh            # start the container (or, if it is already
#                                     # running, open another shell in it)
#   ./docker/docker_run.sh <command>  # run one command in it
#   FALCON_GL=mesa ./docker/docker_run.sh   # render on the integrated GPU instead
#   ROS_LOCALHOST_ONLY=0 ./docker/docker_run.sh   # let ROS talk to other machines
#
# Inside, after changing code:  source /workspace/docker/build.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAME=falcon_container

TTY_ARGS="-i"
[ -t 0 ] && TTY_ARGS="-it"

# Second terminal: join the running container.
if [ "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "[docker_run] ${NAME} is running: opening another shell in it."
    exec docker exec ${TTY_ARGS} -e DISPLAY="${DISPLAY}" ${NAME} /ros_entrypoint.sh "${@:-bash}"
fi

# Allow local X11 connections to display GUI from within Docker
xhost +local:root >/dev/null 2>&1 || true

# Enable --gpus only when the driver actually responds (nvidia-smi may be
# installed while the kernel module is missing, Secure Boot blocks loading, etc.)
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "[docker_run] NVIDIA driver responding. Enabling GPU support."
    GPU_ARGS="--gpus all"
    if [ "${FALCON_GL:-nvidia}" = "nvidia" ]; then
        # OpenGL (Gazebo's gpu_ray lidar, cameras, RViz) on the NVIDIA GPU.  The CUDA
        # base image only exposes compute; "all" adds the GL libraries.  On a laptop
        # whose desktop runs on the integrated GPU (PRIME), GL still goes to the
        # integrated GPU unless the two offload variables are set.
        GPU_ARGS="${GPU_ARGS} -e NVIDIA_DRIVER_CAPABILITIES=all -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia"
    fi
elif command -v nvidia-smi &> /dev/null; then
    echo "[docker_run] nvidia-smi is present but the driver is not responding; running without --gpus."
    echo "[docker_run] Fix the host driver (reinstall, reboot, disable Secure Boot if it blocks modules), then install nvidia-container-toolkit for Docker GPU."
    GPU_ARGS=""
else
    echo "[docker_run] No NVIDIA tools detected. Running in standard CPU mode."
    GPU_ARGS=""
fi

# The container shares the host network.  ROS stays on this machine unless
# ROS_LOCALHOST_ONLY=0: two simulators on one LAN and ROS domain receive each other's
# /clock, TF and sensor topics.

# Coverage reports and sweeps (cone_coverage writes to ~/falcon_runs = /root/falcon_runs)
mkdir -p "${WS_ROOT}/runs"

# build/, install/ and log/ live in named volumes (seeded from the image the first
# time), so a rebuild inside one container is still there in the next.  After
# rebuilding the image: docker volume rm falcon_ws_build falcon_ws_install falcon_ws_log
echo "[docker_run] Starting Falcon Autonomy Container..."
docker run ${TTY_ARGS} --rm --init \
    --net host \
    --ipc host \
    --privileged \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="PYTHONDONTWRITEBYTECODE=1" \
    --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    --env="ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ${GPU_ARGS} \
    --volume="${WS_ROOT}/falcon_ws/src:/workspace/falcon_ws/src:rw" \
    --volume="${WS_ROOT}/runs:/root/falcon_runs:rw" \
    --volume="${WS_ROOT}/docker:/workspace/docker:ro" \
    --volume="falcon_ws_build:/workspace/falcon_ws/build" \
    --volume="falcon_ws_install:/workspace/falcon_ws/install" \
    --volume="falcon_ws_log:/workspace/falcon_ws/log" \
    --workdir="/workspace" \
    --name ${NAME} \
    --entrypoint /ros_entrypoint.sh \
    falcon_autonomy:latest "${@:-bash}"
