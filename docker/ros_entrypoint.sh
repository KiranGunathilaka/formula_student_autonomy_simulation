#!/bin/bash
set -e

# Setup ROS 2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Set up the VENV environment automatically if installed
export FALCON_VENV=/opt/falcon_venv
if [ -f "${FALCON_VENV}/bin/activate" ]; then
    source "${FALCON_VENV}/bin/activate"
fi

# Sourcing workspace if it has been built
if [ -f /workspace/falcon_ws/install/setup.bash ]; then
    source /workspace/falcon_ws/install/setup.bash
fi

# If using eufs_tracks, ensure paths are correct
export EUFS_MASTER="/workspace/falcon_ws"

exec "$@"
