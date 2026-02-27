#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${WS_ROOT}/falcon_ws"

echo "[rebuild_ws] Workspace root: ${WS_ROOT}"

if [ "${FALCON_USE_VENV:-0}" = "1" ] && [ -f "${HOME}/venvs/ros2/bin/activate" ]; then
  echo "[rebuild_ws] Activating venv at ~/venvs/ros2 (FALCON_USE_VENV=1)"
  source "${HOME}/venvs/ros2/bin/activate"
else
  echo "[rebuild_ws] Building with system Python (set FALCON_USE_VENV=1 to use ~/venvs/ros2)"
fi

export PYTHONNOUSERSITE=1
echo "[rebuild_ws] Set PYTHONNOUSERSITE=1"

cd "${WS_DIR}"

echo "[rebuild_ws] Cleaning build, install, log directories"
rm -rf build install log

echo "[rebuild_ws] Sourcing /opt/ros/humble/setup.sh"
source /opt/ros/humble/setup.sh

echo "[rebuild_ws] Running colcon build --symlink-install --merge-install"
colcon build --symlink-install --merge-install || colcon build --merge-install

echo "[rebuild_ws] Sourcing install/setup.sh"
source install/setup.sh

echo "[rebuild_ws] Build complete"
