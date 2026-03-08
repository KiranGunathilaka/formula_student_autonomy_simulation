#!/bin/bash
#   . scripts/setup_falcon.sh           # Incremental build + setup (picks up new packages)
#   . scripts/setup_falcon.sh --clean  # Full clean rebuild + setup
#   . scripts/setup_falcon.sh --setup  # Setup only (no build). Fails if install missing.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${WS_ROOT}/falcon_ws"

# Required by eufs_tracks for GAZEBO_PLUGIN_PATH
export EUFS_MASTER="${WS_DIR}"

_mode="${1:-}"
if [ -n "${_mode}" ] && [ "${_mode}" != "--clean" ] && [ "${_mode}" != "--setup" ]; then
  echo "Usage: source scripts/setup_falcon.sh [--clean|--setup]"
  return 2 2>/dev/null || exit 2
fi

_do_build=true
_do_clean=false
if [ "${_mode}" = "--setup" ]; then
  _do_build=false
elif [ "${_mode}" = "--clean" ]; then
  _do_clean=true
fi

# Setup: export EUFS_MASTER and source install
_do_setup() {
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    return 0
  else
    echo "[setup_falcon] No install found. Run: source scripts/setup_falcon.sh (or --clean)"
    return 1
  fi
}

if [ "${_do_build}" = false ]; then
  _do_setup
  return $?
fi

# Build path
cd "${WS_DIR}" || return 1

if [ "${FALCON_USE_VENV:-0}" = "1" ] && [ -f "${HOME}/venvs/ros2/bin/activate" ]; then
  echo "[setup_falcon] Activating venv at ~/venvs/ros2 (FALCON_USE_VENV=1)"
  source "${HOME}/venvs/ros2/bin/activate"
fi

export PYTHONNOUSERSITE=1

if [ "${_do_clean}" = true ]; then
  echo "[setup_falcon] Cleaning build, install, log"
  rm -rf build install log
fi

echo "[setup_falcon] Sourcing /opt/ros/humble/setup.sh"
source /opt/ros/humble/setup.sh

echo "[setup_falcon] Running colcon build --symlink-install --merge-install"
colcon build --symlink-install --merge-install || colcon build --merge-install

_do_setup
echo "[setup_falcon] Done"
