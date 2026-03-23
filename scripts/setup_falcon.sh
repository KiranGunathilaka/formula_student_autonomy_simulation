#!/bin/bash

#   source scripts/setup_falcon.sh           # venv + incremental build + ROS env (recommended)
#   source scripts/setup_falcon.sh --clean   # full clean rebuild + ROS env
#   source scripts/setup_falcon.sh --setup   # ROS env only (no build; needs install/)
#
# Uses a project virtualenv at $FALCON_VENV (default: repo_root/.falcon). If it
# does not exist, it is created and scripts/requirements-falcon-env.txt is installed.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${WS_ROOT}/falcon_ws"
FALCON_ENV_REQUIREMENTS="${SCRIPT_DIR}/requirements-falcon-env.txt"

# Default: project venv at repo root (override with FALCON_VENV=/path/to/venv)
FALCON_VENV="${FALCON_VENV:-${WS_ROOT}/.falcon}"

# Required by eufs_tracks for GAZEBO_PLUGIN_PATH
export EUFS_MASTER="${WS_DIR}"

# True if file was sourced into the current shell (keeps venv active); false if ./script
_falcon_is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "${0}" ]]
}

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

# --- Python venv: create if missing, activate; toolchain installed when building or on first create ----------
_falcon_ensure_venv() {
  if [ ! -f "${FALCON_ENV_REQUIREMENTS}" ]; then
    echo "[setup_falcon] Missing ${FALCON_ENV_REQUIREMENTS}"
    return 1
  fi

  _venv_new=false
  if [ ! -f "${FALCON_VENV}/bin/activate" ]; then
    echo "[setup_falcon] Creating venv at ${FALCON_VENV} (with --system-site-packages for ROS/apt Python libs)"
    if ! python3 -m venv --system-site-packages "${FALCON_VENV}"; then
      echo "[setup_falcon] python3 -m venv failed. On Ubuntu install: sudo apt install python3-venv"
      return 1
    fi
    _venv_new=true
  elif [ -f "${FALCON_VENV}/pyvenv.cfg" ] && grep -q '^include-system-site-packages = false$' "${FALCON_VENV}/pyvenv.cfg" 2>/dev/null; then
    echo "[setup_falcon] Note: this venv has no system site-packages. If rosidl/CMake errors mention missing"
    echo "  modules (lark, etc.), remove it and recreate: rm -rf ${FALCON_VENV} && source scripts/setup_falcon.sh"
  fi

  # shellcheck source=/dev/null
  source "${FALCON_VENV}/bin/activate"
  hash -r 2>/dev/null || true

  # CMake / rosidl must use the same Python as colcon or ament_cmake_python invokes
  # /usr/bin/python3 and misses venv deps (e.g. tomli for setuptools).
  export PYTHON_EXECUTABLE="${FALCON_VENV}/bin/python3"
  export Python3_EXECUTABLE="${FALCON_VENV}/bin/python3"

  if [ "${_venv_new}" = true ] || [ "${_do_build}" = true ]; then
    echo "[setup_falcon] Installing/updating build toolchain (pip, setuptools, colcon, …)"
    python -m pip install -q --upgrade pip
    python -m pip install -q -r "${FALCON_ENV_REQUIREMENTS}"

    _st="$(python -c "import setuptools; print(setuptools.__version__.split('.')[0])" 2>/dev/null || echo 0)"
    if [ "${_st}" -lt 64 ] 2>/dev/null; then
      echo "[setup_falcon] Warning: setuptools still < 64 after pip install; colcon --symlink-install may fail."
    fi
  fi
  return 0
}

# colcon must be the venv copy so Python packages are built with venv Python (see colcon_python_setup_py).
_falcon_colcon() {
  if [ -x "${FALCON_VENV}/bin/colcon" ]; then
    "${FALCON_VENV}/bin/colcon" "$@"
  else
    echo "[setup_falcon] ${FALCON_VENV}/bin/colcon missing after pip install; using PATH colcon (may use system Python)."
    command colcon "$@"
  fi
}

# Setup: export EUFS_MASTER and source install
_do_setup() {
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # EUFS expects $EUFS_MASTER/install/eufs_plugins for GAZEBO_PLUGIN_PATH, but
    # with colcon --merge-install plugins are in install/lib. Symlink fixes gzserver.
    if [ ! -e "${WS_DIR}/install/eufs_plugins" ] && [ -d "${WS_DIR}/install/lib" ]; then
      ln -sf lib "${WS_DIR}/install/eufs_plugins"
    fi
    source "${WS_DIR}/install/setup.bash"
    return 0
  else
    echo "[setup_falcon] No install found. Run: source scripts/setup_falcon.sh (or --clean)"
    return 1
  fi
}

_falcon_ensure_venv || { _e=$?; return "$_e" 2>/dev/null || exit "$_e"; }

if [ "${_do_build}" = false ]; then
  if [ -f /opt/ros/humble/setup.sh ]; then
    echo "[setup_falcon] Sourcing /opt/ros/humble/setup.sh"
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.sh
  fi
  _do_setup
  _status=$?
  if ! _falcon_is_sourced; then
    echo "[setup_falcon] Tip: use 'source scripts/setup_falcon.sh' so .falcon stays active in this shell." >&2
  fi
  return "$_status" 2>/dev/null || exit "$_status"
fi

# Build path
cd "${WS_DIR}" || { return 1 2>/dev/null || exit 1; }

export PYTHONNOUSERSITE=1

if [ "${_do_clean}" = true ]; then
  echo "[setup_falcon] Cleaning build, install, log"
  rm -rf build install log
fi

# Old installs may leave plain *files* here; --symlink-install creates symlinks and fails
# with Errno 17 (File exists) if a regular file already uses the package name. Symlinks (-type l) are kept.
_falcon_idx="${WS_DIR}/install/share/ament_index/resource_index/packages"
if [ -d "${_falcon_idx}" ]; then
  find "${_falcon_idx}" -maxdepth 1 -type f -delete 2>/dev/null || true
fi

echo "[setup_falcon] Sourcing /opt/ros/humble/setup.sh"
# shellcheck source=/dev/null
source /opt/ros/humble/setup.sh

echo "[setup_falcon] Running colcon build --symlink-install --merge-install"
if ! _falcon_colcon build --symlink-install --merge-install \
  --cmake-args \
  "-DPYTHON_EXECUTABLE=${FALCON_VENV}/bin/python3" \
  "-DPython3_EXECUTABLE=${FALCON_VENV}/bin/python3"; then
  echo "[setup_falcon] Build failed. Try: source scripts/setup_falcon.sh --clean"
  if ! _falcon_is_sourced; then
    echo "[setup_falcon] Tip: use 'source scripts/setup_falcon.sh' so .falcon stays active in this shell." >&2
  fi
  return 1 2>/dev/null || exit 1
fi

_do_setup || { _e=$?; return "$_e" 2>/dev/null || exit "$_e"; }
echo "[setup_falcon] Done (venv: ${FALCON_VENV})"
if ! _falcon_is_sourced; then
  echo "[setup_falcon] Tip: use 'source scripts/setup_falcon.sh' next time so this shell keeps the venv active." >&2
fi
