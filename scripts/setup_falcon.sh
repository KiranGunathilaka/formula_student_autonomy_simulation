#!/bin/bash

#   source scripts/setup_falcon.sh           # venv + incremental build + ROS env (recommended)
#   source scripts/setup_falcon.sh --clean   # full clean rebuild + ROS env
#   source scripts/setup_falcon.sh --setup   # ROS env only (no build; needs install/)
#
# Uses a project virtualenv at $FALCON_VENV (default: repo_root/.falcon). If it
# does not exist, it is created and scripts/requirements-falcon-env.txt is installed.
#
# Verbose pip (download/install progress): default is -v. Quieter: FALCON_PIP_VERBOSE= source …
# More detail: FALCON_PIP_VERBOSE=-vv source …
#
# Foxglove bridge: cloned into falcon_ws/src/foxglove-sdk (no Humble apt package on Jammy)
# and built with the rest via colcon. Streamed build logs: FALCON_COLCON_EVENT_HANDLERS (default console_direct+).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${WS_ROOT}/falcon_ws"
FALCON_ENV_REQUIREMENTS="${SCRIPT_DIR}/requirements-falcon-env.txt"

# Default: project venv at repo root (override with FALCON_VENV=/path/to/venv)
FALCON_VENV="${FALCON_VENV:-${WS_ROOT}/.falcon}"

# Pip verbosity: empty = default, "-v" or "-vv" for download/install progress (override with FALCON_PIP_VERBOSE)
FALCON_PIP_VERBOSE="${FALCON_PIP_VERBOSE:--v}"

# colcon: stream compiler output (like verbose pip). Quieter: FALCON_COLCON_EVENT_HANDLERS=console_cohesion+ or verbose- console_direct+ 
FALCON_COLCON_EVENT_HANDLERS="${FALCON_COLCON_EVENT_HANDLERS:-console_cohesion+}"

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
    _pip() { python -m pip install ${FALCON_PIP_VERBOSE} "$@"; }
    if ! _pip --upgrade pip; then
      echo "[setup_falcon] pip upgrade failed."
      return 1
    fi
    # Force uninstall empy and EmPy to prevent PIP case-sensitivity caching bugs (EmPy 4 shadowing EmPy 3)
    python -m pip uninstall -y empy EmPy >/dev/null 2>&1 || true
    if ! _pip -r "${FALCON_ENV_REQUIREMENTS}"; then
      echo "[setup_falcon] pip install -r ${FALCON_ENV_REQUIREMENTS} failed (check network / errors above)."
      return 1
    fi
    # ROS Humble cv_bridge was built against NumPy 1.x; downgrade if something pulled NumPy 2.
    _npmaj="$(python -c "import numpy; print(int(numpy.__version__.split('.')[0]))" 2>/dev/null || echo 0)"
    if [ "${_npmaj}" = "2" ]; then
      echo "[setup_falcon] NumPy 2.x breaks ROS cv_bridge; installing numpy>=1.21,<2 …"
      if ! _pip "numpy>=1.21.0,<2"; then
        echo "[setup_falcon] numpy downgrade failed."
        return 1
      fi
    fi
    # With --system-site-packages, apt's python3-colcon-* can satisfy colcon-common-extensions
    # without creating ${FALCON_VENV}/bin/colcon. Force a venv-local colcon so sys.executable is venv Python.
    if ! _pip --ignore-installed colcon-common-extensions; then
      echo "[setup_falcon] Failed to install colcon into venv (ignore-installed colcon-common-extensions)."
      return 1
    fi
    if [ ! -x "${FALCON_VENV}/bin/colcon" ]; then
      echo "[setup_falcon] ${FALCON_VENV}/bin/colcon missing after pip install."
      echo "  Install deps, then: python -m pip install -r ${FALCON_ENV_REQUIREMENTS}"
      return 1
    fi

    _st="$(python -c "import setuptools; print(setuptools.__version__.split('.')[0])" 2>/dev/null || echo 0)"
    if [ "${_st}" -lt 64 ] 2>/dev/null; then
      echo "[setup_falcon] Warning: setuptools still < 64 after pip install; colcon --symlink-install may fail."
    fi

    # PyTorch with CUDA 12.6 support (perception stack needs torch + ultralytics).
    if ! python -c "import torch" 2>/dev/null; then
      echo "[setup_falcon] Installing PyTorch (CUDA 12.6) …"
      if ! _pip torch torchvision \
           --index-url https://download.pytorch.org/whl/cu126; then
        echo "[setup_falcon] PyTorch install failed (network or CUDA issue)."
        echo "  For CPU-only: pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu"
        return 1
      fi
    fi
    # torch/ultralytics can upgrade NumPy to 2.x; cv_bridge still needs 1.x
    _npmaj="$(python -c "import numpy; print(int(numpy.__version__.split('.')[0]))" 2>/dev/null || echo 0)"
    if [ "${_npmaj}" = "2" ]; then
      echo "[setup_falcon] Re-pinning numpy<2 for ROS cv_bridge compatibility …"
      if ! _pip "numpy>=1.21.0,<2"; then
        echo "[setup_falcon] numpy re-pin failed."
        return 1
      fi
    fi
    # Pip 23+ frequently caches empy-4.x wheels dynamically downloaded by indirect EmPy dependencies.
    # We must rigorously obliterate and force empy 3.3.4 so rosidl_adapter.resource does not crash on BUFFERED_OPT.
    echo "[setup_falcon] Enforcing empy==3.3.4 for ROS Humble rosidl …"
    if ! _pip 'empy==3.3.4' --force-reinstall --no-deps >/dev/null 2>&1; then
      echo "[setup_falcon] empy 3.3.4 pin failed."
      return 1
    fi
  fi
  return 0
}

# colcon must be the venv copy so Python packages are built with venv Python (see colcon_python_setup_py).
_falcon_colcon() {
  if [ ! -x "${FALCON_VENV}/bin/colcon" ]; then
    echo "[setup_falcon] ${FALCON_VENV}/bin/colcon not found. Activate venv and: pip install -r ${FALCON_ENV_REQUIREMENTS}"
    return 127
  fi
  # --event-handlers belongs to the build verb (streams compiler output; not a top-level colcon flag)
  if [ "${1:-}" = "build" ]; then
    shift
    "${FALCON_VENV}/bin/colcon" build --event-handlers "${FALCON_COLCON_EVENT_HANDLERS}" "$@"
  else
    "${FALCON_VENV}/bin/colcon" "$@"
  fi
}

# Foxglove bridge is not packaged for Humble on Ubuntu 22.04; we clone upstream and colcon-build it.
_falcon_ensure_foxglove_sdk() {
  _fg="${WS_DIR}/src/foxglove-sdk"
  _pkg="${_fg}/ros/src/foxglove_bridge/package.xml"
  if [ ! -f "${_pkg}" ]; then
    echo "[setup_falcon] Cloning foxglove-sdk into falcon_ws/src/foxglove-sdk (provides foxglove_bridge) …"
    if ! git clone --depth 1 --branch main https://github.com/foxglove/foxglove-sdk.git "${_fg}"; then
      echo "[setup_falcon] git clone foxglove-sdk failed (network?)."
      return 1
    fi
  else
    echo "[setup_falcon] foxglove-sdk already present: src/foxglove-sdk"
  fi
  return 0
}

# System deps for foxglove_bridge (websocketpp, ssl, …) and the rest of src/
_falcon_rosdep_install() {
  if ! command -v rosdep >/dev/null 2>&1; then
    echo "[setup_falcon] rosdep not found; install: sudo apt install python3-rosdep && sudo rosdep init && rosdep update"
    return 0
  fi
  echo "[setup_falcon] rosdep install --from-paths src (verbose; may ask for sudo) …"
  # Refresh apt cache since Docker caches get stale and ros apt packages churn frequently
  sudo apt-get update -y >/dev/null 2>&1 || true
  # -r = continue on errors so one missing rosdep key does not abort the whole workspace
  rosdep install --from-paths "${WS_DIR}/src" --ignore-src -y -r || true
  return 0
}

# Empy 4.x breaks rosidl_adapter (foxglove_msgs, etc.); ensure venv uses Empy 3 before colcon.
_falcon_ensure_rosidl_python_compat() {
  # shellcheck source=/dev/null
  source "${FALCON_VENV}/bin/activate"
  _empy_maj="$(python -c "import importlib.metadata as m; print(int(m.version('empy').split('.')[0]))" 2>/dev/null || echo 99)"
  if [ "${_empy_maj}" -ge 4 ] 2>/dev/null; then
    echo "[setup_falcon] empy>=4 detected before colcon; forcing empy>=3.3,<4 for rosidl_adapter …"
    if ! python -m pip install ${FALCON_PIP_VERBOSE} 'empy>=3.3,<4' --force-reinstall; then
      echo "[setup_falcon] empy downgrade failed."
      return 1
    fi
  fi
  return 0
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

_falcon_ensure_foxglove_sdk || { _e=$?; return "$_e" 2>/dev/null || exit "$_e"; }

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

_falcon_rosdep_install

_falcon_ensure_rosidl_python_compat || { _e=$?; return "$_e" 2>/dev/null || exit "$_e"; }

echo "[setup_falcon] Running colcon build --symlink-install --merge-install (event-handlers: ${FALCON_COLCON_EVENT_HANDLERS})"
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
