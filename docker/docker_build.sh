#!/bin/bash
set -e
# Build script for self-contained Falcon Autonomy container
# Works on Ubuntu 22.04 or Ubuntu 24.04 hosts

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[docker_build] Building falcon_autonomy:latest from ${WS_ROOT}"
docker build -t falcon_autonomy:latest -f "${SCRIPT_DIR}/Dockerfile" "${WS_ROOT}"
echo "[docker_build] Build complete."
