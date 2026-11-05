#!/bin/bash
# Build the Falcon packages inside the container and source the result:
#
#   source /workspace/docker/build.sh            # the Falcon packages
#   source /workspace/docker/build.sh <pkg> ...  # just these
#
# EUFS, YOLO and Foxglove come prebuilt in the image.

_falcon_pkgs="${*:-falcon_msgs falcon_common falcon_description falcon_planning falcon_teleop \
falcon_cone_perception falcon_cone_map_builder falcon_evaluation falcon_bringup}"
(
    cd /workspace/falcon_ws &&
    PYTHONNOUSERSITE=1 colcon build --merge-install --symlink-install \
        --event-handlers console_cohesion- --packages-select ${_falcon_pkgs} \
        --cmake-args "-DPYTHON_EXECUTABLE=${FALCON_VENV}/bin/python3" \
        "-DPython3_EXECUTABLE=${FALCON_VENV}/bin/python3"
) && source /workspace/falcon_ws/install/setup.bash
unset _falcon_pkgs
