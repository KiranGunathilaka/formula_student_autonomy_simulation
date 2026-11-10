#!/bin/bash
# Repeat laps for the localisation comparison: each lidar drives lidar_test_track again,
# headless, with other sensor-noise seeds (localisation only: no coverage, no recording),
# then lidar_stats and the page are rebuilt with the spread over all laps.
#
#   OUT_NAME=lidar_loc_2026-11-09 scripts/localization_repeats.sh          # every lidar
#   OUT_NAME=lidar_loc_2026-11-09 SEEDS="2 3 4" scripts/localization_repeats.sh hesai_atx
#   OUT_NAME=... SEEDS="6 7" REPEATS=repeats_detector10 EXTRA="detector_min_points:=10" \
#       scripts/localization_repeats.sh        # a variant of the stack, in its own folder
#
# OUT_NAME is the folder of a scripts/lidar_stats_gazebo.sh run (its laps are seed 1);
# the repeats go to runs/<OUT_NAME>/<REPEATS>/<lidar>_s<seed>/ (REPEATS: default repeats;
# a repeats_<name> folder is a variant, shown beside the main laps).  One lap is about three
# minutes on the RTX 5060 laptop.  Run it on the host, from the repo, with the desktop's
# DISPLAY and XAUTHORITY set (Gazebo's GPU lidar renders through X; on the RTX laptop
# DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority).

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAME=falcon_container
OUT_NAME="${OUT_NAME:?set OUT_NAME to the lidar_stats_gazebo.sh run folder}"
TRACK="${TRACK:-lidar_test_track}"
SPEED="${SPEED:-3.0}"
SEEDS="${SEEDS:-2 3}"
REPEATS="${REPEATS:-repeats}"
EXTRA="${EXTRA:-}"
DEFAULT_LIDARS=(hesai_atx velodyne_vlp16 robosense_helios_1615 robosense_helios_5515
                hesai_xt32 ouster_os1_32 ouster_os1_64 ouster_os1_128 ouster_os2_128)
if [ $# -gt 0 ]; then LIDARS=("$@"); else LIDARS=("${DEFAULT_LIDARS[@]}"); fi

if [ -z "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "${NAME} is not running; start it (docker/docker_run.sh) or run lidar_stats_gazebo.sh first." >&2
    exit 1
fi
in_container() {
    docker exec -e DISPLAY="${DISPLAY:-:0}" "${NAME}" /ros_entrypoint.sh bash -c "$1"
}
if in_container "pgrep -f '[g]zserver' >/dev/null"; then
    echo "A simulator is already running in ${NAME}; stop it first." >&2
    exit 1
fi

for L in "${LIDARS[@]}"; do
    MOUNT=""
    [ "${L}" != "hesai_atx" ] && MOUNT="lidar_z:=0.70"
    for S in ${SEEDS}; do
        RUN="${L}_s${S}"
        echo "[repeats] ${RUN}"
        in_container "timeout 900 ros2 launch falcon_bringup sim.launch.py track:=${TRACK} lidar:=${L} \
${MOUNT} laps:=1 target_speed:=${SPEED} use_camera:=false gazebo_gui:=false rviz:=false \
coverage_exit_on_finish:=true sensor_seed:=${S} ${EXTRA} \
coverage_output_dir:=/root/falcon_runs/${OUT_NAME}/${REPEATS} coverage_run_name:=${RUN} \
> /root/falcon_runs/${OUT_NAME}/${REPEATS}_${RUN}.log 2>&1"
        in_container "pkill -f '[g]zserver|[g]zclient|[i]nstall/lib/|[r]viz2' ; sleep 3; true"
        LINE="$(grep -h "position p50\|partial report" "${ROOT}/runs/${OUT_NAME}/${REPEATS}_${RUN}.log" | tail -1)"
        echo "${LINE}" | cut -c1-400
        # No cones in the map: the lidar gave no clouds (gpu_ray needs the X display:
        # export DISPLAY and XAUTHORITY of the desktop session before running this).
        if [ -z "${LINE}" ] || echo "${LINE}" | grep -q "map 0/"; then
            echo "[repeats] ${RUN} mapped no cones; stopping (DISPLAY=${DISPLAY:-unset})" >&2
            exit 1
        fi
    done
done

echo
VARIANTS=""
for D in "${ROOT}/runs/${OUT_NAME}"/repeats_*/; do
    [ -d "${D}" ] && V="$(basename "${D}")" && VARIANTS+=" --variant ${V#repeats_} /root/falcon_runs/${OUT_NAME}/${V}"
done
in_container "ros2 run falcon_evaluation lidar_stats --runs \
\$(dirname /root/falcon_runs/${OUT_NAME}/*/frames.npz) --repeats /root/falcon_runs/${OUT_NAME}/repeats \
${VARIANTS} --out /root/falcon_runs/${OUT_NAME}" || exit 1
python3 "${ROOT}/docs/build_lidar_statistics.py" "${ROOT}/runs/${OUT_NAME}/lidar_stats.json"
echo "[repeats] done"
