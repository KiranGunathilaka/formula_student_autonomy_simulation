#!/bin/bash
# Lidar statistics from Gazebo: drive lidar_test_track once per lidar with the
# Gazebo window and RViz open, record the screen of each run, then compute the
# statistics from the recorded frames and build docs/lidar_statistics.html.
#
#   scripts/lidar_stats_gazebo.sh                          # every lidar (~1 h)
#   scripts/lidar_stats_gazebo.sh hesai_atx velodyne_vlp16 # just these
#   RECORD=0 scripts/lidar_stats_gazebo.sh                 # record with your own tool
#   OUT_NAME=lidar_gazebo_2026-11-08 scripts/lidar_stats_gazebo.sh ouster_os1_64
#                                   # re-run one lidar into an earlier run's folder
#
# Run it on the host (the GPU machine's desktop), from the repo, in a normal
# terminal.  It starts falcon_container if it is not running, rebuilds the changed
# packages in it, and for each lidar runs
#   ros2 launch falcon_bringup sim.launch.py track:=lidar_test_track lidar:=<lidar>
#       laps:=1 coverage:=true coverage_source:=pointcloud ...
# The car drives the lap with the stack's planner and pure pursuit on the simulated
# cones (the same drive for every lidar); the coverage node counts Gazebo's lidar
# points on every cone in every frame and writes runs/<OUT_NAME>/<lidar>/frames.npz
# when the lap is done, which ends that run.  Cones in RViz turn green when they get
# 10 points on 2 channels.  The car drives on its own pose (localization:=slam: ADS-DV
# CAN and MTi-8 odometry, cone SLAM on this lidar), and localization_eval writes
# localization.json / .npz next to frames.npz, which the page compares too.
#
# Videos: docs/videos/lidar_<lidar>.mp4 (scripts/record_screen.sh, needs ffmpeg).  The
# whole screen is recorded: leave the machine alone while it runs (with wmctrl
# installed the script raises Gazebo and RViz first).
# Results: runs/<OUT_NAME>/lidar_stats.json and docs/lidar_statistics.html.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAME=falcon_container
OUT_NAME="${OUT_NAME:-lidar_gazebo_$(date +%Y-%m-%d)}"
TRACK="${TRACK:-lidar_test_track}"
SPEED="${SPEED:-3.0}"
RECORD="${RECORD:-1}"
RECORD_DELAY="${RECORD_DELAY:-20}"      # s after launch: skip the loading screen
RUN_TIMEOUT="${RUN_TIMEOUT:-2400}"      # s per lidar
DEFAULT_LIDARS=(hesai_atx velodyne_vlp16 robosense_helios_1615 robosense_helios_5515
                hesai_xt32 ouster_os1_32 ouster_os1_64 ouster_os1_128 ouster_os2_128)
if [ $# -gt 0 ]; then LIDARS=("$@"); else LIDARS=("${DEFAULT_LIDARS[@]}"); fi

if [ "${RECORD}" = "1" ] && ! command -v ffmpeg >/dev/null; then
    echo "ffmpeg not found: sudo apt install ffmpeg, or RECORD=0 to record yourself." >&2
    exit 1
fi

# 1. The container (detached; docker_run.sh gives it the GPU, X11 and ROS isolation).
if [ -z "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "[lidar_stats] starting ${NAME}"
    setsid nohup "${ROOT}/docker/docker_run.sh" sleep infinity </dev/null \
        >"${ROOT}/runs/.container.log" 2>&1 &
    for _ in $(seq 60); do
        [ -n "$(docker ps -q -f name="^${NAME}$")" ] && break
        sleep 1
    done
fi
if [ -z "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "${NAME} did not start; see runs/.container.log" >&2
    exit 1
fi
in_container() {
    docker exec -e DISPLAY="${DISPLAY:-:0}" "${NAME}" /ros_entrypoint.sh bash -c "$1"
}
if in_container "pgrep -f '[g]zserver' >/dev/null"; then
    echo "A simulator is already running in ${NAME}; stop it first." >&2
    exit 1
fi

# 2. Build what this test uses.
in_container "source /workspace/docker/build.sh falcon_msgs falcon_description falcon_planning \
falcon_cone_perception falcon_sim_sensors falcon_localization falcon_evaluation falcon_bringup" \
    || exit 1

# 3. One lap per lidar.
mkdir -p "${ROOT}/docs/videos"
for L in "${LIDARS[@]}"; do
    MOUNT=""
    [ "${L}" != "hesai_atx" ] && MOUNT="lidar_z:=0.70"   # spinners on a post above the plate
    echo
    echo "[lidar_stats] ${L}: driving ${TRACK} (run ${OUT_NAME}/${L})"
    in_container "ros2 launch falcon_bringup sim.launch.py track:=${TRACK} lidar:=${L} ${MOUNT} \
laps:=1 target_speed:=${SPEED} use_camera:=false gazebo_gui:=true rviz:=true \
coverage:=true coverage_source:=pointcloud coverage_cone_shape:=eufs \
coverage_min_points:=10 coverage_min_rings:=2 coverage_exit_on_finish:=true \
coverage_output_dir:=/root/falcon_runs/${OUT_NAME} coverage_run_name:=${L} \
> /root/falcon_runs/${OUT_NAME}_${L}.log 2>&1" &
    LAUNCH=$!
    REC=""
    if [ "${RECORD}" = "1" ]; then
        sleep "${RECORD_DELAY}"
        # Bring Gazebo, then RViz, to the front: the recording is of the whole screen, and
        # a window left in front records instead (sudo apt install wmctrl to enable).
        if command -v wmctrl >/dev/null; then
            wmctrl -a Gazebo; sleep 0.5; wmctrl -a RViz
        fi
        setsid "${SCRIPT_DIR}/record_screen.sh" "${ROOT}/docs/videos/lidar_${L}.mp4" &
        REC=$!
    fi
    SECONDS=0
    while kill -0 "${LAUNCH}" 2>/dev/null; do
        if [ "${SECONDS}" -gt "${RUN_TIMEOUT}" ]; then
            echo "[lidar_stats] ${L}: over ${RUN_TIMEOUT} s, stopping it (partial report)"
            in_container "pkill -INT -f '[b]in/ros2 launch falcon_bringup'"
        fi
        sleep 5
    done
    # ffmpeg ends the file cleanly on SIGTERM (a script's background jobs ignore SIGINT).
    [ -n "${REC}" ] && kill -TERM -- "-${REC}" 2>/dev/null && wait "${REC}" 2>/dev/null
    # Nothing of this run may drive the next one.
    in_container "pkill -f '[g]zserver|[g]zclient|[i]nstall/lib/|[r]viz2' ; sleep 3; true"
    grep -h "report written\|partial report\|position p50" \
        "${ROOT}/runs/${OUT_NAME}_${L}.log" | tail -4 | cut -c1-300
done

# 4. Statistics from every finished run in runs/<OUT_NAME> (so re-running one lidar
#    with the same OUT_NAME keeps the others), and the page.
echo
in_container "ros2 run falcon_evaluation lidar_stats --runs \
\$(dirname /root/falcon_runs/${OUT_NAME}/*/frames.npz) --out /root/falcon_runs/${OUT_NAME}" \
    || exit 1
python3 "${ROOT}/docs/build_lidar_statistics.py" "${ROOT}/runs/${OUT_NAME}/lidar_stats.json"
echo "[lidar_stats] done: docs/lidar_statistics.html, videos in docs/videos/"
