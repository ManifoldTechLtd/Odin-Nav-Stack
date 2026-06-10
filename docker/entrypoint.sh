#!/usr/bin/env bash
# =============================================================================
# Odin Navigation Stack — container entrypoint
#
# Modes:
#   launch  (default)  — start whole.launch + neupan_ros.py
#   shell              — drop into an interactive bash with ROS sourced
#   <anything else>    — exec the rest of the args as a command
# =============================================================================
set -e

ROS_DISTRO="${ROS_DISTRO:-noetic}"
WS_DIR="/opt/odin/ros_ws"

# Source ROS + the workspace overlay
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS_DIR}/devel/setup.bash" ]; then
    source "${WS_DIR}/devel/setup.bash"
fi

# Activate the neupan conda env for python deps (path is already on PATH but
# we set CONDA_DEFAULT_ENV for any tooling that checks it).
export CONDA_DEFAULT_ENV=neupan

mode="${1:-launch}"

case "${mode}" in
    launch)
        echo "[entrypoint] starting whole.launch + neupan_ros.py"
        # Start whole.launch in background (it spawns roscore implicitly)
        roslaunch map_planner whole.launch &
        LAUNCH_PID=$!

        # Give roscore + nodes a moment to come up before starting NeuPAN
        sleep 3

        # NeuPAN python node — runs in foreground; if it dies the container exits
        /opt/conda/envs/neupan/bin/python /opt/odin/NeuPAN/neupan/ros/neupan_ros.py &
        NEUPAN_PID=$!

        # Forward SIGTERM/SIGINT to children for clean shutdown
        trap 'kill -TERM ${LAUNCH_PID} ${NEUPAN_PID} 2>/dev/null; wait' TERM INT
        wait -n
        # If either process exits, take the whole container down
        kill -TERM ${LAUNCH_PID} ${NEUPAN_PID} 2>/dev/null || true
        wait
        ;;
    shell)
        exec /bin/bash
        ;;
    *)
        exec "$@"
        ;;
esac
