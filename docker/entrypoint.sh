#!/usr/bin/env bash
set -e

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/ws/okvis_ws/install/setup.bash
source /opt/ws/generalist_bt_gen/install/setup.bash
set -u

exec "$@"
