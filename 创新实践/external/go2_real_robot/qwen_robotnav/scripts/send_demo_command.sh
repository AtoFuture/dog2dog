#!/usr/bin/env bash
# Send one G3 command to the running demo on DEMO_DOMAIN (default 227).
# Usage: ./scripts/send_demo_command.sh EXPLORE|RETURN|PAUSE|RESUME|ESTOP|RESET|STANDBY
set -euo pipefail

CMD="${1:-}"
case "${CMD,,}" in
  explore|return|pause|resume|estop|reset|standby) ;;
  *)
    echo "Usage: $0 EXPLORE|RETURN|PAUSE|RESUME|ESTOP|RESET|STANDBY" >&2
    exit 2
    ;;
esac

DOMAIN="${DEMO_DOMAIN:-227}"
SIM_WS="${G1_SIM_WORKSPACE:-${HOME}/go2_sim_ws/ROS2-Gazebo-GO2}"
exec "${HOME}/.enroot/ros" bash -lc "
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID='${DOMAIN}'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='file://${SIM_WS}/src/docker/cyclonedds.xml'
ros2 topic pub --once /g3/command std_msgs/msg/String \"{data: ${CMD^^}}\"
"
