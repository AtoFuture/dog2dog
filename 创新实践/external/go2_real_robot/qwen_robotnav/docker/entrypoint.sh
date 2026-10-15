#!/bin/bash
set -e

# Source ROS2
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# Source local ROS2 workspace
if [ -f /workspace/qwen_robotnav/ros2_ws/install/setup.bash ]; then
    source /workspace/qwen_robotnav/ros2_ws/install/setup.bash
fi

# Set default DDS for Jetson / multicast-less operation
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/workspace/qwen_robotnav/docker/fastrtps.xml}

exec "$@"
