#!/usr/bin/env bash
set -euo pipefail

source_root="/home/wy/data1/创新实践/external/aws-robomaker-small-warehouse-world"
compat_root="/home/wy/data1/创新实践/external/aws-small-warehouse-gzsim-compat"
go2_ws="/home/wy/go2_sim_ws/ROS2-Gazebo-GO2"
partition="${IGN_PARTITION:-aws_small_warehouse_20260921}"
domain_id="${ROS_DOMAIN_ID:-142}"

exec /home/wy/.enroot/ros-gui bash -lc '
  source /opt/ros/humble/setup.bash
  source "$1/install/setup.bash"
  export IGN_PARTITION="$2"
  export ROS_DOMAIN_ID="$3"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$1/src/docker/cyclonedds.xml"
  export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
  export IGN_GAZEBO_RESOURCE_PATH="$4/models:$5/models"
  exec ign gazebo -v 3 "$4/small_warehouse_gzsim.sdf"
' _ "${go2_ws}" "${partition}" "${domain_id}" "${compat_root}" "${source_root}"
