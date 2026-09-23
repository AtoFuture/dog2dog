#!/usr/bin/env bash
set -euo pipefail

go2_ws="/home/wy/go2_sim_ws/ROS2-Gazebo-GO2"
partition="${IGN_PARTITION:-aws_small_warehouse_20260921}"
domain_id="${ROS_DOMAIN_ID:-142}"

exec /home/wy/.enroot/ros-gui bash -lc '
  cd "$1"
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  source /home/wy/data1/创新实践/external/aws-small-warehouse-gzsim-compat/go2_ik_overlay/install/setup.bash
  export IGN_PARTITION="$2"
  export ROS_DOMAIN_ID="$3"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$1/src/docker/cyclonedds.xml"
  ign service -s /world/default/control \
    --reqtype ignition.msgs.WorldControl \
    --reptype ignition.msgs.Boolean \
    --timeout 3000 --req "pause: false" >/dev/null
  exec ros2 launch gazebo_sim gazebo_go2_sensors.launch.py enable_rviz:=false
' _ "${go2_ws}" "${partition}" "${domain_id}"
