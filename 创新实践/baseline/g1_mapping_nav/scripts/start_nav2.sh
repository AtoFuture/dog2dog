#!/bin/bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
source /opt/ros/humble/setup.bash
source /home/wy/go2_sim_ws/ROS2-Gazebo-GO2/install/setup.bash
exec ros2 launch navigation2 go2_navigation2.launch.py
