#!/bin/bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
source /opt/ros/humble/setup.bash
source /home/wy/go2_sim_ws/ROS2-Gazebo-GO2/install/setup.bash
exec ros2 run slam_toolbox async_slam_toolbox_node --ros-args -r __ns:=/robot1 -p use_sim_time:=true --params-file /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml -p scan_topic:=velodyne -p odom_topic:=odom -p base_frame:=base_link -p odom_frame:=odom -p map_frame:=map -r /tf:=tf -r /tf_static:=tf_static
