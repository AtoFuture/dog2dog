#!/bin/bash
pkill -f rtabmap_go2_rgbd_odom 2>/dev/null
pkill -f rtabmap_slam 2>/dev/null
pkill -f rgbd_sync 2>/dev/null
sleep 2
rm -f "$HOME/data1/创新实践/baseline/g1_mapping_nav/3d/rtabmap/trial1.log"
rm -f "$HOME/data1/创新实践/baseline/g1_mapping_nav/3d/rtabmap/rtabmap_trial1.db"
nohup "$HOME/.enroot/ros-gui" bash -c 'export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml; source /opt/ros/humble/setup.bash; source /home/wy/go2_sim_ws/ROS2-Gazebo-GO2/install/setup.bash; timeout 75 ros2 launch "$HOME/data1/创新实践/baseline/g1_mapping_nav/3d/rtabmap/rtabmap_go2_rgbd_odom.launch.py"' > "$HOME/data1/创新实践/baseline/g1_mapping_nav/3d/rtabmap/trial1.log" 2>&1 &
echo BG:$!
