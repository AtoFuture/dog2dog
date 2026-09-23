#!/bin/bash
# 起 nav2 —— **隔离版**：接到 run_sim_iso.sh 起的那个仿真上
#
# 与 start_nav2.sh 的唯一实质区别：ROS_DOMAIN_ID 42 → 43。
#
# 为什么必须分开（不是洁癖，是会静默连错）：
#   start_nav2.sh 的 42 是**共用仿真**的域号。如果仿真已经在 43 上隔离跑着，
#   你却用 42 起 nav2，AMCL 收不到自己那只狗的 scan/tf，却可能收到别人的 ——
#   表现是 AMCL 一直不收敛或定位跳到别人那里，而且不报任何错。
#
# 配套：
#   ~/go2_sim_ws/run_sim_iso.sh    DISPLAY=:3  ROS_DOMAIN_ID=43  IGN_PARTITION=wy
# ⚠️ nav2 不碰 gz-transport，所以这里**不需要** IGN_PARTITION；
#    但 g1_loctest.py 读真值要，它的 wrapper 里已经带了。
#
# 用法：
#   ./start_nav2_iso.sh
#   ./start_nav2_iso.sh map:=/path/to/other.yaml      # 换地图

export ROS_DOMAIN_ID=43
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
source /opt/ros/humble/setup.bash
source /home/wy/go2_sim_ws/ROS2-Gazebo-GO2/install/setup.bash
exec ros2 launch navigation2 go2_navigation2.launch.py "$@"
