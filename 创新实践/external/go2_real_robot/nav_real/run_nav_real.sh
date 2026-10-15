#!/bin/bash
# 真机导航编排（.18 上执行；感知+导航先行，不含运动指令）
# 前置: baiyanggo-webrtc-bridge 运行中（三流话题可用）
# 环境: 已在调用方 source ROS + cyclonedds_ws，并 export RMW/CYCLONEDDS_URI
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
LOG=/tmp/nav_real
mkdir -p "$LOG"

echo "[1/4] pose_tf_relay: 位姿→TF(odom→base_link)+Odometry"
for i in 1 2 3; do
  python3 -u "$DIR/scripts/pose_tf_relay.py" > "$LOG/tf_relay.log" 2>&1 &
  RELAY_PID=$!
  sleep 5
  kill -0 $RELAY_PID 2>/dev/null && { echo "relay 第${i}次启动成功"; break; }
  echo "relay 第${i}次启动失败(relay 重试)"
  sleep 5
done

echo "[2/5] pointcloud_to_laserscan: voxel_map→/scan (base_link系)"
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args --params-file "$DIR/config/pointcloud_to_laserscan_real.yaml" \
  -r cloud_in:=/baiyanggo/webrtc/voxel_map -r scan:=/scan > "$LOG/pc2scan.log" 2>&1 &
sleep 3

echo "[3/5] 定位: 直通TF（sport_odom 腿部里程计，pose_tf_relay 直发 odom→base_link）"
# BYY修复⑭：主导航里程计 = 运动控制器腿部里程计（rt/lf/sportmodestate，无翻转）。
# utlidar 只供障碍点云；EKF/utlidar直发 均废弃（见 bridge_node 注释⑨⑫⑬）。
# Foxy 版 static_transform_publisher 只认位置参数: x y z qx qy qz qw parent child
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom \
  > "$LOG/static_tf.log" 2>&1 &
sleep 2

echo "[3.6/5] 传感器系挂接: odom→odom_utlidar 静态TF（体素地图挂进导航系）"
python3 -u "$DIR/scripts/static_sensor_frame.py" > "$LOG/static_sensor_frame.log" 2>&1 &
sleep 2

echo "[4/5] Nav2 bringup（无 map_server/AMCL，探索模式）"
# Foxy 的 bringup_launch.py 强制要求 map 参数；探索模式用 navigation_launch.py
# （只起 controller/planner/bt_navigator/behaviors/velocity_smoother，无 map_server/AMCL）
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false autostart:=true \
  params_file:="$DIR/config/nav2_real.yaml" \
  > "$LOG/nav2.log" 2>&1 &
sleep 25
echo "=== 状态 ==="
ros2 node list 2>/dev/null | grep -cE "controller_server|planner_server|bt_navigator|pose_tf_relay" | xargs echo "关键节点数:"
timeout 6 ros2 topic hz /scan 2>/dev/null | tail -1
timeout 6 ros2 topic hz /baiyanggo/webrtc/pose 2>/dev/null | tail -1
echo "日志: $LOG/（tf_relay/pc2scan/slam/nav2）"
echo "探索: python3 $DIR/scripts/frontier_explorer.py （或手动发 goal）"

echo "[nav_real] 全部组件已启动，进入守护等待（systemd 停服务时统一回收）"
wait
