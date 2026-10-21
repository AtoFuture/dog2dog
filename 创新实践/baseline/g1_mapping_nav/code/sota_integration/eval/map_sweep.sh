#!/bin/bash
# map_sweep.sh — 更全更干净的建图：慢速 + 段间站立静止 + 多段。
WS=/home/wy/go2_sim_ws/ROS2-Gazebo-GO2
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$WS/src/docker/cyclonedds.xml
export IGN_PARTITION=go2wy
export GZ_PARTITION=go2wy
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
unset EGL_PLATFORM
SEGS="${1:-10}"
LOG=/tmp/map_sweep.log
exec > >(tee -a $LOG) 2>&1
echo "=== map_sweep start $(date) segs=$SEGS ==="
TOOLS=$(dirname "$(find /tmp/dog2dog_push -path '*code/tools/spawn_robot1.launch.py' 2>/dev/null | head -1)")
[ -z "$TOOLS" ] && TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"
for p in 'ign gazebo.*warehouse_cross' '[p]arameter_bridge' '[Q]uadrupedOdometry' '[e]kf_node' '[c]md_vel_pub' '[r]obot_controller_gazebo' '[a]sync_slam' '[m]ap_relay' '[r]obot_state_publisher'; do pkill -9 -f "$p" 2>/dev/null; done; sleep 5

echo "[1] gazebo"; ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6 > /tmp/ms_gz.log 2>&1 &
sleep 55; echo "  gz=$(pgrep -cf 'ign gazebo.*warehouse_cross')"
echo "[2] bridges"
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=$WS/install/gazebo_sim/share/gazebo_sim/config/gz_bridge.yaml -p use_sim_time:=True > /tmp/ms_br1.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  /robot1/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /robot1/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  --ros-args -r __node:=ros_gz_bridge -r __ns:=/robot1 -p use_sim_time:=True > /tmp/ms_br2.log 2>&1 &
sleep 8; echo "  bridges=$(pgrep -cf '[p]arameter_bridge')"
echo "[3] pause+spawn"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 60000 --req 'pause: true' >/dev/null 2>&1
ros2 launch $TOOLS/spawn_robot1.launch.py > /tmp/ms_spawn.log 2>&1 &
sleep 45; tail -1 /tmp/ms_spawn.log
echo "[4] spawners + control + odom/ekf"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/ms_jsb.log 2>&1 &
sleep 3
ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/ms_jgc.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/robot_controller_gazebo.py --ros-args -r __node:=quadruped_controller -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/ms_ctrl.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/QuadrupedOdometryNode.py --ros-args -r __node:=odom -r __ns:=/robot1 -p use_sim_time:=True -p base_frame_id:=base_link -p odom_frame_id:=odom -p enable_odom_tf:=false $REMAP > /tmp/ms_odom.log 2>&1 &
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node -r __ns:=/robot1 -p use_sim_time:=True --params-file $WS/install/gazebo_sim/share/gazebo_sim/config/ekf.yaml $REMAP > /tmp/ms_ekf.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/ms_cvp.log 2>&1 &
sleep 22
echo "[5] unpause"; ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 120000 --req 'pause: false'
echo "[6] slam+relay (default params, 更保守)"
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml -p use_sim_time:=true -p scan_topic:=scan -p map_frame:=map -p base_frame:=base_link -p odom_frame:=odom -r __ns:=/robot1 -r /scan:=scan -r /odom:=odometry/filtered > /tmp/ms_slam.log 2>&1 &
python3 -u $TOOLS/map_relay2.py > /tmp/ms_relay.log 2>&1 &
sleep 12
echo "[7] wait jgc"; for i in $(seq 1 32); do grep -q 'Configured and activated' /tmp/ms_jgc.log 2>/dev/null && { echo "  jgc ~$((i*15))s"; break; }; sleep 15; done
echo "[8] TROT + sweep (fwd/stand/turn/stand) x$SEGS"
python3 $TOOLS/mode_stick.py TROT > /tmp/ms_trot.log 2>&1
sleep 10
for i in $(seq 1 $SEGS); do
  echo "  seg $i fwd";  python3 -u $TOOLS/drive_vel.py fwd_slow 14 > /dev/null 2>&1
  echo "  seg $i settle"; sleep 5
  echo "  seg $i turn"; python3 -u $TOOLS/drive_vel.py turnl 30 > /dev/null 2>&1
  echo "  seg $i settle"; sleep 5
done
echo "  sweep done"
echo "[9] save map"
mkdir -p /tmp/sweepmap
ros2 run nav2_map_server map_saver_cli -f /tmp/sweepmap/sweep_$(date +%H%M%S) 2>&1 | tail -2
ls -la /tmp/sweepmap/ | tail -4
echo "=== map_sweep DONE $(date) ==="
