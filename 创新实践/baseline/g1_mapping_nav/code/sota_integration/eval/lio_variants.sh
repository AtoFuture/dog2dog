#!/bin/bash
# lio_variants.sh — 一次 bringup，测多个 Point-LIO 配置变体（机器人站立不动）。
WS=/home/wy/go2_sim_ws/ROS2-Gazebo-GO2
LIO=/home/wy/lio_ws
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$WS/src/docker/cyclonedds.xml
export IGN_PARTITION=go2wy
export GZ_PARTITION=go2wy
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
source $LIO/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
unset EGL_PLATFORM
LOG=/tmp/lio_variants.log
exec > >(tee -a $LOG) 2>&1
echo "=== lio_variants start $(date) ==="
TOOLS=$(dirname "$(find /tmp/dog2dog_push -path '*code/tools/spawn_robot1.launch.py' 2>/dev/null | head -1)")
[ -z "$TOOLS" ] && TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
FIXER=/home/wy/go2_sim_ws/sota_integration/lio/gz_lidar_fixer.py
CFG=/tmp/plio_gz.yaml
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"
for p in 'ign gazebo.*warehouse_cross' '[p]arameter_bridge' '[r]obot_controller_gazebo' '[c]md_vel_pub' '[r]obot_state_publisher' '[g]z_lidar_fixer' '[p]ointlio' '[p]lio_probe'; do pkill -9 -f "$p" 2>/dev/null; done; sleep 5

echo "[1] gazebo"; ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6 > /tmp/lv_gz.log 2>&1 &
sleep 55; echo "  gz=$(pgrep -cf 'ign gazebo.*warehouse_cross')"
echo "[2] bridges"
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=$WS/install/gazebo_sim/share/gazebo_sim/config/gz_bridge.yaml -p use_sim_time:=True > /tmp/lv_br1.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  /robot1/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  --ros-args -r __node:=ros_gz_bridge -r __ns:=/robot1 -p use_sim_time:=True > /tmp/lv_br2.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge /robot1/velodyne/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked --ros-args -r __node:=pc_bridge > /tmp/lv_br3.log 2>&1 &
sleep 8
echo "[3] pause+spawn"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 60000 --req 'pause: true' >/dev/null 2>&1
ros2 launch $TOOLS/spawn_robot1.launch.py > /tmp/lv_spawn.log 2>&1 &
sleep 45; tail -1 /tmp/lv_spawn.log
echo "[4] spawners + controller"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/lv_jsb.log 2>&1 &
sleep 3
ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/lv_jgc.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/robot_controller_gazebo.py --ros-args -r __node:=quadruped_controller -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/lv_ctrl.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/lv_cvp.log 2>&1 &
python3 -u $FIXER --ros-args -p in_topic:=/robot1/velodyne/points -p out_topic:=/robot1/velodyne/fixed > /tmp/lv_fix.log 2>&1 &
echo "[5] unpause"; ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 120000 --req 'pause: false'
echo "[6] wait jgc"; for i in $(seq 1 32); do grep -q 'Configured and activated' /tmp/lv_jgc.log 2>/dev/null && { echo "  jgc ~$((i*15))s"; break; }; sleep 15; done
echo "[7] TROT + settle"; python3 $TOOLS/mode_stick.py TROT > /tmp/lv_trot.log 2>&1; sleep 12

run_variant () {
  local name="$1"; shift
  echo "---- VARIANT $name : $* ----"
  ros2 run point_lio pointlio_mapping --ros-args --params-file $CFG "$@" > /tmp/lv_plio_$name.log 2>&1 &
  local pid=$!
  sleep 10
  timeout 40 python3 -u /home/wy/go2_sim_ws/sota_integration/eval/plio_probe.py 22 2>&1 | tail -1
  kill -9 $pid 2>/dev/null
  pkill -9 -f '[p]ointlio' 2>/dev/null
  sleep 3
}

run_variant V1_imu_meas -p use_imu_as_input:=False -p prop_at_freq_of_imu:=True -p point_filter_num:=4
run_variant V2_imu_input -p use_imu_as_input:=True -p prop_at_freq_of_imu:=True -p point_filter_num:=4
run_variant V3_imu_inp_all -p use_imu_as_input:=True -p prop_at_freq_of_imu:=True -p point_filter_num:=1

echo "=== lio_variants DONE $(date) ==="
