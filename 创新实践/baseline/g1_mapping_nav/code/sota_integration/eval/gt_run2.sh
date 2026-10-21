#!/bin/bash
# gt_run2.sh — 全栈 + GT桥接(单向 GZ->ROS) + bag录制 + 探索 + 存图。
# 目的：得到一张"充分探索"的地图 + GT，使 IoU-vs-SDF 有意义。
WS=/home/wy/go2_sim_ws/ROS2-Gazebo-GO2
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$WS/src/docker/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
unset EGL_PLATFORM
EXPLORE_SEC="${1:-420}"
LOG=/tmp/gt_run2.log
exec > >(tee -a $LOG) 2>&1
echo "=== gt_run2 start $(date) ==="
TOOLS=$(dirname "$(find /tmp/dog2dog_push -path '*code/tools/spawn_robot1.launch.py' 2>/dev/null | head -1)")
[ -z "$TOOLS" ] && TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"
for p in '[i]gn gazebo' '[p]arameter_bridge' '[Q]uadrupedOdometry' '[e]kf_node' '[c]md_vel_pub' '[r]obot_controller_gazebo' '[a]sync_slam' '[m]ap_relay' '[r]obot_state_publisher' '[r]os2 bag' '[e]xplore.py'; do pkill -9 -f "$p" 2>/dev/null; done; sleep 4

echo "[1] gazebo"; ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6 > /tmp/g2_gz.log 2>&1 &
sleep 55; echo "  gz=$(pgrep -cf '[i]gn gazebo -s')"

# GT 桥接配置：单向 GZ->ROS，绝不用一行的双向写法
cat > /tmp/gtbridge.yaml <<YAML
- ros_topic_name: "/gt/pose"
  gz_topic_name: "/world/world_demo/pose/info"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "ignition.msgs.Pose_V"
  direction: GZ_TO_ROS
YAML
echo "[2] bridges"
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=$WS/install/gazebo_sim/share/gazebo_sim/config/gz_bridge.yaml -p use_sim_time:=True > /tmp/g2_br1.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  /robot1/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /robot1/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  --ros-args -r __node:=ros_gz_bridge -r __ns:=/robot1 -p use_sim_time:=True > /tmp/g2_br2.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=/tmp/gtbridge.yaml -p use_sim_time:=True > /tmp/g2_br3.log 2>&1 &
sleep 8; echo "  bridges=$(pgrep -cf '[p]arameter_bridge')"; grep -c GZ_TO_ROS /tmp/g2_br3.log

echo "[3] pause+spawn"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 60000 --req 'pause: true' >/dev/null 2>&1
ros2 launch $TOOLS/spawn_robot1.launch.py > /tmp/g2_spawn.log 2>&1 &
sleep 45; tail -1 /tmp/g2_spawn.log

echo "[4] spawners"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/g2_jsb.log 2>&1 &
sleep 3
ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/g2_jgc.log 2>&1 &

echo "[5] nodes"
python3 $WS/install/quadropted_controller/lib/quadropted_controller/robot_controller_gazebo.py --ros-args -r __node:=quadruped_controller -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/g2_ctrl.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/QuadrupedOdometryNode.py --ros-args -r __node:=odom -r __ns:=/robot1 -p use_sim_time:=True -p base_frame_id:=base_link -p odom_frame_id:=odom -p enable_odom_tf:=false $REMAP > /tmp/g2_odom.log 2>&1 &
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node -r __ns:=/robot1 -p use_sim_time:=True --params-file $WS/install/gazebo_sim/share/gazebo_sim/config/ekf.yaml $REMAP > /tmp/g2_ekf.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/g2_cvp.log 2>&1 &
sleep 25
echo "[6] unpause"; ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 120000 --req 'pause: false'

echo "[7] slam+relay"
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml -p use_sim_time:=true -p scan_topic:=scan -p map_frame:=map -p base_frame:=base_link -p odom_frame:=odom -p minimum_travel_distance:=0.0 -p minimum_travel_heading:=0.0 -r __ns:=/robot1 -r /scan:=scan -r /odom:=odometry/filtered > /tmp/g2_slam.log 2>&1 &
python3 -u $TOOLS/map_relay2.py > /tmp/g2_relay.log 2>&1 &
sleep 12

echo "[8] wait controllers"; for i in $(seq 1 32); do
  [ "$(grep -h 'Configured and activated' /tmp/g2_jsb.log /tmp/g2_jgc.log 2>/dev/null | wc -l)" -ge 2 ] && { echo "  active ~$((i*15))s"; break; }; sleep 15; done

echo "[9] bag record"
mkdir -p /tmp/gtbags2
ros2 bag record -o /tmp/gtbags2/run_$(date +%H%M%S) /gt/pose /robot1/odometry/filtered /tf /tf_static /clock /robot1/scan > /tmp/g2_bag.log 2>&1 &
BAGPID=$!
sleep 6; tail -1 /tmp/g2_bag.log

echo "[10] TROT + explore ${EXPLORE_SEC}s"
python3 $TOOLS/mode_stick.py TROT > /tmp/g2_trot.log 2>&1
sleep 10
python3 -u $TOOLS/explore.py $EXPLORE_SEC > /tmp/g2_explore.log 2>&1
echo "  explore rc=$?"; tail -2 /tmp/g2_explore.log

echo "[11] save map"
ros2 run nav2_map_server map_saver_cli -f /tmp/gtbags2/map_$(date +%H%M%S) 2>&1 | tail -2
sleep 2
echo "[12] stop bag"; kill -INT $BAGPID 2>/dev/null; sleep 5
ls -la /tmp/gtbags2/ | tail -5
echo "=== gt_run2 DONE $(date) ==="
