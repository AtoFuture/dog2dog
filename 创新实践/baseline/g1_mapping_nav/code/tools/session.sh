#!/bin/bash
# 在容器内运行：整个栈 + 探索 + 存图，全部在同一个容器会话里
WS=/home/wy/go2_sim_ws/ROS2-Gazebo-GO2
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$WS/src/docker/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
unset EGL_PLATFORM

LOGC=/tmp/session.log
exec > >(tee -a $LOGC) 2>&1
echo "=== session start $(date) ==="

TOOLS=$(dirname "$(find /tmp/dog2dog_push -path "*code/tools/spawn_robot1.launch.py" 2>/dev/null | head -1)")
[ -z "$TOOLS" ] && TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
echo "TOOLS=$TOOLS"
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"

cleanup() {
  pkill -9 -f '[i]gn gazebo' 2>/dev/null
  pkill -9 -f '[p]arameter_bridge' 2>/dev/null
  pkill -9 -f '[Q]uadrupedOdometry' 2>/dev/null
  pkill -9 -f '[e]kf_node' 2>/dev/null
  pkill -9 -f '[c]md_vel_pub' 2>/dev/null
  pkill -9 -f '[r]obot_controller_gazebo' 2>/dev/null
  pkill -9 -f '[a]sync_slam' 2>/dev/null
  pkill -9 -f '[m]ap_relay' 2>/dev/null
  pkill -9 -f '[r]obot_state_publisher' 2>/dev/null
  pkill -9 -f '[e]xplore.py' 2>/dev/null
  sleep 4
}
cleanup

echo "[1] gazebo"
ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6 > /tmp/s_gz.log 2>&1 &
sleep 55
pgrep -cf '[i]gn gazebo -s' | sed 's/^/  gz_alive=/'
grep -ciE segmentation /tmp/s_gz.log | sed 's/^/  segv=/'

echo "[2] bridges"
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=$WS/install/gazebo_sim/share/gazebo_sim/config/gz_bridge.yaml -p use_sim_time:=True > /tmp/s_br1.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  /robot1/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/velodyne_imu@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /robot1/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  --ros-args -r __node:=ros_gz_bridge -r __ns:=/robot1 -p use_sim_time:=True > /tmp/s_br2.log 2>&1 &
sleep 6
pgrep -cf '[p]arameter_bridge' | sed 's/^/  bridges=/'

echo "[3] pause + spawn"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 60000 --req 'pause: true' > /dev/null 2>&1
ros2 launch $TOOLS/spawn_robot1.launch.py > /tmp/s_spawn.log 2>&1 &
sleep 45
tail -1 /tmp/s_spawn.log | sed 's/^/  /'

echo "[4] spawners (long timeout)"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/s_jsb.log 2>&1 &
sleep 3
ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/s_jgc.log 2>&1 &

echo "[5] control nodes"
python3 $WS/install/quadropted_controller/lib/quadropted_controller/robot_controller_gazebo.py --ros-args -r __node:=quadruped_controller -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/s_ctrl.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/QuadrupedOdometryNode.py --ros-args -r __node:=odom -r __ns:=/robot1 -p use_sim_time:=True -p base_frame_id:=base_link -p odom_frame_id:=odom -p enable_odom_tf:=false $REMAP > /tmp/s_odom.log 2>&1 &
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node -r __ns:=/robot1 -p use_sim_time:=True --params-file $WS/install/gazebo_sim/share/gazebo_sim/config/ekf.yaml $REMAP > /tmp/s_ekf.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/s_cvp.log 2>&1 &
sleep 25
echo "[6] unpause"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 120000 --req 'pause: false' | sed 's/^/  /'

echo "[7] wait controllers active (poll up to 8min)"
for i in $(seq 1 32); do
  ST=$(timeout 25 ros2 param get /robot1/controller_manager > /dev/null 2>&1; echo ok)
  ACT=$(grep -c 'Configured and activated' /tmp/s_jsb.log /tmp/s_jgc.log 2>/dev/null | awk -F: '{s+=$2} END{print s}')
  if [ "$ACT" -ge 2 ]; then echo "  both active at ~$((i*15))s"; break; fi
  sleep 15
done
grep -h 'Configured and activated' /tmp/s_jsb.log /tmp/s_jgc.log | sed 's/^/  /'

echo "[8] slam + relay"
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml -p use_sim_time:=true -p scan_topic:=scan -p map_frame:=map -p base_frame:=base_link -p odom_frame:=odom -p minimum_travel_distance:=0.0 -p minimum_travel_heading:=0.0 -r __ns:=/robot1 -r /scan:=scan -r /odom:=odometry/filtered > /tmp/s_slam.log 2>&1 &
python3 -u $TOOLS/map_relay2.py > /tmp/s_relay.log 2>&1 &
sleep 15
pgrep -cf '[a]sync_slam' | sed 's/^/  slam=/'

echo "[9] TF check"
python3 - <<'PYEOF' > /tmp/s_tf.txt 2>&1
import rclpy, threading, time
from tf2_msgs.msg import TFMessage
res={'n':0,'pairs':set()}
def cb(m):
    res['n']+=1
    for t in m.transforms: res['pairs'].add(t.header.frame_id+'->'+t.child_frame_id)
rclpy.init(); nd=rclpy.create_node('tfc')
nd.create_subscription(TFMessage,'/tf',cb,10)
threading.Thread(target=rclpy.spin,args=(nd,),daemon=True).start()
time.sleep(12)
print('tf n=%d pairs=%s' % (res['n'], sorted(res['pairs'])[:8]))
PYEOF
cat /tmp/s_tf.txt | sed 's/^/  /'

echo "[10] robot pose (should settle ~0.23)"
timeout 40 ign topic -e -t /world/world_demo/pose/info -n 1 2>&1 | grep -A7 'name: .robot1_my_bot' | grep -E 'z:' | head -2 | sed 's/^/  /'

echo "[11] TROT"
python3 $TOOLS/mode_stick.py TROT > /tmp/s_trot.log 2>&1
sleep 20

echo "[12] explore 720s"
python3 -u $TOOLS/explore.py 720 > /tmp/s_explore.log 2>&1
echo "  explore rc=$?"
tail -3 /tmp/s_explore.log | sed 's/^/  /'

echo "[13] save map"
mkdir -p /tmp/mapsave
ros2 run nav2_map_server map_saver_cli -f /tmp/mapsave/cross_warehouse_$(date +%Y%m%d_%H%M) 2>&1 | tail -3 | sed 's/^/  /'
ls -la /tmp/mapsave/ | tail -4 | sed 's/^/  /'

echo "[14] stop robot"
python3 $TOOLS/drive_vel.py fwd_slow 0 > /dev/null 2>&1

echo "=== session DONE $(date) ==="