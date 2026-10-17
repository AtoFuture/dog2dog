#!/bin/bash
# explore_nav2.sh — 全栈 + Nav2 + m-explore 前沿自主探索（真正覆盖整间）
WS=/home/wy/go2_sim_ws/ROS2-Gazebo-GO2
MEXP=/home/wy/go2_sim_ws/mexplore_ws
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$WS/src/docker/cyclonedds.xml
export IGN_PARTITION=go2wy
export GZ_PARTITION=go2wy
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
source $MEXP/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
unset EGL_PLATFORM
DUR="${1:-1000}"
LOG=/tmp/explore_nav2.log
exec > >(tee -a $LOG) 2>&1
echo "=== explore_nav2 start $(date) dur=$DUR ==="
TOOLS=$(dirname "$(find /tmp/dog2dog_push -path '*code/tools/spawn_robot1.launch.py' 2>/dev/null | head -1)")
[ -z "$TOOLS" ] && TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
NAV2PARAM=$WS/install/navigation2/share/navigation2/param/go2_nav2.yaml
EXPARAM=/home/wy/go2_sim_ws/sota_integration/explore/config/explore_params.yaml
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"
for p in 'ign gazebo.*warehouse_cross' '[p]arameter_bridge' '[Q]uadrupedOdometry' '[e]kf_node' '[c]md_vel_pub' '[r]obot_controller_gazebo' '[a]sync_slam' '[m]ap_relay' '[r]obot_state_publisher' '[c]ontroller_server' '[p]lanner_server' '[b]t_navigator' '[b]ehavior_server' '[s]moother_server' '[w]aypoint_follower' '[v]elocity_smoother' '[e]xplore_lite'; do pkill -9 -f "$p" 2>/dev/null; done; sleep 5

echo "[1] gazebo"; ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6 > /tmp/en_gz.log 2>&1 &
sleep 55; echo "  gz=$(pgrep -cf 'ign gazebo.*warehouse_cross')"
echo "[2] bridges"
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=$WS/install/gazebo_sim/share/gazebo_sim/config/gz_bridge.yaml -p use_sim_time:=True > /tmp/en_br1.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  /robot1/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /robot1/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  --ros-args -r __node:=ros_gz_bridge -r __ns:=/robot1 -p use_sim_time:=True > /tmp/en_br2.log 2>&1 &
sleep 8; echo "  bridges=$(pgrep -cf '[p]arameter_bridge')"
echo "[3] pause+spawn"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 60000 --req 'pause: true' >/dev/null 2>&1
ros2 launch $TOOLS/spawn_robot1.launch.py > /tmp/en_spawn.log 2>&1 &
sleep 45; tail -1 /tmp/en_spawn.log
echo "[4] spawners + control + odom/ekf"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/en_jsb.log 2>&1 &
sleep 3
ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/en_jgc.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/robot_controller_gazebo.py --ros-args -r __node:=quadruped_controller -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/en_ctrl.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/QuadrupedOdometryNode.py --ros-args -r __node:=odom -r __ns:=/robot1 -p use_sim_time:=True -p base_frame_id:=base_link -p odom_frame_id:=odom -p enable_odom_tf:=false $REMAP > /tmp/en_odom.log 2>&1 &
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node -r __ns:=/robot1 -p use_sim_time:=True --params-file $WS/install/gazebo_sim/share/gazebo_sim/config/ekf.yaml $REMAP > /tmp/en_ekf.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/en_cvp.log 2>&1 &
sleep 22
echo "[5] unpause"; ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 120000 --req 'pause: false'
echo "[6] slam+relay"
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml -p use_sim_time:=true -p scan_topic:=scan -p map_frame:=map -p base_frame:=base_link -p odom_frame:=odom -r __ns:=/robot1 -r /scan:=scan -r /odom:=odometry/filtered > /tmp/en_slam.log 2>&1 &
python3 -u $TOOLS/map_relay2.py > /tmp/en_relay.log 2>&1 &
sleep 12
echo "[7] wait jgc"; for i in $(seq 1 32); do grep -q 'Configured and activated' /tmp/en_jgc.log 2>/dev/null && { echo "  jgc ~$((i*15))s"; break; }; sleep 15; done
echo "[8] Nav2 (navigation_launch)"
ros2 launch nav2_bringup navigation_launch.py namespace:=robot1 use_sim_time:=true params_file:=$NAV2PARAM > /tmp/en_nav2.log 2>&1 &
sleep 60
echo "  -- nav2 nodes --"; timeout 20 ros2 node list 2>/dev/null | grep -cE 'controller_server|planner_server|bt_navigator|behavior_server' 
echo "  -- action --"; timeout 20 ros2 action list 2>/dev/null | grep navigate_to_pose
echo "[9] explore_lite"
ros2 run explore_lite explore --ros-args -r __ns:=/robot1 --params-file $EXPARAM > /tmp/en_explore.log 2>&1 &
sleep 30
echo "  -- explore topics --"; timeout 20 ros2 topic list 2>/dev/null | grep -E 'explore|frontier' | head
echo "  -- explore log --"; tail -6 /tmp/en_explore.log | cut -c1-140
echo "[10] monitor ${DUR}s"
python3 - <<PY &
import rclpy, threading, time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
res={}
class M(Node):
    def __init__(s):
        super().__init__('covmon')
        q=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        s.create_subscription(OccupancyGrid,'/robot1/map',s.cb,q); s.last=None
    def cb(s,m):
        d=list(m.data); tot=len(d); known=sum(1 for v in d if v>=0)
        s.last=100.0*known/tot
rclpy.init(); n=M(); threading.Thread(target=rclpy.spin,args=(n,),daemon=True).start()
t0=time.time()
while time.time()-t0 < $DUR:
    time.sleep(30)
    print("  t=%ds known=%.1f%%"%(int(time.time()-t0), n.last or 0), flush=True)
import os; os._exit(0)
PY
MON=$!
wait $MON
echo "[11] save map"
mkdir -p /tmp/explmap
ros2 run nav2_map_server map_saver_cli -f /tmp/explmap/explore_$(date +%H%M%S) 2>&1 | tail -2
ls -la /tmp/explmap/ | tail -4
echo "=== explore_nav2 DONE $(date) ==="
