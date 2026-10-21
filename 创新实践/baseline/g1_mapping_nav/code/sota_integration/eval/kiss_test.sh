#!/bin/bash
# kiss_test.sh — KISS-ICP(纯雷达) + 静止/运动 + GT 对比
WS=/home/wy/go2_sim_ws/ROS2-Gazebo-GO2
KISS=/home/wy/kiss_ws
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$WS/src/docker/cyclonedds.xml
export IGN_PARTITION=go2wy
export GZ_PARTITION=go2wy
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
source $KISS/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
unset EGL_PLATFORM
LOG=/tmp/kiss_test.log
exec > >(tee -a $LOG) 2>&1
echo "=== kiss_test start $(date) ==="
TOOLS=$(dirname "$(find /tmp/dog2dog_push -path '*code/tools/spawn_robot1.launch.py' 2>/dev/null | head -1)")
[ -z "$TOOLS" ] && TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
EV=/home/wy/go2_sim_ws/sota_integration/eval
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"
for p in 'ign gazebo.*warehouse_cross' '[p]arameter_bridge' '[r]obot_controller_gazebo' '[c]md_vel_pub' '[r]obot_state_publisher' '[k]iss_icp' '[g]t_stamped' '[k]issrec' 'pc_bridge'; do pkill -9 -f "$p" 2>/dev/null; done; sleep 5

echo "[1] gazebo"; ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6 > /tmp/k_gz.log 2>&1 &
sleep 55; echo "  gz=$(pgrep -cf 'ign gazebo.*warehouse_cross')"
cat > /tmp/gtbridge.yaml <<'YAML'
- ros_topic_name: "/gt/pose"
  gz_topic_name: "/world/world_demo/pose/info"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "ignition.msgs.Pose_V"
  direction: GZ_TO_ROS
YAML
echo "[2] bridges"
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=$WS/install/gazebo_sim/share/gazebo_sim/config/gz_bridge.yaml -p use_sim_time:=True > /tmp/k_br1.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  /robot1/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  --ros-args -r __node:=ros_gz_bridge -r __ns:=/robot1 -p use_sim_time:=True > /tmp/k_br2.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge /robot1/velodyne/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked --ros-args -r __node:=pc_bridge > /tmp/k_br3.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=/tmp/gtbridge.yaml -p use_sim_time:=True > /tmp/k_br4.log 2>&1 &
sleep 8; echo "  bridges=$(pgrep -cf '[p]arameter_bridge')"
echo "[3] pause+spawn"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 60000 --req 'pause: true' >/dev/null 2>&1
ros2 launch $TOOLS/spawn_robot1.launch.py > /tmp/k_spawn.log 2>&1 &
sleep 45; tail -1 /tmp/k_spawn.log
echo "[4] spawners + control"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/k_jsb.log 2>&1 &
sleep 3
ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/k_jgc.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/robot_controller_gazebo.py --ros-args -r __node:=quadruped_controller -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/k_ctrl.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/k_cvp.log 2>&1 &
sleep 20
echo "[5] unpause"; ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 120000 --req 'pause: false'
echo "[6] wait jgc"; for i in $(seq 1 32); do grep -q 'Configured and activated' /tmp/k_jgc.log 2>/dev/null && { echo "  jgc ~$((i*15))s"; break; }; sleep 15; done
echo "[7] gt_stamped + KISS-ICP"
python3 -u $EV/gt_stamped.py --model robot1_my_bot --in_topic /gt/pose --out_topic /gt/pose_stamped --mode clock > /tmp/k_gtstamp.log 2>&1 &
ros2 run kiss_icp kiss_icp_node --ros-args -r pointcloud_topic:=/robot1/velodyne/points -p base_frame:=base_link -p publish_odom_tf:=false -p data.deskew:=false -p use_sim_time:=true > /tmp/k_kiss.log 2>&1 &
sleep 15
echo "  kiss topics:"; timeout 8 ros2 topic list 2>/dev/null | grep -iE 'kiss|odom' | head
echo "[8] TROT + record (static then motion)"
python3 - <<'PY' &
import rclpy, threading, time, csv, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
class R(Node):
    def __init__(s):
        super().__init__('kissrec')
        import os; os.makedirs('/tmp/kisstest', exist_ok=True)
        s.f=open('/tmp/kisstest/kiss.csv','w',newline=''); s.w=csv.writer(s.f)
        s.g=open('/tmp/kisstest/gt.csv','w',newline=''); s.wg=csv.writer(s.g)
        s.w.writerow(['t','x','y','z']); s.wg.writerow(['t','x','y','z'])
        s.create_subscription(Odometry,'/kiss/odometry',s.cb,50)
        s.create_subscription(PoseStamped,'/gt/pose_stamped',s.cg,50)
    def cb(s,m):
        p=m.pose.pose.position; s.w.writerow([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z]); s.f.flush()
    def cg(s,m):
        p=m.pose.position; s.wg.writerow([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z]); s.g.flush()
rclpy.init(); n=R(); threading.Thread(target=rclpy.spin,args=(n,),daemon=True).start(); time.sleep(95)
PY
REC=$!
sleep 5
python3 $TOOLS/mode_stick.py TROT > /tmp/k_trot.log 2>&1
echo "  static 20s"; sleep 20
echo "  motion"; python3 -u $TOOLS/drive_vel.py fwd_slow 15 > /dev/null 2>&1
python3 -u $TOOLS/drive_vel.py arc 15 > /dev/null 2>&1
python3 -u $TOOLS/drive_vel.py fwd_slow 12 > /dev/null 2>&1
echo "  driven"; sleep 8
kill -INT $REC 2>/dev/null; sleep 3
echo "[9] compare"
python3 - <<'PY'
import csv, math, numpy as np
def rd(p):
    return np.array([[float(v) for v in r] for r in csv.reader(open(p))][1:])
try:
    K=rd('/tmp/kisstest/kiss.csv'); G=rd('/tmp/kisstest/gt.csv')
except Exception as e:
    print("read err", e); raise SystemExit
print("kiss rows=%d gt rows=%d"%(len(K),len(G)))
if len(K)>5 and len(G)>5:
    t0=K[0,0]
    # 静止段(前20s)与运动段
    st=K[K[:,0]<t0+20]; mo=K[K[:,0]>=t0+20]
    print("STATIC(20s): max|pos|=%.3f m ; MOTION: kiss travel=%.2fm" % (
        max(np.hypot(st[:,1],st[:,2]).max() if len(st) else 0, 0),
        np.hypot(*(K[-1,1:3]-K[0,1:3]))))
    P=[];Q=[]
    for t,x,y,z in K:
        i=np.argmin(np.abs(G[:,0]-t))
        if abs(G[i,0]-t)<0.15: P.append([x,y]); Q.append([G[i,1],G[i,2]])
    P=np.array(P);Q=np.array(Q)
    if len(P)>10:
        # anchor by first sample
        t=Q[0]-P[0]; r=Q-(P+t); d=np.hypot(*(r.T))
        print("PAIRS=%d  RMSE=%.1fcm median=%.1fcm max=%.1fcm"%(len(P),np.sqrt((d**2).mean())*100,np.median(d)*100,d.max()*100))
        print("  kiss travel=%.2fm gt travel=%.2fm"%(np.hypot(*(P[-1]-P[0])),np.hypot(*(Q[-1]-Q[0]))))
PY
echo "=== kiss_test DONE $(date) ==="
