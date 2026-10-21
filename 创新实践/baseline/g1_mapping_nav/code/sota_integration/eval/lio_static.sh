#!/bin/bash
# lio_static.sh — 全栈 + Point-LIO + 短驱动 + GT，比较 LIO 里程计 vs Gazebo 真值。
WS=/home/wy/go2_sim_ws/ROS2-Gazebo-GO2
LIO=/home/wy/lio_ws
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$WS/src/docker/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
source $LIO/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
unset EGL_PLATFORM
LOG=/tmp/lio_static.log
exec > >(tee -a $LOG) 2>&1
echo "=== lio_static start $(date) ==="
TOOLS=$(dirname "$(find /tmp/dog2dog_push -path '*code/tools/spawn_robot1.launch.py' 2>/dev/null | head -1)")
[ -z "$TOOLS" ] && TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
FIXER=/home/wy/go2_sim_ws/sota_integration/lio/gz_lidar_fixer.py
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"
for p in '[i]gn gazebo' '[p]arameter_bridge' '[Q]uadrupedOdometry' '[e]kf_node' '[c]md_vel_pub' '[r]obot_controller_gazebo' '[a]sync_slam' '[m]ap_relay' '[r]obot_state_publisher' '[g]z_lidar_fixer' '[p]ointlio' '[r]os2 bag' '[s]lam_toolbox'; do pkill -9 -f "$p" 2>/dev/null; done; sleep 6

echo "[1] gazebo"; ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6 > /tmp/lt_gz.log 2>&1 &
sleep 55; echo "  gz=$(pgrep -cf '[i]gn gazebo -s')"
cat > /tmp/gtbridge.yaml <<'YAML'
- ros_topic_name: "/gt/pose"
  gz_topic_name: "/world/world_demo/pose/info"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "ignition.msgs.Pose_V"
  direction: GZ_TO_ROS
YAML
echo "[2] bridges"
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=$WS/install/gazebo_sim/share/gazebo_sim/config/gz_bridge.yaml -p use_sim_time:=True > /tmp/lt_br1.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge \
  /robot1/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /robot1/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  --ros-args -r __node:=ros_gz_bridge -r __ns:=/robot1 -p use_sim_time:=True > /tmp/lt_br2.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge /robot1/velodyne/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked --ros-args -r __node:=pc_bridge > /tmp/lt_br3.log 2>&1 &
/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:=/tmp/gtbridge.yaml -p use_sim_time:=True > /tmp/lt_br4.log 2>&1 &
sleep 8; echo "  bridges=$(pgrep -cf '[p]arameter_bridge')"

echo "[3] pause+spawn"
ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 60000 --req 'pause: true' >/dev/null 2>&1
ros2 launch $TOOLS/spawn_robot1.launch.py > /tmp/lt_spawn.log 2>&1 &
sleep 45; tail -1 /tmp/lt_spawn.log
echo "[4] spawners"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/lt_jsb.log 2>&1 &
sleep 3
ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 600 --switch-timeout 180 --ros-args $REMAP > /tmp/lt_jgc.log 2>&1 &
echo "[5] control nodes"
python3 $WS/install/quadropted_controller/lib/quadropted_controller/robot_controller_gazebo.py --ros-args -r __node:=quadruped_controller -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/lt_ctrl.log 2>&1 &
python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP > /tmp/lt_cvp.log 2>&1 &
echo "[6] fixer + point_lio"
python3 -u $FIXER --ros-args -p in_topic:=/robot1/velodyne/points -p out_topic:=/robot1/velodyne/fixed > /tmp/lt_fix.log 2>&1 &
ros2 run point_lio pointlio_mapping --ros-args --params-file /tmp/plio_gz.yaml -p use_imu_as_input:=False -p prop_at_freq_of_imu:=True -p check_satu:=True -p init_map_size:=10 -p point_filter_num:=4 -p space_down_sample:=True -p filter_size_surf:=0.5 -p filter_size_map:=0.5 -p cube_side_length:=1000.0 -p runtime_pos_log_enable:=False > /tmp/lt_plio.log 2>&1 &
sleep 20
echo "[7] unpause"; ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 120000 --req 'pause: false'
sleep 5
echo "  cloud_hz:"; timeout 8 ros2 topic hz /cloud_registered 2>&1 | head -2

echo "[8] wait jgc"; for i in $(seq 1 32); do grep -q 'Configured and activated' /tmp/lt_jgc.log 2>/dev/null && { echo "  jgc ~$((i*15))s"; break; }; sleep 15; done

echo "[9] record"
rm -rf /tmp/liotest; mkdir -p /tmp/liotest
python3 - <<'PY' &
import rclpy, threading, time, csv
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage
import math
class L(Node):
    def __init__(s):
        super().__init__('liorec'); s.f=open('/tmp/liotest/lio.csv','w',newline=''); s.w=csv.writer(s.f)
        s.g=open('/tmp/liotest/gt.csv','w',newline=''); s.wg=csv.writer(s.g)
        s.w.writerow(['t','x','y','z','yaw']); s.wg.writerow(['t','x','y','z','yaw'])
        s.create_subscription(Odometry,'/aft_mapped_to_init',s.cb,50)
        s.create_subscription(TFMessage,'/gt/pose',s.cg,50)
    def cb(s,m):
        q=m.pose.pose.orientation; p=m.pose.pose.position
        s.w.writerow([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]); s.f.flush()
    def cg(s,m):
        for tr in m.transforms:
            nm=tr.child_frame_id.lstrip('/')
            if nm=='robot1_my_bot' or nm.endswith('/robot1_my_bot'):
                p=tr.transform.translation; q=tr.transform.rotation
                s.wg.writerow([tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]); s.g.flush()
rclpy.init(); n=L(); threading.Thread(target=rclpy.spin,args=(n,),daemon=True).start(); time.sleep(55)
PY
REC=$!
sleep 8
echo "[10] TROT + STATIC (no drive)"
python3 $TOOLS/mode_stick.py TROT > /tmp/lt_trot.log 2>&1
echo "  standing still 40s"; sleep 40
kill -INT $REC 2>/dev/null; sleep 3
echo "[11] compare"
python3 - <<'PY'
import csv, math, numpy as np
def rd(p):
    r=[]
    for row in csv.reader(open(p)):
        try: r.append([float(v) for v in row])
        except: pass
    return np.array(r)
L=rd('/tmp/liotest/lio.csv'); G=rd('/tmp/liotest/gt.csv')
print("lio rows=%d gt rows=%d"%(len(L),len(G)))
if len(L)>10 and len(G)>10:
    # 依时间就近配对
    P=[];Q=[]
    for t,x,y,z,yw in L[:,:5]:
        i=np.argmin(np.abs(G[:,0]-t))
        if abs(G[i,0]-t)<0.1: P.append([x,y]); Q.append([G[i,1],G[i,2]])
    P=np.array(P);Q=np.array(Q)
    if len(P)>10:
        de=np.array([L[j,4]-G[np.argmin(np.abs(G[:,0]-L[j,0])),4] for j in range(len(L))][:200])
        yaw=math.degrees(math.atan2(np.mean(np.sin(np.radians(de))),np.mean(np.cos(np.radians(de)))))
        R=np.array([[math.cos(math.radians(yaw)),-math.sin(math.radians(yaw))],[math.sin(math.radians(yaw)),math.cos(math.radians(yaw))]])
        t=Q.mean(0)-R@P.mean(0)
        resid=Q-(P@R.T+t); dr=np.hypot(*(resid.T))
        print("PAIRS=%d yaw=%.2fdeg |t|=%.2f  RMSE=%.1fcm median=%.1fcm max=%.1fcm"%(
            len(P),yaw,np.hypot(*t),np.sqrt((dr**2).mean())*100,np.median(dr)*100,dr.max()*100))
        print("  LIO travel=%.2fm  GT travel=%.2fm"%(
            np.hypot(*(L[-1,:2]-L[0,:2])), np.hypot(*(G[-1,1:3]-G[0,1:3]))))
PY
echo "=== lio_static DONE $(date) ==="
