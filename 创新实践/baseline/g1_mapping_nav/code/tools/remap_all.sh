#!/bin/bash
# ============================================================
# Go2 自动重建成图脚本（服务器恢复后一键执行）
# 用法: bash /tmp/remap_all.sh <运行分钟数>
# 默认 12 分钟
# ============================================================
MIN="${1:-12}"
CORE="export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
export CYCLONEDDS_URI=file:///home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml; \
source /opt/ros/humble/setup.bash; \
source /home/wy/go2_sim_ws/ROS2-Gazebo-GO2/install/setup.bash; \
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:\$GZ_SIM_SYSTEM_PLUGIN_PATH; \
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:\$IGN_GAZEBO_SYSTEM_PLUGIN_PATH"

# ---- 容器自适应：GPU 修复后用 ros-gui（RTF 高）；否则 ros + 软渲染兜底 ----
LOG=/tmp/remap_all.log
echo "=== remap start $(date) ===" > $LOG
if timeout 60 ~/.enroot/ros-gui bash -lc 'true' >/dev/null 2>&1; then
  CONT="~/.enroot/ros-gui"
  BASE="$CORE"
  RENDER_MODE="GPU (ros-gui, RTF expected >1.0)"
else
  CONT="~/.enroot/ros"
  BASE="$CORE; export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3; unset EGL_PLATFORM"
  RENDER_MODE="SOFTWARE (ros+Xvfb, RTF ~0.5)"
fi
echo "[MODE] $RENDER_MODE" | tee -a $LOG
REMAP="-r __ns:=/robot1 -p use_sim_time:=True -r /scan:=scan -r /odom:=odometry/filtered"
# NOTE: 不要给 /tf、/tf_static 加命名空间重映射——TF 必须是全局话题（Nav2/tf2_echo 只读 /tf）
WS="/home/wy/go2_sim_ws/ROS2-Gazebo-GO2"
TOOLS="/tmp/dog2dog_push/创新实践/baseline/g1_mapping_nav/code/tools"
R() { $CONT bash -lc "$BASE; $1"; }
LOG=/tmp/remap_all.log
echo "=== remap start $(date) ===" > $LOG

# ---- 0a. 自愈：/tmp 被清空则重新克隆工具（仓库公开，无需 token）----
if [ ! -d "$TOOLS" ]; then
  echo "[FIX] tools 目录丢失，重新克隆仓库..." | tee -a $LOG
  rm -rf /tmp/dog2dog_push 2>/dev/null
  git clone -q --depth 1 -b feat/go2-cross-warehouse-code \
      https://github.com/AtoFuture/dog2dog.git /tmp/dog2dog_push 2>>$LOG
fi
if [ ! -d "$TOOLS" ]; then
  echo "[FATAL] 工具不可用：$TOOLS（需先 git clone 仓库）" | tee -a $LOG
  exit 1
fi
echo "[FIX] tools OK: $(ls $TOOLS/*.py $TOOLS/*.launch.py 2>/dev/null | wc -l) py + $(ls $TOOLS/*.sh 2>/dev/null | wc -l) sh" | tee -a $LOG

step() { echo "[STEP] $1" | tee -a $LOG; }

# ---- 0. Xvfb（软渲染必需）----
step "Xvfb"
pgrep -cf Xvfb >/dev/null || { nohup Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset >/tmp/xvfb99.log 2>&1 & sleep 5; }
echo "  xvfb=$(pgrep -cf Xvfb)" | tee -a $LOG

# ---- 1. 若 Gazebo 未活则起 ----
step "check gazebo"
if [ "$(pgrep -cf 'ign gazebo -s')" != "1" ]; then
  step "start gazebo"
  nohup $CONT bash -lc "$BASE; exec ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross.sdf --force-version 6" > /tmp/gz_auto.log 2>&1 &
  sleep 70
  SEGV=$(grep -ciE segmentation /tmp/gz_auto.log)
  echo "  alive=$(pgrep -cf 'ign gazebo -s') segv=$SEGV" | tee -a $LOG
  [ "$SEGV" != "0" ] && { echo "  FATAL: gazebo crashed" | tee -a $LOG; exit 1; }
fi

# ---- 2. 暂停→spawn（若无机器人）----
step "check robot"
HASROBOT=$(R "timeout 30 ign topic -e -t /world/world_demo/pose/info -n 1 2>/dev/null | grep -c robot1_my_bot")
echo "  robot_entities=$HASROBOT" | tee -a $LOG
if [ "$HASROBOT" = "0" ]; then
  step "spawn (pause->create)"
  R "timeout 25 ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 10000 --req \"pause: true\"" >/dev/null 2>&1
  nohup $CONT bash -lc "$BASE; exec ros2 launch $TOOLS/spawn_robot1.launch.py" > /tmp/spawn_auto.log 2>&1 &
  sleep 55
  tail -1 /tmp/spawn_auto.log | cut -c1-120 | tee -a $LOG
  step "spawners"
  nohup $CONT bash -lc "$BASE; exec ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /robot1/controller_manager --controller-manager-timeout 120 --switch-timeout 60 --ros-args $REMAP" > /tmp/sp_jsb_auto.log 2>&1 &
  sleep 5
  nohup $CONT bash -lc "$BASE; exec ros2 run controller_manager spawner joint_group_controller --controller-manager /robot1/controller_manager --controller-manager-timeout 120 --switch-timeout 60 --ros-args $REMAP" > /tmp/sp_jgc_auto.log 2>&1 &
  sleep 80
  grep -c 'Configured and activated' /tmp/sp_jsb_auto.log /tmp/sp_jgc_auto.log | tee -a $LOG
fi

# ---- 3. 解除暂停 ----
step "unpause"
R "timeout 40 ign service -s /world/world_demo/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 30000 --req \"pause: false\"" >/dev/null 2>&1
sleep 10

# ---- 4. 检查里程计；死了就重启 odom+ekf ----
step "odom check"
cat > /tmp/_qodom.py <<'PYEOF'
import rclpy, threading, time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
res = {}
def mk(n):
    def cb(m): res[n] = res.get(n, 0) + 1
    return cb
rclpy.init()
nd = rclpy.create_node('qodom')
nd.create_subscription(Odometry, '/robot1/odom', mk('raw_odom'), 10)
nd.create_subscription(Odometry, '/robot1/odometry/filtered', mk('ekf_odom'), 10)
nd.create_subscription(Float64MultiArray, '/robot1/joint_group_controller/commands', mk('cmd'), 10)
threading.Thread(target=rclpy.spin, args=(nd,), daemon=True).start()
time.sleep(15)
open('/tmp/qodom.txt', 'w').write(repr(res))
PYEOF
rm -f /tmp/qodom.txt
nohup $CONT bash -lc "$BASE; exec python3 -u /tmp/_qodom.py" >/dev/null 2>&1 &
sleep 25
echo "  before: $(cat /tmp/qodom.txt 2>/dev/null)" | tee -a $LOG
if ! grep -q "raw_odom.: [1-9]" /tmp/qodom.txt 2>/dev/null; then
  step "RESTART odom+ekf+cvp (odom dead)"
  pkill -9 -f QuadrupedOdometryNode; pkill -9 -f ekf_node; pkill -9 -f cmd_vel_pub; sleep 4
  nohup $CONT bash -lc "$BASE; exec python3 $WS/install/quadropted_controller/lib/quadropted_controller/QuadrupedOdometryNode.py --ros-args -r __node:=odom -r __ns:=/robot1 -p use_sim_time:=True -p base_frame_id:=base_link -p odom_frame_id:=odom -p enable_odom_tf:=false $REMAP" > /tmp/odom2.log 2>&1 &
  nohup $CONT bash -lc "$BASE; exec ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node -r __ns:=/robot1 -p use_sim_time:=True --params-file $WS/install/gazebo_sim/share/gazebo_sim/config/ekf.yaml $REMAP" > /tmp/ekf2.log 2>&1 &
  nohup $CONT bash -lc "$BASE; exec python3 $WS/install/quadropted_controller/lib/quadropted_controller/cmd_vel_pub.py --ros-args -r __node:=cmd_vel_pub -r __ns:=/robot1 -p use_sim_time:=True $REMAP" > /tmp/cvp2.log 2>&1 &
  sleep 30
  rm -f /tmp/qodom.txt
  nohup $CONT bash -lc "$BASE; exec python3 -u /tmp/_qodom.py" >/dev/null 2>&1 &
  sleep 25
  echo "  after: $(cat /tmp/qodom.txt 2>/dev/null)" | tee -a $LOG
fi

# ---- 5. TROT 模式 ----
step "TROT"
nohup $CONT bash -lc "$BASE; exec python3 -u $TOOLS/mode_stick.py TROT" > /tmp/trot.log 2>&1 &
sleep 45

# ---- 6. 跑 explore ----
step "explore ${MIN}min"
nohup $CONT bash -lc "$BASE; exec python3 -u $TOOLS/explore.py $((MIN * 60))" > /tmp/explore.log 2>&1 &
EXPLORE_PID=$!
# 等 explore 结束
while pgrep -f "explore.py" >/dev/null; do sleep 30; done
step "explore finished"

# ---- 7. 存图 ----
step "save map"
mkdir -p /tmp/mapsave
R "timeout 150 ros2 run nav2_map_server map_saver_cli -f /tmp/mapsave/cross_warehouse_$(date +%Y%m%d_%H%M) 2>&1 | tail -3"
ls -la /tmp/mapsave/ | tee -a $LOG

# ---- 8. 收尾：站立静止 ----
step "final stop"
nohup $CONT bash -lc "$BASE; exec python3 -u $TOOLS/drive_vel.py fwd_slow 0" >/dev/null 2>&1 &
sleep 5
step "DONE"
echo "=== remap end $(date) ===" >> $LOG