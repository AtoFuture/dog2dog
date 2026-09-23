#!/bin/bash
# 在隔离域里起「我们这份」Nav2：先保证控制器真被激活，再起速度缩放器，再起 Nav2。
#
# 为什么要有这个脚本（每一步都对应一个踩过的坑）：
#
# 1) 激活 joint_group_controller
#    实测它可能停在 `unconfigured`：此时仿真看着全好（/clock 在走、节点齐全、
#    commands 话题发布者数=1、quadruped_controller 照发命令），
#    但狗一动不动（真值 z 恒在生成高度 0.446 而非站立 0.27）。
#    configure_controller 可能返回 ok=False，用 switch_controller 能激活。
#
# 2) 起 cmd_vel_scaler
#    步态对 rv 的增益约 10 倍，而 Nav2 按配置发 0.26 m/s —— 直接接上会跑出
#    ~2.7 m/s、rv 顶到 0.27（稳定边界是 0.04）。缩放器把 Nav2 的输出压回安全区。
#    见 cmd_vel_scaler.py 顶部的实测表。
#
# 3) 用我们自己的 launch
#    它把 Nav2 的速度输出挪到 cmd_vel_scaled_out，仿真自带的 cmd_vel_pub
#    一行不动（共享代码不碰）。
#
# 用法（在外层用 g1ros.sh 进容器）：
#   ./g1ros.sh bash tools/start_nav2_isolated.sh [地图yaml]
#
# 默认地图：warehouse_map.yaml

set -uo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP="${1:-/home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/navigation2/maps/warehouse_map.yaml}"
LOG_DIR="${G1_LOG_DIR:-/tmp/g1_p1}"
mkdir -p "${LOG_DIR}"

CTRL_MGR="/robot1/controller_manager"

echo "== 1/3 确认 joint_group_controller 已激活 =="
# 注意别用 `tr -d "state='"`：tr 删的是字符集，会把 active 里的 a/t/e 也删掉。
query_controller_state() {
    timeout 20 ros2 service call "${CTRL_MGR}/list_controllers" \
        controller_manager_msgs/srv/ListControllers 2>/dev/null \
        | grep -oE "name='joint_group_controller', state='[a-z]+'" \
        | sed -E "s/.*state='([a-z]+)'.*/\1/"
}

STATE="$(query_controller_state)"
echo "   当前状态：${STATE:-查不到}"

if [ "${STATE}" != "active" ]; then
    echo "   未激活 → 调 switch_controller"
    timeout 30 ros2 service call "${CTRL_MGR}/switch_controller" \
        controller_manager_msgs/srv/SwitchController \
        "{activate_controllers: [joint_group_controller], strictness: 1}" \
        >"${LOG_DIR}/activate_controller.log" 2>&1
    STATE="$(query_controller_state)"
    echo "   激活后状态：${STATE:-查不到}"
    if [ "${STATE}" != "active" ]; then
        echo "ERROR: joint_group_controller 仍未激活，狗不会动。停在这里。" >&2
        exit 1
    fi
fi

echo "== 2/3 起速度缩放器 =="
python3 "${TOOLS_DIR}/cmd_vel_scaler.py" --ros-args \
    -p use_sim_time:=true >"${LOG_DIR}/cmd_vel_scaler.log" 2>&1 &
SCALER_PID=$!
echo "   pid=${SCALER_PID}"

echo "== 3/3 起 Nav2（我们自己那份 launch）=="
echo "   地图：${MAP}"
ros2 launch "${TOOLS_DIR}/nav2_scaled.launch.py" "map:=${MAP}" \
    >"${LOG_DIR}/nav2_scaled.log" 2>&1
RC=$?

kill "${SCALER_PID}" 2>/dev/null
wait "${SCALER_PID}" 2>/dev/null
echo "== Nav2 退出（rc=${RC}）=="
