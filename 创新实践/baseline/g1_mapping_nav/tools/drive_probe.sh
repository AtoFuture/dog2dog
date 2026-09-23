#!/bin/bash
# 定速驱动探针：一边录「真值 + odom + EKF」，一边往 /robot1/cmd_vel 发定速指令。
#
# 用途：暴露足端里程计的「幻觉」——QuadrupedOdometryNode 在**没有任何足端接地**
# 时会退化成用指令速度积分（`delta_x = self.linear_velocity_x * dt`），
# 也就是「我让它走，它就报我走了」。静止测试测不出这个分支，
# 必须发指令、再拿真值对，才能看出 odom 报的位移和物理位移差多少。
#
# 指令链路：/robot1/cmd_vel → cmd_vel_pub.py → robot_velocity → 控制器
#
# 用法（在外层用 g1ros.sh 进容器）：
#   ./g1ros.sh bash tools/drive_probe.sh 40 /tmp/g1_p1/bag_drive 0.2 0.0
#
# 参数：
#   $1 时长（秒）
#   $2 输出目录（rosbag2）
#   $3 线速度 lx（m/s，默认 0.2）
#   $4 角速度 az（rad/s，默认 0.0）
#
# ⚠️ 只在隔离域里用（Domain 43）。别在共享域上发速度指令。

set -uo pipefail

DURATION="${1:?用法: drive_probe.sh <时长秒> <输出目录> [lx] [az]}"
OUT="${2:?缺少输出目录}"
LX="${3:-0.2}"
AZ="${4:-0.0}"

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(dirname "${OUT}")"
mkdir -p "${LOG_DIR}"

echo "== 驱动探针：lx=${LX} az=${AZ}，共 ${DURATION}s =="

# 录包在后台跑（gt_record.sh 自带真值桥与中转）。
# 额外录 foot_contact：odom 的「有没有脚接地」分支靠它，是判断
# 幻觉来自「按指令积分」还是「足端分支自身算错」的关键证据。
bash "${TOOLS_DIR}/gt_record.sh" "${DURATION}" "${OUT}" \
    /robot1/foot_contact /robot1/joint_group_controller/commands \
    /robot1/robot_velocity \
    >"${LOG_DIR}/drive_record.log" 2>&1 &
REC_PID=$!

# 给真值桥/中转留出挂载时间，再开始发指令
sleep 12

if ! kill -0 "${REC_PID}" 2>/dev/null; then
    echo "ERROR: 录包进程已退出，看 ${LOG_DIR}/drive_record.log" >&2
    exit 1
fi

echo "== 发指令 ${DURATION}s（20 Hz）=="
timeout -s INT "$((DURATION - 8))" ros2 topic pub -r 20 /robot1/cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: ${LX}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${AZ}}}" \
    >"${LOG_DIR}/drive_cmd.log" 2>&1
echo "   指令已停"

# 先发一条零速再收尾，别把速度留在总线上
timeout 8 ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1
echo "   已发零速"

wait "${REC_PID}" 2>/dev/null
echo "== 探针结束 =="
