#!/bin/bash
# 发一个 Nav2 目标点，录包，并按**真值**判定是否真的到了。
#
# 为什么不看 action 的 SUCCEEDED：P1 的出口标准明确要求「不得再仅以 action 的
# SUCCEEDED 判定真实到达」。历史现象正是 action 报成功而车没到。
#
# 前置：
#   · 隔离仿真在跑（run_sim_iso.sh）
#   · Nav2 已起（start_nav2_isolated.sh，含控制器激活与速度缩放）
#   · AMCL 已给初始位姿（否则 map→odom 不出，规划直接失败）
#
# 用法（在外层用 g1ros.sh 进容器）：
#   ./g1ros.sh bash tools/goal_test.sh <x> <y> <yaw> <超时秒> <输出目录>
#
# 例：
#   ./g1ros.sh bash tools/goal_test.sh 10.5 0.63 0.0 90 /tmp/g1_p1/goal_1

set -uo pipefail

GOAL_X="${1:?用法: goal_test.sh <x> <y> <yaw> <超时秒> <输出目录>}"
GOAL_Y="${2:?缺少 y}"
GOAL_YAW="${3:?缺少 yaw}"
TIMEOUT_S="${4:?缺少超时秒数}"
OUT="${5:?缺少输出目录}"

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(dirname "${OUT}")"
mkdir -p "${LOG_DIR}"

GT_TOPIC="/g1/ground_truth"
ACTION="/robot1/navigate_to_pose"

echo "== 目标 ($GOAL_X, $GOAL_Y, yaw=$GOAL_YAW)，超时 ${TIMEOUT_S}s =="

# 1) 后台录包：真值 + AMCL + 规划 + 速度 + action 状态/反馈
RECORD_SECS=$((TIMEOUT_S + 20))
# 话题清单对齐 P1 出口标准「rosbag 中同时包含地图、TF、Nav2 goal/result
# 和 ground truth」——真值由 gt_record.sh 负责，这里补齐地图与 TF。
bash "${TOOLS_DIR}/gt_record.sh" "${RECORD_SECS}" "${OUT}" \
    /robot1/map \
    /robot1/tf \
    /robot1/tf_static \
    /robot1/amcl_pose \
    /robot1/cmd_vel \
    /robot1/cmd_vel_scaled_out \
    "${ACTION}/_action/status" \
    "${ACTION}/_action/feedback" \
    >"${LOG_DIR}/goal_record.log" 2>&1 &
REC_PID=$!

# 等真值桥与中转挂上
for _ in $(seq 1 30); do
    ros2 topic list 2>/dev/null | grep -qx "${GT_TOPIC}" && break
    sleep 1
done
sleep 4

if ! kill -0 "${REC_PID}" 2>/dev/null; then
    echo "ERROR: 录包进程已退出，看 ${LOG_DIR}/goal_record.log" >&2
    exit 1
fi

# 2) 发目标
echo "== 发送 NavigateToPose =="
timeout "${TIMEOUT_S}" ros2 action send_goal "${ACTION}" \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: $(python3 -c "import math;print(math.sin(${GOAL_YAW}/2))"), \
w: $(python3 -c "import math;print(math.cos(${GOAL_YAW}/2))")}}}}" \
    >"${LOG_DIR}/goal_action.log" 2>&1
ACTION_RC=$?
echo "   action 返回码 ${ACTION_RC}（124 = 超时未结束）"

# 3) action 结束的即刻真值（这是判「真到了没有」的依据）
echo "== 结束时刻真值 =="
timeout 15 ros2 topic echo "${GT_TOPIC}" --once 2>/dev/null \
    | grep -E "^    x:|^    y:" | head -2 | tee "${LOG_DIR}/goal_end_gt.txt"

# 4) 收尾
kill -INT "${REC_PID}" 2>/dev/null
wait "${REC_PID}" 2>/dev/null

echo "== action 结果摘要 =="
grep -E "status:|result:|Goal finished" "${LOG_DIR}/goal_action.log" | tail -5
