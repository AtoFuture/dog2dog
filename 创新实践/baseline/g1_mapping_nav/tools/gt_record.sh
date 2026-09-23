#!/bin/bash
# 录一段「真值 + odom + EKF」的 rosbag，用于 G1 P1 的可信运动量化。
#
# ── 为什么要起两个中转 ──
# 1) 真值只在 gz-transport 的 /world/world_demo/pose/info 上，不在 launch 的
#    桥接表里（gz_bridge.yaml 只有 clock），SDF 里也没有真值 plugin。
#    所以用 ros_gz_bridge 把它桥成 tf2_msgs/msg/TFMessage。
# 2) 但该桥**不填 header.stamp**（实测全 0），时间基对不上 odom，直接拿去做
#    「到点误差」会得到错位样本相减的假数。所以再过一道 gt_relay.py，
#    用仿真时钟重新打戳成 nav_msgs/msg/Odometry。
#
# ⚠️ 桥走 gz-transport，必须与仿真同一个 IGN_PARTITION（默认 wy），
#    否则读到别人的狗或读不到。见 run_sim_iso.sh 的三层隔离说明。
#
# 用法（在外层用 g1ros.sh 进容器，才有 ROS + 隔离域环境）：
#   ./g1ros.sh bash tools/gt_record.sh 120 /tmp/g1_p1/bag_static
#
# 参数：
#   $1 时长（秒）
#   $2 输出目录（rosbag2）
#   $3.. 额外话题（可选）

set -uo pipefail

DURATION="${1:?用法: gt_record.sh <时长秒> <输出目录> [额外话题...]}"
OUT="${2:?缺少输出目录}"
shift 2
EXTRA_TOPICS=("$@")

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GT_RAW_TOPIC="/world/world_demo/pose/info"
GT_TOPIC="/g1/ground_truth"
BASE_TOPICS=(
    "${GT_TOPIC}"
    /robot1/odom
    /robot1/odometry/filtered
    /clock
)

mkdir -p "$(dirname "${OUT}")"
LOG_DIR="$(dirname "${OUT}")"

# 自清理：脚本被 kill -9 时 trap 不执行，会漏下桥/中继进程；
# 多个同名中继会互相抢发布器，报 "publisher's context is invalid"。
pkill -9 -f "gt_relay.py" 2>/dev/null
pkill -9 -f "__node:=gt_bridge" 2>/dev/null
sleep 1

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        kill "${pid}" 2>/dev/null
        wait "${pid}" 2>/dev/null
    done
}
trap cleanup EXIT

echo "== 起真值桥（IGN_PARTITION=${IGN_PARTITION:-未设}）=="
ros2 run ros_gz_bridge parameter_bridge \
    "${GT_RAW_TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
    --ros-args -r __node:=gt_bridge >"${LOG_DIR}/gt_bridge.log" 2>&1 &
PIDS+=($!)

for _ in $(seq 1 20); do
    ros2 topic list 2>/dev/null | grep -qx "${GT_RAW_TOPIC}" && break
    sleep 0.5
done
if ! ros2 topic list 2>/dev/null | grep -qx "${GT_RAW_TOPIC}"; then
    echo "ERROR: 桥没挂上真值话题，看 ${LOG_DIR}/gt_bridge.log" >&2
    exit 1
fi

echo "== 起真值中转（打仿真时间戳）=="
python3 "${TOOLS_DIR}/gt_relay.py" --ros-args \
    -p use_sim_time:=true >"${LOG_DIR}/gt_relay.log" 2>&1 &
PIDS+=($!)

for _ in $(seq 1 20); do
    ros2 topic list 2>/dev/null | grep -qx "${GT_TOPIC}" && break
    sleep 0.5
done
if ! ros2 topic list 2>/dev/null | grep -qx "${GT_TOPIC}"; then
    echo "ERROR: 中转没挂上 ${GT_TOPIC}，看 ${LOG_DIR}/gt_relay.log" >&2
    exit 1
fi

# 确认真值真的在流（桥挂了不等于有数据）
echo "== 抽检真值是否有数据 =="
if timeout 15 ros2 topic echo "${GT_TOPIC}" --once >/dev/null 2>&1; then
    echo "   真值在流 ✓"
else
    echo "ERROR: ${GT_TOPIC} 无数据；检查 IGN_PARTITION 是否与仿真一致" >&2
    exit 1
fi

echo "== 录包 ${DURATION}s → ${OUT} =="
echo "   话题：${BASE_TOPICS[*]} ${EXTRA_TOPICS[*]:-}"
# 用 SIGINT 让 ros2 bag record 正常收尾（写 metadata）；124 = 按时截断，属正常。
#
# --include-hidden-topics 是必需的：Nav2 的 action 话题是 `/robot1/navigate_to_pose/_action/status`
# 这种下划线开头的**隐藏话题**，不加这个开关 rosbag2 会直接跳过它们
# （只在日志里留一句 "Hidden topics are not recorded"），
# 于是 P1 要求的「rosbag 含 Nav2 goal/result」就永远录不到。
timeout -s INT "${DURATION}" ros2 bag record --include-hidden-topics -o "${OUT}" \
    "${BASE_TOPICS[@]}" "${EXTRA_TOPICS[@]}"
RC=$?
echo "== 录包结束（rc=${RC}；124 = 到时正常截断）=="
ls -la "${OUT}" 2>/dev/null | head -6
