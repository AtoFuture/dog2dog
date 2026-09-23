#!/bin/bash
# 用**真值**给 AMCL 设初始位姿。
#
# ── 为什么需要这个 ──
# 1) AMCL 的配置里没有 set_initial_pose/initial_pose 任何一项，必须外部发
#    `/robot1/initialpose`，否则它不出 map→odom，Nav2 直接无法规划。
#    历史两份 nav2.log 里 79 次 / 329 次 `AMCL cannot publish a pose`
#    和 `Timed out waiting for transform from base_link to map` 就是这个。
# 2) 手工给容易给错：狗在之前的实验里可能已经被开走，凭印象填坐标会填到原点。
#    2026-09-23 我就这么错过一次（狗在 (8.9, 0.6)，我按原点发了）。
#    所以这里一律从真值读。
#
# 前提：地图与世界系恒等对齐（已实测成立，AMCL 误差 0.150 m）。
# 如果换了地图或世界，先用真值核一遍对齐再用本脚本。
#
# 用法（在外层用 g1ros.sh 进容器）：
#   ./g1ros.sh bash tools/set_initial_pose.sh

set -uo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${G1_LOG_DIR:-/tmp/g1_p1}"
mkdir -p "${LOG_DIR}"

GT_RAW="/world/world_demo/pose/info"

# 自清理：脚本被 kill -9 时 trap 不会执行，会漏下桥/中继进程；
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

# 真值走 gz-transport，得先桥+中转才能用 ROS 读
ros2 run ros_gz_bridge parameter_bridge \
    "${GT_RAW}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
    --ros-args -r __node:=gt_bridge_pose >"${LOG_DIR}/gt_bridge_pose.log" 2>&1 &
PIDS+=($!)
python3 "${TOOLS_DIR}/gt_relay.py" --ros-args -p use_sim_time:=true \
    >"${LOG_DIR}/gt_relay_pose.log" 2>&1 &
PIDS+=($!)

for _ in $(seq 1 25); do
    ros2 topic list 2>/dev/null | grep -qx "/g1/ground_truth" && break
    sleep 1
done

# 解析不能按缩进 grep：Odometry 里 position 和 orientation 各有一套 x/y/z，
# 缩进还随层级变化（上一次写成 `^  x:` 就一个都没匹配上，静默失败）。
# 改成按 position/orientation 分块取，position 取 x、y，orientation 取 z、w。
read -r X Y QZ QW <<<"$(timeout 20 ros2 topic echo /g1/ground_truth --once \
    --field pose.pose 2>/dev/null | awk '
        /^position:/    { blk = "p"; next }
        /^orientation:/ { blk = "o"; next }
        blk == "p" && $1 == "x:" { px = $2 }
        blk == "p" && $1 == "y:" { py = $2 }
        blk == "o" && $1 == "z:" { qz = $2 }
        blk == "o" && $1 == "w:" { qw = $2 }
        END { if (px != "" && py != "" && qz != "" && qw != "") print px, py, qz, qw }
    ')"

if [ -z "${X:-}" ]; then
    echo "ERROR: 读不到真值；检查 IGN_PARTITION 是否与仿真一致" >&2
    exit 1
fi
echo "真值位姿：x=${X} y=${Y} qz=${QZ} qw=${QW}"

timeout 25 ros2 topic pub --once /robot1/initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" \
    >"${LOG_DIR}/initialpose.log" 2>&1

echo "已发布 /robot1/initialpose"
