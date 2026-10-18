#!/bin/bash
# P1 第 5 项「边跑边建图」验收 —— 一条命令跑完。
#
# 为什么要有它：这个验收被卡过很多次，每次都是不同的坑（参数键名、配置文件被改回、
# 仿真跑久了走不动）。这个脚本把这些前置条件**全部做成闸门**，任一条不过就停下并
# 说清楚原因，不让你跑完一轮才发现结论没意义。
#
# 前置（脚本会自己查，不替你起仿真 —— 起仿真要 GPU，很重）：
#   · 隔离仿真在跑，且必须加载了 gz_ros2_control（用顶层 gazebo_nav2.launch.py
#     起；手工 ign gazebo 会漏系统插件，控制器/commands 看着正常但狗原地走）
#   · 推荐任意隔离 Domain + IGN_PARTITION；下面是旧共享通路示例：
#       cd ~/go2_sim_ws && GUI=1 GZ_HEADLESS=1 ROS_DOMAIN_ID=43 IGN_PARTITION=wy ./run_sim_iso.sh
#
# 用法（外层用 g1ros.sh 进容器）：
#   ./g1ros.sh bash tools/slam_e2e_check.sh [走多远m] [输出目录]
#   例：./g1ros.sh bash tools/slam_e2e_check.sh 3

set -uo pipefail

DISTANCE="${1:-3.0}"
OUT="${2:-/tmp/g1_slam_e2e_$(date +%H%M%S)}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${OUT}_logs"
mkdir -p "${LOG_DIR}"

PIDS=()
cleanup() {
    trap - EXIT INT TERM
    python3 "${TOOLS_DIR}/owned_processes.py" "${PIDS[@]}"
    for pid in "${PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

fail() { echo "ERROR: $*" >&2; exit 2; }

echo "== 1/5 闸门：仿真在不在 =="
TOPICS="$(timeout 10 ros2 topic list 2>/dev/null || true)"
if ! grep -qE '^/robot1/(odometry/filtered|odom)$' <<<"${TOPICS}"; then
    cat >&2 <<'MSG'
ERROR: 看不到 /robot1/odometry/filtered（或旧版 /robot1/odom）—— 隔离仿真没在跑。

先起仿真（两种已知可用的方式）：
  1) 共享仿真（旧通路，注意 Domain 43 / IGN_PARTITION=wy）：
       cd ~/go2_sim_ws && GUI=1 GZ_HEADLESS=1 ROS_DOMAIN_ID=43 IGN_PARTITION=wy ./run_sim_iso.sh
  2) qwen_robotnav 生成的隔离仿真（水平单线雷达，推荐用于 SLAM）：
       python3 scripts/prepare_simulator_launch.py --single-odom-tf \
         --sim-workspace ~/go2_sim_ws/ROS2-Gazebo-GO2 --output-dir /tmp/g1_slam_sim
       然后在 ros-gui 容器里设 GZ_HEADLESS=1 起 /tmp/g1_slam_sim/gazebo_nav2.launch.py

⚠️ 两种方式都必须设 G1_SINGLE_ODOM_TF=1（或等价配置），让 EKF 独占 odom→base_link。
⚠️ 仿真跑超过一天会「原地蹬腿」（预检全绿但狗走不动），测前请新起一份。
MSG
    exit 2
fi
echo "   /robot1/odometry/filtered 在 ✓"
if ! grep -qx "/robot1/velodyne" <<<"${TOPICS}"; then
    fail "看不到 /robot1/velodyne —— SLAM 没有可用的扫描话题（不要再期望 /scan）。"
fi
VELO_PUBS="$(timeout 15 ros2 topic info /robot1/velodyne 2>/dev/null \
    | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")"
[ "${VELO_PUBS:-0}" -ge 1 ] || fail "/robot1/velodyne 没有发布者，SLAM 无法建图。"
echo "   /robot1/velodyne 在流 ✓"

echo "== 2/5 起 Nav2（slam 模式）=="
# start_nav2_isolated.sh 会自己确认控制器激活；这里放后台，后面单独查。
# G1_LOG_DIR 必须显式导出：它决定 start_nav2_isolated.sh / cmd_vel_scaler 的日志位置，
# 之前不导出时 e2e 在 /tmp 找日志、脚本却写到 /tmp/g1_p1，永远等不到。
export G1_LOG_DIR="${LOG_DIR}"
G1_START_SCALER=0 setsid bash "${TOOLS_DIR}/start_nav2_isolated.sh" --slam >"${LOG_DIR}/slam_e2e_nav2.log" 2>&1 &
NAV_PID=$!
PIDS+=("${NAV_PID}")
echo "   pid=${NAV_PID}，等 slam_toolbox 起来"
for _ in $(seq 1 30); do
    grep -qE "Using solver plugin" "${LOG_DIR}/nav2_scaled.log" 2>/dev/null && break
    kill -0 "${NAV_PID}" 2>/dev/null || fail "Nav2 启动失败，见 ${LOG_DIR}/slam_e2e_nav2.log"
    sleep 3
done
sleep 12

echo "== 3/5 闸门：slam_toolbox 参数有没有生效 =="
# 当前参数不再从 config/go2_nav2.yaml 读，而是 nav2_scaled.launch.py 里的 SLAM_PARAMS。
# 这里核对运行中的真实参数，防止 launch 被改回 scan_topic=/scan 或扫到错误话题。
BF="$(timeout 20 ros2 param get /robot1/slam_toolbox base_frame --no-daemon 2>/dev/null | tail -1)"
ST="$(timeout 20 ros2 param get /robot1/slam_toolbox scan_topic --no-daemon 2>/dev/null | tail -1)"
echo "   base_frame = ${BF}"
echo "   scan_topic = ${ST}"
case "${BF}" in
    *base_link*) ;;
    *) fail "base_frame 不是 base_link（是「${BF}」）—— SLAM 参数没生效。
       检查 tools/nav2_scaled.launch.py 的 SLAM_PARAMS：base_frame 必须是 base_link。" ;;
esac
case "${ST}" in
    *velodyne*) ;;
    *) fail "scan_topic 不是 velodyne（是「${ST}」）—— SLAM 参数没生效。
       检查 tools/nav2_scaled.launch.py 的 SLAM_PARAMS：scan_topic 必须是 /robot1/velodyne，
       不能是 /scan（仿真没有 /scan 发布者）。" ;;
esac
# 结构性回归：SLAM 模式绝不能同时启动 AMCL/map_server。
# 它们和 slam_toolbox 会同时发 /map 与 map→odom，地图会跳、SLAM 结果不可信。
NODES="$(timeout 15 ros2 node list --no-daemon 2>/dev/null || ros2 node list 2>/dev/null || true)"
if grep -qE '^/robot1/(amcl|map_server)$' <<<"${NODES}"; then
    fail "SLAM 模式仍启动了 AMCL/map_server。nav2_scaled.launch.py 必须在 slam:=true 时
       只包含 navigation_launch.py，不能包含 bringup_launch.py 的 localization 部分。"
fi
echo "   参数生效，且没有 AMCL/map_server 抢 /map ✓"

echo "== 4/5 闸门：/robot1/map 有发布者 =="
MAPUBS="$(timeout 15 ros2 topic info /robot1/map --no-daemon 2>/dev/null | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")"
[ "${MAPUBS:-0}" -ge 1 ] || fail "/robot1/map 没有发布者，slam_toolbox 没在建图。"
echo "   Publisher count = ${MAPUBS} ✓"

echo "== 5/5 驱动 + 判定 =="
# This run never starts the scaler, so no shared publisher needs to be stopped.
echo "   让狗走 ${DISTANCE} m（真值闭环），同时录 /robot1/map"
setsid bash "${TOOLS_DIR}/walk_test.sh" "${DISTANCE}" 0.02 "${OUT}" 200 &
WALK_PID=$!
PIDS+=("${WALK_PID}")
wait "${WALK_PID}"
WALK_RC=$?

echo
echo "== 地图增长判定（P1 第 5 项的正身）=="
if [ ! -f "${OUT}/metadata.yaml" ]; then
    echo "   证据包不存在：walk_test 在前置/驱动阶段失败，没有录到 /robot1/map。"
    MAP_RC=2
else
    python3 "${TOOLS_DIR}/map_growth.py" "${OUT}" --json "${LOG_DIR}/map_growth.json"
    MAP_RC=$?
fi


echo
echo "===================== 汇总 ====================="
echo "  走够 ${DISTANCE} m      ：$([ ${WALK_RC} -eq 0 ] && echo 通过 || echo 不通过)（walk_test rc=${WALK_RC}）"
echo "  地图边走边更新        ：$([ ${MAP_RC} -eq 0 ] && echo 通过 || echo 不通过)（map_growth rc=${MAP_RC}）"
echo "  证据包                ：${OUT}"
[ ${WALK_RC} -eq 0 ] && [ ${MAP_RC} -eq 0 ] && { echo "  → P1 第 5 项 ✅"; exit 0; }
echo "  → P1 第 5 项 ❌（两条都要过才算）"
exit 1
