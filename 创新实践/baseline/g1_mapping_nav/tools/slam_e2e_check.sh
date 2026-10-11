#!/bin/bash
# P1 第 5 项「边跑边建图」验收 —— 一条命令跑完。
#
# 为什么要有它：这个验收被卡过很多次，每次都是不同的坑（参数键名、配置文件被改回、
# 仿真跑久了走不动）。这个脚本把这些前置条件**全部做成闸门**，任一条不过就停下并
# 说清楚原因，不让你跑完一轮才发现结论没意义。
#
# 前置（脚本会自己查，不替你起仿真 —— 起仿真要 GPU，很重）：
#   · 隔离仿真在跑（Domain 43 / IGN_PARTITION=wy，run_sim_iso.sh 起的）
#
# 用法（外层用 g1ros.sh 进容器）：
#   ./g1ros.sh bash tools/slam_e2e_check.sh [走多远m] [输出目录]
#   例：./g1ros.sh bash tools/slam_e2e_check.sh 3

set -uo pipefail

DISTANCE="${1:-3.0}"
OUT="${2:-/tmp/g1_slam_e2e_$(date +%H%M%S)}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(dirname "${OUT}")"
mkdir -p "${LOG_DIR}"

fail() { echo "ERROR: $*" >&2; exit 2; }

echo "== 1/5 闸门：仿真在不在 =="
if ! timeout 10 ros2 topic list 2>/dev/null | grep -qx "/robot1/odom"; then
    cat >&2 <<'MSG'
ERROR: 看不到 /robot1/odom —— 隔离仿真没在跑（或不在 Domain 43）。

先起仿真（约 1 分钟）：
    cd ~/go2_sim_ws && GUI=1 GZ_HEADLESS=1 ROS_DOMAIN_ID=43 IGN_PARTITION=wy ./run_sim_iso.sh

⚠️ 必须用 run_sim_iso.sh（它设 IGN_PARTITION=wy 且默认 G1_SINGLE_ODOM_TF=1）。
   用 run_sim.sh 会落到共享 Domain 42，和别人的仿真撞在一起。
⚠️ 仿真跑超过一天会「原地蹬腿」（预检全绿但狗走不动），测前请新起一份。
MSG
    exit 2
fi
echo "   /robot1/odom 在 ✓"

echo "== 2/5 起 Nav2（slam 模式）=="
# start_nav2_isolated.sh 会自己确认控制器激活；这里放后台，后面单独查
bash "${TOOLS_DIR}/start_nav2_isolated.sh" --slam >"${LOG_DIR}/slam_e2e_nav2.log" 2>&1 &
NAV_PID=$!
echo "   pid=${NAV_PID}，等 slam_toolbox 起来"
for _ in $(seq 1 30); do
    grep -qE "Using solver plugin" "${LOG_DIR}/nav2_scaled.log" 2>/dev/null && break
    sleep 3
done
sleep 12

echo "== 3/5 闸门：slam_toolbox 参数有没有生效 =="
# 这是被反复踩的坑：config/go2_nav2.yaml 里 slam 段必须「空壳裸键 + 全限定键」
# 两键共存，否则参数被静默丢弃（节点跑代码默认值），图一条也建不出来。
BF="$(timeout 20 ros2 param get /robot1/slam_toolbox base_frame --no-daemon 2>/dev/null | tail -1)"
ST="$(timeout 20 ros2 param get /robot1/slam_toolbox scan_topic --no-daemon 2>/dev/null | tail -1)"
echo "   base_frame = ${BF}"
echo "   scan_topic = ${ST}"
case "${BF}" in
    *base_link*) ;;
    *) fail "base_frame 不是 base_link（是「${BF}」）—— slam 参数没生效。
       改 config/go2_nav2.yaml：slam 段要**两个键**，
         slam_toolbox: {ros__parameters: {}}          ← 空壳，只为过 HasNodeParams 门禁
         /robot1/slam_toolbox: {ros__parameters: ...} ← 真参数写这里（rcl 只认全限定名）
       实测：只有裸键 → 参数全丢；两键 → 生效。验证只要 1 分钟，不用起仿真。" ;;
esac
case "${ST}" in
    *velodyne*) ;;
    *) fail "scan_topic 不是 velodyne（是「${ST}」）—— 同上，slam 参数没生效。" ;;
esac
echo "   参数生效 ✓"

echo "== 4/5 闸门：/robot1/map 有发布者 =="
MAPUBS="$(timeout 15 ros2 topic info /robot1/map --no-daemon 2>/dev/null | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")"
[ "${MAPUBS:-0}" -ge 1 ] || fail "/robot1/map 没有发布者，slam_toolbox 没在建图。"
echo "   Publisher count = ${MAPUBS} ✓"

echo "== 5/5 驱动 + 判定 =="
echo "   让狗走 ${DISTANCE} m（真值闭环），同时录 /robot1/map"
bash "${TOOLS_DIR}/walk_test.sh" "${DISTANCE}" 0.02 "${OUT}" 200
WALK_RC=$?

echo
echo "== 地图增长判定（P1 第 5 项的正身）=="
python3 "${TOOLS_DIR}/map_growth.py" "${OUT}" --json "${LOG_DIR}/map_growth.json"
MAP_RC=$?

kill "${NAV_PID}" 2>/dev/null
pkill -f "[n]av2_scaled.launch.py" 2>/dev/null
pkill -f "[c]md_vel_scaler.py" 2>/dev/null

echo
echo "===================== 汇总 ====================="
echo "  走够 ${DISTANCE} m      ：$([ ${WALK_RC} -eq 0 ] && echo 通过 || echo 不通过)（walk_test rc=${WALK_RC}）"
echo "  地图边走边更新        ：$([ ${MAP_RC} -eq 0 ] && echo 通过 || echo 不通过)（map_growth rc=${MAP_RC}）"
echo "  证据包                ：${OUT}"
[ ${WALK_RC} -eq 0 ] && [ ${MAP_RC} -eq 0 ] && { echo "  → P1 第 5 项 ✅"; exit 0; }
echo "  → P1 第 5 项 ❌（两条都要过才算）"
exit 1
