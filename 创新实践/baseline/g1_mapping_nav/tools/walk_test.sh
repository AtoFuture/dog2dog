#!/bin/bash
# 测「狗会不会走」：真值闭环走一段，按真值判定，顺带录包留证据。
#
# 为什么不直接用 goal_test.sh：那个走 Nav2，判据依赖 action；实测出现过
# 「目标点落在 xy_goal_tolerance 里 → 40 ms 报成功、狗一步没动」的假通过
# （2026-09-27：17 个目标全在 0.15~0.22 m 外）。本脚本绕开 Nav2 和所有容差，
# 直接驱动运动链：cmd_vel → cmd_vel_pub.py → robot_velocity → 步态。
#
# 开跑前会拦两个已知的「静默死」：
#   1) joint_group_controller 不在 active（README 坑 #1）—— 命令全进空气
#   2) commands 话题 Publisher count ≠ 1（仿真看着全好但狗不走）
# 两条任一不过就停在这里，不浪费一轮。
#
# ⚠️ 退出码是真的（对照问题清单 H3：goal_test.sh 恒 exit 0）：
#     0 = 通过   1 = 判定不通过   2 = 前置/数据有问题
#
# 用法（在外层用 g1ros.sh 进容器）：
#   ./g1ros.sh bash tools/walk_test.sh <距离m> [速度] [输出目录] [超时秒]
# 例：
#   ./g1ros.sh bash tools/walk_test.sh 5                    # 向前走 5 m，最慢档
#   ./g1ros.sh bash tools/walk_test.sh 5 0.02 /tmp/walk_1
#   ./g1ros.sh bash tools/walk_test.sh 1 0.02 /tmp/walk_x 8  # 8 秒超时，用来验失败路径
#
# ⚠️ 超时走**位置参数**而不是 `WALK_TIMEOUT=8 ./g1ros.sh ...`：enroot 不继承宿主
#    环境，宿主机上设的变量进不了容器，脚本会静默用默认 200 s（踩过）。

set -uo pipefail

DISTANCE="${1:?用法: walk_test.sh <距离m> [速度] [输出目录] [超时秒]}"
SPEED="${2:-0.02}"
OUT="${3:-/tmp/g1_walk_$(date +%H%M%S)}"
TIMEOUT_S="${4:-200}"

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(dirname "${OUT}")"
mkdir -p "${LOG_DIR}"

CTRL_MGR="/robot1/controller_manager"
CMD_TOPIC="/robot1/joint_group_controller/commands"
GT_RAW="/world/world_demo/pose/info"
GT_TOPIC="/g1/ground_truth"

# ── 自清理：被 kill -9 时 trap 不执行，会漏下桥/中继；多个同名中继会互抢发布器 ──
pkill -9 -f "gt_relay.py" 2>/dev/null
pkill -9 -f "__node:=gt_bridge" 2>/dev/null
pkill -9 -f "ros2 bag record" 2>/dev/null
sleep 2

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        kill "${pid}" 2>/dev/null
        wait "${pid}" 2>/dev/null
    done
}
trap cleanup EXIT

echo "== 1/5 前置自检：控制器 =="
# 注意别用 tr -d "state='"：tr 删的是字符集，会把 active 里的字母也删掉
query_state() {
    timeout 20 ros2 service call "${CTRL_MGR}/list_controllers" \
        controller_manager_msgs/srv/ListControllers 2>/dev/null \
        | grep -oE "name='joint_group_controller', state='[a-z]+'" \
        | sed -E "s/.*state='([a-z]+)'.*/\1/"
}
STATE="$(query_state)"
if [ "${STATE}" != "active" ]; then
    echo "   当前 ${STATE:-查不到} → 先 configure 再 switch"
    # unconfigured 的控制器直接 switch 会返回 ok=True 但状态不变（实测）
    if [ "${STATE}" != "inactive" ]; then
        timeout 30 ros2 service call "${CTRL_MGR}/configure_controller" \
            controller_manager_msgs/srv/ConfigureController \
            "{name: joint_group_controller}" >"${LOG_DIR}/walk_configure.log" 2>&1
    fi
    timeout 30 ros2 service call "${CTRL_MGR}/switch_controller" \
        controller_manager_msgs/srv/SwitchController \
        "{activate_controllers: [joint_group_controller], strictness: 1}" \
        >"${LOG_DIR}/walk_activate.log" 2>&1
    STATE="$(query_state)"
fi
if [ "${STATE}" != "active" ]; then
    echo "ERROR: joint_group_controller = ${STATE:-查不到}，命令进不去，狗不会动。停。" >&2
    exit 2
fi
echo "   joint_group_controller: active"

echo "== 2/5 前置自检：commands 发布者数 =="
PUBS="$(timeout 15 ros2 topic info "${CMD_TOPIC}" 2>/dev/null \
    | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")"
if [ "${PUBS:-0}" != "1" ]; then
    echo "ERROR: ${CMD_TOPIC} 发布者数 = ${PUBS:-0}（应为 1）。" >&2
    echo "       仿真看着正常但狗不走，多半就是这条（控制器进程死了不会 respawn）。" >&2
    exit 2
fi
echo "   Publisher count: 1"

echo "== 3/5 起真值桥 + 中转 =="
ros2 run ros_gz_bridge parameter_bridge \
    "${GT_RAW}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
    --ros-args -r __node:=gt_bridge >"${LOG_DIR}/walk_bridge.log" 2>&1 &
PIDS+=($!)
python3 "${TOOLS_DIR}/gt_relay.py" --ros-args -p use_sim_time:=true \
    >"${LOG_DIR}/walk_relay.log" 2>&1 &
PIDS+=($!)
for _ in $(seq 1 25); do
    ros2 topic list 2>/dev/null | grep -qx "${GT_TOPIC}" && break
    sleep 1
done
# 桥挂了不等于有数据，真值必须真的在流
if ! timeout 15 ros2 topic echo "${GT_TOPIC}" --once >/dev/null 2>&1; then
    echo "ERROR: ${GT_TOPIC} 无数据；检查 IGN_PARTITION 是否与仿真一致" >&2
    exit 2
fi
echo "   真值在流 ✓"

echo "== 4/5 录包并驱动 =="
REC_SECS=$((TIMEOUT_S + 20))
# 顺带录 /robot1/map：跑 slam 模式时它就是「边走边建图」的证据，
# 收工后用 tools/map_growth.py <包> 判。AMCL 模式下这个话题只有 1 条静态图，
# 录了也无害（map_growth 会如实报「不是 SLAM 边跑边建图」）。
ros2 bag record -o "${OUT}" "${GT_TOPIC}" \
    /robot1/odom /robot1/odometry/filtered /robot1/cmd_vel \
    /robot1/robot_velocity /robot1/foot_contact /robot1/map \
    >"${LOG_DIR}/walk_record.log" 2>&1 &
REC=$!
PIDS+=("${REC}")
sleep 4

python3 "${TOOLS_DIR}/walk_driver.py" "${DISTANCE}" --speed "${SPEED}" --timeout "${TIMEOUT_S}"
RC=$?

echo "== 5/5 收尾 =="
timeout 5 ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
kill -INT "${REC}" 2>/dev/null
wait "${REC}" 2>/dev/null

# 包里必须真有真值，否则这轮什么也没验到（对照问题清单 H4：缺数据静默判通过）
if [ ! -f "${OUT}/metadata.yaml" ] || ! grep -q "${GT_TOPIC}" "${OUT}/metadata.yaml"; then
    echo "ERROR: 包里没有 ${GT_TOPIC}，这轮没验到东西。" >&2
    exit 2
fi
echo "   证据包：${OUT}"

echo
echo "== 判定 =="
if [ "${RC}" -eq 0 ]; then
    echo "   通过 ✅"
else
    echo "   不通过 ❌（walk_driver rc=${RC}）"
fi
exit "${RC}"
