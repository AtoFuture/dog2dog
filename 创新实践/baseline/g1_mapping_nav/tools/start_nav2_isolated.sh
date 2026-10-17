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
#   ./g1ros.sh bash tools/start_nav2_isolated.sh --slam      # P1 第 5 项：边跑边建图
#
# 默认地图：warehouse_map.yaml（--slam 时用不到，bringup 会忽略）
#
# --slam 时由 slam_toolbox 出 map→odom，替掉 map_server+amcl。
# 这时**不要**再跑 set_initial_pose.sh —— 那是给 AMCL 补初始位姿的。
#
# ⚠️ 为什么用命令行开关而不是 `SLAM=1` 环境变量：enroot **不继承宿主环境**，
#    `SLAM=1 ./g1ros.sh ...` 那个变量进不了容器，脚本会静默跑成 AMCL。
#    同样理由，本脚本里的 ${G1_LOG_DIR} 也得在容器内设才生效。

set -uo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_MAP="/home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/navigation2/maps/warehouse_map.yaml"

# --slam 之外的位置参数当地图路径（兼容原来的 `脚本 [地图yaml]` 用法）
MAP=""
SLAM="${SLAM:-0}"
for arg in "$@"; do
    case "${arg}" in
        --slam) SLAM=1 ;;
        *)      MAP="${arg}" ;;
    esac
done
MAP="${MAP:-${DEFAULT_MAP}}"

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

# 当前标准仿真（qwen_robotnav 生成的 Gazebo scene）没有加载
# libgz_ros2_control-system.so，`/robot1/controller_manager` 服务不存在；
# 运动链走自定义 quadruped_controller，命令直接发到
# /robot1/joint_group_controller/commands。此时用 commands 发布者数做等价自检。
SKIP_CONTROLLER_MANAGER=0
if [ -z "${STATE}" ]; then
    CMD_PUBS="$(timeout 15 ros2 topic info /robot1/joint_group_controller/commands 2>/dev/null \
        | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")"
    if [ "${CMD_PUBS:-0}" = "1" ]; then
        echo "   controller_manager 不可用；commands 发布者=1 → 自定义 quadruped 控制器通路"
        SKIP_CONTROLLER_MANAGER=1
    else
        echo "ERROR: controller_manager 查不到，且 commands 发布者=${CMD_PUBS:-0}" >&2
        echo "       狗不会动。检查仿真是否完整启动。" >&2
        exit 1
    fi
fi

if [ "${SKIP_CONTROLLER_MANAGER}" = "0" ] && [ "${STATE}" != "active" ]; then
    # 先 configure 再 activate —— 两步缺一不可。2026-09-24 在新起的仿真上实测：
    # `unconfigured` 的控制器调 switch_controller 会返回 ok=True 但**状态不变**，
    # 于是自检照样失败、停在 1/3（第一次跑就是这么被挡住的）。
    # 交接文档里写的「configure 可能返回 ok=False，但 switch 能激活」是反的：
    # 两个都得调，顺序是 configure → inactive → switch → active。
    if [ "${STATE}" != "inactive" ]; then
        echo "   当前是 ${STATE} → 先调 configure_controller"
        timeout 30 ros2 service call "${CTRL_MGR}/configure_controller" \
            controller_manager_msgs/srv/ConfigureController \
            "{name: joint_group_controller}" \
            >"${LOG_DIR}/configure_controller.log" 2>&1
        STATE="$(query_controller_state)"
        echo "   configure 后：${STATE:-查不到}"
    fi
    echo "   调 switch_controller 激活"
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

if [ "${SKIP_CONTROLLER_MANAGER}" = "0" ]; then
    echo "   确认 joint_state_broadcaster 已激活（N5: spawner 间歇失败致 odom 不发）"
    JSB_STATE="$(timeout 20 ros2 service call "${CTRL_MGR}/list_controllers" \
        controller_manager_msgs/srv/ListControllers 2>/dev/null \
        | grep -oE "name='joint_state_broadcaster', state='[a-z]+'" \
        | sed -E "s/.*state='([a-z]+)'.*/\1/")"
    if [ "${JSB_STATE}" != "active" ]; then
        echo "   joint_state_broadcaster 状态：${JSB_STATE:-查不到} → 激活"
        timeout 30 ros2 service call "${CTRL_MGR}/switch_controller" \
            controller_manager_msgs/srv/SwitchController \
            "{activate_controllers: [joint_state_broadcaster], strictness: 1}" \
            >"${LOG_DIR}/activate_jsb.log" 2>&1
    fi
else
    echo "   跳过 joint_state_broadcaster（controller_manager 不可用）"
fi

echo "== 2/3 起速度缩放器 =="
python3 "${TOOLS_DIR}/cmd_vel_scaler.py" --ros-args \
    -p use_sim_time:=true >"${LOG_DIR}/cmd_vel_scaler.log" 2>&1 &
SCALER_PID=$!
echo "   pid=${SCALER_PID}"
# 存活自检（问题清单「低」）：原来只记了个 pid 就不管了。
# 这个项目有「节点静默死掉」的前科（控制器进程被 SIGKILL、launch 没有 respawn），
# 缩放器真死了的话 Nav2 的输出就没人转发，表现是狗一步不动。
sleep 3
if ! kill -0 "${SCALER_PID}" 2>/dev/null; then
    echo "ERROR: 速度缩放器起来 3 秒内就退出了，狗不会动。日志尾部：" >&2
    tail -5 "${LOG_DIR}/cmd_vel_scaler.log" >&2
    exit 1
fi
echo "   存活 ✓"

echo "== 3/3 起 Nav2（我们自己那份 launch）=="
LAUNCH_ARGS=("map:=${MAP}")
if [ "${SLAM:-0}" != "0" ]; then
    # 必须写 True（Python 风格）：nav2_bringup 用 IfCondition(slam) 求值这个字符串，
    # 传 'true' 会 `name 'true' is not defined` 打挂整个 launch。
    LAUNCH_ARGS+=("slam:=True")
    echo "   模式：SLAM（slam_toolbox 边跑边建图）—— 不用跑 set_initial_pose.sh"
else
    echo "   模式：AMCL + 静态地图"
    echo "   地图：${MAP}"
fi
ros2 launch "${TOOLS_DIR}/nav2_scaled.launch.py" "${LAUNCH_ARGS[@]}" \
    >"${LOG_DIR}/nav2_scaled.log" 2>&1
RC=$?

kill "${SCALER_PID}" 2>/dev/null
wait "${SCALER_PID}" 2>/dev/null
echo "== Nav2 退出（rc=${RC}）=="
# 显式回传（问题清单「低」）：原来最后一句是 echo，rc 恒 0。
exit "${RC}"
