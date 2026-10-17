#!/usr/bin/env bash
# One-command demo: headless Gazebo server + GUI client + unified runtime + web console.
#
# The simulator server is started through gazebo_nav2.launch.py with GZ_HEADLESS=1
# so gz_ros2_control is loaded and RViz stays off; the GUI is a separate
# `ign gazebo -g` client attached to the same partition. The web console then
# mirrors that GUI window and reads the same ROS domain.
#
# Usage:
#   ./scripts/run_gui_console_demo.sh [--explore]
# Environment:
#   DEMO_DOMAIN=227 DEMO_PARTITION=gui_demo_227 DEMO_DISPLAY=:2
#   DEMO_OUT=/home/wy/data1/.runtime/gui_console_demo
#   DEMO_WORLD=warehouse.sdf DEMO_ROBOT_MODEL=robot1_my_bot
#   DEMO_CONSOLE_PORT=8765
#
# Ctrl+C stops the console, runtime, GUI client and simulator started here.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${NAV_ROOT}/../../.." && pwd)"   # 创新实践
OUT="${DEMO_OUT:-/home/wy/data1/.runtime/gui_console_demo}"
DOMAIN="${DEMO_DOMAIN:-227}"
PARTITION="${DEMO_PARTITION:-gui_demo_${DOMAIN}}"
DISPLAY_NUM="${DEMO_DISPLAY:-:2}"
WORLD="${DEMO_WORLD:-warehouse.sdf}"
MODEL="${DEMO_ROBOT_MODEL:-robot1_my_bot}"
CONSOLE_PORT="${DEMO_CONSOLE_PORT:-8765}"
SIM_WS="${G1_SIM_WORKSPACE:-${HOME}/go2_sim_ws/ROS2-Gazebo-GO2}"
ROS_WRAPPER="${G1_ROS_WRAPPER:-${HOME}/.enroot/ros}"
GUI_WRAPPER="${G1_GPU_WRAPPER:-${HOME}/.enroot/ros-gui}"
CYCLONE="file://${SIM_WS}/src/docker/cyclonedds.xml"
PROJECT_CONTAINER="/home/wy/project"
NAV_CONTAINER="${PROJECT_CONTAINER}/external/go2_real_robot/qwen_robotnav"
EXPLORE=0
[[ "${1:-}" == "--explore" ]] && EXPLORE=1

fail() { echo "ERROR: $*" >&2; exit 1; }

# ---- validation -------------------------------------------------------------
if LC_ALL=C grep -q '[^ -~]' <<<"${OUT}"; then fail "DEMO_OUT must be ASCII: ${OUT}"; fi
[[ "${DOMAIN}" =~ ^[0-9]+$ ]] && (( DOMAIN >= 1 && DOMAIN <= 232 )) && [[ "${DOMAIN}" != 0 && "${DOMAIN}" != 42 && "${DOMAIN}" != 43 ]] \
  || fail "DEMO_DOMAIN must be 1..232 and not 0/42/43"
[[ -n "${PARTITION}" ]] || fail "DEMO_PARTITION must not be empty"
[[ -x "${ROS_WRAPPER}" && -x "${GUI_WRAPPER}" ]] || fail "Missing ROS wrappers"
for tool in x11vnc xwininfo xdotool python3 pgrep ss; do command -v "${tool}" >/dev/null || fail "Missing ${tool}"; done
[[ -f "${PROJECT_ROOT}/.env" ]] || fail "Missing ${PROJECT_ROOT}/.env (API key); see .env.example"
if ss -ltn 2>/dev/null | grep -q ":${CONSOLE_PORT} "; then
  fail "Port ${CONSOLE_PORT} is already in use; stop the existing console first"
fi

mkdir -p "${OUT}/logs"
if [[ ! -f "${OUT}/gazebo_nav2.launch.py" || ! -d "${OUT}/progress_install" ]]; then
  echo "== preparing simulator/navigation config in ${OUT} =="
  ( cd "${NAV_ROOT}" && SLAM_ROS_DOMAIN_ID="${DOMAIN}" SLAM_IGN_PARTITION="${PARTITION}" \
      ./scripts/run_slam_frontier.sh "${OUT}" 5 --prepare-only ) >"${OUT}/logs/prepare.log" 2>&1 \
    || { tail -20 "${OUT}/logs/prepare.log" >&2; fail "prepare failed"; }
fi

# EGL vendor config for headless ogre2 (same workaround as run_slam_frontier.sh)
mkdir -p /tmp/gz_egl
if [[ ! -f /tmp/gz_egl/10_nvidia.json ]]; then
  if [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
    cp /usr/share/glvnd/egl_vendor.d/10_nvidia.json /tmp/gz_egl/
  else
    printf '%s\n' '{' '    "file_format_version" : "1.0.0",' '    "ICD" : {' \
      '        "library_path" : "libEGL_nvidia.so.0"' '    }' '}' > /tmp/gz_egl/10_nvidia.json
  fi
fi

PIDS=()
STOPPING=0
collect_tree() {
  local parent="$1" child
  for child in $(pgrep -P "${parent}" 2>/dev/null || true); do
    ALL_PIDS+=("${child}")
    collect_tree "${child}"
  done
}
cleanup() {
  STOPPING=1
  trap - EXIT INT TERM
  echo; echo "== stopping demo =="
  # Collect descendants BEFORE signalling: launch/spawner children can be
  # reparented to init once their parent exits, and then cannot be found.
  ALL_PIDS=()
  for pid in "${PIDS[@]}"; do collect_tree "${pid}"; ALL_PIDS+=("${pid}"); done
  if ((${#ALL_PIDS[@]})); then
    mapfile -t ALL_PIDS < <(printf '%s\n' "${ALL_PIDS[@]}" | awk '!seen[$0]++')
  fi
  # Each top-level component was started with setsid, so -PID addresses its group.
  for pid in "${PIDS[@]}"; do kill -INT -"${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true; done
  for pid in "${ALL_PIDS[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  local alive
  for _ in $(seq 1 20); do
    alive=0
    for pid in "${ALL_PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then alive=1; fi
    done
    if (( alive == 0 )); then break; fi
    sleep 0.5
  done
  for pid in "${ALL_PIDS[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  sleep 2
  for pid in "${ALL_PIDS[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
  # Targeted fallback for the console's private ports (children use new sessions).
  ss -ltn 2>/dev/null | grep -q ':8766 ' && pkill -f "websockify --heartbeat 15 127.0.0.1:8766" 2>/dev/null || true
  ss -ltn 2>/dev/null | grep -q ':5909 ' && pkill -f "x11vnc -display ${DISPLAY_NUM} .*rfbport 5909" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros_env="source /opt/ros/humble/setup.bash
source '${SIM_WS}/install/setup.bash'
export ROS_DOMAIN_ID='${DOMAIN}' IGN_PARTITION='${PARTITION}' GZ_PARTITION='${PARTITION}'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='${CYCLONE}'
export GZ_SIM_RESOURCE_PATH='${SIM_WS}/src/gazebo_sim/models'"

# ---- 1. headless server + sensors/controllers (loads gz_ros2_control) -------
echo "== starting headless simulator on domain ${DOMAIN} / ${PARTITION} =="
setsid "${GUI_WRAPPER}" bash -c "${ros_env}
export GZ_HEADLESS=1
export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/gz_egl/10_nvidia.json
export PYTHONPATH='${NAV_CONTAINER}':\"\${PYTHONPATH:-}\"
exec ros2 launch '${OUT}/gazebo_nav2.launch.py' sensors:=true world:='${WORLD}'
" >"${OUT}/logs/sim.log" 2>&1 < /dev/null &
SIM_PID=$!; PIDS+=("${SIM_PID}")

for _ in $(seq 1 60); do
  grep -q "Configured and activated.*joint_group_controller" "${OUT}/logs/sim.log" 2>/dev/null && break
  kill -0 "${SIM_PID}" 2>/dev/null || { tail -30 "${OUT}/logs/sim.log" >&2; fail "simulator exited"; }
  sleep 2
done
grep -q "Configured and activated.*joint_group_controller" "${OUT}/logs/sim.log" 2>/dev/null \
  || fail "joint_group_controller did not activate; see ${OUT}/logs/sim.log"
echo "   controller active"

# ---- 2. GUI client on the requested display --------------------------------
echo "== starting GUI client on ${DISPLAY_NUM} =="
setsid "${GUI_WRAPPER}" bash -c "
source /opt/ros/humble/setup.bash
source '${SIM_WS}/install/setup.bash'
export IGN_PARTITION='${PARTITION}' GZ_PARTITION='${PARTITION}'
export GZ_SIM_RESOURCE_PATH='${SIM_WS}/src/gazebo_sim/models'
exec ign gazebo -g -v3 --render-engine ogre2
" >"${OUT}/logs/gui.log" 2>&1 < /dev/null &
GUI_PID=$!; PIDS+=("${GUI_PID}")
for _ in $(seq 1 40); do
  DISPLAY="${DISPLAY_NUM}" XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}" \
    xwininfo -root -tree 2>/dev/null | grep -q '"Gazebo":' && break
  kill -0 "${GUI_PID}" 2>/dev/null || { tail -20 "${OUT}/logs/gui.log" >&2; fail "GUI client exited"; }
  sleep 1
done
DISPLAY="${DISPLAY_NUM}" XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}" \
  xwininfo -root -tree 2>/dev/null | grep -q '"Gazebo":' || fail "Gazebo window not found on ${DISPLAY_NUM}"
echo "   GUI window ready"

# ---- 3. unified runtime ----------------------------------------------------
echo "== starting unified runtime =="
rm -f "${OUT}/runtime_manifest.json" "${OUT}/runtime_exit.json"
setsid "${ROS_WRAPPER}" bash -c "${ros_env}
source '${OUT}/progress_install/local_setup.bash'
export PYTHONPATH='${NAV_CONTAINER}:${NAV_CONTAINER}/ros2_ws/src/qwen_robotnav_ros:${PROJECT_CONTAINER}/baseline/g3_integration_eval:${PROJECT_CONTAINER}/baseline/g2_vision_explore:${PROJECT_CONTAINER}/baseline/g2_vision_explore/vision_explorer':\"\${PYTHONPATH:-}\"
set -a; source '${PROJECT_CONTAINER}/.env'; set +a
export MISSION_BRAIN_API_KEY=\"\${MISSION_BRAIN_API_KEY:-\${DEEPSEEK_API_KEY:-}}\"
ARGS=(--profile '${NAV_CONTAINER}/config/sim_runtime.yaml' --navigation-dir '${OUT}' --expected-domain '${DOMAIN}' --enable-motion)
if [[ -z \"\${MISSION_BRAIN_API_KEY:-}\" ]]; then ARGS+=(--candidate-selector deterministic); fi
exec python3 '${NAV_CONTAINER}/scripts/run_navigation_runtime.py' \"\${ARGS[@]}\"
" >"${OUT}/logs/runtime.log" 2>&1 < /dev/null &
RUNTIME_PID=$!; PIDS+=("${RUNTIME_PID}")

for _ in $(seq 1 60); do
  kill -0 "${RUNTIME_PID}" 2>/dev/null || { tail -30 "${OUT}/logs/runtime.log" >&2; fail "runtime exited"; }
  "${ROS_WRAPPER}" bash -c "${ros_env}; timeout 5 ros2 topic echo /mission_state --once 2>/dev/null | grep -q standby" 2>/dev/null && break
  sleep 2
done
echo "   runtime standby"

# ---- 4. web console --------------------------------------------------------
echo "== starting web console =="
setsid python3 "${NAV_ROOT}/web_console/server.py" --display "${DISPLAY_NUM}" \
  --domain "${DOMAIN}" --partition "${PARTITION}" --world world_demo --robot-model "${MODEL}" \
  --ros-env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --ros-env CYCLONEDDS_URI="${CYCLONE}" \
  --log-dir "${OUT}/logs/console" >"${OUT}/logs/console.log" 2>&1 < /dev/null &
CONSOLE_PID=$!; PIDS+=("${CONSOLE_PID}")

for _ in $(seq 1 30); do
  if curl -fsS -m 3 "http://127.0.0.1:${CONSOLE_PORT}/api/state" 2>/dev/null | grep -q '"available": true'; then break; fi
  kill -0 "${CONSOLE_PID}" 2>/dev/null || { tail -20 "${OUT}/logs/console.log" >&2; fail "console exited"; }
  sleep 1
done
curl -fsS -m 3 "http://127.0.0.1:${CONSOLE_PORT}/api/state" 2>/dev/null | grep -q '"available": true' \
  || fail "console did not report an available window"

if [[ "${EXPLORE}" == 1 ]]; then
  # Give G3 health debounce time to settle before accepting EXPLORE.
  sleep 5
  echo "== sending EXPLORE =="
  "${ROS_WRAPPER}" bash -c "${ros_env}; timeout 8 ros2 topic pub --once /g3/command std_msgs/msg/String '{data: EXPLORE}'" >/dev/null 2>&1 || true
fi

cat <<EOF

Demo ready: http://127.0.0.1:${CONSOLE_PORT}
  domain ${DOMAIN} / partition ${PARTITION} / display ${DISPLAY_NUM}
  logs: ${OUT}/logs/
  Ctrl+C stops the console, runtime, GUI and simulator started here.

Remote access: run this on your own computer, not on the server:
  ssh -N -L ${CONSOLE_PORT}:127.0.0.1:${CONSOLE_PORT} wy@<server>
or use the VSCode Ports panel and forward ${CONSOLE_PORT}.
EOF

wait "${RUNTIME_PID}" || true
if (( STOPPING == 0 )); then
  fail "runtime exited unexpectedly; see ${OUT}/logs/runtime.log"
fi
echo "Demo stopped."
