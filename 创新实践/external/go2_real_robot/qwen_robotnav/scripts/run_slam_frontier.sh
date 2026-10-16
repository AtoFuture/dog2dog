#!/usr/bin/env bash
# Existing ROS Humble / GPU wrapper environment; simulator only, no hardware SDK.
set -euo pipefail
if [[ "${1:-}" == "--help" ]]; then
    echo 'Usage: run_slam_frontier.sh [OUTPUT_DIR] [GOALS=5] [--prepare-only]'
    echo 'Environment: SLAM_ROS_DOMAIN_ID, SLAM_IGN_PARTITION, G1_SIM_WORKSPACE, G1_ROS_WRAPPER, G1_GPU_WRAPPER, SLAM_MISSION_BRAIN=1, SLAM_PERCEPTION=1, SLAM_MAX_RANGE, SLAM_RECON_TASK=1, SLAM_RECON_PERSON_MODEL, SLAM_RECON_SCENARIO=single|multi|partial|full|moving_crossing|moving_parallel|moving_occlusion, SLAM_RECON_IDENTITY_POLICY, SLAM_RECON_REQUIRED_PERSONS, SLAM_EXPLORATION_TIMEOUT, SLAM_ACTIVE_OBSERVATION=1, SLAM_INJECT_RETURN_HOLD_S=0..30'
    exit 0
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
PROJECT_DIR="$(realpath "${SCRIPT_DIR}/../../../..")"
OUTPUT_DIR="${1:-/tmp/slam_frontier_$(date +%Y%m%d_%H%M%S)}"
GOALS="${2:-5}"
ROS_WRAPPER="${G1_ROS_WRAPPER:-${HOME}/.enroot/ros}"
GPU_WRAPPER="${G1_GPU_WRAPPER:-${HOME}/.enroot/ros-gui}"
SIM_WORKSPACE="${G1_SIM_WORKSPACE:-${HOME}/go2_sim_ws/ROS2-Gazebo-GO2}"
[[ "${GOALS}" =~ ^[1-9][0-9]*$ ]] || { echo 'GOALS must be positive' >&2; exit 2; }
[[ "${3:-}" == "" || "${3:-}" == "--prepare-only" ]] || { echo 'Unknown third argument' >&2; exit 2; }
REQUIRED_PERSONS="${SLAM_RECON_REQUIRED_PERSONS:-}"
if [[ -z "${REQUIRED_PERSONS}" ]]; then
    if [[ "${SLAM_RECON_SCENARIO:-single}" == "single" ]]; then REQUIRED_PERSONS=1; else REQUIRED_PERSONS=2; fi
fi
[[ "${REQUIRED_PERSONS}" =~ ^[1-9][0-9]*$ ]] || { echo 'Required person count must be positive' >&2; exit 2; }
IDENTITY_POLICY="${SLAM_RECON_IDENTITY_POLICY:-}"
if [[ -z "${IDENTITY_POLICY}" ]]; then
    if [[ "${SLAM_RECON_SCENARIO:-single}" == moving_* ]]; then IDENTITY_POLICY=motion_appearance_v1; else IDENTITY_POLICY=stationary; fi
fi
[[ "${IDENTITY_POLICY}" == "stationary" || "${IDENTITY_POLICY}" == "motion_appearance_v1" ]] || { echo 'Unknown identity policy' >&2; exit 2; }
if [[ "${SLAM_RECON_SCENARIO:-single}" == moving_* ]]; then
    [[ "${SLAM_RECON_TASK:-0}" == "1" && "${IDENTITY_POLICY}" == "motion_appearance_v1" ]] || { echo 'Moving scenes require recon and motion_appearance_v1' >&2; exit 2; }
fi
TRIAL_DOMAIN="${SLAM_ROS_DOMAIN_ID:-192}"
TRIAL_PARTITION="${SLAM_IGN_PARTITION:-frontier_${TRIAL_DOMAIN}_$$}"
for executable in "${ROS_WRAPPER}" "${GPU_WRAPPER}"; do
    [[ -x "${executable}" ]] || { echo "Missing configured ROS wrapper: ${executable}" >&2; exit 2; }
done
[[ ! -e "${OUTPUT_DIR}/bag" ]] || { echo "Choose a new output directory; bag already exists." >&2; exit 2; }
mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(realpath "${OUTPUT_DIR}")"
"${ROS_WRAPPER}" bash -c '
set -e
export PYTHONPATH="$1/..:${PYTHONPATH:-}"
colcon --log-base "$2/progress_log" build --base-paths "$1/../ros2_ws/src/qwen_robotnav_progress" --packages-select qwen_robotnav_progress --build-base "$2/progress_build" --install-base "$2/progress_install" >"$2/progress_build.log" 2>&1
python3 "$1/prepare_simulator_launch.py" --single-odom-tf --sim-workspace "$3" --output-dir "$2"
python3 "$1/prepare_amcl_navigation.py" --localization slam --sim-workspace "$3" --output-dir "$2" --slam-max-range "$4"
if [[ "$6" == "1" ]]; then
    python3 "$1/prepare_recon_scene.py" --world "$3/src/gazebo_sim/world/warehouse.sdf" --person-model "$5" --output-dir "$2" --scenario "$7"
fi
' _ "${SCRIPT_DIR}" "${OUTPUT_DIR}" "${SIM_WORKSPACE}" "${SLAM_MAX_RANGE:-4.0}" "${SLAM_RECON_PERSON_MODEL:-}" "${SLAM_RECON_TASK:-0}" "${SLAM_RECON_SCENARIO:-single}"
if [[ "${3:-}" == "--prepare-only" ]]; then
    echo "Prepared: ${OUTPUT_DIR}"
    exit 0
fi
# Refuse a shared/live ROS domain before starting any robot controller.
"${ROS_WRAPPER}" bash -c '
export ROS_DOMAIN_ID="$1" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$2/src/docker/cyclonedds.xml"
python3 - "$1" <<'"'"'PY'"'"'
import sys,time,rclpy
if int(sys.argv[1]) in (0,42,43):raise SystemExit("Choose an isolated SLAM_ROS_DOMAIN_ID")
rclpy.init();node=rclpy.create_node("frontier_domain_probe")
end=time.monotonic()+1
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
others=[name for name,ns in node.get_node_names_and_namespaces() if name!=node.get_name()]
node.destroy_node();rclpy.shutdown()
if others:raise SystemExit("Selected ROS domain is in use; choose another SLAM_ROS_DOMAIN_ID")
PY
' _ "${TRIAL_DOMAIN}" "${SIM_WORKSPACE}"
TRIAL_WORLD=warehouse.sdf
if [[ "${SLAM_RECON_TASK:-0}" == "1" ]]; then TRIAL_WORLD="${OUTPUT_DIR}/recon_warehouse.sdf"; fi
setsid "${GPU_WRAPPER}" bash -c '
source "$1/install/setup.bash"
export ROS_DOMAIN_ID="$3" IGN_PARTITION="$4" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$1/src/docker/cyclonedds.xml"
export GZ_HEADLESS=1 GZ_SIM_RESOURCE_PATH="$1/src/gazebo_sim/models"
if [[ -f /tmp/gz_egl/10_nvidia.json ]]; then export __EGL_VENDOR_LIBRARY_FILENAMES=/tmp/gz_egl/10_nvidia.json; fi
export PYTHONPATH="$5/..:${PYTHONPATH:-}"
exec ros2 launch "$2/gazebo_nav2.launch.py" sensors:=true world:="$6"
' _ "${SIM_WORKSPACE}" "${OUTPUT_DIR}" "${TRIAL_DOMAIN}" "${TRIAL_PARTITION}" "${SCRIPT_DIR}" "${TRIAL_WORLD}" >"${OUTPUT_DIR}/simulator.log" 2>&1 &
SIM_GROUP=$!
VALIDATION_GROUP=""
cleanup() {
    python3 - "${VALIDATION_GROUP}" "${SIM_GROUP}" <<'PY'
import os,signal,sys,time
for value in sys.argv[1:]:
    if not value:continue
    pgid=int(value)
    for sig,seconds in [(signal.SIGINT,10),(signal.SIGTERM,3)]:
        try:os.killpg(pgid,sig)
        except ProcessLookupError:break
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            try:os.killpg(pgid,0)
            except ProcessLookupError:break
            time.sleep(.1)
        else:continue
        break
    else:
        try:os.killpg(pgid,signal.SIGKILL)
        except ProcessLookupError:pass
PY
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid "${ROS_WRAPPER}" bash -c '
source "$1/install/setup.bash"
export ROS_DOMAIN_ID="$3" IGN_PARTITION="$4" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$1/src/docker/cyclonedds.xml"
if [[ -f "$5/baseline/install/local_setup.bash" ]]; then source "$5/baseline/install/local_setup.bash"; fi
source "$2/progress_install/local_setup.bash"
export PYTHONPATH="$5/baseline/g2_vision_explore:$5/baseline/g2_vision_explore/vision_detector:$5/baseline/g2_vision_explore/vision_explorer:$5/baseline/g3_integration_eval:${PYTHONPATH:-}"
BRAIN_ARGS=()
if [[ "$8" == "1" ]]; then
    set -a
    source "$5/.env"
    set +a
    export MISSION_BRAIN_API_KEY="${MISSION_BRAIN_API_KEY:-${DEEPSEEK_API_KEY:-}}"
    export MISSION_BRAIN_BASE_URL="${MISSION_BRAIN_BASE_URL:-${DEEPSEEK_VLM_BASE_URL:-https://api.deepseek.com}}"
    export MISSION_BRAIN_MODEL="${MISSION_BRAIN_MODEL:-${DEEPSEEK_VLM_MODEL:-deepseek-flash}}"
    BRAIN_ARGS=(--mission-brain)
fi
if [[ "$9" == "1" ]]; then BRAIN_ARGS+=(--perception); fi
if [[ "${10}" == "1" ]]; then BRAIN_ARGS+=(--recon-task --required-person-count "${11}" ); fi
if [[ "${13}" == "1" ]]; then BRAIN_ARGS+=(--active-observation); fi
BRAIN_ARGS+=(--recon-identity-policy "${14}" --inject-return-hold-s "${15}")
exec python3 "$6/validate_slam_frontier.py" "${BRAIN_ARGS[@]}" --output-dir "$2" --expected-domain "$3" --goals "$7" --exploration-timeout "${12}"
' _ "${SIM_WORKSPACE}" "${OUTPUT_DIR}" "${TRIAL_DOMAIN}" "${TRIAL_PARTITION}" "${PROJECT_DIR}" "${SCRIPT_DIR}" "${GOALS}" "${SLAM_MISSION_BRAIN:-${SLAM_RECON_TASK:-0}}" "${SLAM_PERCEPTION:-${SLAM_RECON_TASK:-0}}" "${SLAM_RECON_TASK:-0}" "${REQUIRED_PERSONS}" "${SLAM_EXPLORATION_TIMEOUT:-600}" "${SLAM_ACTIVE_OBSERVATION:-0}" "${IDENTITY_POLICY}" "${SLAM_INJECT_RETURN_HOLD_S:-0}" >"${OUTPUT_DIR}/trial.log" 2>&1 &
VALIDATION_GROUP=$!
if wait "${VALIDATION_GROUP}"; then
    TRIAL_STATUS=0
else
    TRIAL_STATUS=$?
fi
VALIDATION_GROUP=""
if [[ -f "${OUTPUT_DIR}/slam_map.yaml" ]]; then
    "${ROS_WRAPPER}" bash -c '
if [[ -f "$3/baseline/install/local_setup.bash" ]]; then source "$3/baseline/install/local_setup.bash"; fi
set -e
export PYTHONPATH="$3/baseline/g3_integration_eval:${PYTHONPATH:-}"
export MPLCONFIGDIR=/tmp/slam-frontier-mpl
python3 "$1/evaluate_slam_frontier.py" "$2" --reference-map "$4/src/navigation2/maps/warehouse_map.yaml"
python3 "$1/plot_slam_frontier.py" "$2"
if [[ "$5" == "1" ]]; then
    AUDIT_STATUS=0
    if [[ "$6" == moving_* ]]; then python3 "$1/evaluate_moving_recon.py" "$2" || AUDIT_STATUS=1;
    else python3 "$1/evaluate_recon_mission.py" "$2" || AUDIT_STATUS=1; fi
    python3 "$1/evaluate_return_recovery.py" "$2" || AUDIT_STATUS=1
    exit "${AUDIT_STATUS}"
fi
' _ "${SCRIPT_DIR}" "${OUTPUT_DIR}" "${PROJECT_DIR}" "${SIM_WORKSPACE}" "${SLAM_RECON_TASK:-0}" "${SLAM_RECON_SCENARIO:-single}"
fi
echo "Artifacts: ${OUTPUT_DIR}"
exit "${TRIAL_STATUS}"
