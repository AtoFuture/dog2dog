#!/usr/bin/env bash
# Run G2 tests with the existing ROS Humble environment and built interfaces.
set -eo pipefail

G2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASELINE_DIR="$(cd "${G2_DIR}/.." && pwd)"
ROS_SETUP="${G2_ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_WRAPPER="${G2_ROS_WRAPPER:-${HOME}/.enroot/ros}"
LOCAL_ONLY=0
if [[ "${1:-}" == "--local" ]]; then
    LOCAL_ONLY=1
    shift
fi

if [[ -f "${ROS_SETUP}" ]]; then
    source "${ROS_SETUP}"
fi
if [[ -f "${BASELINE_DIR}/install/local_setup.bash" ]]; then
    source "${BASELINE_DIR}/install/local_setup.bash"
fi

cd "${G2_DIR}"
export PYTHONPATH="${G2_DIR}:${G2_DIR}/vision_explorer:${G2_DIR}/vision_detector:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
# ROS launch-testing plugins are not needed by this suite and conflict with
# some installed pytest versions. Keep ordinary unit test collection isolated.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

if ! python3 -c 'import cv2, yaml, pytest, ultralytics, rclpy, cv_bridge, nav2_msgs, vision_interfaces' >/dev/null 2>&1; then
    if [[ "${LOCAL_ONLY}" == 0 && -x "${ROS_WRAPPER}" ]]; then
        exec "${ROS_WRAPPER}" bash "${G2_DIR}/tools/run_tests.sh" --local "$@"
    fi
    echo "G2 test dependencies are incomplete for $(command -v python3)." >&2
    echo "Use the ROS Humble system Python and build baseline/interfaces first." >&2
    python3 -c 'import cv2, yaml, pytest, ultralytics, rclpy, cv_bridge, nav2_msgs, vision_interfaces'
    exit 1
fi

exec python3 -m pytest tests -p no:cacheprovider "$@"
