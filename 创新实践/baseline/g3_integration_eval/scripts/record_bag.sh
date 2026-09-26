#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G3_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TOPIC_FILE="${G3_DIR}/config/rosbag_topics.txt"

EXPERIMENT_NAME="${1:-experiment}"
NAVIGATE_ACTION="${G3_NAVIGATE_ACTION:-${2:-/robot1/navigate_to_pose}}"
BATTERY_TOPIC="${G3_BATTERY_TOPIC:-${3:-/robot1/battery_state}}"
BAG_ROOT="${G3_BAG_DIR:-/home/wy/g3_bags}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${BAG_ROOT}/${EXPERIMENT_NAME}_${TIMESTAMP}"

if [[ ! -f "${TOPIC_FILE}" ]]; then
    echo "ERROR: topic list not found: ${TOPIC_FILE}" >&2
    exit 1
fi

if [[ ! "${NAVIGATE_ACTION}" =~ ^/[A-Za-z0-9_/-]+$ ]] \
    || [[ "${NAVIGATE_ACTION}" == */ ]]; then
    echo "ERROR: navigate action must be an absolute ROS name without a trailing slash" >&2
    exit 2
fi

if [[ ! "${BATTERY_TOPIC}" =~ ^/[A-Za-z0-9_/-]+$ ]] \
    || [[ "${BATTERY_TOPIC}" == */ ]]; then
    echo "ERROR: battery topic must be an absolute ROS name without a trailing slash" >&2
    exit 2
fi

NAV_FEEDBACK_TOPIC="${NAVIGATE_ACTION}/_action/feedback"
NAV_STATUS_TOPIC="${NAVIGATE_ACTION}/_action/status"

mapfile -t TOPICS < <(
    grep -Ev '^[[:space:]]*(#|$)' "${TOPIC_FILE}"
)

for index in "${!TOPICS[@]}"; do
    case "${TOPICS[index]}" in
        /navigate_to_pose/_action/feedback)
            TOPICS[index]="${NAV_FEEDBACK_TOPIC}"
            ;;
        /navigate_to_pose/_action/status)
            TOPICS[index]="${NAV_STATUS_TOPIC}"
            ;;
        /robot1/battery_state)
            TOPICS[index]="${BATTERY_TOPIC}"
            ;;
    esac
done

if [[ ${#TOPICS[@]} -eq 0 ]]; then
    echo "ERROR: topic list is empty" >&2
    exit 1
fi

mkdir -p "${BAG_ROOT}"

echo "Experiment : ${EXPERIMENT_NAME}"
echo "ROS domain : ${ROS_DOMAIN_ID:-unset}"
echo "Nav action : ${NAVIGATE_ACTION}"
echo "Battery    : ${BATTERY_TOPIC}"
echo "Output     : ${OUTPUT_DIR}"
echo "Topics     : ${#TOPICS[@]}"
echo
echo "Press Ctrl+C to stop recording safely."

exec ros2 bag record \
    --include-hidden-topics \
    -o "${OUTPUT_DIR}" \
    "${TOPICS[@]}"
