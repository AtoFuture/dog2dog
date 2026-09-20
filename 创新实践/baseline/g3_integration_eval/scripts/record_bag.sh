#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G3_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TOPIC_FILE="${G3_DIR}/config/rosbag_topics.txt"

EXPERIMENT_NAME="${1:-experiment}"
BAG_ROOT="${G3_BAG_DIR:-/home/wy/g3_bags}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${BAG_ROOT}/${EXPERIMENT_NAME}_${TIMESTAMP}"

if [[ ! -f "${TOPIC_FILE}" ]]; then
    echo "ERROR: topic list not found: ${TOPIC_FILE}" >&2
    exit 1
fi

mapfile -t TOPICS < <(
    grep -Ev '^[[:space:]]*(#|$)' "${TOPIC_FILE}"
)

if [[ ${#TOPICS[@]} -eq 0 ]]; then
    echo "ERROR: topic list is empty" >&2
    exit 1
fi

mkdir -p "${BAG_ROOT}"

echo "Experiment : ${EXPERIMENT_NAME}"
echo "ROS domain : ${ROS_DOMAIN_ID:-unset}"
echo "Output     : ${OUTPUT_DIR}"
echo "Topics     : ${#TOPICS[@]}"
echo
echo "Press Ctrl+C to stop recording safely."

exec ros2 bag record \
    --include-hidden-topics \
    -o "${OUTPUT_DIR}" \
    "${TOPICS[@]}"
