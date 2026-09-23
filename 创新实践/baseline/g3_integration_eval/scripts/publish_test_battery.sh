#!/usr/bin/env bash
set -euo pipefail

PERCENTAGE="${1:-0.19}"
BATTERY_TOPIC="${2:-/robot1/battery_state}"

if [[ "${ROS_DOMAIN_ID:-}" != "142" ]]; then
    echo "ERROR: refusing to publish unless ROS_DOMAIN_ID=142" >&2
    exit 2
fi

if [[ ! "${PERCENTAGE}" =~ ^(0([.][0-9]+)?|1([.]0+)?)$ ]]; then
    echo "ERROR: percentage must be a number from 0.0 through 1.0" >&2
    exit 2
fi

if [[ ! "${BATTERY_TOPIC}" =~ ^/[A-Za-z0-9_/]+$ ]]; then
    echo "ERROR: invalid battery topic: ${BATTERY_TOPIC}" >&2
    exit 2
fi

echo "Publishing BatteryState percentage=${PERCENTAGE} on ${BATTERY_TOPIC}"
echo "Safety domain: ROS_DOMAIN_ID=142"

exec ros2 topic pub --once \
    "${BATTERY_TOPIC}" \
    sensor_msgs/msg/BatteryState \
    "{percentage: ${PERCENTAGE}}"
