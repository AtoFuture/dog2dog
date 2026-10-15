#!/usr/bin/env bash
set -euo pipefail

exec ros2 run g3_integration_eval sim_battery --ros-args \
  -p use_sim_time:=true \
  -p auto_recharge_on_return_success:=true \
  -p return_event_topic:=/g3/return_event \
  -p recharge_percentage:=0.80 \
  "$@"
