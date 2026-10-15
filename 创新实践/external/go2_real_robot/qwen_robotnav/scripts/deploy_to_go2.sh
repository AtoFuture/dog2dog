#!/bin/bash
set -e

# Deploy the Qwen-RobotNav workspace to the Unitree Go2 companion Jetson.
# Usage: ./scripts/deploy_to_go2.sh [user@]<go2_jetson_ip_or_hostname> [remote_dir]
#
# Non-interactive example with default password:
#   SSHPASS=123 ./scripts/deploy_to_go2.sh unitree@192.168.123.18

GO2_TARGET=${1:-"unitree@192.168.123.18"}
REMOTE_DIR=${2:-"/home/unitree/qwen_robotnav"}
PROJECT_ROOT=$(cd "$(dirname "$0")/.." && pwd)

echo "Deploying Qwen-RobotNav to Go2 companion Jetson at ${GO2_TARGET} ..."

SSH_CMD="ssh -o StrictHostKeyChecking=no"
RSYNC_RSH="ssh -o StrictHostKeyChecking=no"
if [ -n "${SSHPASS:-}" ]; then
    if command -v sshpass >/dev/null 2>&1; then
        SSH_CMD="sshpass -p ${SSHPASS} ssh -o StrictHostKeyChecking=no"
        RSYNC_RSH="sshpass -p ${SSHPASS} ssh -o StrictHostKeyChecking=no"
    else
        echo "WARNING: SSHPASS is set but sshpass is not installed; falling back to interactive SSH."
    fi
fi
export RSYNC_RSH

# Ensure the project directory is clean of large build artifacts before rsync.
${SSH_CMD} "${GO2_TARGET}" "mkdir -p ${REMOTE_DIR}" || {
    echo "ERROR: cannot SSH to ${GO2_TARGET}. Make sure the Go2 Jetson is reachable and SSH is enabled."
    exit 1
}

rsync -avz \
    --exclude='.git' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.pytest_cache' \
    --exclude='.modelscope' \
    --exclude='.huggingface' \
    --exclude='*.log' \
    --exclude='ros2_ws/build' \
    --exclude='ros2_ws/install' \
    --exclude='ros2_ws/log' \
    "${PROJECT_ROOT}/" "${GO2_TARGET}:${REMOTE_DIR}/"

echo "Code deployed to ${GO2_TARGET}:${REMOTE_DIR}"
echo "Next steps on Go2 Jetson:"
echo "  1. ssh ${GO2_TARGET}"
echo "  2. cd ${REMOTE_DIR}"
echo "  3. ./docker/build_go2.sh"
echo "  4. ./docker/run_go2.sh"
echo "  5. ros2 launch qwen_robotnav_ros robot_nav.launch.py instruction:=\"go forward\""
