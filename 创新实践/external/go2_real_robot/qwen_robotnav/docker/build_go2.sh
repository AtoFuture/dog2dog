#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

BASE_IMAGE=${BASE_IMAGE:-ubuntu:24.04}

DOCKER="docker"
if [ "$EUID" -ne 0 ] && ! docker info >/dev/null 2>&1; then
    DOCKER="sudo docker"
    echo "Docker requires root; using sudo."
fi

echo "Building qwen-robotnav:go2 image with BASE_IMAGE=${BASE_IMAGE}"
${DOCKER} build \
    -t qwen-robotnav:go2 \
    -f "${SCRIPT_DIR}/Dockerfile.go2" \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    "${PROJECT_ROOT}"

echo "Build complete. Run with: ${SCRIPT_DIR}/run_go2.sh"
