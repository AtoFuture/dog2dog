#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

# The host is JetPack 7.2 (L4T 39.2). We use an Ubuntu 24.04 base image
# and mount the host JetPack 7.2 libraries at runtime (see run.sh).
if [ -z "${BASE_IMAGE:-}" ]; then
    BASE_IMAGE="ubuntu:24.04"
    echo "Using ${BASE_IMAGE} base image; host JetPack 7.2 libraries mounted at runtime."
fi

# Use sudo if the current user is not root and docker requires it.
DOCKER="docker"
if [ "$EUID" -ne 0 ] && ! docker info >/dev/null 2>&1; then
    DOCKER="sudo docker"
    echo "Docker requires root; using sudo."
fi

echo "Building qwen-robotnav image with BASE_IMAGE=${BASE_IMAGE}"
${DOCKER} build \
    -t qwen-robotnav:latest \
    -f "${SCRIPT_DIR}/Dockerfile.nav" \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    "${PROJECT_ROOT}"

echo "Build complete. Run with: ${SCRIPT_DIR}/run.sh"
