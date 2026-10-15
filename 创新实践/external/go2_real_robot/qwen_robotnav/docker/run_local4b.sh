#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DOCKER="docker"
if [ "$EUID" -ne 0 ] && ! docker info >/dev/null 2>&1; then
    DOCKER="sudo docker"
    echo "Docker requires root; using sudo."
fi

IMAGE=${IMAGE:-qwen-robotnav:local4b}
CONTAINER_NAME=${CONTAINER_NAME:-qwen-robotnav-local4b}

if ${DOCKER} ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
    echo "Container ${CONTAINER_NAME} already running. Attaching..."
    ${DOCKER} exec -it "${CONTAINER_NAME}" /bin/bash
    exit 0
fi

${DOCKER} rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "Starting ${CONTAINER_NAME} from ${IMAGE} ..."

${DOCKER} run -it --rm \
    --name "${CONTAINER_NAME}" \
    --privileged \
    --network host \
    --ipc host \
    --runtime nvidia \
    -e DISPLAY="${DISPLAY:-}" \
    -e QT_X11_NO_MITSHM=1 \
    -e MODELSCOPE_CACHE=/workspace/qwen_robotnav/.modelscope \
    -e LOCAL_VLM_MODEL="${LOCAL_VLM_MODEL:-Qwen/Qwen3-VL-4B-Instruct}" \
    -e TRANSFORMERS_OFFLINE=1 \
    -e HF_HUB_OFFLINE=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev:rw \
    -v /run/udev:/run/udev:rw \
    -v "${PROJECT_ROOT}/qwen_robotnav:/workspace/qwen_robotnav/qwen_robotnav:rw" \
    -v "${PROJECT_ROOT}/ros2_ws:/workspace/qwen_robotnav/ros2_ws:rw" \
    -v "${PROJECT_ROOT}/tests:/workspace/qwen_robotnav/tests:rw" \
    -v "${PROJECT_ROOT}/scripts:/workspace/qwen_robotnav/scripts:rw" \
    -v "${PROJECT_ROOT}/.modelscope:/workspace/qwen_robotnav/.modelscope:rw" \
    -w /workspace/qwen_robotnav \
    "${IMAGE}" \
    /bin/bash
