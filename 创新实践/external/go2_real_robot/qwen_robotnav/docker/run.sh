#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DOCKER="docker"
if [ "$EUID" -ne 0 ] && ! docker info >/dev/null 2>&1; then
    DOCKER="sudo docker"
    echo "Docker requires root; using sudo."
fi

IMAGE=${IMAGE:-qwen-robotnav:latest}
CONTAINER_NAME=${CONTAINER_NAME:-qwen-robotnav}

# Default environment
VLLM_BASE_URL=${VLLM_BASE_URL:-http://100.113.176.95:8000}
VLLM_MODEL=${VLLM_MODEL:-Qwen3-VL-30B-A3B-Instruct-FP8}

# Check if container is already running.
if ${DOCKER} ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
    echo "Container ${CONTAINER_NAME} already running. Attaching..."
    ${DOCKER} exec -it "${CONTAINER_NAME}" /bin/bash
    exit 0
fi

# Remove stale container if it exists.
${DOCKER} rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "Starting ${CONTAINER_NAME} from ${IMAGE} ..."

# Mount points for JetPack 7.2 host libraries (CUDA / cuDNN / TensorRT / Tegra).
# The container uses the host drivers; libraries are exposed via LD_LIBRARY_PATH.
JP_LIB_MOUNTS=(
    -v /usr/local/cuda-13.2:/usr/local/cuda-13.2:ro
    -v /usr/local/cuda:/usr/local/cuda:ro
    -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:ro
    -v /opt/nvidia/l4t-gpu-libs:/opt/nvidia/l4t-gpu-libs:ro
)

${DOCKER} run -it --rm \
    --name "${CONTAINER_NAME}" \
    --privileged \
    --network host \
    --ipc host \
    --runtime nvidia \
    -e DISPLAY="${DISPLAY:-}" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    -e VLLM_BASE_URL="${VLLM_BASE_URL}" \
    -e VLLM_MODEL="${VLLM_MODEL}" \
    -e VLLM_API_KEY="${VLLM_API_KEY:-}" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-13.2/lib64:/opt/nvidia/l4t-gpu-libs/nvgpu:${LD_LIBRARY_PATH}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev:rw \
    -v /run/udev:/run/udev:rw \
    "${JP_LIB_MOUNTS[@]}" \
    -v "${PROJECT_ROOT}/qwen_robotnav:/workspace/qwen_robotnav/qwen_robotnav:rw" \
    -v "${PROJECT_ROOT}/ros2_ws:/workspace/qwen_robotnav/ros2_ws:rw" \
    -v "${PROJECT_ROOT}/tests:/workspace/qwen_robotnav/tests:rw" \
    -v "${PROJECT_ROOT}/scripts:/workspace/qwen_robotnav/scripts:rw" \
    -v "${SCRIPT_DIR}/entrypoint.sh:/workspace/qwen_robotnav/entrypoint.sh:ro" \
    -v "${SCRIPT_DIR}/fastrtps.xml:/workspace/qwen_robotnav/docker/fastrtps.xml:ro" \
    -w /workspace/qwen_robotnav \
    "${IMAGE}" \
    /bin/bash
