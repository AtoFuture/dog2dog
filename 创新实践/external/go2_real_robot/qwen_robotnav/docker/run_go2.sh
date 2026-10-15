#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DOCKER="docker"
if [ "$EUID" -ne 0 ] && ! docker info >/dev/null 2>&1; then
    DOCKER="sudo docker"
    echo "Docker requires root; using sudo."
fi

IMAGE=${IMAGE:-qwen-robotnav:go2}
CONTAINER_NAME=${CONTAINER_NAME:-qwen-robotnav-go2}

VLLM_BASE_URL=${VLLM_BASE_URL:-http://100.113.176.95:8000}
VLLM_MODEL=${VLLM_MODEL:-qwen3-vl-fp8}

if ${DOCKER} ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
    echo "Container ${CONTAINER_NAME} already running. Attaching..."
    ${DOCKER} exec -it "${CONTAINER_NAME}" /bin/bash
    exit 0
fi

${DOCKER} rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "Starting ${CONTAINER_NAME} from ${IMAGE} ..."

# Mount points for JetPack 5.x host libraries.
# NOTE: Do NOT mount the whole /usr/lib/aarch64-linux-gnu directory, because the
# Go2 host runs Ubuntu 20.04 while the Jazzy container runs Ubuntu 24.04;
# overlaying the host's 20.04 system libraries would break the container.
#
# NOTE: Only mount ros2_ws/src from the host. The image already contains the
# colcon build/ and install/ artifacts; mounting the whole ros2_ws directory
# would shadow them with the host's (possibly empty) copy.
JP_LIB_MOUNTS=(
    -v /usr/local/cuda:/usr/local/cuda:ro
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro
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
    -e LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda/lib64:/opt/unitree_sdk2/lib:${LD_LIBRARY_PATH}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev:rw \
    -v /run/udev:/run/udev:rw \
    "${JP_LIB_MOUNTS[@]}" \
    -v "${PROJECT_ROOT}/qwen_robotnav:/workspace/qwen_robotnav/qwen_robotnav:rw" \
    -v "${PROJECT_ROOT}/ros2_ws/src:/workspace/qwen_robotnav/ros2_ws/src:rw" \
    -v "${PROJECT_ROOT}/tests:/workspace/qwen_robotnav/tests:rw" \
    -v "${PROJECT_ROOT}/scripts:/workspace/qwen_robotnav/scripts:rw" \
    -v "${SCRIPT_DIR}/entrypoint.sh:/workspace/qwen_robotnav/entrypoint.sh:ro" \
    -v "${SCRIPT_DIR}/fastrtps.xml:/workspace/qwen_robotnav/docker/fastrtps.xml:ro" \
    -w /workspace/qwen_robotnav \
    "${IMAGE}" \
    /bin/bash
