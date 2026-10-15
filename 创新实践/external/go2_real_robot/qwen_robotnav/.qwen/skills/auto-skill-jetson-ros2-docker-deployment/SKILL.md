---
name: jetson-ros2-docker-deployment
description: Deploy AI/ROS2 workloads on Jetson using Ubuntu base containers with host JetPack libraries mounted, avoiding unavailable l4t-jetpack images.
source: auto-skill
extracted_at: '2026-06-20T03:35:00.000Z'
---

# Jetson ROS2 Docker Deployment

## When to use

- Target is NVIDIA Jetson (Orin NX / Orin Nano / AGX Orin) running JetPack 6 or JetPack 7.
- Official `nvcr.io/nvidia/l4t-jetpack` images are unavailable, slow to pull, or the host JetPack version does not have a matching container tag.
- You need a Docker-based workflow that does not pollute the host OS, especially for ROS2, CUDA, cuDNN, TensorRT workloads.

## Procedure

1. **Confirm the actual JetPack version from the host** rather than relying on user memory or the robot datasheet.
   - `cat /etc/nv_tegra_release`
   - `dpkg -l | grep nvidia-l4t-core`
   - JetPack 7.2 maps to L4T 39.x; JetPack 6.x maps to L4T 36.x; JetPack 5.1.x maps to L4T 35.x.
   - In this project the Go2 companion Jetson was reported as JetPack 6 but actually ran JetPack 5.1.1 (L4T 35.3.1).

2. **Base image choice**
   - Use the Ubuntu version that matches the host L4T userspace (JetPack 6 → Ubuntu 22.04, JetPack 7 → Ubuntu 24.04).
   - Do **not** pull `nvcr.io/nvidia/l4t-jetpack:r40.x` unless you have verified it resolves; in practice these tags may not exist.

3. **Mount host JetPack libraries at runtime**
   The container gets GPU support from the host kernel/modules; you only need the userspace libraries:
   ```bash
   -v /usr/local/cuda-13.2:/usr/local/cuda-13.2:ro
   -v /usr/local/cuda:/usr/local/cuda:ro
   -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:ro
   -v /opt/nvidia/l4t-gpu-libs:/opt/nvidia/l4t-gpu-libs:ro
   ```
   Set `LD_LIBRARY_PATH` to include those mounts.

4. **ROS2 version mapping**
   - JetPack 5.1 / Ubuntu 20.04 → ROS2 Foxy (only if forced; see DDS warnings below).
   - JetPack 6 / Ubuntu 22.04 → ROS2 Humble.
   - JetPack 7 / Ubuntu 24.04 → ROS2 Jazzy.
   - Avoid mixing ROS2 distros with the wrong Ubuntu base.
   - **Humble/Jazzy are preferred over Foxy for robot deployments.** On this project, Foxy's Fast-DDS 2.1.4 repeatedly threw `std::bad_alloc` and its Cyclone DDS 0.7.0 segfaulted inside the container. Humble and Jazzy carry newer, more stable DDS versions.
   - For the Unitree Go2 companion Jetson, the user explicitly asked for ROS2 Jazzy inside Docker, regardless of the host's Ubuntu 20.04 / ROS2 Foxy. Always confirm the user's target distro rather than assuming it must match the host OS.

5. **pip inside Ubuntu containers**
   - Ubuntu 20.04 does **not** use `--break-system-packages` (PEP 668 is not enforced). Use plain `pip install`.
   - Ubuntu 22.04/24.04 enforce PEP 668. Use `pip install --break-system-packages` for application dependencies.
   - Do **not** try to upgrade `pip`/`setuptools`/`wheel` if they were installed by Debian; it fails with “RECORD file not found”.
   - Use `--ignore-installed` only when necessary, and prefer a Tsinghua/Aliyun PyPI mirror on Chinese networks to avoid timeouts.

6. **Reduce build time**
   - Switch apt sources to Tsinghua or USTC `ubuntu-ports` mirror for the base Ubuntu packages.
   - For ROS packages, check whether the chosen mirror actually has the required debs. In this project, the USTC `ros2/ubuntu` mirror returned 404 for Foxy Cyclone DDS packages, so we had to fall back to `packages.ros.org` for those specific debs.
   - Avoid installing `ros-<distro>-desktop` unless GUI tools are required; use `ros-<distro>-ros-base` plus needed packages.
   - Avoid PyTorch/transformers in the local container if the heavy model runs remotely (e.g., via vLLM); install only the client libraries (`openai`, `opencv-python`, `numpy`, etc.).

7. **DDS / network for Jetson**
   - Run with `--network host --ipc host --privileged` for ROS2.
   - Default to `rmw_fastrtps_cpp` on Humble/Jazzy; Foxy should be avoided (see item 4).
   - If you must use a custom `fastrtps.xml`, validate it against the target Fast-DDS version. In this project the original profile used `<initialPeersList><address>...</address></initialPeersList>`, which Fast-DDS 2.1 rejected and then crashed with `std::bad_alloc`; the correct Foxy/Humble format wraps each peer in `<locator><udpv4>...</udpv4></locator>`.
   - If using `rmw_cyclonedds_cpp`, watch out for `unitree_sdk2` bundling its own `libddsc.so`; an ABI/version mismatch can segfault the ROS2 node. Keep `LD_LIBRARY_PATH` ordered so the ROS Cyclone library is found first, or avoid loading Unitree SDK2 libraries unless needed.
   - If the deployment target cannot reach a remote VLM/service (e.g., the vLLM endpoint is on a Tailscale network that the Go2 Jetson is not on), route traffic through a reachable intermediary Jetson:
     - On the intermediary Jetson: `iptables -t nat -A POSTROUTING -o tailscale0 -s 192.168.123.0/24 -j MASQUERADE` and allow forwarding between the local Ethernet interface and `tailscale0`.
     - On the target Jetson: `ip route add <vllm-ip> via <intermediary-ip>`.
   - Verify SSH reachability before assuming the target IP. The user reported `192.168.123.161` for the Go2 Jetson, but the actual SSH-reachable address was `192.168.123.18`. Use `sshpass` with the default Unitree password (`123`) for non-interactive deployment.
   - Use Tsinghua TUNA mirrors for both apt and ros2 repositories in China to avoid timeouts. For `ubuntu:24.04`, write `sources.list` manually with `http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports noble ...`; for ROS2 Jazzy, use `http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main` with the TUNA rosdistro key.

8. **Local VLM fallback when remote service is unreachable**
   - If the remote vLLM/Qwen3-VL endpoint is unreachable (network, ACL, Tailscale issues), deploy a smaller model locally on the Jetson.
   - Prefer **Qwen3-VL-4B** over 8B/30B on Jetson Orin NX 16 GB; use CPU/GPU mixed inference with `torch_dtype="auto"` and `device_map="auto"`.
   - Download models from **ModelScope** when HuggingFace is slow/blocked in China, but expect DNS issues for the LFS CDN domain (`cdn-lfs-cn-*.modelscope.cn`) inside Docker. Always pass public DNS to the container:
     ```bash
     docker run --dns 223.5.5.5 --dns 223.6.6.6 ...
     ```
   - `modelscope.snapshot_download()` is convenient but tends to be single-threaded and can hang or fail on large LFS files. For multi-GB weights, prefer a chunked multi-threaded downloader (or `aria2c`) with fresh auth URLs per chunk, and save each chunk to a separate `.part` file before merging.
   - Verify downloaded weight file sizes against the expected `content-length` / ModelScope metadata before declaring success.
   - Keep the model cache on a host volume mounted into the container so it is not lost when the container is removed.
   - ModelScope stores snapshots under `$MODELSCOPE_CACHE/models/<model_id>` (not `$MODELSCOPE_CACHE/<model_id>`). Have `LocalVLM` check both paths and pass the actual directory to `transformers`.
   - Implement a `LocalVLM` class that follows the same interface as the remote `VLMClient` so callers can switch without code changes.
   - Provide a `NavigationEngine` factory that selects remote vs local client based on `vlm.model` name (e.g., model starting with `Qwen/` or `local:` → local HF/ModelScope). Preserve the original model string casing for filesystem paths; only use lowercased checks for the switch logic.
   - When running fully locally, set `TRANSFORMERS_OFFLINE=1` and `HF_HUB_OFFLINE=1` so `transformers` does not try to phone home to HuggingFace.

9. **Validation order**
   1. Unit tests in the container.
   2. Connectivity to remote services (vLLM / API) from both host and container.
   3. Mock end-to-end pipeline with synthetic data (remote or local VLM).
   4. Real sensor/actuator integration.

## Common pitfalls

- Assuming the user-reported JetPack version matches the actual installed version; always verify.
- Trying to log in to `nvcr.io` with an NVIDIA API key; API keys are for NGC, not Docker login to `nvcr.io` (use `$oauthtoken` + personal NGC key).
- Installing heavy ML packages in the local container when the model is served remotely.
- Forgetting `--break-system-packages` or `--ignore-installed` on modern Ubuntu containers.
- Assuming a remote endpoint that works from one machine (e.g., Windows laptop) is reachable from another (e.g., Jetson). Verify from the actual deployment host.
- Letting long downloads run inside a foreground shell command; always use a detached container or `nohup`/`screen` because tool-call timeouts will kill the process.
- Expecting local CPU inference of a 4B VLM to be fast on Jetson. Single-frame forward pass can take seconds to minutes on CPU; multi-frame history makes it much slower. For real-time navigation, keep history short, quantize, or use a remote GPU service.
- Lowercasing the model ID when passing it to `transformers`; preserve original casing for filesystem paths (e.g., `Qwen/Qwen3-VL-4B-Instruct`).

## Related files in this project

- `docker/Dockerfile.nav` — full ROS2 Jazzy + Unitree SDK2 image for local Jetson.
- `docker/Dockerfile.minimal` — lightweight validation image.
- `docker/Dockerfile.local4b` — local Qwen3-VL-4B inference image.
- `docker/Dockerfile.go2` — ROS2 Humble image for the JetPack 6 Go2 companion Jetson.
- `docker/run.sh` — runtime wrapper that mounts host JetPack libraries.
- `qwen_robotnav/local_vlm.py` — drop-in local HF/ModelScope VLM client.
- `scripts/download_model_4b_threaded.py` — chunked multi-threaded ModelScope downloader.
