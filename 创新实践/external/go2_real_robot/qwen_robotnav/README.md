# Qwen-RobotNav on Jetson Orin NX → Unitree Go2

本项目基于 [Qwen-RobotNav](https://qwen.ai/blog?id=qwen-robotnav) 技术报告，在 **NVIDIA Jetson Orin NX (JetPack 7.2 / L4T 39.2)** 上用 Docker 容器化方式实现一套可运行的导航推理系统，并计划迁移到 **宇树 Go2 机器狗**。

> **注意**：官方 `QwenLM/Qwen-RobotNav` 代码与模型权重尚未公开，因此本实现使用远端 `Qwen3-VL-30B-A3B-Instruct-FP8` 作为 VLM 骨干，本地 Jetson 负责任务自适应观察编码、提示构建、waypoint 解码和机器人控制接口。当远端 vLLM 不可达时，可回退到本地 `Qwen/Qwen3-VL-4B-Instruct`。

## 环境信息

- 主机：NVIDIA Jetson Orin NX
- JetPack：7.2 (L4T 39.2, Ubuntu 24.04, CUDA 13.2)
- 远端 VLM：`http://100.113.176.95:8000/v1`，模型名 `qwen3-vl-fp8`
- Docker：Ubuntu 24.04 基础镜像，挂载宿主机 JetPack 7.2 驱动/库

## 项目结构

```
Qwen_RobotNav/
├── docker/                         # Docker 镜像与运行脚本
│   ├── Dockerfile.nav              # 完整镜像（含 ROS2 Jazzy）
│   ├── Dockerfile.minimal          # 最小验证镜像（无 ROS2，构建快）
│   ├── build.sh                    # 镜像构建脚本
│   ├── run.sh                      # 容器运行脚本（挂载 JP7.2 库）
│   ├── entrypoint.sh               # 容器入口脚本
│   └── fastrtps.xml                # DDS 配置
├── qwen_robotnav/                  # 核心 Python 包
│   ├── config.py                   # 配置类
│   ├── observation_encoder.py      # 任务自适应观察编码（Algorithm 1）
│   ├── prompt_builder.py           # embodiment/视角/时间标签提示
│   ├── vlm_client.py               # 远端 vLLM OpenAI 兼容客户端
│   ├── action_head.py              # waypoint MLP（占位，待 hidden-state 接口）
│   ├── waypoint_decoder.py         # 反归一化 / 速度指令转换
│   └── navigation_engine.py        # 端到端导航调用
├── ros2_ws/src/qwen_robotnav_ros/  # ROS2 Jazzy 导航节点
│   ├── qwen_robotnav_ros/
│   │   ├── nav_node.py             # 订阅图像 → VLM → 发布 waypoints
│   │   └── go2_controller.py       # waypoints → cmd_vel
│   └── launch/robot_nav.launch.py
├── scripts/
│   ├── test_vllm_connection.py     # 验证远端 vLLM
│   └── mock_navigation.py          # 本地 mock 端到端测试
├── tests/                          # pytest 单元测试
├── pyproject.toml
└── README.md
```

## 快速开始

### 1. 构建最小验证镜像

```bash
cd /home/shuuyou/Qwen_RobotNav
sg docker -c "docker build -t qwen-robotnav:minimal -f docker/Dockerfile.minimal ."
```

### 2. 运行单元测试

```bash
sg docker -c "docker run --rm --network host \
  -v /home/shuuyou/Qwen_RobotNav:/workspace/qwen_robotnav:rw \
  -w /workspace/qwen_robotnav \
  qwen-robotnav:minimal pytest tests/ -v"
```

预期结果：`17 passed`

### 3. 测试远端 vLLM 连通性

```bash
sg docker -c "docker run --rm --network host \
  -v /home/shuuyou/Qwen_RobotNav:/workspace/qwen_robotnav:rw \
  -w /workspace/qwen_robotnav \
  -e VLLM_BASE_URL=http://100.113.176.95:8000/v1 \
  -e VLLM_MODEL=qwen3-vl-fp8 \
  qwen-robotnav:minimal python3 scripts/test_vllm_connection.py"
```

### 4. 本地 mock 端到端测试

```bash
sg docker -c "docker run --rm --network host \
  -v /home/shuuyou/Qwen_RobotNav:/workspace/qwen_robotnav:rw \
  -w /workspace/qwen_robotnav \
  -e VLLM_BASE_URL=http://100.113.176.95:8000/v1 \
  -e VLLM_MODEL=qwen3-vl-fp8 \
  qwen-robotnav:minimal python3 scripts/mock_navigation.py --steps 2"
```

## 完整 ROS2 镜像

完整镜像包含 ROS2 Jazzy，用于真机部署：

```bash
sg docker -c "docker build -t qwen-robotnav:latest -f docker/Dockerfile.nav ."
```

> 构建完整镜像需要下载 ROS2 Jazzy 包，耗时较长。如果网络不稳定，可先使用 `qwen-robotnav:minimal` 验证核心逻辑。

## 运行 ROS2 导航节点

在完整镜像构建成功后：

```bash
./docker/run.sh
# 容器内
ros2 launch qwen_robotnav_ros robot_nav.launch.py
```

或者覆盖参数：

```bash
ros2 launch qwen_robotnav_ros robot_nav.launch.py \
  instruction:="go to the living room" \
  task_mode:="ObjNav" \
  camera_topic:="/camera/color/image_raw" \
  unitree_iface:="eth0"
```

## 部署到宇树 Go2

1. 将本仓库复制到 Go2 上的 Jetson Orin NX。
2. 在 Go2 侧构建同样的 Docker 镜像（Go2  companion Jetson 为 JetPack 6，建议使用 `nvcr.io/nvidia/l4t-jetpack:r36.4.0` 基础镜像）。
3. 接入 Go2 自带相机（通过 unitree_ros2 或 unitree_sdk2）。
4. 启动导航节点；默认会同时启动 `go2_unitree_bridge`，可用 `unitree_iface:=<网卡>` 指定连狗网卡。确认 `/qwen_robotnav/cmd_vel` 通过 Unitree SDK2 能控制机器狗运动。大模型控制使用独立话题，避免覆盖遥控或 Nav2 的 `/cmd_vel`。如果 SDK 桥已在外部启动，传入 `start_unitree_bridge:=false`。
5. 先低速测试，逐步增加速度。

## 关键设计

- **远端 VLM**：Jetson 不加载大模型，所有视觉-语言推理在 `100.113.176.95:8000` 的 vLLM 服务完成。
- **任务自适应观察编码**：按论文 Algorithm 1 实现，支持 token budget、时间衰减、相机权重、采样模式。
- **自然语言标签**：视角（Front/Left/Right/Back）和时间步完全通过文本标签注入，无需修改 Qwen3-VL 架构。
- **Docker 隔离**：所有依赖在容器内，不污染宿主机；运行时挂载宿主机 JetPack 7.2 的 CUDA/cuDNN/TensorRT/tegra 库。

## 本地 Qwen3-VL-4B 部署（远端 vLLM 不可用时）

1. 构建本地推理镜像：

```bash
sg docker -c "docker build -t qwen-robotnav:local4b -f docker/Dockerfile.local4b ."
```

2. 下载模型权重（ModelScope，约 8.5 GB）：

```bash
sg docker -c "docker run -d --name qwen-download \
  -v /home/shuuyou/Qwen_RobotNav:/workspace/qwen_robotnav:rw \
  -w /workspace/qwen_robotnav \
  --dns 223.5.5.5 --dns 223.6.6.6 \
  qwen-robotnav:local4b \
  python3 -u scripts/download_model_4b_threaded.py"

# 查看进度
sg docker -c "docker logs -f qwen-download"
```

3. 运行本地 mock 测试：

```bash
# 进入容器
./docker/run_local4b.sh

# 容器内
python3 scripts/mock_navigation_local.py --steps 1
```

本地 4B 已成功在 Jetson Orin NX 上跑通，模型加载约 1 秒，单帧 CPU 推理可在数秒内完成。多帧历史会显著增加推理时间。

## 部署到宇树 Go2

1. 在 Jetson 本地验证完镜像和逻辑后，将代码同步到 Go2 companion Jetson：

```bash
./scripts/deploy_to_go2.sh unitree@192.168.123.15
```

2. 在 Go2 Jetson 上构建 Go2 专用镜像（基于 JetPack 6 / ROS2 Humble）：

```bash
ssh unitree@192.168.123.15
cd ~/qwen_robotnav
./docker/build_go2.sh
```

3. 运行 Go2 容器并启动导航节点：

```bash
./docker/run_go2.sh
ros2 launch qwen_robotnav_ros robot_nav.launch.py instruction:="go forward"
```

4. 接入 Go2 自带相机和运动控制接口（根据实际 SDK 版本调整 `nav_node.py` 中的 camera_topic 和 `go2_controller.py` 中的控制模式）。

5. 先低速测试，逐步增加速度。

## 已知限制

- `action_head.py` 当前为占位实现。OpenAI 兼容 API 不暴露 LLM hidden state，因此 VLM 直接生成 waypoint JSON。
- 完整 ROS2 镜像构建依赖 ROS2 apt 源，网络较慢时可能超时。
- Go2 真机相机和运动控制接口需要根据实际 SDK 版本进一步调整。
- 本地 4B 推理在 Jetson Orin NX 16GB 上显存压力较大，如 OOM 需考虑 INT8/INT4 量化或换用 2B 模型。

## 许可证

Apache-2.0
