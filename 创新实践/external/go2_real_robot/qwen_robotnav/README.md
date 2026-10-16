# Qwen-RobotNav on Jetson Orin NX → Unitree Go2

本项目基于Qwen-RobotNav和G2探索流程实现Go2自主导航。
**当前统一链路使用DeepSeek作为Mission Brain**：它选择G2根据雷达地图生成的安全候选点，不根据相机画面直接生成导航坐标。
`qwen_robotnav` 包名、节点名和话题名保留以兼容已有启动脚本。


> 当前唯一正式入口为 `scripts/run_navigation_runtime.py`；普通启动和只读验收均调用它。默认由 DeepSeek 对 G2 安全候选排序，确定性选择作为同一 G2 模块的回退/对照模式。历史人物、推车及旧故障实验已冻结，需显式 `--legacy-validation` 或 `SLAM_LEGACY_VALIDATION=1` 复现。使用方法见 [统一运行说明](docs/navigation-runtime.md)。

正式运行已支持探索时间预算、覆盖增长停滞和候选耗尽后的自动返航；只有多帧确认无 frontier 才记为探索完成。失败区域采用临时拉黑和每轮次数上限。运行 `run_runtime_trial.py --autonomous-completion` 可验证运行栈自行结束任务，详见 [任务闭环](docs/navigation-runtime.md#自主结束探索与有界恢复)。

## 旧视觉航点组件

统一导航运行栈已停止使用相机直接生成导航点。下面的视觉航点节点保留用于旧实验复现；当前正式链路使用G2根据雷达SLAM地图生成安全候选点，DeepSeek Mission Brain只选择候选ID。

- 视觉节点优先读取 `DEEPSEEK_API_KEY`；API 地址与模型通过历史参数名
  `vllm_base_url` / `vllm_model` 配置，默认就是 DeepSeek。
- 视觉节点以单独工作线程执行推理，一次只运行一个请求；ROS 回调继续接收图像和指令。
- `inference_timeout_sec` 默认 15 秒，限制整个 HTTP 请求，关闭 SDK 自动重试。
  客户端通过可取消的异步请求执行总时限，结束后关闭连接；超时不会让工作线程无限等待。
- `max_frame_age_sec` 默认 15 秒；过期图像不提交推理，过期结果不发布航点。
- 推理期间收到新指令时，旧指令对应的结果会被丢弃。
- 文本模型返回的航点使用米/弧度，不再乘以归一化训练头的 5 / π 缩放。
  `WaypointDecoder.denormalise()` 仅保留给显式的归一化输出调用。
- 兼容的同步 `VLMClient.predict_waypoints()` 接口使用 `asyncio.run()`，应从同步代码
  或工作线程调用；不要直接在已经运行的 asyncio 事件循环中调用。
- DeepSeek 排队时可能不断发送保活空行，网络读超时不代表请求总时限。
  参见 [DeepSeek 保活说明](https://api-docs.deepseek.com/quick_start/rate_limit/)。

完整的 DeepSeek、Nav2、G3 和最终速度保护通过统一运行入口启动，见
[统一导航链路](#统一导航链路)。`robot_nav.launch.py` 与 `deepseek_nav2.launch.py` 已默认禁用，仅显式历史开关可启动。

以下 Jetson、vLLM 和本地 Qwen 部署说明保留为历史及可选后端配置，不代表当前默认接入。

## 旧 DeepSeek → Nav2 视觉航点实验

下面的launch只用于复现历史视觉航点实验，不属于当前统一运行链路：

```bash
ros2 launch qwen_robotnav_ros deepseek_nav2.launch.py enable_experimental_waypoints:=true \
  camera_topic:=/robot1/color/image_raw use_sim_time:=true
```

该入口只启动视觉推理节点和 `nav2_waypoint_bridge`，不启动直接速度控制器或真机 SDK 桥。
两节点默认等待 `/mission_state` 的 `exploring` 状态；单独接入时需要由任务状态节点发布该状态。
Nav2 负责代价地图、全局规划、局部跟踪、恢复及速度输出；仿真沿用 G1 的速度缩放器。
不要同时启动旧的 `go2_controller` 向同一个底盘速度话题发指令。
探索目标也应保持单一入口，停用同时向该 Nav2 action 发探索目标的旧发送器。

桥接节点从预测轨迹中选取距离 1–2.5 米的较远航点，按**图像采集时刻**的 TF
转换到 `map`，提交 `NavigateToPose`。当前导航结束前只保留最新预测，不反复抢占。
图像/TF/服务不可用时不发送新目标；视觉输入超时或 `/mission_state` 离开 `exploring`
时异步取消当前目标，取消中继续等待终态，避免接受延迟回调后又发出第二个目标。

默认 TF 话题为 `/robot1/tf`、`/robot1/tf_static`，可通过 `tf_topic`、`tf_static_topic`
调整。`/deepseek/nav2_goal` 发布实际地图目标，`/deepseek/nav2_events` 记录发送、接受、
完成、失败和取消事件。已有 Nav2 栈必须能独立完成雷达避障导航；该入口不会隐式启动定位。

推理调度和输入看门狗使用单调实时时钟；图像和目标坐标仍使用 ROS 采集时间戳，
避免仿真变慢或暂停时网络调度、超时检查也随之停顿。退出时调用 `stop_navigation()`，
它会清除排队预测并锁定停机状态，防止取消终态到达后又提交下一目标。

### 仿真雷达配置与验证

当前外部 Go2 仿真模型的 Velodyne 配置为一条垂直扫描线，但角度范围仍为 ±15°。
联调观测到它向下照到地面，在代价地图里形成封闭的假障碍环，导致目标无法规划。
可生成独立启动副本，仅把该单线雷达的垂直角度改为 0，不修改共享模型文件：

```bash
python3 scripts/prepare_simulator_launch.py --output-dir /tmp/deepseek_nav2_sim
# 在原有 ROS/Gazebo 环境内，确保 qwen_robotnav 包可导入
ros2 launch /tmp/deepseek_nav2_sim/gazebo_nav2.launch.py sensors:=true world:=warehouse.sdf
```

多线雷达及场景障碍保持不变。之后按 G1 流程启动 Nav2，再启动上述 DeepSeek 桥接入口。

2026-10-15 隔离仿真验证：DeepSeek → NavigateToPose → 雷达 costmap → Nav2 → 速度缩放，
首个目标成功，真值净位移 **2.268 m**、到目标误差 **0.217 m**，通过此前阻挡前进的支柱区域，
额外紧急停车护栏干预为 **0**。导航与桥接回归测试 **47 passed**，ROS 包构建和 launch 参数检查通过。
该实验明确使用**仿真真值定位**及上述水平扫描副本，验证的是接线和避障执行，不作为 SLAM/AMCL 或真机验收。

## AMCL 定位与 G3 返航交接

仓库提供两个隔离配置生成器，不修改共享仿真工作区。以下命令在已加载 ROS Humble、
仿真 workspace 且本包可导入的环境中执行，输出目录必须为 ASCII 路径：

```bash
python3 scripts/prepare_simulator_launch.py --single-odom-tf --output-dir /tmp/deepseek_amcl
python3 scripts/prepare_amcl_navigation.py --output-dir /tmp/deepseek_amcl
```

第一份配置将足端里程计放到 `/robot1/leg/odometry`，关闭其 TF，EKF 单独发布
`odom → base_link` 和 `/robot1/odometry/filtered`。融合时只取足端 x/y 位置与原始 IMU
的 yaw/角速度，排除足端消息里反号角速度、未按步态增益换算的命令速度及重复加速度。
第二份启动地图、AMCL 与 Nav2，
用水平 Velodyne 校正 `map → odom`，清除原配置的 `/g1/ground_truth` 依赖。
默认已知出生点为 `(6.86, 6.0, 0)`，可用 `--initial-x/y/yaw` 调整；不会持续读取真值校正。

在同一个隔离 ROS 域和 Gazebo partition 下，启动生成的 `gazebo_nav2.launch.py`，
再启动 `amcl_navigation.launch.py`。Nav2 速度输出为 `/deepseek/nav_cmd`，必须经 G1
`cmd_vel_scaler.py` 送入仿真控制器。该生成器面向当前单机器人 `robot1` 仿真配置。

与 G3 联动时，DeepSeek launch 设置 `require_mission_state:=true`，G3 设置
`require_navigation_handoff:=true`。低电量先广播 `returning` 并发送唯一交接 token；
DeepSeek 桥接清空排队目标，等待其 action 到达终态后确认释放。G3 只接受匹配 token，
再发送返航目标；5 秒无确认进入 ESTOP。这样避免取消探索目标时误伤刚接受的返航目标。
DeepSeek 桥接器与当前 G2 frontier 客户端均实现该交接协议；运行时只启用一个探索目标客户端。

完整的隔离回归入口会自行启动 Nav2、DeepSeek 和 G3，只需预先启动生成的仿真：

```bash
# 环境中需要 API key，并让 G3 Python 包可导入；勿同时启动另一套导航节点。
python3 scripts/validate_amcl_navigation.py \
  --output-dir /tmp/deepseek_amcl --expected-domain 189 --cycles 3
```

测试依次执行真实模型探索目标、模拟低电量、G3 接管返航、再次探索，输出
`report.json`、`trajectory.csv`、`localization.json`、规划路径、事件和截图。
设置 `require_mission_state:=true` 后，返航/待机期间暂停视觉 API 请求，恢复探索时重新推理。
测试里的 Gazebo 真值只用于误差评估和站立保护，不发布导航 TF 或里程计。
它不验证未知地图 frontier、目标检出、真实电池或真机运行。

2026-10-15 三轮实测：6 次到点完成，返航误差 **0.306 / 0.356 / 0.284 m**，
紧急保护干预 0 次。探索目标真值误差 **0.299 / 0.305 / 0.422 m**；
本轮通过标准为探索 action 成功、返航 action 成功且真值距基站 ≤0.4 m，不能视为精确停靠验收。
详见 [验证报告](docs/amcl-navigation-validation.md)。


## 从空地图运行 SLAM / frontier / 返航

当前仿真环境可用一个入口完成配置生成、隔离仿真、在线建图、G2 选点、Nav2 避障、
G3 模拟低电量返航、地图导出、rosbag 记录、离线报告、图表及进程清理：

```bash
./scripts/run_slam_frontier.sh /tmp/slam_frontier_run 5
# 仅准备配置，不启动仿真：
./scripts/run_slam_frontier.sh /tmp/slam_frontier_prepared 5 --prepare-only
```

入口复用本机已配置的 `~/.enroot/ros`、`~/.enroot/ros-gui` 与 Go2 仿真工作区。
默认 ROS domain 192、独立 Gazebo partition；检测到该域已有节点会拒绝启动。
可通过 `SLAM_ROS_DOMAIN_ID`、`SLAM_IGN_PARTITION`、`G1_SIM_WORKSPACE`、
`G1_ROS_WRAPPER`、`G1_GPU_WRAPPER` 指定现有环境。输出目录不能已有 bag，使用 ASCII 路径。
不启动真机 SDK，也不修改共享仿真文件。

SLAM 模式不加载静态地图或 AMCL，`map→odom` 仅由 slam_toolbox 发布。
需要分别启动时，配置生成器使用 `--localization slam`；基站点取 SLAM 启动时的机器人位姿。
G2 在 TF 缺失/过期、地图过期、任务状态不允许时不发新点，等待取消终态后才确认返航交接。
物理节点过滤测地距离小于 0.5 m 的原地点；回归只把 ≥0.75 m 的到点计入有效目标数，
短启动点单列，以适应货架遮挡下的初始小地图。

默认使用 **G2 确定性 frontier**。设置 `SLAM_MISSION_BRAIN=1` 后，DeepSeek 从 G2 提供的安全候选中选择目标，G2 复核后独占 Nav2 探索 action。
设置 `SLAM_PERCEPTION=1` 可同时启用同源 RGB-D、YOLO 和 G3 检测证据记录。独立的 DeepSeek 视觉航点客户端保持停用。

2026-10-15 实测：5 个有效目标 + 2 个启动点，低电量返航误差 0.263 m，
在线已知面积从 3.73 m² 增至 132.62 m²；离线参考自由区域覆盖率 **30.72%**。
最差探索目标真值误差 0.658 m，尚非精确到点验收或全场覆盖完成。
一键入口另以 1 个有效目标跑通，返航误差 0.254 m，已自动停止仿真。
详见 [SLAM/frontier 验证报告](docs/slam-frontier-validation.md)。


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
ros2 launch qwen_robotnav_ros robot_nav.launch.py enable_legacy_direct_control:=true
```

或者覆盖参数：

```bash
ros2 launch qwen_robotnav_ros robot_nav.launch.py enable_legacy_direct_control:=true \
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
ros2 launch qwen_robotnav_ros robot_nav.launch.py enable_legacy_direct_control:=true instruction:="go forward"
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


## DeepSeek 排序与检测证据

```bash
SLAM_LEGACY_VALIDATION=1 SLAM_MISSION_BRAIN=1 SLAM_PERCEPTION=1 SLAM_ROS_DOMAIN_ID=197   ./scripts/run_slam_frontier.sh /tmp/deepseek_frontier_run 5
```

入口读取项目 `.env` 中的 `DEEPSEEK_API_KEY`；可用 `MISSION_BRAIN_*` 覆盖任务脑配置。
模型只接收当前候选点及任务上下文。默认最多 6 点、间隔至少 1 米、15 秒等待上限；返回必须携带匹配的请求 ID。
超时或非法结果使用 G2 原评分最佳点，发点前重查最新地图的间隙、可达性和机器人位姿。
返航和停止会丢弃待处理结果，任务脑的独立 API 进程会被终止。该接口只负责 frontier 排序。

`SLAM_PERCEPTION=1` 使用同一个 D435 RGB-D 传感器的彩色、深度及 CameraInfo，
生成模型中补正光学坐标系；检测关闭“最新 TF”降级。`/perception/status` 记录帧数、图像时刻 TF 和失败计数。
G3 把 `/detections_3d` 与 `/detections_snapshot` 持久化为 `detections/evidence.sqlite3`、`evidence.json` 和 JPEG。
证据 ID 独立于跟踪 ID，消息乱序可配对；无跟踪 ID 的目标或多个不同抓拍无法唯一配对时标为歧义。
当前不会根据普通目标检出自动宣布任务完成。

实测：5 个有效探索目标、11 次真实 DeepSeek 选择、零回退，低电量返航误差 0.240 米；
过程中 2 次 Nav2 失败后重选，参考地图覆盖率 26.94%，最差成功目标位置误差 0.513 米。
本地照片正例已验证实际 YOLO → 三维消息 → G3 JPEG 证据记录，其中深度/TF 为合成输入，不能作为真实 RGB-D 精度证明。
完整证据与已知限制见 [DeepSeek 排序和检测联调](docs/deepseek-frontier-perception-validation.md)。

## 带三维人员目标的侦查任务

```bash
python3 ./scripts/fetch_recon_person.py /tmp/go2_person_asset
SLAM_LEGACY_VALIDATION=1 SLAM_RECON_TASK=1 SLAM_RECON_PERSON_MODEL=/tmp/go2_person_asset \
  SLAM_ROS_DOMAIN_ID=200 ./scripts/run_slam_frontier.sh /tmp/go2_recon_run 1
```

人物模型采用 OpenRobotics 的 CC0 Standing person 第 3 版，资产由 SHA256 清单固定，
无需把大型网格与贴图提交到仓库。场景使用独立 warehouse 副本，模型位置只供离线评测。
本模式自动开启 DeepSeek frontier 排序、同源 RGB-D 和分割检测，默认内缩掩膜 2 像素以排除轮廓边缘的背景深度；质量阈值保持不变。

任务为确认一名 person：同一非零跟踪 ID、置信度至少 0.5、两次不同采集时刻、
间隔不超过 10 秒、位置差不超过 0.75 米，并且两条观测都有可配对 JPEG。
完成指定数量的有效探索目标后，G3 任务节点发出 RETURN，走导航权交接和 Nav2 返航，
以匹配的真实 RETURN action 成功终态确认任务完成。该流程独立于低电量返航验证。

输出包含 `mission_report.json`、`mission_report.md`、`recon_validation.json`、
JPEG 证据、原始 RGB-D bag 和按确认观测时间戳提取的彩色图/深度数组。
离线验收核对人物平面位置误差、抓拍可解码、原始 RGB-D 帧及返航误差。
Z 属于当前 map 坐标系，尚未标定为离地高度；本实验不证明全身三维重建精度或多人检出率。

完整实测：人员平面误差约 0.14 米、返航误差约 0.28 米，任务总用时约 120 秒，录包中的 action 与任务终态均已核对。见 [人员侦查验收报告](docs/recon-person-validation.md)。

多人及实体遮挡场景入口、人数确认规则和实测结果见 [多人及遮挡验证](docs/recon-multi-occlusion-validation.md)。设置 `SLAM_RECON_SCENARIO=multi|partial|full` 后默认要求确认两人；每人仍需两次独立 JPEG 证据，换 ID 不能直接增加人数。

主动绕遮挡观察和搜索预算耗尽后的返航，见 [主动观察验收](docs/recon-active-observation-validation.md)。在侦查入口加 `SLAM_ACTIVE_OBSERVATION=1`；未找齐但回家会记录为 `returned_incomplete`，不算完整任务成功。

## 移动多人身份关联

新增持续身份、运动与抓拍外观关联，以及交叉、并行和遮挡的移动人物仿真入口。
移动场景使用 `SLAM_RECON_SCENARIO=moving_crossing|moving_parallel|moving_occlusion`，
自动启用移动策略；静态场景继续使用原静态去重。
算法边界、复现方法和实际验证见[移动多人跟踪记录](docs/recon-moving-identity-validation.md)。

## 返航停滞与自动回归

私有 Nav2 配置新增基于稳定地图位姿的 `qwen_robotnav_progress` 插件，防止足端打滑里程
被误当作任务进度，并为仓库仿真增加导航余量。单轮入口自动构建和加载插件。
单独生成 AMCL/SLAM 配置时，先在已加载 ROS 的环境中构建并 source：

```bash
colcon build --base-paths ros2_ws/src/qwen_robotnav_progress --packages-select qwen_robotnav_progress
source install/local_setup.bash
```

多轮入口为 `scripts/run_return_regression.py`，包含独立交叉、并行及 18 秒返航零速度保持。
详见[返航诊断与恢复验证](docs/return-progress-recovery-validation.md)。

## 动态障碍仿真

已加入推车横穿与临时堵路基准，记录真实雷达命中、代价地图清障、Nav2 停车/绕行和往返结果。两轮独立隔离仿真通过，84 项导航回归通过；尚未验证实机或移动多人任务与动态避障同时运行。

复现入口：`python3 scripts/run_dynamic_regression.py /tmp/dynamic_nav_suite --start-domain 226 --legacy-validation`。详见 [场景、证据与边界](docs/dynamic-obstacle-validation.md)。

## 移动观察与动态障碍联合任务

新增 `run_joint_recon.py`，将机器人移动、双人取证和推车横穿并行运行，通过 G3 自动返航。安全观察候选由已确认的图像时刻 map 证据生成，DeepSeek 只排序候选 ID；任务支持持续观察时窗和近期证据门槛。参见 [联合场景、独立验收与失败样本](docs/recon-joint-validation.md)。

固定的不同服装双人平行移动与推车横穿场景已完成联合验收：零身份交换/重复，返航误差约 0.24 m，597 项分包回归通过。同衣长遮挡和近距离横穿仍保留为未通过压力样本。

### 统一导航链路

`scripts/run_navigation_runtime.py` 现在始终使用G2作为唯一探索目标所有者。G2从雷达SLAM地图生成并过滤安全Frontier候选；默认由Mission Brain根据任务、信息增益和路径代价选择候选ID，随后由G2用最新地图复核并提交Nav2；离线时可切换为确定性评分。相机视觉航点节点不再由统一入口启动。完整执行链路为：雷达SLAM → G2候选 → DeepSeek选择ID → G2复核 → Nav2 → `/runtime/nav2_cmd` → `/runtime/scaled_cmd` → 速度监护 → `/robot1/cmd_vel`。启动命令与边界见[运行说明](docs/navigation-runtime.md)。
