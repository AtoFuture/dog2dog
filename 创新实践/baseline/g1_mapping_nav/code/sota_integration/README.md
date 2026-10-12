# SOTA 集成包 · 索引与部署序列

> 4 个可选并行产出的**可部署集成包**，落地为真实文件。
> 目标环境：Ubuntu + enroot(`~/.enroot/ros`)、ROS 2 Humble、Ignition Gazebo Fortress(gz-sim6)、
> 命名空间 `/robot1`、TF 全局 `/tf`、slam_toolbox + Nav2、Go2。
> 产出日期：2026-10-13。**均未经运行时验证**，部署时按各包"验证"节逐条确认。

## 部署序列（严格串行，一次一个变量）

| 顺序 | 包 | 风险 | 磁盘 | ROI | 依赖 |
|---|---|---|---|---|---|
| **1** | `explore/` 前沿探索 | 低 | 极小(源码编译,KB级) | **最高** | 现有 slam+Nav2 |
| **2** | `eval/` 量化评价 | **零** | 零(纯Python+evo) | 高(交付) | 无 |
| **3** | `octomap/` 3D→2D 洁净图 | 低 | 小(apt) | 中高 | 3D 点云桥接 |
| **4** | `lio/` LIO 前端 | **高** | 中(0.4–1.2GB) | 中 | §3 之后;磁盘紧张 |

**原则**：每个包只增加"一个消费者"，绝不改动已跑通的 slam_toolbox / Nav2 / TF 拓扑。

## 各包要点（子Agent已对源码核验的关键事实）

- **explore/**：explore_lite 在 Humble **无 apt 二进制**，必须源码编译；Nav2 action 名 `navigate_to_pose`（→`/robot1/navigate_to_pose`）；**Humble 的 tf2_ros 订阅绝对话题 `/tf`,`/tf_static`**（印证我们的全局 TF 修复是对的——绝不能加 `('/tf','tf')` remap）。
- **eval/**：Fortress 的桥接类型串是 **`ignition.msgs.*`**（非 `gz.msgs.*`）；`tf2_msgs/msg/TFMessage` ↔ `ignition.msgs.Pose_V`，且 TFMessage 转换**保留实体名**（PoseArray 会丢）。
- **octomap/**：`ros-humble-octomap-server` 可用；投影话题名 `projected_map`；参数名（`sensor_model.max_range`/`point_cloud_min_z`/`occupancy_min_z`/`filter_speckles`/`latch`）已对源码核验。
- **lio/**：Fortress 的 `PointCloudPacked` **只有 x,y,z,intensity，无 ring/time** → 所有 LIO 的去畸变都会失效 → **必须先用 `gz_lidar_fixer.py` 合成 ring+time**。首选 Point-LIO ROS2(`dfloreaa/point_lio_ros2`)，备选 FAST-LIO2，保底 KISS-ICP；**LIO-SAM/GLIM 因 GTSAM 体积被判红**。

## 硬约束提醒

- 服务器磁盘此前 **100% 满（剩约 40GB）**。包 1/2/3 无压力；**包 4 需先确认磁盘**。
- 服务器为共享机、链路脆弱。**不要并行部署**（多 Agent 同时改一个 ROS2 栈会互抢话题/杀进程）。
