# G1 · 建图导航

> 仿真环境 · SLAM · 定位 · Nav2 · 避障

**人**：A（仿真 → SLAM → 外参标定） · B（Nav2 → 避障 → 真机调参）

---

## 交付物

| # | 交付物 | 验收标准 |
|---|---|---|
| 1 | 可运行仿真环境 | gz-sim 里 Go2 站得住，`/depth/points`、`/odom`、`/tf` 三话题稳定发布 |
| 2 | 栅格与点云地图 | RTAB-Map 输出 `OccupancyGrid` + `PointCloud2`，回环能收敛 |
| 3 | TF 与定位输出 | `map → odom → base_link` 树完整，定位漂移 < 0.15 m |
| 4 | 可执行的导航栈 | 收到 `NavigateToPose` 能自主走到，动态障碍能绕 |

---

## 任务分解

### ① 仿真环境（第 1–4 周）

**路线 A：ROS2 + gz-sim，自行移植 Go2 模型**（已定，见 `docs/选题与规划.md` 7.2）

宇树官方**没有** ROS2 + Gazebo 的包：`unitree_ros` 是 ROS1 时代的（有 Gazebo 但是 ROS1），`unitree_ros2` 只有通信层。所以走自行移植。

要点：

- 从 `unitree_ros/robots/go2_description` 取 URDF，转 SDF 或用 `sdformat` 的 URDF 支持直接加载
- 传感器按**真机实际配置**挂（等 G3 第 1 周设备盘点结果，别自己假设）
- gz-sim 用现成插件：深度相机、IMU、GPU 激光
- **可无头运行**（`-s` / `ogre2` 关渲染），几十人共用的机器上别开着 GUI 吃资源

**第一周唯一必须拿下的**：Go2 站住 + 三个话题发布稳定。场景丑没关系，跑得动最重要——这三个话题一通，RTAB-Map 和 Nav2 当天就能接上去。

批量测试脚本也在这阶段做：一键起仿真、跑一圈、落 rosbag、关掉。后面每次改参数都要用。

### ② SLAM 与定位（第 3–8 周）

- **RTAB-Map**（不是文档里写的 Cartographer——那个 2024-01 就停更了）
- **robot_localization** 做 EKF：融合 IMU 与腿式里程计，输出平滑的 `odom → base_link`
- 足式平台的坑：机身晃动导致点云匹配失败、回环误检。对策是降扫描速度 + 调回环参数
- 外参标定：相机与 IMU、相机与机体的相对位姿。**这部分数据要从 G3 的设备盘点拿**，不复现的话仿真里调好的参数上真机全偏

### ③ Nav2 与避障（第 7–12 周）

- costmap 分层配置、全局与局部规划器、恢复行为
- 接收 G2 的 `NavigateToPose` —— 这是你们唯一的对外接口
- 真机部署时把仿真里调好的参数搬过去，技术栈一致（都是 ROS2）

---

## 对外接口

**发出去（接口 01）**：
- `nav_msgs/OccupancyGrid` — 栅格地图
- `sensor_msgs/PointCloud2` — 点云地图
- TF：`map → odom → base_link`

**收进来（接口 02）**：
- `NavigateToPose` action — 来自 G2 的探索目标点，以及 G3 的低电量返航点

> 这三样（TF、costmap、路径）是 SLAM 和 Nav2 共享的，也是**为什么这两件事在同一组**：建图漂了还是导航配错了，在调试上分不开。

---

## 本周要干的事

- [ ] 等 G3 的设备盘点结果，确定仿真里挂哪些传感器
- [ ] gz-sim 里加载 Go2 模型，让它站住
- [ ] `/odom`、`/tf`、`/depth/points` 三话题 `ros2 topic hz` 稳定
- [ ] 写批量启动脚本的骨架
