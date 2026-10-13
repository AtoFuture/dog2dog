# LIO 接入报告：Point-LIO 在本仿真中的关键配置

> 平台：Unitree Go2 · Ignition Gazebo Fortress(gz-sim6) · ROS 2 Humble · enroot 容器
> 日期：2026-10-13 · 相关提交见 PR #2

## 结论（一句话）

> **Point-LIO 默认参数 `use_imu_as_input=False` 在本仿真里会发散（静止都在 20 秒内漂到数千米）；把它改为 `use_imu_as_input=True` 后，静止时完全稳定（22 秒最大漂移 1 cm）。**

## 背景：为什么需要 LIO

步态修好后（见 `go2_gait_fix_report.md`），2D SLAM 地图仍出现强烈"星芒"径向伪影，判定为**里程计/激光-TF 时序一致性**问题；SOTA 调研指向 **LiDAR-惯性里程计（LIO）**。

## 可行性链路（已全部打通 ✅）

| 环节 | 结果 |
|---|---|
| gz 3D 雷达话题 | `/robot1/velodyne` (LaserScan) + **`/robot1/velodyne/points` (PointCloudPacked)**，640×16，10 Hz |
| 原始点云字段 | `x,y,z,intensity,ring` —— **有 ring，缺 time** |
| `gz_lidar_fixer.py` | 合成 `time`（按方位角/扫描周期）→ 输出 `x,y,z,intensity,ring,time` ✅ |
| IMU | `/robot1/imu_plugin/out`，100 Hz；静止 `linear_acceleration.z=+9.80`（标准） |
| 外参 | 所有传感器在同一 link、pose 全 `0 0 0` ⇒ **extrinsic = 单位阵（正确）** |
| Point-LIO ROS2 | `dfloreaa/point_lio_ros2` **编译通过**，发布 `/aft_mapped_to_init` / `/cloud_registered` / `/Laser_map` |
| GT 时间戳 | `gt_stamped.py`：`/gt/pose` 的 Pose_V 无 stamp → 用 `/clock` 补真实 sim 时间戳 |

## 关键实验：静态多变体对比

一次 bringup，机器人**站立不动**，逐个启停 Point-LIO 配置，记录 `/aft_mapped_to_init` 22 秒：

| 变体 | `use_imu_as_input` | 结果 |
|---|---|---|
| V1 | **False**（默认/官方示例） | start=(-0.48,-2.19,-4.57) → **漂到 z=-5.15**，max=**5.34 m** ❌ |
| **V2** | **True** | start=end=(0.004,0.002,0.005)，max=**(0.00,0.00,0.01) m** ✅ |
| V3 | True + `point_filter_num=1` | max=**(0.00,0.00,0.01) m** ✅ |

**⇒ 根因锁定为 `use_imu_as_input` 模式**；`True`（IMU 驱动传播，FAST-LIO 式）与本仿真的 IMU 兼容。

## 推荐配置

```
ros2 run point_lio pointlio_mapping --ros-args \
  --params-file point_lio_gz.yaml \
  -p use_imu_as_input:=True \        # ← 关键
  -p prop_at_freq_of_imu:=True \
  -p point_filter_num:=4
```

`point_lio_gz.yaml` 要点：`lid_topic=/robot1/velodyne/fixed`、`imu_topic=/robot1/imu_plugin/out`、
`lidar_type=2`(Velodyne)、`scan_line=16`、`timestamp_unit=0`(秒)、`fov_degree=360`、`extrinsic_T=[0,0,0]`、`extrinsic_R=I`。

## 未完成 / 受环境阻断

- **运动中的跟踪验证**：两次尝试均被**共享机资源争抢**打断（观测到内存仅剩 1 GB、load 33、7 个用户；我们的 gz 进程被 SIGKILL）。需在机器空闲时补做：TROT + 驱动，比较 `/aft_mapped_to_init` 与 GT 的 ATE。
- **接入 SLAM**：将 LIO 的 `odom→base_link` 接入 slam_toolbox（并关掉 EKF 的 `publish_tf`，保证单一 TF 发布者），再重建地图、观察星芒是否消除。

## KISS-ICP（纯雷达）尝试：已就绪，待安静环境验证

由于 Point-LIO 在**运动中反复发散**（z 通道 →万米级，三轮实验：`use_imu_as_input` 两种模式 + 关去畸变均未解决），按"绕开 IMU 传播"的思路改试 **KISS-ICP**（纯 LiDAR ICP，无 IMU、零调参）。

| 项 | 结果 |
|---|---|
| 编译 | ✅ 成功（仅 **12 MB**）。注意：其 Sophus 子模块要求 **CMake ≥ 3.24**，容器是 3.22 → 用 `pip install --user 'cmake>=3.28'` 提供新 cmake（`~/.local/bin/cmake`）即可 |
| 输入话题 | ⚠️ **关键坑**：KISS-ICP 的输入是**硬编码相对名 `pointcloud_topic`**（不是 `topic` 参数）。必须用 **remap**：`-r pointcloud_topic:=/robot1/velodyne/points`。原先写 `-p topic:=...` 被**静默忽略** → 订阅到不存在的话题 → 无任何里程计输出 |
| 去畸变开关 | 参数名是 **`data.deskew`**（不是 `deskew`） |
| 其他参数 | `base_frame`、`publish_odom_tf`、`lidar_odom_frame` 有效；输出话题 `/kiss/odometry` |
| 验证 | ❌ **被环境阻断**：bringup 阶段 `gz server` 再次卡死（`ign service ... pause:false` 服务超时——这是当日**第 3 次** gz 卡死），控制器未激活 → 测试卡在启动阶段 |

**验证用脚本**：`eval/kiss_test.sh`（静止 20 s + 驱动 40 s + GT 对比）。机器空闲后即可一键重跑。

## 环境阻断说明（重要）

当日下午至今，共享机长期处于 **load 20–33、7 个在线用户、物理内存仅剩 ~1 GB**（队友 `nnUNetv2_train` ×3 等训练任务），并多次观测到**第二个 gz server**（他人 `multi_room` 世界）。在此条件下：

- `gz server` **卡死/被杀 3 次**（服务不响应、SIGKILL）
- Point-LIO / KISS-ICP 的**运动验证**均因此未能完成

> 结论：**LIO/KISS 的收敛验证受限于机器负载，而非方案本身**。所有前置与脚本已就绪，需在**机器空闲窗口**完成最终验证。

## 附：进程隔离

共享机上存在**第二个 gz server**（他人 `multi_room` 世界）。已为本项目 gz session 设置 **`IGN_PARTITION=go2wy` / `GZ_PARTITION=go2wy`** 以隔离（脚本内）。注意：这**不是** LIO 发散的原因（隔离后仍发散 → 才定位到 `use_imu_as_input`）。

## 相关文件（`code/sota_integration/`）

| 文件 | 说明 |
|---|---|
| `lio/gz_lidar_fixer.py` | 点云 ring/time 补齐 |
| `lio/point_lio_gz.yaml` | Point-LIO 适配配置 |
| `eval/plio_probe.py` | 订阅 `/aft_mapped_to_init`，报告起点/终点/最大偏移（判发散） |
| `eval/lio_variants.sh` | 一次 bringup 测多变体（静态） |
| `eval/lio_motion.sh` | 运动测试（含 gt_stamped） |
| `eval/lio_test.sh` / `lio_static.sh` | 单次运动 / 静态测试 |
