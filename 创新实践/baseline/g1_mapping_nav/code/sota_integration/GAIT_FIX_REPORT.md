# 四足机器人姿态翻转问题定位与修复报告

> 平台：Unitree Go2 · Ignition Gazebo Fortress · ROS 2 Humble
> 组件：自定义对角小跑（trot）IK 控制器 `quadropted_controller`、2D SLAM + Nav2
> 结论：**姿态补偿 PID 因时间源错误而恒为无效，导致机器人全程开环行走并在转向时侧翻。已修复并验证。**
> 修复提交：`ce40f444`（PR #2）

---

## 现象

- 转向过程中**间歇性侧翻**，横滚角（roll）最低达 **-87°**（完全翻倒）。
- 导致建图与评测**多次中断**，无法获得完整地图与有效评测。
- 故障间歇发生，易被误判为随机扰动。

## 根因

**文件**：`src/quadropted_controller/scripts/RobotController/PIDController.py` · 方法 `get_time_in_seconds()`

1. **时间源错误**：原实现用 `rclpy.time.Time()`（未传 clock 节点）→ **恒返回 0**。
2. **步长恒为 0，PID 被短路**：`step = t_now - last_time = 0`，代码 `if step < 1e-6: return [0.0, 0.0]` → **每次调用都返回 [0,0]**。
3. **姿态补偿被完全旁路**：

   ```python
   compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
   rot = rotxyz(-compensation[0], -compensation[1], 0)
   new_foot_locations = rot @ new_foot_locations
   ```
   `compensation ≡ [0,0]` → `rot ≡ 单位阵` → **姿态反馈完全失效，纯开环行走** → 转向时侧翻。

## 修复

`get_time_in_seconds()` 改用 `time.monotonic()`；并将 `step` 钳制到最大 **0.2 s**（防长停后 D 项尖峰）。
**控制增益不变**：`kp=0.15, ki=0.02, kd=0.002, max_I=0.2`。

## 验证

| 指标（最小栈 + TROT + 60s 原地连续转向, angular.z=0.06） | 修复前 | 修复后 |
|---|---|---|
| roll 峰值 | **-87°** | **2.9°** |
| pitch 峰值 | — | 1.8° |
| 是否侧翻 | **~10s 内翻倒** | **未翻倒** |

补充：25s 转向 → 偏航变化 **50.5°**（~2°/s），全程 roll≈0；完整脚本化扫描 **7~8 段零侧翻**；地图由 ~141KB（早期中断的稀疏图）增至 ~668KB / 1.7MB。

**部署注意**：修复为**双写**（`src/...` 与 `install/.../lib/quadropted_controller/RobotController/`，**实际运行的是 install 副本**）；已清理 `__pycache__`。

## 对既往结论的更正

> **作废**：早期记录"加入 IMU 使姿态 25°→5.5°"。因 PID 全程空转，该改善**在因果上不可能来自此补偿**。请勿再引用该数字作为 IMU 补偿有效的证据。

## 遗留问题（独立于本修复，未解决）

- **原地旋转/移动中的"星芒（starburst）"径向射线伪影**：后续地图中强烈存在。
  - 实验结论：**改用弧线转弯（前进+转向）后星芒不减反增** → **不是由原地旋转引起**。
  - 判定：属**里程计 / 激光-TF 时序一致性**问题。
  - 方向（未实施）：引入 **LiDAR-惯性里程计（LIO）**（见 `sota_integration/lio/`，含 Fortress 必需的 `gz_lidar_fixer`）。

## 涉及文件

| 文件 | 说明 |
|---|---|
| `src/quadropted_controller/scripts/RobotController/PIDController.py` | 修复目标（时间源 + step 钳制） |
| `src/quadropted_controller/scripts/RobotController/TrotGaitController.py` | 姿态补偿调用点（未改参数） |
| `install/.../lib/quadropted_controller/RobotController/` | 运行副本，需同步双写 |
