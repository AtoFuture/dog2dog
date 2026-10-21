# 仿真中"行人/车辆自由移动"的 SOTA 与最佳实践

> 目标：让仿真里的行人/车辆**平滑移动（不瞬移）、不穿模**。
> 结论先行：**Gazebo Sim 里移动人物/车辆的最佳实践是 `<actor>`（物理无关 + sim-time 插值轨迹）**；
> 需要与物理交互时，才用"物理刚体 + 运动控制器"，或自写 system plugin 在 `PreUpdate` 里做运动学位姿更新。

## 1. 我们之前为什么瞬移（反面教材）

| 根因 | 说明 |
|---|---|
| **多个驱动进程** | 会话里残留了 **3 个** `objects_motion.py`，同一条轨迹被不同相位同时 set_pose → 对象被来回拽 → 瞬移 |
| **外部进程驱动 + 服务发现开销** | 没有 `set_pose` 主题，只有 service；每 tick `fork/exec ign service` ≈ **1.3s**，实际更新率 ~0.75Hz → 每次跳 `speed×1.3s` |
| **用 wall-clock 积分** | 负载下 RTF 抖动，位置与仿真相位不一致 |

**教训**：**不要从外部进程逐帧 set_pose 来"平滑"移动**。要么把运动放进仿真内部（actor / system plugin），要么用物理驱动。

## 2. SOTA：Gazebo `<actor>`（官方推荐，本项目采用）

来源：Gazebo 官方 "Actors" 教程（`gazebosim.org/docs/*/actors`）。actor 的特性：

- **完全不受物理影响**（无重力/无碰撞/无接触力）→ 不会坠落、不会被顶飞
- **由仿真按 sim-time 在 waypoint 间平滑插值**（Catmull-Rom；`tension∈[0,1]` 控制贴合度）→ **天然不瞬移**
- 支持骨骼动画（COLLADA `.dae` / BVH），也支持纯轨迹移动任意网格
- 轨迹直接写在 SDF 里：`<script><trajectory type="walk"><waypoint><time>..</time><pose>..</pose></waypoint>...`
- `loop/auto_start/delay_start` 控制循环与启动

官方行人资产：Fuel `Mingfei/models/actor`（`walk.dae / run.dae / stand.dae / talk_a.dae ...`），CC 许可。

**本项目做法**：人用 `walk.dae` actor；车把 `SUV.obj` 用自带小工具转成 `.dae`（烘焙旋转+缩放）后同样当 actor。
→ **不再需要任何外部驱动进程**，位移全部由仿真在 sim-time 下插值，**瞬移问题根除**。

## 3. 其它方案对比

| 方案 | 平滑 | 不瞬移 | 与物理/碰撞交互 | 说明 |
|---|---|---|---|---|
| **`<actor>` + `<trajectory>`（采用）** | ✅ | ✅ | ❌ 忽略碰撞 | 官方最佳实践；适合"开放环路"的人流/车流 |
| `TrajectoryFollower` system | 力驱动 | 一般 | ✅ | 实测把 80kg 行人**掀翻**、车**卡死**（力作用于 link 非质心） |
| 自写 **gz-sim system plugin**（`PreUpdate` 里 `SetWorldPoseCmd`/`WorldPose`） | ✅ | ✅ | ❌（除非再做避障） | 想自定义路径/避障逻辑时的正解；本项目未采用 |
| **物理刚体 + 控制器**（DiffDrive/Ackermann/VelocityControl） | ✅ | ✅ | ✅ 真碰撞、不穿模 | 最真实，但要调参；窄通道里车会真撞墙 |
| 外部脚本 set_pose（旧方案） | ❌ | ❌ | ❌ | 见第 1 节，已废弃 |

**行人群体 SOTA（若需"人-人/人-机"避让）**：social-force / ORCA 类社会力模型，ROS 生态用 `pedsim_ros`、`Menge`(C++ ORCA)、`crowd_sim`；这些通常**驱动** actor/模型，可与本方案叠加。

## 4. 不穿模的最佳实践

- actor / 纯运动学体**忽略碰撞** → 轨迹必须落在**真实空闲通道**。判断空闲**不能只看碰撞体**：本项目 `Depot` 的货架/墙**只有视觉没有碰撞体**（其碰撞只是一个 100×100 平面）。用**占用栅格图**（SLAM 图 / `sdf→GT` 图）或仓库地面**通道标线**来定线。
- 需要硬性"撞不过去"时→改用物理刚体；或把路径**沿通道中心线**规划（本项目：人走货架间通道、车走南侧黄线通道）。
- 尺寸要匹配：Fuel `SUV` 原始 **5.02×2.60m**，而通道仅 ~2m → 必须缩放，否则必穿模。

## 5. 本项目落地

- 世界：`src/gazebo_sim/world/warehouse_cross_objects.sdf`（原迷宫 + 5 个 actor）
  - `person_1/2/3`：`walk.dae` actor，速度 ~0.3 m/s，走货架间通道
  - `suv_1/2`：`suv.dae` actor，速度 ~0.8 m/s，走南侧通道
- 工具：`tools/obj2dae.py`（OBJ→COLLADA，含旋转/缩放烘焙），把 `SUV.obj` 转成 actor 皮肤
- 资产：`models/{walking_actor,suv_vehicle}`（`GZ_SIM_RESOURCE_PATH` 已指向 `models/`）
- **无需外部驱动**：`tools/objects_motion.py` 已废弃

## 6. 参考

- Gazebo 官方 Actors 教程（actor/skin/animation/script/trajectory）
- sdformat `<actor>` 规范
- Gazebo Fuel：`Mingfei/models/actor`
- 行人避让：pedsim_ros / Menge(ORCA) / social-force model
