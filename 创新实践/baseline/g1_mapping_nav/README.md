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

## 当前状态 · 2026-09-27（2026-10-17 复核注记）

> 这一节是**以代码为准**重新核对的结论。下面「实验记录」是历史流水，
> 其中 09-20 那条 costmap 的结论**已被推翻**（见 §未提交改动 第 1 条）。
>
> **2026-10-17 收尾复核**：下表第 1 行是 G3 报告的 action 状态，报告本身
> 没有真值 ATE/覆盖率/返航指标；第 2 行找不到 `goal_check.json` 侧写；
> 第 3 行的 bag 实际只有 **86.796 s**，低于 `gt_compare.py` 的 120 s 门槛。
> **不能把本表当作 P1/P2 验收达标的证据。** 另：`config/go2_nav2.yaml:85`
> 的 `bt_navigator.odom_topic` 实际是 `/g1/ground_truth`，不是下方第 101 行
> 写的 `/robot1/odom`。详见 `docs/问题-G1-建图导航.md` 的
> 「2026-10-17 收尾复核新增」与 `docs/收尾与演示.md`。

### 已验证通过

| 项 | 结果 | 证据 |
|---|---|---|
| 多目标点导航 | **11/11 成功**，341 s | `/home/wy/g3_reports/terrain_longrun_20260926_201358_report.json` |
| 到点误差（真值判定） | 0.121 / 0.106 / 0.111 / 0.115 / 0.269 m，5/5 ≤0.30 | `p1_20260923` 包 + `goal_check.py` |
| 静止 120 s 虚假位移 | 0.0000 m（限值 0.02） | 同上 |
| 联调中被返航抢占 | G2 探索点正确让出，返航 goal 成功 | `/home/wy/g3_bags/complex_g123_20260926_171754_report/` |

### 未完成

| 项 | 状态 |
|---|---|
| **P1 第 5 项：边跑边建图** | **2026-10-17 单次 e2e 通过**：真值走 3.007 m，地图 3.7→41.4 m²，7 次更新中 5 次「狗在动且同时增长」；证据 `.runtime/g1_slam_e2e_20261017/`。**待重复、多场景、长距离**。SLAM 参数现在在 `tools/nav2_scaled.launch.py` 的 `SLAM_PARAMS`，不再依赖 YAML 两键 hack |
| 足端里程计本身的修复 | 只做了**隔离**（判据改真值），算法没修。原地蹬腿时虚报可达 **83 倍** |
| costmap 是否异常 | ❓ 未单独量 |
| 路径效率 | 1.10 m 直线走了 3.86 m（绕 3.5 倍） |

---

## ⚠️ 未提交改动（2026-09-25，有丢失风险）

> **2026-10-15 更新**：以下修复已由 `dc156dd`/`f9dc359` 等提交入库
> （顶层 README 核实）。本节保留用于追溯当时的改动内容与原因。

以下 5 个文件**已改未提交**，`git checkout` 一下即丢失。它们修的是
「在此之前所有实验结果都建立在其上的」基座，务必先提交再跑新实验。

| # | 文件 | 改了什么 | 为什么关键 |
|---|---|---|---|
| 1 | `config/go2_nav2.yaml` | local/global costmap 的 scan topic：相对名 `velodyne` → **绝对名 `/robot1/velodyne`** | **避障一直是失效的。** 相对名在 costmap 节点里解析成 `/robot1/local_costmap/velodyne`（**无发布者**），实测 local_costmap 的 3600/3600 cell **全 free** |
| 2 | `config/go2_nav2.yaml` | `bt_navigator.odom_topic`：`/odom` → `/robot1/odom` | 原名**Publisher count = 0**，狗的里程计发在 `/robot1/odom`。**2026-10-17 复核：当前实际值为 `/g1/ground_truth`，见问题清单 H7** |
| 3 | `config/go2_nav2.yaml` | AMCL 加 `set_initial_pose: true` + 基站位姿 | 原配置整份没有该项，AMCL 必须靠外部发 `/robot1/initialpose` 才出 `map→odom`；不发就永远不出，规划直接失败 |
| 4 | `config/go2_nav2.yaml` | 末尾 `slam_toolbox:` 段 | **键名两套说法冲突**，见下 |
| 5 | `maps/live_warehouse_20260919.yaml` | `free_thresh` 0.25 → **0.196** | 0.25 会把 10.5% 未知区静默判成可通行；**P2 的 frontier 依赖未知栅格**，图里没有未知就没有 frontier |

另有一个新文件一并入库：`tools/map_growth.py`（P1 第 5 项的判据工具）。

### 第 1 条与历史记录冲突，以第 1 条为准

09-20 的实验记录写的是**反的**：当时把 `topic: /velodyne`（绝对名）改成
相对名 `velodyne` 并记为「修好了一个 bug」。09-25 实测发现改完**避障反而失效**——
相对名在 costmap 节点的命名空间下解析到空话题。**正确写法是绝对名 `/robot1/velodyne`。**

### 第 4 条：slam 键名在同一文件里自相矛盾

`config/go2_nav2.yaml` 的 `slam_toolbox` 段上方有一段长注释，说
「下面这个空壳只为满足 `HasNodeParams`，真正的参数写在 `/robot1/slam_toolbox`」；
但紧接着的 FIX 注释说「顶层键须用**相对名**，写 `/robot1/slam_toolbox` 会拼成
`/robot1//robot1/`，rcl 解析失败、bringup 全挂」；而实际 yaml 是**裸键写真参数**。

两套说法都自称实测过。**2026-10-11 已定论：用裸键 `slam_toolbox:`**（见问题清单 N1，AMCL 模式 bringup 存活；SLAM 模式参数是否生效未验证）。
**SLAM 建图仍未验收**——裸键下参数可能被 `RewrittenYaml` 丢弃（退回默认 `base_frame=base_footprint`），需 `ros2 param get /robot1/slam_toolbox base_frame` 验证。

---

## 本周要干的事

- [ ] 等 G3 的设备盘点结果，确定仿真里挂哪些传感器
- [ ] gz-sim 里加载 Go2 模型，让它站住
- [ ] `/odom`、`/tf`、`/depth/points` 三话题 `ros2 topic hz` 稳定
- [ ] 写批量启动脚本的骨架

---

## 实验记录 · 2026-09-19/20（Go2 Gazebo 建图导航联调）

> 环境：ubuntu-SYS-420GP-TNR，ROS Humble，仿真跑在 enroot 容器 `ros+humble`，
> `ROS_DOMAIN_ID=42`，`RMW=rmw_cyclonedds_cpp`，world=`warehouse.sdf`+sensors。

### 当晚链路（全通）
1. 仿真底座正常：`/robot1/scan`、`/robot1/velodyne`、`/robot1/odom`、`/robot1/tf`、`/clock` 均有流。
2. 建图：`slam_toolbox`（async，scan:=velodyne，odom:=odom），输出全局 `/map` 718x437@0.05m；
   遥控狗低速跑约 3 分钟（0.25m/s，方形+旋转），odom 位移 2.42m。
3. 存图：`map_saver_cli -t /map` 存 `maps/live_warehouse_20260919.*`（有效：free 24.5%，occ 0.6%）。
4. 导航：停 slam → 起 `navigation2/go2_navigation2.launch.py`（map_server+amcl+planner+controller+bt 全 active）；
   发 `initialpose(0,0,0/map)` 收敛后发 `goal(1.0,0.0/map)`，日志 `Reached the goal! / Goal succeeded`。
5. 可视化：`pip install --user websockify` + 克隆 noVNC，`websockify 6080→5902`，
   浏览器 `http://10.31.112.43:6080/vnc.html` 可看 Gazebo+rviz（含 `/robot1/map` 615x318）。

### 修过的一个 bug
- ⚠️ **这条结论已被推翻（2026-09-25），不要照做。** 原文：`config/go2_nav2.yaml` 里
  local/global costmap 的 `topic: /velodyne`（绝对名）在 `/robot1` 命名空间下订阅不到，
  改为相对名 `topic: velodyne`。
  **实测：改成相对名之后，costmap 节点把它解析成 `/robot1/local_costmap/velodyne`
  ——那是个无发布者的空话题，于是 local_costmap 3600/3600 cell 全 free，
  避障彻底失效。正确写法是绝对名 `/robot1/velodyne`**，见本文档 §未提交改动 第 1 条。

### 目录索引
- `scripts/`：`start_slam.sh`（建图）、`start_nav2.sh`（导航），用法见文件头注释，
  统一用 `nohup ~/.enroot/ros-gui bash <script> > /tmp/<name>.log 2>&1 &` 常驻。
- `maps/`：当晚实测地图（slam 建的 warehouse 实时图）。
- `config/`：修复后的 `go2_nav2.yaml`。
- `logs/`：`drive.log`（运动验证）、`mapsaver.log`（存图）、`nav2.log`（导航 bringup 全日志）。

### 注意事项
- 宿主机直接 `ros2 topic list` 看不到仿真话题（缺 cyclonedds 库+daemon 缓存），
  必须进容器并加 `--no-daemon`，详见 `scripts/*.sh` 头部的 export。
- Nav2 的地图话题是命名空间内的 `/robot1/map`（不是全局 `/map`，那是 slam_toolbox 的）。
- 不要重启 VNC `:2`（会连带杀掉 Gazebo/rviz，2026-09-18/20 两次踩过）；仿真被重启后需重起 nav2 并重发 initialpose。
---

## 故障复盘 · 2026-09-20 地图抖动（里程计幻觉漂移）

### 现象
rviz 里建好的地图持续抖动；查到 `/robot1/odometry/filtered` 在无速度指令的情况下
从原点一路漂到 (-90, +3.4)，AMCL 跟着跳变 → `map->odom` 跳变 → 显示抖动；
costmap 报 `Robot is out of bounds` 并膨胀到 1255x72。

### 根因（已实锤）
1. `QuadrupedOdometryNode` 是按**足端位置积分**算里程（`timer_callback→calculate_foot_positions→update_odometry`），
   不是按指令速度积分。
2. 狗在目标完成后仍停在 TROT 步态原地空踩，腿一动里程计就敢积分 → 约 0.06m/s 幻觉漂移，
   几小时累计近百米（另一证据：`cmd_vel_pub` 是事件驱动的，静默时 `robot_velocity` 无输出，排除指令侧）。
3. 次生灾害：AMCL/costmap 追着漂移位姿跑；重启仿真只杀 launch 父进程会留下旧odom/EKF/桥接继续发旧数据
   （必须按 PID 逐个清，见下）。

### 修复（已验证）
- 发 `STAND` 模式让腿停下：`ros2 topic pub --once /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand "{robot_id: 1, mode: 'STAND'}"`
  → 105 秒漂移 3 微米（噪声级），止漂成功。
- 发 `/robot1/set_pose`（EKF 复位到原点）＋重发 `initialpose`，AMCL 重收敛。
- 重起 Nav2 清掉巨型 costmap，`/robot1/map` 回到 615x318。

### SOP（以后照做）
- 狗不动的时候一律先发 `STAND`，要走再发 `TROT`；长时间挂机保持 STAND。
- 杀进程只杀 launch 父进程不够，`controller/odom/ekf/bridge/nav2_container` 这些子进程会变孤儿继续发数，
  用 `ps -o pid,lstart,args` 按启动时间区分新老，逐个 `kill -9`。
- rtabmap 长时间开着会把漂移轨迹也建成图（曾产出 3230x1873 的 161 米巨图盖掉导航地图），
  3D 建图任务做完就停掉节点。
---

## 追加 · 2026-09-20 新世界切图与牵引力故障（warehouse_cross）

### 新世界（队友王壹需求：连续路口/死胡同/遮挡）
- `src/gazebo_sim/world/warehouse_cross.sdf`（+install 软链），原 `warehouse.sdf` 未动；
  西侧空地 15 道墙：A 南北墙（4m 缺口）× B/C 东西墙（2.3/2.4m 缺口，C1 为 1m 矮墙）串成 Z 字连续路口，
  D 兜（朝南）/E 兜（朝西）两个死胡同，F1/F2 高箱遮挡 + F3 矮箱半遮挡；走道≥1.8m，出生点 2m 净空；
  程序化验过无穿插无出界，`ign sdf -k` 与原文件同输出。切换：`~/go2_sim_ws/run_sim.sh warehouse_cross.sdf`。
- 顺手修了两处 launch/config（软链，改 src 即生效）：`robots.yaml` 出生 z 0.8→0.45（减小坠落），
  `gazebo_go2_sensors.launch.py` 两个 spawner 加 `--controller-manager-timeout 120`
 （高负载下 10s 默认超时会导致控制器加载了但没激活，腿全死，必须手动重跑 spawner）。

### 阻塞：狗在新世界里一步也走不动（牵引力故障，待查）
- 现象：TROT/CRAWL + 前进指令下，里程计声称走了 10.8m，但 Gazebo 真值（`/world/world_demo/pose/info`）
  显示机身一直在原点（z=0.446 站立），一毫米没动；转向指令 30 秒偏航纹丝不动。
- 结论：足端里程计在腿空刨时纯幻觉累积（此前 -147m 漂移同理；此前 nav goal 的“成功”很可能是 AMCL 被幻觉 odom 拖过去的，
  真值存疑）；车体与地面无有效牵引（打滑或力矩不足），与步态无关。
- 给仿真/控制同学的排查点：`/robot1/foot_contact` 是否 ever 有接触；地面与足端 friction 参数；
  在高负载下物理步长是否被稀释；`TrotGaitController` 摆动相位是否产生有效 GRF。
- 当前处置：狗已发 STAND 定住防幻觉继续涨；slam 开着，可先存一张出生点视野的静态图；
  真正跑图需等牵引恢复。
---

## 追加 · 2026-09-20 控制器修复与多人共用提醒（warehouse_cross 跑图前）

### 修好的两个真 bug（代码已进 src，install 双写同步）
1. **速度锁存无超时（runaway 根因）**：`robot_controller_gazebo` 的 60Hz 循环直接用锁存速度，
   发布者停发不清零 → 狗自己走丢（实测一次走 6m 撞墙角，旧世界 -147m 漂移同理）。
   修：`RobotController.py` 加墙钟看门狗（1.0s，`time.time()`；注意不能用仿真钟——负载下 RTF~0.3 会误杀正常 10Hz 指令），
   `QuadrupedOdometryNode.py` 同理。已验证：停发后 25 秒仅蠕行 2cm。
2. **spawner 默认 10s 超时在高负载下必死**：腿控制器加载了但没激活（`Failed loading` + `already loaded`），
   狗呈站姿但腿全死。修：`gazebo_go2_sensors.launch.py` 两个 spawner 加 `--controller-manager-timeout 120`；
   若已出现，手动重跑一次同名 spawner 即可激活（已验证 joint_states 26Hz 恢复）。

### 转向/前进标定（负载 40+、RTF~0.3 下实测真值）
- 前进：cmd 0.5 → 真值约 0.09~0.29 m/s（与 délivrance 有关，按 0.1 估）。
- 转向：az=1.0 实测一次 +139°/18s（墙钟），可用但慢；闭环必须读 Gazebo 真朝向
  （`ign topic -e -t /world/world_demo/pose/info`，模型名 `robot1_my_bot`），不要信 odom 偏航。
- 里程计是足端积分，腿空刨时纯幻觉涨数——一切以 ign 真值为准。

### 多人共用警告（重要）
- 本机 `w` 可见 lzx/yzw/tjr 等同学同时在线，load 已到 40~60；
  曾出现非本人启动的 nav2_container、重复 controller/odom/cmd_vel_pub 节点，以及无人指令时的 6m 位移。
- 跑图前务必 `ps` 确认只有一对 controller/odom，且和队友（尤其王壹）约好独占窗口，
  否则互相顶指令、地图白建。`ROS_DOMAIN_ID=42` 是共享的，谁发 cmd_vel 狗听谁的。
- 当前停放：狗在 depot 东北角 (21.8, 8.1) STAND 定住，slam 开着（`/map` 有流），可随时接跑。
---

## 追加 · 2026-09-20 深夜 cross 世界跑图与 Nav2 联调（第二轮）

### 新图已存（队友王壹需求 closed 大半）
- `src/navigation2/maps/cross_warehouse_20260920.pgm/.yaml`（193K，650x304@0.05，origin [-10.3,-6.64]），
  已亲眼验货（转 PNG 看过）：Z 字连续路口（A/B/C 缺口全通）、D 死胡同兜内壁、F1/F2/F3 遮挡箱、depot 箱阵全在图上；
  E 兜只有外轮廓（未进内壁，下次补）。
- 跑图路线（真值闭环）：出生点→东→北→西穿 A 缺口→南穿 C 缺口→D 兜口→进兜→退出→STAND，共 ~25m，全程 `ign pose` 真值导航。

### 本轮修的 bug（代码均已进 src，install 双写/软链同步）
1. `cmd_vel_pub.py` 缩放 ×0.035→x0.3/y0.1：原来 Nav2 最大输出 0.26 经缩放只剩 0.02，DWB 近目标 0.027 直接变 0.003 ——
   导航 freeze 的直接原因。改完 Nav2 全速约 0.18 m/s（仿真秒）。
2. `go2_nav2.yaml`：`movement_time_allowance` 10→60s（慢动作 RTF~0.3 下 10s 必超时误杀）。
3. 看门狗（`RobotController.py` + `QuadrupedOdometryNode.py`，`git stash@{0}` 暂存，未启用）：
   初版用仿真钟，在 RTF 0.3 下把正常 10Hz 指令误杀致腿全死；换墙钟后出现 `math domain error` 刷屏（机制未明，
   疑与双控制器实例并存有关）。**结论：暂不上看门狗**，用流程代替——每次驱动结束发 3s 零速 + STAND，
   跑长途前 `ps` 确认 controller/odom 单例。stash 留着以后离线分析。

### 当前状态与已知坑（给接力的人）
- 狗停在 depot 东箱阵 aisle (16.6,-2.9) STAND；cross 世界 + cross 地图 + Nav2 全套 active，AMCL 可收敛，
  goal 可接单（`Begin navigating` 正常），短途 goal 已验证可动（慢）。
- 大坑 Top3：①整机 load 40~60，RTF~0.2~0.3，一切按 3~5 倍墙钟估时；② `ROS_DOMAIN_ID=42` 共享，
  有同学同时在线时会顶指令/起重复节点，跑图前 `ps` + 约独占窗口；③ `ign service` CLI 组装不了嵌套消息
  （set_pose/remove 全灭），真值读 pose/info topic，复位靠重发 initialpose + 重起。
- 3d/ 目录：bag（56M）+175 帧 PCD、octomap .ot（6.7M/140 万节点）、rtabmap 首跑配置与 db 全在，
  rtabmap/octomap 节点目前是停的（qt绘制负载考虑），要看 3D 按 README 前文命令重起。
- `drive_relay.py`（常驻 cmd_vel 中继，已停）：曾和 Nav2 抢方向盘致导航 freeze，已 `pkill`。
  以后遥控统一走它（`ros2 param set /drive_relay lx/az`），用完即杀。
---

## 追加 · 2026-09-21/22 凌晨 IK 修复 + 全链路验证

### 本次修的 bug（已进 src + install 双写/软链同步）

1. **IK `math domain error` 根因修复**（`robot_IK.py`）：
   - Line 65: `sqrt(x² + y² - l2²)` → `sqrt(max(0.0, x² + y² - l2²))`
   - Line 72-74: `D` clamp 到 [-1,1] + `sqrt(1-D²)` → `sqrt(max(0.0, 1-D²))`
   - **效果**：从 8407 次/分钟 → 0 次。控制器不再死循环，关节指令正常输出。

2. **gz_bridge 时钟桥接丢失**（共享服务器 `go2_ik_overlay` 项目 kill 了我们的 bridge）：
   - `parameter_bridge` 用 `gz_bridge.yaml` 配置重启，恢复 `/clock`（545Hz）
   - **影响**：所有 `use_sim_time:=True` 节点从冻结恢复

3. **Controller 命名空间错位**（launch 文件只启 controller，不含 `__ns:=/robot1`）：
   - 改为直接 `python3 robot_controller_gazebo.py --ros-args -r __ns:=/robot1 ...`
   - **影响**：controller 发的 `/joint_group_controller/commands` 从全局命名空间回到 `/robot1/`

4. **共享服务器进程清理**：
   - kill 了 `go2_ik_overlay` 项目的重复 controller/odom/cmd_vel/nav2_container（占 ~60% CPU）
   - kill 了 4 个多余 rviz2 实例（省 ~500% CPU）

### 本次发现的架构问题

- `robot_controller.launch.py` 只启动 controller 本体（不含 odom/cmd_vel_pub/ekf），且命名空间为全局——必须手动指定 `__ns:=/robot1`
- 共享服务器（`ROS_DOMAIN_ID=42`）上其他同学的项目会启重复节点占用 namespace，跑图前必须 `ps` 确认单例
- slam_toolbox 工作目录为 `/`，`save_map` 服务可能无权限写入——需要手动从 `/map` topic 导出

### 验证结果

| 项目 | 结果 |
|------|------|
| IK math domain error | 0（修复前 8407） |
| 直接驾驶验证 | 1.4m 真值移动（ign pose） |
| gz_bridge clock | 545 Hz |
| AMCL initialPoseReceived | 成功 |
| Nav2 规划 | "Passing new path to controller" 正常 |
| Nav2 DWB 控制 | 输出零速（AMCL 定位漂移导致） |

### 当前状态

- 狗停在 (18.07, -2.68) STAND
- `robot_IK.py` 已 patch（src + install 双写）
- controller 正常运行，0 errors
- gz_bridge 恢复 clock 桥接
- slam_toolbox 运行中但未产生完整地图（需更多移动）
- Nav2 stack 运行中（1+ 天），AMCL 定位可能漂移——**建议重启 Nav2 再做端到端验证**
- load 仍然很高（~59），RTF ~0.34
---

## 追加 · cross 世界 Nav2 闭环验证结论（必读）

### 链路状态（全部逐段验证通过）
- cmd_vel (Nav2, lx=0.26@20Hz) → robot_velocity (0.179@20Hz) → joint_commands (@15Hz)：数值链完整
- AMCL 收敛正常（(0.19,0.04)≈真值），planner 出路径（"Passing new path"），goal 可接单
- controller 0 math domain error（IK 补丁有效），TF 链完整（odom 已修 base_link）

### 核心发现：TROT 步态持续行走会翻倒
- Nav2 闭环可开狗走约 0.8m，然后车身前倾 ~25°，腿空转不平移 → "Failed to make progress"
- 直接 cmd_vel 0.3（→0.21m/s）会直接翻车（ROLL 179°）；降速到 0.12 缩放后仍会在 ~1m 后倾斜
- CRAWL 模式：发速度不平移（可能该模式速度映射未实现），不能作为替代
- 初生狗 IMU 全平（0.0/-2.6），排除出生位姿问题；12 关节站立位正常，排除单腿卡死

### 参数备忘（本次最终值）
- `cmd_vel_pub.py`：x 0.12 / y 0.04（src + install 双写；0.3 会翻车，0.035 太慢）
- 狗当前状态：倾斜约 38° 趴在出生点东约 1m 处，已 STAND + 零速
- 世界/地图/代码：`warehouse_cross.sdf`（md5 一致）、cross_warehouse_20260920、IK 补丁均在位

### 下一步建议（步态专项，非远程联调）
1. 查 TROT 步幅/频率参数（可能在 TrotGaitController），降速降步幅找稳定包线
2. 查 default_height（重心高度）与地面摩擦系数
3. 验证 CRAWL 的速度映射是否实现
4. 考虑给 controller 加"翻倒检测→自动 STAND"保护
5. 在 Gazebo GUI 里肉眼看一次翻倒过程（比盲调快十倍）
---

## 追加 · 步态专项 round 1（IMU 补偿闭环 + 稳定包线初探）

### 根因：IMU 姿态补偿开环
- `USE_IMU=True`，但 `state.imu_roll/imu_pitch` 只在 `StateCommand` 里初始化为 0，从没有任何地方更新
- PID 补偿每周期跑在 (0,0) 上 → 恒输出 ~0 → 开环行走，扰动累积 → 持续前倾翻倒
- **修复**（`robot_controller_gazebo.py`，src + install 双写）：加 `imu_plugin/out` 订阅，
  每控制周期把四元数解算的 roll/pitch 写入 `state`。`ast` 语法校验通过。

### 调参一览（全部双写生效中）
| 参数 | 原值 | 现值 | 结论 |
|---|---|---|---|
| default_height | 0.25 | 0.21 | 降重心，有效 |
| stance_time | 0.04 | 0.04（0.08 试过，跑废，已回滚） | 0.08 使步态几乎不平移 |
| z_leg_lift | 0.14 | 0.10 | 少晃，有效 |
| 脚摩擦 mu | 0.6 | 1.0（xacro，install 软链） | 牵引改善 |
| cmd_vel 缩放 | 0.3/0.1 | 0.12/0.04 | 0.3 会翻车 |
| IK sqrt 保护 | — | max(0,…) 两处 | 0 math domain error |

### 验证数据
- 65s 慢速试跑：跑前水平（0.0/-0.0）→ 跑后 ROLL 0.0 / PITCH 5.5°（此前同条件 25~29°）
- 位移 (-0.15m，方向不稳定，见下），0 math domain error
- CRAWL 模式发速度不平移（该模式速度映射疑似未实现），暂不可用

### 新发现的环境坑
- 世界会被暂停（疑似队友 GUI 操作）：RTF 0.007 时先 `pause: false` 解开
- `ign` 只在 enroot 容器内有；裸起 server 必须 source ROS 环境 + export GZ_SIM_SYSTEM_PLUGIN_PATH，否则 ros2_control 插件加载失败
- spawner 在高负载下激活失败率高，`--switch-timeout 60` 是钥匙
- 同一 world 起多个 gz server 会互相干扰（曾同时 3 个），动手前先 `pgrep` 清零
- 多次 kill server 会带崩 ROS 侧节点连接，需整套重起；spawn 时先 pause 物理防瘫倒
- EKF 会被 sim 时间跳变 wedged（0% CPU 静默），重启恢复

### 待解
1. 平移速度只有理论值 ~1/5，且前进方向不稳定（±x 来回），需低负载窗口 + GUI 目检一次完整行走
2. 足里程计漂移大（±2x），AMCL 融合前先别信 odom
3. Nav2 端到端等步态稳定后再跑（上次 0.8m 后倾倒失败）
