# 仓库世界加"人员 / 车辆"

`warehouse_cross_objects.sdf` = 原迷宫 + **3 个行人 `<actor>`** + **2 辆车（带轮整车 + 官方 AckermannSteering）**。

| 类别 | 方案 | 特性 |
|---|---|---|
| 行人 | `<actor>` + `walk.dae` + 轨迹 | 物理无关、sim-time 平滑插值 → 不瞬移、不坠落 |
| 车辆 | **带轮整车**（车身+4轮+转向关节）+ 官方 `ignition::gazebo::systems::AckermannSteering` + 自写 `TrajDrive`（路径→`cmd_vel`） | 真物理体：**被机器人顶到会被推动**、**遵守碰撞**、**干净直行不漂移** |

## 文件

| 文件 | 说明 |
|---|---|
| `src/gazebo_sim/world/warehouse_cross_objects.sdf` | 世界 |
| `tools/trajdrive/traj_drive.cc` | **路径→`/model/<name>/cmd_vel` 发布器**（pure-pursuit + 到头倒车） |
| `tools/trajdrive/build.sh` | 编译并装到 `~/.ignition/gazebo/plugins/`（gz 默认加载） |
| `tools/gen_objects_world.py` | 由 `warehouse_cross.sdf` 生成该世界（可改路径/速度） |
| `SOTA_MOVING_ACTORS.md` | 方案调研/最佳实践 |

## 编译插件（一次性）

```bash
bash tools/trajdrive/build.sh     # -> ~/.ignition/gazebo/plugins/libTrajDrive.so
```

## 对象

| 名字 | 类型 | 路径 | 速度 |
|---|---|---|---|
| person_1/2/3 | actor | 货架间通道矩形环 | ~0.3 m/s |
| suv_1 | Ackermann 整车 | 南通道 y≈-6.45, x -6.5↔1.5 | 0.6 m/s |
| suv_2 | Ackermann 整车 | 南通道 y≈-6.45, x 12↔4.5 | 0.6 m/s |

车辆结构照 gz 官方 `ackermann_steering.sdf`（chassis + 4 轮 + 2 转向 link + 6 关节 + AckermannSteering）。
`TrajDrive` 每帧算一条 `Twist` 发到 `cmd_vel`；Ackermann 系统负责轮级控制 → 直行稳。

## 运行

```bash
ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf --force-version 6
# 手动发指令（调试用）：
# ign topic -t /model/suv_1/cmd_vel -m ignition.msgs.Twist -p "linear: {x: 0.5}"
```

## 要点 / 踩坑

1. **车辆最佳实践 = 带轮整车 + AckermannSteering/DiffDrive**，不是自写单刚体控制器（后者会横向漂移）。
2. 自写控制器若直接设 link 速度：`LinearVelocityCmd` 语义易漂移；改成**发布 `cmd_vel` 交给官方系统**即干净。
3. **mesh 碰撞体会让 DART 冻结车** → 车用**盒子碰撞 + 可视化**。
4. `Depot` 货架**只有视觉无碰撞** ⇒ 车对货架不真碰撞（要"完全不穿货架"需给货架补碰撞体）。
5. 插件编译：容器 cmake 4.4.4 拒收 ignition 老包（jsoncpp policy 报错）→ 用 `g++` 直编（见 `build.sh`）。
6. 插件放 `~/.ignition/gazebo/plugins/`；若环境设了 `GZ_SIM_SYSTEM_PLUGIN_PATH`，请把该目录也加进去。
