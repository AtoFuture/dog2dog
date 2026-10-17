# 仓库世界加"人员 / 车辆"

`warehouse_cross_objects.sdf` = 原迷宫 + **3 个行人 `<actor>`** + **2 辆车（带轮整车 + AckermannSteering）**。

| 类别 | 方案 | 特性 |
|---|---|---|
| 行人 | `<actor>` + `walk.dae` + 轨迹 | 物理无关、sim-time 平滑插值 → 不瞬移、不坠落 |
| 车辆 | 带轮整车 + 官方 `AckermannSteering` + `TrajDrive`(路径→`cmd_vel`) | 真物理体（可推、会碰撞）、干净直行 |

## ⚠️ 穿模的根因（关键）

`Depot` / `depot_collision` 的**模型原点在 (7.19, 1.10)**。之前所有路径坐标都**漏加了这个偏移**，
导致"以为在通道、其实压在货架上"。另外：
- `Depot` 的**货架只有视觉网格**（`Crates.dae` + Depot.dae 的 BOXSET/FANS/PILLERS 等），**没有碰撞体**；
- 判定通道**不能看碰撞体**，必须看**视觉**。

**做法**：把 `Depot.dae` + `Crates.dae` 按**场景图变换**解析、取**可通行高度带(z 0.15~2.5m)**的三角面
栅格化 → 得到真实占用图，再**按 0.7m 膨胀取最长空闲直道**选线。生成的样子见 `tools/README`（脚本在 `tools/`）。

## 路径（已避开货架与迷宫）

| 名字 | 类型 | 车道 | 范围 |
|---|---|---|---|
| person_1 | actor | 东侧货架间 x=10.8 | y 0.4 ↔ 7.4 |
| person_2 | actor | 东侧货架间 x=18.75 | y 0.4 ↔ 7.4 |
| person_3 | actor | 横通道 y=0.25 | x 8.0 ↔ 20.5 |
| suv_1 | Ackermann | 西侧空区 x=3.6 | y -5.4 ↔ 7.0 |
| suv_2 | Ackermann | 西侧空区 x=5.4 | y -5.4 ↔ 4.0 |

（迷宫在 x≤2、货架在 x≥7.4，故车放西侧空区、人放货架间通道。）

## 文件

| 文件 | 说明 |
|---|---|
| `src/gazebo_sim/world/warehouse_cross_objects.sdf` | 世界 |
| `tools/trajdrive/traj_drive.cc` | 车辆驱动插件（发布 `/model/<n>/cmd_vel`） |
| `tools/trajdrive/build.sh` | 编译并安装到 `~/.ignition/gazebo/plugins/` |
| `tools/gen_objects_world.py` | 生成世界（改路径/速度就改这里） |
| `tools/obj2dae.py` | OBJ→COLLADA（备用） |
| `SOTA_MOVING_ACTORS.md` | 方案调研/最佳实践 |

## 运行

```bash
bash tools/trajdrive/build.sh            # 一次性
ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf --force-version 6
```

## 其它踩坑

- 车用**盒子碰撞 + 可视化**（mesh 碰撞体会让 DART 冻结）。
- 插件默认放 `~/.ignition/gazebo/plugins/` 即被加载；若设了 `GZ_SIM_SYSTEM_PLUGIN_PATH`，需把该目录也加进去。
- 容器 cmake 4.4.4 拒收 ignition 老包 → 用 `g++` 直编（`build.sh`）。
