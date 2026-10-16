# 仓库世界加"人员 / 车辆"

`warehouse_cross_objects.sdf` = 原迷宫 + **3 个行人 `<actor>`** + **2 辆车（动态刚体 + 插件驱动）**。

设计取舍：
- **行人**：`<actor>`（物理无关、sim-time 平滑插值）→ 不瞬移、不坠落。走**长走廊矩形环**。
- **车辆**：**动态刚体 + 自写 gz-sim 插件 `libTrajDrive.so`** → 是真正的物理体：**被机器人顶到会被推动**、**不会穿过有碰撞体的几何**。用纯追踪按 waypoint 循环行驶。

## 文件

| 文件 | 说明 |
|---|---|
| `src/gazebo_sim/world/warehouse_cross_objects.sdf` | 世界 |
| `tools/trajdrive/traj_drive.cc` | 车辆驱动插件源码 |
| `tools/trajdrive/CMakeLists.txt` / `build.sh` | 构建（本机 g++ 直编，无需 colcon） |
| `tools/obj2dae.py` | OBJ→COLLADA（如需把车当 actor 皮肤用；本方案车辆用 obj 视觉+盒子碰撞，不必需） |
| `SOTA_MOVING_ACTORS.md` | 方案调研/最佳实践 |

## 编译插件（一次性）

```bash
bash tools/trajdrive/build.sh
# -> 安装到 ~/.ignition/gazebo/plugins/libTrajDrive.so（gz 默认就会加载）
```

## 对象

| 名字 | 类型 | 通道/路径 | 速度 |
|---|---|---|---|
| person_1/2/3 | actor `walk.dae` | 货架间通道**矩形环** | ~0.3 m/s |
| suv_1 | 动态刚体 + TrajDrive | 南通道 y≈-6.45, x -6.5↔1.5 | 0.5 m/s |
| suv_2 | 动态刚体 + TrajDrive | 南通道 y≈-6.45, x 12↔4.5 | 0.5 m/s |

## 运行

```bash
ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf --force-version 6
```

## 已知问题 / 待调

- 车辆控制器用"body-frame 线速度 + 偏航"驱动，**存在缓慢横向漂移**：实测车会从 y≈-6.45 漂到 y≈-4.9（货架边缘）并卡住。
  下一步可：改用**纯转向小车的横向误差项**、或降低速度/提高 `kp`、或给这段通道加中心线约束。
- 车用**盒子碰撞 + mesh 视觉**（mesh 碰撞体会让 DART 冻结不动，实测）。
- `Depot` 货架**只有视觉无碰撞** ⇒ 车对货架不真碰撞；要"完全不穿货架"需给货架补碰撞体。

## 附：agent/actor 版车（无物理）

若只要"平滑不穿模"、不需要被推动，把车改回 `<actor>`（`suv.dae` 皮肤）即可（历史版本），
见 `SOTA_MOVING_ACTORS.md`。
