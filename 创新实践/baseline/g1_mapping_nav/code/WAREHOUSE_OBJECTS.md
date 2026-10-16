# 仓库世界加"人员 / 车辆"（动态）

在迷宫世界 `warehouse_cross.sdf` 基础上新增 `warehouse_cross_objects.sdf`：行人 + 车辆沿闭合路径**运动学驱动**
（`/world/<w>/set_pose_vector`），不依赖物理受力，所以不摔、不被卡；但也不会让路（幽灵障碍）。

## 文件

| 文件 | 说明 |
|---|---|
| `src/gazebo_sim/world/warehouse_cross_objects.sdf` | 新世界（原迷宫 + 3 行人 + 2 车） |
| `tools/objects_motion.py` | 运动驱动脚本 |

## 模型

| 名字 | 来源 | 说明 |
|---|---|---|
| `person_standing` | Fuel `OpenRobotics/Standing person` (.dae) | 行人 |
| `suv_vehicle` | Fuel `OpenRobotics/SUV` (.obj) | 车；**原尺寸 5.02×2.60×2.16m 太宽，世界内改为 inline 模型 `scale 0.04` → 3.35×1.74×1.44m** |

> ⚠️ **弃用 Forklift**：该模型视觉网格是 `.glb`，而 Fortress/gz-sim6 的 Ogre2 **不支持 .glb**
> （`Invalid mesh filename extension`），会导致场景几何加载失败。

下载/预缓存：
```bash
ign fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV' -j 4
ign fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person' -j 6
# 复制到 src/gazebo_sim/models/{suv_vehicle,person_standing}
```

## 运动对象（互相错开，均在通道内）

| 对象 | 通道 | 路径 |
|---|---|---|
| person_1 | 货架间 y=-1.15 | x 1.0 ↔ 11.0 |
| person_2 | 货架间 y=1.25 | x 1.0 ↔ 11.0 |
| person_3 | 货架间 y=3.85 | x 11.0 ↔ 1.0 |
| suv_1 | 南侧通道 y=-6.5 | x -6.5 ↔ 1.5 |
| suv_2 | 南侧通道 y=-6.5 | x 12.0 ↔ 4.5 |

## 用法

```bash
ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf --force-version 6
# 另开终端（同 IGN_PARTITION）：
python3 tools/objects_motion.py --world world_demo --rate 15
```

## 关键坑（踩过的）

1. **Depot 模型的碰撞体只是一个 100×100 `<plane>`**，货架/墙**只有视觉、没有碰撞** ⇒ 用碰撞图判断会误判；路径必须落在**真实空闲通道**（看仓库地面的黄色通道标线/俯视图）。
2. **SUV 原始 mesh 5.02×2.60m**，而货架间通道普遍 ~2m，原尺寸必然穿模 ⇒ 必须缩放（现用 0.04）。
3. **`.glb` 不被 Ogre2 支持** ⇒ 别用 Forklift。
4. `TrajectoryFollower`（力驱动）会把 80kg 行人掀翻、SUV 卡死 ⇒ 用 kinematic `set_pose`。
5. 驱动里 `set_pose_vector` 的 `--req` 文本经多层 shell 会被吃引号 ⇒ 用 base64 传内联脚本。
6. 世界与驱动必须同一 `IGN_PARTITION`；GUI 截图要抓**窗口**（root 常被屏保刷白）。
