# 仓库世界加"人员 / 车辆"（actor 版）

`warehouse_cross_objects.sdf` = 原迷宫 + **5 个 `<actor>`**（3 行人 + 2 车）。
actor 不受物理影响、由仿真按 **sim-time 平滑插值**沿轨迹运动 → **不瞬移、不坠落、不穿模**（轨迹在通道内）。
**无需任何外部驱动进程。**

## 文件

| 文件 | 说明 |
|---|---|
| `src/gazebo_sim/world/warehouse_cross_objects.sdf` | 世界（含 actor） |
| `tools/obj2dae.py` | OBJ→COLLADA 转换器（把车 mesh 变成 actor 皮肤） |
| `SOTA_MOVING_ACTORS.md` | **方案调研与最佳实践**（推荐先读） |

## actor

| 名字 | 皮肤 | 通道 | 速度 |
|---|---|---|---|
| person_1/2/3 | `walking_actor/meshes/walk.dae`（Fuel `Mingfei/actor`） | 货架间 y=-1.15 / 1.25 / 3.85 | ~0.3 m/s |
| suv_1/2 | `suv_vehicle/meshes/suv.dae`（`SUV.obj` 转出） | 南侧通道 y=-6.5 | ~0.8 m/s |

轨迹写法（相对初始位姿的 waypoint + 时间）：

```xml
<actor name="person_1">
  <pose>1.0 -1.15 0 0 0 0</pose>
  <skin><filename>model://walking_actor/meshes/walk.dae</filename><scale>1.0</scale></skin>
  <animation name="walk"><filename>model://walking_actor/meshes/walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
  <script>
    <loop>true</loop><auto_start>true</auto_start>
    <trajectory id="0" type="walk" tension="1.0">
      <waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>
      <waypoint><time>33.3</time><pose>10 0 0 0 0 0</pose></waypoint>
      <waypoint><time>66.7</time><pose>0 0 0 0 0 0</pose></waypoint>
    </trajectory>
  </script>
</actor>
```

改速度 = 改 waypoint 的 `<time>`；改路线 = 改 `<pose>`（相对初始位姿）。

## 生成车辆 actor 皮肤

```bash
python3 tools/obj2dae.py models/suv_vehicle/meshes/suv.obj \
        models/suv_vehicle/meshes/suv.dae -90 0.04   # 旋转-90° + 缩放0.04 全部烘焙进 dae
```

## 运行

```bash
ign gazebo -s -r -v4 $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf --force-version 6
```

## 注意

- actor **不与机器人发生碰撞**（幽灵式）；需要真碰撞请改用物理刚体方案（见 `SOTA_MOVING_ACTORS.md`）。
- 轨迹必须在**真实空闲通道**：本世界 Depot 货架**只有视觉、无碰撞体**，不能用碰撞图判断。
- `tools/objects_motion.py`（外部 set_pose 驱动）**已废弃**：进程+服务延迟会瞬移。
