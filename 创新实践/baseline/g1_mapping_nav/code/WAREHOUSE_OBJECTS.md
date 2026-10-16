# 仓库世界加"人员 / 车辆 / 仓储件"（动态）

在迷宫世界 `warehouse_cross.sdf` 基础上，新增一个含 **行人、车辆（动态路径）** 与 **静态仓储件** 的世界
`warehouse_cross_objects.sdf`。行人/车辆由脚本按闭合折线路径**运动学驱动**（`set_pose_vector`，不依赖物理受力），
所以既不会摔倒、也不会被墙卡住。

## 文件

| 文件 | 说明 |
|---|---|
| `src/gazebo_sim/world/warehouse_cross_objects.sdf` | 新世界（= 原迷宫 + 5 个运动体 + 5 个静态件） |
| `tools/objects_motion.py` | 运动驱动脚本（沿路径循环 `set_pose_vector`） |

## 用到的模型（Gazebo Fuel，CC0 / 官方）

| 名字 | Fuel 来源 | 用途 |
|---|---|---|
| `person_standing` | `OpenRobotics/Standing person` | 行人（3 个，动态） |
| `suv_vehicle` | `OpenRobotics/SUV` | SUV（1 个，动态） |
| `forklift_vehicle` | `OpenRobotics/Forklift` | 叉车（1 个，动态） |
| `aws_robomaker_warehouse_PalletJackB_01` / `cardboard_box` / `aws_robomaker_warehouse_Bucket_01` | 已在 `src/gazebo_sim/models` | 静态仓储件 |

离线预缓存 / 重新下载（`GZ_SIM_RESOURCE_PATH` 指向 `src/gazebo_sim/models`）：

```bash
ign fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV' -j 4
ign fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Forklift' -j 4
ign fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person' -j 6
# 然后把 Fuel 缓存里的模型目录复制到 src/gazebo_sim/models/{person_standing,suv_vehicle,forklift_vehicle}
```

## 运动对象与路径

| 对象 | 初始位姿 | 路径（闭合折线） | 速度 |
|---|---|---|---|
| person_1 | (-2, -6.4) | (-2,-6.4) ↔ (11.5,-6.4) | 0.6 m/s |
| person_2 | (1.2, 1.27) | (1.2,1.27) ↔ (11.5,1.27) | 0.6 m/s |
| person_3 | (11.5, 3.86) | (11.5,3.86) ↔ (1.2,3.86) | 0.7 m/s |
| suv_1 | (-13.5, -5.0) | 矩形 (-13.5..-8.5, -5..-1) | 1.5 m/s |
| forklift_1 | (-13.5, 2.0) | 矩形 (-13.5..-8.5, 2..5.5) | 0.9 m/s |

路径都选在**开阔通道**：行人走货架之间的纵向通道/南侧通道，车辆走西侧开阔区（x∈[-14,-8]）。

## 用法

```bash
# 1) 起世界（用 install 路径，和 session.sh 一致）
ign gazebo -s -r -v4 \
  $WS/install/gazebo_sim/share/gazebo_sim/world/warehouse_cross_objects.sdf --force-version 6

# 2) 另开一个终端，起运动驱动（要和仿真同 domain / partition）
python3 tools/objects_motion.py --world world_demo --rate 15 --duration 0
```

改路径 / 速度：编辑 `objects_motion.py` 顶部的 `MOVERS`（并同步改世界文件里对应的初始 `<pose>`，避免起步跳变）。

暂停单个对象（gz 自带话题）：`ign topic -t /model/person_1/... ` 不适用于本方案；
本方案由脚本统一驱动，停止脚本即全部停止。

## 为什么不用 `TrajectoryFollower`

实测 Fortress 6.18 的 `ignition-gazebo-trajectory-follower-system` 是**力驱动**：80kg 行人会被掀翻（四元数翻转），
SUV 走一段后卡死。改用 kinematic `set_pose_vector` 后：行人/车稳定直立、沿路径平滑移动（已无头验证：位姿随时间推进、
z 高度稳定、姿态 w≈1）。

## 注意

- 运动体是**运动学**的、不响应碰撞：被机器人顶到也不会让路（适合做"幽灵障碍/动态目标"）。
  若需真实物理交互，勿用本方案。
- 世界与驱动都要在**同一 `IGN_PARTITION`/`GZ_PARTITION`** 下，否则 `set_pose_vector` 服务连不上。
- `warehouse_cross_objects.sdf` 是**新增文件**，原 `warehouse_cross.sdf` 未改。
