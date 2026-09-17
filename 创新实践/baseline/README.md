# baseline/ — 三组代码

每组的代码放在自己的目录下，**不跨目录改别人的文件**。跨组的改动走接口，不走代码。

```
baseline/
├── g1_mapping_nav/        G1 建图导航：仿真 · SLAM · 定位 · Nav2 · 避障
├── g2_vision_explore/     G2 视觉探索：检测 · 三维定位 · 探索决策 · 异常识别
└── g3_integration_eval/   G3 集成评测：真机通信 · 系统集成 · 指标评测
```

---

## 接口契约

写进代码之前先看这张表。**第一周冻结，之后只加字段，不改语义。**

| # | 方向 | 名称 | 内容 |
|---|---|---|---|
| 01 | G1 → G2 / G3 | 地图与定位 | `nav_msgs/OccupancyGrid` + `sensor_msgs/PointCloud2` + TF(`map → base_link`) |
| 02 | G2 → G1 | 探索目标点 | `NavigateToPose` action |
| 03 | G2 → G3 | 目标消息 | 自定义 `Detection3D`：类别 / 置信度 / 三维坐标 / 时间戳 / 抓拍图 |
| 04 | 全队 | 评测数据 | 统一 rosbag 话题清单 + 指标计算脚本 |

### 为什么接口 02 长这样

「下一个去哪」和「怎么走过去」是两件事，分属两组：

```
G2 视觉探索  ──发目标点──▶  G1 建图导航
  frontier /                Nav2 路径规划
  信息增益                  代价地图 / 避障
  （决策层）                 （执行层）
```

G2 只发 `NavigateToPose`，不碰路径、不碰代价地图、不碰 TF。G1 只管把点走到，不关心这个点是怎么选出来的。**两边可以各自独立测试**——G2 的探索节点对着一个假的目标接收器就能调，G1 的导航栈被一个手点目标就能验。

低电量返航走同一个入口：G3 判断电量，发一个回基站的目标点，复用这条通路，不另开接口。

---

## 三条硬规矩

1. **别动别人的目录。** 需要对方改东西，提 issue 或当面说，不要直接改。三个组并行开工，改别人的文件必然冲突。
2. **接口变更先说。** `Detection3D` 加字段可以，改语义（比如把 `pose` 从相机系改成机体系）必须全队通知，否则三组一起挂。
3. **仿真里验过再上真机。** 真机机时紧张，狗摔一次维修代价高。占着狗调代码是最亏的用法。

---

## 各组技术栈

| 组 | 主要依赖 |
|---|---|
| G1 | gz-sim · RTAB-Map · robot_localization · Nav2 |
| G2 | ultralytics · OpenCV · frontier 探索（explore_lite 或自写） |
| G3 | unitree_ros2 · unitree_sdk2 · rosbag2 · rviz2 |

各组的具体任务、交付物与本周要干的事，见各自目录下的 `README.md`。

---

## 分支约定

```
develop                          ← 集成主体：日常代码一律先合到这里
 ├── g1/slam-tuning              ← 各组从 develop 切出，用自己的前缀
 ├── g2/frontier-gain
 └── g3/dds-bringup
main                             ← 只放能跑通、能演示的东西
```

```bash
git checkout develop
git checkout -b g1/slam-tuning   # 各组用自己的前缀
```

- **各组分支从 `develop` 切出，做完合回 `develop`。**
- **`develop` 是集成主体**，每天的代码都往这里合，允许它暂时跑不通。
- **`main` 只放能跑通、能演示的东西**，由 `develop` 定期合并过去，不直接往 `main` 推。
- 接口相关的改动合进 `develop` 前先在群里说一声。
