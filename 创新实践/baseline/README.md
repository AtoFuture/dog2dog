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
| 03 | G2 → G3 | 目标消息 | 自定义 `Detection3D`（**不含图**，字段见下） |
| 03b | G2 → G3 | 抓拍图 | 自定义 `DetectionSnapshot`（**独立话题**，见下） |
| 04 | 全队 | 评测数据 | 统一 rosbag 话题清单 + 指标计算脚本 |

消息定义在 `baseline/interfaces/vision_interfaces/msg/`，全队共管。

### 接口 03 / 03b 字段定义（冻结）

```msg
# Detection3D —— 轻量消息，永远发布
string                            class_name   # 类别名
float32                           confidence   # 置信度 0~1
geometry_msgs/PoseWithCovariance  pose         # 三维坐标，map 系
builtin_interfaces/Time           stamp        # 图像采集时刻
uint32                            id           # 跟踪 ID，去重用
string                            source       # coco | pose | heuristic
bool                              has_snapshot # 是否有对应抓拍图
```

```msg
# DetectionSnapshot —— 独立话题，按需订阅
uint32                       id      # 对应 Detection3D.id
builtin_interfaces/Time      stamp   # 与对应 Detection3D.stamp 一致
sensor_msgs/CompressedImage  image   # ROI 裁剪 + JPEG
```

**几条必须说清的语义**（这些比坐标系更容易被悄悄改）：

| 项 | 规定 |
|---|---|
| `pose` 坐标系 | **map 系**。改成相机系/机体系必须全队通知 —— 这是点名的红线 |
| `pose.orientation` | **不使用**，恒为单位四元数。检测结果是一个点，没有朝向 |
| `pose.covariance` | 只填左上 3×3 位置方差，其余置零。数值是三维解算的像素离散度换算而来，是**下界**（不含外参/TF/同步误差）。用途是让 G3 筛掉不可信的坐标 |
| `stamp` | **图像采集时刻**，不是处理完成时刻。用处理时刻会让坐标与当时的 TF 对不上 |
| 代表点语义 | `bbox 中心` / `底部中点` 由 G2 参数控制，**变更必须通知 G3** —— 它比坐标系更容易被悄悄改，且同样影响评测口径 |
| `id` | 同一目标连续检出时不变。**检出率/误检率的统计基于去重后的消息**。<br>**`0` 是保留值 = 本次检测没有跟踪 ID**（跟踪器尚未确认轨迹）。G3 **不要**把多条 `id=0` 当成同一目标合并 —— 那会把画面里所有未确认目标并成一个。对 `id=0` 各自独立计数、不去重 |
| `source` | `coco` = 预训练检测器；`pose` = 姿态判据；`heuristic` = 启发式。G3 可据此确定性过滤（例如「异常类不计入误检率」直接判 `source != "coco"`） |

### 为什么抓拍图要独立话题（03b）

内嵌一张 `sensor_msgs/Image` 会让每条消息约 **0.92 MB**（640×480 rgb8）。按 5 Hz、每帧 3 个目标算就是 **13.8 MB/s**，一次 10 分钟实验落 rosbag 约 **8.3 GB** —— 而接口 04 要求每次实验都落盘。

更要紧的是传输层：RTPS 默认走 UDP，大消息要分片，**丢一片就得重传整个 sample**。ROS2 社区公认 ≥1 MB 的消息开始出问题，我们这条正压在门槛上。**丢包的后果不是「图没了」，是整条 `Detection3D` 丢了** —— 而它是检出率指标的唯一数据来源。

拆开后：轻量消息永远可靠，图像按需订阅，数据量降到约 **100 MB/次**。

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

> ⚠️ **这条通路有个待补的缺口**：G2 的探索目标与 G3 的返航点打的是**同一个 action server**，Nav2 一次只接受一个 goal。G3 发返航点时会**抢占** G2 在飞的目标，而契约里**没有 G3 → G2 的停手信号**。G2 侧需要一个 `PASSIVE` 状态来响应抢占，否则会继续选下一个 frontier 再抢占回来，两边打架 —— 直接威胁「返航成功率 100%」这个指标。需在第一次接口评审时补一条接口。

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
