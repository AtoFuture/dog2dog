# baseline/interfaces/ — 跨组共用接口包

放**多个组都要依赖**的消息/服务/动作定义，与 `g1_`/`g2_`/`g3_` 各组的私有代码分开。

**为什么单独提出来**：`baseline/README.md` 的硬规矩是「别动别人的目录」。
接口包如果放在 G2 目录里，G3 要么得构建别人目录里的包，要么每次接口变更都得找 G2。
放在这里，三组平等共管。

---

## vision_interfaces

含**接口 03 与 03b**（G2 → G3）：

```
vision_interfaces/
├── package.xml
├── CMakeLists.txt
└── msg/
    ├── Detection3D.msg        # 接口 03：目标消息（轻量，不含图）
    └── DetectionSnapshot.msg  # 接口 03b：抓拍图（独立话题）
```

### 字段契约

完整契约写在 [`../README.md`](../README.md) 的「接口契约」一节 —— **以那里为准**。
这里是摘要：

| 消息 | 字段 | 语义 |
|---|---|---|
| `Detection3D` | `class_name` | 类别名 |
| | `confidence` | 置信度 0~1 |
| | `pose` | `PoseWithCovariance`，**map 系**；`orientation` 不使用 |
| | `stamp` | **图像采集时刻** |
| | `id` | 跟踪 ID，去重用 |
| | `source` | `coco` / `pose` / `heuristic` |
| | `has_snapshot` | 是否在抓拍图话题上有对应图 |
| `DetectionSnapshot` | `id` + `stamp` | 与 `Detection3D` 的外键 |
| | `image` | ROI 裁剪的 JPEG（`CompressedImage`） |

### 两条设计决定及其理由

**① 抓拍图不内嵌，走独立话题**

内嵌一张 `sensor_msgs/Image` 是 0.92 MB/条（640×480 rgb8）。按 5 Hz × 3 目标算，
带宽 13.8 MB/s，一次 10 分钟实验落 rosbag 约 **8.3 GB**（而接口 04 要求次次落盘）。

更要紧的是传输层：RTPS 走 UDP，大消息分片，**丢一片重传整个 sample**。
ROS2 社区公认 ≥1 MB 开始出问题，我们这条正压在门槛上。丢包的后果是
**整条 `Detection3D` 丢了**，而它是检出率指标的唯一数据来源 —— 指标会莫名偏低且难查。

拆开后轻量消息永远可靠，图像按需订阅，数据量降到约 **100 MB/次**。

**② `pose` 用 `PoseWithCovariance` 而不是 `Pose`**

原契约写的是「三维坐标」，但 `geometry_msgs/Pose` 还带一个 `orientation`，
而契约里完全没规定它填什么 —— 于是不同的人会填不同的东西（留默认 `(0,0,0,0)`
是个不合法的四元数，用它做旋转变换会得到 NaN）。

改用 `PoseWithCovariance` 之后：

- `orientation` 明确规定**不使用**，恒为单位四元数
- `covariance` 只填左上 3×3 位置方差 —— 数值来自三维解算时像素的离散程度，
  让 G3 能**筛掉坐标不可信的检测**，而不必全盘接受或全盘丢弃

⚠️ 协方差是**下界**：只反映深度像素自身的离散，不含外参、TF、时间同步误差。

### 编译

⚠️ **不要在中文路径下编译** —— colcon/CMake 处理不了非 ASCII 路径，
报错会指向一个不存在的路径，极具误导性。详见 [`docs/环境搭建.md`](../../docs/环境搭建.md)。

容器内（仓库被 bind 挂到 ASCII 路径）：

```bash
cd /home/wy/project/baseline/interfaces
colcon build --base-paths .
source install/setup.bash
ros2 interface show vision_interfaces/msg/Detection3D
```

### 变更记录

| 日期 | 变更 | 性质 |
|---|---|---|
| 2026-09-17 | 初版：按契约实现 `Detection3D`，另加 `id` / `source` | 加字段 |
| 2026-09-17 | 抓拍图拆为独立消息 `DetectionSnapshot`（03b）；`pose` 由 `Pose` 改为 `PoseWithCovariance` | **改类型 —— 已全队确认** |

> 按契约硬规矩 2，「改语义必须全队通知」。第二次变更属改类型，
> 经确认后实施；后续任何类型/语义变更同样要先通知再改。
