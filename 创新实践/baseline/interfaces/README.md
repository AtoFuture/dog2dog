# baseline/interfaces/ — 跨组共用接口包

放**多个组都要依赖**的消息/服务/动作定义。与 `g1_`/`g2_`/`g3_` 各组的私有代码分开。

**为什么单独提出来**：`baseline/README.md` 的硬规矩是「别动别人的目录」。
接口包如果放在 G2 目录里，G3 要么得构建别人目录里的包，要么每次接口变更都得找 G2。
放在这里，三组平等共管。

---

## vision_interfaces

当前只含**接口 03：`Detection3D`**（G2 → G3）。

```
vision_interfaces/
├── package.xml
├── CMakeLists.txt
└── msg/
    └── Detection3D.msg
```

### 字段

按 `baseline/README.md` 的接口契约实现：

| 字段 | 类型 | 说明 |
|---|---|---|
| `class_name` | `string` | 类别名 |
| `confidence` | `float32` | 置信度 0~1 |
| `pose` | `geometry_msgs/Pose` | 三维坐标，**map 系** |
| `stamp` | `builtin_interfaces/Time` | **图像采集时刻** |
| `snapshot` | `sensor_msgs/Image` | 抓拍图 |
| `id` | `uint32` | **G2 新增** —— 跟踪 ID，用于去重 |
| `source` | `string` | **G2 新增** —— `coco` / `pose` / `heuristic` |

契约规定「第一周冻结，之后只加字段，不改语义」。新增的两个字段符合这一条；
`pose` 是 map 系、`stamp` 是采集时刻，这两条语义红线**没有动**。

### ⚠️ 两个建议需要全队讨论（G2 提出，尚未决定）

**① `snapshot` 用整张 `sensor_msgs/Image` 会让消息很重**

640×480 rgb8 ≈ **0.92 MB/条**。RTPS 默认报文约 64 KB，大消息要分片，
**丢一片就得重传整个 sample**；ROS2 社区公认 ≥1 MB 的消息开始出问题，
Fast DDS 在 shared memory + BEST_EFFORT 下 >0.5 MB 就有偶发丢包报告。

按 5 Hz × N 个目标、每条带一张整图估算，`/detections_3d` 会成为链路上最重的话题，
跨机回传时更糟。

可选改法（**都属于改类型，需要全队同意**）：
- 只发**裁剪后的目标 ROI** 并 JPEG 编码，典型 20–50 KB/条，两个数量级的差别
- 图放**独立话题**，用 `id` + `stamp` 做外键，G3 按需订阅

**② `pose` 用 `Pose` 但 `orientation` 语义未定义**

检测结果只有一个点。契约里没写 `orientation` 填什么，下游会各自解释
（可能有人当目标朝向用）。可考虑改 `geometry_msgs/Point`，
或用 `PoseWithCovariance` 表达不确定度（三维解算本来就能算出离散度）。
**改类型同样需要全队同意。**

> 在这两条定下来之前，按契约原样使用 `Pose` + `Image`，不要单方面改。

### 编译

⚠️ **不要在中文路径下编译**——colcon/CMake 处理不了非 ASCII 路径，
报错会指向一个不存在的路径，极具误导性。详见 [`docs/环境搭建.md`](../../docs/环境搭建.md)。

容器内（仓库被 bind 挂到 ASCII 路径）：

```bash
cd /home/wy/project/baseline/interfaces
colcon build --base-paths .
source install/setup.bash
ros2 interface show vision_interfaces/msg/Detection3D
```
