# G2 · 视觉探索

> 目标检测 · 三维定位 · 探索决策 · 异常识别
>
> 本页按 2026-09-27 的源码、测试和离线运行产物同步；总体设计与历史审计记录见 [`docs/框架规划.md`](docs/框架规划.md)。

## 当前进度

### 【已实现】

- Frontier 的 `Goal.yaw` 已转换为绕 Z 轴的单位四元数，并发送到 `NavigateToPose.orientation`；目标朝向表示抵达 frontier 后面向附近未知区域的方向。
- `Detection3D.has_snapshot` 已按 Snapshot 实际构造并进入发布路径的结果设置；普通检测和倒地事件共用这一语义。
- 独立 `go2_video_bridge` ROS2 package：`Go2FrontVideoData.video720p` 输入、持久化 H.264 streaming parser、libavcodec 解码、libswscale 转 BGR，以及 `sensor_msgs/Image` 输出。
- Bridge 的 `input_topic`、`output_topic`、`video_stream`、`frame_id` 均参数化，并带有输入消息数、payload 字节数、parser packet 数、解码/发布帧数、错误数、分辨率、频率和原始 `time_frame` 统计。

### 【离线验证通过】

- 针对 Frontier 朝向和 Snapshot 语义的测试：**29 passed，0 failed，0 skipped**。
- 真实 Ultralytics 离线运行：`yolo26n.pt` + `fall-01-cam0.mp4` + 真实 `Detector.track()`；检出 `person`，confidence 约 `0.79–0.86`，`track id=1`。这是真实素材上的离线结果，不是真机结果。
- Go2 Bridge：用现有 `fall-01-cam0.mp4` 生成 Annex-B H.264；chunk 大小 `1 / 7 / 188 / 1024 / 4096 / fixed random(seed=20260927)` 每种均解出 **160 frames**，`packets=160`、`errors=0`、分辨率 `640x240`，每帧缓冲区尺寸正确。
- Bridge 的 C++ build、CTest/colcon test 和节点 launch smoke 均通过。launch smoke 只验证节点可启动，不等同于真机 ROS 图验证。

### 【真机未验证】

- Go2 真机 H.264 解码和 `/frontvideostream` 实际 QoS、`video720p` payload 边界、真机分辨率。
- `time_frame` 单位/时钟源、真实 acquisition timestamp、camera frame 和 camera 外参。
- G2 可用的真实 PointCloud2 输入；RGB + L1 三维目标定位。
- 视觉目标的 map 定位、边建图边搜索完整闭环、真机 Nav2/探索闭环，以及 Foxy ↔ Humble 跨版本链路。

离线 H.264 流验证**不表示 Go2 真机视频已跑通**。

## 测试

在本目录执行：

```bash
bash tools/run_tests.sh
```

入口先加载 ROS Humble 和 `baseline/install` 中已构建的接口；宿主依赖不足时，
自动使用已有的 `~/.enroot/ros` 容器及其系统 Python。可通过 `G2_ROS_WRAPPER`
指定其他兼容的容器入口，或加 `--local` 仅使用当前环境；其余参数传给 pytest。
需要先构建 `baseline/interfaces`，不在测试入口中安装依赖或启动仿真。

2026-10-15 复核：容器系统 Python 下 **247 passed**，包含探索节点和检测消息测试。
本地 `.venv` 缺 PyYAML、Ultralytics，宿主 ROS 缺 Nav2 消息和 cv_bridge，
因此直接使用这些环境会在收集或依赖检查阶段失败。部分旧版 pytest 会提示
不认识 `pythonpath` 配置项，入口已通过 `PYTHONPATH` 显式提供源码路径。

## 当前接口

### Go2 前视视频 Bridge

```text
/frontvideostream (unitree_go/msg/Go2FrontVideoData)
  -> video720p 连续 H.264 字节流
  -> go2_video_bridge_node
  -> /camera/go2/image_raw (sensor_msgs/Image, bgr8)
```

默认 topic：

- 输入：`/frontvideostream`
- 输出：`/camera/go2/image_raw`
- QoS：输入/输出 `SensorDataQoS`

Bridge 的具体边界、构建命令和离线测试见 [`go2_video_bridge/README.md`](go2_video_bridge/README.md)。

`Go2FrontVideoData` 没有标准 ROS Header。Bridge 使用节点时钟在解码/发布时取得的时间作为输出 `header.stamp`；只记录原始 `time_frame`，不猜测其单位或时钟语义。`frame_id` 默认为空，启动时会 warning；在三维定位前必须配置真实 camera frame 和外参。Bridge 不发布 `CameraInfo`，不伪造深度。

正式检测接口以 [`baseline/interfaces/vision_interfaces/msg/Detection3D.msg`](../interfaces/vision_interfaces/msg/Detection3D.msg) 和 [`DetectionSnapshot.msg`](../interfaces/vision_interfaces/msg/DetectionSnapshot.msg) 为准，不在本包内修改消息定义。

## 尚未闭环的任务

1. 上 Go2 真机验证视频 QoS、payload 边界、解码、时间和 frame 语义。
2. 确认/接入 G2 可用的 L1 `PointCloud2`，完成 RGB + L1 三维定位。
3. 完成视觉目标 map 定位、边建图边搜索和真机 Nav2 探索闭环验证。
4. 评估 Foxy 真机运行与 Humble 开发环境之间的跨版本链路。

G2 不修改 G1/G3、`vision_interfaces/*.msg` 或冻结的 topic、消息字段语义、QoS、TF 接口；本轮也不实现 LiDAR 代码。
