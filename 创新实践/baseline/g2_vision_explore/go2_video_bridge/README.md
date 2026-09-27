# go2_video_bridge

独立的 Go2 前视视频输入适配层：

```text
/frontvideostream (unitree_go/msg/Go2FrontVideoData)
  -> video720p 连续 H.264 字节流
  -> libavcodec H.264 parser/decoder
  -> bgr8 sensor_msgs/Image
  -> /camera/go2/image_raw
```

## 接口

节点：`go2_video_bridge_node`

参数：

- `input_topic`，默认 `/frontvideostream`
- `output_topic`，默认 `/camera/go2/image_raw`
- `video_stream`，默认 `video720p`，也支持 `video360p` / `video180p`
- `frame_id`，默认空字符串

输入和输出均使用 `SensorDataQoS`。输入 QoS 是 BEST_EFFORT 兼容的传感器 QoS，真机连接后仍需按实际 publisher QoS 复核。输出消息编码固定为 `bgr8`。

`Go2FrontVideoData` 没有标准 ROS Header，因此输出图像的 `header.stamp` 是节点时钟在解码/发布时取得的时间。`time_frame` 只作为原始诊断值记录，不被猜测为纳秒或毫秒。当前没有正式冻结的 camera frame；`frame_id` 为空时节点启动会 warning，三维定位前必须配置真实 camera frame 和外参。

## 流式边界处理

每次 ROS 回调只调用一次 `H264StreamDecoder::push()`。`AVCodecParserContext` 在调用之间保留不完整的 NAL/access unit，所以：

- 一条 ROS 消息可以产生 0、1 或多帧；
- 一帧可以跨越多个 ROS 消息；
- 不会把一条 ROS 消息当作一帧，也不会在消息边界 flush decoder。

节点只在第一次收到 payload、第一次成功解码，以及每 5 秒统计时打印日志，不逐帧刷屏。统计包含消息数、payload 字节数、parser packet 数、解码/发布帧数、decode errors、图像转换 errors、分辨率、输入消息频率、输出帧率和最近的原始 `time_frame`。

## 状态分级（2026-09-27）

### 【已实现】

- 独立 `go2_video_bridge` ROS2 package 和 `go2_video_bridge_node`。
- `Go2FrontVideoData.video720p` 输入，持久化 `AVCodecParserContext` 流式解析，libavcodec H.264 解码，libswscale 转 BGR。
- 参数化 `input_topic`、`output_topic`、`video_stream`、`frame_id`；输出 `sensor_msgs/Image`。
- `frame_id` 默认为空，启动时 warning；输出时间戳使用节点时钟的解码/发布时刻，`time_frame` 只作为原始诊断值，不做单位换算。
- 运行统计：输入消息、payload bytes、parser packets、decoded/published frames、decode/image errors、分辨率、输入/输出频率、最近原始 `time_frame`。

### 【离线验证通过】

- `fall-01-cam0.mp4` 已用系统 `ffmpeg` 转成临时 Annex-B H.264 `/tmp/go2_bridge_test.h264`。
- chunk 大小 `1 / 7 / 188 / 1024 / 4096 / fixed random(seed=20260927)` 每种均为 `160 frames`、`packets=160`、`errors=0`、`resolution=640x240`，每帧数据尺寸正确。
- C++ build、CTest/colcon test 和 launch smoke 均通过；launch smoke 只证明节点可启动，不证明真机数据链路已连通。

### 【真机未验证】

- `/frontvideostream` 实际 QoS、`video720p` 真机 payload 边界/封装、真实解码和真机分辨率。
- `time_frame` 单位/时钟源、真实 acquisition timestamp、camera frame、相机外参。
- Foxy 真机运行兼容性。

离线 H.264 流验证**不表示 Go2 真机视频已跑通**。

## 构建

在已有 ROS Humble 容器中先 source ROS 和已经构建的 `unitree_go` 工作区：

```bash
source /opt/ros/humble/setup.bash
source /home/wy/project/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build \
  --base-paths /home/wy/project/baseline/g2_vision_explore \
  --packages-select go2_video_bridge \
  --symlink-install
```

该包要求现有容器提供 `pkg-config`、`libavcodec`、`libavutil`、`libswscale` 的开发头文件和库；不会安装依赖。

运行：

```bash
ros2 run go2_video_bridge go2_video_bridge_node \
  --ros-args -p frame_id:=<real_go2_camera_frame>
```

或：

```bash
ros2 launch go2_video_bridge go2_video_bridge.launch.py
```

## 离线流式 chunk 测试

测试使用现有 `fall-01-cam0.mp4`，先在 `/tmp` 提取 Annex-B H.264，不提交生成文件：

```bash
ffmpeg -y -i /home/wy/project/baseline/g2_vision_explore/fall-01-cam0.mp4 \
  -map 0:v:0 -an -c:v copy -bsf:v h264_mp4toannexb /tmp/go2_bridge_test.h264
```

构建时传入临时测试文件：

```bash
colcon build \
  --base-paths /home/wy/project/baseline/g2_vision_explore \
  --packages-select go2_video_bridge \
  --cmake-args -DGO2_BRIDGE_TEST_FILE=/tmp/go2_bridge_test.h264
ctest --test-dir build/go2_video_bridge --output-on-failure
```

测试会用固定 chunk 大小 `1/7/188/1024/4096` 和固定 seed `20260927` 的随机 chunk 重复喂同一个 Annex-B 字节流，检查帧数、分辨率、每帧图像缓冲区和错误计数。当前离线素材解出的分辨率是 `640x240`；这只是测试素材分辨率，不是 Go2 真机分辨率承诺。

本包不发布 `CameraInfo`，不伪造深度，不修改现有 detector 或 `vision_interfaces`。

## 连接 G2 RGB-only detector

Bridge 的输出可以直接作为 `vision_detector` 的 RGB-only 输入：

```text
/camera/go2/image_raw (sensor_msgs/Image, bgr8)
    -> vision_detector(input_mode=rgb)
```

静态配置/启动方式为：

```bash
ros2 launch vision_detector detector.launch.py \
  input_mode:=rgb rgb_topic:=/camera/go2/image_raw
```

该连接已通过代码、参数和离线二维处理路径验证；当前没有 Go2 真机 `/frontvideostream` 数据，因此不代表真机 Bridge → detector ROS 图已经跑通。RGB-only detector 只保留内部二维检测/跟踪结果，不伪造 `Detection3D`、`CameraInfo` 或深度。
