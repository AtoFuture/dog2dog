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

## 验证边界

当前已验证：

- ROS 容器中的 FFmpeg 开发头文件、parser API 和链接库；
- Annex-B H.264 流式 chunk 解码代码；
- `sensor_msgs/Image` 的 `bgr8` 输出代码；
- 现有离线视频的不同 chunk 边界测试：`fall-01-cam0.mp4` 转 Annex-B 后，
  `1 / 7 / 188 / 1024 / 4096 / fixed random(seed=20260927)` 每种 chunk 均为
  **160 frames、packets=160、errors=0、resolution=640x240**；
- C++ build、CTest/colcon test 和 launch smoke 通过；launch smoke 只证明节点可启动，
  不证明真机数据链路已连通。

当前未验证：

- 真机 `/frontvideostream` 的实际 QoS；
- `video720p` 每条消息的 payload 边界和真实封装；
- `time_frame` 的单位和时钟源；
- 真机实际分辨率是否始终为 1280x720；
- 真正的 camera frame、外参和 acquisition timestamp；
- Foxy 真机运行兼容性。

离线 H.264 流验证**不表示 Go2 真机视频已跑通**。

本包不发布 `CameraInfo`，不伪造深度，不修改现有 detector 或 `vision_interfaces`。
