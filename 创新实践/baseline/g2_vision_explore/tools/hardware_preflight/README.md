# G2 Go2 hardware preflight

这是下一次连接真实 Go2 后的第一批验收工具。实现入口在
`vision_detector` package，因此既可以从源码运行，也可以使用：

```bash
ros2 run vision_detector hardware_preflight \
  --duration 15 \
  --capture-count 10 \
  --output-dir /tmp/go2_preflight_01
```

默认检查：

```text
/frontvideostream                 unitree_go/msg/Go2FrontVideoData
/camera/go2/image_raw             sensor_msgs/msg/Image
/utlidar/cloud                    sensor_msgs/msg/PointCloud2
```

输出：

- `hardware_preflight_report.json`：机器可读报告；
- `hardware_preflight_report.md`：人可读报告；
- `captures/image_XXXX.png`：最多按 `capture-count` 保存的图像；
- `captures/cloud_XXXX.cdr`：原始序列化 PointCloud2；
- `captures/metadata_XXXX.json`：图像、点云、真实 header timestamp/frame、差值和 TF snapshot。

采集使用“最近已观察到的 image/cloud 配对”，metadata 会保留真实 timestamp delta，
并标记 `clock_sync_status=UNCONFIRMED`；工具不会把两条消息强行写成同步。

报告中的 `OBSERVED`、`INFERRED`、`UNCONFIRMED` 有严格区别：`time_frame` 的单位和
时钟源不会因为数值量级看起来相似就被自动转换。

## 内参准备

只用真实棋盘格图片：

```bash
ros2 run vision_detector calibrate_intrinsics \
  --input-dir /path/to/real_chessboard_images \
  --output /tmp/go2_intrinsics_only.json \
  --columns 9 --rows 6 --square-size-m 0.025
```

输出包含 `fx/fy/cx/cy`、畸变、图像尺寸和重投影误差，并明确标记
`calibration_status=intrinsics_only`。它不填写 camera frame、LiDAR frame 或
`T_camera_lidar`，不能直接宣称完成 Go2 LiDAR 标定。

## 上狗 checklist

1. `ros2 topic list` 确认 `/frontvideostream` 和 `/utlidar/cloud` 的真实名字。
2. 运行 preflight，先不带 `--capture-count`，确认 topic/type/publisher/QoS/rate。
3. 保持 Go2 静止，运行 `--capture-count 10`，保存棋盘格/共同场景数据。
4. 用真实棋盘格图片运行 `calibrate_intrinsics`，检查有效视角数和重投影误差。
5. 单独确认 `map/odom/base_link`、真实 camera frame、真实 LiDAR frame 的 TF。
6. 在确认 `time_frame` 与 PointCloud2 时钟域前，不把 `READY_FOR_RGB_LIDAR_FUSION` 写成 true。

本工具只做诊断和采集，不发布 TF、不修改 `/detections_3d`，也不实现外参自动标定。
