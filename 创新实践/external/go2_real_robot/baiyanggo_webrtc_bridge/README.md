# baiyanggo_webrtc_bridge

Go2 内置雷达 **WebRTC→ROS2 桥**（狗的 DDS 转发静默时的自建数据通路）。

## 话题

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/baiyanggo/webrtc/pose` | geometry_msgs/PoseStamped | ~19Hz | odom 系位姿 |
| `/baiyanggo/webrtc/voxel_map` | sensor_msgs/PointCloud2 | ~10Hz（运动时） | 体素点云（已解码） |
| `/baiyanggo/webrtc/lidar_state` | std_msgs/String (JSON) | ~5Hz | 雷达自诊断（丢包率/转速/版本） |
| `/baiyanggo/webrtc/status` | std_msgs/String (JSON) | 0.5Hz | 桥健康（连接态/计数器） |

## 部署（.18，狗开机状态下）

```bash
./baiyanggo-install.sh                          # venv+补丁+systemd 注册（幂等）
systemctl --user start baiyanggo-webrtc-bridge
journalctl --user -u baiyanggo-webrtc-bridge -f # 看日志
```

注意：user 级 systemd 需要 `loginctl enable-linger unitree`（SSH 无会话时保持服务运行）。

## 架构

- 信令：HTTP `.161:9991`（con_notify/con_ing，新法）
- 体素流走驱动的二进制帧入口（挂钩 `deal_array_buffer_for_lidar`，向量化解码比参考实现快）
- 自动重连（狗的 WebRTC 服务对快速重连敏感，服务级 RestartSec=15 + 内部会话循环）
- 冒烟测试：`test/test_bridge_mock.py`（mock 驱动，四项断言，已在 Foxy 容器 PASS）

## 已知限制

- 体素点云是体素化产物（无逐点时间戳/intensity），适合建图可视化与数字孪生；
  FAST-LIO 级 SLAM 请用外置 MID-360（官方 uslam 路线）
- 体素流运动门控：狗静止不推送
