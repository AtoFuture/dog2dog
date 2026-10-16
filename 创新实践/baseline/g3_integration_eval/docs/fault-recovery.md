# 探索故障暂停与续跑

新增 `PAUSED / paused` 状态，保留同一个进程内的侦察任务、目标证据、身份解析器、已完成导航目标及观察视点。恢复后由 G2 基于当前地图重新选点；被取消的 Nav2 action 不复用旧 UUID。没有添加进程崩溃后从磁盘重建任务的功能。

## 行为

开启健康监测后，激光断流或 `map → base_link` 定位 TF 不再更新，G3 从 EXPLORE 进入 PAUSED，广播任务暂停并请求 G2 取消导航。必须收到当前 token 的 `released=true` 终态确认，且所有健康源连续恢复 3 秒，才会自动重新进入 EXPLORE。旧确认、重复消息和冻结时间戳不会提前恢复。G2 现有逻辑会在停止探索时取消未完成的模型请求。

- 默认健康源：`/robot1/scan` 和定位 TF。可额外配置 RGB、深度 Image 话题。
- 默认新鲜度 2 秒、恢复稳定期 3 秒、暂停上限 60 秒；这些预算使用单调时间，独立于 `/clock`。
- 消息头必须在 ROS 时钟的有效新鲜度内，且时间戳持续推进。激光至少有一个量程内的有限回波；Image 必须含完整的非空图像数据；TF 必须有限且旋转归一化。
- 导航交接默认最多 5 秒；未确认退出则 ESTOP。恢复超时也进入 ESTOP，后续健康消息不会自动解除。
- 手动 PAUSE 必须手动 RESUME，也要通过取消完成、健康稳定和电量检查。
- 低电量优先返航；暂停时健康尚未稳定则进入 ESTOP，不向失效的导航系统下发返航目标。返航中健康丢失也进入 ESTOP，本版不自动重试返航。
- 暂停期间不消耗搜索预算与最短观察时长；证据新鲜度仍按照真实图像时间检查，暂停不会让旧证据变新。

此功能依赖现有 Nav2 取消链和底层速度看门狗，不提供独立的硬件急停接口，也不对定位精度作保证。全局断网可由消息断流触发；仅 DeepSeek 服务超时仍走现有有界请求与安全候选回退，没有在此改写模型策略。

## 接入

默认 `enable_fault_recovery=false`，保持已有实验配置。启用时必须配合 G2 和导航交接：

```bash
ros2 run g3_integration_eval g3_state_machine --ros-args \
  -p require_navigation_handoff:=true \
  -p enable_fault_recovery:=true \
  -p recovery_scan_topic:=/robot1/scan \
  -p recovery_map_frame:=map \
  -p recovery_robot_frame:=base_link
```

仿真还需 `-p use_sim_time:=true`。若视觉任务要求 RGBD 断流一起停车，补充实际相机话题，例如 `-p 'recovery_image_topics:=[/camera/color/image_raw, /camera/depth/image_raw]'`。不要填写系统中不存在的话题。导航发送仍由原有 `enable_navigation_transmission` 显式控制。

启动后等健康源稳定再发送 EXPLORE；未通过健康检查的启动请求会被拒绝，不会排队自动启动。手动命令仍使用 `/g3/command` 的 `std_msgs/String`，新增 `PAUSE` 与 `RESUME`。

`/g3/recovery_event` 输出故障原因、缺失源、暂停、导航释放、恢复和恢复超时事件，包含单调时间及 ROS 时间。侦察节点将最近 1000 条写入 `mission_report.json.recovery_history`，并记录累计暂停时间。需要保留全部原始时间线时，将此话题加入录包列表。

## 验证范围

新增测试覆盖状态转换、连续健康恢复、旧时间戳拒绝、扫描有效性、取消 token、交接超时、恢复超时、低电量优先、手动暂停、任务进度与预算保留。真实 ROS DDS 测试发布 TF/激光，停止激光，验证暂停，再恢复消息并通过交接确认续跑；测试期间导航发送关闭，不发布机器人速度。

后续已完成普通仓库 Gazebo 运动故障注入，记录见 [运动验收](../../../external/go2_real_robot/qwen_robotnav/docs/fault-recovery-validation.md)。尚未进行实机故障试验；多人跟踪和动态障碍实验保持冻结。

2026-10-16 验证：G3 全量 240 项、G2 兼容性回归 279 项通过。两项 DDS 故障测试分别覆盖激光和定位 TF 断流；未启动 Gazebo 或机器人执行器。
