# G3 · 集成评测

> 真机通信 · 系统集成 · 指标评测

**人**：2 人

---

## 交付物

| # | 交付物 | 验收标准 |
|---|---|---|
| 1 | 真机运行 | 通过 `unitree_ros2` 遥控 Go2，状态机能接管 |
| 2 | 低电量返航 | 触发后安全回到基站，成功率 100% |
| 3 | 指标报告 | 覆盖率、检出率、误差等，每次实验自动落盘 |
| 4 | 演示视频 | |

---

## 第一周就要动手

**DDS 通信是最高风险项，而且它阻塞所有人上真机。** 不等其他模块，第一周就开工。

- `unitree_ros2` / `unitree_sdk2` 打通，`cyclonedds_ws` 编译过
- 先用**官方 demo** 打通遥控，再谈接管控制
- 跑通后立刻发一条消息到群里：「DDS 通了」——这一条决定后面所有人什么时候能上真机

同时做**设备盘点**（见下），G1 在等这个结果。

---

## 任务分解

### ① 真机通信（第 1–4 周）

- `unitree_ros2` 打通，确认网卡配置、DDS 域号、QoS
- 状态机：待机 / 探索 / 返航 / 急停
- 安全策略：急停按钮、通信超时自动停、速度上限

### ② 设备盘点（第 1 周，最优先）

把狗拿出来跑一次官方 demo，然后：

```bash
ros2 topic list      # 真机到底发布了哪些话题
ros2 topic hz /xxx   # 关键传感器的实际频率
```

比看说明书准得多，能发现实际配置与标称配置的差异。

要弄清的五件事：

| # | 要弄清什么 | 为什么关键 |
|---|---|---|
| 1 | **型号**：Air / Pro / Edu / X？ | Air 没有激光雷达，配置完全不同 |
| 2 | **传感器清单**：激光雷达？深度相机？IMU？ | 决定 SLAM 走激光还是视觉 |
| 3 | **机载算力**：有没有 Jetson Orin？ | 决定 YOLO 跑机上还是回传 |
| 4 | **SDK 权限**：关节级还是只有高层速度指令？ | 决定仿真控制层做到哪一级 |
| 5 | **相机安装位**：高度、俯仰角、离重心偏移 | 外参不复现，仿真参数上真机全偏 |

顺手把传感器安装位置**拍照、量尺寸**。

> ⚠️ 如果盘下来是 **Air 版、没有激光雷达**，课题难度会明显上升——纯视觉 SLAM 在足式平台上本就难做。这种情况要立刻通知全队调整方案。**越早盘越好。**

结果写进 `docs/设备清单.md`，G1 靠它决定仿真里挂什么传感器。

### ③ 低电量返航（第 10–12 周）

- 读电量 → 低于阈值 → 发一个回基站的目标点
- 当前已实现：订阅可配置 `BatteryState` 话题，仅在 `EXPLORE` 且电量低于阈值时自动切换到 `RETURN`。默认话题为 `/robot1/battery_state`，默认阈值为 `0.20`。
- `enable_navigation_transmission` 默认值为 `false`：自动返航只生成并打印目标预览，不等待 action server，也不发送 goal。
- 只有显式设为 `true` 时才等待 `/robot1/navigate_to_pose`，发送 home pose，并在 `/g3/return_event` 发布 `std_msgs/String` JSON 事件。事件包含 `attempt_id`、真实 Nav2 `goal_id`（接受/拒绝响应可用后）、触发源、ROS 时间戳、home target 和状态。
- 状态链包括 `preview / waiting_for_server / server_unavailable / goal_request_sent / accepted / rejected / succeeded / aborted / canceled`，回调异常也会形成明确失败事件，便于 rosbag 离线追踪。
- **复用接口 02 的 `NavigateToPose`**，不另开通路。G2 的探索目标点和你们的返航点走同一个入口
- 加一层：返航途中电量继续掉怎么办（降速 / 就近停靠 / 直接趴下）

### ④ 指标评测（第 13–16 周）

| 指标 | 定义 | 目标 |
|---|---|---|
| 探索覆盖率 | 已建图面积 ÷ 可通行总面积 | > 90% |
| 建图精度 | 绝对轨迹误差 ATE | < 0.15 m |
| 目标检出率 | 正确检出 ÷ 场景内全部目标 | > 85% |
| 误检率 | 错误检出 ÷ 全部检出 | < 10% |
| 单次任务耗时 | 出发到完成侦查并返回 | 记录基线 |
| 返航成功率 | 低电量触发后安全返回 | 100% |

**工具做在前面**：统一 rosbag 话题清单 + 指标计算脚本，每次实验自动落盘。别等到第 15 周才发现数据没记全、得重跑。

> 覆盖率随时间的变化曲线是最有说服力的一张图——它同时体现探索策略与建图稳定性。

### 当前离线导航统计规则

- `GoalStatusArray` 可能携带录包开始前的历史 goal，因此离线评估只把**当前 bag 中实际出现过 NavigateToPose feedback 的 goal UUID**计为本次实验的 observed goal。
- observed goal 的唯一终态为 `SUCCEEDED / ABORTED / CANCELED` 时才进入导航成功率分母；没有终态记为 `incomplete`，同一 UUID 出现多个不同终态记为 `conflict` 并排除。
- 导航成功率定义为 `SUCCEEDED / (SUCCEEDED + ABORTED + CANCELED)`。若本次 bag 没有 observed goal 到达终态，则输出 `unavailable`，而不是误报 `0%`。
- `/g3/return_event` 将 G3 `RETURN` 尝试与 Nav2 goal UUID 明确关联。离线评估按 `attempt_id` 去重：preview 不进入成功率分母；真实发送后的成功、取消、中止、拒绝、server unavailable 与发送/回调错误进入分母。旧 bag 不含该话题时，`return_success_rate` 仍保持 `unavailable`，原因是 `no_explicit_return_goal_marker`。

### 返航发送安全开关

默认启动不会发送导航目标：

```bash
ros2 run g3_integration_eval g3_state_machine
```

仅在 Nav2 与传感器链健康、且确认允许控制目标机器人后显式开启：

```bash
ros2 run g3_integration_eval g3_state_machine --ros-args \
  -p enable_navigation_transmission:=true \
  -p navigation_server_timeout_sec:=2.0
```

home target 仍由 `home_frame / home_x / home_y / home_yaw` 参数配置。

---

## 对外接口

- 收 G1：地图与定位（接口 01）
- 收 G2：`Detection3D`（接口 03）
- 发 G1：低电量返航的 `NavigateToPose`（复用接口 02）

---

## 本周要干的事

- [ ] `unitree_ros2` 编译过，官方 demo 遥控走通
- [ ] **设备盘点五项**，结果写进 `docs/设备清单.md`，通知 G1
- [ ] 建 rosbag 话题清单初稿（接口 04）

---

## Mission Brain（DeepSeek Flash 安全 MVP）

G3 集成层已加入 `mission_brain` 节点，默认调用 DeepSeek 官方
OpenAI 兼容接口，模型 ID 为 `deepseek-flash`。密钥只能通过
`MISSION_BRAIN_API_KEY` 环境变量提供，不得写入代码、YAML、launch 文件或日志。

```bash
source /opt/ros/humble/setup.bash
source /tmp/g3_eval_colcon/install/setup.bash
export MISSION_BRAIN_API_KEY="<new-rotated-key>"
export MISSION_BRAIN_MODEL="deepseek-flash"
ros2 run g3_integration_eval mission_brain
```

### 临时 JSON 话题协议

共用 ROS 消息尚未冻结，MVP 先用 `std_msgs/String` 承载 JSON：

- `/mission/context`：至少含 `mission_state` 和 `localization_healthy`。
- `/exploration/candidates`：含 `candidates` 和当前可用 `evidence_ids`。
- `/brain/decision`：受限高层动作，不含任意坐标或速度指令。

候选集示例：

```json
{
  "set_id": "map-17",
  "evidence_ids": [42],
  "candidates": [
    {"id": "f1", "x": 1.2, "y": -0.5, "information_gain": 18.0,
     "path_cost": 4.1, "failure_count": 0}
  ]
}
```

安全边界：

- 模型只能从输入的 frontier ID 中选择，不能自造坐标。
- `COMPLETE` / `VERIFY_DETECTION` 必须引用当前存在的检测证据 ID。
- 非 `EXPLORE` 或定位不健康时只能安全等待。
- API 超时、断网、格式错误或越界输出会自动回退到确定性 frontier 打分。
- 本节点只发布决策，不发布 `cmd_vel`，也不直接发 Nav2 goal。
