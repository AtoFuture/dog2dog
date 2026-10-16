# 三维人员侦查任务验收（2026-10-15）

> 历史验收记录：当前正式主线见 [统一运行说明](navigation-runtime.md)。本页保留既有结果；使用旧场景 runner 复现须显式加 `--legacy-validation`，旧 shell 场景须设置 `SLAM_LEGACY_VALIDATION=1`。人物、动态障碍实验继续冻结，本轮不重跑。

本轮把“侦查对象”接入任务完成条件：检测确认人员、完成有效探索目标、交接导航权、RETURN 成功、生成可复核报告。
任务节点不读取人物位置、Gazebo 真值或参考地图；这些数据只由离线评估使用。

## 场景与复现

Gazebo warehouse 场景副本放入 OpenRobotics Standing person 第 3 版静态三维模型。
来源与许可：
[官方模型元数据](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person)、
[OpenRobotics 模型仓库](https://github.com/osrf/gazebo_models/tree/master/person_standing)，CC0-1.0。
`scripts/recon_person_asset.json` 固定 12 个文件的 SHA256，`fetch_recon_person.py` 验证缓存或下载。
大型模型文件只在运行资产目录，未提交到代码仓库。

```bash
python3 ./scripts/fetch_recon_person.py /tmp/go2_person_asset
SLAM_RECON_TASK=1 SLAM_RECON_PERSON_MODEL=/tmp/go2_person_asset \
  SLAM_ROS_DOMAIN_ID=201 ./scripts/run_slam_frontier.sh /tmp/go2_recon_run 1
```

人物位于场景世界坐标 (10.3, 6.0)，朝向机器人；场景资产清单写入 `scene_manifest.json`，不提供给导航或任务节点。
相机直接拍摄三维模型，深度直接来自同一个 Gazebo RGB-D 传感器，未贴照片或注入常数深度。
起步可先观察最多 30 秒，再执行探索。这个实验验证可见静态目标，不能称为从未知位置搜索整座仓库。

## 完成条件

G3 `recon_task` 默认要求 person：同一个非零跟踪 ID、置信度至少 0.5、两次不同采集时间、间隔不超过 10 秒、三维代表点相距不超过 0.75 米，两条观测都有可配对 JPEG。
完成至少一个测地距离不小于 0.75 米的探索目标后，任务节点发出 RETURN。
G2 取消/交接自己的导航后，G3 发出返航 action；只有匹配 attempt ID 的成功终态才将任务置为 returned。
外部低电量提前返航标为 interrupted，不会冒充目标任务完成。异常退出标为 failed/aborted。

## 深度质量修复

侧面目标初版实验曾被深度质量检查拒绝。真实同步 RGB-D 的掩膜中位深度约 2.98 米，但轮廓边缘混入了最远约 9.86 米的背景。
离线对同一帧验证：原始掩膜投影被拒；内缩 1–3 像素后通过，3 像素时离散度约 0.631 米。
现在检测节点默认内缩 2 像素，保留原有 3 米质量阈值；过细掩膜会因为有效像素不足而丢弃，不会降级伪造距离。
`project_depth_bbox` 的库默认内缩 0，兼容现有调用；检测节点可用 `mask_depth_margin_px` 显式配置。

## 录包完整性

中间轮 `recon_person_frontal` 的节点报告显示任务返航成功、返航误差 0.271 米；人员平面误差约 0.142 米、两张 JPEG 及对应时间戳的 RGB-D 都可复核。
但调试过程中直接查询活动 SQLite 引发了录包写锁错误，后半段 action/任务完成事件缺失。因此这轮**不通过完整验收**，尽管导航和任务节点报告成功。
原始数据库完整性检查为 ok；停止后仅重建 metadata，未补造终态消息。离线验收正确输出 `recorded_return_success=false` 和 `recorded_mission_completed=false`。

后续增加录包进程退出监控和 metadata 完整性要求。完整验收必须在 bag 中同时找到匹配的 G3 RETURN 成功、Nav2 成功终态和任务 returned 事件。
运行期间只查看日志、节点输出 JSON，不查询活动录包数据库；所有 SQLite/rosbag 分析均在录包关闭后进行。

## 回归及范围

按包隔离执行相关回归：G2 为 264 项，G3 为 180 项，导航包为 52 项，合计 496 项。
临时将三个包合并到同一 pytest 进程时，地图导出测试重复失败；停止仿真后该单测 1.26 秒通过，G3 独立全套和 G2 加地图导出的组合也通过。该跨包进程干扰尚未完全定位，因此本轮采用按包隔离的测试结果，不宣称单进程合并全绿。新增 map_saver 日志保留错误输出，未改变原断言。新增测试覆盖掩膜边缘背景、过细掩膜拒绝、单帧/旧证据/跨轨迹拒绝、任务目标与探索门槛、返航 attempt 匹配及场景副本不修改原始文件。

人员位置误差只在世界平面 XY 上与模型原点比较。Z 属于当前 map 坐标系，未标定为离地高度。
本节为单人阶段记录。多人及遮挡的后续结果见 [多人及遮挡验证](recon-multi-occlusion-validation.md)；移动目标、真实世界检出率、全场搜索、倒地事件或真机控制尚未验收。

## 返航速度反馈修复

后续 `recon_person_verified` 完整录包正常，但返航超时。离线反馈显示机器人接近到约 0.4 米后持续原地转向，缩放器长期保持线速度 0、角速度 0.200。
生成配置为 BT 和速度平滑器设置了 `/robot1/odometry/filtered`，却遗漏 controller_server；原始里程计此前已改到 `leg/odometry`，控制器的默认 `odom` 没有发布者。

[Nav2 Humble 里程计订阅实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/odom_subscriber.hpp) 默认使用 `odom_topic=odom`；
[RPP 转向实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp) 根据当前角速度和加速度限制下一条指令。
缺少反馈时，10Hz、1.2 rad/s² 的配置会把转向限制在约 0.12 rad/s，经过缩放正好是日志里的 0.200。
已让控制器也订阅融合里程计，保留原来的位置阈值、加速度与返航超时；新增生成配置测试检查三个消费者一致。


## 最终完整验收

目录：`/home/wy/data1/.runtime/recon_person_odom_fixed`，隔离 ROS 域 203。`report.json`、`mission_report.json` 与 `recon_validation.json` 全部成功，入口退出码 0。

- 任务总用时 119.64 秒；完成 1 个有效探索目标，另有 2 个启动近点。
- DeepSeek 真实选点 3 次，回退 0 次。
- 两条人员确认观测的平面误差分别为 0.1407、0.1410 米；两张 JPEG 均可解码，原始 RGB 与深度都按确认时刻精确匹配并导出。
- 返航真值误差 0.2781 米，触发来自人员任务完成，并非低电量注入。
- 录包正常结束（退出码 0），metadata 存在；bag 同时含 Nav2 RETURN 成功、匹配的 G3 成功事件和任务 returned 事件。
- 四个导航 action 全部成功，无未终结或冲突终态；紧急前向保护介入 0 次。
- 最后接收的检测状态记录 62 帧、24 个二维检出、11 条三维观测；1 个三维解算被质量阈值拒绝，TF 失败和“最新 TF”降级均为 0。

`controller_feedback_check.json` 记录只读 ROS 图核验：控制器确实订阅融合里程计，收到 56 条反馈，最大观测角速度约 0.355 rad/s。
地图、轨迹图、原始 RGB-D bag、抓拍、任务报告和离线误差报告均已生成；仿真已自动清理停止。
