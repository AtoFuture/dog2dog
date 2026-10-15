# AMCL / DeepSeek / G3 隔离仿真验证

日期：2026-10-15。地图为现有 warehouse 静态地图，ROS_DOMAIN_ID=189，独立 Gazebo partition。
调用真实 DeepSeek API；底盘为 Go2 仿真，无真机 SDK。已停止测试仿真及导航进程。

## 本轮改动

- 使用 AMCL 发布 map→odom，EKF 独占 odom→base_link；足端原始话题与滤波输出分离。
- EKF 只融合足端 x/y 位置与原始 IMU yaw/角速度；去除足端反号角速度和命令速度的重复融合。
  足端原始算法本身未重写，不能据此宣称其累计漂移已修复。
- 复用水平单线雷达副本，地图和场景障碍未改动，共享仿真文件只读。
- G3 RETURN 先请求唯一 token，探索桥接等 action 终态后确认释放，再发送返航目标。
  未确认、过期确认和急停后的确认均不放行；等待超时进入 ESTOP。
- 返航和待机期间暂停 DeepSeek 请求；回到探索时恢复。

## 实测结果

用仓库内 prepare_simulator_launch.py --single-odom-tf、prepare_amcl_navigation.py
生成启动配置，再运行 validate_amcl_navigation.py --cycles 3。总时长 278.22 秒。

| 轮次 | 探索目标真值误差 | 返航真值误差 | 结果 |
|---|---:|---:|---|
| 1 | 0.299 m | 0.306 m | 探索 / 低电量 RETURN / STANDBY 完成 |
| 2 | 0.305 m | 0.356 m | 同上，再次探索正常 |
| 3 | 0.422 m | 0.284 m | 同上，覆盖新目标延迟接受时的返航交接 |

3 次探索到点 + 3 次返航到点；额外排队的第 4 个探索目标按交接要求取消。
紧急距离保护干预 0 次，观测前方最近距离 0.367 m；这不是独立的碰撞检测认证。
AMCL 位姿 352 次，按接收时间对齐真值的最大观测位置误差 0.315 m；尚非严格时间同步的 APE 评测。

通过标准：探索 action 为 SUCCEEDED；返航 action 为 SUCCEEDED 且真值距基站 ≤0.4 m。
保留原配置 xy_goal_tolerance=0.25、yaw_goal_tolerance=6.28，未验证到站朝向或充电对接。
探索第三轮真值误差超过 0.4 m，所以本轮不能视为精确到点验收。

本地完整证据位于 .runtime/deepseek_amcl_repeat/：report.json、trajectory.csv、
localization.json、plans.json、trial.log、nav2.log、截图及 amcl_summary.png。
它们属于运行产物，未提交到 Git。回归：导航与 G3 共 215 项测试通过，两包 ROS 构建通过。

## 尚未完成的完整系统验收

未知地图 SLAM 与 frontier 覆盖率、目标检出闭环、真实电量与真机通信/运动、长期漂移、
物理停靠和充电，以及 rosbag 的完整指标评测仍需分别验收。
当前真值只用于评估和仿真站立保护，不进入导航 TF 或里程计。
