# G1 可信运动与建图闭环 · 工具与验证方法

> 2026-09-23。对应计划文档 `docs/项目验证修复与智能大脑集成计划.md` 的 **P1**。
> 出口标准：ground truth 到点误差 ≤ 0.30 m 且成功率 ≥ 4/5；静止 120 s 虚假位移
> ≤ 0.02 m；不出现重复控制器；rosbag 含地图、TF、Nav2 goal/result 和真值。

## 一、仿真侧的前置改动（**不在本仓库**，需自行落地）

工具依赖仿真工作区 `~/go2_sim_ws` 的两处改动。它们改的是共享代码，**没有进本仓库**，
换机器或别人复现时必须先补上：

### 1. `ROS2-Gazebo-GO2/src/gazebo_sim/launch/launch.py` —— 无头支持

原本 `gz_args` 写死 `-r -v4`，没有无头入口。加一个受 `GZ_HEADLESS` 控制的 `-s`，
并在无头时把子 launch 本来就有的 `enable_rviz` 开关转发下去
（不转发的话 rviz2 在无 DISPLAY 下 abort，会把整个 launch 带崩）。

### 2. `~/go2_sim_ws/run_sim_iso.sh` —— 无头渲染的 EGL 修复

`gz sim -s` 起服务端后，传感器渲染线程报
`libEGL warning: egl: failed to create dri2 screen` 刷屏，然后 SIGSEGV。

根因：enroot 只注入 NVIDIA 的 `.so`，**不注入 glvnd 的 EGL vendor 配置**，
容器里 `/usr/share/glvnd/egl_vendor.d/` 只有 `50_mesa.json`，
于是 EGL 被派发给 Mesa 走 DRI2（那是需要 X 的路径）。

解法（非侵入，不动容器镜像）：把 `10_nvidia.json` 放进已挂载的 `/tmp`，
用 `__EGL_VENDOR_LIBRARY_FILENAMES` 显式指过去。**必须用 GUI 容器（`~/.enroot/ros-gui`）
拿 GPU 库**，纯 `ros` 容器没有 NVIDIA 图形库。

### 3. `~/go2_sim_ws/g1ros.sh` —— 隔离域里跑 ROS 命令的入口

`run_sim_iso.sh` 的三层隔离（DISPLAY `:3` + `ROS_DOMAIN_ID=43` + `IGN_PARTITION=wy`）
里有两条必须在**容器内**设置才生效（enroot 不继承宿主环境）。
每次手敲容易漏，漏了会静默连到别人的仿真上。本目录所有工具都通过它进容器：

```bash
cd ~/go2_sim_ws
./g1ros.sh bash <本目录>/<脚本> [参数]
```

## 二、标准流程

```bash
cd ~/go2_sim_ws

# 1) 起隔离无头仿真（约 1 分钟；实时率约 0.72，120 s 仿真要留 167 s 挂钟）
GUI=1 GZ_HEADLESS=1 ROS_DOMAIN_ID=43 IGN_PARTITION=wy ./run_sim_iso.sh &
#    GUI=1 是为了要 ros-gui 容器的 GPU 库，GZ_HEADLESS=1 才不画窗口

# 2) 起控制器 + 速度缩放器 + Nav2（脚本内会检查 joint_group_controller 是否激活）
./g1ros.sh bash <本目录>/start_nav2_isolated.sh

# 3) 给 AMCL 初始位姿（配置里没有 set_initial_pose，必须外部发）
./g1ros.sh bash <本目录>/set_initial_pose.sh

# 4) 跑目标点，录包，按真值判定
./g1ros.sh bash <本目录>/goal_test.sh 10.5 3.0 0.0 80 /tmp/p1/goal_1
```

## 三、工具清单

| 文件 | 作用 |
|---|---|
| `gt_record.sh` | 起真值桥 + 中转并录包。**必须带 `--include-hidden-topics`**，否则 Nav2 的 `_action/*` 隐藏话题录不到 |
| `gt_relay.py` | 关键一环：`ros_gz_bridge` 桥 `Pose_V→TFMessage` **不填 header.stamp（全为 0）**，不打戳的话算出来的误差是错位样本相减的假数 |
| `gt_compare.py` | 真值/odom/EKF 三方比对。**按位移而非绝对坐标比**——odom 以自身起点为原点，狗在世界里未必生成于原点，直接相减会把原点偏移误报成漂移 |
| `drive_probe.sh` | 定速驱动探针，用来暴露里程计幻觉 |
| `odom_replay.py` | 离线复现足端里程计算法，修法可在离线迭代，不必反复跑仿真 |
| `cmd_vel_scaler.py` | 把 Nav2 的速度缩放到步态能执行的区间 |
| `nav2_scaled.launch.py` | 我们自己的 Nav2 启动，把速度输出引到 `cmd_vel_scaled_out` 再经缩放器写回 |
| `start_nav2_isolated.sh` | 激活控制器 → 起缩放器 → 起 Nav2 |
| `set_initial_pose.sh` | 从真值设 AMCL 初始位姿 |
| `goal_test.sh` | 发目标 + 录包 + 真值判定 |

## 四、必须知道的三个坑

### 1. `joint_group_controller` 会停在 `unconfigured`，狗一动不动

仿真看着全好：`/clock` 在走、节点齐全、`commands` 话题发布者数 = 1、
`quadruped_controller` 以 29 Hz 照发命令——**但真值净位移 0.0000 m，z 恒在生成高度
0.446 而非站立 0.27**。查 `list_controllers` 会发现它没被激活。
`start_nav2_isolated.sh` 第 1 步会自检并停下，避免白跑一轮。

### 2. action 的 SUCCEEDED 不能当到达判据

实测 G2 那轮 action 报 SUCCEEDED，真值却在 0.362 m 外——因为 goal checker 判的是
**AMCL 的位姿**，而 AMCL 当时对真值偏了 0.464 m。
根因是 `update_min_d: 0.25` / `update_min_a: 0.2` 让 AMCL 约 0.3 Hz 才更新一次。
收紧到 `0.05` 并把 `transform_tolerance` 从 `10.0` 降到 `1.0` 后，
更新次数涨 3~5 倍、对真值最大偏差降到 ≤0.224 m，到点误差稳定在 ~0.11 m。

### 3. 速度指令被放大 ~10 倍

`cmd_vel_pub.py` 里 `multiply_and_limit` 的 scale 被从上游的 `0.035` 改成了 `0.3`
（仿真工作区里一处未提交改动），而步态对 rv 的增益本身约 10 倍。
合起来 Nav2 按配置发 0.26 m/s 实际会跑出 ~2.7 m/s、rv 顶到 0.27，
远超实测稳定边界 **rv < 0.04**（0.06 档就原地蹬腿不前进）。

`cmd_vel_scaler.py` 的系数 0.115 与上游 0.035 相乘正好 ≈0.0345，
**本质上是把我们这一路恢复到上游默认值**。缩放只落在我们这份配置里，共享代码未动。

## 五、验收结果（2026-09-23）

| 指标 | 结果 |
|---|---|
| 静止 120 s 虚假位移 | **0.0000 m**（限值 0.02） |
| 到点误差（5 目标，收紧 AMCL 后） | 0.121 / 0.105 / 0.111 / 0.115 / 0.269 m，**全部 ≤0.30** |
| 摔倒次数 | 0（五轮 z 全程 0.25~0.28） |
| rosbag 四要素 | 地图 ✅ TF ✅ goal/result ✅ 真值 ✅ |

**尚未做的**：P1 第 5 项「导航期间保持 SLAM 运行，证明地图能边走边更新」——
上述验收用的是 AMCL 加载**静态**地图，不是 SLAM 边跑边建图。
