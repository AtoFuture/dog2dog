# Nav2 前沿自主探索接入报告

> Unitree Go2 · Ignition Gazebo Fortress · ROS 2 Humble · 命名空间 `/robot1`
> 目标：用 `m-explore`(explore_lite) 做前沿自主探索，得到**覆盖整间**的地图
> 日期：2026-10-13/14

## 现状

- Nav2 组件齐全（`nav2_bringup` / `bt_navigator` / `controller` / `planner` / `costmap_2d`）。
- `explore_lite` + `explore_lite_msgs` 已编译（见 `sota_integration/explore/`）。
- **运行卡在 Nav2 配置；且最终被环境阻断。**

## 关键问题：Nav2 `controller_server` 起不来

用标准方式启动：

```bash
ros2 launch nav2_bringup navigation_launch.py namespace:=robot1 use_sim_time:=true \
    params_file:=<...>/navigation2/param/go2_nav2.yaml
```

报错：

```
[controller_server]: Couldn't load critics! Caught exception: No critics defined for FollowPath
[controller_server]: Lifecycle node controller_server does not have error state implemented
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

⇒ `controller_server` 配置失败 → **整条 Nav2 一个节点都没激活** → **没有 `/robot1/navigate_to_pose`** → `explore_lite` 永远 "Waiting to connect"，机器人不动，覆盖率恒为 2.1%。

### 诊断

- `go2_nav2.yaml` 里 `FollowPath.critics` **写得完全正确**（`sed -n '168,172p' | cat -A` 验证无隐藏字符；install 是指向 src 的符号链接，内容一致）。
- `controller_plugins: ["FollowPath"]` **加载成功**（`Created controller: FollowPath of type dwb_core::DWBLocalPlanner`）——即参数文件**被读到了**，但同一 `FollowPath` 块里的 `critics` **没到节点**。
- 结论：**`nav2_bringup` 在带 `namespace` 时对参数文件做 `RewrittenYaml(root_key=namespace)` 重写，`FollowPath` 子块（尤其 `critics` 列表）在这一步丢失。**

## 已实现的修复（绕过 nav2_bringup）

1. **把参数文件整体包在命名空间根下**：生成 `go2_nav2_ns.yaml`，即
   ```yaml
   robot1:
     controller_server:
       ros__parameters: {...}
     planner_server:
       ros__parameters: {...}
     ...
   ```
   （把原 `go2_nav2.yaml` 每行缩进 2 空格、顶层加 `robot1:`）。这样键与 `/robot1/<node>` 精确匹配，且**不经过 RewrittenYaml**。
2. **手工启动 Nav2 节点**（不通过 `nav2_bringup`）：
   - `nav2_controller/controller_server`
   - `nav2_planner/planner_server`
   - `nav2_behaviors/behavior_server`
   - `nav2_bt_navigator/bt_navigator`
   - `nav2_smoother/smoother_server`
   - `nav2_velocity_smoother/velocity_smoother`
   - `nav2_lifecycle_manager/lifecycle_manager`（`autostart:=True`, 管理上述节点）
   全部 `namespace:=robot1`、`--params-file /tmp/go2_nav2_ns.yaml`、`use_sim_time:=true`。
   见 `eval/nav2_manual.launch.py`。

## 未完成：被环境阻断

实现就绪后**四次尝试中，Gazebo server 都在 bringup 阶段被卡死/被杀**（`ign service ... pause:false` 服务超时；观测到 SIGKILL）。当日下午至今主机长期 **load 19–38、8 个在线用户、物理内存反复掉到约 1 GB**（队友 `nnUNetv2_train`×3、PyTorch 训练），且存在**第二个 gz server**（他人 `multi_room` 世界）。

⇒ **Nav2 + 前沿探索的运行验证受限于机器负载，而非方案**。

## 另：进程隔离

本项目 gz session 已设 `IGN_PARTITION=go2wy` / `GZ_PARTITION=go2wy` 隔离他人 server（脚本内）。

## 待机器空闲后的一键流程

```bash
~/.enroot/ros bash -lc 'bash /tmp/explore_nav2b.sh 900'
# 内部：起全栈(gz+桥+机器人+slam) -> 生成 go2_nav2_ns.yaml -> 手工起 Nav2 -> explore_lite -> 监控覆盖 -> 存图
```

期望结果：`/robot1/navigate_to_pose` 出现、Nav2 各节点 `active`、覆盖率 `known%` 持续上升、`explore_lite` 自动终止后存出"覆盖整间"的地图。

## 相关文件（`code/sota_integration/`）

| 文件 | 说明 |
|---|---|
| `explore/config/explore_params.yaml` | explore_lite 参数（迷宫调参） |
| `explore/launch/start_explore.launch.py` | explore_lite 启动 |
| `explore/scripts/explore_watch.py` | 覆盖/终止监控 |
| `eval/nav2_manual.launch.py` | **手工起 Nav2（绕 RewrittenYaml）** |
| `eval/explore_nav2b.sh` | 全栈 + 手工 Nav2 + explore 一键脚本 |
| `eval/explore_nav2.sh` | 初版（用 nav2_bringup，暴露 critics 问题） |
| `eval/map_sweep.sh` | 脚本化温和扫描（备用） |
