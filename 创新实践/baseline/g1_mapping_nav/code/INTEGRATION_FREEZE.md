# Nav2 / TF 集成冻结包（G3 第二轮集成验证用）

> 目的：把 **Nav2 手工启动 + 全局 TF + 参数修复** 固化到**一个确定的 Git commit**，避免共享工作区被并发覆盖。
> 关联：PR #2 · 分支 `feat/go2-cross-warehouse-code`
> 日期：2026-10-14

---

## 0. 背景（为什么需要冻结）

- 运行环境 `~/go2_sim_ws/ROS2-Gazebo-GO2/` 是**多人共享工作区**；当天发现另一个 Agent（`AtoFuture`/data1 的 CodeArts）于 **10:22** 改过 `gazebo_go2_sensors.launch.py`，把 `/tf` 修复**覆盖**了。
- ⇒ **一切以本 Git commit 为准**；改完请立即提交，不要依赖工作区现状。

---

## 1. commit 内含的修复文件清单

Repo 路径前缀 = `创新实践/baseline/g1_mapping_nav/code/`
容器内工作区 = `~/go2_sim_ws/ROS2-Gazebo-GO2/`

### A. 全局 TF（删除 `/tf`、`/tf_static` 命名空间重映射）
| Repo | go2_sim_ws |
|---|---|
| `code/src/gazebo_sim/launch/gazebo_go2_sensors.launch.py` | `src/gazebo_sim/launch/gazebo_go2_sensors.launch.py` |
| `code/src/gazebo_sim/launch/launch_sim.launch.py` | `src/gazebo_sim/launch/launch_sim.launch.py` |
| `code/src/gazebo_sim/launch/gazebo_go2_self.launch.py` | `src/gazebo_sim/launch/gazebo_go2_self.launch.py` |
| `code/src/gazebo_sim/launch/rviz_launch.py` | `src/gazebo_sim/launch/rviz_launch.py` |
| `code/src/navigation2/launch/go2_navigation2.launch.py` | `src/navigation2/launch/go2_navigation2.launch.py` |

要点：删掉以下三类（Humble 的 tf2 只读全局 `/tf`）：
- `remappings=[... ("/tf","tf"), ("/tf_static","tf_static") ...]`
- 组级 `SetRemap(src="/tf", dst="tf")` / `("/tf_static","tf_static")`
- RViz 节点的 `('/tf','tf')`

### B. `odom→base_link` 单一发布者
| Repo | go2_sim_ws | 改动 |
|---|---|---|
| `code/src/gazebo_sim/config/ekf.yaml` | `src/gazebo_sim/config/ekf.yaml` | `publish_tf: true` |
| `code/src/gazebo_sim/launch/gazebo_go2_sensors.launch.py` | 同左 | `enable_odom_tf` 默认 **False** |

### C. Nav2
| Repo | go2_sim_ws | 改动 |
|---|---|---|
| `code/src/navigation2/param/go2_nav2.yaml` | `src/navigation2/param/go2_nav2.yaml` | `bt_navigator odom_topic: /robot1/odometry/filtered` |
| `code/sota_integration/eval/nav2_manual.launch.py` | （新增，见下） | 手工起 Nav2 节点 |

---

## 2. Nav2 必须"手工启动"（绕过 `nav2_bringup`）

**问题**：`nav2_bringup` 带 `namespace` 时用 `RewrittenYaml` 重写参数文件，**把 `FollowPath.critics` 弄丢**：
```
[controller_server] Couldn't load critics! ... No critics defined for FollowPath
[lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.
```
→ 无 `/robot1/navigate_to_pose`。

**做法（两步）**：

1. 生成命名空间根参数文件（把 `go2_nav2.yaml` 整体包在 `robot1:` 下）：
```bash
python3 - <<'PY'
src = open('/home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/navigation2/param/go2_nav2.yaml').read().splitlines()
out = ['robot1:'] + [('  '+l if l.strip() else l) for l in src]
open('/tmp/go2_nav2_ns.yaml','w').write('\n'.join(out)+'\n')
PY
```

2. 手工起节点（`nav2_manual.launch.py`，已随本 commit 提供）：
```bash
ros2 launch /tmp/nav2_manual.launch.py
```
它用 `namespace:=robot1` + `--params-file /tmp/go2_nav2_ns.yaml` 起
`controller_server / planner_server / behavior_server / bt_navigator / smoother_server / velocity_smoother` + `lifecycle_manager(autostart)`。

---

## 3. 最终 Nav2 启动命令（在已起好 gz+桥+机器人+slam 的前提下）

```bash
# 前提：/robot1/map、/robot1/velodyne、/robot1/odometry/filtered、全局 /tf 已就绪
python3 - <<'PY'
src = open('/home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/navigation2/param/go2_nav2.yaml').read().splitlines()
out = ['robot1:'] + [('  '+l if l.strip() else l) for l in src]
open('/tmp/go2_nav2_ns.yaml','w').write('\n'.join(out)+'\n')
PY
ros2 launch /tmp/nav2_manual.launch.py
```

---

## 4. G3 验证清单

```bash
ros2 topic list | grep -E '^/tf$|^/tf_static$'          # TF 必须是全局的
ros2 run tf2_ros tf2_echo map base_link                  # map->odom->base_link 稳定
ros2 run tf2_ros tf2_echo odom base_link
ros2 lifecycle get /robot1/controller_server              # active
ros2 lifecycle get /robot1/bt_navigator                   # active
ros2 action list | grep navigate_to_pose                  # /robot1/navigate_to_pose 存在
ros2 topic info /robot1/velodyne -v                       # costmap 两个均订阅
ros2 param get /robot1/controller_server use_sim_time
# 最后：发 RETURN，观察 Nav2 是否规划并行走
```

---

## 5. ⚠️ 并发编辑警告

共享工作区（`~/go2_sim_ws`）同时有多个 Agent 在改（CodeArts@data1、Codex@data1/2/3、Copilot、Windows 端）。
**约定：同一时间只让一个人/一个 Agent 改该工作区；改完立刻 commit。** 本冻结 commit 即唯一基准。
