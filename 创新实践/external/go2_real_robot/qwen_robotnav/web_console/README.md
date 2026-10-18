# Go2 浏览器仿真工作台

将**当前已有的 Gazebo 窗口**转发到浏览器，旁边显示真实 ROS 运行状态、占据栅格地图、机器人位置、安全监护和探索事件。页面采用白底黑字的黑白界面，使用中性细边框区分面板，选中项通过黑色描边和字重表示，沿用分栏工作台。支持手机布局、画面重连、PNG 截图、全屏和事件筛选。

浏览器默认可以操作现有 Gazebo 窗口：拖动平移、滚轮缩放、中键旋转和键盘输入都会传入原窗口；工具栏可随时切换为「仅查看」。服务端也可以用 `--view-only` 强制关闭 GUI 输入。ROS 默认只订阅；显式启用 `--enable-tasks` 后，任务按钮通过现有 G3 命令话题执行，不发布速度指令，也不启动另一套导航实现。一键演示入口会启用任务控制。

服务不会启动 Gazebo、Nav2 或另一套导航实现；没有连接到相应 ROS 域时显示等待状态，不填充示例数据。

## 启动

在宿主机运行，先打开一套 Gazebo GUI。宿主机需有 Python 3.10+、Node/npm、`x11vnc`、`xwininfo` 和 `xdotool`；ROS 依赖复用现有 ROS 容器。

```bash
cd /home/wy/data1/创新实践/external/go2_real_robot/qwen_robotnav/web_console
npm ci --ignore-scripts
python3 -m pip install --user -r requirements.txt
python3 server.py --display :2 --domain 225 --partition go2obj \
  --world world_demo --robot-model robot1_my_bot
```

访问 **http://127.0.0.1:8765**。`Ctrl+C` 停止本控制台及其自有转发/观察进程，保留原 Gazebo 与导航进程。

默认读取 `../config/sim_runtime.yaml`，通过 `/home/wy/.enroot/ros` 启动一个 ROS 观察节点（默认只读）。指定实际运行栈的域和有效 profile：

```bash
python3 server.py --domain 226 --profile /absolute/path/runtime_profile.yaml
```

ROS 已在本机环境加载时，可以传 `--ros-wrapper /usr/bin/env`。仅看画面用 `--no-ros`，仅接遥测用 `--no-window`，强制只读画面用 `--view-only`。

### 与仿真实例对齐的参数

| 参数 | 默认 | 用途 |
| --- | --- | --- |
| `--display` | `:2` | Gazebo GUI 所在 X display |
| `--window-id` | 自动 | 有多个 Gazebo 窗口时显式指定 |
| `--domain` | `225` | ROS 观察节点使用的 Domain |
| `--partition` | `$IGN_PARTITION` 或 `go2obj` | 调用 Gazebo 服务时的 transport partition |
| `--world` | `world_demo` | `/world/<world>/scene/graph` 里的世界名 |
| `--robot-model` | `robot1_my_bot` | 「找机器狗」要定位的 Gazebo model 名 |
| `--view-only` | 关 | 禁用浏览器到 Gazebo 的鼠标/键盘输入 |
| `--enable-tasks` | 关 | 在隔离 Gazebo 域启用现有 G3 任务控制，与 GUI 交互独立 |
| `--ros-env` | 无 | 给容器内 ROS 观察节点追加环境变量，可重复 |

`--world` 与 `--robot-model` 必须和当前 SDF/生成启动副本一致，否则「找机器狗」会返回 409。

### 与 CycloneDDS 运行栈对接

`qwen_robotnav` 的正式运行栈使用 CycloneDDS，并绑定 `lo` 接口。网页观察节点必须使用同一 RMW 与同一份 Cyclone 配置，否则只能看到节点发现、收不到话题数据：

```bash
python3 server.py --display :2 --domain 227 --partition webdemo227 \
  --world world_demo --robot-model robot1_my_bot \
  --ros-env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --ros-env CYCLONEDDS_URI=file:///home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
```

观察节点在 `~/.enroot/ros` 容器内运行，`--ros-env` 通过容器内的 `env` 传入。若运行栈使用默认 FastDDS，则不需要这两个参数。

## 网页任务控制

已运行统一导航栈时，在同一隔离域启动：

```bash
python3 server.py --domain 227 --partition gui_demo_227 --enable-tasks \
  --ros-env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --ros-env CYCLONEDDS_URI=file:///home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
```

按钮包括探索、暂停、继续、返航、待机、急停和解除急停。按钮按实时任务状态启用；G3 仍负责健康检查、导航释放和电量条件，网页不绕过这些判断。

- HTTP `202` 只表示请求已写入 ROS 桥；页面先显示“已发送，等待 G3 回执”。
- 请求带唯一 `request_id`，G3 在配置的命令话题后加 `/result` 发布接受/拒绝回执。页面仅匹配相同 ID 和命令，拒绝原因保留展示。
- “G3 已接受”表示状态机接受操作，导航执行成功/失败另见任务事件。返航事件也会显示在事件区。
- 12 秒没有回执时显示“未确认”，不会自动重发。任务状态过期后禁用普通操作；ROS 桥存活时仍允许急停，且不受其他请求等待回执的限制。
- `--enable-tasks` 仅支持隔离 Gazebo profile；不支持域 `0/42/43` 或 `--no-ros`。未开启时不创建命令发布者。
- `--view-only` 只控制 Gazebo 鼠标/键盘；要关闭任务控制，请省略 `--enable-tasks`。

命令接口为 `POST /api/mission/<COMMAND>`，只接受固定命令名、同源回环请求和空请求体；命令队列与操作记录均有上限。原有纯文本 G3 命令保持兼容。

## 远程访问

服务仅监听回环地址。电脑与仿真服务器不在同一台机器时，在自己的电脑建立一个 SSH 隧道：

```bash
ssh -N -L 8765:127.0.0.1:8765 wy@服务器地址
```

随后在自己电脑浏览器打开 http://127.0.0.1:8765。页面、ROS 数据与窗口 WebSocket 都经这一个端口，不需要额外转发 VNC 端口。

## 窗口与数据连接

- 自动选择当前显示器中唯一标题为 `Gazebo` 的窗口；有多个时要求 `--window-id 0x...`，不回退到整个桌面。窗口最小化时提示恢复显示。窗口超出屏幕时仅缩小和移动该窗口以适配 X11 捕获，不改变仿真状态。
- 默认端口为网页 `8765`、私有 WebSocket `8766`、私有 VNC `5909`，均只监听回环；CLI 可以指定三个不同端口。网页端口先绑定，防止重复启动相同控制台。
- 画面是原 Gazebo GUI 的镜像，场景、镜头位置、暂停状态与原窗口一致。地图来自 ROS 原始 `OccupancyGrid`，不使用验收真值；位置来自真实 map→base TF，不用里程计坐标冒充地图坐标。
- 任务、监护、速度与 TF 的数据过期后不显示为在线值。地图超过 20 秒未更新时不显示为实时地图。窗口转发进程退出、HTTP 中断和 ROS 没有数据分别显示对应状态。
- 不自动开始探索，也不发布 `/cmd_vel`。启用任务控制后，网页按钮只发布现有 G3 命令，运动仍由统一运行栈执行。
- 详细日志默认在 `/tmp/go2-web-console-<uid>/`，可用 `--log-dir` 指定。仅服务前端、状态和固定的相机/任务操作接口，源码、profile 和环境文件不通过 HTTP 提供。

### ROS 遥测的前置条件

页面要显示地图、位姿、电量、任务状态、安全监护和探索事件，同一 Domain 上必须有对应的发布者：

| 面板 | 话题 / 来源 |
| --- | --- |
| 地图 | `/robot1/map`（SLAM 或 map_server，`transient_local`） |
| 位姿 | TF `map → base_link`（经 `/robot1/tf` 重映射） |
| 电量 | `/runtime/battery_state` |
| 任务状态 | `/mission_state` |
| 安全监护 | `/runtime/velocity_guard_status` |
| 探索事件 | `/exploration/events` |
| 线速度 | `/robot1/odometry/filtered` |

只启动传感器/机器人 launch（没有 SLAM、G3、guard、battery 节点）时，画面正常但上述面板会保持「等待数据」，这是预期行为，不是页面故障。完整运行栈的启动见 `../../docs/navigation-runtime.md`。

### 找机器狗

工具栏的「找机器狗」调用 `POST /api/view/robot`：

1. 通过 Gazebo 服务 `/world/<world>/scene/graph` 检查 `--robot-model` 是否在当前场景；
2. 调用 `/gui/move_to` 让 Gazebo 镜头移到该 model；
3. 返回 200；模型不存在返回 409，GUI 未确认返回 502。

该接口只接受 loopback 同源请求、不接受请求体；`--view-only` 下返回 403，窗口转发不可用时返回 503。它只移动镜头，不改变仿真状态。

## 验证

```bash
python3 -m unittest -v test_server test_task_control
node --check public/app.js
```

HTTP 边界测试覆盖路径穿越、源文件隔离、只读接口、WebSocket 同源限制、Host 校验、窗口选择、转发进程退出、`/api/view/robot` 的同源/只读/请求体/模型检查和容器内环境注入。ROS 运输测试在独立域 232 运行，不需要启动仿真：

```bash
/home/wy/.enroot/ros bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=232; cd /home/wy/project/external/go2_real_robot/qwen_robotnav/web_console; python3 -m unittest -v test_ros_telemetry'
```

运输测试核对真实 DDS 消息到 JSON、地图上下方向、TF 与事件，并检查没有运动/任务指令发布者。合成数据仅存在于隔离测试域，正式页面继续订阅实际配置的域。

浏览器端到端检查（控制台需已启动；`EXPECT_LIVE=1` 时要求同一域上有在线运行栈）：

```bash
EXPECT_LIVE=1 OUT_DIR=/tmp/go2-browser npm run test:browser
```

脚本自行启动无头 Chrome，检查真实 VNC 像素、遥测面板、交互开关、找机器狗、重连、移动端无横向滚动和页面错误，并写出 `desktop.png`、`mobile.png` 和 `result.json`。可用 `CONSOLE_URL`、`CHROME`、`CDP_PORT`、`EXPECT_LIVE`、`EXPECT_FOCUS`、`OUT_DIR` 覆盖默认值。

本机历史证据在 `.runtime/web_console/`：`tests-final.log`、`ros-transport-final.log`、`browser-final.log`，以及真实桌面/手机页面和 Gazebo 截图。交互版验证记录在 `.runtime/web_console_demo/console/`：`browser-run.log`（9/9 通过）、`camera-interaction-check.log`（交互拖动相机位移 2.953，仅查看位移 0.000）、`desktop.png`、`mobile.png`。

## 网上视觉与技术参考

| 参考 | 采用的方向 |
| --- | --- |
| [Anthropic](https://www.anthropic.com/) | 保留留白与排版层次；配色按用户最新要求采用白底黑字 |
| [noVNC](https://novnc.com/info.html) | 通过 WebSocket 在浏览器显示现有远程窗口 |

主题颜色集中在 `public/tokens.css`；画面背景、地图标记、连接状态和弹窗统一使用这套颜色。没有对真实 Gazebo 画面或 ROS 地图栅格做主题染色。

页面和图标独立实现；参考网站的图片没有作为本项目素材分发。noVNC 使用锁定版本的官方 npm 包，保留其 MPL-2.0 许可。
