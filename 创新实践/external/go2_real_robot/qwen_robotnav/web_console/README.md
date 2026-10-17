# Go2 浏览器仿真工作台

将**当前已有的 Gazebo 窗口**转发到浏览器，旁边显示真实 ROS 运行状态、占据栅格地图、机器人位置、安全监护和探索事件。页面采用已选定的 A 方向：暖白纸感底、深棕灰文字与少量陶土橙，沿用分栏工作台。支持手机布局、画面重连、PNG 截图、全屏和事件筛选。

本版为只读观察台。X11 转发端与浏览器端均为只读，ROS 观察节点不发布运动或任务指令。服务不会启动 Gazebo、Nav2 或另一套导航实现；没有连接到相应 ROS 域时显示等待状态，不填充示例数据。

## 启动

在宿主机运行，先打开一套 Gazebo GUI。宿主机需有 Python 3.10+、Node/npm、`x11vnc`、`xwininfo` 和 `xdotool`；ROS 依赖复用现有 ROS 容器。

```bash
cd /home/wy/data1/创新实践/external/go2_real_robot/qwen_robotnav/web_console
npm ci --ignore-scripts
python3 -m pip install --user -r requirements.txt
python3 server.py --display :2 --domain 225
```

访问 **http://127.0.0.1:8765**。`Ctrl+C` 停止本控制台及其自有转发/观察进程，保留原 Gazebo 与导航进程。

默认读取 `../config/sim_runtime.yaml`，通过 `/home/wy/.enroot/ros` 启动一个只读 ROS 观察节点。指定实际运行栈的域和有效 profile：

```bash
python3 server.py --domain 226 --profile /absolute/path/runtime_profile.yaml
```

ROS 已在本机环境加载时，可以传 `--ros-wrapper /usr/bin/env`。仅看画面用 `--no-ros`，仅接遥测用 `--no-window`。默认 Xauthority 是 `~/.Xauthority`，可以通过环境变量覆盖。

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
- 不自动发送 EXPLORE/RETURN，也不接入 `/cmd_vel`。任务操作继续使用现有运行栈。
- 详细日志默认在 `/tmp/go2-web-console-<uid>/`，可用 `--log-dir` 指定。仅服务静态前端与只读状态接口，源码、profile 和环境文件不通过 HTTP 提供。

## 验证

```bash
python3 -m unittest -v test_server
node --check public/app.js
```

HTTP 边界测试覆盖路径穿越、源文件隔离、只读接口、WebSocket 同源限制、Host 校验、窗口选择和转发进程退出。ROS 运输测试在独立域 232 运行，不需要启动仿真：

```bash
/home/wy/.enroot/ros bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=232; cd /home/wy/project/external/go2_real_robot/qwen_robotnav/web_console; python3 -m unittest -v test_ros_telemetry'
```

运输测试核对真实 DDS 消息到 JSON、地图上下方向、TF 与事件，并检查没有运动/任务指令发布者。合成数据仅存在于隔离测试域，正式页面继续订阅实际配置的域。

本机实测证据在 `.runtime/web_console/`：`tests-final.log`、`ros-transport-final.log`、`browser-final.log`，以及真实桌面/手机页面和 Gazebo 截图。浏览器检查复用现有 Chrome，验证实际 VNC 像素、下载、重连、全屏、弹窗、筛选、移动端与断连处理。

## 网上视觉与技术参考

| 参考 | 采用的方向 |
| --- | --- |
| [Anthropic](https://www.anthropic.com/) | 用户选定的 A 方向：暖白底、留白和纸感层次；本项目搭配陶土色交互强调 |
| [noVNC](https://novnc.com/info.html) | 通过 WebSocket 在浏览器显示现有远程窗口 |

主题颜色集中在 `public/tokens.css`；画面背景、地图标记、连接状态和弹窗统一使用这套颜色。没有对真实 Gazebo 画面或 ROS 地图栅格做主题染色。

页面和图标独立实现；参考网站的图片没有作为本项目素材分发。noVNC 使用锁定版本的官方 npm 包，保留其 MPL-2.0 许可。
