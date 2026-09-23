"""我们自己的 Nav2 启动：在 Nav2 与仿真之间插入速度缩放。

## 为什么不能直接改共享的 launch

仿真工作区那份 `go2_navigation2.launch.py` 别人也在用，而且 `nav2_bringup`
内部已经用 `cmd_vel_nav` 这个名字接 controller_server 和 velocity_smoother。
在组级上重映射 `cmd_vel` 会把它的内部接线一起改坏。

## 本文件的做法

不碰 Nav2 内部接线，只在**最外层**把它的最终输出从 `/robot1/cmd_vel`
挪到 `/robot1/cmd_vel_limited`；然后由 `cmd_vel_scaler.py` 读该话题、
按实测标定的系数缩放，写回标准的 `/robot1/cmd_vel`，
交给仿真自带的 `cmd_vel_pub` 照常处理。

     controller_server → (nav2 内部) → velocity_smoother → cmd_vel_limited
                                                                  │
                                                    cmd_vel_scaler │ 缩放+限幅
                                                                  ▼
                                                            cmd_vel → cmd_vel_pub → rv → 步态

注意 `nav2_bringup` 的 velocity_smoother 已经把 `cmd_vel_smoothed` 映射成了
`cmd_vel`，所以对 `cmd_vel` 的重映射正好落在它的输出上。

用法：
    ros2 launch <本文件> map:=/path/to/map.yaml
参数：
    map          地图 yaml（必填）
    params_file  Nav2 参数（默认用我们仓库里那份 config/go2_nav2.yaml）
    use_sim_time 默认 true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

# 我们仓库里的 Nav2 参数（不是仿真工作区那份）
DEFAULT_PARAMS = (
    "/home/wy/data1/创新实践/baseline/g1_mapping_nav/config/go2_nav2.yaml"
)

ROBOT_NAMESPACE = "robot1"
# Nav2 最终速度输出的话题。仿真自带 cmd_vel_pub 订阅的是 /robot1/cmd_vel，
# 所以这里必须换成别的名字，否则 Nav2 的原始指令会直接驱动狗。
# 由 cmd_vel_scaler.py 读该话题、缩放后写回 /robot1/cmd_vel。
NAV_OUTPUT_TOPIC = "cmd_vel_scaled_out"


def generate_launch_description():
    ld = LaunchDescription()

    map_arg = DeclareLaunchArgument("map", description="地图 yaml 路径")
    params_arg = DeclareLaunchArgument(
        "params_file", default_value=DEFAULT_PARAMS, description="Nav2 参数文件"
    )
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )
    ld.add_action(map_arg)
    ld.add_action(params_arg)
    ld.add_action(sim_time_arg)

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
            "use_namespace": "true",
            "namespace": ROBOT_NAMESPACE,
        }.items(),
    )

    # 两条规则，实测出的接线（用 ros2 topic info -v 验过）：
    #
    #   规则 2 把 controller_server / behavior_server 的输出从 cmd_vel 挪到
    #   cmd_vel_limited（nav2_bringup 内部本就有 cmd_vel→cmd_vel_nav 的规则，
    #   组级规则排在前面，first-match 生效）。
    #   velocity_smoother 因此订阅 cmd_vel_limited、平滑后发 cmd_vel。
    #
    #   规则 1 再把 smoother 的输出挪走一格。它只对 velocity_smoother 有效，
    #   因为 nav2 内部只有那个节点用 cmd_vel_smoothed 这个名字。
    #
    # 最终：Nav2 → cmd_vel_scaled_out →[缩放器]→ cmd_vel → cmd_vel_pub → rv → 步态
    ld.add_action(
        GroupAction(
            actions=[
                SetRemap(src="cmd_vel_smoothed", dst=NAV_OUTPUT_TOPIC),
                SetRemap(src="cmd_vel", dst="cmd_vel_limited"),
                bringup,
            ]
        )
    )

    return ld
