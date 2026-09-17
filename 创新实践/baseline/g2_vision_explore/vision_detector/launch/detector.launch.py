"""起 G2 感知链路。

⚠️ **仿真下必须传 use_sim_time:=true**，否则节点用墙钟、而 /clock 是仿真钟，
TF 会报 extrapolation into the past/future —— 而且报错指向 TF，
极难自查到时钟上。

用法::

    ros2 launch vision_detector detector.launch.py
    ros2 launch vision_detector detector.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="gz-sim 下必须为 true。节点用墙钟而 /clock 是仿真钟时，TF 会报 extrapolation。",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("vision_detector"), "config", "detector.yaml"]
            ),
        ),
        Node(
            package="vision_detector",
            executable="detector_node",
            name="vision_detector",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
