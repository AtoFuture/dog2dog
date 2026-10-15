from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/camera/color/image_raw",
            description="Input camera image topic",
        ),
        DeclareLaunchArgument(
            "instruction",
            default_value="Move forward safely.",
            description="Initial navigation instruction",
        ),
        DeclareLaunchArgument(
            "task_mode",
            default_value="VLN",
            description="Task mode: VLN, PointNav, ObjNav, Tracking",
        ),
        DeclareLaunchArgument(
            "vllm_base_url",
            default_value="https://api.deepseek.com",
            description="DeepSeek Vision API endpoint",
        ),
        DeclareLaunchArgument(
            "vllm_model",
            default_value="deepseek-flash",
            description="DeepSeek model name (deepseek-flash = DeepSeek-V4.1-Flash)",
        ),
        DeclareLaunchArgument(
            "start_unitree_bridge",
            default_value="true",
            description="Start the standalone Unitree SDK2 motion/camera bridge",
        ),
        DeclareLaunchArgument(
            "unitree_iface",
            default_value="eth0",
            description="Network interface connected to the Go2",
        ),
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([
                    FindPackagePrefix("go2_unitree_bridge"),
                    "lib",
                    "go2_unitree_bridge",
                    "go2_unitree_bridge_node",
                ]),
                LaunchConfiguration("unitree_iface"),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_unitree_bridge")),
        ),
        Node(
            package="qwen_robotnav_ros",
            executable="go2_ros_bridge",
            name="go2_ros_bridge",
            output="screen",
            parameters=[{
                "camera_topic": LaunchConfiguration("camera_topic"),
                "cmd_vel_topic": "/qwen_robotnav/cmd_vel",
                "cmd_timeout_sec": 0.5,
            }],
        ),
        Node(
            package="qwen_robotnav_ros",
            executable="nav_node",
            name="qwen_robotnav_nav_node",
            output="screen",
            parameters=[{
                "camera_topic": LaunchConfiguration("camera_topic"),
                "instruction": LaunchConfiguration("instruction"),
                "task_mode": LaunchConfiguration("task_mode"),
                "vllm_base_url": LaunchConfiguration("vllm_base_url"),
                "vllm_model": LaunchConfiguration("vllm_model"),
                "history_length": 4,
                "inference_period": 1.0,
            }],
        ),
        Node(
            package="qwen_robotnav_ros",
            executable="go2_controller",
            name="qwen_robotnav_go2_controller",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/qwen_robotnav/cmd_vel",
                "waypoint_topic": "/qwen_robotnav/waypoints",
                "control_rate": 10.0,
                "max_linear": 0.5,
                "max_angular": 0.8,
                "waypoint_dt": 0.5,
                "waypoint_timeout_sec": 2.5,
            }],
        ),
    ])
