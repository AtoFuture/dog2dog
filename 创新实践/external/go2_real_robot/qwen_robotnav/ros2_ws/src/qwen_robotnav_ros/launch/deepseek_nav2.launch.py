"""DeepSeek high-level goals → existing Nav2 stack (no direct velocity controller)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = {
        'camera_topic': '/robot1/color/image_raw',
        'instruction': 'Explore open space safely and avoid obstacles.',
        'vllm_base_url': 'https://api.deepseek.com',
        'vllm_model': 'deepseek-flash',
        'use_sim_time': 'true',
        'nav_action': '/robot1/navigate_to_pose',
        'tf_topic': '/robot1/tf',
        'tf_static_topic': '/robot1/tf_static',
    }
    ld = LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in arguments.items()])
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    ld.add_action(Node(
        package='qwen_robotnav_ros', executable='nav_node', output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'instruction': LaunchConfiguration('instruction'),
            'task_mode': 'VLN', 'history_length': 1, 'inference_period': 3.0,
            'vllm_base_url': LaunchConfiguration('vllm_base_url'),
            'vllm_model': LaunchConfiguration('vllm_model'),
            'use_sim_time': clock,
        }],
    ))
    ld.add_action(Node(
        package='qwen_robotnav_ros', executable='nav2_waypoint_bridge', output='screen',
        parameters=[{'nav_action': LaunchConfiguration('nav_action'), 'use_sim_time': clock}],
        remappings=[('/tf', LaunchConfiguration('tf_topic')),
                    ('/tf_static', LaunchConfiguration('tf_static_topic'))],
    ))
    return ld
