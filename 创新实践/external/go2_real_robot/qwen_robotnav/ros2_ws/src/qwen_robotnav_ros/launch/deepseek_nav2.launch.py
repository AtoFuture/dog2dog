"""DeepSeek high-level goals → existing Nav2 stack (no direct velocity controller)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _experimental_launch_description():
    arguments = {
        'camera_topic': '/robot1/color/image_raw',
        'instruction': 'Explore open space safely and avoid obstacles.',
        'vllm_base_url': 'https://api.deepseek.com',
        'vllm_model': 'deepseek-flash',
        'use_sim_time': 'true',
        'nav_action': '/robot1/navigate_to_pose',
        'require_mission_state': 'true',
        'mission_state_topic': '/mission_state',
        'instruction_topic': '/qwen_robotnav/instruction',
        'waypoint_topic': '/qwen_robotnav/waypoints',
        'debug_image_topic': '/qwen_robotnav/debug_image',
        'observation_frame': 'base_link',
        'global_frame': 'map',
        'handoff_request_topic': '/navigation/handoff_request',
        'handoff_ack_topic': '/navigation/handoff_ack',
        'event_topic': '/deepseek/nav2_events',
        'goal_topic': '/deepseek/nav2_goal',
        'tf_topic': '/robot1/tf',
        'tf_static_topic': '/robot1/tf_static',
    }
    ld = LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in arguments.items()])
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    ld.add_action(Node(
        package='qwen_robotnav_ros', executable='nav_node', output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'instruction_topic': LaunchConfiguration('instruction_topic'),
            'mission_state_topic': LaunchConfiguration('mission_state_topic'),
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            'debug_image_topic': LaunchConfiguration('debug_image_topic'),
            'observation_frame': LaunchConfiguration('observation_frame'),
            'instruction': LaunchConfiguration('instruction'),
            'task_mode': 'VLN', 'history_length': 1, 'inference_period': 3.0,
            'vllm_base_url': LaunchConfiguration('vllm_base_url'),
            'vllm_model': LaunchConfiguration('vllm_model'),
            'use_sim_time': clock,
            'require_mission_state': ParameterValue(LaunchConfiguration('require_mission_state'), value_type=bool),
        }],
    ))
    ld.add_action(Node(
        package='qwen_robotnav_ros', executable='nav2_waypoint_bridge', output='screen',
        parameters=[{
            'nav_action': LaunchConfiguration('nav_action'),
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            'mission_state_topic': LaunchConfiguration('mission_state_topic'),
            'global_frame': LaunchConfiguration('global_frame'),
            'handoff_request_topic': LaunchConfiguration('handoff_request_topic'),
            'handoff_ack_topic': LaunchConfiguration('handoff_ack_topic'),
            'event_topic': LaunchConfiguration('event_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'use_sim_time': clock,
            'require_mission_state': ParameterValue(
                LaunchConfiguration('require_mission_state'), value_type=bool
            ),
        }],
        remappings=[('/tf', LaunchConfiguration('tf_topic')),
                    ('/tf_static', LaunchConfiguration('tf_static_topic'))],
    ))
    return ld


from qwen_robotnav_ros.experimental_gate import assert_no_runtime as _assert_experimental_exclusive


def _experimental_gate(context):
    if LaunchConfiguration('enable_experimental_waypoints').perform(context).lower()!='true':
        raise RuntimeError('Image waypoint navigation is archived experimental functionality. Use run_navigation_runtime.py for the supported G2 candidate pipeline; explicit enable_experimental_waypoints:=true is required here')
    _assert_experimental_exclusive()
    return list(_experimental_launch_description().entities)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_experimental_waypoints',default_value='false'),
        OpaqueFunction(function=_experimental_gate),
    ])
