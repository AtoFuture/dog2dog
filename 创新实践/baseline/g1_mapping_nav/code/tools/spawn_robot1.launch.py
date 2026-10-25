import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    namespace = 'robot1'
    xacro_file = os.path.join(
        get_package_share_directory('go2_description'), 'xacro', 'robot_VLP_D435i.xacro')
    robot_desc = xacro.process_file(xacro_file, mappings={'robot_name': namespace}).toxml()
    remappings = [
        # NOTE(2026-10-07): /tf、/tf_static 保持全局。tf2 的广播者与监听者都用绝对 /tf，
        # 若把 rsp 的静态 TF 重映射进命名空间，slam_toolbox 会找不到 base_link->lidar，丢弃全部 scan。
        ("/scan", "scan"),
        ("/odom", "odometry/filtered"),
    ]
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        remappings=remappings,
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=[
            '-topic', '/robot1/robot_description',
            '-name', 'robot1_my_bot',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.45',
        ],
        output='screen',
    )
    return LaunchDescription([rsp, spawn])
