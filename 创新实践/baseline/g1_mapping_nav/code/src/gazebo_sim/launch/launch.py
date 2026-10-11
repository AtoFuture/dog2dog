import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction  
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import SetParameter

def create_gazebo_action(context: LaunchContext, package_name: str):
    world_name = LaunchConfiguration('world').perform(context)
    pkg_path = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_path, 'world', world_name)
    
    # 无头开关：设了 GZ_HEADLESS=1 才加 -s（只起服务端，不起 GUI）。
    # 默认不设 → gz_args 与上游一字不差，不影响别人的用法。
    # 需要它的原因：本文件原本写死 -r -v4，没有无头入口，而共享机器上不该常驻 GUI。
    headless_prefix = (
        '-s ' if os.environ.get('GZ_HEADLESS', '') not in ('', '0') else ''
    )

    gazebo_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [headless_prefix, '-r -v4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    return [gazebo_action]

def choose_launch_file(context, *args, **kwargs):
    use_sensors = LaunchConfiguration('sensors').perform(context)
    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    
    if use_sensors.lower() == 'true':
        launch_file = 'gazebo_go2_sensors.launch.py'
    else:
        launch_file = 'gazebo_go2_self.launch.py'

    # 无头时同样关掉 rviz2：子 launch 本来就有 enable_rviz 开关，只是顶层没转发。
    # 不转发的话 rviz2 在无 DISPLAY 下 abort（exit -6），会带崩整个 launch。
    headless = os.environ.get('GZ_HEADLESS', '') not in ('', '0')
    include_args = {'enable_rviz': 'false'} if headless else {}

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', launch_file)
            ),
            launch_arguments=include_args.items()
        )
    ]

def generate_launch_description():
    ld = LaunchDescription()
    package_name = 'gazebo_sim'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='是否使用仿真时间'
    ))
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    ld.add_action(DeclareLaunchArgument(
        'sensors',
        default_value='false',
        description='是否启动传感器版launch文件（默认启动无外置传感器版）'
    ))

    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='rmuc_2025_world.sdf',
        description='指定要加载的Gazebo世界文件（需放在gazebo_sim/world目录下）'
    ))

    gazebo_action = OpaqueFunction(
        function=create_gazebo_action,
        args=[package_name]  
    )
    ld.add_action(gazebo_action)


    pause = ExecuteProcess(
        cmd=['sleep', '6'],
        output='screen'
    )
    ld.add_action(pause)

    go2_sensors_launch = OpaqueFunction(function=choose_launch_file)

    launch_after_pause = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pause,
            on_exit=[go2_sensors_launch]
        )
    )
    ld.add_action(launch_after_pause)

    return ld
