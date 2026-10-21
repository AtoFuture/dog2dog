import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap, ComposableNodeContainer
from launch.event_handlers import OnProcessExit
import xacro, yaml


def generate_launch_description():
    ld = LaunchDescription()

    # ---------- 基础配置 ----------
    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    robots_file_path = os.path.join(pkg_path, 'config', 'robots.yaml')

    # 多机器人配置 
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    robots = yaml_data['robots']
    
    # ---------- 声明启动参数 ----------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='仿真时间是否使用'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_use_sim_time)

    # ---------- 多机器人全局话题重映射 ----------
    remappings=[
            # NOTE(2026-10-07): /tf 与 /tf_static 不再做命名空间重映射。
            # TF 必须发布到全局 /tf：tf2_echo / view_frames / Nav2(tf2_ros) 都监听全局 /tf。
            # 旧的 ("/tf","tf") 会把整个 TF 树关进 /robot1/tf，导致 Nav2 与所有诊断工具都看不到 TF。
            # 多机器人场景应使用 frame 前缀，而不是给 /tf 加命名空间。
            ("/scan", "scan"),
            ("/odom", "odometry/filtered")
        ]
    
    # ---------- Gazebo-ROS时间同步配置 ----------
    bridge_params = os.path.join(pkg_path,'config','gz_bridge.yaml')
    ros_gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ld.add_action(ros_gz_bridge_clock)   

    # ---------- 多机器人循环启动 ----------
    last_action = None

    for i, robot in enumerate(robots):
        namespace = robot['name']
        robot_name = robot['name']
        xacro_file = os.path.join(os.path.join(get_package_share_directory('go2_description')), 'xacro', 'robot_VLP_D435i.xacro') ## CHANGE ME!!!!
        robot_desc = xacro.process_file(xacro_file, mappings={'robot_name': robot_name}).toxml()
        params_robot_state_publisher = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[params_robot_state_publisher],
            remappings=remappings
        )

        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-topic', f'/{namespace}/robot_description',
                '-name', f'{namespace}_my_bot',
                '-allow_renaming', 'true',
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', robot['z_pose'],
                # '-Y', robot['Y_pose']
            ],
            output='screen'
        )

        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                f'/{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'/{namespace}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                f'/{namespace}/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                f'/{namespace}/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                f'/{namespace}/velodyne/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                f'/{namespace}/velodyne_imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'/{namespace}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'/{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                f'/{namespace}/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                f'/{namespace}/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/{namespace}/d435i_imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                # f'/{namespace}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ]
        )
        start_gazebo_ros_image_bridge_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            namespace=namespace,
            arguments=['color/image_raw', 
                       ],
            output='screen',
        )
        gz_image_bridge_node = Node(
            package="ros_gz_image",
            executable="image_bridge",
            namespace=namespace,
            arguments=[
                f'/{namespace}/rgbd_d435/depth_image',
                f'/{namespace}/rgbd_d435/image',
            ],
            output="screen",
            parameters=[
                {'use_sim_time': True,
                'camera.image.compressed.jpeg_quality': 75},
            ],
        )

        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_state_broadcaster',
            arguments=['joint_state_broadcaster',
                       '--controller-manager-timeout', '120'],
            output='screen',
            remappings=remappings
        )

        joint_group_controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_group_controller',
            arguments=['joint_group_controller',
                       '--controller-manager-timeout', '120'],
            output='screen',
            remappings=remappings
        )

        controller = Node(
            package='quadropted_controller',
            executable='robot_controller_gazebo.py',
            name='quadruped_controller',
            namespace=namespace,
            output='screen',
            remappings=remappings
        )

        odom = Node(
            package='quadropted_controller',
            executable='QuadrupedOdometryNode.py',
            name='odom',
            namespace=namespace,
            output='screen',
            parameters=[{
                "verbose": False,
                'publish_rate': 50,
                'open_loop': False,
                'has_imu_heading': True,
                'is_gazebo': True,
                'imu_topic': f'/{namespace}/imu',
                'base_frame_id': "base_link",
                'odom_frame_id': "odom",
                'clock_topic': f'/clock',
                # ── 2026-09-25 双发布修复（默认不动，需显式开关）──────────────
                # 同一个 launch 里还起了 ekf_filter_node（ekf.yaml 的 publish_tf: true），
                # 于是 odom→base_link 有**两个发布者**：本节点和 EKF。两者姿态不一致
                # （G3 只读诊断实测 yaw 相差约 142.7°），TF 在两个姿态间跳变，
                # AMCL / costmap 每次拿到哪一个不确定。
                #
                # 设 G1_SINGLE_ODOM_TF=1 时关掉本节点的 TF，让 EKF 独占
                # odom→base_link：EKF 融了 IMU，而出问题的正是足端里程计的偏航。
                # 本节点仍照常发 /robot1/odom 话题（EKF 的 odom0 还要吃它），只是不发 TF。
                #
                # 和 GZ_HEADLESS 一个套路：不设 = 与上游一字不差，不影响别人。
                # NOTE(2026-10-07): TF 唯一发布者 = EKF（ekf.yaml publish_tf: true）。
                # 本节点默认不发 TF，彻底消除"双发布"与"无人发布"两种非法状态。
                # 若要改回由本节点发布：设 G1_ODOM_NODE_TF=1，并把 ekf.yaml 的 publish_tf 改回 false。
                'enable_odom_tf': os.environ.get('G1_ODOM_NODE_TF', '0') == '1',
            }],
            remappings=remappings
        )

        rviz_launch_file = os.path.join(pkg_path, 'launch', 'rviz_launch.py')
        rviz_config_file = os.path.join(pkg_path, 'rviz', 'go2_sensors.rviz')

        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": 'true',
                "rviz_config": rviz_config_file,
            }.items(),
            condition=IfCondition(enable_rviz)
        )

        cmd_vel_pub = Node(
            package='quadropted_controller',
            executable='cmd_vel_pub.py',
            namespace=namespace,
            name='cmd_vel_pub',
            output='screen',
            remappings=remappings
        )

        fake_bms = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', f'/{namespace}/battery_state', 'sensor_msgs/msg/BatteryState',
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 24.0, percentage: 0.8, capacity: 10.0}",
                '-r', '1'
            ],
            output='log'
        )
        robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
        # Start robot localization using an Extended Kalman filter
        start_robot_localization_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[robot_localization_file_path, 
            {'use_sim_time': use_sim_time}],
            remappings=remappings)

        robot_control = GroupAction([
            # NOTE(2026-10-07): 移除组级 /tf、/tf_static 重映射。
            # 这两行会把组内所有子节点（含 EKF）的 TF 重新关回 /robot1/tf，
            # 是 Nav2 / tf2_echo 看不到 TF 的真正原因（比 remappings 列表更隐蔽）。
            # EKF 现为 odom->base_link 的唯一发布者，必须发到全局 /tf。
            joint_state_broadcaster,
            joint_group_controller,
            controller,
            cmd_vel_pub,
            odom,
            start_robot_localization_cmd,
            fake_bms,
        ])

        # 当前机器人动作组
        robot_group = GroupAction([
            node_robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            start_gazebo_ros_image_bridge_cmd,
            robot_control,
            gz_image_bridge_node,
            rviz,
        ])

        if last_action is None:
            ld.add_action(robot_group)
        else:
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[robot_group]
                )
            )
            ld.add_action(spawn_robot_event)

        last_action = joint_group_controller

    return ld
