"""我们自己的 Nav2 启动：在 Nav2 与仿真之间插入速度缩放。

## slam 开关

`slam:=true` 时：
- 只包含 `navigation_launch.py`（controller/planner/bt/smoother/velocity_smoother），
  **不启动 AMCL + map_server**；否则它们会和 slam_toolbox 争 `/map` 与 `map→odom`
- 单独启动 slam_toolbox 节点，用 Node action 显式设置 namespace 和参数
- 单独启动 map_saver + lifecycle_manager_slam
- map→odom TF 由 slam_toolbox 发布

`slam:=false` 时保持原来的 `bringup_launch.py`（AMCL + 静态地图）。

用法：
    ros2 launch <本文件> map:=/path/to/map.yaml
    ros2 launch <本文件> slam:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

DEFAULT_PARAMS = (
    "/home/wy/data1/创新实践/baseline/g1_mapping_nav/config/go2_nav2.yaml"
)
DEFAULT_MAP = (
    "/home/wy/go2_sim_ws/ROS2-Gazebo-GO2/src/navigation2/maps/warehouse_map.yaml"
)

ROBOT_NAMESPACE = "robot1"
NAV_OUTPUT_TOPIC = "cmd_vel_scaled_out"

SLAM_PARAMS = {
    "use_sim_time": True,
    "solver_plugin": "solver_plugins::CeresSolver",
    "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY",
    "ceres_preconditioner": "SCHUR_JACOBI",
    "ceres_trust_strategy": "LEVENBERG_MARQUARDT",
    "ceres_dogleg_type": "TRADITIONAL_DOGLEG",
    "ceres_loss_function": "None",
    "odom_frame": "odom",
    "map_frame": "map",
    "base_frame": "base_link",
    # 仿真桥发布的是 /robot1/velodyne；绝不能写 /scan（没有发布者）。
    # 这些值对齐 .runtime/web_console_demo/slam.yaml 的已验证运行栈配置：
    # 单线雷达 ±15° 会打到地面，限幅 4 m 并抬高 min_laser_range 可避开地面环。
    "scan_topic": "/robot1/velodyne",
    "use_map_saver": True,
    "mode": "mapping",
    "debug_logging": False,
    "throttle_scans": 1,
    "transform_publish_period": 0.02,
    # 与 map_growth.py 的 MOVE_MIN_M=0.5 配套：5 s 更新一次时，
    # 0.13 m/s 的慢走每个间隔约 0.65 m，才可能满足「增长发生在运动时段」。
    "map_update_interval": 5.0,
    "resolution": 0.05,
    "min_laser_range": 0.3,
    "max_laser_range": 4.0,
    "minimum_time_interval": 0.2,
    "transform_timeout": 0.2,
    "tf_buffer_duration": 30.0,
    "stack_size_to_use": 40000000,
    "enable_interactive_mode": False,
    "use_scan_matching": True,
    "use_scan_barycenter": True,
    "minimum_travel_distance": 0.1,
    "minimum_travel_heading": 0.1,
    "scan_buffer_size": 10,
    "scan_buffer_maximum_scan_distance": 10.0,
    "link_match_minimum_response_fine": 0.1,
    "link_scan_maximum_distance": 1.5,
    "loop_search_maximum_distance": 3.0,
    "do_loop_closing": True,
    "loop_match_minimum_chain_size": 10,
    "loop_match_maximum_variance_coarse": 3.0,
    "loop_match_minimum_response_coarse": 0.35,
    "loop_match_minimum_response_fine": 0.45,
    "correlation_search_space_dimension": 0.5,
    "correlation_search_space_resolution": 0.01,
    "correlation_search_space_smear_deviation": 0.1,
    "loop_search_space_dimension": 8.0,
    "loop_search_space_resolution": 0.05,
    "loop_search_space_smear_deviation": 0.03,
    "distance_variance_penalty": 0.5,
    "angle_variance_penalty": 1.0,
    "fine_search_angle_offset": 0.00349,
    "coarse_search_angle_offset": 0.349,
    "coarse_angle_resolution": 0.0349,
    "minimum_angle_penalty": 0.9,
    "minimum_distance_penalty": 0.5,
    "use_response_expansion": True,
    "min_pass_through": 2,
    "occupancy_threshold": 0.1,
}


def generate_launch_description():
    ld = LaunchDescription()

    map_arg = DeclareLaunchArgument(
        "map", default_value=DEFAULT_MAP, description="地图 yaml 路径（AMCL 模式用）"
    )
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="True = 用 slam_toolbox 边跑边建图",
    )
    params_arg = DeclareLaunchArgument(
        "params_file", default_value=DEFAULT_PARAMS, description="Nav2 参数文件"
    )
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )
    ld.add_action(map_arg)
    ld.add_action(slam_arg)
    ld.add_action(params_arg)
    ld.add_action(sim_time_arg)

    nav2_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    amcl_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, "bringup_launch.py")),
        condition=UnlessCondition(LaunchConfiguration("slam")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "slam": "False",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
            "use_namespace": "true",
            "namespace": ROBOT_NAMESPACE,
        }.items(),
    )

    # SLAM 模式只起导航节点；slam_toolbox 由下面的 Node 单独承担。
    # 不能再用 bringup_launch.py slam:=False，否则 AMCL + map_server 会一起启动，
    # 与 slam_toolbox 争 /map 和 map→odom（2026-10-17 实测的结构性问题）。
    navigation_only = GroupAction(
        condition=IfCondition(LaunchConfiguration("slam")),
        actions=[
            PushRosNamespace(ROBOT_NAMESPACE),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_dir, "navigation_launch.py")),
                launch_arguments={
                    "namespace": ROBOT_NAMESPACE,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": "true",
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            ),
        ],
    )

    ld.add_action(
        GroupAction(
            actions=[
                SetRemap(src="cmd_vel_smoothed", dst=NAV_OUTPUT_TOPIC),
                SetRemap(src="cmd_vel", dst="cmd_vel_limited"),
                amcl_bringup,
                navigation_only,
            ]
        )
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace=ROBOT_NAMESPACE,
        parameters=[SLAM_PARAMS],
        # 仿真的 TF 发在 /robot1/tf（gazebo launch 把 /tf 重映射进了命名空间）；
        # tf2_ros 默认订绝对话题 /tf，不重映射就看不到 odom→base_link。
        # slam_toolbox 的地图发布器用绝对话题 /map；项目统一约定是 /robot1/map，
        # 不重映射的话 Nav2 static_layer、网页和 e2e 都以为「没有地图」。
        remappings=[
            ("/tf", "/robot1/tf"),
            ("/tf_static", "/robot1/tf_static"),
            ("/map", "/robot1/map"),
            ("/map_metadata", "/robot1/map_metadata"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    map_saver_node = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        namespace=ROBOT_NAMESPACE,
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    lifecycle_manager_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        namespace=ROBOT_NAMESPACE,
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_saver"]},
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    ld.add_action(slam_toolbox_node)
    ld.add_action(map_saver_node)
    ld.add_action(lifecycle_manager_slam)

    return ld
