import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    default_params = os.path.abspath(os.path.join(here, "..", "config", "explore_params.yaml"))

    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_params = DeclareLaunchArgument("params_file", default_value=default_params)
    declare_ns = DeclareLaunchArgument("namespace", default_value="robot1")
    declare_sim = DeclareLaunchArgument("use_sim_time", default_value="true")

    # IMPORTANT: NO ('/tf','tf') remap. Humble tf2_ros uses ABSOLUTE /tf,/tf_static
    # (not namespace-prefixed). Applying the upstream remap would move TF into
    # /robot1/tf and break map->base_link.
    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        namespace=namespace,
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([declare_params, declare_ns, declare_sim, explore_node])
