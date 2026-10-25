#!/usr/bin/env python3
# nav2_manual.launch.py — 手工起 Nav2 全部节点（绕过 nav2_bringup 的 RewrittenYaml）。
# 参数文件 /tmp/go2_nav2_ns.yaml 把原始 go2_nav2.yaml 整体包在 robot1: 根下（键匹配 /robot1/<node>）。
#
# 覆盖：定位（map_server + amcl + lifecycle_manager_localization）+ 导航
#       （controller/planner/behavior/bt_navigator/smoother/velocity_smoother + lifecycle_manager_navigation）。
# 前提：全局 /tf（无命名空间重映射）；EKF 发 odom->base_link（publish_tf: true）。
# 用法：
#   python3 - <<'PY'
#   src=open('<...>/src/navigation2/param/go2_nav2.yaml').read().splitlines()
#   open('/tmp/go2_nav2_ns.yaml','w').write('\n'.join(['robot1:']+[('  '+l if l.strip() else l) for l in src])+'\n')
#   PY
#   ros2 launch <this>.launch.py map:=/abs/path/to/map.yaml
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PF = '/tmp/go2_nav2_ns.yaml'
NS = 'robot1'

NAV_NODES = [
    ('nav2_controller', 'controller_server', 'controller_server'),
    ('nav2_planner', 'planner_server', 'planner_server'),
    ('nav2_behaviors', 'behavior_server', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ('nav2_smoother', 'smoother_server', 'smoother_server'),
    ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
]
LOC_NODES = ['map_server', 'amcl']


def nav_node(pkg, exe, name):
    return Node(package=pkg, executable=exe, name=name, namespace=NS, output='screen',
                parameters=[PF, {'use_sim_time': True}],
                remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')])


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    acts = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value='/tmp/go2_map.yaml',
                              description='已保存地图 yaml 的绝对路径'),
    ]
    for n in NAV_NODES:
        acts.append(nav_node(*n))
    acts.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=NS, output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': [n[2] for n in NAV_NODES]}]))

    # ---- localization: map_server + amcl ----
    acts.append(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        namespace=NS, output='screen',
        parameters=[PF, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]))
    acts.append(Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        namespace=NS, output='screen', parameters=[PF, {'use_sim_time': use_sim_time}]))
    acts.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', namespace=NS, output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': LOC_NODES}]))

    return LaunchDescription(acts)
