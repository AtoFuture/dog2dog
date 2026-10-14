#!/usr/bin/env python3
# nav2_manual.launch.py — 手工起 Nav2 核心节点（绕过 nav2_bringup 的 RewrittenYaml）。
# 参数文件 go2_nav2_ns.yaml 把原始 go2_nav2.yaml 整体包在 robot1: 根下（键匹配 /robot1/<node>）。
from launch import LaunchDescription
from launch_ros.actions import Node

PF = '/tmp/go2_nav2_ns.yaml'
NS = 'robot1'
NODES = [
    ('nav2_controller', 'controller_server', 'controller_server'),
    ('nav2_planner', 'planner_server', 'planner_server'),
    ('nav2_behaviors', 'behavior_server', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ('nav2_smoother', 'smoother_server', 'smoother_server'),
    ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
]


def node(pkg, exe, name):
    return Node(package=pkg, executable=exe, name=name, namespace=NS, output='screen',
                parameters=[PF, {'use_sim_time': True}],
                remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')])


def generate_launch_description():
    acts = [node(*n) for n in NODES]
    acts.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=NS, output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': [n[2] for n in NODES]}]))
    return LaunchDescription(acts)
