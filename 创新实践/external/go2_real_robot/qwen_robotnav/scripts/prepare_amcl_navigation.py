"""Prepare isolated AMCL/Nav2 configuration without simulator-truth inputs.

Run in ROS Humble. Pair with prepare_simulator_launch.py --single-odom-tf.
The generated navigation output must go through the simulator velocity scaler.
"""

import argparse
from pathlib import Path
import shutil

import yaml


def main():
    project = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('/tmp/deepseek_amcl'))
    parser.add_argument('--sim-workspace', type=Path, default=Path.home()/'go2_sim_ws/ROS2-Gazebo-GO2')
    parser.add_argument('--nav-config', type=Path, default=project/'baseline/g1_mapping_nav/config/go2_nav2.yaml')
    parser.add_argument('--bt-file', type=Path, default=project/'baseline/g1_mapping_nav/config/nav2_go2_bt.xml')
    parser.add_argument('--initial-x', type=float, default=6.86)
    parser.add_argument('--initial-y', type=float, default=6.0)
    parser.add_argument('--initial-yaw', type=float, default=0.0)
    args = parser.parse_args()
    root = args.output_dir.resolve()
    if not str(root).isascii():
        parser.error('Use an ASCII output directory for ROS parameter paths')
    root.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(args.nav_config.read_text())
    amcl = config['amcl']['ros__parameters']
    amcl.update(base_frame_id='base_link', odom_frame_id='odom', scan_topic='/robot1/velodyne',
                set_initial_pose=True, transform_tolerance=0.5,
                initial_pose=dict(x=args.initial_x, y=args.initial_y, z=0.0, yaw=args.initial_yaw))
    # Known dock pose is initialization only; no truth relay or fixed map->odom.
    shutil.copyfile(args.bt_file, root/'nav2_go2_bt.xml')
    config['bt_navigator']['ros__parameters'].update(
        odom_topic='/robot1/odometry/filtered', default_nav_to_pose_bt_xml=str(root/'nav2_go2_bt.xml'))
    config['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = 'odom'
    config['behavior_server']['ros__parameters']['global_frame'] = 'odom'
    config['velocity_smoother']['ros__parameters']['odom_topic'] = '/robot1/odometry/filtered'
    controller = config['controller_server']['ros__parameters']
    controller['controller_frequency'] = 10.0
    controller['progress_checker'].update(required_movement_radius=0.1, movement_time_allowance=30.0)
    controller['FollowPath']['desired_linear_vel'] = 0.20
    (root/'nav2.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    official = Path('/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py').read_text()
    smoother = "('cmd_vel_smoothed', 'cmd_vel')"
    behavior = 'remappings=remappings),'
    if smoother not in official or behavior not in official:
        parser.error('Nav2 launch layout changed; velocity remapping needs review')
    official = official.replace("('/tf', 'tf')", "('/tf', '/robot1/tf')")
    official = official.replace(smoother, "('cmd_vel_smoothed', '/deepseek/nav_cmd')")
    official = official.replace(behavior, "remappings=remappings+[('cmd_vel','/deepseek/nav_cmd')]),")
    (root/'navigation.launch.py').write_text(official)
    map_file = args.sim_workspace/'src/navigation2/maps/warehouse_map.yaml'
    launch = '''import yaml
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    root = ROOT
    config = yaml.safe_load(open(root+'/nav2.yaml'))
    return LaunchDescription([GroupAction([
        PushRosNamespace('robot1'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': MAP_FILE, 'use_sim_time': True}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[config['amcl']['ros__parameters']],
             remappings=[('/tf','/robot1/tf'),('/tf_static','/robot1/tf_static')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server','amcl']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(root+'/navigation.launch.py'),
             launch_arguments={'namespace':'robot1','params_file':root+'/nav2.yaml',
                               'use_sim_time':'true','autostart':'true','use_composition':'False'}.items()),
    ])])
'''.replace('ROOT', repr(str(root))).replace('MAP_FILE', repr(str(map_file)))
    (root/'amcl_navigation.launch.py').write_text(launch)
    print(root/'amcl_navigation.launch.py')
    print('Velocity output: /deepseek/nav_cmd; use cmd_vel_scaler before simulator cmd_vel.')


if __name__ == '__main__':
    main()
