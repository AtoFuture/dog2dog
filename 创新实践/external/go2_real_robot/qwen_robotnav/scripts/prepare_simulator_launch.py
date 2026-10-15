"""Generate a scoped Go2 launch with a horizontal single-line navigation lidar.

Run inside the existing ROS environment. Shared workspace files are only read.
"""

import argparse
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-workspace',type=Path,default=Path.home()/'go2_sim_ws/ROS2-Gazebo-GO2')
    parser.add_argument('--output-dir',type=Path,default=Path('/tmp/deepseek_nav2_sim'))
    parser.add_argument('--single-odom-tf',action='store_true',help='Separate raw leg odometry and let EKF alone publish odom TF')
    args=parser.parse_args()
    output=args.output_dir.resolve()
    if not str(output).isascii():
        parser.error('Use an ASCII output directory to avoid ROS parameter path escaping')
    launch_dir=args.sim_workspace/'src/gazebo_sim/launch'
    child=(launch_dir/'gazebo_go2_sensors.launch.py').read_text()
    top=(launch_dir/'launch.py').read_text()
    needle="robot_desc = xacro.process_file(xacro_file, mappings={'robot_name': robot_name}).toxml()"
    choice="launch_file = 'gazebo_go2_sensors.launch.py'"
    if needle not in child or choice not in top:
        parser.error('Simulator launch layout changed; model generation could not be located')
    child=child.replace(needle,needle+'\n        from qwen_robotnav.sim_model import horizontal_lidar_description, rgbd_optical_description\n        robot_desc = rgbd_optical_description(horizontal_lidar_description(robot_desc))')
    if args.single_odom_tf:
        odom_remaps='remappings=remappings\n        )\n\n        rviz_launch_file'
        ekf_params="{'use_sim_time': use_sim_time}],"
        if "'enable_odom_tf': True" not in child or odom_remaps not in child or ekf_params not in child:
            parser.error('Simulator odometry launch layout changed; refusing an ambiguous TF setup')
        child=child.replace("'enable_odom_tf': True", "'enable_odom_tf': False, 'use_sim_time': True")
        child=child.replace(odom_remaps,"remappings=[('/tf','tf'),('/tf_static','tf_static'),('odom','leg/odometry')]\n        )\n\n        rviz_launch_file")
        # Foot twist is a gait command (not m/s), and its yaw rate is negated.
        # Fuse independent position + raw IMU yaw/rate instead of those duplicates.
        fusion = """{'use_sim_time': use_sim_time,
             'odom0': f'/{namespace}/leg/odometry',
             'imu0': f'/{namespace}/imu_plugin/out',
             'odom0_config': [True, True, False, False, False, False, False, False, False, False, False, False, False, False, False],
             'imu0_config': [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False]}],"""
        child=child.replace(ekf_params, fusion)
    output.mkdir(parents=True,exist_ok=True)
    # Humble spawner ignores --service-call-timeout specifically for load_controller.
    # Patch its imported function in this child process only, leaving /opt/ros intact.
    spawner=output/'controller_spawner'
    spawner.write_text("""#!/usr/bin/env python3
import controller_manager.spawner as spawner
from controller_manager.controller_manager_services import load_controller

def scoped_load(node, manager, name):
    return load_controller(node, manager, name, service_timeout=60., call_timeout=60.)

spawner.load_controller=scoped_load
spawner.main()
""")
    spawner.chmod(0o755)
    if "executable='spawner'" not in child:
        parser.error('Cannot locate controller spawner in simulator launch')
    child=child.replace("executable='spawner'", 'executable='+repr(str(spawner)))
    (output/'go2_sensors_nav2.launch.py').write_text(child)
    top=top.replace(choice,'launch_file = '+repr(str(output/'go2_sensors_nav2.launch.py')))
    target=output/'gazebo_nav2.launch.py'
    target.write_text(top)
    print(target)
    print('Keep qwen_robotnav importable in the ROS environment when launching this file.')


if __name__=='__main__':
    main()
