"""Generate a scoped Go2 launch with a horizontal single-line navigation lidar.

Run inside the existing ROS environment. Shared workspace files are only read.
"""

import argparse
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-workspace',type=Path,default=Path.home()/'go2_sim_ws/ROS2-Gazebo-GO2')
    parser.add_argument('--output-dir',type=Path,default=Path('/tmp/deepseek_nav2_sim'))
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
    child=child.replace(needle,needle+'\n        from qwen_robotnav.sim_model import horizontal_lidar_description\n        robot_desc = horizontal_lidar_description(robot_desc)')
    output.mkdir(parents=True,exist_ok=True)
    (output/'go2_sensors_nav2.launch.py').write_text(child)
    top=top.replace(choice,'launch_file = '+repr(str(output/'go2_sensors_nav2.launch.py')))
    target=output/'gazebo_nav2.launch.py'
    target.write_text(top)
    print(target)
    print('Keep qwen_robotnav importable in the ROS environment when launching this file.')


if __name__=='__main__':
    main()
