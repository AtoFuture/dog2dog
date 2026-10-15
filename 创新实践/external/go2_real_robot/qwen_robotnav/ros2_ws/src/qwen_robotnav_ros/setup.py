from setuptools import find_packages, setup

package_name = 'qwen_robotnav_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_nav.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shuuyou',
    maintainer_email='shuuyou@hdu.edu.cn',
    description='ROS2 node for Qwen-RobotNav navigation on Unitree Go2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_node = qwen_robotnav_ros.nav_node:main',
            'go2_controller = qwen_robotnav_ros.go2_controller:main',
            'go2_ros_bridge = qwen_robotnav_ros.go2_ros_bridge:main',
        ],
    },
)
