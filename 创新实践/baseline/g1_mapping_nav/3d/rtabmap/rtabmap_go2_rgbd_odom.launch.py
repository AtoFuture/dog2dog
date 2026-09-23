# Minimal RTAB-Map RGB-D + wheel odom for Go2 sim (first-run trial, passive, no cmd_vel)
# Trio (trial-1): rgb=/robot1/rgbd_d435/image depth=/robot1/rgbd_d435/depth_image info=/robot1/color/camera_info
# NOTE: info frame_id=camera_face mismatches rgb/depth camera_d435; approx_sync=true tolerates rate diff.
# TF: sim publishes on /robot1/tf + /robot1/tf_static -> remapped. publish_tf=false to avoid fighting amcl map->odom.
import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    db_path = os.path.expanduser('~/data1/创新实践/baseline/g1_mapping_nav/3d/rtabmap/rtabmap_trial1.db')
    params = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_odom': True,
        'subscribe_rgbd': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.5,
        'topic_queue_size': 20,
        'sync_queue_size': 20,
        'qos': 1,
        'qos_image': 1,
        'qos_camera_info': 1,
        'qos_odom': 1,
        'publish_tf': False,
        'publish_map': True,
        'Grid/FromDepth': 'true',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/CellSize': '0.05',
        'RGBD/NeighborLinkRefining': 'true',
        'database_path': db_path,
        'Rtabmap/DetectionRate': '2.0',
    }]
    remap_tf = [
        ('/tf', '/robot1/tf'),
        ('/tf_static', '/robot1/tf_static'),
    ]
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', output='screen',
        namespace='robot1',
        parameters=[{'approx_sync': True,
                     'approx_sync_max_interval': 0.5,
                     'topic_queue_size': 20,
                     'sync_queue_size': 20,
                     'qos': 1,
                     'qos_camera_info': 1}],
        remappings=[
            ('rgb/image', '/robot1/rgbd_d435/image'),
            ('depth/image', '/robot1/rgbd_d435/depth_image'),
            ('rgb/camera_info', '/robot1/color/camera_info'),
            ('rgbd_image', '/robot1/rgbd_d435/rgbd_image'),
        ] + remap_tf,
    )
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        namespace='robot1',
        parameters=params,
        remappings=[
            ('rgb/image', '/robot1/rgbd_d435/image'),
            ('depth/image', '/robot1/rgbd_d435/depth_image'),
            ('rgb/camera_info', '/robot1/color/camera_info'),
            ('odom', '/robot1/odometry/filtered'),
        ] + remap_tf,
        arguments=['--delete_db_on_start'],
    )
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        rgbd_sync,
        rtabmap,
    ])
