from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic", default_value="/frontvideostream"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/camera/go2/image_raw"
            ),
            DeclareLaunchArgument("frame_id", default_value=""),
            DeclareLaunchArgument("video_stream", default_value="video720p"),
            Node(
                package="go2_video_bridge",
                executable="go2_video_bridge_node",
                name="go2_video_bridge_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "video_stream": LaunchConfiguration("video_stream"),
                    }
                ],
            ),
        ]
    )
