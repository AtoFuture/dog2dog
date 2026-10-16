"""Shared startup exclusion for archived visual/direct-control experiments."""
import time


def assert_no_runtime():
    import rclpy
    rclpy.init(args=[]);node=rclpy.create_node('experimental_navigation_probe')
    try:
        end=time.monotonic()+1.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        names={name for name,namespace in node.get_node_names_and_namespaces()}
        if names & {'vision_explorer','mission_brain','velocity_guard','qwen_robotnav_nav_node',
                    'deepseek_nav2_waypoint_bridge','qwen_robotnav_go2_controller','go2_ros_bridge'}:
            raise RuntimeError('Another goal owner/runtime exists; archived experiments cannot run alongside it')
    finally:
        node.destroy_node();rclpy.shutdown()
