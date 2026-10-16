"""Actuator watchdogs still receive state liveness when the ROS clock freezes."""
import time
import pytest
pytest.importorskip('rclpy')
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from g3_integration_eval.state_machine_node import G3StateMachineNode,MISSION_STATE_QOS


def test_state_heartbeat_uses_steady_time_without_clock_messages():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','state_publish_period_sec:=0.1'])
    node=G3StateMachineNode();peer=Node('heartbeat_observer');seen=[]
    peer.create_subscription(String,'/mission_state',lambda msg:seen.append(msg.data),MISSION_STATE_QOS)
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(peer)
    try:
        end=time.monotonic()+2
        while time.monotonic()<end and len(seen)<3:executor.spin_once(timeout_sec=.02)
        assert node.get_clock().now().nanoseconds==0
        assert len(seen)>=3 and set(seen)=={'standby'}
    finally:
        executor.remove_node(node);executor.remove_node(peer);node.destroy_node();peer.destroy_node();executor.shutdown();rclpy.shutdown()
