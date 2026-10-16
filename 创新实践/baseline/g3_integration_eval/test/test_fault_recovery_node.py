"""Exercise real ROS nodes; navigation transmission stays disabled."""
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
pytest.importorskip('rclpy')
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from g3_integration_eval.state_machine_node import G3StateMachineNode
from g3_integration_eval.state_machine import State, Event


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-p','require_navigation_handoff:=true','-p','enable_fault_recovery:=true'])
    n=G3StateMachineNode()
    n.events=[]
    n._recovery_event=lambda status,**kw:n.events.append((status,kw))
    n._recovery_tf=Mock()
    n._recovery_tf.lookup_transform.return_value=SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=0)),
        transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.,z=0.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    yield n
    n.destroy_node();rclpy.shutdown()


def healthy(n):
    now=time.monotonic()
    for k in n._health.sources:n._health.observe(k,now)
    n._health.healthy_since=now-4


def ack(n,token=None):
    n._on_handoff_ack(String(data=json.dumps(dict(token=token or n._handoff_pending[0],released=True))))


def test_resume_requires_fresh_health_and_matching_terminal_ack(node):
    node._state=State.EXPLORE
    node._check_recovery()
    assert node._state is State.PAUSED
    token=node._handoff_pending[0]
    healthy(node)
    node._check_recovery()
    assert node._state is State.PAUSED
    ack(node,'old')
    assert not node._pause_released
    ack(node,token)
    node._check_recovery()
    assert node._state is State.EXPLORE
    assert [s for s,_ in node.events]==['fault','paused','navigation_released','resumed']


def test_manual_pause_never_auto_resumes(node):
    node._state=State.EXPLORE
    node._apply_event(Event.PAUSE,source='command');ack(node);healthy(node)
    node._check_recovery()
    assert node._state is State.PAUSED
    node._apply_event(Event.RESUME,source='command')
    assert node._state is State.EXPLORE


def test_timeout_is_latched_estop(node):
    node._state=State.EXPLORE;node._check_recovery();ack(node)
    node._paused_at=time.monotonic()-61
    node._check_recovery()
    assert node._state is State.ESTOP
    healthy(node);node._check_recovery()
    node._apply_event(Event.RESUME,source='command')
    assert node._state is State.ESTOP


def test_handoff_timeout_blocks_resume(node):
    node._state=State.EXPLORE;node._check_recovery()
    token,trigger,_=node._handoff_pending
    node._handoff_pending=(token,trigger,time.monotonic()-1)
    ack(node,token)
    assert node._state is State.ESTOP


def test_low_battery_return_cannot_override_missing_localization(node):
    node._state=State.EXPLORE;node._check_recovery()
    node._apply_event(Event.RETURN,source='battery')
    assert node._state is State.ESTOP
    assert node._handoff_pending is None


def test_return_fault_never_automatically_resumes(node):
    node._state=State.RETURN
    node._check_recovery()
    assert node._state is State.ESTOP


def test_scan_requires_valid_fresh_data_and_new_stamp(node):
    msg=LaserScan();msg.header.stamp=node.get_clock().now().to_msg()
    msg.range_min=.1;msg.range_max=10.;msg.ranges=[float('nan'),float('inf')]
    node._on_recovery_scan(msg)
    assert node._health.sources['scan'] is None
    msg.ranges=[1.]
    node._on_recovery_scan(msg)
    before=node._health.sources['scan']
    node._on_recovery_scan(msg)
    assert node._health.sources['scan']==before


def test_startup_cannot_explore_before_stable_health(node):
    node._apply_event(Event.EXPLORE,source='command')
    assert node._state is State.STANDBY
    healthy(node);node._apply_event(Event.EXPLORE,source='command')
    assert node._state is State.EXPLORE


@pytest.mark.parametrize('drop_source',['scan','localization'])
def test_ros_transport_dropout_recovery_and_terminal_release(node,drop_source):
    """Actual DDS scan/TF/handoff traffic; no simulator or robot actuator."""
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer, TransformListener, TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    peer=Node('recovery_test_peer')
    node._recovery_tf=Buffer()
    node._recovery_listener=TransformListener(node._recovery_tf,node)
    broadcaster=TransformBroadcaster(peer)
    scan_pub=peer.create_publisher(LaserScan,'/robot1/scan',qos_profile_sensor_data)
    ack_pub=peer.create_publisher(String,'/navigation/handoff_ack',10)
    pending=[]
    peer.create_subscription(String,'/navigation/handoff_request',lambda msg:pending.append(json.loads(msg.data)['token']),10)
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(peer)
    node._health.stale_s=.35;node._health.stable_s=.2;node._health.timeout_s=5.
    send_scan=[True];send_tf=[True]
    def publish():
        stamp=peer.get_clock().now().to_msg()
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='map';tf.child_frame_id='base_link'
        tf.transform.rotation.w=1.
        if send_tf[0]:broadcaster.sendTransform(tf)
        if send_scan[0]:
            msg=LaserScan();msg.header.stamp=stamp;msg.range_min=.1;msg.range_max=10.;msg.ranges=[2.]
            scan_pub.publish(msg)
    peer.create_timer(.04,publish)
    def until(predicate,timeout=4.):
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.025)
        assert predicate(), (node._state,node.events,node._health.missing(time.monotonic()))
    try:
        until(lambda:node._health.ready(time.monotonic()))
        node._on_command(String(data='EXPLORE'))
        assert node._state is State.EXPLORE
        if drop_source=='scan':send_scan[0]=False
        else:send_tf[0]=False
        until(lambda:node._state is State.PAUSED and bool(pending))
        assert node.events[0][1]['missing']==[drop_source]
        send_scan[0]=True;send_tf[0]=True
        until(lambda:node._health.ready(time.monotonic()))
        assert node._state is State.PAUSED  # healthy sensor alone cannot skip cancellation
        ack_pub.publish(String(data=json.dumps({'token':pending[-1],'released':True})))
        until(lambda:node._state is State.EXPLORE)
        assert [s for s,_ in node.events]==['fault','paused','navigation_released','resumed']
    finally:
        executor.remove_node(peer);executor.remove_node(node);executor.shutdown();peer.destroy_node()


def test_image_requires_complete_data_and_new_stamp(node):
    from sensor_msgs.msg import Image
    node._health.sources['rgb']=None
    msg=Image();msg.header.stamp=node.get_clock().now().to_msg()
    msg.width=2;msg.height=2;msg.step=6;msg.encoding='rgb8';msg.data=[0]*6
    node._on_recovery_image('rgb',msg)
    assert node._health.sources['rgb'] is None
    msg.data=[0]*12;node._on_recovery_image('rgb',msg)
    before=node._health.sources['rgb'];assert before is not None
    node._on_recovery_image('rgb',msg)
    assert node._health.sources['rgb']==before


def test_estop_during_pause_invalidates_release(node):
    node._state=State.EXPLORE;node._check_recovery()
    token=node._handoff_pending[0]
    node._apply_event(Event.ESTOP,source='command')
    ack(node,token);healthy(node);node._check_recovery()
    assert node._state is State.ESTOP and not node._pause_released
