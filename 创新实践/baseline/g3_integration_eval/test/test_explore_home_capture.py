"""EXPLORE home reference capture defers within a bounded retry window."""
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch
import pytest
pytest.importorskip('rclpy')
import rclpy
from g3_integration_eval.state_machine_node import G3StateMachineNode
from g3_integration_eval.state_machine import State, Event
from g3_integration_eval.return_localization import HomeReference, Pose2


def make_sample():
    return SimpleNamespace(
        map_pose=Pose2(x=0., y=0., yaw=0.),
        odom_pose=Pose2(x=0., y=0., yaw=0.),
        map_to_odom=Pose2(x=0., y=0., yaw=0.),
        linear_speed=0.,
        angular_speed=0.,
        stamp_ns=0,
    )


@pytest.fixture
def node():
    rclpy.init()
    with patch('g3_integration_eval.state_machine_node.NavigateToPoseClient') as client:
        client.return_value = Mock()
        n = G3StateMachineNode()
    n._return_localization_enabled = True
    n._navigator.cancel_current_goal = Mock(return_value=False)
    yield n
    n.destroy_node(); rclpy.shutdown()


def test_explore_defers_until_home_reference_available(node):
    node._capture_home_reference = Mock(side_effect=[False, True, True])
    node._apply_event(Event.EXPLORE, source='command')
    assert node._state is State.STANDBY
    assert node._explore_capture_pending is not None
    node._check_return_localization()
    assert node._state is State.EXPLORE
    assert node._explore_capture_pending is None


def test_deferred_explore_keeps_waiting_before_deadline(node):
    node._capture_home_reference = Mock(return_value=False)
    node._apply_event(Event.EXPLORE, source='command')
    deadline, source = node._explore_capture_pending
    assert time.monotonic() < deadline
    node._check_return_localization()
    assert node._state is State.STANDBY
    assert node._explore_capture_pending is not None


def test_deferred_explore_times_out_and_stays_bounded(node):
    node._capture_home_reference = Mock(return_value=False)
    node._apply_event(Event.EXPLORE, source='command')
    deadline, source = node._explore_capture_pending
    node._explore_capture_pending = (time.monotonic() - 1.0, source)
    node._check_return_localization()
    assert node._state is State.STANDBY
    assert node._explore_capture_pending is None
    node._check_return_localization()
    assert node._state is State.STANDBY


def test_state_change_cancels_pending_capture(node):
    node._capture_home_reference = Mock(return_value=False)
    node._apply_event(Event.EXPLORE, source='command')
    assert node._explore_capture_pending is not None
    node._apply_event(Event.ESTOP, source='command')
    assert node._state is State.ESTOP
    node._check_return_localization()
    assert node._explore_capture_pending is None
    assert node._state is State.ESTOP


def test_disabled_return_localization_never_defers(node):
    node._return_localization_enabled = False
    node._capture_home_reference = Mock(return_value=False)
    node._apply_event(Event.EXPLORE, source='command')
    assert node._state is State.EXPLORE
    assert node._explore_capture_pending is None
    node._capture_home_reference.assert_not_called()


@pytest.mark.parametrize('events',[(Event.STANDBY,),(Event.ESTOP,Event.RESET)])
def test_stop_cancels_queued_explore_before_next_timer(node,events):
    node._capture_home_reference = Mock(return_value=False)
    node._apply_event(Event.EXPLORE,source='command')
    assert node._explore_capture_pending is not None
    for event in events:node._apply_event(event,source='command')
    node._capture_home_reference = Mock(return_value=True)
    node._check_return_localization()
    assert node._state is State.STANDBY
    assert node._explore_capture_pending is None


def test_return_goal_defers_until_localization_sample_available(node):
    node._home_reference = HomeReference(
        map_pose=Pose2(x=0., y=0., yaw=0.),
        odom_pose=Pose2(x=0., y=0., yaw=0.),
        map_to_odom=Pose2(x=0., y=0., yaw=0.),
        scan_points=[],
        stamp_ns=0,
    )
    node._current_localization = Mock(side_effect=[None, make_sample(), make_sample(), make_sample()])
    node._send_return('exploration:candidates_unavailable')
    assert node._state is State.STANDBY
    assert node._return_goal_pending is not None
    node._state = State.RETURN
    node._check_return_localization()
    assert node._return_goal_pending is None
    node._navigator.request_return.assert_called_once()


def test_return_goal_pending_times_out_bounded(node):
    node._home_reference = Mock()
    node._current_localization = Mock(return_value=None)
    node._state = State.RETURN
    node._send_return('exploration:candidates_unavailable')
    assert node._return_goal_pending is not None
    trigger, target, source, deadline = node._return_goal_pending
    node._return_goal_pending = (trigger, target, source, time.monotonic() - 1.0)
    node._check_return_localization()
    assert node._state is State.ESTOP
    assert node._return_goal_pending is None
    node._navigator.request_return.assert_not_called()


def test_leaving_return_clears_goal_pending(node):
    node._home_reference = Mock()
    node._current_localization = Mock(return_value=None)
    node._state = State.RETURN
    node._send_return('exploration:candidates_unavailable')
    assert node._return_goal_pending is not None
    node._apply_event(Event.ESTOP, source='command')
    assert node._state is State.ESTOP
    assert node._return_goal_pending is None


def make_home(odom_x=5.0, odom_y=1.0):
    return HomeReference(
        map_pose=Pose2(x=odom_x, y=odom_y, yaw=0.),
        odom_pose=Pose2(x=odom_x, y=odom_y, yaw=0.),
        map_to_odom=Pose2(x=0., y=0., yaw=0.),
        scan_points=[],
        stamp_ns=0,
    )


def test_first_return_goal_uses_configured_map_home(node):
    node._home_reference = make_home()
    node._current_localization = Mock(return_value=make_sample())
    node._send_return('low_battery')
    home_target = node._navigator.request_return.call_args.kwargs['home_target']
    assert (home_target['x'], home_target['y']) == (0.0, 0.0)
    assert home_target['target_source'] == 'configured_map_home'


def test_return_uses_dedicated_orientation_checked_tree(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('return_bt_path',value='/tmp/private_return_bt.xml')])
    node._home_reference=make_home()
    node._current_localization=Mock(return_value=make_sample())
    node._send_return('low_battery')
    goal=node._navigator.request_return.call_args.args[0]
    assert goal.behavior_tree=='/tmp/private_return_bt.xml'


def test_localization_queries_tf_at_odometry_time_instead_of_future_latest(node):
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    now=node.get_clock().now().nanoseconds
    stamp=now-50_000_000
    node._latest_odometry=dict(pose=Pose2(.2,0.,0.),linear_speed=0.,angular_speed=0.,stamp_ns=stamp)
    def lookup(_target,_source,when):
        tr=TransformStamped()
        # SLAM deliberately publishes a future-dated transform. Looking up
        # the sample time interpolates its history without accepting future data.
        tr.header.stamp=Time(nanoseconds=(now+200_000_000 if when.nanoseconds==0 else when.nanoseconds)).to_msg()
        tr.transform.rotation.w=1.
        tr.transform.translation.x=.4
        return tr
    node._recovery_tf=Mock()
    node._recovery_tf.lookup_transform.side_effect=lookup
    sample=node._current_localization()
    assert sample is not None
    assert sample.map_pose.x==pytest.approx(.6)
    assert node._recovery_tf.lookup_transform.call_args.args[2].nanoseconds==stamp


def test_localization_keeps_rejecting_stale_odometry(node):
    node._latest_odometry=dict(pose=Pose2(0.,0.,0.),linear_speed=0.,angular_speed=0.,
                              stamp_ns=node.get_clock().now().nanoseconds-2_000_000_000)
    node._recovery_tf=Mock()
    assert node._current_localization() is None
    node._recovery_tf.lookup_transform.assert_not_called()


def test_localization_does_not_accept_future_tf_even_for_sample_time_query(node):
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    now=node.get_clock().now().nanoseconds
    node._latest_odometry=dict(pose=Pose2(0.,0.,0.),linear_speed=0.,angular_speed=0.,stamp_ns=now)
    tr=TransformStamped();tr.transform.rotation.w=1.
    tr.header.stamp=Time(nanoseconds=now+200_000_000).to_msg()
    node._recovery_tf=Mock();node._recovery_tf.lookup_transform.return_value=tr
    assert node._current_localization() is None


def test_rebased_return_goal_uses_dynamic_odometry_home(node):
    node._home_reference = make_home(odom_x=5.0, odom_y=1.0)
    sample = make_sample()
    sample.map_to_odom = Pose2(x=2.0, y=3.0, yaw=0.)
    node._current_localization = Mock(return_value=sample)
    node._send_return('low_battery', target_source='localization_rebased_home')
    home_target = node._navigator.request_return.call_args.kwargs['home_target']
    assert (home_target['x'], home_target['y']) == (7.0, 4.0)
    assert home_target['target_source'] == 'localization_rebased_home'
