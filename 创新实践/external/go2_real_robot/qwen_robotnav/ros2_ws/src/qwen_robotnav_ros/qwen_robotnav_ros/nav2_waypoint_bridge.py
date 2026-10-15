"""Send DeepSeek observation-frame goals to Nav2; never publish cmd_vel."""

import json
import math
import time
from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from qwen_robotnav.nav2_goals import select_goal, transform_goal


class Nav2WaypointBridge(Node):
    def __init__(self):
        super().__init__('deepseek_nav2_waypoint_bridge')
        defaults = {
            'waypoint_topic': '/qwen_robotnav/waypoints',
            'nav_action': '/robot1/navigate_to_pose',
            'global_frame': 'map',
            'min_goal_distance_m': 1.0,
            'max_goal_distance_m': 2.5,
            'waypoint_timeout_sec': 8.0,
            'goal_timeout_sec': 120.0,
            'require_mission_state': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._frame = str(self.get_parameter('global_frame').value)
        self._minimum = float(self.get_parameter('min_goal_distance_m').value)
        self._maximum = float(self.get_parameter('max_goal_distance_m').value)
        self._input_timeout = float(self.get_parameter('waypoint_timeout_sec').value)
        self._goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        select_goal([], self._minimum, self._maximum)
        if not all(math.isfinite(v) and v > 0 for v in (self._input_timeout, self._goal_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        self._allowed = not bool(self.get_parameter('require_mission_state').value)
        self._handoff_token = None
        self._pending = None
        self._last_input = None
        self._attempt = None
        self._warned_at = 0.0
        self._sequence = 0
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._client = ActionClient(self, NavigateToPose, str(self.get_parameter('nav_action').value))
        self._events = self.create_publisher(String, '/deepseek/nav2_events', 10)
        self._goals = self.create_publisher(PoseStamped, '/deepseek/nav2_goal', 10)
        self.create_subscription(PoseArray, str(self.get_parameter('waypoint_topic').value), self._on_waypoints, 1)
        self.create_subscription(String, '/mission_state', self._on_mission, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._handoff_ack = self.create_publisher(String, '/navigation/handoff_ack', 10)
        self.create_subscription(String, '/navigation/handoff_request', self._on_handoff, 10)
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.2, self._tick, clock=self._watchdog_clock)
        self.get_logger().info('DeepSeek → Nav2 bridge ready; Nav2 owns planning and velocity control')

    def _event(self, status, attempt=None, **details):
        payload = {'status': status, 'monotonic_time': time.monotonic(), **details}
        if attempt:
            payload.update(attempt_id=attempt['id'], target=attempt['target'])
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._events.publish(msg)
        self.get_logger().info(msg.data)

    def _warn(self, message):
        now = time.monotonic()
        if now-self._warned_at > 5:
            self.get_logger().warning(message)
            self._warned_at = now

    def _on_waypoints(self, msg):
        if not self._allowed:
            return
        try:
            if not msg.header.frame_id or not (msg.header.stamp.sec or msg.header.stamp.nanosec):
                raise ValueError('An observation frame and nonzero acquisition timestamp are required')
            points = [(p.position.x, p.position.y,
                       2*math.atan2(p.orientation.z, p.orientation.w)) for p in msg.poses]
            selected = select_goal(points, self._minimum, self._maximum)
            if selected is None:
                self._warn('No waypoint in the configured Nav2 goal distance interval')
                return
            # Reject delayed/replayed inputs before they can refresh the watchdog.
            age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds*1e-9
            if age < -0.1 or age > self._input_timeout:
                raise ValueError(f'Observation timestamp is stale or in the future: age={age:.3f}s')
        except ValueError as exc:
            self._warn(str(exc))
            return
        now = time.monotonic()
        self._pending = (msg, selected, now)
        self._last_input = now

    def _on_mission(self, msg):
        self._allowed = msg.data.strip().lower() == 'exploring' and self._handoff_token is None
        if not self._allowed:
            self._pending = None
            self.cancel_navigation('mission:'+msg.data)

    def _on_handoff(self, msg):
        try:
            token = json.loads(msg.data)['token']
            if not isinstance(token, str) or not token or len(token) > 128:
                raise ValueError('Invalid handoff token')
        except (ValueError, KeyError, TypeError):
            self._warn('Invalid navigation handoff request')
            return
        self._handoff_token = token
        self.stop_navigation('mission_handoff')

    def _tick(self):
        if self._handoff_token is not None and self._attempt is None:
            self._handoff_ack.publish(String(data=json.dumps({'token': self._handoff_token, 'released': True})))
            self._handoff_token = None
        now = time.monotonic()
        if self._attempt is not None:
            if now-self._attempt['started'] > self._goal_timeout:
                self.cancel_navigation('goal_timeout')
            elif self._last_input is None or now-self._last_input > self._input_timeout:
                self.cancel_navigation('vision_timeout')
            return
        if not self._allowed or self._pending is None:
            return
        msg, relative, received = self._pending
        if now-received > self._input_timeout:
            self._pending = None
            return
        if not self._client.server_is_ready():
            self._warn('Nav2 action server is not ready; waiting without blocking ROS callbacks')
            return
        try:
            transform = self._buffer.lookup_transform(self._frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException as exc:
            self._warn(f'Waiting for observation-time TF: {exc}')
            return
        q = transform.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        t = transform.transform.translation
        try:
            target = transform_goal(relative, t.x, t.y, yaw)
        except ValueError as exc:
            self._pending = None
            self._warn(str(exc))
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target.x, target.y
        goal.pose.pose.orientation.z = math.sin(target.yaw/2)
        goal.pose.pose.orientation.w = math.cos(target.yaw/2)
        self._sequence += 1
        attempt = {'id': self._sequence, 'started': now, 'handle': None, 'cancel': False,
                   'target': {'x': target.x, 'y': target.y, 'yaw': target.yaw}}
        self._attempt = attempt
        self._pending = None
        self._goals.publish(goal.pose)
        self._event('sending', attempt)
        try:
            future = self._client.send_goal_async(goal)
            future.add_done_callback(partial(self._on_response, attempt))
        except Exception as exc:
            self._event('send_error', attempt, error=str(exc))
            self._attempt = None

    def _on_response(self, attempt, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._event('send_error', attempt, error=str(exc))
            if self._attempt is attempt:
                self._attempt = None
            return
        if not handle.accepted:
            self._event('rejected', attempt)
            if self._attempt is attempt:
                self._attempt = None
            return
        attempt['handle'] = handle
        self._event('accepted', attempt)
        try:
            handle.get_result_async().add_done_callback(partial(self._on_result, attempt))
        except Exception as exc:
            self._event('result_error', attempt, error=str(exc))
            self.cancel_navigation('result_error')
        if attempt['cancel'] or self._attempt is not attempt:
            self._cancel_handle(attempt)

    def _on_result(self, attempt, future):
        try:
            status = future.result().status
        except Exception as exc:
            self._event('result_error', attempt, error=str(exc))
            self.cancel_navigation('result_error')
            return
        self._event({4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(status, 'unknown'), attempt)
        if self._attempt is attempt:
            self._attempt = None

    def _cancel_handle(self, attempt):
        if attempt.get('cancel_sent'):
            return
        attempt['cancel_sent'] = True
        try:
            future = attempt['handle'].cancel_goal_async()
            future.add_done_callback(partial(self._on_cancel, attempt))
        except Exception as exc:
            self._event('cancel_error', attempt, error=str(exc))

    def _on_cancel(self, attempt, future):
        try:
            accepted = bool(future.result().goals_canceling)
            self._event('cancel_accepted' if accepted else 'cancel_rejected', attempt)
        except Exception as exc:
            self._event('cancel_error', attempt, error=str(exc))

    def cancel_navigation(self, reason='shutdown'):
        attempt = self._attempt
        if attempt is None or attempt['cancel']:
            return
        attempt['cancel'] = True
        self._event('cancel_requested', attempt, reason=reason)
        if attempt['handle'] is not None:
            self._cancel_handle(attempt)

    def stop_navigation(self, reason='shutdown'):
        """Latch a stop so a queued prediction cannot restart motion after cancel."""
        self._allowed = False
        self._pending = None
        self.cancel_navigation(reason)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Nav2WaypointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_navigation()
        deadline = time.monotonic()+2
        while rclpy.ok() and node._attempt is not None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
