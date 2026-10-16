"""ROS 2 wrapper for the G3 state machine and RETURN navigation flow."""

import json
import os
import math
import time
import uuid
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, LaserScan, Image
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from g3_integration_eval.recovery import RecoveryHealth
from std_msgs.msg import String

from g3_integration_eval.navigate_to_pose_client import (
    NavigateToPoseClient,
)
from g3_integration_eval.state_machine import (
    Event,
    InvalidTransition,
    State,
    battery_return_event,
    estop_reset_block_reason,
    mission_state_value,
    normalize_battery_percentage,
    parse_event,
    transition,
    validate_low_battery_threshold,
)


MISSION_STATE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class G3StateMachineNode(Node):
    """Expose the controller with navigation transmission disabled by default."""

    def __init__(self) -> None:
        super().__init__("g3_state_machine")

        self.declare_parameter("command_topic", "/g3/command")
        self.declare_parameter("state_topic", "/g3/state")
        self.declare_parameter("mission_state_topic", "/mission_state")
        self.declare_parameter("battery_topic", "/robot1/battery_state")
        self.declare_parameter("low_battery_threshold", 0.20)
        self.declare_parameter("navigate_action", "/robot1/navigate_to_pose")
        self.declare_parameter("return_event_topic", "/g3/return_event")
        self.declare_parameter("enable_navigation_transmission", False)
        self.declare_parameter("navigation_server_timeout_sec", 2.0)
        self.declare_parameter("home_frame", "map")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("battery_timeout_sec", 30.0)

        command_topic = str(self.get_parameter("command_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        mission_state_topic = str(
            self.get_parameter("mission_state_topic").value
        )
        battery_topic = str(self.get_parameter("battery_topic").value)
        return_event_topic = str(self.get_parameter("return_event_topic").value)
        self._low_battery_threshold = validate_low_battery_threshold(
            float(self.get_parameter("low_battery_threshold").value)
        )
        navigate_action = str(self.get_parameter("navigate_action").value)
        transmission_enabled = bool(
            self.get_parameter("enable_navigation_transmission").value
        )
        server_timeout_sec = float(
            self.get_parameter("navigation_server_timeout_sec").value
        )

        self.declare_parameter('require_navigation_handoff', False)
        self.declare_parameter('navigation_handoff_timeout_sec', 5.0)
        self._require_handoff = bool(self.get_parameter('require_navigation_handoff').value)
        self._handoff_timeout = float(self.get_parameter('navigation_handoff_timeout_sec').value)
        if not math.isfinite(self._handoff_timeout) or self._handoff_timeout <= 0:
            raise ValueError('navigation_handoff_timeout_sec must be finite and positive')
        self._handoff_pending = None
        self._handoff_publisher = self.create_publisher(String, '/navigation/handoff_request', 10)
        self.create_subscription(String, '/navigation/handoff_ack', self._on_handoff_ack, 10)
        self._handoff_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._handoff_timer = self.create_timer(0.1, self._check_handoff, clock=self._handoff_clock)
        self._state = State.STANDBY
        self._battery_fault_active = False
        self._last_battery_time = self.get_clock().now()
        self._battery_timeout_sec = float(
            self.get_parameter("battery_timeout_sec").value
        )
        self._state_publisher = self.create_publisher(String, state_topic, 10)
        self._mission_state_publisher = self.create_publisher(
            String,
            mission_state_topic,
            MISSION_STATE_QOS,
        )
        self._return_event_publisher = self.create_publisher(
            String, return_event_topic, 10
        )
        self._command_subscription = self.create_subscription(
            String,
            command_topic,
            self._on_command,
            10,
        )
        self._battery_subscription = self.create_subscription(
            BatteryState,
            battery_topic,
            self._on_battery,
            qos_profile_sensor_data,
        )
        self._navigator = NavigateToPoseClient(
            self,
            navigate_action,
            enable_navigation_transmission=transmission_enabled,
            server_timeout_sec=server_timeout_sec,
            event_callback=self._publish_return_event,
        )

        self._pause_released = False
        self._paused_at = None
        self._automatic_pause = False
        self._low_battery_active = False
        self._recovery_publisher = self.create_publisher(String, '/g3/recovery_event', 30)
        self.declare_parameter('enable_fault_recovery', False)
        self.declare_parameter('recovery_stale_sec', 2.)
        self.declare_parameter('recovery_stable_sec', 3.)
        self.declare_parameter('recovery_timeout_sec', 60.)
        self.declare_parameter('recovery_scan_topic', '/robot1/scan')
        self.declare_parameter('recovery_map_frame', 'map')
        self.declare_parameter('recovery_robot_frame', 'base_link')
        self.declare_parameter('recovery_image_topics', [''])
        self._health = None
        if self.get_parameter('enable_fault_recovery').value:
            if not self._require_handoff:
                raise ValueError('Fault recovery requires require_navigation_handoff:=true')
            images = [v for v in self.get_parameter('recovery_image_topics').value if v]
            self._health = RecoveryHealth(['scan', 'localization']+images,
                stale_s=float(self.get_parameter('recovery_stale_sec').value),
                stable_s=float(self.get_parameter('recovery_stable_sec').value),
                timeout_s=float(self.get_parameter('recovery_timeout_sec').value))
            self._recovery_tf = Buffer()
            self._recovery_listener = TransformListener(self._recovery_tf, self)
            self.create_subscription(LaserScan, self.get_parameter('recovery_scan_topic').value,
                                     self._on_recovery_scan, qos_profile_sensor_data)
            for topic in images:
                self.create_subscription(Image, topic,
                    lambda msg, source=topic: self._on_recovery_image(source, msg), qos_profile_sensor_data)
        self.create_timer(.2, self._check_recovery, clock=self._handoff_clock)

        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        self.get_logger().info(
            f"G3 state machine ready in STANDBY (ROS_DOMAIN_ID={domain_id})"
        )
        self.get_logger().info(
            f"low-battery RETURN monitor: topic={battery_topic}, "
            f"threshold={self._low_battery_threshold:.3f}"
        )
        if transmission_enabled:
            self.get_logger().warning(
                "NavigateToPose transmission ENABLED; RETURN will send a real goal "
                f"to {navigate_action} after waiting up to {server_timeout_sec:.3f}s"
            )
        else:
            self.get_logger().warning(
                "NavigateToPose transmission disabled; RETURN is preview-only. "
                "Set enable_navigation_transmission:=true explicitly to send goals"
            )
        self.get_logger().warning(
            "ESTOP cancels G3 navigation goals but is not a physical robot "
            "emergency-stop command"
        )
        self.get_logger().info(f"RETURN events: topic={return_event_topic}")
        self.get_logger().info(
            "G2 mission-state bridge: "
            f"topic={mission_state_topic}, qos=transient_local"
        )
        self.get_logger().info(
            f"battery timeout watchdog: timeout={self._battery_timeout_sec:.1f}s"
        )
        if domain_id == "42":
            if transmission_enabled:
                self.get_logger().warning(
                    "Shared Domain 42 detected with navigation transmission ENABLED"
                )
            else:
                self.get_logger().warning(
                    "Shared Domain 42 detected; navigation remains preview-only"
                )
        self._battery_watchdog_timer = self.create_timer(
            5.0, self._check_battery_timeout
        )
        self._publish_state()
        self.declare_parameter('state_publish_period_sec', .5)
        period=float(self.get_parameter('state_publish_period_sec').value)
        if not math.isfinite(period) or period<=0:raise ValueError('State heartbeat period must be finite and positive')
        self.create_timer(period,self._publish_state,clock=self._handoff_clock)

    def _on_command(self, message: String) -> None:
        try:
            event = parse_event(message.data)
        except InvalidTransition as exc:
            self.get_logger().warning(str(exc))
            return

        if event is Event.RESET and self._state is State.ESTOP:
            block_reason = estop_reset_block_reason(
                battery_fault_active=self._battery_fault_active,
                navigation_active=self._navigator.has_active_goal,
            )
            if block_reason is not None:
                self.get_logger().error(f"ESTOP reset rejected: {block_reason}")
                return

        self._apply_event(event, source="command")

    def _on_battery(self, message: BatteryState) -> None:
        if not message.present:
            self._battery_fault_active = True
            self.get_logger().error(
                "电量错误：BatteryState.present=false；立即停止工作并进入 ESTOP"
            )
            self._apply_event(Event.ESTOP, source="battery_not_present")
            return

        normalized_percentage = normalize_battery_percentage(message.percentage)
        if normalized_percentage is None:
            self._battery_fault_active = True
            self.get_logger().error(
                "电量错误：BatteryState.percentage 必须是 0 到 1 之间的有限数值，"
                f"收到 {message.percentage!r}；立即停止工作并进入 ESTOP"
            )
            self._apply_event(Event.ESTOP, source="invalid_battery")
            return

        self._battery_fault_active = False
        self._low_battery_active = normalized_percentage < self._low_battery_threshold
        self._last_battery_time = self.get_clock().now()

        event = battery_return_event(
            self._state,
            normalized_percentage,
            self._low_battery_threshold,
        )
        if event is None:
            return

        self.get_logger().warning(
            f"low battery detected: {normalized_percentage:.3f} < "
            f"{self._low_battery_threshold:.3f}; requesting RETURN"
        )
        self._apply_event(event, source="battery")

    def _apply_event(self, event: Event, *, source: str) -> None:
        previous = self._state
        if event is Event.RETURN and previous is State.PAUSED and self._health is not None and not self._health.ready(time.monotonic()):
            self._apply_event(Event.ESTOP, source='return_health_fault')
            return
        if event is Event.RESUME:
            if (previous is not State.PAUSED or not self._pause_released or self._battery_fault_active
                or self._low_battery_active
                or (self.get_clock().now()-self._last_battery_time).nanoseconds*1e-9 > self._battery_timeout_sec
                or (self._health is not None and not self._health.ready(time.monotonic()))):
                self.get_logger().warning('RESUME rejected: cancellation or health recovery is incomplete')
                return
        if event is Event.EXPLORE and previous is State.STANDBY and self._health is not None:
            if not self._health.ready(time.monotonic()):
                self.get_logger().warning('EXPLORE rejected: health checks are not stable')
                return
        try:
            self._state = transition(previous, event)
        except InvalidTransition as exc:
            self.get_logger().warning(str(exc))
            return

        self.get_logger().info(
            f"state transition: {previous.value} --{event.value}--> "
            f"{self._state.value} (source={source})"
        )

        if self._state is State.ESTOP or (
            previous is State.RETURN and self._state is not State.RETURN
        ):
            if self._navigator.cancel_current_goal(reason=source):
                self.get_logger().warning(
                    "RETURN navigation cancellation requested before leaving "
                    f"{previous.value}"
                )

        if self._state not in (State.RETURN, State.PAUSED):
            self._handoff_pending = None
        self._publish_state()

        if self._state is State.PAUSED and previous is not State.PAUSED:
            self._paused_at = time.monotonic()
            self._pause_released = False
            self._automatic_pause = source == 'health_fault'
            self._recovery_event('paused', reason=source)
            token = uuid.uuid4().hex
            self._handoff_pending = (token, 'pause', time.monotonic()+self._handoff_timeout)
            self._handoff_publisher.publish(String(data=json.dumps({'token': token})))
        elif self._state is State.EXPLORE and previous is State.PAUSED:
            self._recovery_event('resumed', paused_seconds=time.monotonic()-self._paused_at)
            self._paused_at = None
        if self._state is State.RETURN and previous is not State.RETURN:
            self._start_return(source)
        elif self._state is State.ESTOP:
            self.get_logger().error(
                "ESTOP entered; G3 navigation cancellation is active, but the "
                "physical robot emergency-stop interface is not wired"
            )

    def _start_return(self, trigger: str) -> None:
        if self._require_handoff:
            token = uuid.uuid4().hex
            self._handoff_pending = (token, trigger, time.monotonic()+self._handoff_timeout)
            self._handoff_publisher.publish(String(data=json.dumps({'token': token})))
            self.get_logger().info('RETURN waiting for exploration navigation to release its goal')
            return
        self._send_return(trigger)

    def _on_handoff_ack(self, message: String) -> None:
        pending = self._handoff_pending
        if pending is None or self._state not in (State.RETURN, State.PAUSED):
            return
        try:
            ack = json.loads(message.data)
            if ack.get('token') != pending[0] or ack.get('released') is not True:
                return
        except (ValueError, AttributeError):
            return
        if time.monotonic() > pending[2]:
            self._check_handoff()
            return
        self._handoff_pending = None
        if self._state is State.PAUSED:
            self._pause_released = True
            self._recovery_event('navigation_released')
        else:
            self._send_return(pending[1])

    def _check_handoff(self) -> None:
        if self._handoff_pending is not None and time.monotonic() > self._handoff_pending[2]:
            self._handoff_pending = None
            self.get_logger().error('Navigation handoff timed out; RETURN goal was not sent')
            self._apply_event(Event.ESTOP, source='navigation_handoff_timeout')

    def _send_return(self, trigger: str) -> None:
        home_target = {
            "frame_id": str(self.get_parameter("home_frame").value),
            "x": float(self.get_parameter("home_x").value),
            "y": float(self.get_parameter("home_y").value),
            "yaw": float(self.get_parameter("home_yaw").value),
        }
        goal = self._navigator.build_goal(
            frame_id=home_target["frame_id"],
            x=home_target["x"],
            y=home_target["y"],
            yaw=home_target["yaw"],
        )
        self._navigator.request_return(
            goal,
            trigger=trigger,
            home_target=home_target,
        )

    def _publish_return_event(self, event: dict) -> None:
        message = String()
        message.data = json.dumps(
            event,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        self._return_event_publisher.publish(message)

        status = event["status"]
        goal_id = event.get("goal_id") or "none"
        if status in {
            "server_unavailable",
            "send_error",
            "goal_response_error",
            "rejected",
            "aborted",
            "canceled",
            "result_error",
            "cancel_error",
            "cancel_rejected",
            "unknown",
        }:
            self.get_logger().error(
                f"RETURN navigation event: status={status}, goal_id={goal_id}, "
                f"attempt_id={event['attempt_id']}"
            )
        else:
            self.get_logger().info(
                f"RETURN navigation event: status={status}, goal_id={goal_id}, "
                f"attempt_id={event['attempt_id']}"
            )

        if self._state is State.RETURN:
            if status == "succeeded":
                self.get_logger().info(
                    "RETURN succeeded; transitioning to STANDBY"
                )
                self._apply_event(Event.STANDBY, source="return_succeeded")
            elif status in {
                "aborted", "canceled", "rejected",
                "server_unavailable", "send_error", "goal_response_error",
                "result_error", "cancel_error", "cancel_rejected", "unknown",
            }:
                self.get_logger().error(
                    f"RETURN failed (status={status}); transitioning to ESTOP"
                )
                self._apply_event(Event.ESTOP, source=f"return_failed:{status}")

    def _check_battery_timeout(self) -> None:
        if self._state not in (State.EXPLORE, State.PAUSED, State.RETURN):
            return
        if self._last_battery_time is None:
            return
        elapsed_sec = (
            (self.get_clock().now() - self._last_battery_time).nanoseconds * 1e-9
        )
        if elapsed_sec > self._battery_timeout_sec:
            self.get_logger().error(
                f"电量话题超时 {elapsed_sec:.1f}s > "
                f"{self._battery_timeout_sec:.1f}s; 进入 ESTOP"
            )
            self._apply_event(Event.ESTOP, source="battery_timeout")

    def _recovery_event(self, status, **extra):
        self._recovery_publisher.publish(String(data=json.dumps(dict(status=status,
            state=self._state.value, monotonic_time=time.monotonic(),
            stamp_ns=self.get_clock().now().nanoseconds, **extra))))

    def _fresh_stamp(self, stamp):
        age = (self.get_clock().now().nanoseconds - stamp.sec*10**9 - stamp.nanosec)*1e-9
        return 0 <= age <= self._health.stale_s

    def _on_recovery_scan(self, msg):
        if (self._fresh_stamp(msg.header.stamp) and math.isfinite(msg.range_min)
            and math.isfinite(msg.range_max) and 0 <= msg.range_min < msg.range_max
            and any(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges)):
            self._health.observe('scan', time.monotonic(), msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)

    def _on_recovery_image(self, source, msg):
        bytes_per_pixel = {'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1,
                           'mono16':2,'16UC1':2,'32FC1':4}.get(msg.encoding)
        if (bytes_per_pixel is not None and self._fresh_stamp(msg.header.stamp)
            and msg.width > 0 and msg.height > 0 and msg.step >= msg.width*bytes_per_pixel
            and len(msg.data) >= msg.step*msg.height):
            self._health.observe(source, time.monotonic(), msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)

    def _check_recovery(self):
        health = self._health
        if health is None:
            return
        now = time.monotonic()
        try:
            tf = self._recovery_tf.lookup_transform(
                self.get_parameter('recovery_map_frame').value,
                self.get_parameter('recovery_robot_frame').value, Time())
            p, q = tf.transform.translation, tf.transform.rotation
            values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
            if (self._fresh_stamp(tf.header.stamp) and all(math.isfinite(v) for v in values)
                and abs(sum(v*v for v in values[3:])-1.) < .01):
                health.observe('localization', now, tf.header.stamp.sec*10**9+tf.header.stamp.nanosec)
        except TransformException:
            pass
        ready = health.ready(now)
        missing = health.missing(now)
        if self._state is State.EXPLORE and missing:
            self._recovery_event('fault', missing=missing)
            self._apply_event(Event.PAUSE, source='health_fault')
        elif self._state is State.RETURN and missing:
            self._recovery_event('return_fault', missing=missing)
            self._apply_event(Event.ESTOP, source='return_health_fault')
        elif self._state is State.PAUSED:
            if now-self._paused_at >= health.timeout_s:
                self._recovery_event('recovery_timeout', missing=missing)
                self._apply_event(Event.ESTOP, source='recovery_timeout')
            elif ready and self._pause_released and self._automatic_pause:
                self._apply_event(Event.RESUME, source='health_recovered')

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.value
        self._state_publisher.publish(message)

        mission_message = String()
        mission_message.data = mission_state_value(self._state)
        self._mission_state_publisher.publish(mission_message)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = G3StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
