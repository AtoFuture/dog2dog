"""ROS 2 wrapper for the G3 state machine and RETURN navigation flow."""

import json
import os
import math
import time
import uuid
from dataclasses import replace
from typing import Optional, Sequence

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan, Image
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from g3_integration_eval.recovery import RecoveryHealth
from g3_integration_eval.return_localization import (
    HomeReference,
    LocalizationIntegrity,
    LocalizationSample,
    Pose2,
    compose_pose,
    dynamic_home_target,
    match_home_scan,
    pose_dict,
    pose_error,
    relative_pose,
    scan_corrected_home_target,
    scan_to_points,
    validate_return_localization_config,
)
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

EXPLORE_HOME_CAPTURE_TIMEOUT_S = 30.0
RETURN_GOAL_LOCALIZATION_TIMEOUT_S = 30.0


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
        self.declare_parameter("return_bt_path", "")
        self.declare_parameter("battery_timeout_sec", 30.0)
        self.declare_parameter("odometry_topic", "/robot1/odometry/filtered")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("localization_status_topic", "/g3/localization_status")
        self.declare_parameter("return_localization_config", "")

        raw_return_config = str(
            self.get_parameter("return_localization_config").value
        ).strip()
        try:
            return_config_value = (
                json.loads(raw_return_config) if raw_return_config else {}
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("return_localization_config must be valid JSON") from exc
        self._return_localization_config = validate_return_localization_config(
            return_config_value
        )
        self._return_localization_enabled = self._return_localization_config["enabled"]
        self._localization_integrity = (
            LocalizationIntegrity(self._return_localization_config)
            if self._return_localization_enabled else None
        )
        self._latest_odometry = None
        self._latest_scan_points = None
        self._latest_scan_stamp_ns = None
        self._home_reference = None
        self._explore_capture_pending = None
        self._return_goal_pending = None
        self._return_logical_id = None
        self._return_trigger = None
        self._return_correction_count = 0
        self._return_active_target = None
        self._return_pending_success = None
        self._return_verification_started = None
        self._return_stable_since = None
        self._return_last_scan_match_stamp = None
        self._return_last_scan_match = None
        self._return_replan_pending = False
        self._return_replan_started = None
        self._last_integrity_stamp = None
        self._last_integrity_result = None
        self._integrity_healthy_since = None

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
        self._exploration_session = None
        self._exploration_result = None
        self._exploration_session_pub = self.create_publisher(String, '/g3/exploration_session', MISSION_STATE_QOS)
        self._exploration_result_pub = self.create_publisher(String, '/g3/exploration_result', MISSION_STATE_QOS)
        self.declare_parameter('enable_exploration_completion', False)
        self._enable_exploration_completion = bool(self.get_parameter('enable_exploration_completion').value)
        self.create_subscription(String, '/exploration/completion', self._on_exploration_completion, MISSION_STATE_QOS)
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
        self._localization_status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("localization_status_topic").value),
            30,
        )
        self._command_result_publisher = self.create_publisher(String, command_topic + '/result', 20)
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
            for topic in images:
                self.create_subscription(Image, topic,
                    lambda msg, source=topic: self._on_recovery_image(source, msg), qos_profile_sensor_data)
        self._recovery_tf = None
        if self._health is not None or self._return_localization_enabled:
            self._recovery_tf = Buffer()
            self._recovery_listener = TransformListener(self._recovery_tf, self)
            self.create_subscription(
                LaserScan,
                self.get_parameter('recovery_scan_topic').value,
                self._on_recovery_scan,
                qos_profile_sensor_data,
            )
        if self._return_localization_enabled:
            self.create_subscription(
                Odometry,
                str(self.get_parameter("odometry_topic").value),
                self._on_odometry,
                qos_profile_sensor_data,
            )
        self.create_timer(.2, self._check_recovery, clock=self._handoff_clock)
        self.create_timer(.2, self._check_return_localization, clock=self._handoff_clock)

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
        if self._return_localization_enabled:
            self.get_logger().info(
                "RETURN localization guard enabled: dynamic odometry home, "
                "LiDAR home verification, and bounded correction"
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

    def _on_exploration_completion(self, message: String) -> None:
        if not self._enable_exploration_completion or self._state is not State.EXPLORE:
            return
        try:
            result = json.loads(message.data)
            reasons = {'no_frontiers': True, 'time_budget': False, 'coverage_stalled': False,
                       'candidates_unavailable': False, 'navigation_failures': False}
            if (not isinstance(result, dict) or not self._exploration_session
                or result.get('session_id') != self._exploration_session
                or result.get('reason') not in reasons
                or result.get('completed') is not reasons[result['reason']]):
                return
        except (ValueError, TypeError):
            return
        self._exploration_result = result
        self._exploration_result_pub.publish(String(data=json.dumps(result)))
        self._apply_event(Event.RETURN, source='exploration:'+result['reason'])

    def _on_command(self, message: String) -> None:
        # Keep legacy plain strings; web requests carry an ID for an exact receipt.
        request_id = None
        raw = message.data
        try:
            if raw.lstrip().startswith('{'):
                envelope = json.loads(raw)
                request_id = envelope.get('request_id')
                if not isinstance(request_id, str) or not 1 <= len(request_id) <= 80:
                    raise ValueError('invalid request_id')
                raw = envelope.get('command')
                if not isinstance(raw, str):
                    raise ValueError('invalid command')
            event = parse_event(raw)
        except (InvalidTransition, ValueError, TypeError, AttributeError) as exc:
            self.get_logger().warning(str(exc))
            return

        def receipt(accepted, reason=''):
            if request_id is not None:
                self._command_result_publisher.publish(String(data=json.dumps({
                    'request_id': request_id, 'command': event.value.upper(),
                    'accepted': accepted, 'state': mission_state_value(self._state), 'reason': reason,
                })))

        if event is Event.RESET and self._state is State.ESTOP:
            block_reason = estop_reset_block_reason(
                battery_fault_active=self._battery_fault_active,
                navigation_active=self._navigator.has_active_goal,
            )
            if block_reason is not None:
                self.get_logger().error(f"ESTOP reset rejected: {block_reason}")
                receipt(False, block_reason)
                return
        try:
            expected = transition(self._state, event)
        except InvalidTransition as exc:
            self.get_logger().warning(str(exc))
            receipt(False, str(exc))
            return
        self._apply_event(event, source="command")
        accepted = self._state is expected
        receipt(accepted, '' if accepted else '健康检查或导航释放条件未满足')

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
        if event is not Event.EXPLORE:
            # A queued start must not survive a user stop, even when ESTOP is
            # reset before the next localization timer callback runs.
            self._explore_capture_pending = None
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
        if (event is Event.EXPLORE and previous is State.STANDBY
                and self._return_localization_enabled):
            self._return_logical_id = None
            if not self._capture_home_reference():
                if self._explore_capture_pending is None:
                    self._explore_capture_pending = (
                        time.monotonic() + EXPLORE_HOME_CAPTURE_TIMEOUT_S,
                        source,
                    )
                self.get_logger().warning(
                    'EXPLORE deferred: waiting for a fresh odometry pose and LiDAR home keyframe'
                )
                return
        try:
            self._state = transition(previous, event)
        except InvalidTransition as exc:
            self.get_logger().warning(str(exc))
            return

        if self._state is State.EXPLORE and previous is State.STANDBY:
            self._exploration_session = uuid.uuid4().hex
            self._exploration_result = None
            if self._localization_integrity is not None:
                self._localization_integrity.reset()
                self._last_integrity_stamp = None
                self._last_integrity_result = None
                self._integrity_healthy_since = None
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
        if previous is State.RETURN and self._state is not State.RETURN:
            self._return_pending_success = None
            self._return_verification_started = None
            self._return_stable_since = None
            self._return_replan_pending = False
            self._return_replan_started = None
            self._return_goal_pending = None
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
        self._return_logical_id = uuid.uuid4().hex
        self._return_trigger = trigger
        self._return_correction_count = 0
        self._return_active_target = None
        self._return_pending_success = None
        self._return_verification_started = None
        self._return_stable_since = None
        self._return_last_scan_match_stamp = None
        self._return_last_scan_match = None
        self._return_replan_pending = False
        self._return_replan_started = None
        self._return_goal_pending = None
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

    def _send_return(
        self,
        trigger: str,
        *,
        target: Optional[Pose2] = None,
        target_source: str = "configured_map_home",
    ) -> None:
        if self._return_localization_enabled:
            if self._home_reference is None:
                self._fail_return_without_goal("home_reference_missing")
                return
            sample = self._current_localization()
            if sample is None:
                if self._return_goal_pending is None:
                    self._return_goal_pending = (
                        trigger,
                        target,
                        target_source,
                        time.monotonic() + RETURN_GOAL_LOCALIZATION_TIMEOUT_S,
                    )
                self.get_logger().warning(
                    'RETURN goal deferred: waiting for a fresh localization sample'
                )
                return
            if target is None:
                if target_source == "configured_map_home":
                    target = Pose2(
                        x=float(self.get_parameter("home_x").value),
                        y=float(self.get_parameter("home_y").value),
                        yaw=float(self.get_parameter("home_yaw").value),
                    )
                else:
                    target = dynamic_home_target(
                        sample.map_to_odom,
                        self._home_reference.odom_pose,
                    )
            home_target = {
                "frame_id": str(self.get_parameter("recovery_map_frame").value),
                **pose_dict(target),
                "target_source": target_source,
                "logical_return_id": self._return_logical_id,
                "correction_index": self._return_correction_count,
            }
            self._return_active_target = target
            self._localization_event(
                "return_goal_prepared",
                target=pose_dict(target),
                target_source=target_source,
                correction_index=self._return_correction_count,
                map_to_odom=pose_dict(sample.map_to_odom),
            )
        else:
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
        return_bt_path = str(self.get_parameter("return_bt_path").value)
        if return_bt_path:
            goal.behavior_tree = return_bt_path
        self._navigator.request_return(
            goal,
            trigger=trigger,
            home_target=home_target,
        )

    def _publish_return_event(self, event: dict) -> None:
        event = dict(event)
        home_target = event.get("home_target")
        if isinstance(home_target, dict):
            logical_id = home_target.get("logical_return_id")
            if logical_id:
                event["logical_return_id"] = logical_id
            if "correction_index" in home_target:
                event["correction_index"] = home_target["correction_index"]

        status = event["status"]
        if (self._return_localization_enabled and self._state is State.RETURN
                and self._return_replan_pending and status == "canceled"):
            if (self._return_correction_count
                    >= self._return_localization_config["max_corrections"]):
                failed = dict(event)
                failed.update(
                    status="localization_failed",
                    timestamp=self._timestamp(),
                    detail="localization changed during RETURN and the replan budget was exhausted",
                    verification=self._verification_payload(
                        reason="localization_replan_budget_exhausted"
                    ),
                )
                self._return_replan_pending = False
                self._emit_return_event(failed)
                self._localization_event(
                    "return_failed",
                    attempt_id=event.get("attempt_id"),
                    reason="localization_replan_budget_exhausted",
                )
                self._apply_event(
                    Event.ESTOP,
                    source="return_localization_failed:replan_budget",
                )
                return
            rejected = dict(event)
            rejected.update(
                status="localization_rejected",
                timestamp=self._timestamp(),
                detail="RETURN goal canceled because localization changed",
                verification=self._verification_payload(
                    reason="localization_changed_during_return"
                ),
            )
            self._return_correction_count += 1
            self._emit_return_event(rejected)
            return
        if (self._return_localization_enabled and self._return_replan_pending
                and status == "succeeded"):
            self._return_replan_pending = False
            self._return_replan_started = None
        if (self._return_localization_enabled and self._state is State.RETURN
                and status == "succeeded"):
            pending = dict(event)
            pending["status"] = "nav_succeeded_pending_verification"
            pending["nav2_status"] = "succeeded"
            self._emit_return_event(pending)
            self._return_pending_success = event
            self._return_verification_started = time.monotonic()
            self._return_stable_since = None
            self._return_last_scan_match_stamp = None
            self._return_last_scan_match = None
            self._localization_event(
                "verification_started",
                attempt_id=event.get("attempt_id"),
                goal_id=event.get("goal_id"),
                correction_index=self._return_correction_count,
            )
            return

        self._emit_return_event(event)

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

    def _emit_return_event(self, event: dict) -> None:
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
            "localization_rejected",
            "localization_failed",
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

    def _fail_return_without_goal(self, reason: str) -> None:
        event = {
            "attempt_id": "localization-" + uuid.uuid4().hex,
            "logical_return_id": self._return_logical_id,
            "trigger": self._return_trigger,
            "status": "localization_failed",
            "detail": reason,
            "timestamp": self._timestamp(),
        }
        self._emit_return_event(event)
        self._localization_event("return_failed", reason=reason)
        if self._state is State.RETURN:
            self._apply_event(Event.ESTOP, source="return_localization_failed:" + reason)

    def _timestamp(self) -> dict:
        stamp = self.get_clock().now().to_msg()
        return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}

    def _localization_event(self, status: str, **extra) -> None:
        payload = {
            "status": status,
            "state": self._state.value,
            "logical_return_id": self._return_logical_id,
            "timestamp": self._timestamp(),
            **extra,
        }
        self._localization_status_publisher.publish(String(data=json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )))

    @staticmethod
    def _yaw(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    @classmethod
    def _pose_from_transform(cls, transform) -> Pose2:
        translation = transform.translation
        return Pose2(
            float(translation.x),
            float(translation.y),
            cls._yaw(transform.rotation),
        )

    @classmethod
    def _pose_from_odometry(cls, message: Odometry) -> Pose2:
        position = message.pose.pose.position
        return Pose2(
            float(position.x),
            float(position.y),
            cls._yaw(message.pose.pose.orientation),
        )

    @staticmethod
    def _valid_quaternion(quaternion) -> bool:
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        return (
            all(math.isfinite(value) for value in values)
            and abs(sum(value * value for value in values) - 1.0) < 0.01
        )

    def _on_odometry(self, message: Odometry) -> None:
        expected_frame = str(self.get_parameter("odom_frame").value).lstrip("/")
        if message.header.frame_id.lstrip("/") != expected_frame:
            return
        position = message.pose.pose.position
        twist = message.twist.twist
        values = (
            position.x, position.y, position.z,
            twist.linear.x, twist.linear.y, twist.angular.z,
        )
        if (not all(math.isfinite(value) for value in values)
                or not self._valid_quaternion(message.pose.pose.orientation)
                or not self._fresh_stamp(
                    message.header.stamp,
                    self._return_localization_config["stale_s"],
                )):
            return
        self._latest_odometry = {
            "pose": self._pose_from_odometry(message),
            "linear_speed": math.hypot(twist.linear.x, twist.linear.y),
            "angular_speed": abs(twist.angular.z),
            "stamp_ns": (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            ),
        }

    def _current_localization(self) -> Optional[LocalizationSample]:
        odometry = self._latest_odometry
        if odometry is None or self._recovery_tf is None:
            return None
        now_ns = self.get_clock().now().nanoseconds
        age_s = (now_ns - odometry["stamp_ns"]) * 1e-9
        if not 0 <= age_s <= self._return_localization_config["stale_s"]:
            return None
        try:
            transform = self._recovery_tf.lookup_transform(
                str(self.get_parameter("recovery_map_frame").value),
                str(self.get_parameter("odom_frame").value),
                # SLAM's latest TF can be future-dated. Compose the frame
                # transform at the measured odometry time, not at its latest
                # broadcast time; freshness still rejects stale/future data.
                Time(nanoseconds=odometry["stamp_ns"]),
            )
        except TransformException:
            return None
        if (not self._fresh_stamp(
                transform.header.stamp,
                self._return_localization_config["stale_s"],
        ) or not self._valid_quaternion(transform.transform.rotation)):
            return None
        translation = transform.transform.translation
        if not all(math.isfinite(value) for value in (
                translation.x, translation.y, translation.z)):
            return None
        map_to_odom = self._pose_from_transform(transform.transform)
        odom_pose = odometry["pose"]
        return LocalizationSample(
            map_pose=compose_pose(map_to_odom, odom_pose),
            odom_pose=odom_pose,
            map_to_odom=map_to_odom,
            linear_speed=odometry["linear_speed"],
            angular_speed=odometry["angular_speed"],
            stamp_ns=odometry["stamp_ns"],
        )

    def _capture_home_reference(self) -> bool:
        sample = self._current_localization()
        points = self._latest_scan_points
        scan_stamp_ns = self._latest_scan_stamp_ns
        config = self._return_localization_config
        if sample is None:
            return False
        if config["scan_match_required"]:
            if points is None or scan_stamp_ns is None:
                return False
            age_s = (self.get_clock().now().nanoseconds - scan_stamp_ns) * 1e-9
            if (not 0 <= age_s <= config["stale_s"]
                    or len(points) < config["scan_match_min_points"]):
                return False
        else:
            points = np.empty((0, 2), dtype=float)
        self._home_reference = HomeReference(
            map_pose=sample.map_pose,
            odom_pose=sample.odom_pose,
            map_to_odom=sample.map_to_odom,
            scan_points=np.array(points, dtype=float, copy=True),
            stamp_ns=max(sample.stamp_ns, scan_stamp_ns or 0),
        )
        self._localization_event(
            "home_reference_captured",
            map_pose=pose_dict(sample.map_pose),
            odom_pose=pose_dict(sample.odom_pose),
            scan_points=len(points),
        )
        return True

    def _observe_localization_integrity(
        self,
        sample: LocalizationSample,
        now: float,
    ):
        if self._localization_integrity is None:
            return None
        if sample.stamp_ns == self._last_integrity_stamp:
            return self._last_integrity_result
        previous = self._last_integrity_result
        result = self._localization_integrity.observe(sample)
        self._last_integrity_stamp = sample.stamp_ns
        self._last_integrity_result = result
        if result.healthy:
            if self._integrity_healthy_since is None:
                self._integrity_healthy_since = now
            if previous is not None and not previous.healthy:
                self._localization_event("integrity_recovered")
        else:
            self._integrity_healthy_since = None
            self._localization_event(
                "integrity_fault",
                reason=result.reason,
                map_odom_step_m=result.map_odom_step_m,
                map_odom_step_yaw=result.map_odom_step_yaw,
                stationary_drift_m=result.stationary_drift_m,
                stationary_drift_yaw=result.stationary_drift_yaw,
            )
            if self._state is State.EXPLORE and self._health is not None:
                self._health.invalidate("localization")
                self._recovery_event(
                    "fault",
                    missing=["localization_consistency"],
                    reason=result.reason,
                )
                self._apply_event(Event.PAUSE, source="health_fault")
            elif (self._state is State.RETURN
                  and self._return_pending_success is None
                  and not self._return_replan_pending
                  and self._navigator.has_active_goal):
                self._return_replan_pending = True
                self._return_replan_started = now
                self._localization_event(
                    "return_replan_requested",
                    reason=result.reason,
                    correction_index=self._return_correction_count,
                )
                if not self._navigator.cancel_current_goal(
                        reason="localization_drift_replan"):
                    self._return_replan_pending = False
        return result

    def _check_return_localization(self) -> None:
        if not self._return_localization_enabled:
            return
        now = time.monotonic()
        sample = self._current_localization()
        integrity = (
            self._observe_localization_integrity(sample, now)
            if sample is not None else None
        )
        pending = self._explore_capture_pending
        if pending is not None:
            deadline, pending_source = pending
            if self._state is not State.STANDBY:
                self._explore_capture_pending = None
            elif now >= deadline:
                self._explore_capture_pending = None
                self.get_logger().warning(
                    'EXPLORE rejected: home reference capture timed out'
                )
            elif self._capture_home_reference():
                self._explore_capture_pending = None
                self.get_logger().info(
                    'EXPLORE resumed: home reference captured'
                )
                self._apply_event(Event.EXPLORE, source=pending_source)
            return
        goal_pending = self._return_goal_pending
        if goal_pending is not None:
            goal_trigger, goal_target, goal_source, goal_deadline = goal_pending
            if self._state is not State.RETURN:
                self._return_goal_pending = None
            elif now >= goal_deadline:
                self._return_goal_pending = None
                self._fail_return_without_goal("localization_unavailable_before_goal")
            elif self._current_localization() is not None:
                self._return_goal_pending = None
                self.get_logger().info(
                    'RETURN goal resumed: fresh localization sample available'
                )
                self._send_return(
                    goal_trigger,
                    target=goal_target,
                    target_source=goal_source,
                )
            return
        if self._state is not State.RETURN:
            return
        if self._return_replan_pending and self._return_pending_success is None:
            config = self._return_localization_config
            recovered = (
                sample is not None
                and integrity is not None
                and integrity.healthy
                and self._integrity_healthy_since is not None
                and now - self._integrity_healthy_since >= config["stable_s"]
                and not self._navigator.has_active_goal
            )
            if recovered:
                self._return_replan_pending = False
                self._return_replan_started = None
                self._localization_event(
                    "return_replan_started",
                    correction_index=self._return_correction_count,
                )
                self._send_return(
                    self._return_trigger,
                    target_source="localization_rebased_home",
                )
            elif (self._return_replan_started is not None
                  and now - self._return_replan_started
                  >= config["verification_timeout_s"]):
                self._return_replan_pending = False
                self._return_replan_started = None
                self._fail_return_without_goal("localization_replan_timeout")
            return
        if self._return_pending_success is None:
            return

        config = self._return_localization_config
        elapsed = now - self._return_verification_started
        if sample is None:
            self._return_stable_since = None
            if elapsed >= config["verification_timeout_s"]:
                self._fail_pending_return("localization_stale")
            return

        stopped = (
            sample.linear_speed <= config["stationary_linear_mps"]
            and sample.angular_speed <= config["stationary_angular_radps"]
        )
        map_error_m, map_error_yaw = pose_error(
            self._return_active_target,
            sample.map_pose,
        )
        odom_error_m, odom_error_yaw = pose_error(
            self._home_reference.odom_pose,
            sample.odom_pose,
        )

        scan_match = None
        scan_fresh = False
        if config["scan_match_required"]:
            scan_stamp_ns = self._latest_scan_stamp_ns
            if scan_stamp_ns is not None:
                scan_age_s = (self.get_clock().now().nanoseconds - scan_stamp_ns) * 1e-9
                scan_fresh = 0 <= scan_age_s <= config["stale_s"]
            if (scan_fresh and scan_stamp_ns != self._return_last_scan_match_stamp
                    and self._latest_scan_points is not None):
                initial = relative_pose(
                    self._home_reference.odom_pose,
                    sample.odom_pose,
                )
                self._return_last_scan_match = match_home_scan(
                    self._home_reference.scan_points,
                    self._latest_scan_points,
                    initial,
                    config,
                )
                self._return_last_scan_match_stamp = scan_stamp_ns
            scan_match = self._return_last_scan_match if scan_fresh else None

        scan_ok = not config["scan_match_required"]
        scan_error_m = scan_error_yaw = 0.0
        if scan_match is not None and scan_match.valid:
            scan_error_m = math.hypot(
                scan_match.current_in_home.x,
                scan_match.current_in_home.y,
            )
            scan_error_yaw = abs(scan_match.current_in_home.yaw)
            scan_ok = (
                scan_error_m <= config["position_tolerance_m"]
                and scan_error_yaw <= config["yaw_tolerance_rad"]
            )
            if stopped and not scan_ok:
                self._start_scan_correction(sample, scan_match)
                return

        if (scan_ok and config["scan_match_required"]
                and (odom_error_m > config["position_tolerance_m"]
                     or odom_error_yaw > config["yaw_tolerance_rad"])):
            old_error = {"position_m": odom_error_m, "yaw_rad": odom_error_yaw}
            self._home_reference = replace(
                self._home_reference,
                odom_pose=sample.odom_pose,
            )
            odom_error_m = odom_error_yaw = 0.0
            self._localization_event(
                "odometry_home_rebased",
                reason="lidar_home_match",
                prior_odom_error=old_error,
            )

        if (scan_ok and config["scan_match_required"]
                and (map_error_m > config["position_tolerance_m"]
                     or map_error_yaw > config["yaw_tolerance_rad"])):
            old_error = {"position_m": map_error_m, "yaw_rad": map_error_yaw}
            self._return_active_target = sample.map_pose
            self._home_reference = replace(
                self._home_reference,
                map_pose=sample.map_pose,
                map_to_odom=sample.map_to_odom,
            )
            map_error_m = map_error_yaw = 0.0
            self._localization_event(
                "map_home_rebased",
                reason="lidar_home_match",
                prior_map_target_error=old_error,
                map_pose=pose_dict(sample.map_pose),
            )

        integrity_ok = (
            integrity is not None
            and integrity.healthy
            and self._integrity_healthy_since is not None
            and now - self._integrity_healthy_since >= config["stable_s"]
        )
        valid = (
            stopped
            and integrity_ok
            and map_error_m <= config["position_tolerance_m"]
            and map_error_yaw <= config["yaw_tolerance_rad"]
            and odom_error_m <= config["position_tolerance_m"]
            and odom_error_yaw <= config["yaw_tolerance_rad"]
            and scan_ok
        )
        if valid:
            if self._return_stable_since is None:
                self._return_stable_since = now
            elif now - self._return_stable_since >= config["stable_s"]:
                self._complete_verified_return(
                    map_error_m,
                    map_error_yaw,
                    odom_error_m,
                    odom_error_yaw,
                    scan_match,
                )
                return
        else:
            self._return_stable_since = None

        if elapsed >= config["verification_timeout_s"]:
            if scan_match is None:
                reason = "home_scan_stale_or_missing"
            elif not scan_match.valid:
                reason = "home_scan_" + scan_match.reason
            elif not integrity_ok:
                reason = "localization_integrity_unstable"
            elif not stopped:
                reason = "robot_not_stationary"
            else:
                reason = "home_pose_not_stable"
            self._fail_pending_return(reason)

    def _start_scan_correction(self, sample, scan_match) -> None:
        config = self._return_localization_config
        correction_m = math.hypot(
            scan_match.current_in_home.x,
            scan_match.current_in_home.y,
        )
        correction_yaw = abs(scan_match.current_in_home.yaw)
        if (self._return_correction_count >= config["max_corrections"]
                or correction_m > config["max_scan_correction_m"]
                or correction_yaw > config["max_scan_correction_yaw_rad"]):
            self._fail_pending_return("scan_correction_out_of_bounds")
            return
        target = scan_corrected_home_target(sample.map_pose, scan_match)
        event = dict(self._return_pending_success)
        event.update(
            status="localization_rejected",
            timestamp=self._timestamp(),
            detail="Nav2 reached its map goal but the LiDAR home keyframe did not match",
            verification=self._verification_payload(
                scan_match=scan_match,
                scan_error_m=correction_m,
                scan_error_yaw=correction_yaw,
            ),
        )
        self._emit_return_event(event)
        self._localization_event(
            "correction_requested",
            attempt_id=event.get("attempt_id"),
            correction_m=correction_m,
            correction_yaw=correction_yaw,
            corrected_target=pose_dict(target),
        )
        self._return_pending_success = None
        self._return_stable_since = None
        self._return_correction_count += 1
        self._send_return(
            self._return_trigger,
            target=target,
            target_source="lidar_home_correction",
        )

    def _verification_payload(
        self,
        *,
        scan_match=None,
        scan_error_m=None,
        scan_error_yaw=None,
        **extra,
    ) -> dict:
        payload = {
            "method": "dynamic_map_odom_and_lidar_home_keyframe",
            "stable_s": self._return_localization_config["stable_s"],
            "correction_count": self._return_correction_count,
            **extra,
        }
        if scan_match is not None:
            payload["scan_match"] = {
                "valid": scan_match.valid,
                "reason": scan_match.reason,
                "rmse_m": scan_match.rmse_m,
                "inlier_ratio": scan_match.inlier_ratio,
                "inlier_count": scan_match.inlier_count,
                "iterations": scan_match.iterations,
                "position_error_m": scan_error_m,
                "yaw_error_rad": scan_error_yaw,
            }
        return payload

    def _complete_verified_return(
        self,
        map_error_m,
        map_error_yaw,
        odom_error_m,
        odom_error_yaw,
        scan_match,
    ) -> None:
        event = dict(self._return_pending_success)
        scan_error_m = (
            math.hypot(scan_match.current_in_home.x, scan_match.current_in_home.y)
            if scan_match is not None else None
        )
        scan_error_yaw = (
            abs(scan_match.current_in_home.yaw)
            if scan_match is not None else None
        )
        event.update(
            status="succeeded",
            timestamp=self._timestamp(),
            verification=self._verification_payload(
                scan_match=scan_match,
                scan_error_m=scan_error_m,
                scan_error_yaw=scan_error_yaw,
                verified_map_home=pose_dict(self._return_active_target),
                map_error_m=map_error_m,
                map_error_yaw=map_error_yaw,
                odom_error_m=odom_error_m,
                odom_error_yaw=odom_error_yaw,
            ),
        )
        self._return_pending_success = None
        self._emit_return_event(event)
        self._localization_event(
            "return_verified",
            attempt_id=event.get("attempt_id"),
            goal_id=event.get("goal_id"),
            verification=event["verification"],
        )
        self.get_logger().info(
            "RETURN passed stable map, odometry, and LiDAR verification; "
            "transitioning to STANDBY"
        )
        self._apply_event(Event.STANDBY, source="return_verified")

    def _fail_pending_return(self, reason: str) -> None:
        event = dict(self._return_pending_success)
        event.update(
            status="localization_failed",
            timestamp=self._timestamp(),
            detail=reason,
            verification=self._verification_payload(reason=reason),
        )
        self._return_pending_success = None
        self._emit_return_event(event)
        self._localization_event(
            "return_failed",
            attempt_id=event.get("attempt_id"),
            reason=reason,
        )
        if self._state is State.RETURN:
            self._apply_event(Event.ESTOP, source="return_localization_failed:" + reason)

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

    def _fresh_stamp(self, stamp, stale_s=None):
        age = (self.get_clock().now().nanoseconds - stamp.sec*10**9 - stamp.nanosec)*1e-9
        if stale_s is None:
            if self._health is None:
                return False
            stale_s = self._health.stale_s
        return 0 <= age <= stale_s

    def _on_recovery_scan(self, msg):
        stale_s = (
            self._return_localization_config["stale_s"]
            if self._return_localization_enabled
            else self._health.stale_s
        )
        valid = (
            self._fresh_stamp(msg.header.stamp, stale_s)
            and math.isfinite(msg.range_min)
            and math.isfinite(msg.range_max)
            and 0 <= msg.range_min < msg.range_max
            and any(
                math.isfinite(value) and msg.range_min <= value <= msg.range_max
                for value in msg.ranges
            )
        )
        if not valid:
            return
        stamp_ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self._health is not None:
            self._health.observe('scan', time.monotonic(), stamp_ns)
        if not self._return_localization_enabled or self._recovery_tf is None:
            return
        if (not math.isfinite(msg.angle_min)
                or not math.isfinite(msg.angle_increment)
                or msg.angle_increment == 0):
            return
        base_frame = str(self.get_parameter("recovery_robot_frame").value)
        scan_frame = msg.header.frame_id
        try:
            if scan_frame.lstrip("/") == base_frame.lstrip("/"):
                base_from_sensor = Pose2(0.0, 0.0, 0.0)
            else:
                transform = self._recovery_tf.lookup_transform(
                    base_frame,
                    scan_frame,
                    Time.from_msg(msg.header.stamp),
                )
                if not self._valid_quaternion(transform.transform.rotation):
                    return
                base_from_sensor = self._pose_from_transform(transform.transform)
            points = scan_to_points(
                msg.ranges,
                angle_min=float(msg.angle_min),
                angle_increment=float(msg.angle_increment),
                range_min=float(msg.range_min),
                range_max=float(msg.range_max),
                base_from_sensor=base_from_sensor,
                max_points=self._return_localization_config["scan_match_max_points"],
            )
        except (TransformException, ValueError, TypeError, FloatingPointError):
            return
        if len(points) >= self._return_localization_config["scan_match_min_points"]:
            self._latest_scan_points = points
            self._latest_scan_stamp_ns = stamp_ns

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
        self._exploration_session_pub.publish(String(data=json.dumps(dict(
            session_id=self._exploration_session, state=mission_message.data))))


def main(args: Optional[Sequence[str]] = None) -> None:
    from g3_integration_eval.node_lifecycle import run_node
    run_node(G3StateMachineNode, args)


if __name__ == "__main__":
    main()
