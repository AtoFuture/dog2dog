"""ROS 2 wrapper for the G3 state machine and RETURN navigation flow."""

import json
import os
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from g3_integration_eval.navigate_to_pose_client import (
    NavigateToPoseClient,
)
from g3_integration_eval.state_machine import (
    Event,
    InvalidTransition,
    State,
    battery_return_event,
    parse_event,
    transition,
    validate_low_battery_threshold,
)


class G3StateMachineNode(Node):
    """Expose the controller with navigation transmission disabled by default."""

    def __init__(self) -> None:
        super().__init__("g3_state_machine")

        self.declare_parameter("command_topic", "/g3/command")
        self.declare_parameter("state_topic", "/g3/state")
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

        command_topic = str(self.get_parameter("command_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
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

        self._state = State.STANDBY
        self._state_publisher = self.create_publisher(String, state_topic, 10)
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
            "ESTOP currently changes logical state only and is not a physical "
            "robot stop command"
        )
        self.get_logger().info(f"RETURN events: topic={return_event_topic}")
        if domain_id == "42":
            if transmission_enabled:
                self.get_logger().warning(
                    "Shared Domain 42 detected with navigation transmission ENABLED"
                )
            else:
                self.get_logger().warning(
                    "Shared Domain 42 detected; navigation remains preview-only"
                )
        self._publish_state()

    def _on_command(self, message: String) -> None:
        try:
            event = parse_event(message.data)
        except InvalidTransition as exc:
            self.get_logger().warning(str(exc))
            return

        self._apply_event(event, source="command")

    def _on_battery(self, message: BatteryState) -> None:
        event = battery_return_event(
            self._state,
            message.percentage,
            self._low_battery_threshold,
        )
        if event is None:
            return

        self.get_logger().warning(
            f"low battery detected: {message.percentage:.3f} < "
            f"{self._low_battery_threshold:.3f}; requesting RETURN"
        )
        self._apply_event(event, source="battery")

    def _apply_event(self, event: Event, *, source: str) -> None:
        previous = self._state
        try:
            self._state = transition(previous, event)
        except InvalidTransition as exc:
            self.get_logger().warning(str(exc))
            return

        self.get_logger().info(
            f"state transition: {previous.value} --{event.value}--> "
            f"{self._state.value} (source={source})"
        )
        self._publish_state()

        if self._state is State.RETURN and previous is not State.RETURN:
            self._start_return(source)
        elif self._state is State.ESTOP:
            self.get_logger().error(
                "ESTOP logical state entered; physical stop integration is not "
                "yet wired"
            )

    def _start_return(self, trigger: str) -> None:
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

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.value
        self._state_publisher.publish(message)


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
