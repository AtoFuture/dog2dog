"""ROS 2 wrapper for the G3 state machine."""

import os
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from g3_integration_eval.navigate_to_pose_client import (
    NavigateToPoseClientSkeleton,
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
    """Expose the four-state controller while keeping navigation preview-only."""

    def __init__(self) -> None:
        super().__init__("g3_state_machine")

        self.declare_parameter("command_topic", "/g3/command")
        self.declare_parameter("state_topic", "/g3/state")
        self.declare_parameter("battery_topic", "/robot1/battery_state")
        self.declare_parameter("low_battery_threshold", 0.20)
        self.declare_parameter("navigate_action", "/robot1/navigate_to_pose")
        self.declare_parameter("home_frame", "map")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)

        command_topic = str(self.get_parameter("command_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        battery_topic = str(self.get_parameter("battery_topic").value)
        self._low_battery_threshold = validate_low_battery_threshold(
            float(self.get_parameter("low_battery_threshold").value)
        )
        navigate_action = str(self.get_parameter("navigate_action").value)

        self._state = State.STANDBY
        self._state_publisher = self.create_publisher(String, state_topic, 10)
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
        self._navigator = NavigateToPoseClientSkeleton(self, navigate_action)

        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        self.get_logger().info(
            f"G3 state machine ready in STANDBY (ROS_DOMAIN_ID={domain_id})"
        )
        self.get_logger().info(
            f"low-battery RETURN monitor: topic={battery_topic}, "
            f"threshold={self._low_battery_threshold:.3f}"
        )
        self.get_logger().warning(
            "NavigateToPose transmission is hard-disabled; ESTOP currently changes "
            "logical state only and is not a physical robot stop command"
        )
        if domain_id == "42":
            self.get_logger().warning(
                "Shared Domain 42 detected. No navigation goal can be transmitted "
                "by this scaffold. Use an isolated domain for development tests."
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
            f"{self._low_battery_threshold:.3f}; requesting RETURN preview"
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
            self._preview_return_goal()
        elif self._state is State.ESTOP:
            self.get_logger().error(
                "ESTOP logical state entered; physical stop integration is not yet wired"
            )

    def _preview_return_goal(self) -> None:
        goal = self._navigator.build_goal(
            frame_id=str(self.get_parameter("home_frame").value),
            x=float(self.get_parameter("home_x").value),
            y=float(self.get_parameter("home_y").value),
            yaw=float(self.get_parameter("home_yaw").value),
        )
        self._navigator.preview_goal(goal)

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
