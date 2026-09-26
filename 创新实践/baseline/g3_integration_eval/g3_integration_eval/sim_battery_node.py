"""Publish a gradually discharging BatteryState for Gazebo integration."""

from math import floor
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from g3_integration_eval.sim_battery import (
    BatteryDischargeModel,
    elapsed_seconds,
    twist_is_moving,
    validate_nonnegative,
    validate_positive,
)


class SimulatedBatteryNode(Node):
    """Model idle/moving discharge and publish a standard BatteryState."""

    def __init__(self) -> None:
        super().__init__("g3_sim_battery")
        self.declare_parameter("battery_topic", "/g3/sim_battery_state")
        self.declare_parameter("cmd_vel_topic", "/robot1/cmd_vel")
        self.declare_parameter("initial_percentage", 0.80)
        self.declare_parameter("idle_drain_per_sec", 0.0001)
        self.declare_parameter("moving_drain_per_sec", 0.0010)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("motion_timeout_sec", 1.0)
        self.declare_parameter("motion_epsilon", 0.01)
        self.declare_parameter("capacity_ah", 10.0)
        self.declare_parameter("empty_voltage", 21.0)
        self.declare_parameter("full_voltage", 25.2)

        battery_topic = str(self.get_parameter("battery_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._publish_rate_hz = validate_positive(
            self.get_parameter("publish_rate_hz").value,
            "publish_rate_hz",
        )
        self._motion_timeout_sec = validate_nonnegative(
            self.get_parameter("motion_timeout_sec").value,
            "motion_timeout_sec",
        )
        self._motion_epsilon = validate_nonnegative(
            self.get_parameter("motion_epsilon").value,
            "motion_epsilon",
        )
        self._capacity_ah = validate_positive(
            self.get_parameter("capacity_ah").value,
            "capacity_ah",
        )
        self._empty_voltage = validate_positive(
            self.get_parameter("empty_voltage").value,
            "empty_voltage",
        )
        self._full_voltage = validate_positive(
            self.get_parameter("full_voltage").value,
            "full_voltage",
        )
        if self._full_voltage < self._empty_voltage:
            raise ValueError("full_voltage must be >= empty_voltage")

        self._model = BatteryDischargeModel(
            self.get_parameter("initial_percentage").value,
            self.get_parameter("idle_drain_per_sec").value,
            self.get_parameter("moving_drain_per_sec").value,
        )
        self._last_update_ns = None
        self._last_motion_ns = None
        self._last_log_bucket = floor(self._model.percentage * 20.0)

        self._publisher = self.create_publisher(
            BatteryState,
            battery_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self._on_cmd_vel,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(
            1.0 / self._publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "simulated battery ready: "
            f"topic={battery_topic}, cmd_vel={cmd_vel_topic}, "
            f"initial={self._model.percentage:.3f}"
        )
        self._publish(self.get_clock().now(), moving=False)

    def _on_cmd_vel(self, message: Twist) -> None:
        moving = twist_is_moving(
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
            self._motion_epsilon,
        )
        self._last_motion_ns = (
            self.get_clock().now().nanoseconds if moving else None
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        elapsed_sec = elapsed_seconds(self._last_update_ns, now_ns)
        self._last_update_ns = now_ns
        moving = (
            self._last_motion_ns is not None
            and now_ns - self._last_motion_ns
            <= self._motion_timeout_sec * 1e9
        )
        self._model.advance(elapsed_sec, moving=moving)
        self._publish(now, moving=moving)

        bucket = floor(self._model.percentage * 20.0)
        if bucket < self._last_log_bucket:
            self.get_logger().info(
                f"battery={self._model.percentage:.3f}, moving={moving}"
            )
            self._last_log_bucket = bucket

    def _publish(self, now, *, moving: bool) -> None:
        percentage = self._model.percentage
        message = BatteryState()
        message.header.stamp = now.to_msg()
        message.percentage = percentage
        message.capacity = self._capacity_ah
        message.design_capacity = self._capacity_ah
        message.charge = percentage * self._capacity_ah
        message.voltage = self._empty_voltage + percentage * (
            self._full_voltage - self._empty_voltage
        )
        message.current = -2.0 if moving else -0.2
        message.present = True
        message.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        self._publisher.publish(message)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimulatedBatteryNode()
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
