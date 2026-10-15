"""Publish a gradually discharging BatteryState for Gazebo integration."""

from math import floor
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from g3_integration_eval.sim_battery import (
    BatteryDischargeModel,
    elapsed_seconds,
    successful_return_attempt_id,
    twist_is_moving,
    validate_nonnegative,
    validate_percentage,
    validate_positive,
)


class SimulatedBatteryNode(Node):
    """Model idle/moving discharge and publish a standard BatteryState."""

    def __init__(self) -> None:
        super().__init__("g3_sim_battery")
        self.declare_parameter("battery_topic", "/g3/sim_battery_state")
        self.declare_parameter("cmd_vel_topic", "/robot1/cmd_vel")
        self.declare_parameter("return_event_topic", "/g3/return_event")
        self.declare_parameter("auto_recharge_on_return_success", False)
        self.declare_parameter("recharge_percentage", 0.80)
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
        return_event_topic = str(
            self.get_parameter("return_event_topic").value
        )
        self._auto_recharge = bool(
            self.get_parameter("auto_recharge_on_return_success").value
        )
        self._recharge_percentage = validate_percentage(
            self.get_parameter("recharge_percentage").value,
            "recharge_percentage",
        )
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
        self._recharged_attempt_ids = set()

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
        self._return_event_subscription = None
        if self._auto_recharge:
            self._return_event_subscription = self.create_subscription(
                String,
                return_event_topic,
                self._on_return_event,
                10,
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
        if self._auto_recharge:
            self.get_logger().info(
                "automatic simulated recharge enabled: "
                f"return_event_topic={return_event_topic}, "
                f"target={self._recharge_percentage:.3f}"
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

    def _on_return_event(self, message: String) -> None:
        try:
            attempt_id = successful_return_attempt_id(message.data)
        except ValueError as exc:
            self.get_logger().warning(f"invalid RETURN event ignored: {exc}")
            return

        if attempt_id is None or attempt_id in self._recharged_attempt_ids:
            return

        previous_percentage = self._model.percentage
        recharged_percentage = self._model.recharge_to(self._recharge_percentage)
        self._recharged_attempt_ids.add(attempt_id)

        now = self.get_clock().now()
        self._last_update_ns = now.nanoseconds
        self._last_motion_ns = None
        self._last_log_bucket = floor(recharged_percentage * 20.0)
        self._publish(now, moving=False)
        self.get_logger().warning(
            f"simulated recharge applied: attempt_id={attempt_id}, "
            f"battery={previous_percentage:.3f} -> {recharged_percentage:.3f}"
        )


    def _on_timer(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        clock_rewound = (
            self._last_update_ns is not None
            and now_ns < self._last_update_ns
        )
        if clock_rewound:
            self._last_motion_ns = None
        elapsed_sec = elapsed_seconds(self._last_update_ns, now_ns)
        self._last_update_ns = now_ns
        moving = (
            not clock_rewound
            and self._last_motion_ns is not None
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
