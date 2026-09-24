"""Adapt Unitree LowState battery fields to sensor_msgs/BatteryState."""

from math import isfinite, nan
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from unitree_go.msg import LowState

from g3_integration_eval.battery_adapter import unitree_soc_to_percentage


class UnitreeBatteryBridgeNode(Node):
    """Publish the battery contract consumed by the G3 state machine."""

    def __init__(self) -> None:
        super().__init__("g3_unitree_battery_bridge")
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("battery_topic", "/battery_state")

        lowstate_topic = str(self.get_parameter("lowstate_topic").value)
        battery_topic = str(self.get_parameter("battery_topic").value)
        self._publisher = self.create_publisher(
            BatteryState,
            battery_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            LowState,
            lowstate_topic,
            self._on_lowstate,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Unitree battery bridge: {lowstate_topic} -> {battery_topic}"
        )

    def _on_lowstate(self, message: LowState) -> None:
        try:
            percentage = unitree_soc_to_percentage(message.bms_state.soc)
        except ValueError as exc:
            self.get_logger().warning(f"ignoring invalid Unitree battery SOC: {exc}")
            return

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.percentage = percentage
        battery.present = True
        battery.voltage = self._finite_or_nan(message.power_v)
        battery.current = self._finite_or_nan(message.power_a)
        self._publisher.publish(battery)

    @staticmethod
    def _finite_or_nan(value) -> float:
        converted = float(value)
        return converted if isfinite(converted) else nan


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = UnitreeBatteryBridgeNode()
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
