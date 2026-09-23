#!/usr/bin/env python3
"""把 Nav2 的速度指令缩放到这款步态真正能执行的区间。

## 为什么需要它（2026-09-23 实测）

链路是 `cmd_vel` → `cmd_vel_pub.py` → `robot_velocity`(rv) → 步态控制器，
其中 `cmd_vel_pub` 对小的 cmd 近似 `rv ≈ 1.05 × cmd`（几乎 1:1），
而**步态对 rv 的增益约 10 倍**（rv=0.02 时真值 0.206 m/s，见下）。

结果：Nav2 按配置发 0.26 m/s，实际会跑出 ~2.7 m/s，rv 顶到 0.27 ——
而实测的稳定边界是 **rv < 0.04**，超了就会原地蹬腿或横向崩掉。

    cmd_vel   rv      真值速度   稳定？
    0.02      0.0203  0.206 m/s  稳、直行
    0.04      0.0392  0.183 m/s  稳，但横漂 0.53 m
    0.06      0.0568  ~0（原地蹬腿，路径 1.92 m 净位移 0.03 m）

本节点把 Nav2 的输出线性压到 `sim_cmd ∈ [0, linear_max]`，使：
  · Nav2 的「m/s」与实际米/秒大致对齐（差 20% 量级，够 Nav2 的进度检查用）
  · rv 始终落在 0.04 以内

## 为什么不改 cmd_vel_pub.py

它在共享仿真工作区里，别人也在用同一份；而且该工作区已有一堆未提交改动。
所以缩放只落在我们这份配置里：Nav2 的输出被挪到 `cmd_vel_nav`（见
`nav2_scaled.launch.py`），本节点读它、写回标准的 `cmd_vel`，
让仿真自带的 `cmd_vel_pub` 照常工作，共享代码一行不动。

## 参数（默认值由实测推出，换机器人/改步态后要重新标定）

  linear_scale   0.115   sim_linear = nav_linear × 该系数（0.26→0.030）
  linear_max     0.030   sim 线速度上限（对应 rv≈0.030，留出边界余量）
  angular_scale  1.667   实测偏航增益约 0.60，取倒数补偿
  angular_max    1.0     与 cmd_vel_pub 的裁剪一致
  deadband       0.002   小于此值的速度置零，避免持续抖动
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelScaler(Node):
    def __init__(self):
        super().__init__("cmd_vel_scaler")

        self.declare_parameter("input_topic", "/robot1/cmd_vel_scaled_out")
        self.declare_parameter("output_topic", "/robot1/cmd_vel")
        self.declare_parameter("linear_scale", 0.115)
        self.declare_parameter("linear_max", 0.030)
        self.declare_parameter("angular_scale", 1.667)
        self.declare_parameter("angular_max", 1.0)
        self.declare_parameter("deadband", 0.002)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._linear_scale = float(self.get_parameter("linear_scale").value)
        self._linear_max = float(self.get_parameter("linear_max").value)
        self._angular_scale = float(self.get_parameter("angular_scale").value)
        self._angular_max = float(self.get_parameter("angular_max").value)
        self._deadband = float(self.get_parameter("deadband").value)

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self._sub = self.create_subscription(
            Twist, input_topic, self._on_cmd, 10
        )
        self._last_logged = None
        self.get_logger().info(
            f"速度缩放就绪：{input_topic} → {output_topic}；"
            f"linear×{self._linear_scale}（上限 {self._linear_max}），"
            f"angular×{self._angular_scale}（上限 {self._angular_max}）"
        )

    def _clip(self, value, limit):
        return max(-limit, min(limit, value))

    def _on_cmd(self, msg: Twist) -> None:
        out = Twist()

        # 线速度：缩放 + 限幅 + 死区
        lx = msg.linear.x * self._linear_scale
        ly = msg.linear.y * self._linear_scale
        if abs(lx) < self._deadband:
            lx = 0.0
        if abs(ly) < self._deadband:
            ly = 0.0
        out.linear.x = self._clip(lx, self._linear_max)
        out.linear.y = self._clip(ly, self._linear_max)
        out.linear.z = 0.0

        # 角速度：补偿实测增益后裁剪（cmd_vel_pub 也会裁一次）
        az = msg.angular.z * self._angular_scale
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = self._clip(az, self._angular_max)

        self._pub.publish(out)

        summary = (
            f"nav=({msg.linear.x:.3f}, az={msg.angular.z:.3f}) → "
            f"sim=({out.linear.x:.4f}, az={out.angular.z:.3f})"
        )
        if summary != self._last_logged:
            self.get_logger().info(summary)
            self._last_logged = summary


def main():
    rclpy.init()
    node = CmdVelScaler()
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
