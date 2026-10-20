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
  linear_min     0.020   sim 线速度下限：只要 Nav2 表示「要动」，至少给这么多
  nav_deadband   0.005   Nav2 域的死区，低于此值视为「不要动」→ 输出 0
  angular_scale  1.667   实测偏航增益约 0.60，取倒数补偿
  angular_max    1.0     与 cmd_vel_pub 的裁剪一致

## ⚠️ 2026-09-24 修：死区原来判在错误的域上

原实现是 `lx = nav × 0.115; if abs(lx) < 0.002: lx = 0`——死区判的是**缩放之后**
的值，等价于把 Nav2 的 `0 ~ 0.0174 m/s` 整段抹成精确的 0。而这正是 DWB 在
准直/贴路径阶段输出的区间（实测稳定在 0.0137），于是：

    amcl_G5 那一轮（goal (8.00,3.50)）真值原地冻结 44 s，
    cmd_vel_scaled_out lx=0.01368 → cmd_vel lx=0.0000，
    progress checker 连报两次 Failed to make progress，
    到点耗时 86 s 超出 80 s 测试预算。

现在死区判在 Nav2 域，并且给一个最低可执行速度——因为这台狗**根本没有「慢速」**：
实测 cmd 0.02 就已经是 0.206 m/s，比它更慢的指令只会让它原地蹬腿。
所以只要 Nav2 说要动，就直接给到 `linear_min`。

## ⚠️ 角速度补偿在 |az| > 0.6 之后会被裁平（问题清单 M5）

`angular_scale = 1.667` 是照着「实测偏航增益约 0.60」取的倒数补偿，但
`az × 1.667` 一旦超过 `angular_max = 1.0`（即输入 |az| > 0.6）就被 `_clip` 截断，
补偿**凭空消失**——那一整段的偏航增益退回 0.60，转得比 Nav2 以为的慢 40%。

线速度那条的副作用上面写了，这条原来没写。实测影响面不小：
amcl_G5 那个包里 694 条指令有 **163 条**被裁，goal_G5 里 802 条有 **288 条**。

要用到 >0.6 的角速度（急转、原地掉头）时，得先想清楚这段怎么办：
要么把 `angular_scale` 调到让常用段不越界，要么接受它并记住 Nav2 的
「rad/s」在这一段是虚的。
"""

import math
import json
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class CmdVelScaler(Node):
    def __init__(self):
        super().__init__("cmd_vel_scaler")

        self.declare_parameter("input_topic", "/robot1/cmd_vel_scaled_out")
        self.declare_parameter("output_topic", "/robot1/cmd_vel")
        self.declare_parameter("linear_scale", 0.115)
        self.declare_parameter("linear_max", 0.030)
        self.declare_parameter("linear_min", 0.020)
        self.declare_parameter("nav_deadband", 0.005)
        self.declare_parameter("angular_scale", 1.667)
        self.declare_parameter("angular_max", 1.0)
        self.declare_parameter("linear_accel", 0.060)
        self.declare_parameter("linear_decel", 0.100)
        self.declare_parameter("angular_accel", 2.0)
        self.declare_parameter("angular_decel", 2.0)
        self.declare_parameter("ramp_rate", 50.0)
        self.declare_parameter("cmd_timeout", 1.0)
        self.declare_parameter("corridor_config", "")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._linear_scale = float(self.get_parameter("linear_scale").value)
        self._linear_max = float(self.get_parameter("linear_max").value)
        self._linear_min = float(self.get_parameter("linear_min").value)
        self._nav_deadband = float(self.get_parameter("nav_deadband").value)
        self._angular_scale = float(self.get_parameter("angular_scale").value)
        self._angular_max = float(self.get_parameter("angular_max").value)
        self._linear_accel = float(self.get_parameter("linear_accel").value)
        self._linear_decel = float(self.get_parameter("linear_decel").value)
        self._angular_accel = float(self.get_parameter("angular_accel").value)
        self._angular_decel = float(self.get_parameter("angular_decel").value)
        ramp_rate = float(self.get_parameter("ramp_rate").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._corridor = None
        self._raw_command = (0., 0.)
        config = str(self.get_parameter('corridor_config').value)
        if config:
            from qwen_robotnav.corridor import CorridorGuidance, CorridorTurnSettler, CorridorTurnRecovery
            from sensor_msgs.msg import LaserScan
            from nav_msgs.msg import Path
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
            from tf2_ros import Buffer, TransformListener
            self._corridor_config = json.loads(config)
            self._corridor = CorridorGuidance(self._corridor_config['settings'])
            self._corridor_turn_settler = CorridorTurnSettler()
            self._corridor_recovery = CorridorTurnRecovery()
            self._corridor_sensor_radius = math.inf
            self._corridor_range_min = math.inf
            self._corridor_scan_at = -math.inf
            self._corridor_mission_at = -math.inf
            self._corridor_mission = 'standby'
            self._corridor_status_at = -math.inf
            self._corridor_mode = None
            self._corridor_path = None
            self._corridor_path_at = -math.inf
            self._corridor_tf = Buffer()
            self._corridor_listener = TransformListener(self._corridor_tf, self)
            self._corridor_pub = self.create_publisher(String, self._corridor_config['status_topic'], 10)
            self.create_subscription(LaserScan, self._corridor_config['scan_topic'], self._corridor_scan, qos_profile_sensor_data)
            self.create_subscription(Path, self._corridor_config['path_topic'], self._corridor_plan, 10)
            self.create_subscription(String, self._corridor_config['mission_topic'], self._corridor_state,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self._sub = self.create_subscription(
            Twist, input_topic, self._on_cmd, 10
        )

        self._target_lx = 0.0
        self._target_az = 0.0
        self._current_lx = 0.0
        self._current_az = 0.0
        self._dt = 1.0 / ramp_rate
        self._last_cmd_time = self.get_clock().now()
        self._timer = self.create_timer(self._dt, self._ramp_loop)
        self._last_logged = None
        self.get_logger().info(
            f"速度缩放就绪（带渐变）：{input_topic} → {output_topic}；"
            f"linear×{self._linear_scale}（{self._linear_min}~{self._linear_max}），"
            f"nav 死区 {self._nav_deadband}，"
            f"angular×{self._angular_scale}（上限 {self._angular_max}），"
            f"渐变 accel={self._linear_accel} decel={self._linear_decel} @ {ramp_rate:.0f}Hz"
        )

    def _clip(self, value, limit):
        return max(-limit, min(limit, value))

    def _linear(self, nav_value: float) -> float:
        """Nav2 线速度 → sim 线速度。死区判在 **Nav2 域** 而不是缩放后的域。"""
        if abs(nav_value) < self._nav_deadband:
            return 0.0
        scaled = nav_value * self._linear_scale
        if abs(scaled) < self._linear_min:
            scaled = math.copysign(self._linear_min, nav_value)
        return self._clip(scaled, self._linear_max)

    def _ramp_step(self, current: float, target: float, accel: float, decel: float) -> float:
        """将 current 向 target 线性逼近一步，加速用 accel、减速用 decel。"""
        diff = target - current
        if abs(diff) < 1e-6:
            return target
        rate = decel if abs(target) < abs(current) else accel
        max_step = rate * self._dt
        if abs(diff) <= max_step:
            return target
        return current + math.copysign(max_step, diff)

    def _ramp_loop(self) -> None:
        now = self.get_clock().now()
        if (now - self._last_cmd_time).nanoseconds * 1e-9 > self._cmd_timeout:
            self._target_lx = 0.0
            self._target_az = 0.0
            self._raw_command = (0., 0.)
        if self._corridor is not None:
            self._guide_corridor()
        out = Twist()
        self._current_lx = self._ramp_step(
            self._current_lx, self._target_lx, self._linear_accel, self._linear_decel
        )
        self._current_az = self._ramp_step(
            self._current_az, self._target_az, self._angular_accel, self._angular_decel
        )
        out.linear.x = self._current_lx
        out.linear.y = 0.0
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = self._current_az
        self._pub.publish(out)

        summary = (
            f"target=({self._target_lx:.4f}, az={self._target_az:.3f}) → "
            f"cur=({self._current_lx:.4f}, az={self._current_az:.3f})"
        )
        if summary != self._last_logged:
            self.get_logger().info(summary)
            self._last_logged = summary

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._raw_command = (msg.linear.x, msg.angular.z)
        self._target_lx = self._linear(msg.linear.x)
        az = msg.angular.z * self._angular_scale
        self._target_az = self._clip(az, self._angular_max)

    def _corridor_state(self, msg):
        if msg.data != self._corridor_mission:
            self._corridor_recovery.reset()
        self._corridor_mission = msg.data
        self._corridor_mission_at = time.monotonic()

    def _corridor_plan(self, msg):
        self._corridor_path = msg if msg.poses and msg.header.frame_id else None
        self._corridor_path_at = time.monotonic()

    def _corridor_target(self):
        from rclpy.time import Time
        path = self._corridor_path
        if path is None or time.monotonic()-self._corridor_path_at>2.:
            return None
        try:
            age=(self.get_clock().now().nanoseconds-(path.header.stamp.sec*10**9+path.header.stamp.nanosec))*1e-9
            if not -.1<=age<=2.:return None
            if path.header.frame_id==self._corridor_config['base_frame']:
                points=[(pose.pose.position.x,pose.pose.position.y) for pose in path.poses]
            else:
                tr=self._corridor_tf.lookup_transform(self._corridor_config['base_frame'],path.header.frame_id,Time()).transform
                q=tr.rotation
                if (not all(math.isfinite(v) for v in (tr.translation.x,tr.translation.y,q.x,q.y,q.z,q.w))
                        or abs(q.x)+abs(q.y)>.01 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):return None
                yaw=2*math.atan2(q.z,q.w);c,s=math.cos(yaw),math.sin(yaw)
                points=[(tr.translation.x+c*pose.pose.position.x-s*pose.pose.position.y,
                         tr.translation.y+s*pose.pose.position.x+c*pose.pose.position.y)
                        for pose in path.poses]
            if not points or not all(all(math.isfinite(v) for v in point) for point in points):return None
            lookahead=self._corridor_config['settings'].get('path_lookahead_m',1.)
            return next((point for point in points if math.hypot(*point)>=lookahead),points[-1])
        except Exception:
            return None

    def _corridor_scan(self, msg):
        from rclpy.time import Time
        from qwen_robotnav.velocity_safety import scan_points
        try:
            age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec))*1e-9
            if not -.1 <= age <= self._corridor_config['sensor_timeout_s']:
                raise ValueError('stale scan')
            base = self._corridor_config['base_frame']
            if msg.header.frame_id == base:
                sensor = (0., 0., 0.)
            else:
                tr = self._corridor_tf.lookup_transform(base, msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
                q = tr.rotation
                if (not all(math.isfinite(v) for v in (tr.translation.x,tr.translation.y,q.x,q.y,q.z,q.w))
                        or abs(q.x)+abs(q.y) > .01 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
                    raise ValueError('invalid planar laser transform')
                sensor = (tr.translation.x, tr.translation.y, 2*math.atan2(q.z,q.w))
            points = scan_points(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,sensor)
            if points is None:
                raise ValueError('incomplete scan')
            stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            if self._corridor.observe(points, stamp):
                self._corridor_scan_at = time.monotonic()
                self._corridor_sensor_radius = math.hypot(*sensor[:2])
                self._corridor_range_min = msg.range_min
        except Exception:
            self._corridor_scan_at = -math.inf
            self._corridor.geometry = self._corridor.previous = None
            self._corridor.confirmations = 0

    def _guide_corridor(self):
        from std_msgs.msg import String
        now = time.monotonic()
        raw_x, raw_z = self._raw_command
        target = self._corridor_target()
        if not all(math.isfinite(v) for v in (raw_x,raw_z)):
            self._corridor.reset_alignment()
            x,z,mode = 0.,0.,'invalid_command'
        elif self._corridor_mission not in ('exploring','returning') or now-self._corridor_mission_at > 2.:
            self._corridor.reset_alignment()
            x,z,mode = 0.,0.,'inactive'
        elif now-self._corridor_scan_at > self._corridor_config['sensor_timeout_s']:
            self._corridor.geometry = self._corridor.previous = None
            self._corridor.confirmations = 0
            self._corridor.reset_alignment()
            x,z,mode = 0.,0.,'scan_unavailable'
        else:
            x,z,mode = self._corridor.guide(0. if abs(raw_x)<self._nav_deadband else raw_x, raw_z,target)
        recovery_enabled=(self._corridor.settings.get('steering_enabled',False)
            and self._corridor.settings.get('turn_recenter_enabled',False)
            and mode not in ('inactive','invalid_command','scan_unavailable','awaiting_path'))
        x,z,mode = self._corridor_recovery.filter(x,z,mode,
            geometry=self._corridor.geometry,sensor_radius=self._corridor_sensor_radius,
            range_min=self._corridor_range_min,requested=abs(raw_x)+abs(raw_z)>1e-6,
            now=self.get_clock().now().nanoseconds*1e-9,
            current_linear=self._current_lx,current_angular=self._current_az,enabled=recovery_enabled)
        z, settling = self._corridor_turn_settler.filter(
            x, z, self._current_lx, self._current_az,
            self.get_clock().now().nanoseconds * 1e-9,
            enabled=(self._corridor.settings.get('steering_enabled', False)
                     and self._corridor.settings.get('turn_settle_enabled', False)
                     and self._corridor.geometry is not None))
        if settling and mode not in ('inactive', 'invalid_command', 'scan_unavailable', 'awaiting_path'):
            mode = 'turn_settling'
        self._target_lx = self._linear(x)
        self._target_az = self._clip(z*self._angular_scale, self._angular_max)
        if mode != self._corridor_mode or now-self._corridor_status_at >= .5:
            self._corridor_mode, self._corridor_status_at = mode, now
            snapshot = self._corridor.snapshot()
            snapshot.update(self._corridor_recovery.snapshot())
            snapshot.update(mode=mode, source='measured_scan', scan_fresh=now-self._corridor_scan_at <= self._corridor_config['sensor_timeout_s'],
                            requested=[raw_x,raw_z], guided=[x,z], scan_stamp_ns=self._corridor.last_stamp,
                            path_target_base=target,
                            stamp_ns=self.get_clock().now().nanoseconds)
            self._corridor_pub.publish(String(data=json.dumps(snapshot)))


def main():
    # Defer shutdown until the DDS take/callback has returned to Python.
    # rclpy's asynchronous context shutdown can invalidate an in-flight take.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False
    def request_stop(_signal, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = CmdVelScaler()
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown(uninstall_handlers=False)


if __name__ == "__main__":
    main()
