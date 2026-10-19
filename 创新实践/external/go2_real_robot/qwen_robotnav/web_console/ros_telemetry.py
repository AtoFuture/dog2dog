"""ROS telemetry observer with opt-in, allowlisted G3 command input on stdin."""
import argparse
import base64
import io
import json
import math
import queue
import threading
import os
from pathlib import Path
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--domain', type=int, required=True)
    p.add_argument('--profile', type=Path, required=True)
    p.add_argument('--enable-commands', action='store_true')
    args = p.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.clock import Clock, ClockType
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_msgs.msg import String
    from sensor_msgs.msg import BatteryState
    from tf2_ros import Buffer, TransformListener
    from PIL import Image
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from qwen_robotnav.runtime_profile import load_profile
    profile = load_profile(args.profile)
    topics = profile['topics']
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-r', '/tf:=' + topics['tf'], '-r', '/tf_static:=' + topics['tf_static']])

    class Telemetry(Node):
        def __init__(self):
            super().__init__('go2_web_console_observer')
            self.values, self.seen, self.events, self.receipts = {}, {}, [], []
            self.last_pose_stamp = -1
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, topics['mission'], lambda m: self.save('mission', m.data), durable)
            self.create_subscription(String, topics['guard_status'], lambda m: self.structured('guard', m.data), 10)
            self.create_subscription(String, topics['guard_status']+'/corridor', lambda m: self.structured('corridor', m.data), 10)
            self.create_subscription(BatteryState, topics['battery'], lambda m: self.save('battery', m.percentage if m.present and math.isfinite(m.percentage) else None), qos_profile_sensor_data)
            self.create_subscription(Odometry, topics['odometry'], lambda m: self.save('odometry', {'linear': m.twist.twist.linear.x, 'angular': m.twist.twist.angular.z}), qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, topics['map'], self.mapping, durable)
            self.create_subscription(String, '/exploration/events', lambda m: self.structured('event', m.data), 20)
            self.create_subscription(String, topics['command'] + '/result', self.receipt, 20)
            self.create_subscription(String, '/g3/return_event', lambda m: self.structured('event', m.data, 'return'), 20)
            self.create_subscription(String, '/g3/localization_status', lambda m: self.structured('event', m.data, 'localization'), 20)
            if args.enable_commands:
                self.commands = queue.Queue(maxsize=20)
                self.publisher = self.create_publisher(String, topics['command'], 10)
                threading.Thread(target=self.read_commands, daemon=True).start()
                self.create_timer(.05, self.send_commands, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.create_timer(1., self.emit, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def receipt(self, message):
            try:
                data = json.loads(message.data)
                if isinstance(data, dict):
                    self.receipts = [*self.receipts[-39:], data]
            except ValueError:
                pass

        def read_commands(self):
            from task_control import COMMANDS
            for line in sys.stdin:
                try:
                    data = json.loads(line)
                    if (data.get('command') in COMMANDS and isinstance(data.get('request_id'), str)
                            and len(data['request_id']) == 32):
                        self.commands.put_nowait(data)
                except (ValueError, AttributeError, queue.Full):
                    continue

        def send_commands(self):
            while not self.commands.empty():
                request = self.commands.get_nowait()
                self.publisher.publish(String(data=json.dumps(request)))


        def save(self, key, value):
            self.values[key] = value
            self.seen[key] = time.monotonic()

        def structured(self, key, value, purpose=None):
            try:
                data = json.loads(value)
                if not isinstance(data, dict):
                    return
                json.dumps(data, allow_nan=False)
                if key == 'event':
                    event = {k: data[k] for k in ('status', 'purpose', 'attempt_id', 'target', 'reason') if k in data}
                    if purpose:
                        event['purpose'] = purpose
                    self.events.append(event)
                    self.events = self.events[-30:]
                else:
                    self.save(key, data)
            except (ValueError, TypeError):
                pass

        def mapping(self, m):
            w, h = m.info.width, m.info.height
            if not 0 < w * h <= 4_000_000 or len(m.data) != w * h or not math.isfinite(m.info.resolution) or m.info.resolution <= 0:
                return
            pixels = bytes(66 if v < 0 else round(239 - min(100, v) * 2.12) for v in m.data)
            image = Image.frombytes('L', (w, h), pixels).transpose(Image.Transpose.FLIP_TOP_BOTTOM)
            out = io.BytesIO()
            image.save(out, format='PNG')
            q = m.info.origin.orientation
            self.save('map', {'width': w, 'height': h, 'resolution': m.info.resolution,
                             'frame': m.header.frame_id, 'origin': [m.info.origin.position.x, m.info.origin.position.y,
                              math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))],
                             'png': base64.b64encode(out.getvalue()).decode()})

        def emit(self):
            try:
                stamped = self.tf.lookup_transform(profile['frames']['map'], profile['frames']['base'], Time())
                stamp = stamped.header.stamp.sec * 10**9 + stamped.header.stamp.nanosec
                if stamp <= self.last_pose_stamp:
                    raise ValueError('No fresh transform')
                self.last_pose_stamp = stamp
                tr = stamped.transform
                q = tr.rotation
                self.save('pose', {'x': tr.translation.x, 'y': tr.translation.y,
                                  'yaw': math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))})
            except Exception:
                pass
            now = time.monotonic()
            data = {'type': 'telemetry', 'values': self.values, 'ages': {k: now-v for k, v in self.seen.items()},
                    'events': self.events, 'command_receipts': self.receipts, 'sim_time_s': self.get_clock().now().nanoseconds / 1e9}
            try:
                print(json.dumps(data, allow_nan=False), flush=True)
            except (ValueError, BrokenPipeError):
                pass

    node = Telemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
