"""Isolated DDS transport check; never publishes a motion or task command."""
import base64
import importlib.util
import io
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy') and os.environ.get('ROS_DOMAIN_ID') == '232', 'Run in ROS with isolated domain 232')
class TelemetryTransport(unittest.TestCase):
    def test_real_messages_reach_json_and_preserve_map_orientation(self):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import BatteryState
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster
        from PIL import Image
        root = Path(__file__).resolve().parent
        rclpy.init(args=['--ros-args','-r','/tf:=/robot1/tf'])
        node = rclpy.create_node('console_transport_fixture')
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        mission = node.create_publisher(String, '/mission_state', durable)
        mapping = node.create_publisher(OccupancyGrid, '/robot1/map', durable)
        battery = node.create_publisher(BatteryState, '/runtime/battery_state', 10)
        odom = node.create_publisher(Odometry, '/robot1/odometry/filtered', 10)
        guard = node.create_publisher(String, '/runtime/velocity_guard_status', 10)
        events = node.create_publisher(String, '/exploration/events', 10)
        tf = TransformBroadcaster(node)
        child = subprocess.Popen([sys.executable, '-u', str(root/'ros_telemetry.py'), '--domain', '232',
                                  '--profile', str(root.parent/'config/sim_runtime.yaml')], stdout=subprocess.PIPE, text=True)
        messages = queue.Queue()
        def read_messages():
            for line in child.stdout:
                messages.put(line)
        reader=threading.Thread(target=read_messages, daemon=True)
        reader.start()
        try:
            result = None
            deadline = time.monotonic()+20
            while time.monotonic()<deadline and result is None:
                mission.publish(String(data='standby'))
                m = OccupancyGrid()
                m.header.frame_id='map'
                m.info.width=3
                m.info.height=2
                m.info.resolution=.1
                m.info.origin.orientation.w=1.
                m.data=[-1,0,100,100,0,-1]
                mapping.publish(m)
                battery.publish(BatteryState(present=True, percentage=.77))
                o = Odometry()
                o.twist.twist.linear.x=.25
                odom.publish(o)
                guard.publish(String(data=json.dumps({'reason':'mission:standby','footprint_clearance_m':.8})))
                events.publish(String(data=json.dumps({'status':'accepted','purpose':'frontier','attempt_id':7})))
                transforms=[]
                for parent, frame, x, y in [('map','odom',0.,0.),('odom','base_link',.75,.25)]:
                    tr=TransformStamped()
                    tr.header.frame_id=parent
                    tr.header.stamp=node.get_clock().now().to_msg()
                    tr.child_frame_id=frame
                    tr.transform.rotation.w=1.
                    tr.transform.translation.x=x
                    tr.transform.translation.y=y
                    transforms.append(tr)
                tf.sendTransform(transforms)
                rclpy.spin_once(node,timeout_sec=.1)
                while not messages.empty():
                    try:
                        data=json.loads(messages.get_nowait())
                        if {'mission','map','battery','odometry','guard','pose'}<=set(data.get('values',{})) and data.get('events'):
                            result=data
                    except ValueError:
                        pass
            self.assertIsNotNone(result,'No complete telemetry snapshot received')
            values=result['values']
            self.assertEqual(values['mission'],'standby')
            self.assertAlmostEqual(values['odometry']['linear'],.25)
            self.assertAlmostEqual(values['pose']['x'],.75)
            self.assertEqual(result['events'][-1]['attempt_id'],7)
            image=Image.open(io.BytesIO(base64.b64decode(values['map']['png'])))
            self.assertEqual(image.size,(3,2))
            self.assertEqual([image.getpixel((x,y)) for y in range(2) for x in range(3)],[27,239,66,66,239,27])
            self.assertEqual(node.get_publishers_info_by_topic('/robot1/cmd_vel'),[])
            self.assertEqual(node.get_publishers_info_by_topic('/g3/command'),[])
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=5)
            reader.join(timeout=2)
            child.stdout.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def test_enabled_bridge_publishes_only_g3_envelope_and_returns_receipt(self):
        import rclpy
        from std_msgs.msg import String
        root = Path(__file__).resolve().parent
        rclpy.init()
        node = rclpy.create_node('console_command_transport_fixture')
        received = []
        receipt = node.create_publisher(String, '/g3/command/result', 20)
        def command(message):
            request = json.loads(message.data)
            received.append(request)
            receipt.publish(String(data=json.dumps(dict(request, accepted=True, state='estop', reason=''))))
        node.create_subscription(String, '/g3/command', command, 10)
        child = subprocess.Popen([sys.executable, '-u', str(root/'ros_telemetry.py'), '--domain', '232',
            '--profile', str(root.parent/'config/sim_runtime.yaml'), '--enable-commands'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        messages = queue.Queue()
        reader = threading.Thread(target=lambda: [messages.put(line) for line in child.stdout], daemon=True)
        reader.start()
        request = dict(command='ESTOP', request_id='a'*32)
        try:
            deadline = time.monotonic()+15
            while time.monotonic()<deadline and not node.get_publishers_info_by_topic('/g3/command'):
                rclpy.spin_once(node, timeout_sec=.1)
            child.stdin.write(json.dumps(request)+'\n'); child.stdin.flush()
            result = None
            while time.monotonic()<deadline and result is None:
                rclpy.spin_once(node, timeout_sec=.1)
                while not messages.empty():
                    data=json.loads(messages.get_nowait())
                    if data.get('command_receipts'): result=data
            self.assertEqual(received,[request])
            self.assertIsNotNone(result)
            self.assertEqual(result['command_receipts'][-1]['request_id'],request['request_id'])
            self.assertEqual(node.get_publishers_info_by_topic('/robot1/cmd_vel'),[])
        finally:
            child.send_signal(signal.SIGINT)
            try: child.wait(timeout=8)
            except subprocess.TimeoutExpired: child.terminate();child.wait(timeout=5)
            child.stdin.close();child.stdout.close();reader.join(timeout=2)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    unittest.main()
