#!/usr/bin/env python3
"""桥节点冒烟测试（无需真机）：mock 驱动 → 验证三话题发布 + 体素解码上链。

在 baiyanggo:foxy-dev 容器内运行（挂载本仓库 robot/ 与 wheels）：
  python3 test/test_bridge_mock.py
"""
import asyncio
import json
import lz4.block
import os
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

# ---------------- mock go2_webrtc_driver ----------------

SRC_POSE = "rt/utlidar/robot_pose"
SRC_LIDAR_STATE = "rt/utlidar/lidar_state"


class FakePubSub:
    def __init__(self):
        self.cbs = {}

    def subscribe(self, topic, cb):
        self.cbs[topic] = cb

    def inject_pose(self):
        d = {"data": {
            "header": {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": "odom"},
            "pose": {"position": {"x": 1.5, "y": -0.25, "z": 0.05},
                     "orientation": {"x": 0, "y": 0, "z": 0.1, "w": 0.99}}}}
        if SRC_POSE in self.cbs:
            self.cbs[SRC_POSE](d)

    def inject_state(self):
        if SRC_LIDAR_STATE in self.cbs:
            self.cbs[SRC_LIDAR_STATE]({"data": {"error_state": 0,
                                                "cloud_packet_loss_rate": 2.2}})


class FakeDataChannel:
    def __init__(self):
        self.data_channel_opened = True
        self.pub_sub = FakePubSub()


class FakeConnection:
    def __init__(self, *a, **kw):
        self.datachannel = FakeDataChannel()

    async def connect(self):
        pass

    async def disconnect(self):
        self.datachannel.data_channel_opened = False


def make_fake_driver_module():
    import types
    mod = types.ModuleType("go2_webrtc_driver")

    class Method:
        LocalSTA = 2

    consts = types.ModuleType("go2_webrtc_driver.constants")
    consts.WebRTCConnectionMethod = Method
    driver = types.ModuleType("go2_webrtc_driver.webrtc_driver")
    driver.Go2WebRTCConnection = FakeConnection

    dc = types.ModuleType("go2_webrtc_driver.webrtc_datachannel")

    class WebRTCDataChannel:
        def deal_array_buffer_for_lidar(self, buffer):
            raise AssertionError("应被桥节点挂钩替换")

    dc.WebRTCDataChannel = WebRTCDataChannel

    mod.constants = consts
    mod.webrtc_driver = driver
    mod.webrtc_datachannel = dc
    return mod


def make_voxel_frame(n_bytes=4096, seed=7):
    rng = np.random.default_rng(seed)
    occ = (rng.random(n_bytes) > 0.98).astype(np.uint8).tobytes()
    meta = {"src_size": n_bytes, "origin": [0.5, -1.25, 0.0], "resolution": 0.05}
    hdr = json.dumps({"data": meta}).encode()
    blob = struct.pack("<I", len(hdr)) + b"\0\0\0\0" + hdr + \
        lz4.block.compress(occ, store_size=False)
    expected = int(np.unpackbits(np.frombuffer(occ, dtype=np.uint8)).sum())
    return blob, expected


# ---------------- 测试 ----------------

def main():
    sys.modules["go2_webrtc_driver"] = make_fake_driver_module()
    sys.modules["go2_webrtc_driver.constants"] = sys.modules["go2_webrtc_driver"].constants
    sys.modules["go2_webrtc_driver.webrtc_driver"] = sys.modules["go2_webrtc_driver"].webrtc_driver
    sys.modules["go2_webrtc_driver.webrtc_datachannel"] = sys.modules["go2_webrtc_driver"].webrtc_datachannel

    from baiyanggo_webrtc_bridge.bridge_node import WebrtcBridgeNode

    rclpy.init()
    node = WebrtcBridgeNode()

    got = {"pose": [], "cloud": [], "state": [], "status": []}
    test = Node("mock_verifier")
    test.create_subscription(PoseStamped, "/baiyanggo/webrtc/pose",
                             lambda m: got["pose"].append(m), 10)
    test.create_subscription(PointCloud2, "/baiyanggo/webrtc/voxel_map",
                             lambda m: got["cloud"].append(m), 10)
    test.create_subscription(String, "/baiyanggo/webrtc/lidar_state",
                             lambda m: got["state"].append(m), 10)
    test.create_subscription(String, "/baiyanggo/webrtc/status",
                             lambda m: got["status"].append(json.loads(m.data)), 10)

    spinner = threading.Thread(target=rclpy.spin, args=(test,), daemon=True)
    spinner.start()

    deadline = time.time() + 12
    frame, expected_pts = make_voxel_frame()
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.connected:
            ch = None
            # 找到 fake datachannel（经 bridge 内部 conn）
            # 直接通过挂钩方法注入体素帧
            from go2_webrtc_driver.webrtc_datachannel import WebRTCDataChannel
            hooked = WebRTCDataChannel.deal_array_buffer_for_lidar
            hooked(object(), frame)
            node._handle_pose = node._handle_pose  # noqa
            # 注入 pose/state 走 pub_sub 回调
            # （fake conn 的 datachannel 在 bridge 内部，从外部触发一次即可）
            break
    time.sleep(0.5)

    # 外部注入 pose/state（直接调 bridge 的 handler 等价于回调路径）
    node._handle_pose({"data": {
        "header": {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": "odom"},
        "pose": {"position": {"x": 1.5, "y": -0.25, "z": 0.05},
                 "orientation": {"x": 0, "y": 0, "z": 0.1, "w": 0.99}}}})
    node._handle_lidar_state({"data": {"error_state": 0}})

    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.2)
        time.sleep(0.2)

    ok_pose = len(got["pose"]) >= 1 and abs(got["pose"][0].pose.position.x - 1.5) < 1e-6
    ok_state = len(got["state"]) >= 1
    ok_cloud = (len(got["cloud"]) >= 1 and
                got["cloud"][0].width == expected_pts and
                got["cloud"][0].point_step == 12)
    ok_status = any(s.get("connected") for s in got["status"])

    print(f"pose: {'PASS' if ok_pose else 'FAIL'} ({len(got['pose'])}条)")
    print(f"lidar_state: {'PASS' if ok_state else 'FAIL'} ({len(got['state'])}条)")
    print(f"voxel_cloud: {'PASS' if ok_cloud else 'FAIL'} "
          f"({len(got['cloud'])}条, 点数={got['cloud'][0].width if got['cloud'] else 0}/期望{expected_pts})")
    print(f"status: {'PASS' if ok_status else 'FAIL'}")

    node.destroy_node()
    test.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all([ok_pose, ok_state, ok_cloud, ok_status]) else 1)


if __name__ == "__main__":
    main()
