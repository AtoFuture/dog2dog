#!/usr/bin/env python3
"""端到端验收：订阅 /detections_3d 与 /detections_snapshot，打印收到的消息。

用法（先起 detector_node 与 replay_images）::

    python3 tools/e2e_check.py --seconds 40
"""
import argparse, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from vision_interfaces.msg import Detection3D, DetectionSnapshot


class Checker(Node):
    def __init__(self):
        super().__init__("e2e_check")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=20)
        self.dets = []
        self.snaps = []
        self.create_subscription(Detection3D, "/detections_3d", self._on_det, q)
        self.create_subscription(DetectionSnapshot, "/detections_snapshot", self._on_snap, q)

    def _on_det(self, m):
        self.dets.append(m)
        p = m.pose.pose.position
        print(f"  Detection3D  {m.class_name:10s} conf={m.confidence:.2f} id={m.id} "
              f"map=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) snap={m.has_snapshot}")

    def _on_snap(self, m):
        self.snaps.append(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=40.0)
    args = ap.parse_args()

    rclpy.init()
    n = Checker()
    end = time.time() + args.seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)

    print()
    print(f"收到 {len(n.dets)} 条 Detection3D，{len(n.snaps)} 条 DetectionSnapshot")
    if n.dets:
        zs = [m.pose.pose.position.z for m in n.dets]
        names = {m.class_name for m in n.dets}
        print(f"类别: {sorted(names)}   z 范围: {min(zs):.3f} ~ {max(zs):.3f} m")
        if n.snaps:
            kb = [len(s.image.data) / 1024 for s in n.snaps]
            print(f"抓拍图大小: {min(kb):.1f} ~ {max(kb):.1f} KB（对比整帧 rgb8 的 921 KB）")
        print(">>> 端到端打通 ✅")
    else:
        print(">>> 没有收到任何检测消息 ❌")
    n.destroy_node(); rclpy.shutdown()
    return 0 if n.dets else 1


if __name__ == "__main__":
    sys.exit(main())
