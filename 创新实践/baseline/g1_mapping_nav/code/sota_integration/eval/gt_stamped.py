#!/usr/bin/env python3
"""gt_stamped.py — republish Gazebo GT (from /gt/pose TFMessage, stamp=0) as
PoseStamped on /gt/pose_stamped with a usable sim-time stamp (from /clock)."""
import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def parse_args(argv):
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="robot1_my_bot")
    p.add_argument("--in_topic", default="/gt/pose")
    p.add_argument("--out_topic", default="/gt/pose_stamped")
    p.add_argument("--mode", choices=["clock", "receipt"], default="clock")
    a, _ = p.parse_known_args(argv)
    return a


class GtStamped(Node):
    def __init__(self, args):
        super().__init__("gt_stamped")
        self.model = args.model.lstrip("/")
        self.mode = args.mode
        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PoseStamped, args.out_topic, qos)
        self.create_subscription(TFMessage, args.in_topic, self.on_pose, qos)
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.latest_clock = None
        self.count = 0
        self.last_stamp = None
        self._warned = False
        self.create_timer(5.0, self.on_summary)

    def on_clock(self, m):
        self.latest_clock = m.clock

    def on_pose(self, m):
        try:
            match = None
            seen = []
            for tf in m.transforms:
                child = (tf.child_frame_id or "").lstrip("/")
                seen.append(child)
                if child == self.model or child.endswith(self.model):
                    match = tf
                    break
            if match is None:
                if not self._warned:
                    self._warned = True
                    self.get_logger().warning("no match; children=%s" % seen)
                return
            out = PoseStamped()
            parent = (match.header.frame_id or "").lstrip("/")
            out.header.frame_id = parent if parent else "world"
            if self.mode == "clock" and self.latest_clock is not None:
                out.header.stamp = self.latest_clock
            else:
                out.header.stamp = self.get_clock().now().to_msg()
            t = match.transform.translation
            r = match.transform.rotation
            out.pose.position.x, out.pose.position.y, out.pose.position.z = t.x, t.y, t.z
            out.pose.orientation.x, out.pose.orientation.y = r.x, r.y
            out.pose.orientation.z, out.pose.orientation.w = r.z, r.w
            self.pub.publish(out)
            self.count += 1
            self.last_stamp = out.header.stamp
        except Exception as e:
            self.get_logger().error("gt_stamped: %s" % e)

    def on_summary(self):
        s = "n/a" if self.last_stamp is None else "%d.%09d" % (self.last_stamp.sec, self.last_stamp.nanosec)
        self.get_logger().info("gt_stamped | republished=%d last_stamp=%s" % (self.count, s))


def main():
    rclpy.init()
    a = parse_args(sys.argv[1:])
    n = GtStamped(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
