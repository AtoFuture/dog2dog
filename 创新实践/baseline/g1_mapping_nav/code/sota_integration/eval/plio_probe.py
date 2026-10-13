#!/usr/bin/env python3
# plio_probe.py DUR — 订阅 /aft_mapped_to_init，报告起点/终点/最大偏移（判发散）。
import rclpy, time, sys, threading, os
from rclpy.node import Node
from nav_msgs.msg import Odometry


class P(Node):
    def __init__(s):
        super().__init__('plio_probe')
        s.p = []
        s.create_subscription(Odometry, '/aft_mapped_to_init', s.cb, 50)

    def cb(s, m):
        p = m.pose.pose.position
        s.p.append((p.x, p.y, p.z))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = P()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    time.sleep(dur)
    if not n.p:
        print("NO_MSGS", flush=True)
        os._exit(0)
    mx = max(abs(x) for x, _, _ in n.p)
    my = max(abs(y) for _, y, _ in n.p)
    mz = max(abs(z) for _, _, z in n.p)
    print("samples=%d start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) max=(%.2f,%.2f,%.2f)" % (
        len(n.p), n.p[0][0], n.p[0][1], n.p[0][2],
        n.p[-1][0], n.p[-1][1], n.p[-1][2], mx, my, mz), flush=True)
    os._exit(0)   # avoid rclpy shutdown abort


main()
