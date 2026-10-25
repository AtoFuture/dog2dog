#!/usr/bin/env python3
# explore_watch.py - terminate-by-coverage monitor (replaces "drive N minutes blindly").
# Subscribes: /<ns>/map, /<ns>/explore/frontiers, /<ns>/odometry/filtered[, /<ns>/explore/status]
# Declares EXPLORATION_COMPLETE when frontier count stays 0 for --zero-cycles cycles.
import argparse
import csv
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray, Marker

try:
    from explore_lite_msgs.msg import ExploreStatus
    _HAS_STATUS = True
except Exception:
    _HAS_STATUS = False

MARKER_POINTS = Marker.POINTS
MARKER_ADD = Marker.ADD


def count_clusters(mask):
    if not mask.any():
        return 0
    try:
        from scipy import ndimage
        _, n = ndimage.label(mask, structure=np.ones((3, 3), dtype=np.int32))
        return int(n)
    except Exception:
        h, w = mask.shape
        seen = np.zeros_like(mask, dtype=bool)
        n = 0
        for r, c in np.argwhere(mask):
            r, c = int(r), int(c)
            if seen[r, c]:
                continue
            n += 1
            q = deque([(r, c)])
            seen[r, c] = True
            while q:
                y, x = q.popleft()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            q.append((ny, nx))
        return n


class ExploreWatcher(Node):
    def __init__(self, a):
        super().__init__("explore_watch")
        self.a = a
        self.ns = a.namespace.strip("/")
        self.map_msg = None
        self.map_stamp = 0.0
        self.frontier_markers = 0
        self.saw_marker_array = False
        self.status = ""
        self.rx = self.ry = float("nan")
        self.known_pct = self.free_pct = self.occ_pct = 0.0
        self.frontier_clusters = self.frontier_cells = 0
        self.t0 = time.time()
        self.zero_run = 0
        self.saw_nonzero = False
        self.last_move = None
        self.no_progress = False
        self.complete = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, f"/{self.ns}/map", self.on_map, qos)
        self.create_subscription(MarkerArray, f"/{self.ns}/explore/frontiers", self.on_mark, 10)
        self.create_subscription(Odometry, f"/{self.ns}/odometry/filtered", self.on_odom, 10)
        if _HAS_STATUS:
            self.create_subscription(ExploreStatus, f"/{self.ns}/explore/status", self.on_status, 10)

        new = not os.path.exists(a.csv)
        self.f = open(a.csv, "a", newline="")
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(["wall_time", "sim_time", "elapsed_s", "known_pct", "free_pct",
                             "occ_pct", "frontier_markers", "frontier_clusters", "frontier_cells",
                             "robot_x", "robot_y", "status", "event"])
            self.f.flush()
        self.create_timer(1.0 / a.rate, self.tick)

    def on_map(self, m):
        self.map_msg = m
        self.map_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        w, h = m.info.width, m.info.height
        if w * h != len(m.data):
            return
        d = np.asarray(m.data, dtype=np.int16).reshape((h, w))
        unk = d < 0
        free = d == 0
        occ = d == 100
        t = d.size
        self.known_pct = 100.0 * (~unk).sum() / t
        self.free_pct = 100.0 * free.sum() / t
        self.occ_pct = 100.0 * occ.sum() / t
        pf = np.pad(free, 1)
        nbf = (pf[:-2, 1:-1] | pf[2:, 1:-1] | pf[1:-1, :-2] | pf[1:-1, 2:])
        cand = unk & nbf
        self.frontier_cells = int(cand.sum())
        self.frontier_clusters = count_clusters(cand)

    def on_mark(self, m):
        self.saw_marker_array = True
        self.frontier_markers = sum(1 for x in m.markers
                                    if x.action == MARKER_ADD and x.type == MARKER_POINTS)

    def on_status(self, m):
        self.status = m.status

    def on_odom(self, m):
        self.rx = m.pose.pose.position.x
        self.ry = m.pose.pose.position.y

    def eff_frontiers(self):
        return self.frontier_markers if self.saw_marker_array else self.frontier_clusters

    def tick(self):
        if self.complete:
            return
        now = time.time()
        elapsed = now - self.t0
        ev = ""
        if self.map_msg is None:
            ev = "waiting_for_map"
        else:
            fr = self.eff_frontiers()
            if fr > 0 or self.frontier_cells > 0:
                self.saw_nonzero = True
            if not (np.isnan(self.rx) or np.isnan(self.ry)):
                if self.last_move is None:
                    self.last_move = (self.rx, self.ry, now)
                lx, ly, lt = self.last_move
                if np.hypot(self.rx - lx, self.ry - ly) > self.a.stall_dist:
                    self.last_move = (self.rx, self.ry, now)
                    self.no_progress = False
                elif self.saw_nonzero and fr > 0 and (now - lt) > self.a.stall_window and not self.no_progress:
                    ev = "NO_PROGRESS"
                    self.no_progress = True
                    self.get_logger().warn(f"NO_PROGRESS frontier={fr}")
            if fr == 0 and self.frontier_cells == 0:
                self.zero_run += 1
            else:
                self.zero_run = 0
            if self.a.status_complete and self.status == "exploration_complete":
                self.zero_run = max(self.zero_run, self.a.zero_cycles)
            if (self.saw_nonzero or elapsed >= self.a.warmup) and self.zero_run >= self.a.zero_cycles:
                self.complete = True
                ev = "EXPLORATION_COMPLETE"
                self.get_logger().info(f"*** EXPLORATION COMPLETE *** known={self.known_pct:.1f}%")
        self.w.writerow([f"{now:.3f}", f"{self.map_stamp:.3f}", f"{elapsed:.1f}",
                         f"{self.known_pct:.2f}", f"{self.free_pct:.2f}", f"{self.occ_pct:.2f}",
                         self.frontier_markers, self.frontier_clusters, self.frontier_cells,
                         "" if np.isnan(self.rx) else f"{self.rx:.3f}",
                         "" if np.isnan(self.ry) else f"{self.ry:.3f}", self.status, ev])
        self.f.flush()
        if self.complete:
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="robot1")
    ap.add_argument("--csv", default=os.path.expanduser("~/explore_watch.csv"))
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--zero-cycles", type=int, default=15)
    ap.add_argument("--warmup", type=float, default=30.0)
    ap.add_argument("--stall-window", type=float, default=25.0)
    ap.add_argument("--stall-dist", type=float, default=0.10)
    ap.add_argument("--status-complete", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = ExploreWatcher(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            n.f.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
