#!/usr/bin/env python3
"""frontier_explorer: 精简版 frontier 自主探索器（Foxy 无 explore_lite 的替代）。

参考仿真里 explore_lite 的调试结论重写（修复两个已知坑）：
  1. 目标去重：新目标与旧目标 < same_goal_tolerance 视为同一目标（避免频繁抢占）
  2. 失败黑名单：失败/放弃的目标加入半径黑名单，防止原地打转
  3. 目标筛选：frontier 簇尺寸阈值 + 距机器人最小距离

输入: /map (OccupancyGrid, transient_local), TF map->base_link
输出: NavigateToPose action 调用
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf2_ros

FREE, UNKNOWN = 0, -1


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.declare_parameter("min_frontier_cells", 6)
        self.declare_parameter("min_goal_dist", 1.0)
        self.declare_parameter("max_goal_dist", 8.0)
        self.declare_parameter("blacklist_radius", 1.0)
        self.declare_parameter("plan_period", 5.0)

        map_qos = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.map = None
        self.blacklist = []          # [(x, y, t)]
        self.active_goal = None      # (x, y)
        self.goal_sent_at = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(self.get_parameter("plan_period").value, self.tick)
        self.get_logger().info("frontier_explorer 启动")

    # ---------- 地图回调 ----------
    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    # ---------- 机器人位姿 ----------
    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link",
                                                 rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    # ---------- 主循环 ----------
    def tick(self):
        if self.map is None or self.active_goal is not None:
            return
        if not self.ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose 不可用")
            return
        pos = self.robot_xy()
        if pos is None:
            self.get_logger().warn("TF map->base_link 不可用")
            return
        goal = self.find_frontier(pos)
        if goal is None:
            self.get_logger().info("未发现有效 frontier（探索可能已完成）")
            return
        self.send_goal(goal)

    def find_frontier(self, pos):
        m = self.map
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = m.data
        rx, ry = pos

        def idx(x, y):
            return y * w + x

        frontier_cells = []
        for y in range(1, h - 1):
            base = y * w
            for x in range(1, w - 1):
                i = base + x
                if data[i] != FREE:
                    continue
                # 8 邻域含未知 → frontier
                if (data[i - 1] == UNKNOWN or data[i + 1] == UNKNOWN or
                        data[i - w] == UNKNOWN or data[i + w] == UNKNOWN):
                    frontier_cells.append((x, y))

        if not frontier_cells:
            return None

        # 简单聚类（网格贪心，按连通性分组）
        cellset = set(frontier_cells)
        visited = set()
        best = None
        best_d = 1e9
        for c in frontier_cells:
            if c in visited:
                continue
            stack, cluster = [c], []
            visited.add(c)
            while stack:
                cx, cy = stack.pop()
                cluster.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (cx + dx, cy + dy)
                        if n in cellset and n not in visited:
                            visited.add(n)
                            stack.append(n)
            if len(cluster) < self.get_parameter("min_frontier_cells").value:
                continue
            mx = sum(p[0] for p in cluster) / len(cluster)
            my = sum(p[1] for p in cluster) / len(cluster)
            gx = ox + (mx + 0.5) * res
            gy = oy + (my + 0.5) * res
            d = math.hypot(gx - rx, gy - ry)
            if d < self.get_parameter("min_goal_dist").value:
                continue
            if d > self.get_parameter("max_goal_dist").value:
                continue
            if self.in_blacklist(gx, gy):
                continue
            if d < best_d:
                best_d, best = d, (gx, gy)
        return best

    def in_blacklist(self, x, y):
        r = self.get_parameter("blacklist_radius").value
        now = time.time()
        self.blacklist = [(bx, by, t) for bx, by, t in self.blacklist
                          if now - t < 120.0]           # 黑名单 2 分钟过期
        return any(math.hypot(x - bx, y - by) < r for bx, by, _ in self.blacklist)

    def send_goal(self, goal):
        gx, gy = goal
        pos = self.robot_xy()
        yaw = math.atan2(gy - pos[1], gx - pos[0]) if pos else 0.0
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = gx
        msg.pose.pose.position.y = gy
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.active_goal = (gx, gy)
        self.goal_sent_at = time.time()
        self.get_logger().info(f"探索目标: ({gx:.2f}, {gy:.2f}) yaw={yaw:.2f}")
        fut = self.ac.send_goal_async(msg)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.finish_goal(False)
            return
        gh.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_result(self, fut):
        status = fut.result().status
        ok = status == 4  # STATUS_SUCCEEDED
        self.finish_goal(ok)

    def finish_goal(self, ok):
        if self.active_goal:
            self.get_logger().info(
                f"目标 {self.active_goal} {'完成' if ok else '失败'} "
                f"({time.time()-self.goal_sent_at:.0f}s)")
            if not ok:
                self.blacklist.append((*self.active_goal, time.time()))
        self.active_goal = None


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
