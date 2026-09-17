#!/usr/bin/env python3
"""把磁盘上的彩色+深度图当成实时相机话题回放。

--------------------------------------------------------------------------------
为什么需要它

``detector_node`` 要的输入是 ROS2 话题，而手头只有一堆 png/npy 文件。
没有真实的相机、也没有仿真环境时，这个回放工具就是**唯一能把感知链路
端到端跑起来**的东西。

它比 rosbag 方便的地方：数据集是散文件（按时间戳配对的彩色/深度），
不用先转包；而且可以指定相机内参和 TF，把「相机装在哪」也一并模拟。

--------------------------------------------------------------------------------
它发布什么

* ``/camera/color/image_raw``   —— 彩色（bgr8）
* ``/camera/depth/image_raw``   —— 深度（**16UC1 毫米**，与 RealSense 一致）
* ``/camera/color/camera_info`` —— 内参（latch，晚启动也能拿到）
* 可选：静态 TF ``map → camera_color_optical_frame``

深度文件是 ``.npy``（uint16，单位毫米）；彩色是 ``.png``。
两者按文件名里的时间戳配对，与 ``tools/fetch_from_remote_zip.py`` 取的格式一致。

用法::

    ros2 run vision_detector replay_images --ros-args \\
        -p color_dir:=~/g2_testdata/low_angle_big \\
        -p depth_dir:=~/g2_testdata/depth_big \\
        -p fps:=5.0
"""

from __future__ import annotations

import glob
import os
import re
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

from .qos import LATCHED_QOS, SENSOR_QOS

TS_RE = re.compile(r"(\d{8})-(\d{6})_(\d{6})")


def _ts(name: str) -> float | None:
    m = TS_RE.search(name)
    if not m:
        return None
    _, t, us = m.groups()
    return int(t[:2]) * 3600 + int(t[2:4]) * 60 + int(t[4:6]) + int(us) / 1e6


class ImageReplayer(Node):
    def __init__(self) -> None:
        super().__init__("image_replayer")
        d = self.declare_parameter

        d("color_dir", "")
        d("depth_dir", "")
        d("fps", 5.0)
        d("loop", True)

        # RealSense D435i 在 640x480 下的典型值。
        # ⚠️ 这是**近似值**，只够把链路跑通；真实标定请从 CameraInfo 拿。
        d("fx", 615.0)
        d("fy", 615.0)
        d("cx", 320.0)
        d("cy", 240.0)

        # 是否广播一个静态 TF map -> camera
        d("publish_tf", True)
        d("map_frame", "map")
        d("camera_frame", "camera_color_optical_frame")
        # 相机相对 map 的位置（默认放在原点、离地 0.35 m，模拟 Go2 的安装高度）
        d("cam_x", 0.0)
        d("cam_y", 0.0)
        d("cam_z", 0.35)

        self._pairs = self._collect_pairs()
        if not self._pairs:
            raise RuntimeError("没有配到任何彩色/深度对，检查 color_dir / depth_dir")

        self.get_logger().info(f"配到 {len(self._pairs)} 对彩色/深度图")

        self._color_pub = self.create_publisher(Image, "/camera/color/image_raw", SENSOR_QOS)
        self._depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", SENSOR_QOS)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", LATCHED_QOS)

        if self.get_parameter("publish_tf").value:
            self._tf_bcast = StaticTransformBroadcaster(self)
            self._broadcast_tf()

        self._idx = 0
        period = 1.0 / max(1e-6, float(self.get_parameter("fps").value))
        self._timer = self.create_timer(period, self._tick)

    # ------------------------------------------------------------------
    def _collect_pairs(self):
        cdir = os.path.expanduser(self.get_parameter("color_dir").value)
        ddir = os.path.expanduser(self.get_parameter("depth_dir").value)
        if not cdir or not ddir:
            return []

        depth_by_ts = {}
        for f in sorted(glob.glob(os.path.join(ddir, "*.npy"))):
            t = _ts(os.path.basename(f))
            if t is not None:
                depth_by_ts[t] = f
        if not depth_by_ts:
            return []
        dts = np.array(sorted(depth_by_ts))

        pairs = []
        for f in sorted(glob.glob(os.path.join(cdir, "*.png"))):
            t = _ts(os.path.basename(f))
            if t is None:
                continue
            i = int(np.argmin(np.abs(dts - t)))
            if abs(dts[i] - t) <= 0.2:      # 两路独立流，容差 0.2 s
                pairs.append((f, depth_by_ts[float(dts[i])]))
        return pairs

    def _broadcast_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter("map_frame").value
        t.child_frame_id = self.get_parameter("camera_frame").value
        t.transform.translation.x = float(self.get_parameter("cam_x").value)
        t.transform.translation.y = float(self.get_parameter("cam_y").value)
        t.transform.translation.z = float(self.get_parameter("cam_z").value)

        # 相机光学系 → map 系的固定旋转。
        #
        # 光学系：x 右 / y 下 / z 前（沿光轴）
        # map 系（REP-103）：x 前 / y 左 / z 上
        #
        # 对应关系：
        #     光学 z(前) → map +x
        #     光学 x(右) → map -y
        #     光学 y(下) → map -z
        # 即旋转矩阵 R = [[0,0,1],[-1,0,0],[0,-1,0]]，trace=0，
        # 按标准公式换算得四元数 (x,y,z,w) = (0.5, -0.5, 0.5, -0.5)。
        #
        # ⚠️ 这里原先手写的是 (0.707, -0.707, 0, 0) —— 那是绕 (1,-1,0)/√2
        # 转 180°，会把光学"前"映射到 map 的 **-z（朝下）**。
        # 后果是三维点整体转了 90°，z 变成负的、位置全错，
        # 而且**不会报任何错** —— 看起来只是"坐标有点怪"。
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = -0.5
        self._tf_bcast.sendTransform(t)
        self.get_logger().info(
            f"已广播静态 TF {t.header.frame_id} -> {t.child_frame_id} "
            f"({t.transform.translation.x}, {t.transform.translation.y}, "
            f"{t.transform.translation.z})"
        )

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._idx >= len(self._pairs):
            if not self.get_parameter("loop").value:
                self.get_logger().info("回放完毕")
                return
            self._idx = 0

        cpath, dpath = self._pairs[self._idx]
        self._idx += 1

        # ⚠️ 单个文件坏掉**不能**让整个回放器崩掉。
        # 踩过的坑：数据集里有一个 0 字节的 .npy（下载中断留下的），
        # `np.load` 抛 EOFError 直接把节点干掉了 —— 而调用方看到的是
        # 「话题没数据」，会往完全错误的方向排查。
        try:
            color = cv2.imread(cpath)
            depth = np.load(dpath)
        except Exception as exc:
            self.get_logger().warn(
                f"跳过损坏的文件对 {os.path.basename(cpath)} / "
                f"{os.path.basename(dpath)}：{type(exc).__name__}: {exc}"
            )
            return
        if color is None or depth is None or depth.size == 0:
            self.get_logger().warn(
                f"跳过空文件对 {os.path.basename(cpath)} / {os.path.basename(dpath)}"
            )
            return

        now = self.get_clock().now().to_msg()

        cm = Image()
        cm.header.stamp = now
        cm.header.frame_id = self.get_parameter("camera_frame").value
        cm.height, cm.width = color.shape[:2]
        cm.encoding = "bgr8"
        cm.step = color.shape[1] * 3
        cm.data = color.tobytes()
        self._color_pub.publish(cm)

        dm = Image()
        dm.header.stamp = now
        dm.header.frame_id = cm.header.frame_id
        dm.height, dm.width = depth.shape[:2]
        dm.encoding = "16UC1"          # 毫米，与 RealSense 一致
        dm.step = depth.shape[1] * 2
        dm.data = depth.astype(np.uint16).tobytes()
        self._depth_pub.publish(dm)

        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = cm.header.frame_id
        info.width, info.height = cm.width, cm.height
        info.k = [
            float(self.get_parameter("fx").value), 0.0, float(self.get_parameter("cx").value),
            0.0, float(self.get_parameter("fy").value), float(self.get_parameter("cy").value),
            0.0, 0.0, 1.0,
        ]
        info.d = [0.0] * 5
        self._info_pub.publish(info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageReplayer()
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
