#!/usr/bin/env python3
"""G2 感知链路节点：图像 → 检测 → 三维坐标 → 发布接口 03 / 03b。

--------------------------------------------------------------------------------
数据流

    color/image ─┐
                 ├─► 时间同步 ─► 抽帧 ─► YOLO 检测 ─┐
    depth/image ─┘                                    │
                                                      ├─► 三维反投影（g2_core.projector）
    camera_info ─► 内参（只需一次）                     │
                                                      ├─► tf2: camera → map
    TF: map→camera ──────────────────────────────────┘
                                                      │
                                                      ├─► /detections_3d      (Detection3D)
                                                      └─► /detections_snapshot (抓拍图 ROI JPEG)

算法全在 ``g2_core`` 里，本模块只做胶水：订阅、同步、抽帧、调 tf2、打包消息。

--------------------------------------------------------------------------------
几个刻意的设计（都是踩过坑换来的，见 docs/框架规划.md）

1. **QoS 显式指定**（见 ``qos.py``）。默认的 RELIABLE 配相机话题的 BEST_EFFORT
   会**静默零帧**，不报错、只在启动打一行警告。

2. **TF 查询用图像时间戳**，而不是 ``Time(0)``（最新）。
   用最新的变换去变换一张几十毫秒前的图，目标位置会偏 —— 而 Go2 在移动，
   这个偏差是系统性的。查不到时**降级用最新**并计数，而不是丢弃整帧检测。

3. **抽帧**：相机 30 Hz，检测不需要 30 Hz。默认 5 Hz。
   ⚠️ 注意 Go2 原生前视相机是 15 fps 档，30 Hz 只在加装 D435i 时成立。

4. **按 track_id 去重**：同一个目标连续检出时按 ``min_publish_interval`` 节流，
   避免把「同一个人的 50 次检出」当成 50 个目标上报 —— 那会让 G3 的
   检出率/误检率统计彻底失真。

5. **深度与彩色必须尺寸一致**，否则直接报错而不是静默采错像素
   （``projector`` 里也有同样的断言，这里提前拦一道，报错信息更贴近现场）。

6. **``use_sim_time``**：仿真下必须由 launch 给本节点设 True，否则 TF 会报
   extrapolation。这里不做隐式处理 —— 时钟是环境的事，不该由节点猜。
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseWithCovariance
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from g2_core.detector import Detector, DetectorConfig, DetectorUnavailableError
from g2_core.projector import (
    CameraIntrinsics,
    ProjectionResult,
    depth_to_meters,
    project_depth_bbox,
)
from vision_interfaces.msg import Detection3D, DetectionSnapshot

from .qos import LATCHED_QOS, RESULT_QOS, SENSOR_QOS


class VisionDetector(Node):
    def __init__(self) -> None:
        super().__init__("vision_detector")
        self._declare_params()

        self._bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 内参：从 CameraInfo 拿一次就够（准静态）
        self._intrinsics: CameraIntrinsics | None = None
        # 去重台账：track_id -> 上次发布的时刻
        self._last_published: dict[int, float] = {}
        # 运行统计：用于周期性「为什么没出消息」的定位。
        # 没有它的话，「检测为 0」「三维解算全失败」「TF 全失败」在日志上
        # 长得一模一样，都是「什么都没发生」。
        self._stat = {"frames": 0, "dets": 0, "proj_fail": 0, "dedup": 0, "tf_fail": 0,
                      "published": 0}
        # TF 降级计数（用不到图像时间戳时 +1）
        self._tf_fallback_count = 0

        self._load_detector()

        # ---- 发布 ----
        self._det_pub = self.create_publisher(
            Detection3D, self.get_parameter("detections_topic").value, RESULT_QOS
        )
        self._snap_pub = self.create_publisher(
            DetectionSnapshot, self.get_parameter("snapshots_topic").value, RESULT_QOS
        )

        # ---- 订阅 ----
        # YOLO 推理是**同步阻塞**的，所以把它放进独立的 callback group，
        # 并用 MultiThreadedExecutor —— 否则推理期间 TF 回调、参数回调全被饿死。
        self._vision_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            LATCHED_QOS,
            callback_group=self._vision_group,
        )

        color_sub = Subscriber(
            self, Image, self.get_parameter("color_topic").value, qos_profile=SENSOR_QOS
        )
        depth_sub = Subscriber(
            self, Image, self.get_parameter("depth_topic").value, qos_profile=SENSOR_QOS
        )
        # 近似同步：彩色与深度是两路独立流，时间戳通常差几毫秒到几十毫秒。
        # slop 默认 0.05 s —— 这个值直接进三维坐标误差：
        # Go2 行走约 1 m/s，50 ms 就是 5 cm。跑得快/转得快时应当调小。
        self._sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=10,
            slop=self.get_parameter("sync_slop_s").value,
        )
        self._sync.registerCallback(self._on_images)

        self.get_logger().info(
            f"感知链路就绪：抽帧 {self.get_parameter('detect_hz').value} Hz ｜ "
            f"彩色 {self.get_parameter('color_topic').value} ｜ "
            f"深度 {self.get_parameter('depth_topic').value}"
        )
        self.get_logger().info(
            "⚠️ 仿真下记得给本节点设 use_sim_time:=true，否则 TF 会报 extrapolation"
        )
        # 每 10 秒报一次处理统计 —— 出问题时一眼能看出卡在哪一步
        self.create_timer(10.0, self._report_stats)

    def _report_stats(self) -> None:
        st = self._stat
        if st["frames"] == 0:
            return
        self.get_logger().info(
            f"[统计] 处理 {st['frames']} 帧 ｜ 检测 {st['dets']} 个 ｜ "
            f"三维失败 {st['proj_fail']} ｜ 去重跳过 {st['dedup']} ｜ "
            f"TF 失败 {st['tf_fail']} ｜ **已发布 {st['published']}**"
        )
        if st["published"] == 0 and st["dets"] > 0:
            # 有检出却没有发布 —— 必须说清楚是哪一步拦下的
            self.get_logger().warn(
                "有检测但一条都没发出去。看上面的分解："
                "proj_fail 高 -> 深度不可信；tf_fail 高 -> TF 树没建好；"
                "dedup 高 -> 节流间隔设得太长。"
            )

    # ------------------------------------------------------------------
    def _declare_params(self) -> None:
        d = self.declare_parameter

        # --- 话题 ---
        d("color_topic", "/camera/color/image_raw")
        d("depth_topic", "/camera/depth/image_raw")
        d("camera_info_topic", "/camera/color/camera_info")
        d("detections_topic", "/detections_3d")
        d("snapshots_topic", "/detections_snapshot")

        # --- 检测 ---
        d("weights", "yolo26n.pt")
        d("conf", 0.25)
        d("imgsz", 640)
        d("device", "cpu")
        d("detect_hz", 5.0)
        d("sync_slop_s", 0.05)

        # --- 三维解算 ---
        d("anchor", "center")     # center | bottom
        d("max_dispersion_m", 3.0)
        d("min_valid_pixels", 10)

        # --- 坐标变换 ---
        d("map_frame", "map")
        d("tf_timeout_s", 0.05)
        d("tf_fallback_to_latest", True)

        # --- 去重与抓拍 ---
        d("min_publish_interval_s", 2.0)
        d("publish_snapshot", True)
        d("snapshot_quality", 80)
        d("snapshot_margin_px", 8)

    def _load_detector(self) -> None:
        """加载检测器。**启动时就加载**，让配置错误立刻暴露。

        不惰性加载的理由：惰性加载会把「权重文件不存在」「类别白名单一个都没
        匹配上」这类错误推迟到第一帧图像到达时才抛 —— 而那时候往往还伴随着
        别的问题（话题不对、QoS 不匹配），混在一起更难定位。
        """
        try:
            self._detector = Detector(
                DetectorConfig(
                    weights=self.get_parameter("weights").value,
                    conf=self.get_parameter("conf").value,
                    imgsz=self.get_parameter("imgsz").value,
                    device=self.get_parameter("device").value,
                )
            )
        except DetectorUnavailableError as exc:
            self.get_logger().fatal(f"检测器加载失败，节点无法工作：\n{exc}")
            raise

    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._intrinsics is not None:
            return
        self._intrinsics = CameraIntrinsics.from_camera_info(msg.k)
        self.get_logger().info(
            f"内参已就位：fx={self._intrinsics.fx:.1f} fy={self._intrinsics.fy:.1f} "
            f"cx={self._intrinsics.cx:.1f} cy={self._intrinsics.cy:.1f}"
        )
        if abs(msg.d[0]) > 1e-6:
            # 有畸变系数却没去畸变 —— 三维坐标会系统性偏移，尤其画面边缘。
            self.get_logger().warn(
                f"CameraInfo 报告了非零畸变系数（d[0]={msg.d[0]:.4f}），"
                f"但本节点喂给针孔模型的是**未去畸变**的图。"
                f"请在上游接 image_proc / image_undistort，或用去畸变后的话题。"
            )

    # ------------------------------------------------------------------
    def _on_images(self, color_msg: Image, depth_msg: Image) -> None:
        # 第一对图像到达时报一次 —— 这是判断「相机链路通了没」最直接的信号。
        # 没有它的话，「订阅建了但回调不触发」（QoS 不匹配、同步 slop 太小、
        # 时间戳对不上）和「回调跑了但下游失败」这两种情况从日志上分不出来。
        if not getattr(self, "_logged_first_frame", False):
            self._logged_first_frame = True
            self.get_logger().info(
                f"收到第一对同步图像：彩色 {color_msg.width}x{color_msg.height} "
                f"深度 {depth_msg.width}x{depth_msg.height}（{depth_msg.encoding}）"
            )

        # --- 抽帧 ---
        now = time.monotonic()
        period = 1.0 / max(1e-6, float(self.get_parameter("detect_hz").value))
        if now - getattr(self, "_last_detect", 0.0) < period:
            return
        self._last_detect = now

        if self._intrinsics is None:
            self.get_logger().warn(
                "还没收到 CameraInfo，无法做三维解算 —— 跳过本帧",
                throttle_duration_sec=5.0,
            )
            return

        # --- 转图 ---
        try:
            color = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            # 深度用 passthrough：保持 16UC1 / 32FC1 原样，
            # 由 depth_to_meters 按 encoding 归一化（搞错就是 1000 倍误差）
            depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"图像转换失败：{exc}", throttle_duration_sec=5.0)
            return

        if depth_raw.shape[:2] != color.shape[:2]:
            self.get_logger().error(
                f"深度图 {depth_raw.shape[:2]} 与彩色图 {color.shape[:2]} 尺寸不一致。"
                f"三维解算无法进行 —— 请在上游做深度对齐（align_depth:=true）。",
                throttle_duration_sec=10.0,
            )
            return

        try:
            depth_m = depth_to_meters(depth_raw, depth_msg.encoding)
        except Exception as exc:
            self.get_logger().error(f"深度编码处理失败：{exc}", throttle_duration_sec=10.0)
            return

        # --- 检测 ---
        # track 而不是 detect：拿到稳定的 track_id，才能做去重。
        detections = self._detector.track(color)
        self._stat["frames"] += 1
        self._stat["dets"] += len(detections)

        # 抓拍图要从原图上裁 ROI，这里存一下当前帧。
        # （脏但直接；抓拍只发一次/目标，不需要保留历史帧。）
        self._last_color = color

        # --- 逐目标处理 ---
        stamp = color_msg.header.stamp
        for det in detections:
            res = project_depth_bbox(
                depth_m,
                self._intrinsics,
                det.bbox_xyxy,
                mask=det.mask,
                min_valid_pixels=self.get_parameter("min_valid_pixels").value,
                max_dispersion_m=self.get_parameter("max_dispersion_m").value,
                anchor=self.get_parameter("anchor").value,
            )
            if res is None:
                # 深度不可信（被遮挡、框里混了远近两个面）—— 丢弃而不是硬报一个坐标
                self._stat["proj_fail"] += 1
                continue

            if not self._should_publish(det.track_id, now):
                self._stat["dedup"] += 1
                continue

            pose = self._to_map(res, color_msg.header.frame_id, stamp)
            if pose is None:
                self._stat["tf_fail"] += 1
                continue
            self._stat["published"] += 1

            self._publish_detection(det, pose, stamp)

    # ------------------------------------------------------------------
    def _should_publish(self, track_id: int | None, now: float) -> bool:
        """按 track_id 节流。

        ⚠️ 不去重的后果：同一个人站在那 10 秒、5 Hz 检出 → 50 条 Detection3D。
        G3 按「正确检出数 ÷ 全部检出数」算误检率时会被彻底带偏。
        """
        if track_id is None:
            return True     # 没跟踪信息时不节流（宁多报也不能漏）
        interval = float(self.get_parameter("min_publish_interval_s").value)
        last = self._last_published.get(track_id)
        if last is not None and now - last < interval:
            return False
        self._last_published[track_id] = now
        return True

    def _to_map(self, res: ProjectionResult, camera_frame: str, stamp) -> Pose | None:
        """把相机系三维点变换到 map 系。

        用**图像时间戳**查 TF，而不是 ``Time(0)``（最新）：Go2 在移动，
        用最新的变换去变换几十毫秒前的图会引入系统性偏差。
        查不到时降级用最新并计数，而不是丢弃整帧检测 —— 丢帧同样会让指标失真。
        """
        from tf2_geometry_msgs import do_transform_point  # 局部导入，减少节点启动时间
        from geometry_msgs.msg import PointStamped

        map_frame = self.get_parameter("map_frame").value
        timeout = Duration(seconds=self.get_parameter("tf_timeout_s").value)

        p = PointStamped()
        p.header.frame_id = camera_frame
        p.header.stamp = stamp
        p.point = Point(x=float(res.point[0]), y=float(res.point[1]), z=float(res.point[2]))

        tf = None
        try:
            tf = self.tf_buffer.lookup_transform(map_frame, camera_frame, stamp, timeout)
        except TransformException as exc:
            if not self.get_parameter("tf_fallback_to_latest").value:
                self.get_logger().warn(
                    f"在图像时间戳上查不到 {map_frame}<-{camera_frame} 的变换：{exc}",
                    throttle_duration_sec=5.0,
                )
                self._stat["tf_fail"] += 1
                return None
            try:
                tf = self.tf_buffer.lookup_transform(map_frame, camera_frame, rclpy.time.Time())
                self._tf_fallback_count += 1
                if self._tf_fallback_count % 50 == 1:
                    self.get_logger().warn(
                        f"TF 降级到「最新」已 {self._tf_fallback_count} 次。"
                        f"目标位置会有系统性偏差（Go2 在移动）。"
                        f"若频繁出现，检查 TF 发布频率与 use_sim_time 设置。"
                    )
            except TransformException as exc2:
                self.get_logger().warn(
                    f"TF 完全查不到 {map_frame}<-{camera_frame}：{exc2}",
                    throttle_duration_sec=5.0,
                )
                return None

        q = do_transform_point(p, tf)
        pose = Pose()
        pose.position.x = q.point.x
        pose.position.y = q.point.y
        pose.position.z = q.point.z
        pose.orientation.w = 1.0     # 契约规定：不使用，恒为单位四元数
        return pose

    def _publish_detection(self, det, pose: Pose, stamp) -> None:
        msg = Detection3D()
        msg.class_name = det.class_name
        msg.confidence = float(det.confidence)
        msg.pose = PoseWithCovariance()
        msg.pose.pose = pose
        # 协方差一律置零 —— 按 ROS 惯例表示「未知」。
        # 我们**无法**给出有意义的测量协方差：从检测框反投影得到的像素离散
        # 衡量的是物体自身尺寸，不是测量误差。见 Detection3D.msg 的说明。
        msg.stamp = stamp
        msg.id = int(det.track_id) if det.track_id is not None else 0
        msg.source = "coco"
        msg.has_snapshot = bool(self.get_parameter("publish_snapshot").value)

        self._det_pub.publish(msg)

        if msg.has_snapshot:
            self._publish_snapshot(det, msg.id, stamp)

    def _publish_snapshot(self, det, track_id: int, stamp) -> None:
        """发 ROI 裁剪 + JPEG 的抓拍图。

        只发 ROI 不发整帧：整帧 rgb8 是 0.92 MB/条，会把这个话题变成链路上
        最重的，且超 1 MB 后 DDS 分片丢包会连累整条检测消息（见接口 03b 说明）。
        裁剪后典型 5-20 KB，小两个数量级。
        """
        color = getattr(self, "_last_color", None)
        if color is None:
            return

        margin = int(self.get_parameter("snapshot_margin_px").value)
        h, w = color.shape[:2]
        x1, y1, x2, y2 = (int(round(v)) for v in det.bbox_xyxy)
        x1, y1 = max(0, x1 - margin), max(0, y1 - margin)
        x2, y2 = min(w, x2 + margin), min(h, y2 + margin)
        if x2 <= x1 or y2 <= y1:
            return

        roi = color[y1:y2, x1:x2]
        ok, buf = cv2.imencode(
            ".jpg",
            roi,
            [int(cv2.IMWRITE_JPEG_QUALITY),
             int(self.get_parameter("snapshot_quality").value)],
        )
        if not ok:
            return

        snap = DetectionSnapshot()
        snap.id = track_id
        snap.stamp = stamp
        snap.image.format = "jpeg"
        snap.image.data = buf.tobytes()
        self._snap_pub.publish(snap)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = VisionDetector()
    except Exception:
        rclpy.shutdown()
        raise

    # 必须多线程：YOLO 推理是同步阻塞的，单线程 executor 下
    # 推理期间 TF 回调、参数服务全被饿死。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
