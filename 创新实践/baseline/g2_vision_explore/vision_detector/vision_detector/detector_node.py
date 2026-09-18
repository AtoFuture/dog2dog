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

import math
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

from g2_core.anomaly import keypoints_to_torso_3d, torso_tilt_from_vertical
from g2_core.detector import Detector, DetectorConfig, DetectorUnavailableError
from g2_core.fall_tracker import FallTracker, TorsoObservation
from g2_core.projector import (
    CameraIntrinsics,
    ProjectionResult,
    depth_to_meters,
    project_depth_bbox,
    rotate_translate,
)
from vision_interfaces.msg import Detection3D, DetectionSnapshot

from .qos import LATCHED_QOS, RESULT_QOS, SENSOR_QOS

#: map 系（REP-103，重力对齐）里的「上」。
#: 注意这**只对 map 系成立** —— 相机光学系里的「上」是 (0,-1,0)。
UP_IN_MAP = np.array([0.0, 0.0, 1.0])


def build_fall_detection(ev, stamp, map_point, confidence: float, has_snapshot: bool):
    """由倒地事件构造接口 03 的消息。

    **抽成模块级纯函数是为了可测** —— 它承载的是契约里最容易出错的那几条
    （边沿触发、onset_stamp 语义、两个原始观测量的单位），
    而这些既不该靠「跑一整条 ROS 链路」来验证，也不该埋在节点方法里。

    契约要点（见 baseline/README.md 的「异常类三条附加规则」）：

    * ``class_name`` 是异常类，``source="pose"``
    * ``id`` **不允许为 0** —— 时间判据依赖轨迹连续性，轨迹没确认时宁可不发
    * ``onset_stamp`` 是**事件起始时刻**，不是发送时刻也不是本帧采集时刻
    * ``torso_tilt_deg`` / ``torso_height_m`` 是**原始观测量**，供 G3 在录好的
      bag 上重扫阈值
    * ``pose`` 填躯干中点（map 系）—— 与普通检测的「代表点」口径**不同**
    """
    if ev.track_id == 0:
        raise ValueError("异常类不允许 id=0：时间判据依赖轨迹连续性，"
                         "轨迹未确认时不应产生倒地事件")

    msg = Detection3D()
    msg.class_name = "person_fallen"
    msg.confidence = float(confidence)

    msg.pose = PoseWithCovariance()
    msg.pose.pose.position.x = float(map_point[0])
    msg.pose.pose.position.y = float(map_point[1])
    msg.pose.pose.position.z = float(map_point[2])
    msg.pose.pose.orientation.w = 1.0        # 契约：不使用，恒为单位四元数

    msg.stamp = stamp
    msg.id = int(ev.track_id)
    msg.source = "pose"
    msg.has_snapshot = bool(has_snapshot)

    msg.onset_stamp.sec = int(math.floor(ev.onset_stamp))
    msg.onset_stamp.nanosec = int(round((ev.onset_stamp - math.floor(ev.onset_stamp)) * 1e9))
    if msg.onset_stamp.nanosec >= 1_000_000_000:      # 浮点误差可能凑到 1e9
        msg.onset_stamp.sec += 1
        msg.onset_stamp.nanosec -= 1_000_000_000
    msg.torso_tilt_deg = float(ev.tilt_deg)
    msg.torso_height_m = float("nan") if ev.height_m is None else float(ev.height_m)
    return msg


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
                      "published": 0,
                      # 倒地判据的分解。和上面同理：没有它的话
                      # 「没检出人」「关键点置信度低」「深度取不到」「人体尺度不合格」
                      # 「轨迹还没确认」在日志上全是「没有倒地结论」，分不出是哪一步。
                      "fall_seen": 0, "fall_no_keypoints": 0, "fall_no_track": 0,
                      "fall_unusable": 0, "fall_events": 0, "fall_tf_fail": 0,
                      "fall_stamp_regress": 0}
        # TF 降级计数（用不到图像时间戳时 +1）
        self._tf_fallback_count = 0

        self._fall_enabled = bool(self.get_parameter("detect_fall").value)
        _max_h = float(self.get_parameter("fall_max_height_m").value)
        self._fall_tracker = FallTracker(
            transition_max_s=float(self.get_parameter("fall_transition_max_s").value),
            persist_s=float(self.get_parameter("fall_persist_s").value),
            refire_cooldown_s=float(self.get_parameter("fall_refire_cooldown_s").value),
            max_height_m=(_max_h if _max_h > 0.0 else None),
        )
        if self._fall_enabled:
            self.get_logger().info(
                f"倒地识别已启用（时间判据）："
                f"转水平窗口 {self.get_parameter('fall_transition_max_s').value}s ｜ "
                f"保持 {self.get_parameter('fall_persist_s').value}s ｜ "
                f"高度门 {'关闭（未标定）' if _max_h <= 0 else f'{_max_h} m'}"
            )
            self.get_logger().info(
                "⚠️ 倒地识别只在权重为 pose 模型时生效；"
                "另外它依赖跟踪 ID 跨帧稳定，纯 detect 模式下没有 id 就不会出结论"
            )

        self._load_detector()

        # ---- 发布 ----
        self._det_pub = self.create_publisher(
            Detection3D, self.get_parameter("detections_topic").value, RESULT_QOS
        )
        self._snap_pub = self.create_publisher(
            DetectionSnapshot, self.get_parameter("snapshots_topic").value, RESULT_QOS
        )

        # ---- 订阅 ----
        # ⚠️ 关于 callback group，注释一度与实际不符（2026-09-18 审核 P3）。
        #
        # 原注释写「把 YOLO 推理放进独立 callback group …… 否则推理期间 TF 回调
        # 会被饿死」。实情是：
        #
        #   * 真正阻塞的**图像同步订阅**用的是节点**默认**的 MutuallyExclusive 组，
        #     只有下面的 CameraInfo 订阅进了 _vision_group；
        #   * 而 **tf2_ros 自带 ReentrantCallbackGroup**，所以 TF 不会被饿死 ——
        #     原注释担心的那件事并不会发生。
        #
        # 实际后果只有两条，都不严重：推理期间**统计定时器**和 set_parameters
        # 服务请求会被挡住。留着这个组是因为它确实把 CameraInfo 的回调隔开了，
        # 但**不要**照着旧注释去理解这里的行为。
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

        # ⚠️ 这一行的触发条件**不能只看 fall_seen**。
        #
        # 原先写的是 `if ... and st["fall_seen"]`，而 TF 完全查不到时
        # fall_seen 恒为 0（`_assess_fall` 在喂进 tracker 之前就 return 了）
        # → **整行都不打印**。运维看到的是「什么都没发生」，
        # 和「场景里没人」长得一模一样，而这两件事的处理方式完全不同。
        if self._fall_enabled and any(
            st[k] for k in ("fall_seen", "fall_no_keypoints", "fall_no_track",
                            "fall_unusable", "fall_events", "fall_tf_fail",
                            "fall_stamp_regress")
        ):
            self.get_logger().info(
                f"[倒地] 可用躯干 {st['fall_seen']} ｜ 无关键点 {st['fall_no_keypoints']} ｜ "
                f"轨迹未确认 {st['fall_no_track']} ｜ TF 失败 {st['fall_tf_fail']} ｜ "
                f"躯干不可用 {st['fall_unusable']} ｜ "
                f"**事件 {st['fall_events']}** ｜ 在用轨迹 {self._fall_tracker.n_tracks}"
            )
            # ⚠️ 这里的条件是「不可用占了一半以上」。
            #
            # 原先写的是 `fall_unusable > fall_seen` —— 而 unusable 只在
            # `fall_seen += 1` **之后**才可能自增，所以恒有 unusable <= seen，
            # **条件永远不成立**：这是一条死代码。
            # 后果是倒地链路最主流的失效模式（深度取到背景 → 躯干不可用，
            # 实测占 74%）恰恰没有任何自动提示。
            if st["fall_events"] == 0 and st["fall_unusable"] * 2 > st["fall_seen"]:
                self.get_logger().warn(
                    f"躯干大多不可用（{st['fall_unusable']}/{st['fall_seen']}）"
                    "—— 看 debug 日志里的原因。"
                    "最常见的是「关键点落到身体轮廓外，深度取到了背景」，"
                    "表现为反投影超出人体尺度。"
                )
            if st["fall_stamp_regress"]:
                self.get_logger().warn(
                    f"时间戳回退了 {st['fall_stamp_regress']} 次，那些帧的判据输入被丢弃。"
                )
            if st["fall_tf_fail"]:
                self.get_logger().warn(
                    f"有 {st['fall_tf_fail']} 次因为查不到 TF 而**整帧跳过了倒地判据**。"
                    "时间判据完全依赖连续观测，丢帧会让它失效 —— "
                    "查 map 帧是否建好、TF 频率、以及 use_sim_time。"
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
        # 输入短边小于它时先放大（INTER_LANCZOS4 整数倍）再推理，0 = 关闭。
        # 理由：低分辨率输入上**躺姿的置信度会塌到 conf 以下**，
        # 而「躺下」正是要检测的事件 —— 静默丢帧，下游永远收不到结论。
        # Kinect 320x240 → 放大 2x；640x480 输入下倍数 1，行为不变。
        # ⚠️ 放在 conf 旁边是因为这两条**必须一起看**：
        # 抬 conf 会把人丢掉，而放大是把人找回来。详见 DetectorConfig.upscale_min_side。
        d("upscale_min_side", 480)
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

        # --- 倒地识别（时间判据）---
        # 只有当权重是 **pose 模型**（能出关键点）时才会真正生效；
        # 纯检测模型下 det.keypoints 是 None，这一段自动跳过。
        d("detect_fall", True)
        d("fall_min_keypoint_conf", 0.5)
        d("fall_transition_max_s", 3.0)
        d("fall_persist_s", 2.0)
        d("fall_refire_cooldown_s", 10.0)
        # 离地高度上限。**0 = 关闭**（当前默认）——
        # 实测标定不出来：真倒地一侧量出过 1.58 m 这种不可能的值。
        # 没标定过的门只会误杀真倒地，所以宁可不关。
        d("fall_max_height_m", 0.0)
        d("fall_track_max_age_s", 30.0)

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
                    upscale_min_side=self.get_parameter("upscale_min_side").value,
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
        # TF 每帧查一次就够：同一帧里所有目标用的是同一个变换。
        # （原先每个目标各查一次，既浪费又让 tf_fail 计数变得难解释。）
        tf = self._lookup_tf(color_msg.header.frame_id, stamp)

        for det in detections:
            # ⚠️ 倒地判据**每帧都喂**，不受下面的 min_publish_interval_s 节流影响。
            # 节流是给 Detection3D 去重用的；而时间判据要的是**连续观测序列**，
            # 按节流间隔（默认 2 s）喂进去的话，3 s 的转水平窗口里只有一两个样本，
            # 判据直接失效。
            if self._fall_enabled:
                self._assess_fall(det, depth_m, tf, stamp)

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

            if tf is None:
                self._stat["tf_fail"] += 1
                continue

            self._stat["published"] += 1
            self._publish_detection(det, self._to_map(res, tf), stamp)

    # ------------------------------------------------------------------
    # 倒地识别
    # ------------------------------------------------------------------
    def _assess_fall(self, det, depth_m: np.ndarray, tf, stamp) -> None:
        """把一帧观测喂给时间判据，确认倒地时发一条事件。

        和 ``project_depth_bbox`` 那条路**完全独立**：它只用关键点，
        不用代表点反投影，所以不会因为 bbox 里混了远近两个面而被丢帧。

        每一道检查失败都单独计数 —— 没有这些计数的话，
        「没检出人」「置信度低」「深度取不到」「人体尺度不合格」「轨迹没确认」
        在日志上全是「没有倒地结论」，出问题时无从下手。
        """
        if det.keypoints is None:
            # 权重不是 pose 模型，或者模型没输出关键点
            self._stat["fall_no_keypoints"] += 1
            return
        if det.track_id is None:
            # 契约：异常类**不允许** id=0（时间判据依赖轨迹连续性）。
            # 轨迹还没被跟踪器确认时不发结论 —— 宁可晚一点，也不要发一个
            # G3 无法与其它帧关联的消息。
            self._stat["fall_no_track"] += 1
            return
        if tf is None:
            self._stat["fall_tf_fail"] += 1
            return

        self._stat["fall_seen"] += 1

        torso, why = keypoints_to_torso_3d(
            det.keypoints, depth_m, self._intrinsics,
            min_keypoint_conf=float(self.get_parameter("fall_min_keypoint_conf").value),
        )
        if torso is None:
            self._stat["fall_unusable"] += 1
            self.get_logger().debug(
                f"轨迹 {det.track_id} 这一帧的躯干不可用：{why}"
            )
            return

        # 相机系 -> map 系。map 系按 REP-103 重力对齐，且约定的地面是 z=0。
        # ⚠️ 「z=0 是地面」依赖 SLAM 的地图原点建在地面上 —— 真机上要确认这一点，
        # 否则 torso_height_m 会带一个常数偏置。
        q, t = tf.transform.rotation, tf.transform.translation
        quat = (q.x, q.y, q.z, q.w)
        trans = (t.x, t.y, t.z)
        pts = [rotate_translate(p, quat, trans) for p in
               (torso.shoulder_left, torso.shoulder_right,
                torso.hip_left, torso.hip_right)]
        s_mid = (pts[0] + pts[1]) / 2.0
        h_mid = (pts[2] + pts[3]) / 2.0

        tilt = math.degrees(torso_tilt_from_vertical(s_mid, h_mid, UP_IN_MAP))
        height = float((s_mid[2] + h_mid[2]) / 2.0)
        # 每帧都记 —— 判据不触发时这是唯一能看出「到底量出了什么」的东西
        self.get_logger().debug(
            f"轨迹 {det.track_id} 倾角 {tilt:.1f}° 离地 {height:.2f} m"
        )

        # ⚠️ 这个 try 不是防御性编程，是**必须有的**。
        #
        # `FallTracker.update()` 在时间戳回退时会主动抛 ValueError（设计如此，
        # 因为状态机靠时间差判断，回退的时间戳会给出没有意义的结果）。
        # 但异常若从这里逃出去，会穿过 `_on_images`（检测循环那段没有 try），
        # 而 rclpy 的 executor 会在 **spin 线程**上重新抛出回调异常 ——
        # 倒地和普通检测**一起停**，进程带 traceback 退出。
        #
        # 触发条件是现实的：ApproximateTimeSynchronizer **不保证回调的时间戳单调**
        # （两路 BEST_EFFORT 图像乱序、rosbag --loop、/clock 跳变都会造成）。
        #
        # 所以在这一层拦住并计数：丢掉这一帧的**判据**输入，
        # 但保留这一帧的普通检测 —— 这符合「宁可丢这一帧，也不能静默错，
        # 更不能带走整条链路」。
        t_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        try:
            ev = self._fall_tracker.update(det.track_id, TorsoObservation(
                stamp=t_sec, tilt_deg=tilt, height_m=height,
            ))
        except ValueError as exc:
            self._stat["fall_stamp_regress"] += 1
            if self._stat["fall_stamp_regress"] % 20 == 1:
                self.get_logger().warn(
                    f"倒地判据的时间戳回退（第 {self._stat['fall_stamp_regress']} 次）：{exc} "
                    f"—— 本帧的判据输入被丢弃。频繁出现说明上游时间戳乱序，"
                    f"查 rosbag --loop / use_sim_time / 相机时间戳来源。"
                )
            return
        self._prune_fall_tracks(now=t_sec)

        if ev is not None:
            self._stat["fall_events"] += 1
            self.get_logger().warn(
                f"⚠️ 倒地：轨迹 {ev.track_id} ｜ 倾角 {ev.tilt_deg:.0f}° ｜ "
                f"离地 {ev.height_m:.2f} m ｜ 起始于 {ev.onset_stamp:.3f}"
            )
            self._publish_fall(
                ev, det, stamp,
                map_point=(s_mid + h_mid) / 2.0,
                confidence=float(np.mean(torso.keypoint_confs)),
            )

    def _publish_fall(self, ev, det, stamp, map_point, confidence: float) -> None:
        """发一条倒地事件。消息构造见模块级 build_fall_detection（可单测）。"""
        msg = build_fall_detection(
            ev, stamp, map_point, confidence,
            has_snapshot=bool(self.get_parameter("publish_snapshot").value),
        )
        self._det_pub.publish(msg)
        if msg.has_snapshot:
            self._publish_snapshot(det, msg.id, stamp)

    def _prune_fall_tracks(self, now: float) -> None:
        """清掉久未出现的轨迹。不清的话跟踪器一换 id 就留一份状态，只涨不跌。"""
        n = self._fall_tracker.prune(
            now, max_age_s=float(self.get_parameter("fall_track_max_age_s").value))
        if n:
            self.get_logger().debug(f"清理了 {n} 条陈旧轨迹")

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

    def _lookup_tf(self, camera_frame: str, stamp):
        """查 ``map <- camera`` 的变换。查不到返回 ``None``。

        用**图像时间戳**查，而不是 ``Time(0)``（最新）：Go2 在移动，
        用最新的变换去变换几十毫秒前的图会引入系统性偏差。
        查不到时降级用「最新」并计数，而不是丢弃整帧检测 —— 丢帧同样会让指标失真。

        ⚠️ 失败时**不在这里**加 ``tf_fail`` 计数，由调用方加。
        原先两处都加，同一个失败被记了两次，统计就对不上了。
        """
        map_frame = self.get_parameter("map_frame").value
        timeout = Duration(seconds=self.get_parameter("tf_timeout_s").value)

        try:
            return self.tf_buffer.lookup_transform(map_frame, camera_frame, stamp, timeout)
        except TransformException as exc:
            if not self.get_parameter("tf_fallback_to_latest").value:
                self.get_logger().warn(
                    f"在图像时间戳上查不到 {map_frame}<-{camera_frame} 的变换：{exc}",
                    throttle_duration_sec=5.0,
                )
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
                return tf
            except TransformException as exc2:
                self.get_logger().warn(
                    f"TF 完全查不到 {map_frame}<-{camera_frame}：{exc2}",
                    throttle_duration_sec=5.0,
                )
                return None

    def _to_map(self, res: ProjectionResult, tf) -> Pose:
        """把相机系三维点变换到 map 系。``tf`` 由 ``_lookup_tf`` 得到。"""
        from tf2_geometry_msgs import do_transform_point  # 局部导入，减少节点启动时间
        from geometry_msgs.msg import PointStamped

        p = PointStamped()
        # 源系是 tf 的**子**系（camera），不是 header 里的父系（map）——
        # 写反了 do_transform_point 在部分版本上会直接抛，而错误信息指向
        # 「frame_id 不匹配」，很容易被误诊成 TF 树没建好。
        p.header.frame_id = tf.child_frame_id
        p.point = Point(x=float(res.point[0]), y=float(res.point[1]), z=float(res.point[2]))

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

        # ---- 异常类专属字段：非异常类必须显式填「无」----
        #
        # ⚠️ 这三个字段**不能靠消息类型的默认值**。float32 的默认是 0.0，
        # 而契约要求非异常类填 NaN。尤其 `torso_height_m = 0.0` ——
        # **那正好是「躺在地上」的值**，等于给 G3 凭空造出一批最像倒地、
        # 数量又占绝对多数的样本。而两条消息唯一的区别只在 `source` 上，
        # 不报任何错。
        #
        # （G3 按 README 的承诺「在录好的 bag 上重扫阈值」时，
        #   这些假的 0 会直接污染 `torso_*` 的分布。）
        msg.onset_stamp.sec = 0            # 契约：非异常类填 0
        msg.onset_stamp.nanosec = 0
        msg.torso_tilt_deg = float("nan")
        msg.torso_height_m = float("nan")

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
