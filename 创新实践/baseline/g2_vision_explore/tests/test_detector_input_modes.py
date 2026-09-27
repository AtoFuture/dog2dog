"""RGB / RGB-D 输入模式的回归测试。

这些测试故意不启动完整 ROS graph：它们验证订阅集合和公共二维处理路径，
把「RGB-only 意外等待深度」与「RGB-only 伪造 Detection3D」这两类静默失效
直接钉死在节点代码上。
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from g2_core.detector import Detection2D
from g2_core.projector import ProjectionResult
from vision_detector import detector_node
from vision_detector.detector_node import VisionDetector


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.infos = []
        self.warns = []

    def info(self, message, **kwargs):
        self.infos.append(message)

    def warn(self, message, **kwargs):
        self.warns.append(message)


class _SubscriptionNode:
    def __init__(self, mode):
        self._input_mode = mode
        self._localization_backend = "rgbd" if mode == "rgbd" else "none"
        self._lidar_backend = None
        self._vision_group = object()
        self.calls = []
        self._log = _Logger()
        self.params = {
            "rgb_topic": "/camera/go2/image_raw",
            "color_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "sync_slop_s": 0.05,
            "detect_hz": 5.0,
            "pointcloud_topic": "/utlidar/cloud",
        }

    def get_parameter(self, name):
        return _Param(self.params[name])

    def create_subscription(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return object()

    def get_logger(self):
        return self._log

    def _on_rgb(self, _message):
        pass

    def _on_camera_info(self, _message):
        pass

    def _on_images(self, _color, _depth):
        pass

    def _on_pointcloud(self, _message):
        pass


def _detection(track_id=7):
    return Detection2D(
        class_name="person",
        class_id=0,
        confidence=0.84,
        bbox_xyxy=(10.0, 20.0, 40.0, 80.0),
        track_id=track_id,
    )


def test_rgb_mode_creates_only_rgb_image_subscription(monkeypatch):
    node = _SubscriptionNode("rgb")

    def fail_message_filter(*args, **kwargs):
        raise AssertionError("RGB-only mode must not create message_filters subscribers")

    monkeypatch.setattr(detector_node, "Subscriber", fail_message_filter)
    VisionDetector._setup_subscriptions(node)

    assert len(node.calls) == 1
    args, kwargs = node.calls[0]
    assert args[0] is Image
    assert args[1] == "/camera/go2/image_raw"
    assert args[2].__name__ == "_on_rgb"
    assert node._camera_info_sub is None
    assert node._sync is None
    assert node._rgb_sub is not None


def test_rgbd_mode_keeps_camera_info_and_approximate_sync(monkeypatch):
    node = _SubscriptionNode("rgbd")
    mf_calls = []

    class _FakeSubscriber:
        def __init__(self, *args, **kwargs):
            mf_calls.append(("subscriber", args, kwargs))

    class _FakeSynchronizer:
        def __init__(self, subscribers, queue_size, slop):
            mf_calls.append(("sync", subscribers, queue_size, slop))
            self.callback = None

        def registerCallback(self, callback):
            self.callback = callback

    monkeypatch.setattr(detector_node, "Subscriber", _FakeSubscriber)
    monkeypatch.setattr(detector_node, "ApproximateTimeSynchronizer", _FakeSynchronizer)
    VisionDetector._setup_subscriptions(node)

    assert len(node.calls) == 1
    assert node.calls[0][0][0] is CameraInfo
    assert len([c for c in mf_calls if c[0] == "subscriber"]) == 2
    sync = next(c for c in mf_calls if c[0] == "sync")
    assert sync[2:] == (10, 0.05)
    assert node._rgb_sub is None
    assert node._camera_info_sub is not None
    assert node._sync is not None


def test_rgb_lidar_mode_adds_only_configured_pointcloud_subscription():
    node = _SubscriptionNode("rgb")
    node._localization_backend = "lidar"
    node._lidar_backend = object()

    VisionDetector._setup_subscriptions(node)

    assert len(node.calls) == 2
    assert node.calls[0][0][0] is Image
    assert node.calls[0][0][1] == "/camera/go2/image_raw"
    assert node.calls[1][0][0] is PointCloud2
    assert node.calls[1][0][1] == "/utlidar/cloud"
    assert node._camera_info_sub is None
    assert node._sync is None


def test_rgb_and_rgbd_use_the_same_2d_detector_path():
    image = np.zeros((32, 48, 3), dtype=np.uint8)

    class _FakeDetector:
        def __init__(self):
            self.calls = []
            self.result = [_detection()]

        def track(self, frame):
            self.calls.append(frame)
            return self.result

    class _Node:
        def __init__(self, mode):
            self._input_mode = mode
            self._detector = _FakeDetector()
            self._stat = {"frames": 0, "dets": 0}
            self._last_color = None
            self.rgbd_calls = []

        def _run_2d(self, color):
            return VisionDetector._run_2d(self, color)

        def _process_rgbd_detections(self, *args):
            self.rgbd_calls.append(args)

    rgb = _Node("rgb")
    stamp = Time()
    frame_2d = VisionDetector._process_frame(
        rgb, image, stamp, "go2_camera", None, 1.0
    )
    assert len(rgb._detector.calls) == 1
    assert rgb._detector.calls[0] is image
    assert rgb._stat == {"frames": 1, "dets": 1}
    assert rgb.rgbd_calls == []
    assert frame_2d.stamp is stamp
    assert frame_2d.frame_id == "go2_camera"
    assert frame_2d.image_shape == (32, 48)
    assert frame_2d.detections[0].track_id == 7
    assert rgb._last_2d is frame_2d

    rgbd = _Node("rgbd")
    depth = np.ones((32, 48), dtype=np.float32)
    frame_2d = VisionDetector._process_frame(
        rgbd, image, Time(), "camera", depth, 1.0
    )
    assert len(rgbd._detector.calls) == 1
    assert len(rgbd.rgbd_calls) == 1
    assert rgbd.rgbd_calls[0][0] == [_detection()]
    assert rgbd.rgbd_calls[0][1] is depth
    assert frame_2d.image_shape == (32, 48)


def test_rgb_only_does_not_publish_detection3d_or_snapshot():
    image = np.zeros((16, 16, 3), dtype=np.uint8)

    class _Node:
        _input_mode = "rgb"
        _stat = {"frames": 0, "dets": 0}
        _last_color = None

        def __init__(self):
            self._detector = SimpleNamespace(track=lambda frame: [_detection()])
            self.detection3d_messages = []
            self.snapshot_messages = []

        def _run_2d(self, color):
            return VisionDetector._run_2d(self, color)

        def _process_rgbd_detections(self, *args):
            raise AssertionError("RGB-only mode must not enter the RGB-D backend")

    node = _Node()
    VisionDetector._process_frame(node, image, Time(), "go2_camera", None, 1.0)

    assert node._stat == {"frames": 1, "dets": 1}
    assert node.detection3d_messages == []
    assert node.snapshot_messages == []


def test_rgbd_backend_still_publishes_valid_projection(monkeypatch):
    depth = np.ones((20, 20), dtype=np.float32)
    det = _detection()
    published = []

    class _Node:
        _input_mode = "rgbd"
        _fall_enabled = False
        _intrinsics = object()

        def __init__(self):
            self._stat = {
                "proj_fail": 0,
                "dedup": 0,
                "tf_fail": 0,
                "published": 0,
            }

        def get_parameter(self, name):
            return _Param({
                "min_valid_pixels": 1,
                "max_dispersion_m": 3.0,
                "anchor": "center",
            }[name])

        def _lookup_tf(self, camera_frame, stamp):
            return "tf"

        def _should_publish(self, track_id, now):
            return True

        def _to_map(self, result, tf):
            assert tf == "tf"
            return "pose"

        def _publish_detection(self, detection, pose, stamp):
            published.append((detection, pose, stamp))

    monkeypatch.setattr(
        detector_node,
        "project_depth_bbox",
        lambda *args, **kwargs: ProjectionResult(
            point=np.array([1.0, 2.0, 3.0]),
            valid_pixels=20,
            dispersion=0.1,
            spread_per_axis=np.array([0.1, 0.1, 0.1]),
        ),
    )

    node = _Node()
    VisionDetector._process_rgbd_detections(
        node, [det], depth, "camera", Time(), 1.0
    )

    assert len(published) == 1
    assert published[0][0] is det
    assert published[0][1] == "pose"
    assert node._stat["published"] == 1


def test_detector_config_defaults_to_rgbd_and_exposes_go2_rgb_topic():
    config = (
        __import__("pathlib").Path(__file__).parent.parent
        / "vision_detector" / "config" / "detector.yaml"
    ).read_text(encoding="utf-8")
    assert "input_mode: rgbd" in config
    assert "rgb_topic: /camera/go2/image_raw" in config
    assert "localization_backend: auto" in config
    assert "pointcloud_topic: /utlidar/cloud" in config
