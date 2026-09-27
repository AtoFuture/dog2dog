"""Go2 真机联调前置诊断与 RGB-LiDAR 采集工具。

本模块只观察和记录真实 ROS 图，不修改 Bridge、TF 或正式消息语义：

* ``/frontvideostream``：消息类型、发布端 QoS、payload 统计、H.264 NAL 特征；
* Bridge Image：尺寸、编码、frame、header timestamp 和输出频率；
* ``PointCloud2``：布局、字段、XYZ 有效点、NaN/Inf 和范围；
* header/time_frame/node clock 的连续样本，以及不做单位猜测的时钟诊断；
* TF 可用性和可选的 image/cloud 配对采集。

所有 readiness 判断都保守：观察到的数据会标记为 ``OBSERVED``，启发式解释只
标记为 ``INFERRED``，没有证据的单位/同步结论标记为 ``UNCONFIRMED``。
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any, Iterable, Mapping

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


VIDEO_TYPE = "unitree_go/msg/Go2FrontVideoData"
IMAGE_TYPE = "sensor_msgs/msg/Image"
POINTCLOUD_TYPE = "sensor_msgs/msg/PointCloud2"
_POINT_FIELD_TYPES = {
    PointField.INT8: "INT8",
    PointField.UINT8: "UINT8",
    PointField.INT16: "INT16",
    PointField.UINT16: "UINT16",
    PointField.INT32: "INT32",
    PointField.UINT32: "UINT32",
    PointField.FLOAT32: "FLOAT32",
    PointField.FLOAT64: "FLOAT64",
}


def stamp_to_seconds(stamp: Any) -> float:
    """Convert a ROS time-like object without assigning clock semantics."""
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value)


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


@dataclass
class ScalarSeries:
    """Bounded numeric samples and deltas for diagnostic output."""

    max_samples: int = 128
    values: deque[float] = field(default_factory=lambda: deque(maxlen=128))
    deltas: deque[float] = field(default_factory=lambda: deque(maxlen=127))

    def add(self, value: float) -> None:
        value = float(value)
        if self.values:
            self.deltas.append(value - self.values[-1])
        self.values.append(value)

    def summary(self) -> dict[str, Any]:
        values = list(self.values)
        deltas = list(self.deltas)
        return {
            "count": len(values),
            "last": values[-1] if values else None,
            "min": min(values) if values else None,
            "max": max(values) if values else None,
            "mean": statistics.fmean(values) if values else None,
            "deltas": deltas,
            "delta_min": min(deltas) if deltas else None,
            "delta_max": max(deltas) if deltas else None,
            "delta_mean": statistics.fmean(deltas) if deltas else None,
        }


@dataclass
class RateCounter:
    count: int = 0
    first_wall: float | None = None
    last_wall: float | None = None

    def add(self, wall_time: float) -> None:
        self.count += 1
        if self.first_wall is None:
            self.first_wall = wall_time
        self.last_wall = wall_time

    @property
    def rate_hz(self) -> float | None:
        if self.count < 2 or self.first_wall is None or self.last_wall is None:
            return None
        elapsed = self.last_wall - self.first_wall
        return (self.count - 1) / elapsed if elapsed > 0.0 else None


def inspect_h264_payload(payload: bytes | bytearray | memoryview) -> dict[str, Any]:
    """Inspect Annex-B markers without treating the payload as a frame boundary."""
    data = bytes(payload)
    starts: list[int] = []
    i = 0
    while i + 3 < len(data):
        if data[i:i + 4] == b"\x00\x00\x00\x01":
            starts.append(i)
            i += 4
        elif data[i:i + 3] == b"\x00\x00\x01":
            starts.append(i)
            i += 3
        else:
            i += 1

    nal_types: set[int] = set()
    for offset in starts:
        code_len = 4 if data[offset:offset + 4] == b"\x00\x00\x00\x01" else 3
        nal_offset = offset + code_len
        if nal_offset < len(data):
            nal_types.add(data[nal_offset] & 0x1F)

    return {
        "bytes": len(data),
        "annex_b_start_code_count": len(starts),
        "has_annex_b_start_code": bool(starts),
        "nal_types": sorted(nal_types),
        "has_sps": 7 in nal_types,
        "has_pps": 8 in nal_types,
        "has_idr": 5 in nal_types,
    }


@dataclass
class H264PayloadStats:
    message_rate: RateCounter = field(default_factory=RateCounter)
    payload_lengths: ScalarSeries = field(default_factory=ScalarSeries)
    time_frames: ScalarSeries = field(default_factory=ScalarSeries)
    annex_b_messages: int = 0
    sps_messages: int = 0
    pps_messages: int = 0
    idr_messages: int = 0
    nal_types: set[int] = field(default_factory=set)

    def add(self, payload: bytes, time_frame: int, wall_time: float) -> None:
        self.message_rate.add(wall_time)
        self.payload_lengths.add(len(payload))
        self.time_frames.add(float(time_frame))
        features = inspect_h264_payload(payload)
        if features["has_annex_b_start_code"]:
            self.annex_b_messages += 1
        if features["has_sps"]:
            self.sps_messages += 1
        if features["has_pps"]:
            self.pps_messages += 1
        if features["has_idr"]:
            self.idr_messages += 1
        self.nal_types.update(features["nal_types"])

    def summary(self) -> dict[str, Any]:
        return {
            "messages": self.message_rate.count,
            "message_rate_hz": self.message_rate.rate_hz,
            "payload_bytes": self.payload_lengths.summary(),
            "time_frame_raw": self.time_frames.summary(),
            "annex_b_messages": self.annex_b_messages,
            "sps_messages": self.sps_messages,
            "pps_messages": self.pps_messages,
            "idr_messages": self.idr_messages,
            "observed_nal_types": sorted(self.nal_types),
        }


def _xyz_from_pointcloud(msg: PointCloud2) -> np.ndarray:
    fields = {field.name for field in msg.fields}
    missing = {"x", "y", "z"} - fields
    if missing:
        raise ValueError(f"PointCloud2 missing XYZ fields: {sorted(missing)}")
    raw = point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=False
    )
    if isinstance(raw, np.ndarray):
        if raw.dtype.names:
            values = np.column_stack((raw["x"], raw["y"], raw["z"]))
        else:
            values = raw
    else:
        values = np.asarray(list(raw), dtype=np.float64)
    if values.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(values, dtype=np.float64).reshape(-1, 3)


def point_field_summary(field: PointField) -> dict[str, Any]:
    return {
        "name": str(field.name),
        "offset": int(field.offset),
        "datatype": int(field.datatype),
        "datatype_name": _POINT_FIELD_TYPES.get(int(field.datatype), "UNKNOWN"),
        "count": int(field.count),
    }


def summarize_pointcloud_message(msg: PointCloud2) -> dict[str, Any]:
    """Summarize one PointCloud2, including actual layout and XYZ quality."""
    xyz_error = None
    try:
        xyz = _xyz_from_pointcloud(msg)
    except (TypeError, ValueError, RuntimeError) as exc:
        xyz = np.empty((0, 3), dtype=np.float64)
        xyz_error = str(exc)

    finite = np.isfinite(xyz).all(axis=1) if xyz.size else np.empty(0, dtype=bool)
    nan_rows = np.isnan(xyz).any(axis=1) if xyz.size else np.empty(0, dtype=bool)
    inf_rows = np.isinf(xyz).any(axis=1) if xyz.size else np.empty(0, dtype=bool)
    valid_xyz = xyz[finite] if xyz.size else np.empty((0, 3), dtype=np.float64)
    ranges = None
    if valid_xyz.size:
        ranges = {
            "x": [float(np.min(valid_xyz[:, 0])), float(np.max(valid_xyz[:, 0]))],
            "y": [float(np.min(valid_xyz[:, 1])), float(np.max(valid_xyz[:, 1]))],
            "z": [float(np.min(valid_xyz[:, 2])), float(np.max(valid_xyz[:, 2]))],
        }
    total = int(xyz.shape[0])
    return {
        "frame_id": str(msg.header.frame_id),
        "stamp_sec": stamp_to_seconds(msg.header.stamp),
        "width": int(msg.width),
        "height": int(msg.height),
        "point_step": int(msg.point_step),
        "row_step": int(msg.row_step),
        "is_dense": bool(msg.is_dense),
        "fields": [point_field_summary(field) for field in msg.fields],
        "xyz_points": total,
        "valid_xyz_points": int(np.count_nonzero(finite)),
        "invalid_xyz_points": int(total - np.count_nonzero(finite)),
        "nan_rows": int(np.count_nonzero(nan_rows)),
        "inf_rows": int(np.count_nonzero(inf_rows)),
        "invalid_ratio": (float(total - np.count_nonzero(finite)) / total) if total else 0.0,
        "ranges": ranges,
        "xyz_error": xyz_error,
    }


@dataclass
class PointCloudStats:
    message_rate: RateCounter = field(default_factory=RateCounter)
    header_stamps: ScalarSeries = field(default_factory=ScalarSeries)
    latest: dict[str, Any] | None = None

    def add(self, msg: PointCloud2, wall_time: float) -> dict[str, Any]:
        self.message_rate.add(wall_time)
        self.latest = summarize_pointcloud_message(msg)
        self.header_stamps.add(float(self.latest["stamp_sec"]))
        return self.latest

    def summary(self) -> dict[str, Any]:
        return {
            "messages": self.message_rate.count,
            "message_rate_hz": self.message_rate.rate_hz,
            "header_stamp": self.header_stamps.summary(),
            "latest": self.latest,
        }


@dataclass
class ImageStats:
    message_rate: RateCounter = field(default_factory=RateCounter)
    header_stamps: ScalarSeries = field(default_factory=ScalarSeries)
    node_clock_stamps: ScalarSeries = field(default_factory=ScalarSeries)
    latest: dict[str, Any] | None = None

    def add(self, msg: Image, wall_time: float, node_clock_sec: float) -> None:
        self.message_rate.add(wall_time)
        stamp = stamp_to_seconds(msg.header.stamp)
        self.header_stamps.add(stamp)
        self.node_clock_stamps.add(node_clock_sec)
        self.latest = {
            "stamp_sec": stamp,
            "frame_id": str(msg.header.frame_id),
            "width": int(msg.width),
            "height": int(msg.height),
            "encoding": str(msg.encoding),
            "step": int(msg.step),
            "is_bigendian": int(msg.is_bigendian),
        }

    def summary(self) -> dict[str, Any]:
        return {
            "messages": self.message_rate.count,
            "message_rate_hz": self.message_rate.rate_hz,
            "header_stamp": self.header_stamps.summary(),
            "node_clock_now": self.node_clock_stamps.summary(),
            "latest": self.latest,
        }


def _qos_summary(profile: Any) -> dict[str, Any]:
    return {
        "reliability": _enum_name(getattr(profile, "reliability", "unknown")),
        "durability": _enum_name(getattr(profile, "durability", "unknown")),
        "history": _enum_name(getattr(profile, "history", "unknown")),
        "depth": int(getattr(profile, "depth", 0)),
    }


def _endpoint_summary(info: Any) -> dict[str, Any]:
    return {
        "node_name": str(getattr(info, "node_name", "")),
        "node_namespace": str(getattr(info, "node_namespace", "")),
        "topic_type": str(getattr(info, "topic_type", "")),
        "endpoint_type": _enum_name(getattr(info, "endpoint_type", "unknown")),
        "qos": _qos_summary(getattr(info, "qos_profile", object())),
    }


def topic_status(node: Any, topic: str, expected_type: str) -> dict[str, Any]:
    """Graph inspection that returns a blocker instead of throwing."""
    result: dict[str, Any] = {
        "topic": topic,
        "expected_type": expected_type,
        "exists": False,
        "types": [],
        "type_matches": False,
        "publisher_count": 0,
        "publishers": [],
        "error": None,
    }
    try:
        names_and_types = dict(node.get_topic_names_and_types())
        result["types"] = list(names_and_types.get(topic, []))
        result["exists"] = topic in names_and_types
        result["type_matches"] = expected_type in result["types"]
        publishers = node.get_publishers_info_by_topic(topic)
        result["publishers"] = [_endpoint_summary(info) for info in publishers]
        result["publisher_count"] = len(publishers)
    except Exception as exc:  # graph discovery may be unavailable during startup
        result["error"] = str(exc)
    return result


def tf_unavailable(source: str, target: str, reason: str) -> dict[str, Any]:
    return {
        "source": source,
        "target": target,
        "available": False,
        "latest_timestamp_sec": None,
        "translation": None,
        "rotation_xyzw": None,
        "reason": reason,
    }


def tf_available(source: str, target: str, transform: Any) -> dict[str, Any]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        "source": source,
        "target": target,
        "available": True,
        "latest_timestamp_sec": stamp_to_seconds(transform.header.stamp),
        "translation": [float(translation.x), float(translation.y), float(translation.z)],
        "rotation_xyzw": [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)],
        "reason": None,
    }


def validate_calibration_data(data: Mapping[str, Any] | None) -> dict[str, Any]:
    """Validate optional JSON calibration without inventing missing extrinsics."""
    if data is None:
        return {
            "provided": False,
            "intrinsics_valid": False,
            "extrinsic_valid": False,
            "errors": ["no calibration JSON supplied"],
        }

    params: Mapping[str, Any] = data
    if isinstance(data.get("ros__parameters"), Mapping):
        params = data["ros__parameters"]
    elif isinstance(data.get("/**"), Mapping) and isinstance(data["/**"].get("ros__parameters"), Mapping):
        params = data["/**"]["ros__parameters"]

    def first(*names: str) -> Any:
        for name in names:
            if name in params:
                return params[name]
        return None

    fx, fy = first("lidar_fx", "fx"), first("lidar_fy", "fy")
    cx, cy = first("lidar_cx", "cx"), first("lidar_cy", "cy")
    width, height = first("lidar_image_width", "image_width"), first("lidar_image_height", "image_height")
    errors: list[str] = []
    if not all(_finite(v) and float(v) > 0.0 for v in (fx, fy)):
        errors.append("fx/fy missing or non-positive")
    if not all(_finite(v) for v in (cx, cy)):
        errors.append("cx/cy missing or non-finite")
    if not all(_finite(v) and int(v) > 0 for v in (width, height)):
        errors.append("image size missing or non-positive")
    intrinsics_valid = not errors

    translation = first("lidar_translation_xyz", "translation_xyz")
    quaternion = first("lidar_quaternion_xyzw", "quaternion_xyzw")
    lidar_frame = str(first("lidar_frame") or "").strip()
    camera_frame = str(first("lidar_camera_frame", "camera_frame") or "").strip()
    extrinsic_valid = False
    if not isinstance(translation, (list, tuple)) or len(translation) != 3:
        errors.append("LiDAR-camera translation missing")
    elif not all(_finite(v) for v in translation):
        errors.append("LiDAR-camera translation non-finite")
    if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4:
        errors.append("LiDAR-camera quaternion missing")
    elif not all(_finite(v) for v in quaternion):
        errors.append("LiDAR-camera quaternion non-finite")
    elif not math.isclose(float(np.linalg.norm(np.asarray(quaternion, dtype=float))), 1.0, rel_tol=1e-3, abs_tol=1e-3):
        errors.append("LiDAR-camera quaternion is not unit length")
    if not lidar_frame or not camera_frame:
        errors.append("LiDAR/camera frame missing")
    extrinsic_errors = [error for error in errors if "LiDAR-camera" in error or "frame" in error]
    extrinsic_valid = not extrinsic_errors
    return {
        "provided": True,
        "source": data.get("source") if isinstance(data, Mapping) else None,
        "intrinsics_valid": intrinsics_valid,
        "extrinsic_valid": extrinsic_valid,
        "errors": errors,
        "intrinsics": {
            "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "image_width": width, "image_height": height,
        },
        "frames": {"lidar": lidar_frame, "camera": camera_frame},
    }


def _status(ready: bool, status: str, reasons: Iterable[str]) -> dict[str, Any]:
    return {"ready": bool(ready), "status": status, "reasons": list(reasons)}


def build_readiness(
    *,
    video_topic: Mapping[str, Any],
    image_topic: Mapping[str, Any],
    lidar_topic: Mapping[str, Any],
    image_messages: int,
    cloud_messages: int,
    valid_xyz_points: int,
    calibration: Mapping[str, Any],
    tf_entries: Iterable[Mapping[str, Any]],
    required_tf_pairs: Iterable[tuple[str, str]] = (),
    clock_confirmed: bool = False,
) -> dict[str, Any]:
    image_ready = bool(image_topic.get("type_matches") and image_messages > 0)
    lidar_ready = bool(lidar_topic.get("type_matches") and cloud_messages > 0 and valid_xyz_points > 0)
    tf_entries = list(tf_entries)
    required_tf_pairs = list(required_tf_pairs)
    available_tf_pairs = {
        (str(entry.get("source", "")), str(entry.get("target", "")))
        for entry in tf_entries
        if entry.get("available")
    }
    missing_tf_pairs = [
        f"{source} -> {target}"
        for source, target in required_tf_pairs
        if (source, target) not in available_tf_pairs
    ]
    tf_ready = bool(required_tf_pairs) and not missing_tf_pairs
    tf_reasons = (
        []
        if tf_ready
        else [
            "required TF frame names were not observed"
            if not required_tf_pairs
            else "missing TF: " + ", ".join(missing_tf_pairs)
        ]
    )
    video_reasons = []
    if not video_topic.get("exists"):
        video_reasons.append("front video topic not present")
    if not image_topic.get("type_matches"):
        video_reasons.append("bridge Image topic type mismatch or absent")
    if image_messages == 0:
        video_reasons.append("no bridge Image received")
    lidar_reasons = []
    if not lidar_topic.get("exists"):
        lidar_reasons.append("PointCloud2 topic not present")
    if not lidar_topic.get("type_matches"):
        lidar_reasons.append("PointCloud2 type mismatch or absent")
    if cloud_messages == 0:
        lidar_reasons.append("no PointCloud2 received")
    if valid_xyz_points == 0:
        lidar_reasons.append("no valid XYZ point observed")

    intrinsics_ready = bool(calibration.get("intrinsics_valid"))
    extrinsic_ready = bool(calibration.get("extrinsic_valid"))
    clock_reasons = ["time_frame clock and PointCloud2/header clock domain remain unconfirmed"]
    fusion_ready = (
        image_ready
        and lidar_ready
        and intrinsics_ready
        and extrinsic_ready
        and tf_ready
        and clock_confirmed
    )
    return {
        "READY_FOR_RGB_ONLY": _status(
            image_ready,
            "ready" if image_ready else "blocked",
            video_reasons,
        ),
        "READY_FOR_LIDAR_LOCALIZATION": _status(
            lidar_ready and intrinsics_ready and extrinsic_ready and tf_ready,
            "ready" if lidar_ready and intrinsics_ready and extrinsic_ready and tf_ready else "blocked",
            lidar_reasons
            + ([] if intrinsics_ready else ["camera intrinsics not validated/supplied"])
            + ([] if extrinsic_ready else ["camera-LiDAR extrinsic not validated/supplied"])
            + tf_reasons,
        ),
        "READY_FOR_RGB_LIDAR_FUSION": _status(
            fusion_ready,
            "ready" if fusion_ready else "blocked",
            video_reasons + lidar_reasons
            + ([] if intrinsics_ready else ["camera intrinsics not validated/supplied"])
            + ([] if extrinsic_ready else ["camera-LiDAR extrinsic not validated/supplied"])
            + tf_reasons
            + ([] if clock_confirmed else clock_reasons),
        ),
        "CLOCK_SYNC": _status(clock_confirmed, "confirmed" if clock_confirmed else "unconfirmed", [] if clock_confirmed else clock_reasons),
    }


def infer_time_frame_interpretation(time_frame_summary: Mapping[str, Any]) -> dict[str, Any]:
    """Give hypotheses only; this function never changes timestamp behavior."""
    deltas = [float(value) for value in time_frame_summary.get("deltas", [])]
    if not deltas:
        return {"status": "UNCONFIRMED", "hypotheses": [], "reason": "not enough samples"}
    median_delta = statistics.median(deltas)
    hypotheses: list[str] = []
    if math.isclose(median_delta, 1.0, rel_tol=0.05, abs_tol=0.05):
        hypotheses.append("could be a frame counter")
    if 100.0 <= abs(median_delta) <= 10_000_000.0:
        hypotheses.append("could be a device tick/time unit; unit is unknown")
    return {
        "status": "INFERRED",
        "median_delta": median_delta,
        "hypotheses": hypotheses or ["no stable common unit hypothesis"],
        "used_for_ros_timestamp": False,
    }


@dataclass
class PendingImage:
    sequence: int
    message: Image
    stamp_sec: float
    receive_wall: float


@dataclass
class PendingCloud:
    sequence: int
    message: PointCloud2
    stamp_sec: float
    receive_wall: float


@dataclass
class PreflightOptions:
    video_topic: str = "/frontvideostream"
    image_topic: str = "/camera/go2/image_raw"
    lidar_topic: str = "/utlidar/cloud"
    video_stream: str = "video720p"
    duration_s: float = 10.0
    capture_count: int = 0
    output_dir: Path = Path("hardware_preflight_report")
    calibration_json: Path | None = None
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    camera_frame: str = ""
    lidar_frame: str = ""


class HardwarePreflightNode(Node):
    """ROS sampling node; it never publishes or modifies the robot graph."""

    def __init__(self, options: PreflightOptions):
        super().__init__("hardware_preflight")
        self.options = options
        self.startup_errors: list[str] = []
        self.video_stats = H264PayloadStats()
        self.image_stats = ImageStats()
        self.cloud_stats = PointCloudStats()
        self.pair_deltas = ScalarSeries()
        self._image_sequence = 0
        self._cloud_sequence = 0
        self._latest_image: PendingImage | None = None
        self._latest_cloud: PendingCloud | None = None
        self._last_pair: tuple[int, int] | None = None
        self.capture_records: list[dict[str, Any]] = []
        self.options.output_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_subscription = self.create_subscription(
            Image, options.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.cloud_subscription = self.create_subscription(
            PointCloud2, options.lidar_topic, self._on_cloud, qos_profile_sensor_data
        )

        self.video_subscription = None
        try:
            from unitree_go.msg import Go2FrontVideoData
            self.video_subscription = self.create_subscription(
                Go2FrontVideoData,
                options.video_topic,
                self._on_video,
                qos_profile_sensor_data,
            )
        except (ImportError, ModuleNotFoundError) as exc:
            self.startup_errors.append(f"unitree_go Go2FrontVideoData unavailable: {exc}")

    def _node_clock_seconds(self) -> float:
        try:
            return stamp_to_seconds(self.get_clock().now().to_msg())
        except Exception:
            return 0.0

    def _on_video(self, message: Any) -> None:
        payload = getattr(message, self.options.video_stream, None)
        if payload is None:
            self.startup_errors.append(
                f"Go2FrontVideoData has no field {self.options.video_stream}"
            )
            return
        try:
            raw_time_frame = int(getattr(message, "time_frame"))
        except (AttributeError, TypeError, ValueError):
            raw_time_frame = 0
        self.video_stats.add(bytes(payload), raw_time_frame, time.monotonic())

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        self.image_stats.add(message, now, self._node_clock_seconds())
        self._image_sequence += 1
        self._latest_image = PendingImage(
            self._image_sequence, message, stamp_to_seconds(message.header.stamp), now
        )
        self._try_capture_pair()

    def _on_cloud(self, message: PointCloud2) -> None:
        now = time.monotonic()
        self.cloud_stats.add(message, now)
        self._cloud_sequence += 1
        self._latest_cloud = PendingCloud(
            self._cloud_sequence, message, stamp_to_seconds(message.header.stamp), now
        )
        self._try_capture_pair()

    def _try_capture_pair(self) -> None:
        if self.options.capture_count <= 0 or len(self.capture_records) >= self.options.capture_count:
            return
        if self._latest_image is None or self._latest_cloud is None:
            return
        pair = (self._latest_image.sequence, self._latest_cloud.sequence)
        if pair == self._last_pair:
            return
        self._last_pair = pair
        image = self._latest_image
        cloud = self._latest_cloud
        if image.stamp_sec > 0.0 and cloud.stamp_sec > 0.0:
            delta = abs(image.stamp_sec - cloud.stamp_sec)
            delta_basis = "header_stamp"
            self.pair_deltas.add(delta)
        else:
            delta = abs(image.receive_wall - cloud.receive_wall)
            delta_basis = "host_receive_monotonic"
        self.capture_records.append(self._save_capture(image, cloud, delta, delta_basis))

    def _tf_snapshot(self, camera_frame: str = "", lidar_frame: str = "") -> list[dict[str, Any]]:
        pairs = [
            (self.options.map_frame, self.options.base_frame),
            (self.options.odom_frame, self.options.base_frame),
        ]
        image_frame = self.image_stats.latest.get("frame_id") if self.image_stats.latest else ""
        cloud_frame = self.cloud_stats.latest.get("frame_id") if self.cloud_stats.latest else ""
        camera = self.options.camera_frame or camera_frame or image_frame
        lidar = self.options.lidar_frame or lidar_frame or cloud_frame
        if camera:
            pairs.append((self.options.base_frame, camera))
        if lidar:
            pairs.append((self.options.base_frame, lidar))
        output = []
        seen = set()
        for source, target in pairs:
            if not source or not target or source == target or (source, target) in seen:
                continue
            seen.add((source, target))
            try:
                transform = self.tf_buffer.lookup_transform(
                    target, source, rclpy.time.Time(), Duration(seconds=0.0)
                )
                output.append(tf_available(source, target, transform))
            except TransformException as exc:
                output.append(tf_unavailable(source, target, str(exc)))
        return output

    def _save_capture(
        self,
        image: PendingImage,
        cloud: PendingCloud,
        delta: float,
        delta_basis: str,
    ) -> dict[str, Any]:
        index = len(self.capture_records) + 1
        capture_dir = self.options.output_dir / "captures"
        capture_dir.mkdir(parents=True, exist_ok=True)
        image_path = capture_dir / f"image_{index:04d}.png"
        cloud_path = capture_dir / f"cloud_{index:04d}.cdr"
        metadata_path = capture_dir / f"metadata_{index:04d}.json"
        image_error = None
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            bgr = bridge.imgmsg_to_cv2(image.message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".png", bgr)
            if not ok:
                raise RuntimeError("cv2.imencode(.png) returned false")
            image_path.write_bytes(encoded.tobytes())
        except Exception as exc:  # keep metadata even if one encoding is unsupported
            image_error = str(exc)

        cloud_error = None
        try:
            cloud_path.write_bytes(serialize_message(cloud.message))
        except Exception as exc:
            cloud_error = str(exc)

        metadata = {
            "capture_index": index,
            "pairing": {
                "method": "nearest latest observed pair",
                "timestamp_delta_s": delta,
                "delta_basis": delta_basis,
                "clock_sync_status": "UNCONFIRMED",
            },
            "image": {
                "topic": self.options.image_topic,
                "stamp_sec": image.stamp_sec,
                "frame_id": str(image.message.header.frame_id),
                "width": int(image.message.width),
                "height": int(image.message.height),
                "encoding": str(image.message.encoding),
                "path": str(image_path.relative_to(self.options.output_dir)) if image_error is None else None,
                "error": image_error,
            },
            "pointcloud": {
                "topic": self.options.lidar_topic,
                "path": str(cloud_path.relative_to(self.options.output_dir)) if cloud_error is None else None,
                "error": cloud_error,
                "summary": summarize_pointcloud_message(cloud.message),
            },
            "tf_snapshot": self._tf_snapshot(),
        }
        metadata_path.write_text(json.dumps(_jsonable(metadata), ensure_ascii=False, indent=2), encoding="utf-8")
        return {
            "capture_index": index,
            "metadata": str(metadata_path.relative_to(self.options.output_dir)),
            "image": str(image_path.relative_to(self.options.output_dir)) if image_error is None else None,
            "pointcloud": str(cloud_path.relative_to(self.options.output_dir)) if cloud_error is None else None,
            "timestamp_delta_s": delta,
            "delta_basis": delta_basis,
            "image_error": image_error,
            "pointcloud_error": cloud_error,
        }

    def report(self) -> dict[str, Any]:
        video_topic = topic_status(self, self.options.video_topic, VIDEO_TYPE)
        image_topic = topic_status(self, self.options.image_topic, IMAGE_TYPE)
        lidar_topic = topic_status(self, self.options.lidar_topic, POINTCLOUD_TYPE)
        calibration = {"provided": False, "intrinsics_valid": False, "extrinsic_valid": False, "errors": []}
        if self.options.calibration_json is not None:
            try:
                data = json.loads(self.options.calibration_json.read_text(encoding="utf-8"))
                data = dict(data)
                data.setdefault("source", str(self.options.calibration_json))
                calibration = validate_calibration_data(data)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                calibration = {
                    "provided": True,
                    "intrinsics_valid": False,
                    "extrinsic_valid": False,
                    "errors": [f"cannot read calibration JSON: {exc}"],
                }
        calibration_frames = calibration.get("frames", {})
        camera_frame = str(calibration_frames.get("camera", "")) if isinstance(calibration_frames, Mapping) else ""
        lidar_frame = str(calibration_frames.get("lidar", "")) if isinstance(calibration_frames, Mapping) else ""
        camera_frame = self.options.camera_frame or camera_frame
        lidar_frame = self.options.lidar_frame or lidar_frame
        if not camera_frame and self.image_stats.latest:
            camera_frame = str(self.image_stats.latest.get("frame_id", ""))
        if not lidar_frame and self.cloud_stats.latest:
            lidar_frame = str(self.cloud_stats.latest.get("frame_id", ""))
        tf_entries = self._tf_snapshot(camera_frame=camera_frame, lidar_frame=lidar_frame)
        required_tf_pairs = [(self.options.map_frame, self.options.base_frame)]
        if camera_frame:
            required_tf_pairs.append((self.options.base_frame, camera_frame))
        if lidar_frame:
            required_tf_pairs.append((self.options.base_frame, lidar_frame))
        cloud_latest = self.cloud_stats.latest or {}
        readiness = build_readiness(
            video_topic=video_topic,
            image_topic=image_topic,
            lidar_topic=lidar_topic,
            image_messages=self.image_stats.message_rate.count,
            cloud_messages=self.cloud_stats.message_rate.count,
            valid_xyz_points=int(cloud_latest.get("valid_xyz_points", 0) or 0),
            calibration=calibration,
            tf_entries=tf_entries,
            required_tf_pairs=required_tf_pairs,
        )
        timing = {
            "video_time_frame": self.video_stats.time_frames.summary(),
            "video_time_frame_interpretation": infer_time_frame_interpretation(
                self.video_stats.time_frames.summary()
            ),
            "bridge_image_header_stamp": self.image_stats.header_stamps.summary(),
            "pointcloud_header_stamp": self.cloud_stats.header_stamps.summary(),
            "image_cloud_pair_delta": self.pair_deltas.summary(),
            "latest_image_cloud_header_delta_s": (
                abs(self.image_stats.latest["stamp_sec"] - self.cloud_stats.latest["stamp_sec"])
                if self.image_stats.latest and self.cloud_stats.latest
                and self.image_stats.latest["stamp_sec"] > 0.0
                and self.cloud_stats.latest["stamp_sec"] > 0.0
                else None
            ),
            "node_clock_now": self.image_stats.node_clock_stamps.summary(),
            "clock_semantics": {
                "status": "UNCONFIRMED",
                "observed": [
                    "Go2FrontVideoData.time_frame raw values",
                    "bridge Image.header.stamp",
                    "PointCloud2.header.stamp",
                    "node clock now()",
                ],
                "inference": "No unit conversion or timestamp substitution was performed.",
            },
        }
        blockers = []
        blockers.extend(self.startup_errors)
        for name, entry in readiness.items():
            if not entry["ready"]:
                blockers.append(f"{name}: " + "; ".join(entry["reasons"]))
        return _jsonable({
            "schema_version": 1,
            "tool": "g2_hardware_preflight",
            "generated_at_unix": time.time(),
            "VIDEO": {
                "topic": video_topic,
                "stream": self.options.video_stream,
                "payload": self.video_stats.summary(),
            },
            "BRIDGE": {
                "topic": image_topic,
                "image": self.image_stats.summary(),
                "decode_errors": None,
                "decode_errors_note": "Bridge V0 exposes decode stats in logs, not a ROS diagnostic topic.",
            },
            "LIDAR": {
                "topic": lidar_topic,
                "pointcloud": self.cloud_stats.summary(),
            },
            "TIMING": timing,
            "TF": {
                "definition": "source -> target means lookup_transform(target, source)",
                "required_pairs": required_tf_pairs,
                "entries": tf_entries,
            },
            "CALIBRATION": calibration,
            "CAPTURE": {
                "requested_count": self.options.capture_count,
                "completed_count": len(self.capture_records),
                "output_dir": str(self.options.output_dir),
                "records": self.capture_records,
            },
            "READINESS": readiness,
            "BLOCKERS": blockers,
        })


def render_markdown(report: Mapping[str, Any]) -> str:
    """Render a compact human report from the same JSON structure."""
    lines = [
        "# G2 Go2 hardware preflight report",
        "",
        f"Generated: `{report.get('generated_at_unix')}`",
        "",
        "## READINESS",
        "",
        "| condition | ready | status | reasons |",
        "|---|---:|---|---|",
    ]
    for name, entry in report.get("READINESS", {}).items():
        reasons = "; ".join(entry.get("reasons", [])) or "-"
        lines.append(f"| `{name}` | `{entry.get('ready')}` | `{entry.get('status')}` | {reasons} |")

    video = report.get("VIDEO", {})
    payload = video.get("payload", {})
    lines.extend([
        "",
        "## VIDEO",
        "",
        f"- topic: `{video.get('topic', {}).get('topic')}`; type match: `{video.get('topic', {}).get('type_matches')}`; publishers: `{video.get('topic', {}).get('publisher_count')}`",
        f"- received messages: `{payload.get('messages')}`; rate: `{payload.get('message_rate_hz')}` Hz",
        f"- payload bytes: `{payload.get('payload_bytes')}`",
        f"- Annex-B/SPS/PPS/IDR messages: `{payload.get('annex_b_messages')}` / `{payload.get('sps_messages')}` / `{payload.get('pps_messages')}` / `{payload.get('idr_messages')}`",
        "",
        "## BRIDGE",
        "",
        f"- image: `{report.get('BRIDGE', {}).get('image')}`",
        f"- decode errors: `{report.get('BRIDGE', {}).get('decode_errors')}` (not exposed as a ROS topic)",
        "",
        "## LIDAR",
        "",
        f"- topic: `{report.get('LIDAR', {}).get('topic', {}).get('topic')}`; type match: `{report.get('LIDAR', {}).get('topic', {}).get('type_matches')}`",
        f"- pointcloud: `{report.get('LIDAR', {}).get('pointcloud')}`",
        "",
        "## TIMING",
        "",
        f"- clock semantics: `{report.get('TIMING', {}).get('clock_semantics', {}).get('status')}`",
        f"- time_frame interpretation: `{report.get('TIMING', {}).get('video_time_frame_interpretation')}`",
        f"- image/cloud pair delta: `{report.get('TIMING', {}).get('image_cloud_pair_delta')}`",
        "",
        "## TF",
        "",
    ])
    for entry in report.get("TF", {}).get("entries", []):
        lines.append(
            f"- `{entry.get('source')}` -> `{entry.get('target')}`: "
            f"available=`{entry.get('available')}`, stamp=`{entry.get('latest_timestamp_sec')}`, "
            f"reason=`{entry.get('reason') or '-'}`"
        )
    lines.extend(["", "## CALIBRATION", "", f"```json\n{json.dumps(report.get('CALIBRATION', {}), ensure_ascii=False, indent=2)}\n```", ""])
    lines.extend(["## CAPTURE", "", f"- `{json.dumps(report.get('CAPTURE', {}), ensure_ascii=False)}`", "", "## BLOCKERS", ""])
    blockers = report.get("BLOCKERS", [])
    lines.extend([f"- {blocker}" for blocker in blockers] or ["- none"])
    return "\n".join(lines) + "\n"


def _parse_args(argv: list[str] | None = None) -> tuple[PreflightOptions, list[str]]:
    parser = argparse.ArgumentParser(description="G2 Go2 hardware preflight and capture")
    parser.add_argument("--video-topic", default="/frontvideostream")
    parser.add_argument("--image-topic", default="/camera/go2/image_raw")
    parser.add_argument("--lidar-topic", default="/utlidar/cloud")
    parser.add_argument("--video-stream", choices=("video720p", "video360p", "video180p"), default="video720p")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--capture-count", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--calibration-json", type=Path, default=None)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--camera-frame", default="")
    parser.add_argument("--lidar-frame", default="")
    args, ros_args = parser.parse_known_args(argv)
    output_dir = args.output_dir or Path.cwd() / f"hardware_preflight_{time.strftime('%Y%m%d_%H%M%S')}"
    return PreflightOptions(
        video_topic=args.video_topic,
        image_topic=args.image_topic,
        lidar_topic=args.lidar_topic,
        video_stream=args.video_stream,
        duration_s=max(0.1, float(args.duration)),
        capture_count=max(0, int(args.capture_count)),
        output_dir=output_dir,
        calibration_json=args.calibration_json,
        map_frame=args.map_frame,
        odom_frame=args.odom_frame,
        base_frame=args.base_frame,
        camera_frame=args.camera_frame,
        lidar_frame=args.lidar_frame,
    ), ros_args


def main(argv: list[str] | None = None) -> int:
    options, ros_args = _parse_args(argv)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = HardwarePreflightNode(options)
        deadline = time.monotonic() + options.duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
    except Exception as exc:
        report = {
            "schema_version": 1,
            "tool": "g2_hardware_preflight",
            "READINESS": {},
            "BLOCKERS": [f"preflight runtime error: {exc}"],
        }
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    output_dir = options.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "hardware_preflight_report.json"
    markdown_path = output_dir / "hardware_preflight_report.md"
    json_path.write_text(json.dumps(_jsonable(report), ensure_ascii=False, indent=2), encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    print(f"hardware_preflight JSON: {json_path}")
    print(f"hardware_preflight Markdown: {markdown_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
