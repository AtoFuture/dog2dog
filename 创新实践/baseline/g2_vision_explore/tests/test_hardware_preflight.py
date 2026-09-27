"""Deterministic tests for Go2 hardware preflight helpers.

These tests use synthetic ROS messages only. They do not claim any real Go2 topic,
clock, calibration, TF, or hardware result.
"""

from __future__ import annotations

import numpy as np
import pytest
import cv2

from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from vision_detector.calibrate_intrinsics import calibrate_from_images
from vision_detector.hardware_preflight import (
    H264PayloadStats,
    build_readiness,
    infer_time_frame_interpretation,
    inspect_h264_payload,
    render_markdown,
    summarize_pointcloud_message,
    tf_unavailable,
    topic_status,
    validate_calibration_data,
)


def _topic_node(names_and_types=None, publishers=None):
    class _Node:
        def get_topic_names_and_types(self):
            return names_and_types or []

        def get_publishers_info_by_topic(self, _topic):
            return publishers or []

    return _Node()


def test_h264_payload_detects_annexb_sps_pps_idr():
    payload = (
        b"\x00\x00\x00\x01\x67\x64\x00\x1f"
        b"\x00\x00\x01\x68\xee\x3c\x80"
        b"\x00\x00\x01\x65\x88\x84"
    )
    result = inspect_h264_payload(payload)

    assert result["has_annex_b_start_code"] is True
    assert result["has_sps"] is True
    assert result["has_pps"] is True
    assert result["has_idr"] is True
    assert result["nal_types"] == [5, 7, 8]


def test_h264_stats_preserve_ros_message_boundaries_as_observations_only():
    stats = H264PayloadStats()
    stats.add(b"\x00\x00\x01\x67", 100, 1.0)
    stats.add(b"\x00\x00\x01\x65", 120, 1.1)
    summary = stats.summary()

    assert summary["messages"] == 2
    assert summary["payload_bytes"]["min"] == 4
    assert summary["payload_bytes"]["max"] == 4
    assert summary["time_frame_raw"]["deltas"] == [20.0]
    assert summary["annex_b_messages"] == 2


def test_pointcloud_summary_reports_layout_frame_and_invalid_xyz():
    header = Header()
    header.frame_id = "actual_lidar_frame"
    header.stamp.sec = 12
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg = point_cloud2.create_cloud(
        header,
        fields,
        [[1.0, 2.0, 3.0, 0.5], [np.nan, 0.0, 1.0, 0.2], [4.0, 5.0, np.inf, 0.1]],
    )

    result = summarize_pointcloud_message(msg)

    assert result["frame_id"] == "actual_lidar_frame"
    assert result["width"] == 3
    assert result["point_step"] == 16
    assert {field["name"] for field in result["fields"]} == {"x", "y", "z", "intensity"}
    assert result["xyz_points"] == 3
    assert result["valid_xyz_points"] == 1
    assert result["nan_rows"] == 1
    assert result["inf_rows"] == 1
    assert result["ranges"] == {"x": [1.0, 1.0], "y": [2.0, 2.0], "z": [3.0, 3.0]}


def test_missing_topic_probe_is_a_reported_blocker_not_exception():
    result = topic_status(_topic_node(), "/missing", "sensor_msgs/msg/Image")

    assert result["exists"] is False
    assert result["publisher_count"] == 0
    assert result["error"] is None


def test_time_frame_interpretation_is_explicitly_only_a_hypothesis():
    result = infer_time_frame_interpretation({"deltas": [1.0, 1.0, 1.0]})

    assert result["status"] == "INFERRED"
    assert result["used_for_ros_timestamp"] is False
    assert "could be a frame counter" in result["hypotheses"]


def test_missing_tf_entry_is_serializable_and_actionable():
    result = tf_unavailable("camera_actual", "base_link", "lookup failed")

    assert result["available"] is False
    assert result["source"] == "camera_actual"
    assert result["target"] == "base_link"
    assert result["reason"] == "lookup failed"


def test_placeholder_calibration_never_becomes_valid():
    result = validate_calibration_data({
        "lidar_fx": 0.0,
        "lidar_fy": 0.0,
        "lidar_cx": 0.0,
        "lidar_cy": 0.0,
        "lidar_image_width": 0,
        "lidar_image_height": 0,
        "lidar_frame": "utlidar_lidar",
        "lidar_camera_frame": "",
        "lidar_translation_xyz": [0.0, 0.0, 0.0],
        "lidar_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    })

    assert result["intrinsics_valid"] is False
    assert result["extrinsic_valid"] is False


def test_readiness_report_distinguishes_rgb_only_from_fusion():
    topic_ok = {"exists": True, "type_matches": True, "publisher_count": 1}
    readiness = build_readiness(
        video_topic=topic_ok,
        image_topic=topic_ok,
        lidar_topic=topic_ok,
        image_messages=3,
        cloud_messages=3,
        valid_xyz_points=100,
        calibration={"intrinsics_valid": False, "extrinsic_valid": False},
        tf_entries=[],
    )

    assert readiness["READY_FOR_RGB_ONLY"]["ready"] is True
    assert readiness["READY_FOR_LIDAR_LOCALIZATION"]["ready"] is False
    assert readiness["READY_FOR_RGB_LIDAR_FUSION"]["status"] == "blocked"
    assert readiness["CLOCK_SYNC"]["status"] == "unconfirmed"
    markdown = render_markdown({"READINESS": readiness})
    assert "READY_FOR_RGB_ONLY" in markdown
    assert "unconfirmed" in markdown


def test_readiness_requires_directional_tf_and_confirmed_clock_for_fusion():
    topic_ok = {"exists": True, "type_matches": True, "publisher_count": 1}
    calibration_ok = {"intrinsics_valid": True, "extrinsic_valid": True}
    tf_entries = [
        {"source": "map", "target": "base_link", "available": True},
        {"source": "base_link", "target": "camera", "available": True},
        {"source": "base_link", "target": "lidar", "available": True},
    ]

    readiness = build_readiness(
        video_topic=topic_ok,
        image_topic=topic_ok,
        lidar_topic=topic_ok,
        image_messages=3,
        cloud_messages=3,
        valid_xyz_points=100,
        calibration=calibration_ok,
        tf_entries=tf_entries,
        required_tf_pairs=[
            ("map", "base_link"),
            ("base_link", "camera"),
            ("base_link", "lidar"),
        ],
    )

    assert readiness["READY_FOR_LIDAR_LOCALIZATION"]["ready"] is True
    assert readiness["READY_FOR_RGB_LIDAR_FUSION"]["ready"] is False
    assert readiness["CLOCK_SYNC"]["status"] == "unconfirmed"
    assert "clock domain" in readiness["READY_FOR_RGB_LIDAR_FUSION"]["reasons"][-1]


def test_intrinsics_tool_refuses_empty_directory_without_output(tmp_path):
    output = tmp_path / "intrinsics.json"

    with pytest.raises(FileNotFoundError):
        calibrate_from_images(tmp_path, pattern_size=(9, 6), square_size_m=0.025)

    assert not output.exists()


def test_intrinsics_tool_emits_explicit_intrinsics_only_result(tmp_path):
    columns, rows, square = 10, 7, 30
    board = np.full((rows * square, columns * square), 255, dtype=np.uint8)
    for y in range(rows):
        for x in range(columns):
            if (x + y) % 2 == 0:
                board[y * square:(y + 1) * square, x * square:(x + 1) * square] = 0

    source = np.float32([
        [0, 0],
        [columns * square, 0],
        [columns * square, rows * square],
        [0, rows * square],
    ])
    placements = [
        [[45, 45], [435, 35], [445, 315], [35, 325]],
        [[60, 35], [430, 55], [440, 320], [50, 305]],
        [[35, 60], [445, 45], [420, 325], [60, 310]],
    ]
    for index, points in enumerate(placements):
        target = np.float32(points)
        matrix = cv2.getPerspectiveTransform(source, target)
        image = cv2.warpPerspective(board, matrix, (480, 360), borderValue=255)
        assert cv2.imwrite(str(tmp_path / f"view_{index}.png"), image)

    result = calibrate_from_images(tmp_path, pattern_size=(9, 6), square_size_m=0.025)

    assert result["calibration_status"] == "intrinsics_only"
    assert result["image_count"] == 3
    assert result["image_width"] == 480
    assert result["image_height"] == 360
    assert result["fx"] > 0.0
    assert result["fy"] > 0.0
    assert result["rms_reprojection_error_px"] >= 0.0
    assert result["camera_lidar_extrinsic"] is None
    assert result["lidar_frame"] == ""
