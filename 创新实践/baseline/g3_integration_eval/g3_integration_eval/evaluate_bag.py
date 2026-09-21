"""Offline rosbag evaluator for G3 experiments."""

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from g3_integration_eval.metrics import (
    availability,
    battery_statistics,
    duration_seconds,
)


BATTERY_TOPIC = "/robot1/battery_state"
NAV_FEEDBACK_TOPIC = "/robot1/navigate_to_pose/_action/feedback"
NAV_STATUS_TOPIC = "/robot1/navigate_to_pose/_action/status"

DETECTION_TOPIC = "/detections_3d"
DETECTION_SNAPSHOT_TOPIC = "/detections_snapshot"


def evaluate_bag(bag_path: str) -> dict:
    bag_dir = Path(bag_path).expanduser().resolve()

    if not bag_dir.exists():
        raise FileNotFoundError(f"bag does not exist: {bag_dir}")

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id="sqlite3",
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader.open(storage_options, converter_options)

    topic_metadata = reader.get_all_topics_and_types()
    topic_types = {
        item.name: item.type
        for item in topic_metadata
    }

    topic_counts = {
        topic_name: 0
        for topic_name in topic_types
    }

    battery_values = []
    first_timestamp_ns = None
    last_timestamp_ns = None

    battery_msg_type = None
    if BATTERY_TOPIC in topic_types:
        battery_msg_type = get_message(topic_types[BATTERY_TOPIC])

    while reader.has_next():
        topic_name, raw_data, timestamp_ns = reader.read_next()

        topic_counts[topic_name] = topic_counts.get(topic_name, 0) + 1

        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns
        last_timestamp_ns = timestamp_ns

        if topic_name == BATTERY_TOPIC and battery_msg_type is not None:
            message = deserialize_message(raw_data, battery_msg_type)
            battery_values.append(message.percentage)

    if first_timestamp_ns is None or last_timestamp_ns is None:
        raise RuntimeError("bag contains no messages")

    battery = battery_statistics(battery_values)

    duration = duration_seconds(
        first_timestamp_ns,
        last_timestamp_ns,
    )

    detection_present = (
        DETECTION_TOPIC in topic_types
        or DETECTION_SNAPSHOT_TOPIC in topic_types
    )

    report = {
        "experiment": bag_dir.name,
        "bag_path": str(bag_dir),
        "start_timestamp_ns": first_timestamp_ns,
        "end_timestamp_ns": last_timestamp_ns,
        "duration_seconds": duration,
        "total_messages": sum(topic_counts.values()),
        "topic_counts": dict(sorted(topic_counts.items())),
        "navigation": {
            "feedback_messages": topic_counts.get(
                NAV_FEEDBACK_TOPIC,
                0,
            ),
            "status_messages": topic_counts.get(
                NAV_STATUS_TOPIC,
                0,
            ),
        },
        "battery": {
            "sample_count": battery.sample_count,
            "minimum": battery.minimum,
            "maximum": battery.maximum,
            "average": battery.average,
        },
        "metrics": {
            "coverage": availability(
                None,
                "coverage evaluator not implemented in first version",
            ),
            "ate": availability(
                None,
                "ground-truth trajectory not provided",
            ),
            "detection_rate": availability(
                None,
                (
                    "ground-truth detections not provided"
                    if detection_present
                    else "detection topics not present in this bag"
                ),
            ),
            "false_positive_rate": availability(
                None,
                (
                    "ground-truth detections not provided"
                    if detection_present
                    else "detection topics not present in this bag"
                ),
            ),
        },
    }

    return report


def write_report(report: dict, output_dir: str) -> Path:
    output_path = Path(output_dir).expanduser().resolve()
    output_path.mkdir(parents=True, exist_ok=True)

    report_file = output_path / (
        f"{report['experiment']}_report.json"
    )

    with report_file.open("w", encoding="utf-8") as file:
        json.dump(
            report,
            file,
            ensure_ascii=False,
            indent=2,
        )
        file.write("\n")

    return report_file


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate a G3 ROS 2 experiment rosbag."
    )
    parser.add_argument(
        "bag_path",
        help="Path to the rosbag2 directory.",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/wy/g3_reports",
        help="Report output directory.",
    )

    args = parser.parse_args()

    report = evaluate_bag(args.bag_path)
    report_file = write_report(report, args.output_dir)

    print(f"Experiment: {report['experiment']}")
    print(f"Duration: {report['duration_seconds']:.3f} s")
    print(f"Messages: {report['total_messages']}")

    print(
        "NavigateToPose feedback:",
        report["navigation"]["feedback_messages"],
    )
    print(
        "NavigateToPose status:",
        report["navigation"]["status_messages"],
    )

    battery = report["battery"]

    print(f"Battery samples: {battery['sample_count']}")

    if battery["sample_count"] > 0:
        print(f"Battery min: {battery['minimum']:.3f}")
        print(f"Battery max: {battery['maximum']:.3f}")
        print(f"Battery avg: {battery['average']:.3f}")
    else:
        print("Battery: unavailable")

    print("ATE: unavailable")
    print("Coverage: unavailable")
    print("Detection metrics: unavailable")

    print(f"Report: {report_file}")


if __name__ == "__main__":
    main()
