"""Offline rosbag evaluator for G3 experiments."""

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from g3_integration_eval.coverage import calculate_coverage
from g3_integration_eval.metrics import (
    availability,
    battery_statistics,
    duration_seconds,
    navigation_statistics,
)


BATTERY_TOPIC = "/robot1/battery_state"
NAV_FEEDBACK_TOPIC = "/robot1/navigate_to_pose/_action/feedback"
NAV_STATUS_TOPIC = "/robot1/navigate_to_pose/_action/status"

DETECTION_TOPIC = "/detections_3d"
DETECTION_SNAPSHOT_TOPIC = "/detections_snapshot"


def evaluate_bag(
    bag_path: str,
    reference_map: str = None,
    slam_map: str = None,
) -> dict:
    bag_dir = Path(bag_path).expanduser().resolve()

    if not bag_dir.exists():
        raise FileNotFoundError(
            f"bag does not exist: {bag_dir}"
        )

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_dir),
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

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
    nav_feedback_msg_type = None
    nav_status_msg_type = None

    if BATTERY_TOPIC in topic_types:
        battery_msg_type = get_message(
            topic_types[BATTERY_TOPIC]
        )

    if NAV_FEEDBACK_TOPIC in topic_types:
        nav_feedback_msg_type = get_message(
            topic_types[NAV_FEEDBACK_TOPIC]
        )

    if NAV_STATUS_TOPIC in topic_types:
        nav_status_msg_type = get_message(
            topic_types[NAV_STATUS_TOPIC]
        )

    observed_nav_goal_ids = set()
    nav_statuses_by_goal = {}

    while reader.has_next():
        topic_name, raw_data, timestamp_ns = (
            reader.read_next()
        )

        topic_counts[topic_name] = (
            topic_counts.get(topic_name, 0) + 1
        )

        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns

        last_timestamp_ns = timestamp_ns

        if (
            topic_name == BATTERY_TOPIC
            and battery_msg_type is not None
        ):
            message = deserialize_message(
                raw_data,
                battery_msg_type,
            )

            battery_values.append(
                message.percentage
            )

        elif (
            topic_name == NAV_FEEDBACK_TOPIC
            and nav_feedback_msg_type is not None
        ):
            message = deserialize_message(
                raw_data,
                nav_feedback_msg_type,
            )
            observed_nav_goal_ids.add(
                bytes(message.goal_id.uuid).hex()
            )

        elif (
            topic_name == NAV_STATUS_TOPIC
            and nav_status_msg_type is not None
        ):
            message = deserialize_message(
                raw_data,
                nav_status_msg_type,
            )

            for item in message.status_list:
                goal_id = bytes(
                    item.goal_info.goal_id.uuid
                ).hex()
                nav_statuses_by_goal.setdefault(
                    goal_id,
                    [],
                ).append(int(item.status))

    if (
        first_timestamp_ns is None
        or last_timestamp_ns is None
    ):
        raise RuntimeError(
            "bag contains no messages"
        )

    battery = battery_statistics(
        battery_values
    )

    duration = duration_seconds(
        first_timestamp_ns,
        last_timestamp_ns,
    )

    navigation = navigation_statistics(
        observed_nav_goal_ids,
        nav_statuses_by_goal,
    )

    navigation_success_metric = availability(
        navigation.success_rate,
        (
            None
            if navigation.success_rate is not None
            else "no_terminal_navigation_goal_observed"
        ),
    )

    return_success_metric = availability(
        None,
        "no_explicit_return_goal_marker",
    )

    detection_present = (
        DETECTION_TOPIC in topic_types
        or DETECTION_SNAPSHOT_TOPIC in topic_types
    )

    if reference_map is not None and slam_map is not None:
        coverage_result = calculate_coverage(
            reference_map,
            slam_map,
        )

        coverage_metric = {
            "available": True,
            "value": coverage_result.coverage,
            "percent": (
                coverage_result.coverage * 100.0
            ),
            "reference_free_cells": (
                coverage_result.reference_free_cells
            ),
            "observed_reference_free_cells": (
                coverage_result.observed_reference_free_cells
            ),
            "observed_as_free": (
                coverage_result.observed_as_free
            ),
            "observed_as_occupied": (
                coverage_result.observed_as_occupied
            ),
            "outside_slam_canvas": (
                coverage_result.outside_slam_canvas
            ),
            "reference_map": str(
                Path(reference_map)
                .expanduser()
                .resolve()
            ),
            "slam_map": str(
                Path(slam_map)
                .expanduser()
                .resolve()
            ),
        }
    else:
        coverage_metric = availability(
            None,
            "reference map and SLAM map not provided",
        )

    report = {
        "experiment": bag_dir.name,
        "bag_path": str(bag_dir),
        "start_timestamp_ns": first_timestamp_ns,
        "end_timestamp_ns": last_timestamp_ns,
        "duration_seconds": duration,
        "total_messages": sum(
            topic_counts.values()
        ),
        "topic_counts": dict(
            sorted(topic_counts.items())
        ),
        "navigation": {
            "feedback_messages": topic_counts.get(
                NAV_FEEDBACK_TOPIC,
                0,
            ),
            "status_messages": topic_counts.get(
                NAV_STATUS_TOPIC,
                0,
            ),
            "observed_goal_count": (
                navigation.observed_goal_count
            ),
            "succeeded": navigation.succeeded,
            "aborted": navigation.aborted,
            "canceled": navigation.canceled,
            "incomplete": navigation.incomplete,
            "conflict": navigation.conflict,
            "terminal_goal_count": (
                navigation.terminal_goal_count
            ),
            "observed_goal_ids": list(
                navigation.observed_goal_ids
            ),
            "conflicting_goal_ids": list(
                navigation.conflicting_goal_ids
            ),
        },
        "battery": {
            "sample_count": battery.sample_count,
            "minimum": battery.minimum,
            "maximum": battery.maximum,
            "average": battery.average,
        },
        "metrics": {
            "coverage": coverage_metric,
            "navigation_success_rate": (
                navigation_success_metric
            ),
            "return_success_rate": (
                return_success_metric
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


def write_report(
    report: dict,
    output_dir: str,
) -> Path:
    output_path = (
        Path(output_dir)
        .expanduser()
        .resolve()
    )

    output_path.mkdir(
        parents=True,
        exist_ok=True,
    )

    report_file = (
        output_path
        / f"{report['experiment']}_report.json"
    )

    with report_file.open(
        "w",
        encoding="utf-8",
    ) as file:
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
        description=(
            "Evaluate a G3 ROS 2 experiment rosbag."
        )
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

    parser.add_argument(
        "--reference-map",
        help=(
            "Reference map YAML used as the "
            "coverage denominator."
        ),
    )

    parser.add_argument(
        "--slam-map",
        help=(
            "SLAM map YAML used to determine "
            "observed reference area."
        ),
    )

    args = parser.parse_args()

    if bool(args.reference_map) != bool(args.slam_map):
        parser.error(
            "--reference-map and --slam-map "
            "must be provided together"
        )

    report = evaluate_bag(
        args.bag_path,
        reference_map=args.reference_map,
        slam_map=args.slam_map,
    )

    report_file = write_report(
        report,
        args.output_dir,
    )

    print(
        f"Experiment: {report['experiment']}"
    )
    print(
        f"Duration: "
        f"{report['duration_seconds']:.3f} s"
    )
    print(
        f"Messages: {report['total_messages']}"
    )

    print(
        "NavigateToPose feedback:",
        report["navigation"]["feedback_messages"],
    )

    print(
        "NavigateToPose status:",
        report["navigation"]["status_messages"],
    )

    navigation = report["navigation"]

    print(
        "Observed navigation goals:",
        navigation["observed_goal_count"],
    )
    print(
        "Navigation outcomes: "
        f"succeeded={navigation['succeeded']} "
        f"aborted={navigation['aborted']} "
        f"canceled={navigation['canceled']} "
        f"incomplete={navigation['incomplete']} "
        f"conflict={navigation['conflict']}"
    )

    navigation_success = (
        report["metrics"]["navigation_success_rate"]
    )
    if navigation_success["available"]:
        print(
            "Navigation success rate: "
            f"{navigation_success['value']:.2%}"
        )
    else:
        print(
            "Navigation success rate: unavailable "
            f"({navigation_success['reason']})"
        )

    return_success = (
        report["metrics"]["return_success_rate"]
    )
    print(
        "Return success rate: unavailable "
        f"({return_success['reason']})"
    )

    battery = report["battery"]

    print(
        f"Battery samples: "
        f"{battery['sample_count']}"
    )

    if battery["sample_count"] > 0:
        print(
            f"Battery min: "
            f"{battery['minimum']:.3f}"
        )
        print(
            f"Battery max: "
            f"{battery['maximum']:.3f}"
        )
        print(
            f"Battery avg: "
            f"{battery['average']:.3f}"
        )
    else:
        print("Battery: unavailable")

    coverage = report["metrics"]["coverage"]

    if coverage["available"]:
        print(
            f"Coverage: "
            f"{coverage['percent']:.2f}%"
        )
    else:
        print("Coverage: unavailable")

    print("ATE: unavailable")
    print("Detection metrics: unavailable")

    print(
        f"Report: {report_file}"
    )


if __name__ == "__main__":
    main()
