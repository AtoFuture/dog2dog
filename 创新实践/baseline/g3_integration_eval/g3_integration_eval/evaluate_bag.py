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
    detection_metrics,
    duration_seconds,
    navigation_statistics,
    return_statistics,
    return_timing_statistics,
    state_statistics,
)


BATTERY_TOPIC = "/robot1/battery_state"
DEFAULT_NAVIGATE_ACTION = "/robot1/navigate_to_pose"
RETURN_EVENT_TOPIC = "/g3/return_event"

DETECTION_TOPIC = "/detections_3d"
DETECTION_SNAPSHOT_TOPIC = "/detections_snapshot"


def navigation_topic_names(action_name: str) -> tuple[str, str, str]:
    """Return the canonical action name and its feedback/status topics."""
    if not isinstance(action_name, str) or not action_name.strip():
        raise ValueError("navigate action name must not be empty")

    canonical_name = "/" + action_name.strip().strip("/")
    if canonical_name == "/":
        raise ValueError("navigate action name must not be root")

    return (
        canonical_name,
        f"{canonical_name}/_action/feedback",
        f"{canonical_name}/_action/status",
    )


def canonical_topic_name(topic_name: str) -> str:
    """Return a canonical absolute ROS topic name."""
    if not isinstance(topic_name, str) or not topic_name.strip():
        raise ValueError("topic name must not be empty")
    canonical_name = "/" + topic_name.strip().strip("/")
    if canonical_name == "/":
        raise ValueError("topic name must not be root")
    return canonical_name


def evaluate_bag(
    bag_path: str,
    reference_map: str = None,
    slam_map: str = None,
    navigate_action: str = DEFAULT_NAVIGATE_ACTION,
    battery_topic: str = BATTERY_TOPIC,
    ground_truth_detections: str = None,
) -> dict:
    (
        navigate_action,
        nav_feedback_topic,
        nav_status_topic,
    ) = navigation_topic_names(navigate_action)
    battery_topic = canonical_topic_name(battery_topic)

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
    return_event_msg_type = None
    detection_msg_type = None
    state_msg_types = {}

    if battery_topic in topic_types:
        battery_msg_type = get_message(
            topic_types[battery_topic]
        )

    if nav_feedback_topic in topic_types:
        nav_feedback_msg_type = get_message(
            topic_types[nav_feedback_topic]
        )

    if nav_status_topic in topic_types:
        nav_status_msg_type = get_message(
            topic_types[nav_status_topic]
        )

    if RETURN_EVENT_TOPIC in topic_types:
        return_event_msg_type = get_message(
            topic_types[RETURN_EVENT_TOPIC]
        )

    if DETECTION_TOPIC in topic_types:
        detection_msg_type = get_message(
            topic_types[DETECTION_TOPIC]
        )

    for state_topic in ("/g3/state", "/mission_state"):
        if state_topic in topic_types:
            state_msg_types[state_topic] = get_message(
                topic_types[state_topic]
            )

    observed_nav_goal_ids = set()
    nav_statuses_by_goal = {}
    return_events = []
    invalid_return_events = 0
    actual_detections = []
    state_samples = {
        "/g3/state": [],
        "/mission_state": [],
    }

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
            topic_name == battery_topic
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
            topic_name == nav_feedback_topic
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
            topic_name == nav_status_topic
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

        elif (
            topic_name == RETURN_EVENT_TOPIC
            and return_event_msg_type is not None
        ):
            message = deserialize_message(
                raw_data,
                return_event_msg_type,
            )
            try:
                event = json.loads(message.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                invalid_return_events += 1
            else:
                if isinstance(event, dict):
                    return_events.append(event)
                else:
                    invalid_return_events += 1

        elif topic_name in state_msg_types:
            message = deserialize_message(
                raw_data,
                state_msg_types[topic_name],
            )
            state_samples[topic_name].append(
                (getattr(message, "data", None), timestamp_ns)
            )

        elif (
            topic_name == DETECTION_TOPIC
            and detection_msg_type is not None
        ):
            message = deserialize_message(
                raw_data,
                detection_msg_type,
            )
            t_sec = (
                message.stamp.sec
                + message.stamp.nanosec * 1e-9
            )
            actual_detections.append({
                "t_sec": t_sec,
                "class": message.class_name,
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "z": float(message.pose.pose.position.z),
                "source": getattr(message, "source", ""),
                "id": int(getattr(message, "id", 0)),
            })

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

    returns = return_statistics(return_events)
    return_timing = return_timing_statistics(return_events)
    state_reports = {}
    for topic_name, samples in state_samples.items():
        if topic_name not in topic_types:
            state_reports[topic_name] = {
                "available": False,
                "message_count": 0,
                "state_counts": {},
                "transition_count": 0,
                "transitions": [],
                "duration_summary": {},
                "reason": "topic_not_present_in_bag",
            }
        else:
            state_reports[topic_name] = state_statistics(samples)

    navigation = navigation_statistics(
        observed_nav_goal_ids,
        nav_statuses_by_goal,
        return_goal_ids=returns.goal_ids,
    )

    navigation_success_metric = availability(
        navigation.success_rate,
        (
            None
            if navigation.success_rate is not None
            else "no_terminal_navigation_goal_observed"
        ),
    )

    if RETURN_EVENT_TOPIC not in topic_types:
        return_success_metric = availability(
            None,
            "no_explicit_return_goal_marker",
        )
    else:
        return_success_metric = availability(
            returns.success_rate,
            (
                None
                if returns.success_rate is not None
                else "no_terminal_return_attempt_observed"
            ),
        )

    detection_present = (
        DETECTION_TOPIC in topic_types
        or DETECTION_SNAPSHOT_TOPIC in topic_types
    )

    gt_detections = []
    gt_match_radius_m = 0.5
    gt_match_time_s = 1.0
    gt_loaded = False

    if ground_truth_detections is not None:
        gt_path = Path(ground_truth_detections).expanduser().resolve()
        if gt_path.exists():
            with gt_path.open("r", encoding="utf-8") as f:
                gt_data = json.load(f)
            gt_detections = gt_data.get("detections", [])
            gt_match_radius_m = float(
                gt_data.get("match_radius_m", 0.5)
            )
            gt_match_time_s = float(
                gt_data.get("match_time_s", 1.0)
            )
            gt_loaded = True
        else:
            raise FileNotFoundError(
                f"ground-truth file not found: {gt_path}"
            )

    if gt_loaded and detection_present:
        det_metrics = detection_metrics(
            gt_detections,
            actual_detections,
            match_radius_m=gt_match_radius_m,
            match_time_s=gt_match_time_s,
        )
        detection_rate_metric = availability(
            det_metrics.detection_rate,
            (
                None
                if det_metrics.detection_rate is not None
                else "no ground-truth detections"
            ),
        )
        false_positive_rate_metric = availability(
            det_metrics.false_positive_rate,
            (
                None
                if det_metrics.false_positive_rate is not None
                else "no detections in bag"
            ),
        )
        detection_detail = {
            "ground_truth_count": det_metrics.ground_truth_count,
            "detection_count": det_metrics.detection_count,
            "true_positives": det_metrics.true_positives,
            "false_positives": det_metrics.false_positives,
            "false_negatives": det_metrics.false_negatives,
            "match_radius_m": gt_match_radius_m,
            "match_time_s": gt_match_time_s,
        }
    elif gt_loaded and not detection_present:
        detection_rate_metric = availability(
            None, "detection topics not present in this bag"
        )
        false_positive_rate_metric = availability(
            None, "detection topics not present in this bag"
        )
        detection_detail = {"ground_truth_count": len(gt_detections)}
    else:
        detection_rate_metric = availability(
            None,
            (
                "ground-truth detections not provided"
                if detection_present
                else "detection topics not present in this bag"
            ),
        )
        false_positive_rate_metric = availability(
            None,
            (
                "ground-truth detections not provided"
                if detection_present
                else "detection topics not present in this bag"
            ),
        )
        detection_detail = None

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
            "action_name": navigate_action,
            "feedback_topic": nav_feedback_topic,
            "status_topic": nav_status_topic,
            "feedback_messages": topic_counts.get(
                nav_feedback_topic,
                0,
            ),
            "status_messages": topic_counts.get(
                nav_status_topic,
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
            "preempted": navigation.preempted,
            "terminal_goal_count": (
                navigation.terminal_goal_count
            ),
            "observed_goal_ids": list(
                navigation.observed_goal_ids
            ),
            "conflicting_goal_ids": list(
                navigation.conflicting_goal_ids
            ),
            "preempted_goal_ids": list(
                navigation.preempted_goal_ids
            ),
        },
        "return_navigation": {
            "event_messages": topic_counts.get(
                RETURN_EVENT_TOPIC,
                0,
            ),
            "valid_events": returns.event_count,
            "invalid_events": invalid_return_events,
            "attempt_count": returns.attempt_count,
            "preview": returns.preview,
            "succeeded": returns.succeeded,
            "aborted": returns.aborted,
            "canceled": returns.canceled,
            "rejected": returns.rejected,
            "server_unavailable": returns.server_unavailable,
            "error": returns.error,
            "unknown": returns.unknown,
            "incomplete": returns.incomplete,
            "conflict": returns.conflict,
            "terminal_attempt_count": returns.terminal_attempt_count,
            "goal_ids": list(returns.goal_ids),
            "conflicting_attempt_ids": list(
                returns.conflicting_attempt_ids
            ),
            "attempt_durations": [
                {
                    "attempt_id": attempt.attempt_id,
                    "status": attempt.status,
                    "available": attempt.available,
                    "duration_seconds": attempt.duration_seconds,
                    **(
                        {"reason": attempt.reason}
                        if attempt.reason
                        else {}
                    ),
                }
                for attempt in return_timing.attempts
            ],
        },
        "state_statistics": state_reports,
        "battery": {
            "topic": battery_topic,
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
            "return_duration": {
                "available": (
                    return_timing.available
                    and RETURN_EVENT_TOPIC in topic_types
                ),
                "average_seconds": return_timing.average_seconds,
                "minimum_seconds": return_timing.minimum_seconds,
                "maximum_seconds": return_timing.maximum_seconds,
                "successful_attempt_count": (
                    return_timing.successful_attempt_count
                ),
                "measured_successful_attempt_count": (
                    return_timing.measured_successful_attempt_count
                ),
                "unavailable_successful_attempt_count": (
                    return_timing.unavailable_successful_attempt_count
                ),
                **(
                    {}
                    if return_timing.available
                    and RETURN_EVENT_TOPIC in topic_types
                    else {
                        "reason": (
                            "return_event_topic_missing"
                            if RETURN_EVENT_TOPIC not in topic_types
                            else return_timing.reason
                        )
                    }
                ),
            },
            "ate": availability(
                None,
                "ground-truth trajectory not provided",
            ),
            "detection_rate": detection_rate_metric,
            "false_positive_rate": false_positive_rate_metric,
            "detection_detail": detection_detail,
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

    parser.add_argument(
        "--navigate-action",
        default=DEFAULT_NAVIGATE_ACTION,
        help=(
            "NavigateToPose action name. Its _action/feedback and "
            "_action/status topics are evaluated (default: "
            f"{DEFAULT_NAVIGATE_ACTION})."
        ),
    )

    parser.add_argument(
        "--battery-topic",
        default=BATTERY_TOPIC,
        help=(
            "BatteryState topic evaluated (default: "
            f"{BATTERY_TOPIC})."
        ),
    )

    parser.add_argument(
        "--ground-truth-detections",
        help=(
            "Path to a JSON file with ground-truth detections for "
            "detection-rate / false-positive-rate computation."
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
        navigate_action=args.navigate_action,
        battery_topic=args.battery_topic,
        ground_truth_detections=args.ground_truth_detections,
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
    if return_success["available"]:
        print(
            "Return success rate: "
            f"{return_success['value']:.2%}"
        )
    else:
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

    detection_rate = report["metrics"]["detection_rate"]
    if detection_rate["available"]:
        print(f"Detection rate: {detection_rate['value']:.2%}")
    else:
        print(f"Detection rate: unavailable ({detection_rate['reason']})")

    fpr = report["metrics"]["false_positive_rate"]
    if fpr["available"]:
        print(f"False positive rate: {fpr['value']:.2%}")
    else:
        print(f"False positive rate: unavailable ({fpr['reason']})")

    print(
        f"Report: {report_file}"
    )


if __name__ == "__main__":
    main()
