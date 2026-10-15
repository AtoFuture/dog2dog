import importlib
import json
import sys
from types import ModuleType, SimpleNamespace

import pytest


class FakeReader:
    def __init__(self, topics, messages):
        self._topics = topics
        self._messages = iter(messages)
        self._next = None

    def open(self, _storage_options, _converter_options):
        pass

    def get_all_topics_and_types(self):
        return [
            SimpleNamespace(name=name, type=message_type)
            for name, message_type in self._topics.items()
        ]

    def has_next(self):
        if self._next is not None:
            return True
        try:
            self._next = next(self._messages)
        except StopIteration:
            return False
        return True

    def read_next(self):
        value = self._next
        self._next = None
        return value


def load_evaluator(monkeypatch, topics, messages):
    rosbag2_py = ModuleType("rosbag2_py")
    rosbag2_py.SequentialReader = lambda: FakeReader(topics, messages)
    rosbag2_py.StorageOptions = lambda **kwargs: kwargs
    rosbag2_py.ConverterOptions = lambda **kwargs: kwargs

    serialization = ModuleType("rclpy.serialization")
    serialization.deserialize_message = lambda raw, _message_type: raw
    rclpy = ModuleType("rclpy")
    rclpy.serialization = serialization

    utilities = ModuleType("rosidl_runtime_py.utilities")
    utilities.get_message = lambda _type_name: object
    rosidl_runtime_py = ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.utilities = utilities

    monkeypatch.setitem(sys.modules, "rosbag2_py", rosbag2_py)
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.serialization", serialization)
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py", rosidl_runtime_py)
    monkeypatch.setitem(
        sys.modules,
        "rosidl_runtime_py.utilities",
        utilities,
    )
    sys.modules.pop("g3_integration_eval.evaluate_bag", None)
    return importlib.import_module("g3_integration_eval.evaluate_bag")


def return_message(attempt_id, status, goal_id=None, timestamp=None):
    event = {
        "attempt_id": attempt_id,
        "status": status,
        "goal_id": goal_id,
    }
    if timestamp is not None:
        event["timestamp"] = timestamp
    return SimpleNamespace(
        data=json.dumps(event)
    )


def nav_goal_id(values=range(16)):
    return SimpleNamespace(uuid=list(values))


def nav_feedback(values=range(16)):
    return SimpleNamespace(goal_id=nav_goal_id(values))


def nav_status(status, values=range(16)):
    return SimpleNamespace(
        status_list=[
            SimpleNamespace(
                goal_info=SimpleNamespace(
                    goal_id=nav_goal_id(values),
                ),
                status=status,
            )
        ]
    )


def detection_3d_message(*, x=1.0, y=2.0, z=0.0, sec=1, nanosec=0):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        class_name="person",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z)
            )
        ),
    )


def write_detection_ground_truth(tmp_path, detections):
    path = tmp_path / "ground_truth.json"
    path.write_text(
        json.dumps({"detections": detections}),
        encoding="utf-8",
    )
    return str(path)


def test_old_bag_keeps_original_return_unavailable_reason(tmp_path, monkeypatch):
    evaluator = load_evaluator(
        monkeypatch,
        {"/clock": "rosgraph_msgs/msg/Clock"},
        [
            ("/clock", SimpleNamespace(), 1_000_000_000),
            ("/clock", SimpleNamespace(), 2_000_000_000),
        ],
    )

    report = evaluator.evaluate_bag(str(tmp_path))

    assert report["metrics"]["return_success_rate"] == {
        "available": False,
        "value": None,
        "reason": "no_explicit_return_goal_marker",
    }
    assert report["metrics"]["return_duration"]["available"] is False
    assert report["metrics"]["return_duration"]["reason"] == (
        "return_event_topic_missing"
    )
    assert report["metrics"]["detection_rate"]["reason"] == (
        "detection topics not present in this bag"
    )


def test_snapshot_only_bag_does_not_enable_detection_metrics(
    tmp_path, monkeypatch
):
    snapshot_topic = "/detections_snapshot"
    ground_truth = write_detection_ground_truth(
        tmp_path,
        [{"t_sec": 1.0, "class": "person", "x": 1.0, "y": 2.0, "z": 0.0}],
    )
    evaluator = load_evaluator(
        monkeypatch,
        {snapshot_topic: "vision_interfaces/msg/DetectionSnapshot"},
        [(snapshot_topic, SimpleNamespace(), 1_000_000_000)],
    )

    report = evaluator.evaluate_bag(
        str(tmp_path), ground_truth_detections=ground_truth
    )

    for metric_name in ("detection_rate", "false_positive_rate"):
        metric = report["metrics"][metric_name]
        assert metric["available"] is False
        assert metric["value"] is None
        assert metric["reason"] == "detection topics not present in this bag"
    assert report["metrics"]["detection_detail"] == {
        "ground_truth_count": 1
    }


def test_detection_3d_matches_ground_truth(tmp_path, monkeypatch):
    detection_topic = "/detections_3d"
    ground_truth = write_detection_ground_truth(
        tmp_path,
        [{"t_sec": 1.0, "class": "person", "x": 1.0, "y": 2.0, "z": 0.0}],
    )
    evaluator = load_evaluator(
        monkeypatch,
        {detection_topic: "vision_interfaces/msg/Detection3D"},
        [(detection_topic, detection_3d_message(), 1_000_000_000)],
    )

    report = evaluator.evaluate_bag(
        str(tmp_path), ground_truth_detections=ground_truth
    )

    assert report["metrics"]["detection_rate"]["value"] == pytest.approx(1.0)
    assert report["metrics"]["false_positive_rate"]["value"] == pytest.approx(
        0.0
    )
    assert report["metrics"]["detection_detail"]["true_positives"] == 1


def test_detection_3d_false_positive_is_counted(tmp_path, monkeypatch):
    detection_topic = "/detections_3d"
    ground_truth = write_detection_ground_truth(
        tmp_path,
        [{"t_sec": 1.0, "class": "person", "x": 1.0, "y": 2.0, "z": 0.0}],
    )
    evaluator = load_evaluator(
        monkeypatch,
        {detection_topic: "vision_interfaces/msg/Detection3D"},
        [
            (detection_topic, detection_3d_message(), 1_000_000_000),
            (
                detection_topic,
                detection_3d_message(x=8.0, y=9.0),
                1_100_000_000,
            ),
        ],
    )

    report = evaluator.evaluate_bag(
        str(tmp_path), ground_truth_detections=ground_truth
    )

    assert report["metrics"]["detection_rate"]["value"] == pytest.approx(1.0)
    assert report["metrics"]["false_positive_rate"]["value"] == pytest.approx(
        0.5
    )
    assert report["metrics"]["detection_detail"]["false_positives"] == 1


def test_return_events_produce_offline_success_rate(tmp_path, monkeypatch):
    topic = "/g3/return_event"
    evaluator = load_evaluator(
        monkeypatch,
        {topic: "std_msgs/msg/String"},
        [
            (topic, return_message("a", "accepted", "goal-a"), 1),
            (topic, return_message("a", "succeeded", "goal-a"), 2),
            (topic, return_message("b", "rejected", "goal-b"), 3),
        ],
    )

    report = evaluator.evaluate_bag(str(tmp_path))

    assert report["metrics"]["return_success_rate"]["value"] == pytest.approx(0.5)
    assert report["return_navigation"]["attempt_count"] == 2
    assert report["return_navigation"]["goal_ids"] == ["goal-a", "goal-b"]


def test_return_duration_is_added_to_report_without_changing_success_rate(
    tmp_path, monkeypatch
):
    topic = "/g3/return_event"
    evaluator = load_evaluator(
        monkeypatch,
        {topic: "std_msgs/msg/String"},
        [
            (
                topic,
                return_message(
                    "a", "waiting_for_server", timestamp={"sec": 10, "nanosec": 0}
                ),
                10_000_000_000,
            ),
            (
                topic,
                return_message(
                    "a", "succeeded", timestamp={"sec": 13, "nanosec": 500_000_000}
                ),
                13_500_000_000,
            ),
        ],
    )

    report = evaluator.evaluate_bag(str(tmp_path))

    assert report["metrics"]["return_success_rate"]["value"] == pytest.approx(1.0)
    assert report["metrics"]["return_duration"]["average_seconds"] == pytest.approx(
        3.5
    )
    assert report["return_navigation"]["attempt_durations"][0] == {
        "attempt_id": "a",
        "status": "succeeded",
        "available": True,
        "duration_seconds": pytest.approx(3.5),
    }


def test_state_topics_report_occurrences_transitions_and_durations(
    tmp_path, monkeypatch
):
    g3_state = "/g3/state"
    mission_state = "/mission_state"
    evaluator = load_evaluator(
        monkeypatch,
        {
            g3_state: "std_msgs/msg/String",
            mission_state: "std_msgs/msg/String",
        },
        [
            (g3_state, SimpleNamespace(data="EXPLORE"), 1_000_000_000),
            (g3_state, SimpleNamespace(data="RETURN"), 5_000_000_000),
            (mission_state, SimpleNamespace(data="exploring"), 1_000_000_000),
            (mission_state, SimpleNamespace(data="returning"), 5_000_000_000),
        ],
    )

    report = evaluator.evaluate_bag(str(tmp_path))

    assert report["state_statistics"][g3_state]["state_counts"] == {
        "EXPLORE": 1,
        "RETURN": 1,
    }
    assert report["state_statistics"][g3_state]["transition_count"] == 1
    assert report["state_statistics"][g3_state]["duration_summary"][
        "EXPLORE"
    ]["average_seconds"] == pytest.approx(4.0)
    assert report["state_statistics"][mission_state]["transition_count"] == 1


def test_malformed_return_event_is_reported_not_fatal(tmp_path, monkeypatch):
    topic = "/g3/return_event"
    evaluator = load_evaluator(
        monkeypatch,
        {topic: "std_msgs/msg/String"},
        [
            (topic, SimpleNamespace(data="not json"), 1),
            (topic, return_message("a", "preview"), 2),
        ],
    )

    report = evaluator.evaluate_bag(str(tmp_path))

    assert report["return_navigation"]["invalid_events"] == 1
    assert report["return_navigation"]["preview"] == 1
    assert report["metrics"]["return_success_rate"]["reason"] == (
        "no_terminal_return_attempt_observed"
    )


def test_namespaced_navigation_action_is_evaluated(tmp_path, monkeypatch):
    action = "/robot1/navigate_to_pose"
    feedback_topic = f"{action}/_action/feedback"
    status_topic = f"{action}/_action/status"
    evaluator = load_evaluator(
        monkeypatch,
        {
            feedback_topic: "nav2_msgs/action/NavigateToPose_FeedbackMessage",
            status_topic: "action_msgs/msg/GoalStatusArray",
        },
        [
            (feedback_topic, nav_feedback(), 1),
            (status_topic, nav_status(2), 2),
            (status_topic, nav_status(4), 3),
        ],
    )

    report = evaluator.evaluate_bag(
        str(tmp_path),
        navigate_action="robot1/navigate_to_pose/",
    )

    assert report["navigation"]["action_name"] == action
    assert report["navigation"]["feedback_topic"] == feedback_topic
    assert report["navigation"]["status_topic"] == status_topic
    assert report["navigation"]["feedback_messages"] == 1
    assert report["navigation"]["status_messages"] == 2
    assert report["navigation"]["succeeded"] == 1
    assert report["metrics"]["navigation_success_rate"][
        "value"
    ] == pytest.approx(1.0)


def test_custom_battery_topic_is_evaluated(tmp_path, monkeypatch):
    topic = "/g3/sim_battery_state"
    evaluator = load_evaluator(
        monkeypatch,
        {topic: "sensor_msgs/msg/BatteryState"},
        [
            (topic, SimpleNamespace(percentage=0.30), 1),
            (topic, SimpleNamespace(percentage=0.20), 2),
        ],
    )

    report = evaluator.evaluate_bag(
        str(tmp_path),
        battery_topic="g3/sim_battery_state/",
    )

    assert report["battery"]["topic"] == topic
    assert report["battery"]["sample_count"] == 2
    assert report["battery"]["minimum"] == pytest.approx(0.20)
    assert report["battery"]["maximum"] == pytest.approx(0.30)
