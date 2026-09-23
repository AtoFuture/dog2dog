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


def return_message(attempt_id, status, goal_id=None):
    return SimpleNamespace(
        data=json.dumps(
            {
                "attempt_id": attempt_id,
                "status": status,
                "goal_id": goal_id,
            }
        )
    )


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
