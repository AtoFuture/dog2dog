"""Tests for VLM client waypoint parsing."""

from qwen_robotnav.vlm_client import VLMClient


def test_parse_waypoints_json_array():
    text = '[{"x": 0.1, "y": 0.2, "theta": 0.3}, {"x": 0.4, "y": 0.5, "theta": 0.6}]'
    waypoints = VLMClient._parse_waypoints(text)
    assert len(waypoints) == 2
    assert waypoints[0]["x"] == 0.1


def test_parse_waypoints_markdown():
    text = "Here are the waypoints:\n```json\n[{\"x\": 1.0, \"y\": 0.0, \"theta\": 0.1}]\n```"
    waypoints = VLMClient._parse_waypoints(text)
    assert len(waypoints) == 1
    assert waypoints[0]["x"] == 1.0


def test_normalise_waypoint():
    w = VLMClient._normalise_waypoint({"x": "0.5", "y": None, "theta": 1})
    assert w["x"] == 0.5
    assert w["y"] == 0.0
    assert w["theta"] == 1.0
