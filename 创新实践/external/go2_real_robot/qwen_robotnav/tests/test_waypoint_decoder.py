"""Unit tests for waypoint denormalisation and velocity commands."""

import numpy as np

from qwen_robotnav import WaypointDecoder, WaypointDecoderConfig


def test_denormalise():
    cfg = WaypointDecoderConfig(scale_x=5.0, scale_y=5.0, scale_theta=np.pi)
    decoder = WaypointDecoder(cfg)
    raw = [{"x": 0.5, "y": -0.5, "theta": 1.0}]
    out = decoder.denormalise(raw)
    assert out.shape == (1, 3)
    assert np.isclose(out[0, 0], 2.5)
    assert np.isclose(out[0, 1], -2.5)
    assert np.isclose(out[0, 2], np.pi)


def test_velocity_command_forward():
    decoder = WaypointDecoder()
    waypoints = np.array([[1.0, 0.0, 0.0]])
    cmd = decoder.to_velocity_command(waypoints, dt=0.5)
    assert cmd["linear_x"] > 0
    assert np.isclose(cmd["angular_z"], 0.0, atol=0.1)


def test_velocity_command_turn():
    decoder = WaypointDecoder()
    waypoints = np.array([[0.0, 0.5, 0.0]])
    cmd = decoder.to_velocity_command(waypoints, dt=0.5)
    assert cmd["angular_z"] != 0.0


def test_empty_waypoints():
    decoder = WaypointDecoder()
    cmd = decoder.to_velocity_command(np.zeros((0, 3)))
    assert cmd["linear_x"] == 0.0
    assert cmd["angular_z"] == 0.0


def test_smooth():
    decoder = WaypointDecoder()
    waypoints = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    smoothed = decoder.smooth(waypoints, alpha=0.5)
    assert smoothed.shape == waypoints.shape
    assert smoothed[1, 0] == 0.5
