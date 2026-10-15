"""Unit tests for the task-adaptive observation encoder."""

import numpy as np
import pytest
from PIL import Image

from qwen_robotnav import ObservationConfig, ObservationEncoder


def _make_frames(num_steps: int, num_cams: int = 2, size: tuple = (640, 480)):
    frames = []
    for _ in range(num_steps):
        frame = {}
        for c in range(num_cams):
            color = tuple(np.random.randint(0, 255, 3).tolist())
            frame[f"cam{c}"] = Image.new("RGB", size, color)
        frames.append(frame)
    return frames


def test_sample_latest():
    cfg = ObservationConfig(frame_sample_mode="latest", max_frames=3)
    enc = ObservationEncoder(cfg)
    frames = _make_frames(10)
    sampled = enc.sample_frames(frames)
    assert len(sampled) == 3


def test_sample_random():
    cfg = ObservationConfig(frame_sample_mode="random", max_frames=4)
    enc = ObservationEncoder(cfg)
    frames = _make_frames(10)
    sampled = enc.sample_frames(frames)
    assert len(sampled) == 4
    assert sampled[-1] == frames[-1]


def test_temporal_weights():
    enc = ObservationEncoder(ObservationConfig(temporal_decay=2.0))
    omega = enc.temporal_weights(4)
    assert len(omega) == 4
    assert omega[-1] > omega[0]
    # gamma=2 -> newest/oldest ratio approx exp(2) = 7.389
    assert np.isclose(omega[-1] / omega[0], np.exp(2.0), rtol=0.05)


def test_constrained_allocation():
    cfg = ObservationConfig(
        token_budget=128,
        token_floor=4,
        token_ceil=64,
        temporal_decay=2.0,
        camera_weights={"cam0": 2.0, "cam1": 1.0},
    )
    enc = ObservationEncoder(cfg)
    weights = np.array([[1.0, 0.5], [2.0, 1.0], [4.0, 2.0]])
    allocation = enc.constrained_allocation(weights, ["cam0", "cam1"])
    assert allocation.shape == (3, 2)
    assert allocation.min() >= cfg.token_floor
    assert allocation.max() <= cfg.token_ceil
    assert allocation.sum() <= cfg.token_budget
    # Last timestep should receive more tokens due to recency bias.
    assert allocation[-1].sum() >= allocation[0].sum()


def test_encode_pipeline():
    cfg = ObservationConfig(
        token_budget=256,
        max_frames=2,
        camera_weights={"Front": 2.0, "Back": 0.5},
    )
    enc = ObservationEncoder(cfg)
    frames = [
        {"Front": Image.new("RGB", (640, 480), (255, 0, 0)),
         "Back": Image.new("RGB", (640, 480), (0, 0, 255))},
        {"Front": Image.new("RGB", (640, 480), (0, 255, 0)),
         "Back": Image.new("RGB", (640, 480), (0, 0, 255))},
    ]
    allocated = enc.encode(frames, camera_order=["Front", "Back"])
    assert len(allocated) == 4  # 2 timesteps x 2 cameras
    front_tokens = sum(a.tokens for a in allocated if a.camera_name == "Front")
    back_tokens = sum(a.tokens for a in allocated if a.camera_name == "Back")
    assert front_tokens > back_tokens
