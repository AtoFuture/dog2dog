"""End-to-end mock navigation test using synthetic frames and the remote VLM."""

import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageDraw


def create_corridor_frame(
    step: int,
    size: tuple = (640, 480),
    num_obstacles: int = 3,
    target_x: float = 320.0,
) -> Image.Image:
    """Draw a synthetic corridor scene with a target marker."""
    img = Image.new("RGB", size, (180, 180, 180))
    draw = ImageDraw.Draw(img)

    # Floor / ceiling lines converging to target.
    w, h = size
    vanishing = (int(target_x), h // 3)
    draw.line([(0, 0), vanishing], fill=(120, 120, 120), width=2)
    draw.line([(w, 0), vanishing], fill=(120, 120, 120), width=2)
    draw.line([(0, h), vanishing], fill=(120, 120, 120), width=2)
    draw.line([(w, h), vanishing], fill=(120, 120, 120), width=2)

    # Obstacles as rectangles receding into the corridor.
    rng = np.random.default_rng(seed=step)
    for _ in range(num_obstacles):
        y = rng.integers(h // 3, h - 40)
        x = rng.integers(50, w - 50)
        color = tuple(rng.integers(50, 200, 3).tolist())
        draw.rectangle([x, y, x + 40, y + 40], fill=color, outline=(0, 0, 0))

    # Target marker ahead.
    draw.ellipse(
        [int(target_x) - 15, h // 3 - 15, int(target_x) + 15, h // 3 + 15],
        fill=(0, 255, 0),
        outline=(0, 0, 0),
    )

    return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=3)
    parser.add_argument("--instruction", default="Move forward through the corridor toward the green target.")
    parser.add_argument("--task-mode", default="VLN")
    parser.add_argument(
        "--base-url",
        default=os.environ.get("VLLM_BASE_URL", "http://100.113.176.95:8000/v1"),
    )
    parser.add_argument(
        "--model",
        default=os.environ.get("VLLM_MODEL", "qwen3-vl-fp8"),
    )
    args = parser.parse_args()

    from qwen_robotnav import (
        NavigationConfig,
        NavigationEngine,
        ObservationConfig,
        TaskMode,
        VLMConfig,
    )

    try:
        task_mode = TaskMode(args.task_mode)
    except ValueError:
        print(f"Invalid task mode: {args.task_mode}", file=sys.stderr)
        sys.exit(1)

    cfg = NavigationConfig(
        task_mode=task_mode,
        instruction=args.instruction,
        embodiment="quadruped robot",
        vlm=VLMConfig(
            base_url=args.base_url,
            model=args.model,
            api_key=(
                os.environ.get("DEEPSEEK_API_KEY")
                or os.environ.get("VLLM_API_KEY")
            ),
        ),
        observation=ObservationConfig(
            token_budget=2048,
            temporal_decay=2.0,
            frame_sample_mode="latest",
            max_frames=args.steps,
            camera_weights={"Front": 1.0},
        ),
        cameras=[type("CameraConfig", (), {"name": "Front", "azimuth_deg": 0.0, "default_weight": 1.0})()],
    )

    engine = NavigationEngine(cfg)

    print("Generating synthetic frame history ...")
    frames = []
    for i in range(args.steps):
        img = create_corridor_frame(i)
        frames.append({"Front": img})

    print(f"Calling Qwen-RobotNav with instruction: {args.instruction}")
    waypoints, raw_text = engine(frames)

    print("\n=== RAW VLM OUTPUT ===")
    print(raw_text)
    print("=== WAYPOINTS ===")
    for i, w in enumerate(waypoints):
        print(f"  {i}: x={w[0]:.3f}, y={w[1]:.3f}, theta={w[2]:.3f}")

    cmd = engine.decoder.to_velocity_command(waypoints)
    print("=== VELOCITY COMMAND ===")
    print(f"  linear_x={cmd['linear_x']:.3f}, angular_z={cmd['angular_z']:.3f}")


if __name__ == "__main__":
    main()
