"""Configuration dataclasses and enums for Qwen-RobotNav."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class TaskMode(str, Enum):
    """Navigation task modes exposed to the upper-level planner."""

    VLN = "VLN"          # Vision-and-language navigation (follow instruction)
    POINTNAV = "PointNav"  # Move to coordinate / waypoint
    OBJNAV = "ObjNav"    # Search for an object
    TRACKING = "Tracking"  # Follow a target


@dataclass
class CameraConfig:
    """Per-camera metadata used for viewpoint tagging and weighting."""

    name: str
    azimuth_deg: float  # approximate egocentric heading of the camera
    default_weight: float = 1.0


@dataclass
class ObservationConfig:
    """Task-adaptive observation parameters (Algorithm 1, Phi)."""

    # Total visual token budget across all cameras and timesteps.
    token_budget: int = 3072

    # Temporal decay gamma. 0 = uniform; larger = more tokens to recent frames.
    temporal_decay: float = 2.0

    # Per-camera importance weights, keyed by camera name.
    camera_weights: Dict[str, float] = field(default_factory=lambda: {
        "Front": 2.0,
        "Right": 1.0,
        "Back": 0.5,
        "Left": 1.0,
    })

    # Frame sampling mode: "random" for history coverage, "latest" for recency window.
    frame_sample_mode: str = "latest"

    # Number of historical frames to keep after sampling.
    max_frames: int = 4

    # Per-image token floor and ceiling.
    token_floor: int = 4
    token_ceil: int = 256

    # Native image resize target: encoder will rescale so that token count matches.
    # Qwen3-VL SigLIP-2 uses patch size 14 and spatial merge size 2 by default.
    patch_size: int = 14
    spatial_merge_size: int = 2


@dataclass
class ActionHeadConfig:
    """Lightweight MLP head mapping LLM hidden state to waypoints."""

    hidden_dim: int = 512
    num_layers: int = 4
    num_waypoints: int = 8
    waypoint_dim: int = 3  # (x, y, theta)


@dataclass
class WaypointDecoderConfig:
    """Denormalisation / coordinate transform parameters."""

    # Scale factors are dataset-dependent. We provide sensible defaults for indoor
    # quadruped operation (meters / radians). They should be calibrated per dataset.
    scale_x: float = 5.0
    scale_y: float = 5.0
    scale_theta: float = 3.14159


@dataclass
class VLMConfig:
    """Connection parameters for the DeepSeek Vision API (deepseek-flash)."""

    base_url: str = "https://api.deepseek.com"
    api_key: Optional[str] = None
    model: str = "deepseek-flash"
    # deepseek-flash emits reasoning_content before the final waypoint JSON.
    # 1024 tokens can end at finish_reason=length with an empty content field.
    max_tokens: int = 4096
    temperature: float = 0.2
    top_p: float = 0.95
    timeout: float = 120.0


@dataclass
class NavigationConfig:
    """Top-level configuration for a navigation call."""

    task_mode: TaskMode = TaskMode.VLN
    instruction: str = "Move forward safely."

    observation: ObservationConfig = field(default_factory=ObservationConfig)
    action_head: ActionHeadConfig = field(default_factory=ActionHeadConfig)
    waypoint_decoder: WaypointDecoderConfig = field(default_factory=WaypointDecoderConfig)
    vlm: VLMConfig = field(default_factory=VLMConfig)

    # Robot embodiment preamble.
    embodiment: str = "quadruped robot"
    embodiment_preamble: Optional[str] = None

    # Camera configuration, in the order they will be serialised.
    cameras: List[CameraConfig] = field(default_factory=lambda: [
        CameraConfig("Front", 0.0, 2.0),
        CameraConfig("Right", 90.0, 1.0),
        CameraConfig("Back", 180.0, 0.5),
        CameraConfig("Left", -90.0, 1.0),
    ])

    def get_preamble(self) -> str:
        """Return the natural-language embodiment preamble."""
        if self.embodiment_preamble:
            return self.embodiment_preamble
        return (
            f"Imagine you are a {self.embodiment} programmed for navigation tasks. "
            "You observe the world through onboard cameras and predict a sequence of "
            "future waypoints (x, y, theta) in your egocentric frame."
        )
