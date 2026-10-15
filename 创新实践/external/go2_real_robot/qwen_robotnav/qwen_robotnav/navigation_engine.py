"""High-level navigation call: observation encoding -> VLM -> waypoints."""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

from .action_head import ActionHead
from .config import NavigationConfig, VLMConfig
from .local_vlm import LocalVLM
from .observation_encoder import ObservationEncoder
from .prompt_builder import PromptBuilder
from .vlm_client import VLMClient
from .waypoint_decoder import WaypointDecoder


class NavigationEngine:
    """End-to-end navigation executor used by the agentic system.

    A single call corresponds to ``nav_qwennav(L_i, tau_i, Phi_i)`` in the report:
      - ``L_i``: instruction stored in ``NavigationConfig``
      - ``tau_i``: task mode stored in ``NavigationConfig``
      - ``Phi_i``: observation configuration stored in ``NavigationConfig``
    """

    def __init__(self, config: Optional[NavigationConfig] = None):
        self.config = config or NavigationConfig()
        self.encoder = ObservationEncoder(self.config.observation)
        self.prompt_builder = PromptBuilder(self.config)
        self.vlm_client = self._create_vlm_client(self.config.vlm)
        self.action_head = ActionHead(self.config.action_head)
        self.decoder = WaypointDecoder(self.config.waypoint_decoder)

    @staticmethod
    def _create_vlm_client(vlm_config: VLMConfig) -> VLMClient:
        """Create remote or local VLM client based on model name.

        A model is treated as local only if it is explicitly prefixed with
        ``local:`` or is a HuggingFace-style model id (contains a slash). Remote
        service aliases such as ``qwen3-vl-fp8`` should use the provided
        ``base_url`` and the VLMClient remote implementation.
        """
        model_lower = vlm_config.model.lower()
        if model_lower.startswith("local:"):
            local_model = vlm_config.model.replace("local:", "", 1)
            return LocalVLM(VLMConfig(
                base_url="",
                model=local_model,
                max_tokens=vlm_config.max_tokens,
                temperature=vlm_config.temperature,
                top_p=vlm_config.top_p,
            ))
        if "/" in vlm_config.model:
            return LocalVLM(VLMConfig(
                base_url="",
                model=vlm_config.model,
                max_tokens=vlm_config.max_tokens,
                temperature=vlm_config.temperature,
                top_p=vlm_config.top_p,
            ))
        return VLMClient(vlm_config)

    def __call__(
        self,
        frames: List[Dict[str, Image.Image]],
        instruction: Optional[str] = None,
    ) -> Tuple[np.ndarray, str]:
        """Run one navigation step.

        Args:
            frames: Multi-camera frame history, oldest first.
            instruction: Optional override for the configured instruction.

        Returns:
            Tuple of (waypoints array of shape (K, 3), raw VLM text).
        """
        if instruction is not None:
            self.config.instruction = instruction

        allocated = self.encoder.encode(
            frames,
            camera_order=[c.name for c in self.config.cameras],
        )
        messages, images = self.prompt_builder.build_messages(allocated)
        waypoints_raw, raw_text = self.vlm_client.predict_waypoints(messages, images)
        # Both remote and local text backends receive the same metric prompt.
        # Scaling by the learned-head normalization factors here multiplies
        # distances by five and yaw by pi a second time.
        waypoints = self.decoder.from_metric(waypoints_raw)
        waypoints = self.decoder.smooth(waypoints)
        return waypoints, raw_text

    def get_velocity_command(
        self,
        frames: List[Dict[str, Image.Image]],
        instruction: Optional[str] = None,
    ) -> Dict[str, float]:
        """Convenience wrapper returning a velocity command for the robot."""
        waypoints, _ = self(frames, instruction)
        return self.decoder.to_velocity_command(waypoints)
