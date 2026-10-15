"""Lightweight MLP action head that maps LLM hidden state to waypoints."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .config import ActionHeadConfig


class ActionHead:
    """4-layer MLP with hidden dimension 512 and GELU activations.

    The report predicts K=8 waypoints each with 3 DoF (x, y, theta) giving a
    24-dimensional output. Since a remote OpenAI-compatible endpoint does not
    expose hidden states, this head is provided as a drop-in module for:
      - Future hidden-state based inference.
      - Local refinement of text-generated waypoints (optional).

    The implementation intentionally has no train-time dependencies: weights can
    be loaded from a checkpoint if one becomes available, otherwise the module
    can act as a deterministic identity / placeholder.
    """

    def __init__(self, config: Optional[ActionHeadConfig] = None):
        self.config = config or ActionHeadConfig()
        self._weights_loaded = False

    @property
    def input_dim(self) -> int:
        # Qwen3-VL 30B-A3B hidden dimension. This is model-specific.
        # For the FP8 MoE variant the hidden size is 4096.
        return 4096

    @property
    def output_dim(self) -> int:
        return self.config.num_waypoints * self.config.waypoint_dim

    def load_weights(self, path: str) -> None:
        """Load MLP weights from a numpy or torch checkpoint."""
        # Placeholder: real weights would be supplied by the Qwen-RobotNav release.
        import warnings

        warnings.warn(f"ActionHead.load_weights not implemented; checkpoint {path} ignored.")
        self._weights_loaded = False

    def forward(self, hidden_state: np.ndarray) -> np.ndarray:
        """Run the MLP forward pass.

        Args:
            hidden_state: Array of shape (..., input_dim).

        Returns:
            Waypoints of shape (..., num_waypoints, waypoint_dim).
        """
        # Without real weights, return a deterministic zero trajectory. A real
        # deployment would run the PyTorch MLP here.
        shape = hidden_state.shape[:-1] + (
            self.config.num_waypoints,
            self.config.waypoint_dim,
        )
        return np.zeros(shape, dtype=np.float32)

    def refine_waypoints(
        self,
        waypoints: np.ndarray,
        hidden_state: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Optional local refinement of already-generated waypoints.

        Currently a no-op identity. Override with trained weights when available.
        """
        return np.asarray(waypoints, dtype=np.float32)
