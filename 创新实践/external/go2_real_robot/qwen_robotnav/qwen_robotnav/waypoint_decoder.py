"""Waypoint denormalisation and coordinate transforms."""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from .config import WaypointDecoderConfig


class WaypointDecoder:
    """Convert normalised model outputs to robot egocentric waypoints."""

    def __init__(self, config: Optional[WaypointDecoderConfig] = None):
        self.config = config or WaypointDecoderConfig()

    def denormalise(
        self,
        waypoints: List[Dict[str, float]],
    ) -> np.ndarray:
        """Map [-1, 1] network outputs to metric waypoints.

        Args:
            waypoints: List of dicts with keys x, y, theta.

        Returns:
            Array of shape (K, 3) with columns [x, y, theta].
        """
        cfg = self.config
        arr = np.zeros((len(waypoints), 3), dtype=np.float64)
        for i, w in enumerate(waypoints):
            arr[i, 0] = float(w.get("x", 0.0)) * cfg.scale_x
            arr[i, 1] = float(w.get("y", 0.0)) * cfg.scale_y
            arr[i, 2] = float(w.get("theta", 0.0)) * cfg.scale_theta
        return arr

    def to_velocity_command(
        self,
        waypoints: np.ndarray,
        current_speed: float = 0.0,
        dt: float = 0.5,
        max_linear: float = 1.0,
        max_angular: float = 1.0,
    ) -> Dict[str, float]:
        """Convert the first predicted waypoint to a simple velocity command.

        Args:
            waypoints: Array of shape (K, 3) in egocentric frame.
            current_speed: Current forward speed (m/s).
            dt: Time horizon per waypoint (s).
            max_linear: Maximum linear speed (m/s).
            max_angular: Maximum angular speed (rad/s).

        Returns:
            Dict with keys ``linear_x`` and ``angular_z``.
        """
        if len(waypoints) == 0:
            return {"linear_x": 0.0, "angular_z": 0.0}

        first = waypoints[0]
        x, y, theta = first[0], first[1], first[2]

        # Desired heading towards the first waypoint.
        desired_heading = np.arctan2(y, x)

        # Desired linear speed to reach the waypoint in dt seconds.
        distance = np.hypot(x, y)
        desired_linear = min(distance / dt, max_linear)

        # Desired angular velocity (simple proportional).
        desired_angular = np.clip(theta / dt, -max_angular, max_angular)
        # Blend with heading correction to face the target.
        heading_gain = 1.0
        desired_angular = np.clip(
            desired_angular + heading_gain * desired_heading,
            -max_angular,
            max_angular,
        )

        return {"linear_x": float(desired_linear), "angular_z": float(desired_angular)}

    def smooth(self, waypoints: np.ndarray, alpha: float = 0.3) -> np.ndarray:
        """Apply exponential smoothing to a waypoint trajectory."""
        if len(waypoints) <= 1:
            return waypoints
        smoothed = waypoints.copy()
        for i in range(1, len(waypoints)):
            smoothed[i] = alpha * smoothed[i] + (1 - alpha) * smoothed[i - 1]
        return smoothed
