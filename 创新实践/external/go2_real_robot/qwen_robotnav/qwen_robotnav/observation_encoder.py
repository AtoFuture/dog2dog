"""Task-adaptive observation encoding (Algorithm 1 from the report)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

from .config import ObservationConfig


@dataclass
class AllocatedFrame:
    """One resized image together with its metadata."""

    image: Image.Image
    camera_name: str
    timestep: int
    tokens: int
    weight: float


class ObservationEncoder:
    """Encode a stream of multi-camera observations for the VLM.

    The implementation follows Algorithm 1 in the Qwen-RobotNav report:
      1. Sample frames according to ``frame_sample_mode``.
      2. Compute temporal weights with exponential recency bias.
      3. Form joint weight matrix W[t, c] = omega_t * w_c.
      4. Run constrained token allocation.
      5. Resize each image to its allocated pixel budget.
    """

    def __init__(self, config: Optional[ObservationConfig] = None):
        self.config = config or ObservationConfig()

    def sample_frames(
        self,
        frames: List[Dict[str, Image.Image]],
    ) -> List[Dict[str, Image.Image]]:
        """Sub-sample or select T' frames from the history.

        Args:
            frames: List of dicts mapping camera name to PIL Image, oldest first.

        Returns:
            A list with at most ``max_frames`` entries.
        """
        cfg = self.config
        t = len(frames)
        if t == 0:
            return []

        t_prime = min(t, cfg.max_frames)

        if cfg.frame_sample_mode == "latest":
            return frames[-t_prime:]

        # Random / history-covering mode: pick uniformly spaced frames plus the latest.
        if t_prime == 1:
            return [frames[-1]]

        indices = np.linspace(0, t - 1, t_prime, dtype=int)
        # Always include the latest frame.
        if indices[-1] != t - 1:
            indices[-1] = t - 1
        return [frames[int(i)] for i in indices]

    def temporal_weights(self, t_prime: int) -> np.ndarray:
        """Compute omega_t = exp(gamma * t / (T' - 1)) for t = 0..T'-1.

        Returns an array of length T' where the last entry is the most recent.
        """
        cfg = self.config
        if t_prime <= 1:
            return np.ones(1, dtype=np.float64)

        t = np.arange(t_prime, dtype=np.float64)
        gamma = cfg.temporal_decay
        omega = np.exp(gamma * t / (t_prime - 1))
        return omega

    def constrained_allocation(
        self,
        weights: np.ndarray,
        camera_names: List[str],
    ) -> np.ndarray:
        """Run constrained token allocation (Algorithm 1, lines 11-18).

        Args:
            weights: Array of shape (T', N) with joint weights W[t, c].
            camera_names: List of N camera names.

        Returns:
            Integer array A of shape (T', N) with allocated tokens per cell.
        """
        cfg = self.config
        t_prime, n_cam = weights.shape

        # Feasibility clipping for budget B.
        min_budget = t_prime * n_cam * cfg.token_floor
        max_budget = t_prime * n_cam * cfg.token_ceil
        budget = int(np.clip(cfg.token_budget, min_budget, max_budget))

        # Initial floor allocation.
        allocation = np.full((t_prime, n_cam), cfg.token_floor, dtype=np.int64)
        remaining = budget - allocation.sum()

        if remaining <= 0:
            return allocation

        # Iterative proportional redistribution with ceiling.
        active = np.ones_like(allocation, dtype=bool)
        while remaining > 0 and active.any():
            active_weights = np.where(active, weights, 0.0)
            total_weight = active_weights.sum()
            if total_weight <= 0:
                break

            share = (active_weights / total_weight) * remaining
            new_allocation = allocation + share

            # Clamp to ceiling and compute surplus.
            clamped = np.minimum(new_allocation, cfg.token_ceil)
            surplus = new_allocation - clamped

            # Any cell that hit the ceiling becomes inactive.
            active = clamped < cfg.token_ceil

            allocation = clamped.astype(np.int64)
            remaining = int(round(surplus.sum()))

        return allocation

    def tokens_to_resolution(
        self,
        tokens: int,
        aspect_ratio: float,
    ) -> Tuple[int, int]:
        """Map allocated tokens to a pixel resolution preserving aspect ratio.

        For Qwen3-VL / SigLIP-2 with dynamic resolution:
            tokens ≈ (width / patch_size / merge) * (height / patch_size / merge)
        We solve for (width, height) under the aspect ratio constraint.
        """
        cfg = self.config
        grid_factor = cfg.patch_size * cfg.spatial_merge_size

        # Approximate grid cells along the shorter axis given aspect ratio.
        if aspect_ratio >= 1.0:
            # width >= height
            h_cells = int(math.isqrt(int(tokens / aspect_ratio)))
            w_cells = int(round(tokens / max(h_cells, 1)))
        else:
            w_cells = int(math.isqrt(int(tokens * aspect_ratio)))
            h_cells = int(round(tokens / max(w_cells, 1)))

        # Ensure at least one cell.
        w_cells = max(w_cells, 1)
        h_cells = max(h_cells, 1)

        width = w_cells * grid_factor
        height = h_cells * grid_factor
        return width, height

    def encode(
        self,
        frames: List[Dict[str, Image.Image]],
        camera_order: Optional[List[str]] = None,
    ) -> List[AllocatedFrame]:
        """Run the full observation encoding pipeline.

        Args:
            frames: Multi-camera frame history, oldest first.
            camera_order: Optional fixed ordering for camera serialization.

        Returns:
            A flat list of allocated frames ready for prompt construction.
        """
        sampled = self.sample_frames(frames)
        if not sampled:
            return []

        # Determine camera order.
        if camera_order is None:
            camera_order = sorted(sampled[0].keys())
        n_cam = len(camera_order)
        t_prime = len(sampled)

        # Temporal weights: shape (T',)
        omega = self.temporal_weights(t_prime)

        # Joint weight matrix W[t, c].
        weights = np.zeros((t_prime, n_cam), dtype=np.float64)
        for ti, frame in enumerate(sampled):
            for ci, cam_name in enumerate(camera_order):
                w_c = self.config.camera_weights.get(cam_name, 1.0)
                weights[ti, ci] = omega[ti] * w_c

        allocation = self.constrained_allocation(weights, camera_order)

        # Resize each image according to its token allocation.
        allocated: List[AllocatedFrame] = []
        for ti, frame in enumerate(sampled):
            for ci, cam_name in enumerate(camera_order):
                img = frame.get(cam_name)
                if img is None:
                    continue
                tokens = int(allocation[ti, ci])
                aspect = img.width / max(img.height, 1)
                width, height = self.tokens_to_resolution(tokens, aspect)
                # Pillow < 9.1 exposes LANCZOS directly on Image; newer
                # releases moved it under Image.Resampling.
                resampling = getattr(Image, "Resampling", Image)
                resized = img.resize((width, height), resampling.LANCZOS)
                allocated.append(
                    AllocatedFrame(
                        image=resized,
                        camera_name=cam_name,
                        timestep=ti,
                        tokens=tokens,
                        weight=float(weights[ti, ci]),
                    )
                )

        return allocated
