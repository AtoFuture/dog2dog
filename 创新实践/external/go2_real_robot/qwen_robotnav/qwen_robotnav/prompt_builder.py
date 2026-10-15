"""Natural-language prompt construction with viewpoint/temporal tags."""

from __future__ import annotations

from typing import List, Optional

from .config import NavigationConfig, TaskMode
from .observation_encoder import AllocatedFrame


class PromptBuilder:
    """Builds the visual-language sequence fed into Qwen3-VL.

    The model receives:
      - An embodiment-aware system preamble.
      - A sequence of images grouped by timestep, each prefixed by camera tags.
      - A task-specific navigation instruction.
      - A request for the waypoint trajectory.
    """

    def __init__(self, config: NavigationConfig):
        self.config = config

    def build_system_prompt(self) -> str:
        """Return the system prompt with embodiment preamble."""
        preamble = self.config.get_preamble()
        return (
            f"{preamble}\n\n"
            "You are given a sequence of egocentric camera observations. "
            "Predict a safe trajectory for the next few seconds as 8 waypoints. "
            "Each waypoint is given as (x, y, theta) in the robot's egocentric frame, "
            "where x is forward, y is left, and theta is the heading in radians."
        )

    def _task_instruction(self) -> str:
        """Wrap the user instruction with the current task mode."""
        mode = self.config.task_mode
        instruction = self.config.instruction

        if mode == TaskMode.VLN:
            return f"Follow this instruction: {instruction}"
        if mode == TaskMode.POINTNAV:
            return f"Navigate to the target: {instruction}"
        if mode == TaskMode.OBJNAV:
            return f"Search for and approach: {instruction}"
        if mode == TaskMode.TRACKING:
            return f"Track and follow: {instruction}"
        return instruction

    def build_user_text(
        self,
        allocated: List[AllocatedFrame],
        camera_order: Optional[List[str]] = None,
    ) -> str:
        """Build the textual part of the user prompt.

        Args:
            allocated: Output from ``ObservationEncoder.encode``.
            camera_order: Optional camera ordering; inferred from ``allocated`` if omitted.

        Returns:
            A string containing timestep headers and camera tags only (no base64).
            Images are supplied separately to the VLM client.
        """
        if not allocated:
            # Fallback: no images.
            return (
                f"Task mode: {self.config.task_mode.value}\n"
                f"{self._task_instruction()}\n\n"
                "No visual observations are available. Predict a safe forward trajectory."
            )

        if camera_order is None:
            seen: List[str] = []
            for frame in allocated:
                if frame.camera_name not in seen:
                    seen.append(frame.camera_name)
            camera_order = seen

        # Group by timestep index.
        groups: dict = {}
        for frame in allocated:
            groups.setdefault(frame.timestep, []).append(frame)

        lines: List[str] = []
        lines.append(f"Task mode: {self.config.task_mode.value}")
        lines.append(self._task_instruction())
        lines.append("")

        for t in sorted(groups.keys()):
            lines.append(f"Time step {t}:")
            # Preserve requested camera order.
            by_cam = {f.camera_name: f for f in groups[t]}
            for cam in camera_order:
                if cam in by_cam:
                    lines.append(f"  {cam} camera: <image>")
            lines.append("")

        lines.append(
            "Predict the next 8 waypoints (x, y, theta) in the robot egocentric frame. "
            "Return ONLY a JSON object like "
            "{\"waypoints\":[{\"x\":0.0,\"y\":0.0,\"theta\":0.0}, ...]} "
            "containing exactly 8 waypoint objects."
        )
        return "\n".join(lines)

    def build_messages(
        self,
        allocated: List[AllocatedFrame],
    ) -> tuple:
        """Build OpenAI-compatible messages and the list of PIL images.

        Returns:
            A tuple (messages, images) where ``messages`` is a list of dicts ready
            for the OpenAI chat completions API, and ``images`` is a flat list of
            PIL Images in the order they appear in the prompt.
        """
        system_prompt = self.build_system_prompt()
        user_text = self.build_user_text(allocated)

        # Extract images in the order they appear in ``user_text``.
        # Since <image> placeholders are ordered by timestep and camera, iterate allocated.
        images = [frame.image for frame in allocated]

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text},
        ]
        return messages, images
