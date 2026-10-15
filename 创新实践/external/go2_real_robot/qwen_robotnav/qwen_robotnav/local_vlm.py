"""Local Qwen3-VL inference on Jetson for offline operation.

This module provides a drop-in replacement for ``VLMClient`` that loads
Qwen3-VL-4B/8B directly on the Jetson. It is used when the remote vLLM
service is unreachable or when the robot must operate without network access.
"""

from __future__ import annotations

import json
import re
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

from .config import VLMConfig
from .vlm_client import VLMClient


class LocalVLM(VLMClient):
    """Run Qwen3-VL locally with transformers.

    The implementation mirrors ``VLMClient.predict_waypoints`` so callers can
    switch between remote and local inference by swapping the client instance.
    """

    def __init__(self, config: Optional[VLMConfig] = None):
        super().__init__(config)
        self._processor = None
        self._model = None

    def _load(self):
        """Lazy-load model and processor."""
        if self._model is not None:
            return

        try:
            from transformers import AutoProcessor, Qwen3VLForConditionalGeneration
        except ImportError as exc:
            raise RuntimeError(
                "transformers and torch are required for LocalVLM. "
                "Install them with: pip install transformers torch torchvision"
            ) from exc

        model_id = self.config.model

        # Prefer ModelScope cache if it exists; otherwise fall back to HF.
        import os
        from pathlib import Path

        cache_dir = os.environ.get("MODELSCOPE_CACHE", "/workspace/qwen_robotnav/.modelscope")
        # ModelScope snapshot_download stores models under $MODELSCOPE_CACHE/models/<model_id>.
        potential_path = Path(cache_dir) / "models" / model_id
        if potential_path.exists():
            model_id = str(potential_path)

        self._processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
        self._model = Qwen3VLForConditionalGeneration.from_pretrained(
            model_id,
            torch_dtype="auto",
            device_map="auto",
            trust_remote_code=True,
        )

    def predict_waypoints(
        self,
        messages: List[Dict[str, str]],
        images: List[Image.Image],
    ) -> Tuple[List[Dict[str, float]], str]:
        """Run local Qwen3-VL and parse waypoint JSON."""
        self._load()

        # Interleave images into the user message.
        request_messages = []
        for msg in messages:
            if msg.get("role") == "user":
                content = self._build_content(msg["content"], images)
                request_messages.append({"role": "user", "content": content})
            else:
                request_messages.append(msg)

        # Apply chat template.
        text = self._processor.apply_chat_template(
            request_messages,
            tokenize=False,
            add_generation_prompt=True,
        )
        inputs = self._processor(
            text=[text],
            images=[np.array(img) for img in images],
            return_tensors="pt",
            padding=True,
        ).to(self._model.device)

        generated_ids = self._model.generate(
            **inputs,
            max_new_tokens=self.config.max_tokens,
            temperature=self.config.temperature,
            top_p=self.config.top_p,
        )
        generated_ids = generated_ids[:, inputs["input_ids"].size(1):]
        raw_text = self._processor.batch_decode(
            generated_ids, skip_special_tokens=True
        )[0]

        waypoints = self._parse_waypoints(raw_text)
        return waypoints, raw_text
