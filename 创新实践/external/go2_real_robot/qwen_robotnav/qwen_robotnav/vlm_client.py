"""Client for the DeepSeek Vision API (deepseek-flash, OpenAI-compatible)."""

from __future__ import annotations

import base64
import asyncio
import json
import math
import re
from io import BytesIO
from typing import Any, Dict, List, Optional, Tuple

from PIL import Image

from .config import VLMConfig


class VLMClient:
    """Thin wrapper around the DeepSeek Vision API (OpenAI-compatible).

    Uses ``deepseek-flash`` (DeepSeek-V4.1-Flash) which accepts images alongside
    text via the standard ``image_url`` content block with base64 data URLs.
    The client asks the VLM to emit a JSON waypoint list.
    """

    def __init__(self, config: Optional[VLMConfig] = None):
        self.config = config or VLMConfig()
        if not math.isfinite(self.config.timeout) or self.config.timeout <= 0:
            raise ValueError("VLM timeout must be finite and positive")

    async def _request_completion(self, request_kwargs):
        """Cancel the entire HTTP request even if the provider sends keep-alives."""
        try:
            from openai import AsyncOpenAI
        except ImportError as exc:
            raise RuntimeError(
                "openai package is required for VLMClient. "
                "Install it with: pip install openai"
            ) from exc

        async def request():
            # Each synchronous call owns its event loop and HTTP client. Never
            # reuse an async connection pool after asyncio.run closes its loop.
            async with AsyncOpenAI(
                base_url=self.config.base_url,
                api_key=self.config.api_key or "not-needed",
                timeout=self.config.timeout,
                max_retries=0,
            ) as client:
                return await client.chat.completions.create(**request_kwargs)

        try:
            return await asyncio.wait_for(request(), timeout=self.config.timeout)
        except asyncio.TimeoutError as exc:
            raise TimeoutError(
                f"VLM request exceeded {self.config.timeout:g}s total deadline"
            ) from exc

    @staticmethod
    def _pil_to_base64(image: Image.Image, fmt: str = "PNG") -> str:
        """Encode a PIL image to a base64 data URL."""
        buffer = BytesIO()
        # Convert palette/RGBA to RGB if necessary for JPEG.
        if fmt.upper() == "JPEG" and image.mode in ("RGBA", "P"):
            image = image.convert("RGB")
        image.save(buffer, format=fmt)
        b64 = base64.b64encode(buffer.getvalue()).decode("utf-8")
        return f"data:image/{fmt.lower()};base64,{b64}"

    def _build_content(
        self,
        text: str,
        images: List[Image.Image],
    ) -> List[Dict[str, Any]]:
        """Build OpenAI chat content with interleaved image data URLs.

        The prompt text contains ``<image>`` placeholders. We split the text and
        insert image_url parts at the corresponding positions.
        """
        parts: List[Dict[str, Any]] = []
        placeholder = "<image>"

        segments = text.split(placeholder)
        for i, segment in enumerate(segments):
            if segment:
                parts.append({"type": "text", "text": segment})
            if i < len(images):
                data_url = self._pil_to_base64(images[i])
                parts.append({"type": "image_url", "image_url": {"url": data_url}})

        return parts

    def predict_waypoints(
        self,
        messages: List[Dict[str, str]],
        images: List[Image.Image],
    ) -> Tuple[List[Dict[str, float]], str]:
        """Call the remote VLM and parse a JSON waypoint list.

        Args:
            messages: OpenAI-compatible message list (system + user).
            images: PIL Images to be interleaved into the user message.

        Returns:
            A tuple ``(waypoints, raw_text)`` where ``waypoints`` is a list of
            dicts with keys ``x``, ``y``, ``theta``.
        """
        # Replace the last user message text with interleaved image content.
        request_messages = []
        for msg in messages:
            if msg.get("role") == "user":
                content = self._build_content(msg["content"], images)
                request_messages.append({"role": "user", "content": content})
            else:
                request_messages.append(msg)

        request_kwargs = dict(
            model=self.config.model,
            messages=request_messages,
            max_tokens=self.config.max_tokens,
            temperature=self.config.temperature,
            top_p=self.config.top_p,
        )
        if "api.deepseek.com" in self.config.base_url:
            # Navigation needs a short structured answer.  DeepSeek thinking is
            # enabled by default and can consume the entire token budget before
            # producing content, so disable it for this reactive control path.
            request_kwargs["response_format"] = {"type": "json_object"}
            request_kwargs["extra_body"] = {"thinking": {"type": "disabled"}}

        response = asyncio.run(self._request_completion(request_kwargs))

        raw_text = response.choices[0].message.content or ""
        waypoints = self._parse_waypoints(raw_text)
        return waypoints, raw_text

    @staticmethod
    def _parse_waypoints(text: str) -> List[Dict[str, float]]:
        """Parse waypoint JSON from model output, with several fallbacks."""
        # Preferred schema: {"waypoints": [{"x": ..., "y": ..., "theta": ...}]}.
        try:
            data = json.loads(text)
            if isinstance(data, dict) and isinstance(data.get("waypoints"), list):
                return [
                    VLMClient._normalise_waypoint(w)
                    for w in data["waypoints"]
                ]
            if isinstance(data, list):
                return [VLMClient._normalise_waypoint(w) for w in data]
        except json.JSONDecodeError:
            pass

        # Try to find a JSON array anywhere in the text.
        match = re.search(r"\[[\s\S]*?\]", text)
        if match:
            try:
                data = json.loads(match.group(0))
                if isinstance(data, list):
                    return [VLMClient._normalise_waypoint(w) for w in data]
            except json.JSONDecodeError:
                pass

        # Fallback: look for individual {"x":..., "y":..., "theta":...} objects.
        objects = re.findall(r"\{[^}]*\"x\"[^}]*\}", text)
        if objects:
            waypoints = []
            for obj in objects:
                try:
                    waypoints.append(VLMClient._normalise_waypoint(json.loads(obj)))
                except (json.JSONDecodeError, ValueError):
                    continue
            if waypoints:
                return waypoints

        raise ValueError(f"Could not parse waypoints from VLM output: {text!r}")

    @staticmethod
    def _normalise_waypoint(w: Dict[str, Any]) -> Dict[str, float]:
        """Ensure a waypoint has float x, y, theta keys."""
        keys = [("x", 0.0), ("y", 0.0), ("theta", 0.0)]
        out = {}
        for key, default in keys:
            try:
                out[key] = float(w.get(key, default))
            except (TypeError, ValueError):
                out[key] = default
        return out

    def extract_hidden_state(self, *args, **kwargs) -> Any:
        """Stub for future hidden-state extraction.

        The report's action head consumes the LLM final hidden state. Standard
        OpenAI-compatible endpoints do not expose this. If your vLLM build supports
        it, implement this method to retrieve hidden states and feed them to
        ``ActionHead``.
        """
        raise NotImplementedError(
            "extract_hidden_state is not available via OpenAI-compatible API. "
            "Use predict_waypoints() which parses waypoints from generated text."
        )
