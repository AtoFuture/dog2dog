"""Safe, ROS-independent mission-brain provider and decision guardrails.

The language model may rank only caller-supplied frontier IDs.  It never
creates coordinates or sends robot motion commands.  Invalid or unavailable
model output falls back to a deterministic frontier score.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from enum import Enum
from math import isfinite
from typing import Any, Mapping, Sequence


class BrainAction(str, Enum):
    SELECT_FRONTIER = "SELECT_FRONTIER"
    VERIFY_DETECTION = "VERIFY_DETECTION"
    CONTINUE = "CONTINUE"
    COMPLETE = "COMPLETE"
    RETURN = "RETURN"
    WAIT = "WAIT"


@dataclass(frozen=True)
class FrontierCandidate:
    candidate_id: str
    x: float
    y: float
    information_gain: float
    path_cost: float
    failure_count: int = 0

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "FrontierCandidate":
        candidate = cls(
            candidate_id=str(value["id"]).strip(),
            x=float(value["x"]),
            y=float(value["y"]),
            information_gain=float(value["information_gain"]),
            path_cost=float(value["path_cost"]),
            failure_count=int(value.get("failure_count", 0)),
        )
        if not candidate.candidate_id:
            raise ValueError("frontier candidate id must not be empty")
        numeric = (
            candidate.x,
            candidate.y,
            candidate.information_gain,
            candidate.path_cost,
        )
        if not all(isfinite(item) for item in numeric):
            raise ValueError("frontier candidate values must be finite")
        if candidate.path_cost < 0.0 or candidate.failure_count < 0:
            raise ValueError("path_cost and failure_count must be non-negative")
        return candidate

    def prompt_dict(self) -> dict[str, Any]:
        return {
            "id": self.candidate_id,
            "x": self.x,
            "y": self.y,
            "information_gain": self.information_gain,
            "path_cost": self.path_cost,
            "failure_count": self.failure_count,
        }


@dataclass(frozen=True)
class BrainDecision:
    action: BrainAction
    candidate_id: str | None
    evidence_ids: tuple[int, ...]
    reason: str
    confidence: float
    provider: str
    fallback: bool = False

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["action"] = self.action.value
        result["evidence_ids"] = list(self.evidence_ids)
        return result


@dataclass(frozen=True)
class DeepSeekConfig:
    api_key: str
    base_url: str = "https://api.deepseek.com"
    model: str = "deepseek-flash"
    timeout_s: float = 15.0

    @classmethod
    def from_env(cls) -> "DeepSeekConfig":
        api_key = os.environ.get("MISSION_BRAIN_API_KEY", "").strip()
        if not api_key:
            raise ValueError("MISSION_BRAIN_API_KEY is not set")
        base_url = os.environ.get(
            "MISSION_BRAIN_BASE_URL", "https://api.deepseek.com"
        ).strip()
        model = os.environ.get("MISSION_BRAIN_MODEL", "deepseek-flash").strip()
        timeout_s = float(os.environ.get("MISSION_BRAIN_TIMEOUT_S", "15"))
        if not base_url.startswith("https://"):
            raise ValueError("MISSION_BRAIN_BASE_URL must use https")
        if not model:
            raise ValueError("MISSION_BRAIN_MODEL must not be empty")
        if not isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("MISSION_BRAIN_TIMEOUT_S must be positive and finite")
        return cls(api_key, base_url.rstrip("/"), model, timeout_s)


class BrainProviderError(RuntimeError):
    """Raised when the remote provider fails or returns an invalid envelope."""


_SYSTEM_PROMPT = """You are a high-level mission planner for a quadruped robot.
Return one JSON object only. Never invent coordinates, candidate IDs, detection
IDs, or observations. Choose candidate_id only from the supplied candidates.
Allowed actions: SELECT_FRONTIER, VERIFY_DETECTION, CONTINUE, COMPLETE, RETURN,
WAIT. COMPLETE and VERIFY_DETECTION must cite supplied detection evidence IDs.
Safety state and low-battery rules override exploration. Do not output cmd_vel,
joint commands, paths, prose outside JSON, or markdown.
Schema: {"action": string, "candidate_id": string|null,
"evidence_ids": [integer], "reason": string, "confidence": number 0..1}.
"""


def _json_from_model_text(raw: str) -> Mapping[str, Any]:
    text = raw.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines).strip()
    try:
        value = json.loads(text)
    except json.JSONDecodeError as exc:
        raise BrainProviderError("model output is not valid JSON") from exc
    if not isinstance(value, dict):
        raise BrainProviderError("model output must be a JSON object")
    return value


def validate_decision(
    raw: Mapping[str, Any],
    candidates: Sequence[FrontierCandidate],
    available_evidence_ids: Sequence[int],
    *,
    provider: str,
) -> BrainDecision:
    """Validate model output against the current candidate/evidence snapshot."""

    try:
        action = BrainAction(str(raw["action"]).strip().upper())
    except (KeyError, ValueError) as exc:
        raise ValueError("unknown or missing brain action") from exc

    candidate_raw = raw.get("candidate_id")
    candidate_id = None if candidate_raw is None else str(candidate_raw).strip()
    valid_candidate_ids = {item.candidate_id for item in candidates}
    if action is BrainAction.SELECT_FRONTIER:
        if not candidate_id or candidate_id not in valid_candidate_ids:
            raise ValueError("SELECT_FRONTIER must reference a current candidate")
    elif candidate_id is not None:
        raise ValueError(f"{action.value} must not include candidate_id")

    evidence_raw = raw.get("evidence_ids", [])
    if not isinstance(evidence_raw, list):
        raise ValueError("evidence_ids must be a list")
    try:
        evidence_ids = tuple(int(item) for item in evidence_raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("evidence_ids must contain integers") from exc
    if len(set(evidence_ids)) != len(evidence_ids):
        raise ValueError("evidence_ids must not contain duplicates")
    known_evidence = {int(item) for item in available_evidence_ids}
    if not set(evidence_ids).issubset(known_evidence):
        raise ValueError("decision cites unavailable detection evidence")
    if action in (BrainAction.COMPLETE, BrainAction.VERIFY_DETECTION):
        if not evidence_ids:
            raise ValueError(f"{action.value} requires detection evidence")
    elif evidence_ids:
        raise ValueError(f"{action.value} must not cite detection evidence")

    try:
        confidence = float(raw["confidence"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("confidence must be a number") from exc
    if not isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be finite and between 0 and 1")

    reason = str(raw.get("reason", "")).strip()
    if not reason or len(reason) > 500:
        raise ValueError("reason must contain 1 to 500 characters")

    return BrainDecision(
        action=action,
        candidate_id=candidate_id,
        evidence_ids=evidence_ids,
        reason=reason,
        confidence=confidence,
        provider=provider,
    )


class DeepSeekBrainProvider:
    """Small stdlib-only client for DeepSeek's OpenAI-compatible endpoint."""

    def __init__(self, config: DeepSeekConfig) -> None:
        self._config = config

    def decide(
        self,
        context: Mapping[str, Any],
        candidates: Sequence[FrontierCandidate],
        available_evidence_ids: Sequence[int],
    ) -> BrainDecision:
        user_payload = {
            "mission": dict(context),
            "frontier_candidates": [item.prompt_dict() for item in candidates],
            "available_detection_evidence_ids": [
                int(item) for item in available_evidence_ids
            ],
        }
        body = {
            "model": self._config.model,
            "messages": [
                {"role": "system", "content": _SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": json.dumps(
                        user_payload, ensure_ascii=False, separators=(",", ":")
                    ),
                },
            ],
            "response_format": {"type": "json_object"},
            "thinking": {"type": "disabled"},
            "temperature": 0.1,
            "max_tokens": 500,
            "stream": False,
        }
        request = urllib.request.Request(
            f"{self._config.base_url}/chat/completions",
            data=json.dumps(body).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self._config.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=self._config.timeout_s
            ) as response:
                response_body = response.read().decode("utf-8")
        except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError) as exc:
            raise BrainProviderError(f"DeepSeek request failed: {exc}") from exc

        try:
            envelope = json.loads(response_body)
            content = envelope["choices"][0]["message"]["content"]
        except (json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
            raise BrainProviderError("DeepSeek response envelope is invalid") from exc
        raw_decision = _json_from_model_text(str(content))
        return validate_decision(
            raw_decision,
            candidates,
            available_evidence_ids,
            provider=f"deepseek:{self._config.model}",
        )


def deterministic_fallback(
    context: Mapping[str, Any],
    candidates: Sequence[FrontierCandidate],
    *,
    reason: str,
) -> BrainDecision:
    """Choose a safe deterministic action after provider failure/rejection."""

    state = str(context.get("mission_state", "")).strip().upper()
    localization_healthy = context.get("localization_healthy", False) is True
    if state != "EXPLORE" or not localization_healthy:
        return BrainDecision(
            BrainAction.WAIT,
            None,
            (),
            f"safe fallback: {reason}",
            1.0,
            "deterministic",
            True,
        )
    if not candidates:
        return BrainDecision(
            BrainAction.WAIT,
            None,
            (),
            f"no frontier candidate: {reason}",
            1.0,
            "deterministic",
            True,
        )
    best = max(
        candidates,
        key=lambda item: (
            item.information_gain - item.path_cost - 2.0 * item.failure_count,
            item.candidate_id,
        ),
    )
    return BrainDecision(
        BrainAction.SELECT_FRONTIER,
        best.candidate_id,
        (),
        f"deterministic frontier fallback: {reason}",
        1.0,
        "deterministic",
        True,
    )


def decide_with_fallback(
    provider: DeepSeekBrainProvider,
    context: Mapping[str, Any],
    candidates: Sequence[FrontierCandidate],
    available_evidence_ids: Sequence[int],
) -> BrainDecision:
    """Call the provider, converting any failure into a deterministic decision."""

    if str(context.get("mission_state", "")).strip().upper() != "EXPLORE":
        return deterministic_fallback(context, candidates, reason="mission is not EXPLORE")
    if context.get("localization_healthy", False) is not True:
        return deterministic_fallback(context, candidates, reason="localization unhealthy")
    try:
        return provider.decide(context, candidates, available_evidence_ids)
    except (BrainProviderError, ValueError, TypeError) as exc:
        return deterministic_fallback(context, candidates, reason=str(exc)[:300])
