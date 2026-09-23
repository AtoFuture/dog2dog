import json

import pytest

from g3_integration_eval.mission_brain import (
    BrainAction,
    DeepSeekBrainProvider,
    DeepSeekConfig,
    FrontierCandidate,
    _json_from_model_text,
    deterministic_fallback,
    validate_decision,
)


def candidate(candidate_id="f1", gain=10.0, cost=2.0, failures=0):
    return FrontierCandidate(candidate_id, 1.0, 2.0, gain, cost, failures)


def valid_raw(**overrides):
    value = {
        "action": "SELECT_FRONTIER",
        "candidate_id": "f1",
        "evidence_ids": [],
        "reason": "best information gain",
        "confidence": 0.8,
    }
    value.update(overrides)
    return value


def test_validate_frontier_decision():
    result = validate_decision(valid_raw(), [candidate()], [], provider="test")
    assert result.action is BrainAction.SELECT_FRONTIER
    assert result.candidate_id == "f1"
    assert not result.fallback


def test_rejects_invented_frontier():
    with pytest.raises(ValueError, match="current candidate"):
        validate_decision(
            valid_raw(candidate_id="invented"), [candidate()], [], provider="test"
        )


@pytest.mark.parametrize("action", ["COMPLETE", "VERIFY_DETECTION"])
def test_evidence_actions_require_real_evidence(action):
    raw = valid_raw(
        action=action,
        candidate_id=None,
        evidence_ids=[42],
    )
    result = validate_decision(raw, [candidate()], [42], provider="test")
    assert result.evidence_ids == (42,)


def test_rejects_unavailable_evidence():
    with pytest.raises(ValueError, match="unavailable"):
        validate_decision(
            valid_raw(
                action="COMPLETE", candidate_id=None, evidence_ids=[99]
            ),
            [candidate()],
            [42],
            provider="test",
        )


def test_rejects_completion_without_evidence():
    with pytest.raises(ValueError, match="requires detection evidence"):
        validate_decision(
            valid_raw(action="COMPLETE", candidate_id=None),
            [candidate()],
            [],
            provider="test",
        )


def test_non_frontier_action_cannot_smuggle_candidate():
    with pytest.raises(ValueError, match="must not include"):
        validate_decision(
            valid_raw(action="RETURN"), [candidate()], [], provider="test"
        )


def test_rejects_out_of_range_confidence():
    with pytest.raises(ValueError, match="between 0 and 1"):
        validate_decision(valid_raw(confidence=1.1), [candidate()], [], provider="test")


def test_markdown_json_is_tolerated_but_still_parsed():
    raw = _json_from_model_text("```json\n" + json.dumps(valid_raw()) + "\n```")
    assert raw["candidate_id"] == "f1"


def test_fallback_uses_deterministic_score():
    result = deterministic_fallback(
        {"mission_state": "EXPLORE", "localization_healthy": True},
        [candidate("high-fail", 20, 1, 10), candidate("safe", 10, 2, 0)],
        reason="provider timeout",
    )
    assert result.action is BrainAction.SELECT_FRONTIER
    assert result.candidate_id == "safe"
    assert result.fallback


@pytest.mark.parametrize(
    "context",
    [
        {"mission_state": "RETURN", "localization_healthy": True},
        {"mission_state": "EXPLORE", "localization_healthy": False},
    ],
)
def test_fallback_waits_when_exploration_is_unsafe(context):
    result = deterministic_fallback(context, [candidate()], reason="unsafe")
    assert result.action is BrainAction.WAIT
    assert result.candidate_id is None


def test_candidate_rejects_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        FrontierCandidate.from_mapping(
            {
                "id": "f1",
                "x": float("nan"),
                "y": 0,
                "information_gain": 2,
                "path_cost": 1,
            }
        )


def test_config_requires_key(monkeypatch):
    monkeypatch.delenv("MISSION_BRAIN_API_KEY", raising=False)
    with pytest.raises(ValueError, match="API_KEY"):
        DeepSeekConfig.from_env()


def test_config_defaults_to_official_flash(monkeypatch):
    monkeypatch.setenv("MISSION_BRAIN_API_KEY", "test-only")
    config = DeepSeekConfig.from_env()
    assert config.base_url == "https://api.deepseek.com"
    assert config.model == "deepseek-flash"


def test_provider_uses_official_endpoint_and_validates_response(monkeypatch):
    captured = {}

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            content = json.dumps(valid_raw())
            return json.dumps(
                {"choices": [{"message": {"content": content}}]}
            ).encode()

    def fake_urlopen(request, timeout):
        captured["url"] = request.full_url
        captured["authorization"] = request.get_header("Authorization")
        captured["timeout"] = timeout
        captured["body"] = json.loads(request.data)
        return FakeResponse()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)
    provider = DeepSeekBrainProvider(
        DeepSeekConfig("secret-for-test", timeout_s=3.0)
    )
    result = provider.decide(
        {"mission_state": "EXPLORE", "localization_healthy": True},
        [candidate()],
        [],
    )

    assert result.candidate_id == "f1"
    assert captured["url"] == "https://api.deepseek.com/chat/completions"
    assert captured["authorization"] == "Bearer secret-for-test"
    assert captured["timeout"] == 3.0
    assert captured["body"]["model"] == "deepseek-flash"
    assert captured["body"]["response_format"] == {"type": "json_object"}
    assert captured["body"]["thinking"] == {"type": "disabled"}
