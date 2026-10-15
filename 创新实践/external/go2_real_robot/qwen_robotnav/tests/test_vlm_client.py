"""Tests for VLM client waypoint parsing."""

from qwen_robotnav.vlm_client import VLMClient
from qwen_robotnav.config import VLMConfig

import asyncio
import sys
from types import SimpleNamespace

import pytest


def test_parse_waypoints_json_array():
    text = '[{"x": 0.1, "y": 0.2, "theta": 0.3}, {"x": 0.4, "y": 0.5, "theta": 0.6}]'
    waypoints = VLMClient._parse_waypoints(text)
    assert len(waypoints) == 2
    assert waypoints[0]["x"] == 0.1


def test_parse_waypoints_markdown():
    text = "Here are the waypoints:\n```json\n[{\"x\": 1.0, \"y\": 0.0, \"theta\": 0.1}]\n```"
    waypoints = VLMClient._parse_waypoints(text)
    assert len(waypoints) == 1
    assert waypoints[0]["x"] == 1.0


def test_normalise_waypoint():
    w = VLMClient._normalise_waypoint({"x": "0.5", "y": None, "theta": 1})
    assert w["x"] == 0.5
    assert w["y"] == 0.0
    assert w["theta"] == 1.0


def test_deepseek_request_has_no_retries_and_keeps_image_and_json_options(monkeypatch):
    from PIL import Image

    captured = {}

    class FakeClient:
        def __init__(self, **kwargs):
            captured['client'] = kwargs
            self.chat = SimpleNamespace(completions=self)

        async def __aenter__(self):
            return self

        async def __aexit__(self, *args):
            captured['closed'] = True

        async def create(self, **kwargs):
            captured['request'] = kwargs
            return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(
                content='{"waypoints":[{"x":0,"y":0,"theta":0}]}'
            ))])

    monkeypatch.setitem(sys.modules, 'openai', SimpleNamespace(AsyncOpenAI=FakeClient))
    client = VLMClient(VLMConfig(api_key='test-only', timeout=0.5))
    waypoints, raw = client.predict_waypoints(
        [{'role': 'user', 'content': 'Return JSON for <image>'}],
        [Image.new('RGB', (2, 2))],
    )
    assert waypoints == [{'x': 0.0, 'y': 0.0, 'theta': 0.0}]
    assert captured['client']['max_retries'] == 0
    assert captured['closed']
    assert captured['request']['model'] == 'deepseek-flash'
    assert captured['request']['response_format'] == {'type': 'json_object'}
    assert captured['request']['extra_body'] == {'thinking': {'type': 'disabled'}}
    assert captured['request']['messages'][0]['content'][1]['image_url']['url'].startswith('data:image/png;base64,')


def test_total_deadline_cancels_request_despite_continuous_keepalive_work(monkeypatch):
    state = {'ticks': 0, 'canceled': False, 'closed': False}

    class KeepAliveClient:
        def __init__(self, **kwargs):
            self.chat = SimpleNamespace(completions=self)

        async def __aenter__(self):
            return self

        async def __aexit__(self, *args):
            state['closed'] = True

        async def create(self, **kwargs):
            try:
                while True:
                    state['ticks'] += 1
                    await asyncio.sleep(0.001)
            finally:
                state['canceled'] = True

    monkeypatch.setitem(sys.modules, 'openai', SimpleNamespace(AsyncOpenAI=KeepAliveClient))
    client = VLMClient(VLMConfig(timeout=0.03))
    with pytest.raises(TimeoutError, match='total deadline'):
        client.predict_waypoints([{'role': 'user', 'content': 'JSON'}], [])
    assert state['ticks'] > 1
    assert state['canceled']
    assert state['closed']


@pytest.mark.parametrize('timeout', [0, -1, float('nan'), float('inf')])
def test_rejects_unbounded_or_invalid_timeout(timeout):
    with pytest.raises(ValueError, match='finite and positive'):
        VLMClient(VLMConfig(timeout=timeout))
