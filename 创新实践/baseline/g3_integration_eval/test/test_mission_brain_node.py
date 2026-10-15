"""A hung API process must be stopped on wall-time deadline or mission return."""
import json
import multiprocessing
import time
from types import SimpleNamespace

import pytest
pytest.importorskip('rclpy')
from g3_integration_eval.mission_brain_node import MissionBrainNode


def fixture_node():
    node = MissionBrainNode.__new__(MissionBrainNode)
    ctx = multiprocessing.get_context('spawn')
    receiver, sender = ctx.Pipe(duplex=False)
    process = ctx.Process(target=time.sleep, args=(30,), daemon=True)
    process.start()
    sender.close()
    node._active = dict(id='trial', process=process, receiver=receiver,
                        deadline=time.monotonic()-1)
    node._mission = 'exploring'
    node.published = []
    node._publisher = SimpleNamespace(publish=lambda msg: node.published.append(json.loads(msg.data)))
    return node, process


def test_wall_deadline_kills_stalled_network_worker():
    node, process = fixture_node()
    started = time.monotonic()
    try:
        node._poll_result()
        assert time.monotonic()-started < 2
        assert not process.is_alive() and node._active is None
        assert node.published == [{'error': 'brain wall-time deadline exceeded', 'request_id': 'trial'}]
    finally:
        node._stop_worker()


def test_return_kills_pending_worker_without_publishing_a_decision():
    node, process = fixture_node()
    try:
        node._on_mission(SimpleNamespace(data='returning'))
        node._poll_result()
        assert not process.is_alive() and not node.published
    finally:
        node._stop_worker()
