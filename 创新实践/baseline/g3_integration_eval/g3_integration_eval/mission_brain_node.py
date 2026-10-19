"""Correlated frontier ranking with a killable, wall-time-bounded API worker.

This node publishes decisions only. G2 owns Nav2 and rechecks delayed targets.
"""
from __future__ import annotations

import json
import math
import multiprocessing
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from g3_integration_eval.mission_brain import (
    DeepSeekBrainProvider, DeepSeekConfig, FrontierCandidate, decide_with_fallback,
)


def _worker(connection, config, context, candidates, evidence_ids):
    try:
        result = decide_with_fallback(DeepSeekBrainProvider(config), context, candidates, evidence_ids)
        connection.send(result.to_dict())
    except Exception:
        # Never include credentials, headers or arbitrary response bodies in errors.
        connection.send({'error': 'brain worker failed'})
    finally:
        connection.close()


class MissionBrainNode(Node):
    def __init__(self):
        super().__init__('mission_brain')
        self.declare_parameter('context_topic', '/mission/context')
        self.declare_parameter('candidates_topic', '/exploration/candidates')
        self.declare_parameter('decision_topic', '/brain/decision')
        self.declare_parameter('mission_state_topic', '/mission_state')
        self._config = DeepSeekConfig.from_env()
        self._context = None
        self._active = None
        self._process_context = multiprocessing.get_context('spawn')
        self._publisher = self.create_publisher(String, self.get_parameter('decision_topic').value, 10)
        self.create_subscription(String, self.get_parameter('context_topic').value, self._on_context, 10)
        self.create_subscription(String, self.get_parameter('candidates_topic').value, self._on_candidates, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mission = None
        self.create_subscription(
            String, self.get_parameter('mission_state_topic').value, self._on_mission, qos
        )
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self._poll_result, clock=self._steady)
        self.get_logger().info(f'Mission Brain ready: {self._config.model}; bounded candidate ranking')

    def _on_mission(self, message):
        self._mission = message.data
        if self._mission != 'exploring':
            self._stop_worker()

    def _on_context(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('context must be an object')
            self._context = value
            if value.get('mission_state') != 'EXPLORE' or value.get('localization_healthy') is not True:
                self._stop_worker()
        except (ValueError, TypeError):
            self.get_logger().warning('Invalid mission context')

    def _on_candidates(self, message):
        if self._active is not None or self._mission != 'exploring':
            return
        try:
            payload = json.loads(message.data)
            request_id = payload['request_id']
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
                raise ValueError('invalid request ID')
            # Inline snapshot avoids racing two independent topics.
            context = payload.get('context', self._context)
            if not isinstance(context, dict) or context.get('mission_state') != 'EXPLORE' or context.get('localization_healthy') is not True:
                raise ValueError('unsafe context')
            raw = payload['candidates']
            if not isinstance(raw, list) or not 1 <= len(raw) <= 16:
                raise ValueError('invalid candidate count')
            candidates = tuple(FrontierCandidate.from_mapping(item) for item in raw)
            if len({item.candidate_id for item in candidates}) != len(candidates):
                raise ValueError('duplicate candidate IDs')
            # This endpoint ranks frontiers only; detection evidence uses its own collector.
            if payload.get('evidence_ids', []) != []:
                raise ValueError('frontier requests must not supply detection claims')
            budget = float(payload.get('timeout_s', self._config.timeout_s))
            if not math.isfinite(budget) or not .1 <= budget <= 60:
                raise ValueError('invalid request budget')
        except (ValueError, KeyError, TypeError, AttributeError):
            self.get_logger().warning('Invalid frontier ranking request')
            return
        receiver, sender = self._process_context.Pipe(duplex=False)
        process = self._process_context.Process(
            target=_worker, args=(sender, self._config, dict(context), candidates, ()), daemon=True)
        process.start()
        sender.close()
        self._active = dict(id=request_id, process=process, receiver=receiver,
                            deadline=time.monotonic()+min(budget, self._config.timeout_s))

    def _stop_worker(self):
        active, self._active = self._active, None
        if active is None:
            return
        process = active['process']
        if process.is_alive():
            process.terminate()
        process.join(timeout=.2)
        if process.is_alive():
            process.kill()
            process.join(timeout=.2)
        active['receiver'].close()

    def _poll_result(self):
        active = self._active
        if active is None:
            return
        result = None
        if time.monotonic() >= active['deadline']:
            result = {'error': 'brain wall-time deadline exceeded'}
        elif active['receiver'].poll():
            try:
                result = active['receiver'].recv()
            except EOFError:
                result = {'error': 'brain worker exited'}
        elif not active['process'].is_alive():
            result = {'error': 'brain worker exited'}
        if result is None:
            return
        result['request_id'] = active['id']
        self._stop_worker()
        if self._mission == 'exploring':
            self._publisher.publish(String(data=json.dumps(result, ensure_ascii=False)))

    def destroy_node(self):
        self._stop_worker()
        return super().destroy_node()


def main(args=None):
    from g3_integration_eval.node_lifecycle import run_node
    run_node(MissionBrainNode, args)


if __name__ == '__main__':
    main()
