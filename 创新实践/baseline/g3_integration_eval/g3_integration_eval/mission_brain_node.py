"""ROS 2 wrapper for the guarded DeepSeek mission brain.

Input topics use JSON in std_msgs/String until the shared mission interfaces are
frozen.  The node publishes decisions only; it never publishes motion commands.
"""

from __future__ import annotations

import json
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Optional, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from g3_integration_eval.mission_brain import (
    DeepSeekBrainProvider,
    DeepSeekConfig,
    FrontierCandidate,
    decide_with_fallback,
)


class MissionBrainNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_brain")
        self.declare_parameter("context_topic", "/mission/context")
        self.declare_parameter("candidates_topic", "/exploration/candidates")
        self.declare_parameter("decision_topic", "/brain/decision")

        self._provider = DeepSeekBrainProvider(DeepSeekConfig.from_env())
        self._context: dict[str, Any] | None = None
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._future: Future | None = None
        self._generation = 0

        self._publisher = self.create_publisher(
            String, str(self.get_parameter("decision_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("context_topic").value),
            self._on_context,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("candidates_topic").value),
            self._on_candidates,
            10,
        )
        self.create_timer(0.1, self._poll_result)
        self.get_logger().info(
            "Mission Brain ready: provider=deepseek, model=deepseek-flash; "
            "publishes guarded high-level decisions only"
        )

    def _on_context(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("context must be a JSON object")
            self._context = value
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().warning(f"invalid mission context: {exc}")

    def _on_candidates(self, message: String) -> None:
        if self._context is None:
            self.get_logger().warning("candidate set ignored: no mission context yet")
            return
        if self._future is not None and not self._future.done():
            self.get_logger().warning("candidate set ignored: brain request already active")
            return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("candidate payload must be a JSON object")
            candidates_raw = payload.get("candidates", [])
            if not isinstance(candidates_raw, list):
                raise ValueError("candidates must be a list")
            candidates = tuple(
                FrontierCandidate.from_mapping(item) for item in candidates_raw
            )
            evidence_ids = tuple(int(item) for item in payload.get("evidence_ids", []))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid candidate set: {exc}")
            return

        self._generation += 1
        generation = self._generation
        context = dict(self._context)
        self._future = self._executor.submit(
            self._decide, generation, context, candidates, evidence_ids
        )

    def _decide(self, generation, context, candidates, evidence_ids):
        decision = decide_with_fallback(
            self._provider, context, candidates, evidence_ids
        )
        return generation, decision

    def _poll_result(self) -> None:
        if self._future is None or not self._future.done():
            return
        future = self._future
        self._future = None
        try:
            generation, decision = future.result()
        except Exception as exc:  # defensive boundary around worker failures
            self.get_logger().error(f"brain worker failed without a decision: {exc}")
            return
        if generation != self._generation:
            self.get_logger().warning("stale brain decision discarded")
            return
        message = String()
        message.data = json.dumps(decision.to_dict(), ensure_ascii=False)
        self._publisher.publish(message)
        self.get_logger().info(
            f"brain decision: action={decision.action.value}, "
            f"candidate={decision.candidate_id}, provider={decision.provider}, "
            f"fallback={decision.fallback}"
        )

    def destroy_node(self):
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    try:
        node = MissionBrainNode()
    except ValueError as exc:
        rclpy.shutdown()
        raise SystemExit(f"Mission Brain configuration error: {exc}") from exc
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
