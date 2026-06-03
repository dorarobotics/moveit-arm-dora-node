"""Conformance-suite adapter for MoveItArmNode.

Drives the node through the SAME request-handling path as the live dora runtime
(`MoveItArmRuntime.handle_request`), so conformance tests exercise the real
cmd_request → cmd_response contract — just without a dora process in the loop.

This is test/conformance-only — NOT registered in pyproject.toml as a runtime
dependency.
"""
from __future__ import annotations

from typing import Any

from moveit_arm_node.node import MoveItArmNode
from moveit_arm_node.runtime import MoveItArmRuntime


class InProcessAdapter:
    """Send SPEC-V1 cmd_request envelopes to a MoveItArmNode without dora."""

    def __init__(self, node: MoveItArmNode) -> None:
        self._runtime = MoveItArmRuntime(node)

    def send(self, envelope: dict[str, Any]) -> dict[str, Any]:
        return self._runtime.handle_request(envelope)
