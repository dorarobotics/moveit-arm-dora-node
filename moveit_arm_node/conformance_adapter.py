"""Conformance-suite adapter for MoveItArmNode.

Bridges the verb-dispatch surface to the SPEC-V1 conformance suite's Transport
Protocol. Inputs are SPEC-V1 envelopes; outputs are SPEC-V1 cmd_response dicts.

This is test/conformance-only — NOT registered in pyproject.toml as a runtime
dependency.
"""
from __future__ import annotations

from typing import Any

from moveit_arm_node._envelope import InvalidEnvelope, build_cmd_response, parse_cmd_request
from moveit_arm_node.node import MoveItArmNode


class InProcessAdapter:
    """Send envelopes to a MoveItArmNode without dora in the loop."""

    def __init__(self, node: MoveItArmNode) -> None:
        self._node = node

    def send(self, envelope: dict[str, Any]) -> dict[str, Any]:
        try:
            req = parse_cmd_request(envelope)
        except InvalidEnvelope as e:
            return build_cmd_response(
                request_id=str(envelope.get("id", "")),
                ok=False, code="INVALID_PARAMS", msg=str(e),
            )
        out = self._node.dispatch(req.verb, req.args)
        return build_cmd_response(
            request_id=req.id,
            ok=bool(out.get("ok")),
            code=str(out.get("code", "0")),
            data=out.get("data"),
            msg=out.get("msg"),
        )
