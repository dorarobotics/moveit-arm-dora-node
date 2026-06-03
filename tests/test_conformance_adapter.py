"""Test the in-process conformance adapter against the real wire contract."""
from __future__ import annotations

from moveit_arm_node.conformance_adapter import InProcessAdapter
from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def _adapter() -> InProcessAdapter:
    node = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=FakeMoveItBridge())
    node.install_common_verbs()
    return InProcessAdapter(node)


def test_adapter_routes_verb_and_echoes_request_id():
    adapter = _adapter()
    env = {"verb": "robot.heartbeat", "params": {}, "request_id": "abc-123",
           "target": "ur5e-test"}
    resp = adapter.send(env)
    assert resp["ok"] is True
    assert resp["request_id"] == "abc-123"
    assert resp["code"] == "0"


def test_adapter_returns_invalid_envelope_on_missing_verb():
    adapter = _adapter()
    resp = adapter.send({"request_id": "x"})  # no verb
    assert resp["ok"] is False
    assert resp["code"] == "INVALID_PARAMS"
    assert resp["request_id"] == "x"


def test_adapter_unknown_verb_is_invalid_params():
    adapter = _adapter()
    resp = adapter.send({"verb": "vendor.moveit.arm.nope", "params": {}, "request_id": "y"})
    assert resp["ok"] is False
    assert resp["code"] == "INVALID_PARAMS"
