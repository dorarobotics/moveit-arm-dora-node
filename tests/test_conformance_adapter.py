"""Test the in-process conformance adapter."""
from __future__ import annotations

import pytest

from moveit_arm_node.conformance_adapter import InProcessAdapter
from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def test_adapter_routes_verb():
    node = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=FakeMoveItBridge())
    node.install_common_verbs()
    adapter = InProcessAdapter(node)

    env = {"id": "x", "verb": "robot.heartbeat", "args": {}}
    resp = adapter.send(env)
    assert resp["ok"] is True


def test_adapter_returns_invalid_envelope_on_bad_input():
    node = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=FakeMoveItBridge())
    node.install_common_verbs()
    adapter = InProcessAdapter(node)

    resp = adapter.send({"id": "x"})  # missing verb
    assert resp["ok"] is False
    assert resp["code"] == "INVALID_PARAMS"
