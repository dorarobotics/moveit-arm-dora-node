"""Tests for safety event queue."""
from moveit_arm_node.gripper.noop import NoopGripper
from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def _node():
    n = MoveItArmNode(
        robot_id="ur5e-test",
        moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(),
    )
    n.install_common_verbs()
    return n


def test_estop_emits_safety_event():
    n = _node()
    n.dispatch("robot.estop", {"reason": "test"})
    events = n.drain_safety_events()
    assert len(events) == 1
    assert events[0]["kind"] == "estop"
    assert events[0]["reason"] == "test"


def test_drain_clears_queue():
    n = _node()
    n.dispatch("robot.estop", {"reason": "x"})
    _ = n.drain_safety_events()
    assert n.drain_safety_events() == []


def test_heartbeat_timeout_emits_safety_event():
    n = MoveItArmNode(
        robot_id="ur5e-test",
        moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(),
        heartbeat_timeout_ms=10,
    )
    n.install_common_verbs()
    n._on_heartbeat_timeout(0.0)
    events = n.drain_safety_events()
    assert len(events) == 1
    assert events[0]["kind"] == "heartbeat_timeout"
