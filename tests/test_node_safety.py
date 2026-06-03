"""Tests for safety event queue."""
import time

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


def test_watchdog_deadman_engages_estop_through_real_arm_and_tick():
    """Drive the real arm→timeout→tick path (not the private callback)."""
    n = MoveItArmNode(
        robot_id="ur5e-test",
        moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(),
        heartbeat_timeout_ms=20,
    )
    n.install_common_verbs()
    n.install_motion_verbs()
    # Commanding motion arms the watchdog.
    out = n.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0] * 6, "control_source": "c"},
    )
    assert out["ok"] is True
    # No heartbeats arrive; after the window the loop's tick fires the deadman.
    time.sleep(0.05)
    n._watchdog.tick()
    assert n.is_estopped is True
    assert n.estop_reason == "heartbeat_timeout"
    assert any(e["kind"] == "heartbeat_timeout" for e in n.drain_safety_events())


def test_heartbeat_keeps_watchdog_from_firing():
    n = MoveItArmNode(
        robot_id="ur5e-test",
        moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(),
        heartbeat_timeout_ms=50,
    )
    n.install_common_verbs()
    n.install_motion_verbs()
    n.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0] * 6, "control_source": "c"},
    )
    # Heartbeats keep arriving within the window → no deadman.
    for _ in range(5):
        time.sleep(0.01)
        n.dispatch("robot.heartbeat", {})
        n._watchdog.tick()
    assert n.is_estopped is False
