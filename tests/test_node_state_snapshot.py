"""Tests for state_snapshot method."""
from moveit_arm_node.gripper.noop import NoopGripper
from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def test_state_snapshot_has_required_fields():
    n = MoveItArmNode(
        robot_id="ur5e-test",
        moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(),
    )
    n.install_common_verbs()
    n.install_motion_verbs()
    n.install_gripper_verbs()

    snap = n.state_snapshot()
    assert snap["robot_id"] == "ur5e-test"
    assert "joint_positions" in snap
    assert len(snap["joint_positions"]) == 6
    assert "gripper_width" in snap
    assert "estopped" in snap
    assert "controller_holder" in snap


def test_state_snapshot_has_monotonic_seq():
    n = MoveItArmNode(
        robot_id="ur5e-test",
        moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(),
    )
    n.install_common_verbs()
    assert n.state_snapshot()["seq"] == 1
    assert n.state_snapshot()["seq"] == 2


def test_state_snapshot_reflects_estop():
    n = MoveItArmNode(
        robot_id="ur5e-test",
        moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(),
    )
    n.install_common_verbs()
    n.install_motion_verbs()
    n.install_gripper_verbs()
    n.dispatch("robot.estop", {"reason": "manual"})
    snap = n.state_snapshot()
    assert snap["estopped"] is True
    assert snap["estop_reason"] == "manual"
