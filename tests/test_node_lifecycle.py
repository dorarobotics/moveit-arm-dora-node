"""Lifecycle invariants: clean construction, verb registration, estop reset."""
from moveit_arm_node.gripper.noop import NoopGripper
from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def test_node_starts_with_no_verbs():
    n = MoveItArmNode(robot_id="ur5e-test")
    assert n.dispatch("robot.heartbeat", {}) == {
        "ok": False, "code": "INVALID_PARAMS",
        "msg": "unknown verb: robot.heartbeat",
    }


def test_install_common_verbs_idempotent_double_call_raises():
    n = MoveItArmNode(robot_id="ur5e-test")
    n.install_common_verbs()
    try:
        n.install_common_verbs()
    except ValueError as e:
        assert "already registered" in str(e)
    else:
        raise AssertionError("expected ValueError on duplicate registration")


def test_full_install_sequence():
    n = MoveItArmNode(
        robot_id="ur5e-test", moveit_bridge=FakeMoveItBridge(), gripper=NoopGripper()
    )
    n.install_common_verbs()
    n.install_motion_verbs()
    n.install_gripper_verbs()
    n.install_scene_verbs()
    caps = n.dispatch("robot.get_capabilities", {})
    verbs = {cmd["verb"] for cmd in caps["data"]["commands"]}
    expected_subset = {
        "robot.heartbeat", "robot.estop", "robot.release_control",
        "robot.get_capabilities",
        "vendor.moveit.arm.move_to_pose",
        "vendor.moveit.arm.move_to_joint_state",
        "vendor.moveit.arm.move_to_named",
        "vendor.moveit.arm.plan", "vendor.moveit.arm.execute",
        "vendor.moveit.arm.gripper.set",
        "vendor.moveit.arm.gripper.open",
        "vendor.moveit.arm.gripper.close",
        "vendor.moveit.arm.scene.add_collision",
        "vendor.moveit.arm.scene.clear",
    }
    assert expected_subset.issubset(set(verbs))


def test_estop_remains_after_more_calls():
    n = MoveItArmNode(
        robot_id="ur5e-test", moveit_bridge=FakeMoveItBridge(), gripper=NoopGripper()
    )
    n.install_common_verbs()
    n.install_motion_verbs()
    n.dispatch("robot.estop", {"reason": "test"})
    blocked = n.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0] * 6, "control_source": "x"},
    )
    assert blocked["ok"] is False
    # Spec §8.1: estop is a latching state. There's no `clear_estop` verb in v1 —
    # the operator must restart the node. This test documents that intent.
