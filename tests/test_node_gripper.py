from moveit_arm_node.gripper.noop import NoopGripper
from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def _node():
    gripper = NoopGripper()
    n = MoveItArmNode(
        robot_id="ur5e-test", moveit_bridge=FakeMoveItBridge(), gripper=gripper
    )
    n.install_common_verbs()
    n.install_gripper_verbs()
    return n, gripper


def test_gripper_set_drives_width():
    n, g = _node()
    out = n.dispatch("vendor.moveit.arm.gripper.set", {"width": 0.040})
    assert out["ok"] is True
    assert g.last_set == 0.040


def test_gripper_set_clamps_negative():
    n, _ = _node()
    out = n.dispatch("vendor.moveit.arm.gripper.set", {"width": -0.1})
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"


def test_gripper_open_and_close():
    n, g = _node()
    n.dispatch("vendor.moveit.arm.gripper.open", {})
    assert g.last_set > 0.0
    n.dispatch("vendor.moveit.arm.gripper.close", {})
    assert g.last_set == 0.0


def test_gripper_set_blocked_by_estop():
    n, _ = _node()
    n.dispatch("robot.estop", {"reason": "test"})
    out = n.dispatch("vendor.moveit.arm.gripper.set", {"width": 0.04})
    assert out["ok"] is False
    assert out["code"] == "VENDOR_ERROR"
