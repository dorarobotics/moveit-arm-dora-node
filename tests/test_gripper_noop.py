from moveit_arm_node.gripper import Gripper
from moveit_arm_node.gripper.noop import NoopGripper


def test_noop_gripper_implements_protocol():
    g: Gripper = NoopGripper()
    g.set(0.5)
    g.open()
    g.close()
    assert g.width() == 0.0


def test_noop_records_last_width():
    g = NoopGripper()
    g.set(0.085)
    assert g.last_set == 0.085
