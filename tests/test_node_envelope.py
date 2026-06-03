"""MoveItArmNode boots and accepts a no-op cmd_request envelope."""
from moveit_arm_node.node import MoveItArmNode


def test_node_constructs_with_minimal_config():
    node = MoveItArmNode(robot_id="ur5e-test")
    assert node.robot_id == "ur5e-test"


def test_node_module_callable():
    """`python -m moveit_arm_node` must be importable (smoke test for __main__)."""
    import moveit_arm_node.__main__  # noqa: F401
