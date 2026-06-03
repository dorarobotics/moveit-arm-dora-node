"""Test Mode C client library — drive the arm without dora-rs."""
from moveit_arm_node.client import MoveItArmClient, ArmConfig
from moveit_arm_node.client.types import Pose
from moveit_arm_node.client.state import RobotState
from tests.fakes import FakeMoveItBridge


def test_robot_state_from_snapshot():
    snap = {
        "robot_id": "ur5e-test",
        "joint_positions": [0.0, -1.0, 0.0, -1.0, 0.0, 0.0],
        "gripper_width": 0.04,
        "estopped": False,
        "estop_reason": None,
        "controller_holder": None,
    }
    s = RobotState.from_snapshot(snap)
    assert s.robot_id == "ur5e-test"
    assert s.joint_positions == (0.0, -1.0, 0.0, -1.0, 0.0, 0.0)
    assert s.gripper_width == 0.04
    assert s.estopped is False


def test_client_constructs_from_config():
    cfg = ArmConfig(robot_config_module="ur5e")
    client = MoveItArmClient(config=cfg, _bridge=FakeMoveItBridge())
    assert client.config.robot_config_module == "ur5e"


def test_client_move_to_joint_state_calls_bridge():
    bridge = FakeMoveItBridge()
    client = MoveItArmClient(config=ArmConfig(robot_config_module="ur5e"), _bridge=bridge)
    client.move_to_joint_state([0.0, -1.0, 0.0, -1.0, 0.0, 0.0])
    assert bridge.calls[0][0] == "move_to_joint_state"


def test_client_move_to_pose_takes_typed_pose():
    bridge = FakeMoveItBridge()
    client = MoveItArmClient(config=ArmConfig(robot_config_module="ur5e"), _bridge=bridge)
    p = Pose(position=(0.4, 0.0, 0.3), orientation=(0, 0, 0, 1))
    client.move_to_pose(p)
    assert bridge.calls[0][0] == "move_to_pose"


def test_client_module_loads_without_dora():
    """Smoke test: importing client must not require dora-rs."""
    import importlib
    importlib.import_module("moveit_arm_node.client")
