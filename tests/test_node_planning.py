from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def _node():
    bridge = FakeMoveItBridge()
    n = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=bridge)
    n.install_common_verbs()
    n.install_motion_verbs()
    return n, bridge


def test_plan_unsupported_in_sim():
    n, _ = _node()
    out = n.dispatch(
        "vendor.moveit.arm.plan",
        {"target": {"position": [0.4, 0, 0.3], "orientation": [0, 0, 0, 1]}},
    )
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"


def test_execute_unsupported_in_sim():
    n, bridge = _node()
    out = n.dispatch(
        "vendor.moveit.arm.execute",
        {"trajectory": {"fake_trajectory": True}, "control_source": "test"},
    )
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"
    # The verb is rejected before touching the bridge.
    assert not any(c[0] == "execute" for c in bridge.calls)
