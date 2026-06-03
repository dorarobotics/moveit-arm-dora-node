from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def _node():
    bridge = FakeMoveItBridge()
    n = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=bridge)
    n.install_common_verbs()
    n.install_motion_verbs()
    return n, bridge


def test_plan_returns_trajectory_payload():
    n, _ = _node()
    out = n.dispatch(
        "vendor.moveit.arm.plan",
        {"target": {"position": [0.4, 0, 0.3], "orientation": [0, 0, 0, 1]}},
    )
    assert out["ok"] is True
    assert "trajectory" in out["data"]


def test_execute_runs_trajectory():
    n, bridge = _node()
    plan = n.dispatch(
        "vendor.moveit.arm.plan",
        {"target": {"position": [0.4, 0, 0.3], "orientation": [0, 0, 0, 1]}},
    )
    traj = plan["data"]["trajectory"]
    out = n.dispatch(
        "vendor.moveit.arm.execute",
        {"trajectory": traj, "control_source": "test"},
    )
    assert out["ok"] is True
    assert any(c[0] == "execute" for c in bridge.calls)


def test_execute_rejects_missing_trajectory():
    n, _ = _node()
    out = n.dispatch(
        "vendor.moveit.arm.execute",
        {"control_source": "test"},
    )
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"
