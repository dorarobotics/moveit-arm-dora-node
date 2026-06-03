"""Tests for scene manipulation verbs."""
from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def _node():
    b = FakeMoveItBridge()
    n = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=b)
    n.install_common_verbs()
    n.install_scene_verbs()
    return n, b


def test_scene_add_collision_calls_bridge():
    n, b = _node()
    obj = {
        "id": "box1",
        "shape": "box",
        "size": [0.1, 0.1, 0.1],
        "pose": {"position": [0.5, 0, 0.3], "orientation": [0, 0, 0, 1]},
    }
    out = n.dispatch("vendor.moveit.arm.scene.add_collision", {"object": obj})
    assert out["ok"] is True
    assert ("add_collision", (obj,), {}) in b.calls


def test_scene_clear_calls_bridge():
    n, b = _node()
    out = n.dispatch("vendor.moveit.arm.scene.clear", {})
    assert out["ok"] is True
    assert any(c[0] == "clear_scene" for c in b.calls)


def test_scene_add_collision_rejects_missing_object():
    n, _ = _node()
    out = n.dispatch("vendor.moveit.arm.scene.add_collision", {})
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"
