from moveit_arm_node.node import MoveItArmNode
from tests.fakes import FakeMoveItBridge


def _node_with_fake() -> tuple[MoveItArmNode, FakeMoveItBridge]:
    bridge = FakeMoveItBridge()
    node = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=bridge)
    node.install_common_verbs()
    node.install_motion_verbs()
    return node, bridge


def test_move_to_joint_state_defers_and_fires_async():
    node, bridge = _node_with_fake()
    out = node.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0, -1.0, 0.0, -1.0, 0.0, 0.0], "control_source": "test"},
    )
    assert out["code"] == "DEFERRED"
    assert ("start_move_to_joint_state", ([0.0, -1.0, 0.0, -1.0, 0.0, 0.0],), {}) in bridge.calls


def test_move_to_joint_state_rejects_wrong_arity():
    node, _ = _node_with_fake()
    out = node.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0, 0.0], "control_source": "test"},
    )
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"


def test_move_to_joint_state_blocked_by_estop():
    node, _ = _node_with_fake()
    node.dispatch("robot.estop", {"reason": "test"})
    out = node.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0] * 6, "control_source": "test"},
    )
    assert out["ok"] is False
    assert out["code"] == "VENDOR_ERROR"
    assert "estop" in out["msg"].lower()


def test_move_to_joint_state_immediate_vendor_error_when_start_raises():
    bridge = FakeMoveItBridge(fail_next="VENDOR_ERROR")
    node = MoveItArmNode(robot_id="ur5e-test", moveit_bridge=bridge)
    node.install_common_verbs()
    node.install_motion_verbs()
    out = node.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0] * 6, "control_source": "test"},
    )
    assert out["ok"] is False
    assert out["code"] == "VENDOR_ERROR"


def test_move_to_joint_state_acquires_motion_lock():
    node, _ = _node_with_fake()
    node._guard.acquire("other")
    out = node.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0] * 6, "control_source": "me"},
    )
    assert out["ok"] is False
    assert out["code"] == "CONTROLLER_BUSY"


def test_move_to_pose_defers_and_fires_async():
    node, bridge = _node_with_fake()
    pose = {"position": [0.4, 0.0, 0.3], "orientation": [0, 0, 0, 1]}
    out = node.dispatch(
        "vendor.moveit.arm.move_to_pose",
        {"pose": pose, "control_source": "test"},
    )
    assert out["code"] == "DEFERRED"
    assert ("start_move_to_pose", (pose,), {}) in bridge.calls


def test_move_to_pose_requires_position_and_orientation():
    node, _ = _node_with_fake()
    out = node.dispatch(
        "vendor.moveit.arm.move_to_pose",
        {"pose": {"position": [0.4, 0.0, 0.3]}, "control_source": "test"},
    )
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"


def test_move_to_pose_blocked_by_estop():
    node, _ = _node_with_fake()
    node.dispatch("robot.estop", {"reason": "test"})
    out = node.dispatch(
        "vendor.moveit.arm.move_to_pose",
        {
            "pose": {"position": [0.4, 0, 0.3], "orientation": [0, 0, 0, 1]},
            "control_source": "test",
        },
    )
    assert out["ok"] is False
    assert out["code"] == "VENDOR_ERROR"


def test_move_to_named_defers_and_fires_async():
    node, bridge = _node_with_fake()
    out = node.dispatch(
        "vendor.moveit.arm.move_to_named",
        {"name": "home", "control_source": "test"},
    )
    assert out["code"] == "DEFERRED"
    assert ("start_move_to_named", ("home",), {}) in bridge.calls


def test_move_to_named_rejects_empty_name():
    node, _ = _node_with_fake()
    out = node.dispatch(
        "vendor.moveit.arm.move_to_named",
        {"name": "", "control_source": "test"},
    )
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"


def test_begin_motion_arms_watchdog_and_acquires_lock():
    node, _ = _node_with_fake()
    node.begin_motion("test")
    assert node._guard.holder == "test"
    assert node._watchdog._armed is True


def test_end_motion_releases_lock_and_disarms_watchdog():
    node, _ = _node_with_fake()
    node.begin_motion("test")
    node.end_motion()
    assert node._guard.holder is None
    assert node._watchdog._armed is False


def test_motion_status_delegates_to_bridge():
    node, bridge = _node_with_fake()
    bridge.status = ("succeeded", "")
    assert node.motion_status() == ("succeeded", "")


def test_plan_verb_unsupported_in_sim():
    node, _ = _node_with_fake()
    out = node.dispatch("vendor.moveit.arm.plan", {"target": {"joints": [0.0] * 6}})
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"


def test_execute_verb_unsupported_in_sim():
    node, _ = _node_with_fake()
    out = node.dispatch("vendor.moveit.arm.execute", {"trajectory": {}})
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"
