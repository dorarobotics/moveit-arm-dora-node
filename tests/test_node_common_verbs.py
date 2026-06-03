from moveit_arm_node.node import MoveItArmNode


def test_heartbeat_verb_returns_ok_and_pets_watchdog():
    node = MoveItArmNode(robot_id="ur5e-test", heartbeat_timeout_ms=1000)
    node.install_common_verbs()
    out = node.dispatch("robot.heartbeat", {})
    assert out == {"ok": True, "code": "0"}


def test_heartbeat_when_watchdog_disabled_still_ok():
    node = MoveItArmNode(robot_id="ur5e-test", heartbeat_timeout_ms=0)
    node.install_common_verbs()
    out = node.dispatch("robot.heartbeat", {})
    assert out["ok"] is True


def test_estop_records_state_and_returns_ok():
    node = MoveItArmNode(robot_id="ur5e-test")
    node.install_common_verbs()
    out = node.dispatch("robot.estop", {"reason": "manual"})
    assert out["ok"] is True
    assert node.is_estopped is True
    assert node.estop_reason == "manual"


def test_estop_reason_defaults_to_unspecified():
    node = MoveItArmNode(robot_id="ur5e-test")
    node.install_common_verbs()
    out = node.dispatch("robot.estop", {})
    assert out["ok"] is True
    assert node.estop_reason == "unspecified"
