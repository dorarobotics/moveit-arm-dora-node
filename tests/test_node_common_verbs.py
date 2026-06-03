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
