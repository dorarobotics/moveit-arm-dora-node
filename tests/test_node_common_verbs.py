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


def test_release_control_frees_motion_lock():
    node = MoveItArmNode(robot_id="ur5e-test")
    node.install_common_verbs()
    node._guard.acquire("caller-a")  # simulate prior motion verb acquired it
    out = node.dispatch("robot.release_control", {"control_source": "caller-a"})
    assert out["ok"] is True
    assert node._guard.holder is None


def test_release_control_by_non_holder_is_ok_but_noop():
    node = MoveItArmNode(robot_id="ur5e-test")
    node.install_common_verbs()
    node._guard.acquire("caller-a")
    out = node.dispatch("robot.release_control", {"control_source": "stranger"})
    assert out["ok"] is True   # spec: idempotent; never an error
    assert node._guard.holder == "caller-a"


def test_get_capabilities_returns_spec_advert_shape():
    node = MoveItArmNode(robot_id="ur5e-test")
    node.install_common_verbs()
    out = node.dispatch("robot.get_capabilities", {})
    assert out["ok"] is True
    data = out["data"]
    assert data["spec_version"] == "1.0.0"
    assert data["vendor"] == "moveit"
    assert data["model"] == "arm"
    assert "robot.heartbeat" in data["verbs"]
    assert "robot.estop" in data["verbs"]
    assert "robot.release_control" in data["verbs"]
    assert "robot.get_capabilities" in data["verbs"]
    assert "state" in data["streams"]
    assert "safety_event" in data["streams"]
