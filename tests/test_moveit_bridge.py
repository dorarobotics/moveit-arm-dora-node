"""Tests for MoveItBridge protocol and FakeMoveItBridge test double."""
from tests.fakes import FakeMoveItBridge


def test_fake_records_move_to_joint_state_calls():
    bridge = FakeMoveItBridge()
    bridge.move_to_joint_state([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
    assert bridge.calls == [("move_to_joint_state", ([0.0, -1.57, 0.0, -1.57, 0.0, 0.0],), {})]


def test_fake_records_plan_and_execute():
    bridge = FakeMoveItBridge()
    traj = bridge.plan({"position": [0.4, 0.0, 0.3], "orientation": [0, 0, 0, 1]})
    assert traj["fake_trajectory"] is True
    bridge.execute(traj)
    assert bridge.calls[-1][0] == "execute"


def test_fake_can_simulate_failure():
    bridge = FakeMoveItBridge(fail_next="VENDOR_ERROR")
    try:
        bridge.move_to_joint_state([0.0] * 6)
    except RuntimeError as e:
        assert "VENDOR_ERROR" in str(e)
    else:
        raise AssertionError("expected RuntimeError")
