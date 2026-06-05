import math

from moveit_arm_node.moveit_bridge import MoveGroupBridge
from tests.fakes import FakeMoveGroup


def test_move_to_joint_state_calls_go():
    mg = FakeMoveGroup()
    MoveGroupBridge(mg).move_to_joint_state([0.0, -1.0, 0.0, -1.0, 0.0, 0.0])
    assert ("go", [0.0, -1.0, 0.0, -1.0, 0.0, 0.0], True) in mg.calls


def test_move_to_named_sets_target_then_go():
    mg = FakeMoveGroup()
    MoveGroupBridge(mg).move_to_named("home")
    assert ("set_named_target", "home") in mg.calls
    assert any(c[0] == "go" for c in mg.calls)


def test_move_to_pose_converts_quaternion_to_rpy_then_go():
    mg = FakeMoveGroup()
    s = math.sin(math.pi / 4)
    MoveGroupBridge(mg).move_to_pose(
        {"position": [0.4, 0.0, 0.3], "orientation": [0.0, 0.0, s, s]}
    )
    target = next(c[1] for c in mg.calls if c[0] == "set_pose_target")
    assert target[:3] == [0.4, 0.0, 0.3]
    assert abs(target[5] - math.pi / 2) < 1e-6
    assert any(c[0] == "go" for c in mg.calls)


def test_failed_go_raises_runtimeerror():
    mg = FakeMoveGroup(fail_next=True)
    try:
        MoveGroupBridge(mg).move_to_joint_state([0.0] * 6)
    except RuntimeError:
        pass
    else:
        raise AssertionError("expected RuntimeError on failed go()")


def test_add_collision_dispatches_by_shape():
    mg = FakeMoveGroup()
    MoveGroupBridge(mg).add_collision(
        {"id": "box1", "shape": "box", "size": [0.1, 0.1, 0.1],
         "pose": {"position": [0.5, 0.0, 0.3], "orientation": [0, 0, 0, 1]}}
    )
    assert any(c[0] == "add_box" for c in mg.calls)


def test_clear_scene_and_current_joints():
    mg = FakeMoveGroup()
    b = MoveGroupBridge(mg)
    b.clear_scene()
    assert ("clear",) in mg.calls
    assert b.current_joint_positions() == [0.0] * 6


def test_start_move_to_joint_state_fires_async():
    mg = FakeMoveGroup()
    bridge = MoveGroupBridge(mg)
    bridge.start_move_to_joint_state([0.0, -1.0, 0.0, -1.0, 0.0, 0.0])
    assert ("begin_motion_async", [0.0, -1.0, 0.0, -1.0, 0.0, 0.0]) in mg.calls


def test_start_move_to_named_sets_target_then_async():
    mg = FakeMoveGroup()
    bridge = MoveGroupBridge(mg)
    bridge.start_move_to_named("home")
    assert ("set_named_target", "home") in mg.calls
    assert ("begin_motion_async", None) in mg.calls


def test_start_move_to_pose_sets_pose_then_async():
    mg = FakeMoveGroup()
    bridge = MoveGroupBridge(mg)
    bridge.start_move_to_pose({"position": [0.4, 0.0, 0.3], "orientation": [0, 0, 0, 1]})
    assert any(c[0] == "set_pose_target" for c in mg.calls)
    assert ("begin_motion_async", None) in mg.calls


def test_motion_status_pending_then_succeeded():
    mg = FakeMoveGroup()
    bridge = MoveGroupBridge(mg)
    assert bridge.motion_status() == ("pending", "")
    mg._plan_done = True
    mg._plan_success = True
    assert bridge.motion_status() == ("pending", "")  # exec not done yet
    mg._exec_done = True
    mg._exec_success = True
    assert bridge.motion_status() == ("succeeded", "")


def test_motion_status_plan_failure_carries_message():
    mg = FakeMoveGroup()
    bridge = MoveGroupBridge(mg)
    mg._plan_done = True
    mg._plan_success = False
    mg._plan_message = "no path found"
    status, msg = bridge.motion_status()
    assert status == "failed"
    assert msg == "no path found"


def test_motion_status_exec_failure():
    mg = FakeMoveGroup()
    bridge = MoveGroupBridge(mg)
    mg._plan_done = True
    mg._plan_success = True
    mg._exec_done = True
    mg._exec_success = False
    assert bridge.motion_status() == ("failed", "execution failed")


def test_stop_delegates_to_move_group():
    mg = FakeMoveGroup()
    bridge = MoveGroupBridge(mg)
    bridge.stop()
    assert ("stop",) in mg.calls
