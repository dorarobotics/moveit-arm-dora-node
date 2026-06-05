"""Test doubles — let unit tests run without dora-moveit2 installed."""
from __future__ import annotations

from typing import Any


class FakeMoveItBridge:
    """Records calls; can be primed to raise to simulate vendor errors."""

    def __init__(self, *, fail_next: str | None = None) -> None:
        self.calls: list[tuple[str, tuple[Any, ...], dict[str, Any]]] = []
        self._fail_next = fail_next
        self._joints = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        # Drives node/runtime deferred-motion tests: ("pending"|"succeeded"|"failed", msg)
        self.status: tuple[str, str] = ("pending", "")
        self.stopped = False

    def _record(self, name: str, *a: Any, **kw: Any) -> None:
        if self._fail_next is not None:
            code, self._fail_next = self._fail_next, None
            raise RuntimeError(code)
        self.calls.append((name, a, kw))

    def move_to_joint_state(self, joints: list[float]) -> None:
        self._record("move_to_joint_state", joints)
        self._joints = list(joints)

    def move_to_pose(self, pose: dict[str, Any]) -> None:
        self._record("move_to_pose", pose)

    def move_to_named(self, name: str) -> None:
        self._record("move_to_named", name)

    def start_move_to_joint_state(self, joints: list[float]) -> None:
        self._record("start_move_to_joint_state", joints)
        self._joints = list(joints)

    def start_move_to_pose(self, pose: dict[str, Any]) -> None:
        self._record("start_move_to_pose", pose)

    def start_move_to_named(self, name: str) -> None:
        self._record("start_move_to_named", name)

    def motion_status(self) -> tuple[str, str]:
        return self.status

    def stop(self) -> None:
        self.stopped = True

    def plan(self, target: dict[str, Any]) -> dict[str, Any]:
        self._record("plan", target)
        return {"fake_trajectory": True, "target": target}

    def execute(self, trajectory: dict[str, Any]) -> None:
        self._record("execute", trajectory)

    def add_collision(self, obj: dict[str, Any]) -> None:
        self._record("add_collision", obj)

    def clear_scene(self) -> None:
        self._record("clear_scene")

    def current_joint_positions(self) -> list[float]:
        return list(self._joints)


class FakeMoveGroup:
    """Records MoveGroup calls; lets MoveGroupBridge be tested without dora/mujoco."""

    def __init__(self, *, fail_next: bool = False, num_joints: int = 6):
        self.calls: list[tuple] = []
        self._fail_next = fail_next
        self._joints = [0.0] * num_joints
        # Completion flags MoveGroupBridge.motion_status() reads. Tests set these
        # to drive a fake motion through pending -> succeeded/failed.
        self._plan_done = False
        self._plan_success = False
        self._plan_message = ""
        self._exec_done = False
        self._exec_success = False

    def _maybe_fail(self):
        if self._fail_next:
            self._fail_next = False
            return False
        return True

    def go(self, joint_goal=None, wait=True, timeout=30.0):
        self.calls.append(("go", joint_goal, wait))
        if joint_goal is not None:
            self._joints = list(joint_goal)
        return self._maybe_fail()

    def begin_motion_async(self, joint_goal=None):
        self.calls.append(("begin_motion_async", joint_goal))
        if joint_goal is not None:
            self._joints = list(joint_goal)

    def set_pose_target(self, pose):
        self.calls.append(("set_pose_target", pose))

    def set_named_target(self, name):
        self.calls.append(("set_named_target", name))

    def plan(self, joint_goal=None, timeout=10.0):
        self.calls.append(("plan", joint_goal))
        return {"fake_trajectory": True}

    def execute(self, trajectory, wait=True, timeout=30.0):
        self.calls.append(("execute", trajectory))
        return self._maybe_fail()

    def add_box(self, name, position, size, color=None):
        self.calls.append(("add_box", name, position, size))

    def add_sphere(self, name, position, radius, color=None):
        self.calls.append(("add_sphere", name, position, radius))

    def add_cylinder(self, name, position, radius, height, color=None):
        self.calls.append(("add_cylinder", name, position, radius, height))

    def clear(self):
        self.calls.append(("clear",))

    def clear_pose_targets(self):
        self.calls.append(("clear_pose_targets",))

    def stop(self):
        self.calls.append(("stop",))

    def get_current_joint_values(self):
        return list(self._joints)
