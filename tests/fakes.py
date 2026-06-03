"""Test doubles — let unit tests run without dora-moveit2 installed."""
from __future__ import annotations

from typing import Any


class FakeMoveItBridge:
    """Records calls; can be primed to raise to simulate vendor errors."""

    def __init__(self, *, fail_next: str | None = None) -> None:
        self.calls: list[tuple[str, tuple[Any, ...], dict[str, Any]]] = []
        self._fail_next = fail_next
        self._joints = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

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
