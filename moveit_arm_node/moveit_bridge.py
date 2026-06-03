"""MoveItBridge Protocol — the verb handlers talk to this, not to dora-moveit2 directly.

The default implementation (`MoveGroupBridge`) wraps dora-moveit2's MoveGroup.
Tests use FakeMoveItBridge or FakeMoveGroup from tests/fakes.py.
"""
from __future__ import annotations

from typing import Any, Protocol

from moveit_arm_node._geometry import pose_to_rpy


class MoveItBridge(Protocol):
    """Protocol for interacting with MoveIt.

    Verb handlers depend on this, not on dora-moveit2 directly, so tests can
    inject FakeMoveItBridge without requiring dora-moveit2 to be installed.
    """

    def move_to_joint_state(self, joints: list[float]) -> None: ...

    def move_to_pose(self, pose: dict[str, Any]) -> None: ...

    def move_to_named(self, name: str) -> None: ...

    def plan(self, target: dict[str, Any]) -> dict[str, Any]: ...

    def execute(self, trajectory: dict[str, Any]) -> None: ...

    def add_collision(self, obj: dict[str, Any]) -> None: ...

    def clear_scene(self) -> None: ...

    def current_joint_positions(self) -> list[float]: ...


class MoveGroupBridge:
    """MoveItBridge implementation backed by dora-moveit2's MoveGroup.

    The MoveGroup is injected (the runtime builds it sharing moveit_arm_node's
    dora Node). MoveGroup ops are synchronous/blocking. Failures (False return /
    exception) become RuntimeError so the node maps them to VENDOR_ERROR.
    """

    def __init__(self, move_group: Any) -> None:
        self._mg = move_group

    @staticmethod
    def _check(ok: Any, what: str) -> None:
        if ok is False:
            raise RuntimeError(f"MoveGroup {what} failed")

    def move_to_joint_state(self, joints: list[float]) -> None:
        self._check(self._mg.go(joints, wait=True), "go(joint_state)")

    def move_to_pose(self, pose: dict[str, Any]) -> None:
        self._mg.set_pose_target(pose_to_rpy(pose))
        ok = self._mg.go(wait=True)
        self._mg.clear_pose_targets()
        self._check(ok, "go(pose)")

    def move_to_named(self, name: str) -> None:
        self._mg.set_named_target(name)
        self._check(self._mg.go(wait=True), f"go(named={name})")

    def plan(self, target: dict[str, Any]) -> dict[str, Any]:
        if "position" in target and "orientation" in target:
            self._mg.set_pose_target(pose_to_rpy(target))
            traj = self._mg.plan()
            self._mg.clear_pose_targets()
        else:
            traj = self._mg.plan(target.get("joints"))
        if traj is None:
            raise RuntimeError("MoveGroup plan failed")
        return traj

    def execute(self, trajectory: dict[str, Any]) -> None:
        self._check(self._mg.execute(trajectory), "execute")

    def add_collision(self, obj: dict[str, Any]) -> None:
        shape = obj.get("shape", "box")
        name = obj.get("id", "obj")
        pos = obj.get("pose", {}).get("position", [0.0, 0.0, 0.0])
        if shape == "box":
            self._mg.add_box(name, pos, obj.get("size", [0.1, 0.1, 0.1]))
        elif shape == "sphere":
            self._mg.add_sphere(name, pos, obj.get("radius", 0.05))
        elif shape == "cylinder":
            self._mg.add_cylinder(name, pos, obj.get("radius", 0.05), obj.get("height", 0.1))
        else:
            raise RuntimeError(f"unknown collision shape: {shape}")

    def clear_scene(self) -> None:
        self._mg.clear()

    def current_joint_positions(self) -> list[float]:
        return list(self._mg.get_current_joint_values())
