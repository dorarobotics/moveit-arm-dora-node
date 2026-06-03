"""MoveItBridge Protocol — the verb handlers talk to this, not to dora-moveit2 directly.

The default implementation (`RealMoveItBridge`) wraps dora-moveit2's MoveGroup.
Tests use FakeMoveItBridge from tests/fakes.py.
"""
from __future__ import annotations

from typing import Any, Protocol


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


class RealMoveItBridge:
    """Default implementation backed by dora-moveit2's MoveGroup API.

    Imported lazily so unit tests (which use FakeMoveItBridge) need not have
    dora-moveit2 installed.
    """

    def __init__(self, *, robot_config_module: str) -> None:
        # Lazy import — keeps the package importable in test environments.
        from dora_moveit2 import MoveGroup  # noqa: PLC0415

        self._mg = MoveGroup(robot_config_module=robot_config_module)

    def move_to_joint_state(self, joints: list[float]) -> None:
        self._mg.move_to_joint_state(joints)

    def move_to_pose(self, pose: dict[str, Any]) -> None:
        self._mg.move_to_pose(pose)

    def move_to_named(self, name: str) -> None:
        self._mg.move_to_named(name)

    def plan(self, target: dict[str, Any]) -> dict[str, Any]:
        return self._mg.plan(target)

    def execute(self, trajectory: dict[str, Any]) -> None:
        self._mg.execute(trajectory)

    def add_collision(self, obj: dict[str, Any]) -> None:
        self._mg.scene.add_collision(obj)

    def clear_scene(self) -> None:
        self._mg.scene.clear()

    def current_joint_positions(self) -> list[float]:
        return list(self._mg.current_joint_positions())
