"""MoveItArmClient — wraps MoveGroupBridge with typed Python API (no dora)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from moveit_arm_node.client.types import Pose
from moveit_arm_node.moveit_bridge import MoveItBridge


@dataclass(frozen=True)
class ArmConfig:
    robot_config_module: str


class MoveItArmClient:
    def __init__(self, *, config: ArmConfig, _bridge: MoveItBridge | None = None) -> None:
        self.config = config
        if _bridge is None:
            from dora_moveit2 import MoveGroup  # lazy
            from moveit_arm_node.moveit_bridge import MoveGroupBridge
            mg = MoveGroup(robot_config_module=config.robot_config_module)
            _bridge = MoveGroupBridge(mg)
        self._bridge = _bridge

    def move_to_joint_state(self, joints: list[float]) -> None:
        self._bridge.move_to_joint_state(joints)

    def move_to_pose(self, pose: Pose) -> None:
        self._bridge.move_to_pose(pose.to_dict())

    def move_to_named(self, name: str) -> None:
        self._bridge.move_to_named(name)

    def plan(self, target: dict[str, Any]) -> dict[str, Any]:
        return self._bridge.plan(target)

    def execute(self, trajectory: dict[str, Any]) -> None:
        self._bridge.execute(trajectory)

    def current_joint_positions(self) -> list[float]:
        return self._bridge.current_joint_positions()
