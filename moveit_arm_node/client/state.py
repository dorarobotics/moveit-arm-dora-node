"""Mode C state types — mirror what the dora `state` topic emits."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotState:
    """Snapshot of arm + gripper state. Mirrors MoveItArmNode.state_snapshot()."""
    robot_id: str
    joint_positions: tuple[float, ...]
    gripper_width: float
    estopped: bool
    estop_reason: str | None
    controller_holder: str | None

    @classmethod
    def from_snapshot(cls, snap: dict) -> "RobotState":
        return cls(
            robot_id=snap["robot_id"],
            joint_positions=tuple(snap["joint_positions"]),
            gripper_width=float(snap["gripper_width"]),
            estopped=bool(snap["estopped"]),
            estop_reason=snap.get("estop_reason"),
            controller_holder=snap.get("controller_holder"),
        )
