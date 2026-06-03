"""MujocoGripper — Gripper Protocol backed by a dora-topic emit callback.

set/open/close publish {"width": w} via the injected `emit`; the runtime wires
`emit` to publish on moveit_arm_node's `gripper_command` output, which the
gripper_merge node folds into the MuJoCo control vector.
"""
from __future__ import annotations

from typing import Any, Callable


class MujocoGripper:
    def __init__(self, *, emit: Callable[[dict[str, Any]], None], open_width: float = 0.085) -> None:
        self._emit = emit
        self._open_width = open_width
        self._width = 0.0

    def set(self, width: float) -> None:
        self._width = float(width)
        self._emit({"width": float(width)})

    def open(self) -> None:
        self._width = self._open_width
        self._emit({"width": self._open_width})

    def close(self) -> None:
        self._width = 0.0
        self._emit({"width": 0.0})

    def width(self) -> float:
        return self._width
