"""Robotiq 2F-85 parallel gripper driver.

Transport is injected so unit tests can use a fake. The real transport (Modbus-TCP
or URCap, depending on deployment) is wired up in the hardware bringup task
outside this plan.
"""
from __future__ import annotations

from typing import Protocol


class RobotiqTransport(Protocol):
    def send(self, op: str, value: float | None = None) -> None: ...


class Robotiq2F85:
    """Implements the Gripper Protocol over an injected transport."""

    OPEN_WIDTH_M = 0.085  # 2F-85 max stroke

    def __init__(self, *, transport: RobotiqTransport) -> None:
        self._t = transport
        self._width = 0.0

    def set(self, width: float) -> None:
        self._t.send("set_position", float(width))
        self._width = float(width)

    def open(self) -> None:
        self._t.send("open")
        self._width = self.OPEN_WIDTH_M

    def close(self) -> None:
        self._t.send("close")
        self._width = 0.0

    def width(self) -> float:
        return self._width
