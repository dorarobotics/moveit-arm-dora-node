"""Gripper Protocol — vendor-specific drivers live in this subpackage."""
from __future__ import annotations

from typing import Protocol


class Gripper(Protocol):
    def set(self, width: float) -> None:
        """Move gripper to specified width in meters (0.0 = closed)."""

    def open(self) -> None:
        """Move gripper to its maximum open width."""

    def close(self) -> None:
        """Move gripper fully closed."""

    def width(self) -> float:
        """Most recent gripper width in meters (0.0 if unknown)."""
