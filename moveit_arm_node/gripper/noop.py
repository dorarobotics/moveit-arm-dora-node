"""NoopGripper — used when the dataflow has no real gripper hardware."""
from __future__ import annotations


class NoopGripper:
    def __init__(self) -> None:
        self.last_set: float = 0.0

    def set(self, width: float) -> None:
        self.last_set = float(width)

    def open(self) -> None:
        self.last_set = 0.085   # arbitrary "wide" default

    def close(self) -> None:
        self.last_set = 0.0

    def width(self) -> float:
        return 0.0  # we don't observe physical width
