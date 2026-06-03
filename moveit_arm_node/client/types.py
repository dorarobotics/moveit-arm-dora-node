"""Typed payloads for Mode C client."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]  # xyzw quaternion

    def to_dict(self) -> dict[str, list[float]]:
        return {
            "position": list(self.position),
            "orientation": list(self.orientation),
        }
