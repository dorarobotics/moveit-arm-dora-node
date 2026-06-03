"""Geometry helpers for the SPEC<->MoveGroup pose-convention bridge.

SPEC poses are {"position": [x,y,z], "orientation": [x,y,z,w]} (quaternion).
dora-moveit2's MoveGroup.set_pose_target expects [x, y, z, roll, pitch, yaw].

NOTE: the Euler order here (XYZ extrinsic / roll-pitch-yaw) must match what
dora-moveit2's IK expects; verified on-sim with a compute_fk(compute_ik) round-trip.
"""
from __future__ import annotations

import math
from typing import Any


def quat_to_euler_xyz(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Quaternion (x,y,z,w) -> (roll, pitch, yaw), XYZ extrinsic. Standard formula."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def pose_to_rpy(pose: dict[str, Any]) -> list[float]:
    """SPEC pose dict -> [x, y, z, roll, pitch, yaw] for MoveGroup.set_pose_target."""
    px, py, pz = pose["position"]
    qx, qy, qz, qw = pose["orientation"]
    roll, pitch, yaw = quat_to_euler_xyz(qx, qy, qz, qw)
    return [px, py, pz, roll, pitch, yaw]
