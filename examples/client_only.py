#!/usr/bin/env python3
"""Mode C — drive the arm directly via the client library.

Requires dora-moveit2 to be installed and a robot config module to be importable.
Set ARM_ROBOT_CONFIG_MODULE if running against a non-UR5e config.
"""
from __future__ import annotations

import os

from moveit_arm_node.client import ArmConfig, MoveItArmClient
from moveit_arm_node.client.types import Pose


def main() -> None:
    cfg = ArmConfig(robot_config_module=os.environ.get("ARM_ROBOT_CONFIG_MODULE", "ur5e"))
    client = MoveItArmClient(config=cfg)

    print("current joints:", client.current_joint_positions())
    client.move_to_named("home")
    print("at home")

    target = Pose(position=(0.4, 0.0, 0.3), orientation=(0.0, 0.0, 0.0, 1.0))
    client.move_to_pose(target)
    print("reached target pose")


if __name__ == "__main__":
    main()
