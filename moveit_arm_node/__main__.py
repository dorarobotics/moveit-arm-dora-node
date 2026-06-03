"""Dora entry point — `python -m moveit_arm_node`."""
from __future__ import annotations

import os
import sys


def main() -> int:
    """Construct the node from env, hook up dora, run until shutdown.

    Wiring to dora-rs is added in Task 3 (envelope helpers); this stub exists so
    `python -m moveit_arm_node --help` and conformance imports work today.
    """
    robot_id = os.environ.get("ROBOT_ID")
    if not robot_id:
        sys.stderr.write("ROBOT_ID env var is required\n")
        return 2
    sys.stdout.write(f"moveit_arm_node starting for {robot_id}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
