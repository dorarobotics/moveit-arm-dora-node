"""Dora entry point — `python -m moveit_arm_node`."""
from __future__ import annotations

from moveit_arm_node.runtime import main

if __name__ == "__main__":
    raise SystemExit(main())
