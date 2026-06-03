"""MoveItArmNode — dora node bridging SPEC-V1 verbs to dora-moveit2's MoveGroup API."""
from __future__ import annotations

import logging
from typing import Any, Callable

logger = logging.getLogger(__name__)


class MoveItArmNode:
    """Spec-conforming dora node. Verbs are dispatched by name in `_handle_request`."""

    def __init__(
        self,
        *,
        robot_id: str,
        robot_config_module: str | None = None,
        gripper_driver: str = "noop",
        heartbeat_timeout_ms: int = 1000,
    ) -> None:
        self.robot_id = robot_id
        self.robot_config_module = robot_config_module
        self.gripper_driver_name = gripper_driver
        self.heartbeat_timeout_ms = heartbeat_timeout_ms
        self._verbs: dict[str, Callable[..., Any]] = {}

    def register_verb(self, name: str, handler: Callable[..., Any]) -> None:
        if name in self._verbs:
            raise ValueError(f"verb already registered: {name}")
        self._verbs[name] = handler

    def dispatch(self, verb: str, args: dict[str, Any]) -> dict[str, Any]:
        if verb not in self._verbs:
            return {"ok": False, "code": "INVALID_PARAMS", "msg": f"unknown verb: {verb}"}
        return self._verbs[verb](**args)
