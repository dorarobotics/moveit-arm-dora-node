"""MoveItArmNode — dora node bridging SPEC-V1 verbs to dora-moveit2's MoveGroup API."""
from __future__ import annotations

import logging
from typing import Any, Callable

from moveit_arm_node._watchdog import HeartbeatWatchdog

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
        self.is_estopped: bool = False
        self.estop_reason: str | None = None

    def register_verb(self, name: str, handler: Callable[..., Any]) -> None:
        if name in self._verbs:
            raise ValueError(f"verb already registered: {name}")
        self._verbs[name] = handler

    def dispatch(self, verb: str, args: dict[str, Any]) -> dict[str, Any]:
        if verb not in self._verbs:
            return {"ok": False, "code": "INVALID_PARAMS", "msg": f"unknown verb: {verb}"}
        return self._verbs[verb](**args)

    def install_common_verbs(self) -> None:
        """Register the four SPEC-V1 §8.1 common verbs."""
        self._watchdog = HeartbeatWatchdog(
            timeout_s=self.heartbeat_timeout_ms / 1000.0,
            on_timeout=self._on_heartbeat_timeout,
        )
        self.register_verb("robot.heartbeat", self._verb_heartbeat)
        self.register_verb("robot.estop", self._verb_estop)

    def _verb_heartbeat(self) -> dict[str, Any]:
        self._watchdog.heartbeat()
        return {"ok": True, "code": "0"}

    def _on_heartbeat_timeout(self, _t: float) -> None:
        # Emitted as safety_event in Task 20; for now, log only.
        logger.warning("heartbeat timeout on %s", self.robot_id)

    def _verb_estop(self, *, reason: str = "unspecified") -> dict[str, Any]:
        self.is_estopped = True
        self.estop_reason = reason
        # Cancel any in-flight motion in Task 11 onward when motion verbs are added.
        return {"ok": True, "code": "0"}
