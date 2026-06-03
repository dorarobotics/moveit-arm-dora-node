"""MoveItArmNode — dora node bridging SPEC-V1 verbs to dora-moveit2's MoveGroup API."""
from __future__ import annotations

import logging
from typing import Any, Callable

from moveit_arm_node._watchdog import HeartbeatWatchdog
from moveit_arm_node.controller_guard import ControllerGuard
from moveit_arm_node.gripper import Gripper
from moveit_arm_node.gripper.noop import NoopGripper
from moveit_arm_node.moveit_bridge import MoveItBridge, RealMoveItBridge

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
        moveit_bridge: MoveItBridge | None = None,
        gripper: Gripper | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.robot_config_module = robot_config_module
        self.gripper_driver_name = gripper_driver
        self.heartbeat_timeout_ms = heartbeat_timeout_ms
        self._verbs: dict[str, Callable[..., Any]] = {}
        self.is_estopped: bool = False
        self.estop_reason: str | None = None
        self._guard = ControllerGuard()
        self._bridge = moveit_bridge
        self._gripper = gripper

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
        self.register_verb("robot.release_control", self._verb_release_control)
        self.register_verb("robot.get_capabilities", self._verb_get_capabilities)

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

    def _verb_release_control(self, *, control_source: str = "") -> dict[str, Any]:
        self._guard.release(control_source)
        return {"ok": True, "code": "0"}

    def _verb_get_capabilities(self) -> dict[str, Any]:
        return {
            "ok": True,
            "code": "0",
            "data": {
                "spec_version": "1.0.0",
                "vendor": "moveit",
                "model": "arm",
                "robot_id": self.robot_id,
                "heartbeat_timeout_ms": self.heartbeat_timeout_ms,
                "verbs": sorted(self._verbs.keys()),
                "streams": ["state", "capabilities", "safety_event"],
            },
        }

    def install_motion_verbs(self) -> None:
        """Register motion planning and execution verbs."""
        if self._bridge is None:
            if not self.robot_config_module:
                raise ValueError("robot_config_module required to build RealMoveItBridge")
            self._bridge = RealMoveItBridge(robot_config_module=self.robot_config_module)
        self.register_verb(
            "vendor.moveit.arm.move_to_joint_state", self._verb_move_to_joint_state
        )
        self.register_verb("vendor.moveit.arm.move_to_pose", self._verb_move_to_pose)
        self.register_verb("vendor.moveit.arm.move_to_named", self._verb_move_to_named)
        self.register_verb("vendor.moveit.arm.plan", self._verb_plan)
        self.register_verb("vendor.moveit.arm.execute", self._verb_execute)

    def _verb_move_to_joint_state(
        self, *, joints: list[float], control_source: str = ""
    ) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        if len(joints) != 6:
            return {
                "ok": False,
                "code": "INVALID_PARAMS",
                "msg": f"joints must have length 6, got {len(joints)}",
            }
        try:
            self._guard.acquire(control_source)
        except Exception as e:
            return {"ok": False, "code": "CONTROLLER_BUSY", "msg": str(e)}
        try:
            assert self._bridge is not None
            self._bridge.move_to_joint_state(joints)
        except RuntimeError as e:
            return {"ok": False, "code": "VENDOR_ERROR", "msg": str(e)}
        return {"ok": True, "code": "0"}

    def _verb_move_to_pose(
        self, *, pose: dict[str, Any], control_source: str = ""
    ) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        if not isinstance(pose, dict) or "position" not in pose or "orientation" not in pose:
            return {
                "ok": False,
                "code": "INVALID_PARAMS",
                "msg": "pose must include `position` (xyz) and `orientation` (xyzw)",
            }
        try:
            self._guard.acquire(control_source)
        except Exception as e:
            return {"ok": False, "code": "CONTROLLER_BUSY", "msg": str(e)}
        try:
            assert self._bridge is not None
            self._bridge.move_to_pose(pose)
        except RuntimeError as e:
            return {"ok": False, "code": "VENDOR_ERROR", "msg": str(e)}
        return {"ok": True, "code": "0"}

    def _verb_move_to_named(
        self, *, name: str, control_source: str = ""
    ) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        if not name:
            return {"ok": False, "code": "INVALID_PARAMS", "msg": "name must be non-empty"}
        try:
            self._guard.acquire(control_source)
        except Exception as e:
            return {"ok": False, "code": "CONTROLLER_BUSY", "msg": str(e)}
        try:
            assert self._bridge is not None
            self._bridge.move_to_named(name)
        except RuntimeError as e:
            return {"ok": False, "code": "VENDOR_ERROR", "msg": str(e)}
        return {"ok": True, "code": "0"}

    def _verb_plan(self, *, target: dict[str, Any]) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        try:
            assert self._bridge is not None
            traj = self._bridge.plan(target)
        except RuntimeError as e:
            return {"ok": False, "code": "VENDOR_ERROR", "msg": str(e)}
        return {"ok": True, "code": "0", "data": {"trajectory": traj}}

    def _verb_execute(
        self, *, trajectory: dict[str, Any] | None = None, control_source: str = ""
    ) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        if trajectory is None:
            return {
                "ok": False,
                "code": "INVALID_PARAMS",
                "msg": "trajectory is required",
            }
        try:
            self._guard.acquire(control_source)
        except Exception as e:
            return {"ok": False, "code": "CONTROLLER_BUSY", "msg": str(e)}
        try:
            assert self._bridge is not None
            self._bridge.execute(trajectory)
        except RuntimeError as e:
            return {"ok": False, "code": "VENDOR_ERROR", "msg": str(e)}
        return {"ok": True, "code": "0"}

    def install_gripper_verbs(self) -> None:
        """Register gripper control verbs."""
        if self._gripper is None:
            self._gripper = self._build_gripper(self.gripper_driver_name)
        self.register_verb("vendor.moveit.arm.gripper.set", self._verb_gripper_set)
        self.register_verb("vendor.moveit.arm.gripper.open", self._verb_gripper_open)
        self.register_verb("vendor.moveit.arm.gripper.close", self._verb_gripper_close)

    @staticmethod
    def _build_gripper(name: str) -> Gripper:
        if name == "noop":
            return NoopGripper()
        # robotiq_2f85 wired in Task 17
        raise ValueError(f"unknown gripper driver: {name}")

    def _verb_gripper_set(self, *, width: float) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        if width < 0.0:
            return {
                "ok": False,
                "code": "INVALID_PARAMS",
                "msg": "width must be >= 0",
            }
        assert self._gripper is not None
        self._gripper.set(float(width))
        return {"ok": True, "code": "0"}

    def _verb_gripper_open(self) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        assert self._gripper is not None
        self._gripper.open()
        return {"ok": True, "code": "0"}

    def _verb_gripper_close(self) -> dict[str, Any]:
        if self.is_estopped:
            return {
                "ok": False,
                "code": "VENDOR_ERROR",
                "msg": f"node is estopped: {self.estop_reason}",
            }
        assert self._gripper is not None
        self._gripper.close()
        return {"ok": True, "code": "0"}
