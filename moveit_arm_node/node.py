"""MoveItArmNode — dora node bridging SPEC-V1 verbs to dora-moveit2's MoveGroup API."""
from __future__ import annotations

import logging
from typing import Any, Callable, cast

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
        self._safety_events: list[dict[str, Any]] = []

    def register_verb(self, name: str, handler: Callable[..., Any]) -> None:
        if name in self._verbs:
            raise ValueError(f"verb already registered: {name}")
        self._verbs[name] = handler

    def dispatch(self, verb: str, args: dict[str, Any]) -> dict[str, Any]:
        if verb not in self._verbs:
            return {"ok": False, "code": "INVALID_PARAMS", "msg": f"unknown verb: {verb}"}
        try:
            return cast("dict[str, Any]", self._verbs[verb](**args))
        except TypeError as e:
            # Missing/extra/wrong-typed args reach the handler via **args; surface
            # them as a clean INVALID_PARAMS envelope instead of crashing the loop.
            return {"ok": False, "code": "INVALID_PARAMS", "msg": f"bad arguments for {verb}: {e}"}

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
        # Deadman fired: a motion was in flight and heartbeats lapsed. Engage estop
        # (latching) so no further motion is accepted, and disarm the watchdog.
        # NOTE: the periodic self._watchdog.tick() that triggers this must be driven
        # by the dora event loop (see __main__/node runtime — pending).
        logger.warning("heartbeat timeout on %s — engaging estop", self.robot_id)
        self.is_estopped = True
        self.estop_reason = "heartbeat_timeout"
        self._watchdog.disarm()
        self._safety_events.append({"kind": "heartbeat_timeout", "robot_id": self.robot_id})

    def _verb_estop(self, *, reason: str = "unspecified") -> dict[str, Any]:
        self.is_estopped = True
        self.estop_reason = reason
        self._watchdog.disarm()
        self._safety_events.append({"kind": "estop", "reason": reason})
        return {"ok": True, "code": "0"}

    def _verb_release_control(self, *, control_source: str = "") -> dict[str, Any]:
        self._guard.release(control_source)
        self._watchdog.disarm()
        return {"ok": True, "code": "0"}

    def capabilities_advert(self) -> dict[str, Any]:
        """The SPEC-V1 advert published on the `capabilities` stream.

        Commands are a list of ``{"verb", "safety_tier"}`` objects — the shape the
        octos bridge consumes (it reads ``advert["commands"][*]["verb"]``). A flat
        verb-name list is NOT recognized by the bridge.
        """
        return {
            "spec_version": "1.0.0",
            "vendor": "moveit",
            "model": "arm",
            "robot_id": self.robot_id,
            "heartbeat_timeout_ms": self.heartbeat_timeout_ms,
            "commands": [
                {"verb": verb, "safety_tier": "emergency_override"}
                for verb in sorted(self._verbs.keys())
            ],
            "streams": ["state", "capabilities", "safety_event"],
        }

    def _verb_get_capabilities(self) -> dict[str, Any]:
        return {"ok": True, "code": "0", "data": self.capabilities_advert()}

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
        self._watchdog.arm()
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
        self._watchdog.arm()
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
        self._watchdog.arm()
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
        self._watchdog.arm()
        try:
            assert self._bridge is not None
            self._bridge.execute(trajectory)
        except RuntimeError as e:
            return {"ok": False, "code": "VENDOR_ERROR", "msg": str(e)}
        return {"ok": True, "code": "0"}

    def install_scene_verbs(self) -> None:
        """Register scene manipulation verbs."""
        if self._bridge is None:
            if not self.robot_config_module:
                raise ValueError("robot_config_module required to build RealMoveItBridge")
            self._bridge = RealMoveItBridge(robot_config_module=self.robot_config_module)
        self.register_verb(
            "vendor.moveit.arm.scene.add_collision", self._verb_scene_add_collision
        )
        self.register_verb("vendor.moveit.arm.scene.clear", self._verb_scene_clear)

    def _verb_scene_add_collision(
        self, *, object: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        if object is None:
            return {"ok": False, "code": "INVALID_PARAMS", "msg": "object is required"}
        try:
            assert self._bridge is not None
            self._bridge.add_collision(object)
        except RuntimeError as e:
            return {"ok": False, "code": "VENDOR_ERROR", "msg": str(e)}
        return {"ok": True, "code": "0"}

    def _verb_scene_clear(self) -> dict[str, Any]:
        try:
            assert self._bridge is not None
            self._bridge.clear_scene()
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
        if name == "robotiq_2f85":
            # Real transport injection is deployment-specific; for now we error out
            # to make the misconfiguration obvious. Hardware bringup task will provide
            # a concrete Modbus/URCap transport.
            raise NotImplementedError(
                "robotiq_2f85 transport must be injected explicitly via the "
                "MoveItArmNode(gripper=...) ctor argument in hardware deployment"
            )
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

    def state_snapshot(self) -> dict[str, Any]:
        """Capture current state for 10 Hz state stream."""
        joints: list[float] = (
            self._bridge.current_joint_positions()
            if self._bridge is not None
            else [0.0] * 6
        )
        return {
            "robot_id": self.robot_id,
            "joint_positions": joints,
            "gripper_width": (self._gripper.width() if self._gripper is not None else 0.0),
            "estopped": self.is_estopped,
            "estop_reason": self.estop_reason,
            "controller_holder": self._guard.holder,
        }

    def drain_safety_events(self) -> list[dict[str, Any]]:
        """Drain and clear the safety event queue."""
        out = list(self._safety_events)
        self._safety_events.clear()
        return out
