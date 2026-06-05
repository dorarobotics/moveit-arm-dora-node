"""Dora runtime for MoveItArmNode — the piece that makes it a runnable dora node.

Kept separate from node.py so the verb logic stays import-light and unit-testable
without a dora runtime. The loop here is verifiable too: `handle_request`,
`on_event`, `publish_state_once`, and `tick_watchdog_once` all run against a
``DoraNodeLike`` fake. Only `main()` touches the real `dora.Node()` (which needs a
running dora daemon) and is therefore the single unverified-by-unit-test seam.

Threading model mirrors the agibot-a2 reference: dora delivers cmd_request events
serially to `on_event` on the main thread, while two daemon workers publish the
`state` stream (10 Hz) and tick the heartbeat watchdog (10 Hz). dora's
`Node.send_output` is documented thread-safe.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Protocol

import pyarrow as pa

from moveit_arm_node._envelope import (
    InvalidEnvelope,
    build_cmd_response,
    error_response_for_raw,
    parse_cmd_request,
)
from moveit_arm_node.node import MoveItArmNode

logger = logging.getLogger(__name__)

STATE_PERIOD_S = 0.1  # 10 Hz state stream
WATCHDOG_PERIOD_S = 0.1  # 10 Hz watchdog tick

MOTION_DEADLINE_S = 55.0  # < bridge CMD_TIMEOUT_S (60) so we resolve before it gives up

MOTION_VERBS = frozenset({
    "vendor.moveit.arm.move_to_joint_state",
    "vendor.moveit.arm.move_to_pose",
    "vendor.moveit.arm.move_to_named",
})


@dataclass
class PendingOp:
    """A motion verb whose cmd_response is withheld until the motion completes."""
    request: Any  # CmdRequest (carries request_id, spec_version, trace_id)
    started: float  # time.monotonic() at dispatch


class DoraNodeLike(Protocol):
    def send_output(self, output_id: str, data: Any) -> None: ...


def _decode_value(value: Any) -> dict[str, Any] | None:
    """Decode a dora INPUT value (pyarrow array of one JSON string) to a dict."""
    try:
        decoded = value.to_pylist() if hasattr(value, "to_pylist") else list(value)
    except Exception:  # noqa: BLE001
        logger.exception("failed to read dora input value")
        return None
    if not decoded:
        return None
    first = decoded[0]
    try:
        return json.loads(first) if isinstance(first, str) else dict(first)
    except (json.JSONDecodeError, TypeError, ValueError):
        logger.exception("failed to decode JSON envelope")
        return None


def _emit(dora_node: DoraNodeLike, output_id: str, payload: dict[str, Any]) -> None:
    dora_node.send_output(output_id, pa.array([json.dumps(payload)]))


class MoveItArmRuntime:
    """Binds a MoveItArmNode to a dora node: publishes the advert, dispatches
    cmd_request → cmd_response, and streams state + safety events."""

    def __init__(self, node: MoveItArmNode, move_group: Any = None) -> None:
        self._node = node
        self._mg = move_group
        self._started = False
        self._stop_event = threading.Event()
        self._workers: list[threading.Thread] = []
        self._pending: PendingOp | None = None
        self.MOTION_DEADLINE_S = MOTION_DEADLINE_S

    # ---- request handling (pure: dict in, dict out) ----

    def handle_request(self, envelope: dict[str, Any]) -> dict[str, Any] | None:
        """Turn a cmd_request into a cmd_response, or return None to withhold it
        (deferred motion). Pure except for recording the pending op."""
        try:
            req = parse_cmd_request(envelope)
        except InvalidEnvelope as exc:
            return error_response_for_raw(envelope, "INVALID_PARAMS", str(exc))
        # A motion is already in flight — reject a second one (single in-flight).
        if self._pending is not None and req.verb in MOTION_VERBS:
            return build_cmd_response(
                req, ok=False, code="CONTROLLER_BUSY",
                msg="a motion is already in progress",
            )
        result = self._node.dispatch(req.verb, req.params)
        if result.get("code") == "DEFERRED":
            self._pending = PendingOp(request=req, started=time.monotonic())
            return None
        return build_cmd_response(
            req,
            ok=bool(result.get("ok", False)),
            code=str(result.get("code", "0")),
            data=result.get("data"),
            msg=str(result.get("msg", "")),
        )

    # ---- dora event loop ----

    def on_event(self, event: dict[str, Any], dora_node: DoraNodeLike) -> bool:
        """Process one dora event. Returns False when the loop should stop."""
        etype = event.get("type")
        if etype == "STOP":
            return False
        if etype != "INPUT":
            return True
        if event.get("id") == "cmd_request":
            envelope = _decode_value(event.get("value"))
            if envelope is not None:
                target = envelope.get("target")
                if target is None or target == self._node.robot_id:
                    response = self.handle_request(envelope)
                    if response is not None:
                        _emit(dora_node, "cmd_response", response)
        else:
            # joint_positions / plan_status / trajectory / execution_status / ik_* /
            # command_result belong to the shared MoveGroup orchestration.
            if self._mg is not None:
                self._mg.handle_event(event)
        # Resolve any in-flight deferred motion (the 10 Hz joint stream keeps this
        # firing throughout a move, and enforces the deadline).
        self._check_pending(dora_node)
        return True

    def _check_pending(self, dora_node: DoraNodeLike) -> None:
        if self._pending is None:
            return
        op = self._pending
        # estop aborts an in-flight motion promptly (reuses the node's latched flag).
        if self._node.is_estopped:
            self._node.stop_motion()
            self._emit_pending(dora_node, op, ok=False, code="VENDOR_ERROR",
                               msg="aborted by estop")
            return
        status, msg = self._node.motion_status()
        if status == "succeeded":
            self._emit_pending(dora_node, op, ok=True, code="0")
        elif status == "failed":
            self._emit_pending(dora_node, op, ok=False, code="VENDOR_ERROR", msg=msg)
        elif time.monotonic() - op.started > self.MOTION_DEADLINE_S:
            self._emit_pending(dora_node, op, ok=False, code="BRIDGE_TIMEOUT",
                               msg="motion did not complete in time")

    def _emit_pending(self, dora_node: DoraNodeLike, op: "PendingOp", *,
                      ok: bool, code: str, msg: str = "") -> None:
        _emit(dora_node, "cmd_response",
              build_cmd_response(op.request, ok=ok, code=code, msg=msg))
        self._pending = None
        self._node.end_motion()  # release control lock + disarm watchdog

    # ---- streams (one-shot; exposed for deterministic testing) ----

    def publish_state_once(self, dora_node: DoraNodeLike) -> None:
        _emit(dora_node, "state", self._node.state_snapshot())
        for ev in self._node.drain_safety_events():
            _emit(dora_node, "safety_event", ev)

    def tick_watchdog_once(self, dora_node: DoraNodeLike) -> None:
        watchdog = getattr(self._node, "_watchdog", None)
        if watchdog is not None:
            watchdog.tick()
        # A fired deadman queues a safety event + latches estop; flush it promptly.
        for ev in self._node.drain_safety_events():
            _emit(dora_node, "safety_event", ev)

    # ---- lifecycle ----

    def start(self, dora_node: DoraNodeLike) -> None:
        """Publish the capabilities advert once and spawn the stream workers."""
        if self._started:
            return
        _emit(dora_node, "capabilities", self._node.capabilities_advert())
        self._stop_event.clear()
        self._workers = [
            threading.Thread(
                target=self._state_loop, args=(dora_node,),
                name="moveit-arm-state-loop", daemon=True,
            ),
            threading.Thread(
                target=self._watchdog_loop, args=(dora_node,),
                name="moveit-arm-watchdog-loop", daemon=True,
            ),
        ]
        for t in self._workers:
            t.start()
        self._started = True

    def stop(self) -> None:
        self._stop_event.set()
        for t in self._workers:
            t.join(timeout=2.0)
        self._workers = []
        self._started = False

    def _state_loop(self, dora_node: DoraNodeLike) -> None:
        while not self._stop_event.wait(STATE_PERIOD_S):
            try:
                self.publish_state_once(dora_node)
            except Exception:  # noqa: BLE001
                logger.exception("state loop iteration failed")

    def _watchdog_loop(self, dora_node: DoraNodeLike) -> None:
        while not self._stop_event.wait(WATCHDOG_PERIOD_S):
            try:
                self.tick_watchdog_once(dora_node)
            except Exception:  # noqa: BLE001
                logger.exception("watchdog loop iteration failed")


def main() -> int:
    """Dora entry point. Imports dora lazily so Mode C / unit tests never need it."""
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))
    robot_id = os.environ.get("ROBOT_ID")
    if not robot_id:
        raise SystemExit("ROBOT_ID env var is required")
    group = os.environ.get("ROBOT_CONFIG_MODULE", "ur5e")
    hb = int(os.environ.get("HEARTBEAT_TIMEOUT_MS", "1000"))

    from dora import Node  # noqa: PLC0415 — lazy
    from dora_moveit.workflow.move_group import MoveGroup  # noqa: PLC0415
    from moveit_arm_node.moveit_bridge import MoveGroupBridge  # noqa: PLC0415
    from moveit_arm_node.gripper.mujoco import MujocoGripper  # noqa: PLC0415

    dora_node = Node()
    mg = MoveGroup(group, node=dora_node)
    bridge = MoveGroupBridge(mg)

    def _emit_gripper(payload: dict[str, Any]) -> None:
        import json
        import pyarrow as pa
        dora_node.send_output("gripper_command", pa.array([json.dumps(payload)]))

    gripper = MujocoGripper(emit=_emit_gripper)
    node = MoveItArmNode(
        robot_id=robot_id, heartbeat_timeout_ms=hb,
        moveit_bridge=bridge, gripper=gripper,
    )
    node.install_common_verbs()
    node.install_motion_verbs()
    node.install_gripper_verbs()
    node.install_scene_verbs()

    runtime = MoveItArmRuntime(node, move_group=mg)
    try:
        runtime.start(dora_node)
        for event in dora_node:
            if not runtime.on_event(event, dora_node):
                break
    finally:
        runtime.stop()
    return 0
