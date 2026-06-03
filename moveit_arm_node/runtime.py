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

    def __init__(self, node: MoveItArmNode) -> None:
        self._node = node
        self._started = False
        self._stop_event = threading.Event()
        self._workers: list[threading.Thread] = []

    # ---- request handling (pure: dict in, dict out) ----

    def handle_request(self, envelope: dict[str, Any]) -> dict[str, Any]:
        """Turn a cmd_request envelope into a cmd_response envelope. Pure — no
        transport, no target filtering (the caller decides routing)."""
        try:
            req = parse_cmd_request(envelope)
        except InvalidEnvelope as exc:
            return error_response_for_raw(envelope, "INVALID_PARAMS", str(exc))
        result = self._node.dispatch(req.verb, req.params)
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
        if etype != "INPUT" or event.get("id") != "cmd_request":
            return True
        envelope = _decode_value(event.get("value"))
        if envelope is None:
            return True
        # SPEC §6.1 target filtering — silently drop foreign-addressed commands.
        target = envelope.get("target")
        if target is not None and target != self._node.robot_id:
            return True
        response = self.handle_request(envelope)
        _emit(dora_node, "cmd_response", response)
        return True

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


def build_node_from_env() -> MoveItArmNode:
    """Construct + fully install a MoveItArmNode from environment variables."""
    robot_id = os.environ.get("ROBOT_ID")
    if not robot_id:
        raise SystemExit("ROBOT_ID env var is required")
    node = MoveItArmNode(
        robot_id=robot_id,
        robot_config_module=os.environ.get("ROBOT_CONFIG_MODULE"),
        gripper_driver=os.environ.get("GRIPPER_DRIVER", "noop"),
        heartbeat_timeout_ms=int(os.environ.get("HEARTBEAT_TIMEOUT_MS", "1000")),
    )
    node.install_common_verbs()
    node.install_motion_verbs()
    node.install_gripper_verbs()
    node.install_scene_verbs()
    return node


def main() -> int:
    """Dora entry point. Imports dora lazily so Mode C / unit tests never need it."""
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))
    node = build_node_from_env()
    runtime = MoveItArmRuntime(node)
    from dora import Node  # noqa: PLC0415 — lazy; needs a running dora daemon

    dora_node = Node()
    try:
        runtime.start(dora_node)
        for event in dora_node:
            if not runtime.on_event(event, dora_node):
                break
    finally:
        runtime.stop()
    return 0
