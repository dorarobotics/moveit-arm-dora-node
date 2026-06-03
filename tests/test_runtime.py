"""Runtime/event-loop tests driven by a fake dora node (no real dora required)."""
import json
import time

import pyarrow as pa

from moveit_arm_node.gripper.noop import NoopGripper
from moveit_arm_node.node import MoveItArmNode
from moveit_arm_node.runtime import MoveItArmRuntime
from tests.fakes import FakeMoveItBridge


class FakeDoraNode:
    """Records send_output calls; decodes the pyarrow/JSON payloads."""

    def __init__(self) -> None:
        self.outputs: list[tuple[str, dict]] = []

    def send_output(self, output_id: str, data) -> None:  # noqa: ANN001
        self.outputs.append((output_id, json.loads(data.to_pylist()[0])))

    def by_id(self, output_id: str) -> list[dict]:
        return [payload for (oid, payload) in self.outputs if oid == output_id]


def _runtime() -> tuple[MoveItArmRuntime, MoveItArmNode]:
    node = MoveItArmNode(
        robot_id="ur5e-001", moveit_bridge=FakeMoveItBridge(), gripper=NoopGripper()
    )
    node.install_common_verbs()
    node.install_motion_verbs()
    node.install_gripper_verbs()
    node.install_scene_verbs()
    return MoveItArmRuntime(node), node


def _cmd_request(envelope: dict) -> dict:
    return {"type": "INPUT", "id": "cmd_request", "value": pa.array([json.dumps(envelope)])}


def test_start_publishes_capabilities_advert_with_commands():
    rt, _ = _runtime()
    fake = FakeDoraNode()
    rt.start(fake)
    rt.stop()
    caps = fake.by_id("capabilities")
    assert len(caps) == 1
    verbs = {cmd["verb"] for cmd in caps[0]["commands"]}
    assert "vendor.moveit.arm.move_to_pose" in verbs
    assert all("safety_tier" in cmd for cmd in caps[0]["commands"])


def test_on_event_dispatches_and_echoes_request_id():
    rt, _ = _runtime()
    fake = FakeDoraNode()
    env = {"verb": "robot.heartbeat", "params": {}, "request_id": "r1", "target": "ur5e-001"}
    assert rt.on_event(_cmd_request(env), fake) is True
    resp = fake.by_id("cmd_response")
    assert len(resp) == 1
    assert resp[0]["ok"] is True
    assert resp[0]["request_id"] == "r1"


def test_on_event_motion_verb_round_trip():
    rt, _ = _runtime()
    fake = FakeDoraNode()
    env = {
        "verb": "vendor.moveit.arm.move_to_named",
        "params": {"name": "home"},
        "request_id": "r2",
        "target": "ur5e-001",
    }
    rt.on_event(_cmd_request(env), fake)
    assert fake.by_id("cmd_response")[0]["ok"] is True


def test_on_event_drops_foreign_target():
    rt, _ = _runtime()
    fake = FakeDoraNode()
    env = {"verb": "robot.heartbeat", "params": {}, "request_id": "r1", "target": "other"}
    rt.on_event(_cmd_request(env), fake)
    assert fake.by_id("cmd_response") == []


def test_on_event_malformed_json_does_not_crash():
    rt, _ = _runtime()
    fake = FakeDoraNode()
    ev = {"type": "INPUT", "id": "cmd_request", "value": pa.array(["{not json"])}
    assert rt.on_event(ev, fake) is True
    assert fake.by_id("cmd_response") == []


def test_on_event_stop_returns_false():
    rt, _ = _runtime()
    assert rt.on_event({"type": "STOP"}, FakeDoraNode()) is False


def test_publish_state_once_emits_snapshot():
    rt, _ = _runtime()
    fake = FakeDoraNode()
    rt.publish_state_once(fake)
    states = fake.by_id("state")
    assert len(states) == 1
    assert states[0]["robot_id"] == "ur5e-001"
    assert len(states[0]["joint_positions"]) == 6


def test_publish_state_flushes_estop_safety_event():
    rt, node = _runtime()
    fake = FakeDoraNode()
    node.dispatch("robot.estop", {"reason": "manual"})
    rt.publish_state_once(fake)
    assert any(e["kind"] == "estop" for e in fake.by_id("safety_event"))


def test_watchdog_tick_publishes_deadman_and_estops():
    node = MoveItArmNode(
        robot_id="ur5e-001", moveit_bridge=FakeMoveItBridge(),
        gripper=NoopGripper(), heartbeat_timeout_ms=20,
    )
    node.install_common_verbs()
    node.install_motion_verbs()
    rt = MoveItArmRuntime(node)
    fake = FakeDoraNode()
    node.dispatch(
        "vendor.moveit.arm.move_to_joint_state",
        {"joints": [0.0] * 6, "control_source": "c"},
    )
    time.sleep(0.05)
    rt.tick_watchdog_once(fake)
    assert node.is_estopped is True
    assert any(e["kind"] == "heartbeat_timeout" for e in fake.by_id("safety_event"))
