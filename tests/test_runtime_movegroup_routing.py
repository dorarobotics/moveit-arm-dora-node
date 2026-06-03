"""The runtime routes cmd_request->dispatch and all other events->MoveGroup.handle_event."""
import json

import pyarrow as pa

from moveit_arm_node.node import MoveItArmNode
from moveit_arm_node.runtime import MoveItArmRuntime
from tests.fakes import FakeMoveItBridge


class _RecordingMoveGroup:
    def __init__(self):
        self.events = []

    def handle_event(self, event):
        self.events.append(event.get("id", event.get("type")))


class FakeDoraNode:
    def __init__(self):
        self.outputs = []

    def send_output(self, output_id, data):
        self.outputs.append((output_id, json.loads(data.to_pylist()[0])))


def _node():
    n = MoveItArmNode(robot_id="ur5e-001", moveit_bridge=FakeMoveItBridge())
    n.install_common_verbs()
    return n


def test_non_cmd_events_go_to_movegroup_handle_event():
    n = _node()
    mg = _RecordingMoveGroup()
    rt = MoveItArmRuntime(n, move_group=mg)
    fake = FakeDoraNode()
    ev = {"type": "INPUT", "id": "joint_positions", "value": pa.array(['[0,0,0,0,0,0]'])}
    rt.on_event(ev, fake)
    assert "joint_positions" in mg.events
    assert all(oid != "cmd_response" for oid, _ in fake.outputs)


def test_cmd_request_still_dispatches_and_responds():
    n = _node()
    rt = MoveItArmRuntime(n, move_group=_RecordingMoveGroup())
    fake = FakeDoraNode()
    env = {"verb": "robot.heartbeat", "params": {}, "request_id": "r1", "target": "ur5e-001"}
    ev = {"type": "INPUT", "id": "cmd_request", "value": pa.array([json.dumps(env)])}
    rt.on_event(ev, fake)
    assert any(oid == "cmd_response" for oid, _ in fake.outputs)
