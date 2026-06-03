from moveit_arm_node._envelope import (
    build_cmd_response,
    parse_cmd_request,
    InvalidEnvelope,
)


def test_parse_cmd_request_extracts_fields():
    env = {
        "id": "abc-123",
        "verb": "robot.heartbeat",
        "args": {},
        "target": "ur5e-001",
        "source": "test-suite",
    }
    parsed = parse_cmd_request(env)
    assert parsed.id == "abc-123"
    assert parsed.verb == "robot.heartbeat"
    assert parsed.args == {}
    assert parsed.target == "ur5e-001"


def test_parse_cmd_request_rejects_missing_verb():
    env = {"id": "x", "args": {}}
    try:
        parse_cmd_request(env)
    except InvalidEnvelope as e:
        assert "verb" in str(e)
    else:
        raise AssertionError("expected InvalidEnvelope")


def test_build_cmd_response_ok():
    out = build_cmd_response(request_id="abc-123", ok=True, code="0", data={"x": 1})
    assert out["id"] == "abc-123"
    assert out["ok"] is True
    assert out["code"] == "0"
    assert out["data"] == {"x": 1}


def test_build_cmd_response_error():
    out = build_cmd_response(
        request_id="abc", ok=False, code="INVALID_PARAMS", msg="bad x"
    )
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"
    assert out["msg"] == "bad x"
