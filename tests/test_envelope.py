from moveit_arm_node._envelope import (
    CmdRequest,
    InvalidEnvelope,
    build_cmd_response,
    parse_cmd_request,
)


def test_parse_cmd_request_extracts_bridge_fields():
    env = {
        "envelope_version": "1.0",
        "spec_version": "1.0.0",
        "verb": "vendor.moveit.arm.move_to_named",
        "target": "ur5e-001",
        "request_id": "abc-123",
        "params": {"name": "home"},
    }
    parsed = parse_cmd_request(env)
    assert parsed.request_id == "abc-123"
    assert parsed.verb == "vendor.moveit.arm.move_to_named"
    assert parsed.params == {"name": "home"}
    assert parsed.target == "ur5e-001"
    assert parsed.spec_version == "1.0.0"


def test_parse_cmd_request_rejects_missing_verb():
    try:
        parse_cmd_request({"request_id": "x", "params": {}})
    except InvalidEnvelope as e:
        assert "verb" in str(e)
    else:
        raise AssertionError("expected InvalidEnvelope")


def test_build_cmd_response_echoes_correlation_fields():
    req = CmdRequest(
        request_id="abc-123", verb="robot.heartbeat", params={},
        target="ur5e-001", spec_version="1.0.0", trace_id="t-1",
    )
    out = build_cmd_response(req, ok=True, code="0", data={"x": 1})
    assert out["request_id"] == "abc-123"
    assert out["ok"] is True
    assert out["code"] == "0"
    assert out["data"] == {"x": 1}
    assert out["trace_id"] == "t-1"
    assert out["envelope_version"] == "1.0"


def test_build_cmd_response_error_defaults_data_to_empty_dict():
    req = CmdRequest(
        request_id="abc", verb="x", params={},
        target=None, spec_version="1.0.0", trace_id=None,
    )
    out = build_cmd_response(req, ok=False, code="INVALID_PARAMS", msg="bad x")
    assert out["ok"] is False
    assert out["code"] == "INVALID_PARAMS"
    assert out["msg"] == "bad x"
    assert out["data"] == {}
    assert "trace_id" not in out
