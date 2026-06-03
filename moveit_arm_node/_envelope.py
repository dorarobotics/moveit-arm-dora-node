"""SPEC-V1 envelope helpers — the single source of truth for the wire shape.

The octos bridge (octos_spec_bridge.translator) sends cmd_request envelopes of
the form::

    {"envelope_version": "1.0", "spec_version": "1.0.0", "verb": <str>,
     "target": <robot_id>, "request_id": <uuid>, "ts": <iso>,
     "auth": {...}, "params": {<args>}}

and matches responses back by ``request_id``. These helpers parse that request
and build the matching SPEC §7.2 cmd_response. Both the dora runtime and the
in-process conformance adapter use them, so there is exactly one contract.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any


class InvalidEnvelope(ValueError):
    """Raised when a cmd_request envelope is missing required fields."""


def _now_iso() -> str:
    """ISO-8601 UTC timestamp, millisecond precision, trailing Z."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


@dataclass(frozen=True)
class CmdRequest:
    request_id: str
    verb: str
    params: dict[str, Any]
    target: str | None
    spec_version: str
    trace_id: str | None


def parse_cmd_request(env: dict[str, Any]) -> CmdRequest:
    """Parse a SPEC §7.1 cmd_request. ``verb`` is required; everything else has
    a sane default so a partial-but-routable envelope still dispatches."""
    if "verb" not in env:
        raise InvalidEnvelope("cmd_request missing required field: verb")
    return CmdRequest(
        request_id=str(env.get("request_id", "")),
        verb=str(env["verb"]),
        params=dict(env.get("params") or {}),
        target=env.get("target"),
        spec_version=str(env.get("spec_version", "1.0.0")),
        trace_id=env.get("trace_id"),
    )


def build_cmd_response(
    request: CmdRequest,
    *,
    ok: bool,
    code: str,
    data: dict[str, Any] | None = None,
    msg: str = "",
) -> dict[str, Any]:
    """Build a SPEC §7.2 cmd_response that echoes the request's correlation
    fields (request_id, spec_version, trace_id)."""
    response: dict[str, Any] = {
        "envelope_version": "1.0",
        "spec_version": request.spec_version,
        "request_id": request.request_id,
        "ok": bool(ok),
        "code": str(code),
        "msg": msg or "",
        "ts": _now_iso(),
        "data": data if data is not None else {},
    }
    if request.trace_id is not None:
        response["trace_id"] = request.trace_id
    return response


def error_response_for_raw(env: dict[str, Any], code: str, msg: str) -> dict[str, Any]:
    """Build an error cmd_response when the envelope could not be parsed into a
    CmdRequest (e.g. missing verb). Echoes whatever correlation fields exist."""
    response: dict[str, Any] = {
        "envelope_version": "1.0",
        "spec_version": str(env.get("spec_version", "1.0.0")),
        "request_id": str(env.get("request_id", "")),
        "ok": False,
        "code": code,
        "msg": msg,
        "ts": _now_iso(),
        "data": {},
    }
    if env.get("trace_id") is not None:
        response["trace_id"] = env["trace_id"]
    return response
