"""SPEC-V1 envelope helpers — parse cmd_request, build cmd_response.

Mirrors the JSON envelope shape used by agibot-a2-dora-node so the bridge
treats vendors uniformly.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


class InvalidEnvelope(ValueError):
    """Raised when a cmd_request envelope is missing required fields."""


@dataclass(frozen=True)
class CmdRequest:
    id: str
    verb: str
    args: dict[str, Any]
    target: str | None
    source: str | None


def parse_cmd_request(env: dict[str, Any]) -> CmdRequest:
    for key in ("id", "verb"):
        if key not in env:
            raise InvalidEnvelope(f"cmd_request missing required field: {key}")
    return CmdRequest(
        id=str(env["id"]),
        verb=str(env["verb"]),
        args=dict(env.get("args") or {}),
        target=env.get("target"),
        source=env.get("source"),
    )


def build_cmd_response(
    *,
    request_id: str,
    ok: bool,
    code: str,
    data: dict[str, Any] | None = None,
    msg: str | None = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {"id": request_id, "ok": ok, "code": code}
    if data is not None:
        out["data"] = data
    if msg is not None:
        out["msg"] = msg
    return out
