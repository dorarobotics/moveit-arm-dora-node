"""
Transport abstraction — supports dora, ROS2, or mixed deployments.

Transport: abstract base with request, publish, subscribe, action.
ActionHandle: abstract base with wait, cancel, feedback for long-running actions.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable, Dict, Iterator, List, Optional
import json
import socket as _socket
import time
import uuid

try:
    import yaml
except ImportError:
    yaml = None

try:
    from .tools import Tool, ToolResult, ToolRegistry
except (ImportError, TypeError):
    # Fallback for direct loading via importlib (avoids __init__.py on Python 3.8)
    import importlib.util as _ilu
    import os as _os
    _tools_path = _os.path.join(_os.path.dirname(__file__), "tools.py")
    _ts = _ilu.spec_from_file_location("octos_py.tools", _tools_path)
    _tm = _ilu.module_from_spec(_ts)
    _ts.loader.exec_module(_tm)
    Tool, ToolResult, ToolRegistry = _tm.Tool, _tm.ToolResult, _tm.ToolRegistry


class ActionHandle(ABC):
    """Abstract handle for a long-running action."""

    @abstractmethod
    def wait(self, timeout: float = 30.0) -> dict:
        ...

    @abstractmethod
    def cancel(self) -> None:
        ...

    @abstractmethod
    def feedback(self) -> Iterator[dict]:
        ...


class Transport(ABC):
    """Abstract transport — mirrors octos_agent::transport::Transport trait."""

    @abstractmethod
    def request(self, tool_name: str, args: dict, timeout: float = 30.0) -> dict:
        """Send a request and wait for a response (tool calls, ROS2 services)."""
        ...

    @abstractmethod
    def publish(self, channel: str, data: dict) -> None:
        """Publish data to a channel (fire-and-forget)."""
        ...

    @abstractmethod
    def subscribe(self, channel: str, callback: Callable[[dict], None]) -> None:
        """Subscribe to a channel with a callback."""
        ...

    @abstractmethod
    def action(self, tool_name: str, args: dict) -> ActionHandle:
        """Start a long-running action and return a handle."""
        ...


class DoraActionHandle(ActionHandle):
    """ActionHandle that wraps a blocking request (dora has no native actions)."""

    def __init__(self, result: dict):
        self._result = result

    def wait(self, timeout: float = 30.0) -> dict:
        return self._result

    def cancel(self) -> None:
        pass

    def feedback(self) -> Iterator[dict]:
        return iter([])


class DoraTransport(Transport):
    """Transport backed by a dora Node -- wraps send_output/next pattern."""

    def __init__(self, node):
        self._node = node

    @staticmethod
    def _to_pa_uint8(raw: bytes):
        import pyarrow as pa
        return pa.array(list(raw), type=pa.uint8())

    def request(self, tool_name: str, args: dict, timeout: float = 30.0) -> dict:
        request_data = {"tool": tool_name, "args": args}
        raw = json.dumps(request_data).encode("utf-8")
        self._node.send_output("skill_request", self._to_pa_uint8(raw))
        deadline = time.time() + timeout
        while time.time() < deadline:
            event = self._node.next(timeout=1.0)
            if event is None:
                continue
            if event["type"] == "INPUT" and event["id"] == "skill_result":
                raw_resp = bytes(event["value"].to_pylist())
                return json.loads(raw_resp.decode("utf-8"))
        return {"error": "timeout"}

    def publish(self, channel: str, data: dict) -> None:
        raw = json.dumps(data).encode("utf-8")
        self._node.send_output(channel, self._to_pa_uint8(raw))

    def subscribe(self, channel: str, callback: Callable[[dict], None]) -> None:
        raise NotImplementedError(
            "DoraTransport.subscribe() not supported -- "
            "dora subscriptions are handled via the dataflow event loop"
        )

    def action(self, tool_name: str, args: dict) -> ActionHandle:
        result = self.request(tool_name, args)
        return DoraActionHandle(result)


# ---------------------------------------------------------------------------
# ROS2 bridge transport (Unix socket + JSON, one-shot pattern)
# ---------------------------------------------------------------------------

def _send_socket_request(sock_path: str, request: dict, timeout: float = 30.0) -> dict:
    """Send a JSON request over a Unix socket and return the JSON response."""
    try:
        sock = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(sock_path)
    except (FileNotFoundError, ConnectionRefusedError, OSError) as e:
        raise ConnectionError(
            "Cannot connect to ROS2 bridge at {}: {}".format(sock_path, e)
        )
    try:
        sock.sendall(json.dumps(request).encode("utf-8"))
        sock.shutdown(_socket.SHUT_WR)  # signal end of request
        chunks = []
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
        return json.loads(b"".join(chunks).decode("utf-8"))
    finally:
        sock.close()


class Ros2ActionHandle(ActionHandle):
    """ActionHandle backed by a ROS2 bridge action call over Unix socket."""

    def __init__(self, sock_path, request_id, endpoint, args):
        self._sock_path = sock_path
        self._request_id = request_id
        self._endpoint = endpoint
        self._args = args
        self._result = None

    def wait(self, timeout: float = 30.0) -> dict:
        if self._result is not None:
            return self._result
        resp = _send_socket_request(
            self._sock_path,
            {
                "id": self._request_id,
                "pattern": "action",
                "endpoint": self._endpoint,
                "args": self._args,
            },
            timeout=timeout,
        )
        self._result = resp
        return resp

    def cancel(self) -> None:
        _send_socket_request(
            self._sock_path,
            {"id": self._request_id, "pattern": "cancel"},
            timeout=5.0,
        )

    def feedback(self) -> Iterator[dict]:
        return iter([])


class Ros2Transport(Transport):
    """Transport backed by an external ROS2 bridge process via Unix socket."""

    def __init__(self, socket_path: str):
        self._socket_path = socket_path

    def request(self, tool_name: str, args: dict, timeout: float = 30.0) -> dict:
        request_id = "req-{}".format(uuid.uuid4().hex[:8])
        return _send_socket_request(
            self._socket_path,
            {
                "id": request_id,
                "pattern": "service",
                "endpoint": tool_name,
                "args": args,
            },
            timeout=timeout,
        )

    def publish(self, channel: str, data: dict) -> None:
        request_id = "pub-{}".format(uuid.uuid4().hex[:8])
        try:
            _send_socket_request(
                self._socket_path,
                {
                    "id": request_id,
                    "pattern": "publish",
                    "endpoint": channel,
                    "args": data,
                },
                timeout=5.0,
            )
        except ConnectionError:
            pass  # fire-and-forget

    def subscribe(self, channel: str, callback: Callable[[dict], None]) -> None:
        raise NotImplementedError(
            "Ros2Transport.subscribe() requires a persistent connection. "
            "Use the ROS2 bridge's push mechanism instead."
        )

    def action(self, tool_name: str, args: dict) -> ActionHandle:
        request_id = "act-{}".format(uuid.uuid4().hex[:8])
        return Ros2ActionHandle(
            self._socket_path, request_id, tool_name, args
        )


# ---------------------------------------------------------------------------
# TransportBridgeTool — generic Tool that delegates execute() to a Transport
# ---------------------------------------------------------------------------

class TransportBridgeTool(Tool):
    """Generic tool that delegates execute() to a Transport."""

    def __init__(
        self,
        tool_name: str,
        tool_description: str,
        transport: Transport,
        pattern: str = "request",
        endpoint: Optional[str] = None,
        schema: Optional[Dict] = None,
        safety_tier: str = "observe",
    ):
        self._name = tool_name
        self._description = tool_description
        self._transport = transport
        self._pattern = pattern
        self._endpoint = endpoint or tool_name
        self._schema = schema or {"type": "object", "properties": {}}
        self._safety_tier = safety_tier

    def name(self) -> str:
        return self._name

    def description(self) -> str:
        return self._description

    def input_schema(self) -> dict:
        return self._schema

    def tags(self) -> List[str]:
        return ["transport"]

    def required_safety_tier(self) -> str:
        return self._safety_tier

    def execute(self, args: dict) -> ToolResult:
        if self._pattern == "action":
            handle = self._transport.action(self._endpoint, args)
            result = handle.wait()
        elif self._pattern == "publish":
            self._transport.publish(self._endpoint, args)
            result = {"status": "published"}
        else:  # "request" or "service"
            result = self._transport.request(self._endpoint, args)
        return ToolResult(output=json.dumps(result, default=str))


# ---------------------------------------------------------------------------
# TransportRouter — reads YAML config, creates TransportBridgeTools
# ---------------------------------------------------------------------------

class TransportRouter:
    """Reads a YAML config and creates TransportBridgeTools routed to the correct transport."""

    def __init__(self, config_path: str, transports: Dict[str, Transport]):
        if yaml is None:
            raise ImportError("PyYAML required: pip install pyyaml")
        with open(config_path) as f:
            config = yaml.safe_load(f)
        self.registry = ToolRegistry()
        for tool_name, tool_cfg in config.get("tools", {}).items():
            transport_key = tool_cfg["transport"]
            if transport_key not in transports:
                raise KeyError(
                    "Tool '{}' references transport '{}' but available: {}".format(
                        tool_name, transport_key, list(transports.keys())
                    )
                )
            tool = TransportBridgeTool(
                tool_name=tool_name,
                tool_description=tool_cfg.get("description", tool_name),
                transport=transports[transport_key],
                pattern=tool_cfg.get("pattern", "request"),
                endpoint=tool_cfg.get("endpoint"),
                schema=tool_cfg.get("schema"),
                safety_tier=tool_cfg.get("safety_tier", "observe"),
            )
            self.registry.register(tool)
