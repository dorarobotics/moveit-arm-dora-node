#!/usr/bin/env python3
"""Mode B driver — emits one robot.heartbeat envelope per 2 s tick.

Sends the §7.1 envelope shape defined in SPEC-VENDOR-NODE-V1 to exercise the
vendor node end-to-end without vendor_sdk_legacy in the loop.
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone

import pyarrow as pa
from dora import Node


def main() -> None:
    node = Node()
    for event in node:
        if event["type"] != "INPUT":
            continue
        # Tick on any incoming event; this example wires no inputs, so this
        # body only fires for synthesised TICK events when running via dora cli.
        env = {
            "id": str(uuid.uuid4()),
            "verb": "robot.heartbeat",
            "args": {},
            "target": "ur5e-demo",
            "source": "cmd_driver",
            "ts": datetime.now(timezone.utc).isoformat(),
        }
        node.send_output("cmd_request", pa.array([json.dumps(env)]))


if __name__ == "__main__":
    main()
