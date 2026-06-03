#!/usr/bin/env python3
"""Mode B sink — print everything the vendor node emits."""
from __future__ import annotations

from dora import Node


def main() -> None:
    node = Node()
    for event in node:
        if event["type"] != "INPUT":
            continue
        topic = event["id"]
        try:
            text = event["value"][0].as_py()
        except Exception:
            text = str(event["value"])
        print(f"[{topic}] {text}", flush=True)


if __name__ == "__main__":
    main()
