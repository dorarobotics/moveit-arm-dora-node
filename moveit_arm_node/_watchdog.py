"""HeartbeatWatchdog — fires `on_timeout` once when heartbeat lapses during motion.

Vendor-node-local copy. Ported byte-for-byte from the equivalent file in
agibot-a2-dora-node so behavior is consistent across spec-conforming nodes.
No external SDK imports; only stdlib.
"""
from __future__ import annotations

import time
from typing import Callable


class HeartbeatWatchdog:
    def __init__(
        self,
        timeout_s: float,
        on_timeout: Callable[[float], None],
    ) -> None:
        self._timeout_s = max(0.0, float(timeout_s))
        self._on_timeout = on_timeout
        self._last_heartbeat: float | None = None
        self._armed = False
        self._fired = False

    @property
    def enabled(self) -> bool:
        return self._timeout_s > 0.0

    def arm(self) -> None:
        self._armed = True
        self._last_heartbeat = time.monotonic()
        self._fired = False

    def disarm(self) -> None:
        self._armed = False

    def heartbeat(self) -> None:
        self._last_heartbeat = time.monotonic()
        self._fired = False

    def tick(self) -> None:
        if not self.enabled or not self._armed or self._fired:
            return
        if self._last_heartbeat is None:
            return
        if (time.monotonic() - self._last_heartbeat) > self._timeout_s:
            self._fired = True
            self._on_timeout(time.monotonic())
