"""ControllerGuard — exclusive motion-slot lock by `control_source` string."""
from __future__ import annotations


class ControllerBusy(RuntimeError):
    """Raised when a different caller holds the motion slot."""


class ControllerGuard:
    def __init__(self) -> None:
        self._holder: str | None = None

    @property
    def holder(self) -> str | None:
        return self._holder

    def acquire(self, caller: str) -> None:
        if self._holder is None or self._holder == caller:
            self._holder = caller
            return
        raise ControllerBusy(
            f"motion slot is held by {self._holder!r}; release first or use the same caller id"
        )

    def release(self, caller: str) -> None:
        if self._holder == caller:
            self._holder = None
