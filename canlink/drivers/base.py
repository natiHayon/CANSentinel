"""Abstract driver interface."""
from __future__ import annotations

import abc
from typing import Optional

from ..frame import CanFrame


class DriverError(Exception):
    """Raised for driver open/send/config problems with a human-readable message."""


class CanDriver(abc.ABC):
    name: str = "base"
    supports_fd: bool = False

    @abc.abstractmethod
    def open(self) -> None:
        ...

    @abc.abstractmethod
    def close(self) -> None:
        ...

    @abc.abstractmethod
    def send(self, frame: CanFrame) -> None:
        ...

    @abc.abstractmethod
    def recv(self, timeout: float = 0.1) -> Optional[CanFrame]:
        """Return one frame or None if nothing arrived within `timeout` seconds."""
        ...

    def info(self) -> dict:
        return {"name": self.name, "supports_fd": self.supports_fd}
