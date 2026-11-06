"""The CanFrame value object shared by every driver and the UI.

A single class represents both classic CAN (<=8 data bytes) and CAN FD
(up to 64 data bytes, optional bit-rate switch). Drivers that cannot do FD
reject FD frames explicitly rather than silently truncating them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import time


# Valid CAN FD payload lengths (DLC -> byte count). Classic CAN is 0..8.
FD_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


@dataclass
class CanFrame:
    can_id: int
    data: bytes = b""
    is_extended: bool = False          # 29-bit identifier
    is_remote: bool = False            # remote-transmission-request (classic only)
    is_fd: bool = False                # CAN FD frame
    brs: bool = False                  # FD bit-rate switch (fast data phase)
    timestamp: float = field(default_factory=time.time)
    direction: str = "rx"              # "rx" or "tx"
    seq: int = 0                       # assigned by BusManager for incremental fetch
    channel: int | str | None = None   # actual interface channel; None means unknown
    source_seq: int | None = None      # original logger's record number, never renumbered
    timestamp_domain: str = "unix"     # unix | elapsed | unknown
    timestamp_source: str = "unspecified"
    source_timestamp: float | int | None = None
    source_time_unit: str | None = None
    format_known: bool = True

    @property
    def dlc(self) -> int:
        return len(self.data)

    @property
    def id_hex(self) -> str:
        width = 8 if self.is_extended else 3
        return f"0x{self.can_id:0{width}X}"

    @property
    def data_hex(self) -> str:
        return " ".join(f"{b:02X}" for b in self.data)

    def key(self) -> tuple[int, bool]:
        """Identity of the 'channel' this frame belongs to (id + ext flag)."""
        return (self.can_id, self.is_extended)

    def analysis_key(self) -> tuple:
        """Never combine distinct buses, frame formats or payload layouts offline."""
        return (self.can_id, self.is_extended, self.channel, self.is_fd,
                self.brs, self.dlc, self.is_remote, self.format_known)

    def to_dict(self) -> dict:
        return {
            "id": self.can_id,
            "id_hex": self.id_hex,
            "ext": self.is_extended,
            "rtr": self.is_remote,
            "fd": self.is_fd,
            "brs": self.brs,
            "dlc": self.dlc,
            "data": list(self.data),
            "data_hex": self.data_hex,
            "ts": round(self.timestamp, 4),
            "dir": self.direction,
            "seq": self.seq,
            "channel": self.channel,
            "source_seq": self.source_seq,
            "timestamp_domain": self.timestamp_domain,
            "timestamp_source": self.timestamp_source,
            "source_timestamp": self.source_timestamp,
            "source_time_unit": self.source_time_unit,
            "format_known": self.format_known,
        }

    @classmethod
    def from_hex(
        cls,
        can_id: int,
        data_hex: str,
        is_extended: bool = False,
        is_fd: bool = False,
        brs: bool = False,
    ) -> "CanFrame":
        cleaned = data_hex.replace(",", " ").replace("0x", " ").split()
        data = bytes(int(x, 16) for x in cleaned) if cleaned else b""
        return cls(
            can_id=can_id,
            data=data,
            is_extended=is_extended,
            is_fd=is_fd,
            brs=brs,
            direction="tx",
        )
