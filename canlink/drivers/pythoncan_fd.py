"""Optional backend built on python-can. This is how the app talks to CAN FD
hardware (and any other interface python-can supports: PCAN-USB FD, Kvaser,
Vector, CANable 2.0 via gs_usb, SocketCAN, slcan, ...).

Install with:  pip install "python-can>=4.3"

Pick interface/channel to match your hardware, e.g.:
    SocketCAN (Linux):      interface="socketcan"  channel="can0"
    PEAK PCAN-USB FD:       interface="pcan"       channel="PCAN_USBBUS1"
    Kvaser:                 interface="kvaser"     channel="0"
    CANable 2.0 (gs_usb):   interface="gs_usb"     channel="0"
    slcan serial:           interface="slcan"      channel="COM5"
"""
from __future__ import annotations

from typing import Optional

from .base import CanDriver, DriverError
from ..frame import CanFrame


class PythonCanDriver(CanDriver):
    name = "pythoncan"
    supports_fd = True

    def __init__(
        self,
        interface: str = "socketcan",
        channel: str = "can0",
        bitrate: int = 500000,
        fd: bool = False,
        data_bitrate: int = 2000000,
    ):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.fd = fd
        self.data_bitrate = data_bitrate
        self._bus = None
        self._can = None

    def open(self) -> None:
        try:
            import can
        except ImportError as e:
            raise DriverError(
                'python-can is not installed. Run: pip install "python-can>=4.3"'
            ) from e
        self._can = can
        kwargs = dict(interface=self.interface, channel=self.channel, bitrate=self.bitrate)
        if self.fd:
            kwargs["fd"] = True
            kwargs["data_bitrate"] = self.data_bitrate
        try:
            self._bus = can.Bus(**kwargs)
        except Exception as e:
            raise DriverError(f"python-can could not open {self.interface}:{self.channel}: {e}") from e

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
            self._bus = None

    def send(self, frame: CanFrame) -> None:
        if self._bus is None:
            raise DriverError("Bus not open.")
        if frame.is_fd and not self.fd:
            raise DriverError("This bus was opened in classic mode; reconnect with FD enabled to send FD frames.")
        msg = self._can.Message(
            arbitration_id=frame.can_id,
            is_extended_id=frame.is_extended,
            is_remote_frame=frame.is_remote,
            is_fd=frame.is_fd,
            bitrate_switch=frame.brs,
            data=frame.data,
        )
        try:
            self._bus.send(msg, timeout=1.0)
        except Exception as e:
            raise DriverError(f"Send failed: {e}") from e

    def recv(self, timeout: float = 0.1) -> Optional[CanFrame]:
        if self._bus is None:
            return None
        msg = self._bus.recv(timeout=timeout)
        if msg is None:
            return None
        return CanFrame(
            can_id=msg.arbitration_id,
            data=bytes(msg.data),
            is_extended=bool(msg.is_extended_id),
            is_remote=bool(msg.is_remote_frame),
            is_fd=bool(getattr(msg, "is_fd", False)),
            brs=bool(getattr(msg, "bitrate_switch", False)),
            timestamp=msg.timestamp or 0.0,
            direction="rx",
        )

    def info(self) -> dict:
        return {
            "name": self.name,
            "supports_fd": True,
            "interface": self.interface,
            "channel": self.channel,
            "bitrate": self.bitrate,
            "fd": self.fd,
            "data_bitrate": self.data_bitrate,
        }
