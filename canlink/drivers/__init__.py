"""Pluggable CAN hardware backends.

Every driver implements the CanDriver interface in base.py:
    open(), close(), send(frame), recv(timeout) -> CanFrame | None, info()

Available drivers:
    usb_can_a    - Waveshare USB-CAN-A (classic CAN 2.0 only, serial protocol)
    simulator    - fake bus for testing the app with no hardware
    pythoncan    - thin wrapper over python-can (classic + CAN FD hardware)
"""
from .base import CanDriver, DriverError

__all__ = ["CanDriver", "DriverError", "create_driver", "list_serial_ports"]


def list_serial_ports() -> list[dict]:
    """Return available serial ports (for the Connect screen)."""
    try:
        import serial.tools.list_ports as lp
    except Exception:
        return []
    out = []
    for p in lp.comports():
        out.append({"device": p.device, "description": p.description or "", "hwid": p.hwid or ""})
    return out


def create_driver(cfg: dict) -> CanDriver:
    """Build a driver from a config dict (see storage.DEFAULT_CONFIG)."""
    name = cfg.get("driver", "simulator")
    if name == "usb_can_a":
        from .usb_can_a import UsbCanA
        return UsbCanA(
            port=cfg["serial_port"],
            bitrate=int(cfg.get("bitrate", 500000)),
            mode=cfg.get("mode", "normal"),
            frame_type=cfg.get("frame_type", "standard"),
            serial_baud=int(cfg.get("serial_baud", 2000000)),
        )
    if name == "simulator":
        from .simulator import Simulator
        return Simulator()
    if name == "pythoncan":
        from .pythoncan_fd import PythonCanDriver
        return PythonCanDriver(
            interface=cfg.get("pythoncan_interface", "socketcan"),
            channel=cfg.get("pythoncan_channel", "can0"),
            bitrate=int(cfg.get("bitrate", 500000)),
            fd=bool(cfg.get("fd", False)),
            data_bitrate=int(cfg.get("data_bitrate", 2000000)),
        )
    if name == "usb_can_fd":
        from .usb_can_fd import UsbCanFd
        return UsbCanFd(
            channel=int(cfg.get("fd_channel", 0)),
            bitrate=int(cfg.get("bitrate", 500000)),
            fd=bool(cfg.get("fd", True)),
            data_bitrate=int(cfg.get("data_bitrate", 2000000)),
            listen_only=bool(cfg.get("fd_listen_only", True)),
            resistor=bool(cfg.get("fd_resistor", False)),
            canfd_iso=bool(cfg.get("fd_iso", True)),
        )
    if name == "marathon":
        from .usb_can_marathon import UsbCanMarathon
        return UsbCanMarathon(
            channel=int(cfg.get("mar_channel", 0)),
            bitrate=int(cfg.get("bitrate", 500000)),
            listen_only=bool(cfg.get("mar_listen_only", True)),
        )
    raise DriverError(f"Unknown driver: {name!r}")
