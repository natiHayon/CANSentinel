"""JSON persistence for app config and learned button mappings."""
from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(_HERE, "config.json")
MAPPINGS_PATH = os.path.join(_HERE, "mappings.json")

_lock = threading.Lock()

DEFAULT_CONFIG = {
    "driver": "simulator",            # simulator | usb_can_a | pythoncan
    "serial_port": "",
    "serial_baud": 2000000,
    "bitrate": 500000,
    "mode": "normal",                 # normal | silent | loopback | silent_loopback
    "frame_type": "standard",         # standard | extended
    "fd": False,
    "data_bitrate": 2000000,
    "pythoncan_interface": "socketcan",
    "pythoncan_channel": "can0",
    # Waveshare USB-CAN-FD (ZCAN SDK) options
    "fd_channel": 1,                  # adapter channel index; the car's "CAN2" = ch 1
    "fd_listen_only": True,           # safe by default (no ACK/TX while learning)
    "fd_resistor": False,             # internal 120 ohm terminator (off when tapping a terminated bus)
    "fd_iso": True,                   # ISO CAN FD (vs non-ISO)
    # Marathon CAN-bus-USBnp (CHAI SDK) options
    "mar_channel": 0,                 # adapter CAN port (0 or 1)
    "mar_listen_only": True,
    "transmit_enabled": False,        # safety gate: no TX until the user opts in
}


def _read(path: str, default):
    if not os.path.exists(path):
        return default
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def _write(path: str, obj) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
    os.replace(tmp, path)


def load_config() -> dict:
    with _lock:
        cfg = dict(DEFAULT_CONFIG)
        cfg.update(_read(CONFIG_PATH, {}))
        return cfg


def save_config(cfg: dict) -> dict:
    with _lock:
        merged = dict(DEFAULT_CONFIG)
        merged.update(_read(CONFIG_PATH, {}))
        merged.update(cfg or {})
        _write(CONFIG_PATH, merged)
        return merged


def load_mappings() -> list[dict]:
    with _lock:
        data = _read(MAPPINGS_PATH, [])
        return data if isinstance(data, list) else []


def save_mappings(mappings: list[dict]) -> None:
    with _lock:
        _write(MAPPINGS_PATH, mappings)


def add_mapping(mapping: dict) -> list[dict]:
    with _lock:
        data = _read(MAPPINGS_PATH, [])
        if not isinstance(data, list):
            data = []
        mapping = dict(mapping)
        mapping.setdefault("id", (max((m.get("id", 0) for m in data), default=0) + 1))
        mapping.setdefault("created", datetime.now(timezone.utc).isoformat(timespec="seconds"))
        data.append(mapping)
        _write(MAPPINGS_PATH, data)
        return data


def delete_mapping(mapping_id: int) -> list[dict]:
    with _lock:
        data = _read(MAPPINGS_PATH, [])
        if not isinstance(data, list):
            data = []
        data = [m for m in data if m.get("id") != mapping_id]
        _write(MAPPINGS_PATH, data)
        return data
