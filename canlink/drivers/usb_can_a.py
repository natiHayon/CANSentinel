"""Driver for the Waveshare USB-CAN-A (a.k.a. the cheap "USB-CAN Analyzer",
same hardware as the Seeed Studio USB-CAN Analyzer).

IMPORTANT: this device is **classic CAN 2.0 only**. It has no CAN FD support
in hardware or firmware. Attempting to send an FD frame (or >8 data bytes)
raises DriverError. For CAN FD use the python-can backend with FD hardware.

Transport
---------
The adapter shows up as a CH340 USB-serial port. By default the USB side runs
at 2,000,000 baud (NOT the CAN bitrate - that is configured separately with a
settings packet). On open() we send a 20-byte settings packet to select the
CAN bitrate and mode, then data frames flow in the "variable length" protocol.

Variable-length data frame (both TX and RX):
    byte 0      : 0xAA                      start of frame
    byte 1      : type = 0xC0 | ext<<5 | rtr<<4 | dlc
    byte 2..    : identifier, little-endian (2 bytes std / 4 bytes ext)
    next 'dlc'  : data bytes
    last byte   : 0x55                      end of frame

20-byte settings packet:
    0xAA 0x55 0x12 <speed> <frame_type>
    <filter[4] LE> <mask[4] LE> <mode> <auto_retx> <reserved[4]> <checksum>
    checksum = sum(bytes[2..18]) & 0xFF

These constants match the publicly documented protocol. They are kept as
named tables below so they are easy to verify/adjust against your unit, and
the Live Monitor in the app lets you confirm frames are decoding correctly.
"""
from __future__ import annotations

import struct
from typing import Optional

from .base import CanDriver, DriverError
from ..frame import CanFrame

START = 0xAA
END = 0x55
CFG_MARK = 0x55          # second byte of a settings packet
CFG_TYPE = 0x12          # settings packet "type2" value (variable protocol)

# CAN bitrate -> speed code
SPEED_CODES = {
    1000000: 0x01,
    800000: 0x02,
    500000: 0x03,
    400000: 0x04,
    250000: 0x05,
    200000: 0x06,
    125000: 0x07,
    100000: 0x08,
    50000: 0x09,
    20000: 0x0A,
    10000: 0x0B,
    5000: 0x0C,
}

# Controller mode -> code
MODE_CODES = {
    "normal": 0x00,
    "loopback": 0x01,
    "silent": 0x02,          # listen-only: receives but never ACKs/transmits
    "silent_loopback": 0x03,
}

FRAME_TYPE_CODES = {
    "standard": 0x01,        # 11-bit identifiers
    "extended": 0x02,        # 29-bit identifiers
}


class UsbCanA(CanDriver):
    name = "usb_can_a"
    supports_fd = False

    def __init__(
        self,
        port: str,
        bitrate: int = 500000,
        mode: str = "normal",
        frame_type: str = "standard",
        serial_baud: int = 2000000,
    ):
        if not port:
            raise DriverError("No serial port selected for USB-CAN-A.")
        if bitrate not in SPEED_CODES:
            raise DriverError(
                f"Unsupported CAN bitrate {bitrate}. Supported: "
                + ", ".join(str(b) for b in SPEED_CODES)
            )
        if mode not in MODE_CODES:
            raise DriverError(f"Unknown mode {mode!r}.")
        if frame_type not in FRAME_TYPE_CODES:
            raise DriverError(f"Unknown frame_type {frame_type!r}.")
        self.port = port
        self.bitrate = bitrate
        self.mode = mode
        self.frame_type = frame_type
        self.serial_baud = serial_baud
        self._ser = None
        self._buf = bytearray()

    # ---- lifecycle -------------------------------------------------------
    def open(self) -> None:
        try:
            import serial
        except ImportError as e:
            raise DriverError(
                "pyserial is not installed. Run: pip install pyserial"
            ) from e
        try:
            self._ser = serial.Serial(
                self.port, baudrate=self.serial_baud, timeout=0.02, write_timeout=1.0
            )
        except Exception as e:  # serial.SerialException and friends
            raise DriverError(f"Could not open {self.port}: {e}") from e
        self._buf.clear()
        try:
            self._ser.reset_input_buffer()
            self._ser.write(self._settings_packet())
            self._ser.flush()
        except Exception as e:
            self.close()
            raise DriverError(f"Failed to configure adapter: {e}") from e

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _settings_packet(self) -> bytes:
        pkt = bytearray(20)
        pkt[0] = START
        pkt[1] = CFG_MARK
        pkt[2] = CFG_TYPE
        pkt[3] = SPEED_CODES[self.bitrate]
        pkt[4] = FRAME_TYPE_CODES[self.frame_type]
        # bytes 5..8 filter id, 9..12 mask id: 0/0 => accept everything
        pkt[13] = MODE_CODES[self.mode]
        pkt[14] = 0x01            # automatic retransmission on
        # bytes 15..18 reserved (0)
        pkt[19] = sum(pkt[2:19]) & 0xFF
        return bytes(pkt)

    # ---- transmit --------------------------------------------------------
    def send(self, frame: CanFrame) -> None:
        if self._ser is None:
            raise DriverError("Adapter not open.")
        if frame.is_fd:
            raise DriverError(
                "The Waveshare USB-CAN-A is classic CAN 2.0 only and cannot send "
                "CAN FD frames. Use the python-can backend with FD-capable hardware."
            )
        if frame.dlc > 8:
            raise DriverError("Classic CAN allows at most 8 data bytes.")
        if self.mode in ("silent", "silent_loopback"):
            raise DriverError(
                f"Adapter is in {self.mode!r} (listen-only) mode and will not "
                "transmit. Reconnect in 'normal' mode to send frames."
            )
        try:
            self._ser.write(self._encode(frame))
        except Exception as e:
            raise DriverError(f"Write failed: {e}") from e

    @staticmethod
    def _encode(frame: CanFrame) -> bytes:
        dlc = frame.dlc & 0x0F
        type_byte = 0xC0 | dlc
        if frame.is_extended:
            type_byte |= 0x20
        if frame.is_remote:
            type_byte |= 0x10
        out = bytearray([START, type_byte])
        if frame.is_extended:
            out += struct.pack("<I", frame.can_id & 0x1FFFFFFF)
        else:
            out += struct.pack("<H", frame.can_id & 0x7FF)
        out += frame.data
        out.append(END)
        return bytes(out)

    # ---- receive ---------------------------------------------------------
    def recv(self, timeout: float = 0.1) -> Optional[CanFrame]:
        if self._ser is None:
            return None
        frame = self._parse_buffer()
        if frame is not None:
            return frame
        try:
            n = self._ser.in_waiting or 1
            chunk = self._ser.read(n)
        except Exception as e:
            raise DriverError(f"Read failed: {e}") from e
        if chunk:
            self._buf += chunk
        return self._parse_buffer()

    def _parse_buffer(self) -> Optional[CanFrame]:
        buf = self._buf
        # Resync to a start byte.
        while buf and buf[0] != START:
            del buf[0]
        if len(buf) < 2:
            return None
        type_byte = buf[1]
        if (type_byte & 0xC0) != 0xC0:
            # Not a variable-length data frame; drop the start byte and resync.
            del buf[0]
            return None
        is_ext = bool(type_byte & 0x20)
        is_rtr = bool(type_byte & 0x10)
        dlc = type_byte & 0x0F
        id_len = 4 if is_ext else 2
        total = 2 + id_len + dlc + 1          # header + id + data + end marker
        if dlc > 8 or total > 2 + 4 + 8 + 1:
            del buf[0]
            return None
        if len(buf) < total:
            return None                        # wait for more bytes
        chunk = bytes(buf[:total])
        if chunk[-1] != END:
            del buf[0]                         # misaligned, resync
            return None
        if is_ext:
            can_id = struct.unpack("<I", chunk[2:6])[0]
            data = chunk[6:6 + dlc]
        else:
            can_id = struct.unpack("<H", chunk[2:4])[0]
            data = chunk[4:4 + dlc]
        del buf[:total]
        return CanFrame(
            can_id=can_id,
            data=bytes(data),
            is_extended=is_ext,
            is_remote=is_rtr,
            direction="rx",
        )

    def info(self) -> dict:
        return {
            "name": self.name,
            "supports_fd": False,
            "port": self.port,
            "bitrate": self.bitrate,
            "mode": self.mode,
            "frame_type": self.frame_type,
            "serial_baud": self.serial_baud,
        }
