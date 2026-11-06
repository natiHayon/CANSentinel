"""Driver for the Marathon CAN-bus-USBnp (and other Marathon CAN adapters).

These are classic CAN 2.0 (SJA1000-based) devices accessed through Marathon's
CHAI library (chai.dll), wrapped here with ctypes. Install CHAI 2.14.x from
Marathon first - it provides the driver and chai.dll (into System32).

API sequence (from chai.h):
    CiInit() once -> CiOpen(chan, CIO_CAN11|CIO_CAN29) -> CiSetBaud(chan,bt0,bt1)
    -> CiSetFilter(chan,0,0xFFFFFFFF) -> [CiSetLom for listen-only] -> CiStart
    -> CiRead / CiTransmit -> CiStop -> CiClose

Channels: a CAN-bus-USBnp exposes two CAN ports as channel 0 and channel 1.
This device is classic CAN only (no CAN FD).
"""
from __future__ import annotations

import ctypes as C
import os
import time
from collections import deque
from typing import Optional

from .base import CanDriver, DriverError
from ._ts import TimestampAnchor
from ..frame import CanFrame

# CiOpen flags
CIO_CAN11 = 0x2
CIO_CAN29 = 0x4
# canmsg_t.flags bits
FRAME_RTR = 0x1
FRAME_EFF = 0x4
# listen-only
CI_LOM_ON = 0x1

# bitrate -> SJA1000 (BTR0, BTR1) @ 16 MHz, from chai.h BCI_* constants
BTR = {
    1000000: (0x00, 0x14),
    800000: (0x00, 0x16),
    500000: (0x00, 0x1C),
    250000: (0x01, 0x1C),
    125000: (0x03, 0x1C),
    100000: (0x04, 0x1C),
    50000: (0x09, 0x1C),
    20000: (0x18, 0x1C),
    10000: (0x31, 0x1C),
}


class canmsg_t(C.Structure):
    # Natural alignment (NOT packed): id, data[8], len, pad, flags, ts  -> 20 bytes
    _fields_ = [
        ("id", C.c_uint32),
        ("data", C.c_uint8 * 8),
        ("len", C.c_uint8),
        ("flags", C.c_uint16),
        ("ts", C.c_uint32),
    ]


_lib = None
_inited = False


def _load_lib():
    global _lib
    if _lib is not None:
        return _lib
    candidates = [
        "chai",  # System32 (on default search path after CHAI install)
        r"C:\Program Files (x86)\CHAI-2.14.0\x64\chai.dll",
        r"C:\Program Files (x86)\CHAI-2.14.0\x32\chai.dll",
    ]
    lib = None
    last = None
    for name in candidates:
        try:
            lib = C.CDLL(name)   # CHAI functions are __cdecl
            break
        except OSError as e:
            last = e
    if lib is None:
        raise DriverError(f"Could not load chai.dll (is CHAI installed?). {last}")
    s16, u8, u16, u32 = C.c_short, C.c_ubyte, C.c_uint16, C.c_uint32
    lib.CiInit.restype = s16; lib.CiInit.argtypes = []
    lib.CiOpen.restype = s16; lib.CiOpen.argtypes = [u8, u8]
    lib.CiClose.restype = s16; lib.CiClose.argtypes = [u8]
    lib.CiStart.restype = s16; lib.CiStart.argtypes = [u8]
    lib.CiStop.restype = s16; lib.CiStop.argtypes = [u8]
    lib.CiSetBaud.restype = s16; lib.CiSetBaud.argtypes = [u8, u8, u8]
    lib.CiSetFilter.restype = s16; lib.CiSetFilter.argtypes = [u8, u32, u32]
    lib.CiSetLom.restype = s16; lib.CiSetLom.argtypes = [u8, u8]
    lib.CiRead.restype = s16; lib.CiRead.argtypes = [u8, C.POINTER(canmsg_t), C.c_short]
    lib.CiTransmit.restype = s16; lib.CiTransmit.argtypes = [u8, C.POINTER(canmsg_t)]
    try:
        lib.CiStrError.restype = None
        lib.CiStrError.argtypes = [s16, C.c_char_p, s16]
    except Exception:
        pass
    _lib = lib
    return lib


def _strerr(ret: int) -> str:
    try:
        buf = C.create_string_buffer(128)
        _lib.CiStrError(abs(ret), buf, 128)
        s = buf.value.decode("ascii", "ignore").strip()
        return s or f"error {ret}"
    except Exception:
        return f"error {ret}"


class UsbCanMarathon(CanDriver):
    name = "marathon"
    supports_fd = False

    def __init__(self, channel: int = 0, bitrate: int = 500000, listen_only: bool = True):
        if bitrate not in BTR:
            raise DriverError(f"Unsupported bitrate {bitrate}. Supported: " + ", ".join(map(str, BTR)))
        self.channel = int(channel)
        self.bitrate = int(bitrate)
        self.listen_only = bool(listen_only)
        self._open = False
        self._buf: deque[CanFrame] = deque()
        self._ts = TimestampAnchor(scale=1e-6)   # CHAI canmsg timer (assumed microseconds)
        self._rx_cap_hits = 0                    # times CiRead returned a full batch (falling behind)

    def open(self) -> None:
        global _inited
        lib = _load_lib()
        if not _inited:
            r = lib.CiInit()
            if r < 0:
                raise DriverError(f"CiInit failed: {_strerr(r)}")
            _inited = True
        ch = self.channel
        r = lib.CiOpen(ch, CIO_CAN11 | CIO_CAN29)
        if r < 0:
            raise DriverError(
                f"CiOpen(channel {ch}) failed: {_strerr(r)}. Check the adapter is plugged in, "
                "the CHAI driver bound to it (Device Manager shows CAN-bus-USBnp as OK, not Error), "
                "and the channel/port is correct."
            )
        self._open = True
        bt0, bt1 = BTR[self.bitrate]
        for fn, args, label in [
            (lib.CiSetBaud, (ch, bt0, bt1), "CiSetBaud"),
            (lib.CiSetFilter, (ch, 0, 0xFFFFFFFF), "CiSetFilter"),
        ]:
            r = fn(*args)
            if r < 0:
                self.close()
                raise DriverError(f"{label} failed on channel {ch}: {_strerr(r)}")
        if self.listen_only:
            # A silent fallback to normal (ACKing) mode would break the read-only guarantee
            # and let the adapter transmit/ACK on the vehicle bus. Treat failure as fatal.
            try:
                r = lib.CiSetLom(ch, CI_LOM_ON)
            except Exception as e:
                self.close()
                raise DriverError(
                    f"Listen-only was requested but CiSetLom raised ({e}); refusing to start, "
                    "because the adapter would otherwise ACK/transmit on the bus.")
            if r < 0:
                self.close()
                raise DriverError(
                    f"Listen-only was requested but CiSetLom failed ({_strerr(r)}); refusing to "
                    "start, because the adapter would otherwise ACK/transmit on the bus.")
        self._ts.reset()
        r = lib.CiStart(ch)
        if r < 0:
            self.close()
            raise DriverError(f"CiStart failed on channel {ch}: {_strerr(r)}")

    def close(self) -> None:
        if _lib and self._open:
            try:
                _lib.CiStop(self.channel)
            except Exception:
                pass
            try:
                _lib.CiClose(self.channel)
            except Exception:
                pass
        self._open = False

    def recv(self, timeout: float = 0.1) -> Optional[CanFrame]:
        if self._buf:
            return self._buf.popleft()
        deadline = time.time() + timeout
        batch = (canmsg_t * 64)()
        while True:
            got = _lib.CiRead(self.channel, batch, 64)
            if got and got > 0:
                if got >= 64:                 # full batch -> we may be falling behind
                    self._rx_cap_hits += 1
                for i in range(min(got, 64)):
                    m = batch[i]
                    self._buf.append(CanFrame(
                        can_id=m.id & 0x1FFFFFFF,
                        data=bytes(bytearray(m.data[:min(m.len, 8)])),
                        is_extended=bool(m.flags & FRAME_EFF),
                        is_remote=bool(m.flags & FRAME_RTR),
                        is_fd=False,
                        timestamp=self._ts.stamp(int(m.ts)),
                        direction="rx",
                    ))
                return self._buf.popleft()
            if time.time() >= deadline:
                return None
            time.sleep(0.002)

    def send(self, frame: CanFrame) -> None:
        if not self._open:
            raise DriverError("Adapter not open.")
        if frame.is_fd:
            raise DriverError("The Marathon CAN-bus-USBnp is classic CAN only (no CAN FD).")
        if frame.dlc > 8:
            raise DriverError("Classic CAN allows at most 8 data bytes.")
        if self.listen_only:
            raise DriverError("Adapter is in listen-only mode and will not transmit. "
                              "Reconnect with listen-only off to send frames.")
        m = canmsg_t()
        m.id = frame.can_id & 0x1FFFFFFF
        m.len = frame.dlc
        m.flags = (FRAME_EFF if frame.is_extended else 0) | (FRAME_RTR if frame.is_remote else 0)
        for i in range(frame.dlc):
            m.data[i] = frame.data[i]
        r = _lib.CiTransmit(self.channel, C.byref(m))
        if r < 0:
            raise DriverError(f"CiTransmit failed: {_strerr(r)}")

    def info(self) -> dict:
        return {"name": self.name, "supports_fd": False, "channel": self.channel,
                "bitrate": self.bitrate, "listen_only": self.listen_only,
                "rx_cap_hits": self._rx_cap_hits, "hw_timestamps": True}
