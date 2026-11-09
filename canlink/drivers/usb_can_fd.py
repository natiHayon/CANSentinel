"""Driver for the Waveshare USB-CAN-FD / USB-CAN-FD-B.

Unlike the USB-CAN-A (a serial/COM device), this model is a WinUSB device that
speaks a proprietary ZLG/ZCAN protocol. We talk to it through Waveshare's own
SDK DLL (ControlCANFD.dll), wrapped here with ctypes. This gives native CAN and
CAN FD, on either of the adapter's two channels.

SDK files live in <project>/sdk/{x64,x86}/ControlCANFD.dll and come from
Waveshare's "USB-CAN-FD Secondary Development Library". The struct layouts and
call sequence below follow Waveshare's own Python demo (python3.8.0.py) and the
ControlCANFD.h header exactly.

Key behaviours:
  * Classic and FD frames arrive in SEPARATE receive queues -> we drain both.
  * Baud is set per channel BEFORE InitCAN (arbitration + data bitrate).
  * listen_only maps to the channel mode flag (safe, no ACK/TX while learning).
  * The internal 120 ohm terminator is OFF by default (you are usually tapping a
    bus that is already terminated).
"""
from __future__ import annotations

import ctypes as C
import os
import time
from collections import deque
from typing import Optional

from .base import CanDriver, DriverError
from ._ts import TimestampAnchor
from ..frame import CanFrame, FD_LENGTHS

# ---- SDK constants ----
ZCAN_USBCANFD_200U = 41
STATUS_OK = 1
TYPE_CAN = 0
TYPE_CANFD = 1

# valid FD payload lengths (for padding transmit data)
_FD_LENS = FD_LENGTHS


# ---- ctypes structs (match Waveshare python3.8.0.py / ControlCANFD.h) ----
class _CAN_INIT(C.Structure):
    _fields_ = [("acc_code", C.c_uint), ("acc_mask", C.c_uint), ("reserved", C.c_uint),
                ("filter", C.c_ubyte), ("timing0", C.c_ubyte), ("timing1", C.c_ubyte),
                ("mode", C.c_ubyte)]


class _CANFD_INIT(C.Structure):
    _fields_ = [("acc_code", C.c_uint), ("acc_mask", C.c_uint), ("abit_timing", C.c_uint),
                ("dbit_timing", C.c_uint), ("brp", C.c_uint), ("filter", C.c_ubyte),
                ("mode", C.c_ubyte), ("pad", C.c_ushort), ("reserved", C.c_uint)]


class _INIT_UNION(C.Union):
    _fields_ = [("can", _CAN_INIT), ("canfd", _CANFD_INIT)]


class ZCAN_CHANNEL_INIT_CONFIG(C.Structure):
    _fields_ = [("can_type", C.c_uint), ("config", _INIT_UNION)]


class ZCAN_CAN_FRAME(C.Structure):
    _fields_ = [("can_id", C.c_uint, 29), ("err", C.c_uint, 1), ("rtr", C.c_uint, 1),
                ("eff", C.c_uint, 1), ("can_dlc", C.c_ubyte), ("__pad", C.c_ubyte),
                ("__res0", C.c_ubyte), ("__res1", C.c_ubyte), ("data", C.c_ubyte * 8)]


class ZCAN_CANFD_FRAME(C.Structure):
    _fields_ = [("can_id", C.c_uint, 29), ("err", C.c_uint, 1), ("rtr", C.c_uint, 1),
                ("eff", C.c_uint, 1), ("len", C.c_ubyte), ("brs", C.c_ubyte, 1),
                ("esi", C.c_ubyte, 1), ("__res", C.c_ubyte, 6), ("__res0", C.c_ubyte),
                ("__res1", C.c_ubyte), ("data", C.c_ubyte * 64)]


class ZCAN_Transmit_Data(C.Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", C.c_uint)]


class ZCAN_Receive_Data(C.Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", C.c_ulonglong)]


class ZCAN_TransmitFD_Data(C.Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", C.c_uint)]


class ZCAN_ReceiveFD_Data(C.Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", C.c_ulonglong)]


def _project_root() -> str:
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _dll_path() -> str:
    arch = "x64" if C.sizeof(C.c_void_p) == 8 else "x86"
    return os.path.join(_project_root(), "sdk", arch, "ControlCANFD.dll")


class UsbCanFd(CanDriver):
    name = "usb_can_fd"
    supports_fd = True

    def __init__(self, channel: int = 0, bitrate: int = 500000, fd: bool = True,
                 data_bitrate: int = 2000000, listen_only: bool = True,
                 resistor: bool = False, canfd_iso: bool = True,
                 device_type: int = ZCAN_USBCANFD_200U, device_index: int = 0):
        self.channel = int(channel)
        self.bitrate = int(bitrate)
        self.fd = bool(fd)
        self.data_bitrate = int(data_bitrate)
        self.listen_only = bool(listen_only)
        self.resistor = bool(resistor)
        self.canfd_iso = bool(canfd_iso)
        self.device_type = int(device_type)
        self.device_index = int(device_index)
        self._dll = None
        self._dev = None
        self._ch = None
        self._buf: deque[CanFrame] = deque()
        self._ts = TimestampAnchor(scale=1e-6)   # ZCAN 200U hardware timer is microseconds
        self._rx_backlog_max = 0                 # deepest queue seen (overflow signal)
        self._rx_cap_hits = 0                    # times a drain was capped (falling behind)

    _RX_CAP = 500                                # max frames pulled from each queue per drain

    # ---- DLL loading ----
    def _load(self):
        path = _dll_path()
        if not os.path.exists(path):
            raise DriverError(f"SDK DLL not found at {path}. Is the sdk/ folder present?")
        try:
            os.add_dll_directory(os.path.dirname(path))
        except Exception:
            pass
        try:
            dll = C.WinDLL(path)
        except OSError as e:
            raise DriverError(f"Failed to load ControlCANFD.dll: {e}") from e
        dll.ZCAN_OpenDevice.argtypes = (C.c_uint, C.c_uint, C.c_uint)
        dll.ZCAN_OpenDevice.restype = C.c_void_p
        dll.ZCAN_CloseDevice.argtypes = (C.c_void_p,)
        dll.ZCAN_SetAbitBaud.argtypes = (C.c_void_p, C.c_ulong, C.c_ulong)
        dll.ZCAN_SetDbitBaud.argtypes = (C.c_void_p, C.c_ulong, C.c_ulong)
        dll.ZCAN_SetCANFDStandard.argtypes = (C.c_void_p, C.c_ulong, C.c_ulong)
        dll.ZCAN_SetResistanceEnable.argtypes = (C.c_void_p, C.c_ulong, C.c_ulong)
        dll.ZCAN_InitCAN.argtypes = (C.c_void_p, C.c_ulong, C.c_void_p)
        dll.ZCAN_InitCAN.restype = C.c_void_p
        dll.ZCAN_StartCAN.argtypes = (C.c_void_p,)
        dll.ZCAN_ResetCAN.argtypes = (C.c_void_p,)
        dll.ZCAN_GetReceiveNum.argtypes = (C.c_void_p, C.c_ubyte)
        dll.ZCAN_GetReceiveNum.restype = C.c_uint
        dll.ZCAN_Receive.argtypes = (C.c_void_p, C.c_void_p, C.c_ulong, C.c_int)
        dll.ZCAN_Receive.restype = C.c_uint
        dll.ZCAN_ReceiveFD.argtypes = (C.c_void_p, C.c_void_p, C.c_ulong, C.c_int)
        dll.ZCAN_ReceiveFD.restype = C.c_uint
        dll.ZCAN_Transmit.argtypes = (C.c_void_p, C.c_void_p, C.c_ulong)
        dll.ZCAN_Transmit.restype = C.c_uint
        dll.ZCAN_TransmitFD.argtypes = (C.c_void_p, C.c_void_p, C.c_ulong)
        dll.ZCAN_TransmitFD.restype = C.c_uint
        self._dll = dll

    # ---- lifecycle ----
    def open(self) -> None:
        self._load()
        dll = self._dll
        dev = dll.ZCAN_OpenDevice(self.device_type, self.device_index, 0)
        if not dev:
            raise DriverError(
                "ZCAN_OpenDevice failed. Check the adapter is plugged in, the Waveshare "
                "driver is installed (device shows as 'WinUSB Device'), and no other program "
                "(e.g. the CANFD Tool) has it open."
            )
        self._dev = dev
        ch = self.channel

        if dll.ZCAN_SetAbitBaud(dev, ch, self.bitrate) != STATUS_OK:
            self.close()
            raise DriverError(f"Set arbitration bitrate {self.bitrate} failed on channel {ch}.")
        if self.fd:
            if dll.ZCAN_SetDbitBaud(dev, ch, self.data_bitrate) != STATUS_OK:
                self.close()
                raise DriverError(f"Set data bitrate {self.data_bitrate} failed on channel {ch}.")
            dll.ZCAN_SetCANFDStandard(dev, ch, 0 if self.canfd_iso else 1)
        try:
            dll.ZCAN_SetResistanceEnable(dev, ch, 1 if self.resistor else 0)
        except Exception:
            pass  # not fatal if unsupported

        cfg = ZCAN_CHANNEL_INIT_CONFIG()
        mode = 1 if self.listen_only else 0      # 0 normal, 1 listen-only
        if self.fd:
            cfg.can_type = TYPE_CANFD
            cfg.config.canfd.mode = mode
            cfg.config.canfd.acc_code = 0
            cfg.config.canfd.acc_mask = 0xFFFFFFFF
            cfg.config.canfd.filter = 0
        else:
            cfg.can_type = TYPE_CAN
            cfg.config.can.mode = mode
            cfg.config.can.acc_code = 0
            cfg.config.can.acc_mask = 0xFFFFFFFF
            cfg.config.can.filter = 0

        chh = dll.ZCAN_InitCAN(dev, ch, C.byref(cfg))
        if not chh:
            self.close()
            raise DriverError(f"ZCAN_InitCAN failed on channel {ch}.")
        self._ch = chh
        self._ts.reset()
        if dll.ZCAN_StartCAN(chh) != STATUS_OK:
            self.close()
            raise DriverError(f"ZCAN_StartCAN failed on channel {ch}.")

    def close(self) -> None:
        try:
            if self._dll and self._ch:
                self._dll.ZCAN_ResetCAN(self._ch)
        except Exception:
            pass
        try:
            if self._dll and self._dev:
                self._dll.ZCAN_CloseDevice(self._dev)
        except Exception:
            pass
        self._ch = None
        self._dev = None

    # ---- receive (drains both classic and FD queues) ----
    def _drain(self) -> None:
        dll, ch = self._dll, self._ch
        batch: list[tuple[float, int, CanFrame]] = []
        order = 0
        # FD queue
        n = int(dll.ZCAN_GetReceiveNum(ch, TYPE_CANFD))
        if n > 0:
            self._rx_backlog_max = max(self._rx_backlog_max, n)
            if n > self._RX_CAP:
                self._rx_cap_hits += 1
                n = self._RX_CAP
            arr = (ZCAN_ReceiveFD_Data * n)()
            got = min(max(int(dll.ZCAN_ReceiveFD(ch, C.byref(arr), n, 0)), 0), n)
            for i in range(got):
                ts = self._ts.stamp(int(arr[i].timestamp))
                batch.append((ts, order, self._conv_fd(arr[i].frame, ts)))
                order += 1
        # classic queue
        n = int(dll.ZCAN_GetReceiveNum(ch, TYPE_CAN))
        if n > 0:
            self._rx_backlog_max = max(self._rx_backlog_max, n)
            if n > self._RX_CAP:
                self._rx_cap_hits += 1
                n = self._RX_CAP
            arr = (ZCAN_Receive_Data * n)()
            got = min(max(int(dll.ZCAN_Receive(ch, C.byref(arr), n, 0)), 0), n)
            for i in range(got):
                ts = self._ts.stamp(int(arr[i].timestamp))
                batch.append((ts, order, self._conv_classic(arr[i].frame, ts)))
                order += 1
        # Merge the two queues back into hardware-timestamp order before enqueueing.
        batch.sort(key=lambda p: (p[0], p[1]))
        for _, _, fr in batch:
            self._buf.append(fr)

    def recv(self, timeout: float = 0.1) -> Optional[CanFrame]:
        if self._buf:
            return self._buf.popleft()
        deadline = time.time() + timeout
        while True:
            self._drain()
            if self._buf:
                return self._buf.popleft()
            if time.time() >= deadline:
                return None
            time.sleep(0.002)

    @staticmethod
    def _conv_fd(f: ZCAN_CANFD_FRAME, ts: float) -> CanFrame:
        dlen = min(f.len, 64)
        return CanFrame(can_id=f.can_id, data=bytes(f.data[:dlen]),
                        is_extended=bool(f.eff), is_remote=bool(f.rtr),
                        is_fd=True, brs=bool(f.brs), timestamp=ts, direction="rx")

    @staticmethod
    def _conv_classic(f: ZCAN_CAN_FRAME, ts: float) -> CanFrame:
        dlen = min(f.can_dlc, 8)
        return CanFrame(can_id=f.can_id, data=bytes(f.data[:dlen]),
                        is_extended=bool(f.eff), is_remote=bool(f.rtr),
                        is_fd=False, timestamp=ts, direction="rx")

    # ---- transmit ----
    def send(self, frame: CanFrame) -> None:
        if self._dll is None or self._ch is None:
            raise DriverError("Adapter not open.")
        if self.listen_only:
            raise DriverError("Adapter is in listen-only mode and will not transmit. "
                              "Reconnect with listen-only off to send frames.")
        if frame.is_fd:
            if not self.fd:
                raise DriverError("Channel opened in classic mode; reconnect with CAN FD on to send FD frames.")
            msg = ZCAN_TransmitFD_Data()
            msg.transmit_type = 0
            msg.frame.can_id = frame.can_id & 0x1FFFFFFF
            msg.frame.eff = 1 if frame.is_extended else 0
            msg.frame.rtr = 1 if frame.is_remote else 0
            msg.frame.brs = 1 if frame.brs else 0
            data, dlen = self._pad_fd(frame.data)
            msg.frame.len = dlen
            for i in range(dlen):
                msg.frame.data[i] = data[i]
            sent = self._dll.ZCAN_TransmitFD(self._ch, C.byref(msg), 1)
        else:
            if len(frame.data) > 8:
                raise DriverError("Classic CAN allows at most 8 data bytes.")
            msg = ZCAN_Transmit_Data()
            msg.transmit_type = 0
            msg.frame.can_id = frame.can_id & 0x1FFFFFFF
            msg.frame.eff = 1 if frame.is_extended else 0
            msg.frame.rtr = 1 if frame.is_remote else 0
            msg.frame.can_dlc = len(frame.data)
            for i in range(len(frame.data)):
                msg.frame.data[i] = frame.data[i]
            sent = self._dll.ZCAN_Transmit(self._ch, C.byref(msg), 1)
        if not sent:
            raise DriverError("Transmit reported 0 frames sent (bus error, wrong bitrate, or no ACK on the bus?).")

    @staticmethod
    def _pad_fd(data: bytes):
        n = len(data)
        if n > 64:
            raise DriverError("CAN FD allows at most 64 data bytes.")
        target = next((L for L in _FD_LENS if L >= n), 64)
        return (bytes(data) + bytes(target - n)), target

    def info(self) -> dict:
        return {
            "name": self.name, "supports_fd": True, "channel": self.channel,
            "bitrate": self.bitrate, "fd": self.fd, "data_bitrate": self.data_bitrate,
            "listen_only": self.listen_only, "resistor": self.resistor,
            "canfd_iso": self.canfd_iso, "device_type": self.device_type,
            "rx_backlog_max": self._rx_backlog_max, "rx_cap_hits": self._rx_cap_hits,
            "hw_timestamps": True,
        }
