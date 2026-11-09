"""A fake CAN bus so you can exercise the whole app with no hardware attached.

It continuously emits a mix of periodic "background" frames (engine rpm with a
rolling counter, vehicle speed, a static door-status frame, etc). When the UI
calls inject(event), it adds the frame(s) that a real action would produce, so
the Learn workflow can actually find a candidate and you can test replay.

This lets you validate the learn -> save -> trigger loop end to end before you
ever connect to a car.
"""
from __future__ import annotations

import random
import threading
import time
from collections import deque
from typing import Optional

from .base import CanDriver
from ..frame import CanFrame


class Simulator(CanDriver):
    name = "simulator"
    supports_fd = True            # the fake bus happily carries FD frames too

    # Background periodic signals: id -> (period_s, builder(counter) -> bytes)
    def _background(self):
        return {
            0x0C9: (0.01, lambda c: bytes([0x10, c & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, (c * 7) & 0xFF])),  # rpm-ish + counter
            0x100: (0.02, lambda c: bytes([(c * 3) & 0xFF, 0x00, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00])),       # speed-ish
            0x1A0: (0.05, lambda c: bytes([0x00, 0x00, 0x00, 0x00])),                                          # quiet/static
            0x320: (0.10, lambda c: bytes([0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00])),                  # door status (locked bit here)
            0x224: (0.02, lambda c: bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),                  # brake pedal position (0 at rest)
        }

    def __init__(self):
        self._q: deque[CanFrame] = deque()
        self._counters = {}
        self._next = {}
        self._lock = threading.Lock()
        self._running = False
        self._pending: list[CanFrame] = []
        self._bg = self._background()

    def open(self) -> None:
        now = time.time()
        self._counters = {cid: 0 for cid in self._bg}
        self._next = {cid: now for cid in self._bg}
        self._running = True

    def close(self) -> None:
        self._running = False
        self._q.clear()
        self._pending.clear()

    def send(self, frame: CanFrame) -> None:
        # Loop back transmitted frames so the monitor shows them.
        f = CanFrame(
            can_id=frame.can_id, data=frame.data, is_extended=frame.is_extended,
            is_remote=frame.is_remote, is_fd=frame.is_fd, brs=frame.brs, direction="tx",
        )
        with self._lock:
            self._q.append(f)

    def inject(self, event: str) -> None:
        """Simulate a physical action producing CAN traffic."""
        event = (event or "").lower()
        frames: list[CanFrame] = []
        if "unlock" in event:
            frames.append(CanFrame(0x19B, bytes([0x40, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])))
        elif "lock" in event:
            frames.append(CanFrame(0x19B, bytes([0x80, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])))
        elif "horn" in event:
            frames.append(CanFrame(0x1D0, bytes([0x01, 0x00, 0x00, 0x00])))
        elif "brake" in event:
            # brake is a STATUS signal: byte 0 of 0x224 jumps from rest to pressed
            frames.append(CanFrame(0x224, bytes([0x64, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])))
        else:
            # generic event frame on an otherwise-silent id
            frames.append(CanFrame(0x2F0, bytes([0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])))
        with self._lock:
            # Real buttons emit a short burst. Enqueue DISTINCT frame objects (fresh
            # timestamp, small spacing) per repeat — not the same object 3x — so seq and
            # timestamp are unique per copy and the burst exercises real intra-burst timing.
            now = time.time()
            k = 0
            for _ in range(3):
                for src in frames:
                    self._pending.append(CanFrame(
                        can_id=src.can_id, data=bytes(src.data),
                        is_extended=src.is_extended, is_remote=src.is_remote,
                        is_fd=src.is_fd, brs=src.brs,
                        timestamp=now + k * 0.002, direction="rx"))
                    k += 1

    def recv(self, timeout: float = 0.1) -> Optional[CanFrame]:
        deadline = time.time() + timeout
        while self._running:
            with self._lock:
                if self._pending:
                    return self._pending.pop(0)
                if self._q:
                    return self._q.popleft()
            now = time.time()
            due = [cid for cid, t in self._next.items() if t <= now]
            if due:
                cid = min(due, key=lambda c: self._next[c])
                period, builder = self._bg[cid]
                self._counters[cid] += 1
                self._next[cid] = now + period
                data = builder(self._counters[cid])
                # add a little noise to one byte so some ids look "dynamic"
                if cid == 0x100:
                    data = bytes([data[0], random.randint(0, 3)]) + data[2:]
                return CanFrame(can_id=cid, data=data, direction="rx")
            if now >= deadline:
                return None
            time.sleep(0.001)
        return None

    def info(self) -> dict:
        return {"name": self.name, "supports_fd": True, "note": "simulated bus (no hardware)"}
