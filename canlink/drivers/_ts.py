"""Map a hardware device tick-counter to wall-clock seconds.

The adapters stamp each received frame with their own monotonic hardware timer.
Previously those stamps were discarded and ``time.time()`` was written at Python
batch-convert time, so a whole drained burst got one near-identical wall-clock
stamp — destroying the inter-frame timing that signal identification depends on
(period, jitter, burst grouping).

``TimestampAnchor`` keeps the first ``(device_tick, host_time)`` pair it sees and
then reports ``host0 + (tick - tick0) * scale``, so the returned value stays in
the wall-clock domain that the rest of the app assumes (``frames_since``, capture
windows) while preserving the true spacing between frames in one burst.

Safety: the result is clamped to ``<= now`` so an unexpectedly fast tick unit (or
a wrong ``scale``) can never push a frame into the future — in the worst case the
spacing benefit is lost and behaviour degrades to the old "stamp at now", never
worse. A device counter that runs backwards (unavailable, or a uint32 rollover)
re-anchors. ``scale`` is seconds per tick (default 1e-6 = microseconds).

The caller is expected to sort a drained batch by the returned timestamp, so this
class does NOT force monotonic output across calls (which would corrupt the true
order when two receive queues are drained one after the other).
"""
from __future__ import annotations

import time


class TimestampAnchor:
    def __init__(self, scale: float = 1e-6):
        self.scale = float(scale)
        self._dev0 = None
        self._host0 = 0.0

    def reset(self) -> None:
        self._dev0 = None
        self._host0 = 0.0

    def stamp(self, dev_ts: int) -> float:
        host = time.time()
        if not dev_ts:                       # device tick unavailable -> host time
            return host
        if self._dev0 is None or dev_ts < self._dev0:   # first frame, or counter wrapped
            self._dev0 = dev_ts
            self._host0 = host
            return host
        t = self._host0 + (dev_ts - self._dev0) * self.scale
        return t if t <= host else host      # never stamp into the future
