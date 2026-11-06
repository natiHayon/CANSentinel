"""BusManager owns the active driver, a background reader thread, a rolling
buffer of recent frames (for the live monitor) and an optional capture buffer
(used by the Learn workflow to grab a time window of traffic).
"""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Optional

from .denylist import is_denied, reason
from .drivers.base import CanDriver, DriverError
from .frame import CanFrame


class BusManager:
    def __init__(self, recent_size: int = 40000):
        self._driver: Optional[CanDriver] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.recent: deque[CanFrame] = deque(maxlen=recent_size)
        self._seq = 0
        self._rx_count = 0
        self._tx_count = 0
        self._capture: Optional[list[CanFrame]] = None
        self._last_error: Optional[str] = None
        # Per-id indexes so latest_frame / frames_for_id_since are O(1)/O(new) instead
        # of scanning the whole 40k buffer under the lock at 40-50 Hz (which could
        # throttle the reader and cause RX overflow on a busy bus).
        self._latest_rx: dict[tuple, CanFrame] = {}
        self._by_id: dict[tuple, deque] = {}
        self._by_id_max = 4096
        self._reconnects = 0
        self._reconnecting = False
        self.transmit_guard = None  # optional application gate, checked on EVERY send
        self._send_lock = threading.Lock()
        self._last_send = 0.0
        self._captured_test_after = 0.0
        self._channel = None
        self._timestamp_source = 'unspecified'
        self._health_events = deque(maxlen=128)
        self._rx_groups = {}
        self._last_rx_stamp = {}

    # ---- connection ------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._driver is not None and self._thread is not None and self._thread.is_alive()

    @property
    def driver(self) -> Optional[CanDriver]:
        return self._driver

    def connect(self, driver: CanDriver) -> None:
        if self.connected:
            raise DriverError("Already connected. Disconnect first.")
        driver.open()
        self._driver = driver
        info = driver.info()
        self._channel = info.get('channel')
        self._timestamp_source = 'hardware_anchored' if info.get('hw_timestamps') else 'host_or_driver'
        self._stop.clear()
        self._last_error = None
        with self._lock:
            self.recent.clear()
            self._latest_rx.clear()
            self._by_id.clear()
            self._rx_count = 0
            self._tx_count = 0
            self._rx_groups.clear()
            self._last_rx_stamp.clear()
            self._health_events.clear()
        self._thread = threading.Thread(target=self._reader, name="can-reader", daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
        self._thread = None
        if self._driver is not None:
            self._driver.close()
            self._driver = None

    def _reader(self) -> None:
        drv = self._driver
        while not self._stop.is_set() and drv is not None:
            try:
                frame = drv.recv(timeout=0.1)
            except DriverError as e:
                if not self._try_reconnect(drv, str(e)):
                    break
                continue
            except Exception as e:  # noqa: BLE001 - surface unexpected errors to UI
                if not self._try_reconnect(drv, f"reader error: {e}"):
                    break
                continue
            if frame is None:
                continue
            self._ingest(frame)

    def _try_reconnect(self, drv: CanDriver, err: str, attempts: int = 5) -> bool:
        """Bounded auto-reconnect after a transient driver/USB error. Preserves the
        recent and capture buffers; returns True if the driver is live again."""
        self._last_error = f"{err} — reconnecting..."
        with self._lock:
            self._health_events.append({'ts': time.time(), 'event': 'reconnect', 'detail': err})
        for i in range(1, attempts + 1):
            if self._stop.is_set():
                return False
            self._reconnecting = True
            try:
                drv.close()
            except Exception:
                pass
            if self._stop.wait(min(0.5 * i, 3.0)):   # backoff, but stay responsive to stop
                return False
            try:
                drv.open()
                self._reconnects += 1
                self._reconnecting = False
                self._last_error = None
                return True
            except Exception as e:  # noqa: BLE001
                self._last_error = f"{err} — reconnect {i}/{attempts} failed: {e}"
        self._reconnecting = False
        self._last_error = f"{err} — gave up after {attempts} reconnect attempts"
        return False

    def _ingest(self, frame: CanFrame) -> None:
        with self._lock:
            if frame.channel is None:
                frame.channel = self._channel
            if frame.timestamp_source == 'unspecified':
                frame.timestamp_source = self._timestamp_source
            self._seq += 1
            frame.seq = self._seq
            if frame.direction == "tx":
                self._tx_count += 1
            else:
                self._rx_count += 1
            self.recent.append(frame)
            if frame.direction == "rx":
                clock_key = (frame.channel, frame.timestamp_domain)
                prior = self._last_rx_stamp.get(clock_key)
                if prior is not None and frame.timestamp < prior:
                    self._health_events.append({'ts': time.time(), 'event': 'timestamp_reversal',
                                                'previous': prior, 'received': frame.timestamp})
                self._last_rx_stamp[clock_key] = frame.timestamp
                if frame.timestamp_domain == 'unix' and abs(time.time()-frame.timestamp) > 2:
                    if not self._health_events or self._health_events[-1]['event'] != 'timestamp_offset':
                        self._health_events.append({'ts': time.time(), 'event': 'timestamp_offset',
                                                   'offset_s': time.time()-frame.timestamp})
                group_key = frame.analysis_key()
                group = self._rx_groups.get(group_key)
                now = time.monotonic()
                if group is None:
                    group = {'count': 0, 'changed_at': now, 'frame': frame}
                    self._rx_groups[group_key] = group
                if group['frame'].data != frame.data:
                    group['changed_at'] = now
                group.update(count=group['count']+1, seen_at=now, frame=frame)
                key = frame.key()
                self._latest_rx[key] = frame
                d = self._by_id.get(key)
                if d is None:
                    d = deque(maxlen=self._by_id_max)
                    self._by_id[key] = d
                d.append((frame.seq, frame.timestamp, bytes(frame.data)))
            if self._capture is not None:
                self._capture.append(frame)

    # ---- transmit --------------------------------------------------------
    def send(self, frame: CanFrame) -> None:
        # A shared limit prevents concurrent HTTP/hold workers multiplying TX rate.
        with self._send_lock:
            delay = 0.010 - (time.monotonic() - self._last_send)
            if delay > 0:
                time.sleep(delay)
            self._send_checked(frame)
            self._last_send = time.monotonic()

    def _send_checked(self, frame: CanFrame) -> None:
        if not self.connected or self._driver is None:
            raise DriverError("Not connected.")
        # All general send paths retain the diagnostic deny list. The fixed
        # captured-test method below is the only narrowly scoped exception.
        if is_denied(frame.can_id, frame.is_extended):
            raise DriverError(f"TX refused: {reason(frame.can_id, frame.is_extended)} "
                              f"({frame.id_hex}).")
        if self.transmit_guard is not None:
            self.transmit_guard()
        self._driver.send(frame)
        frame.direction = "tx"
        self._ingest(frame)       # driver submission record, NOT independent wire evidence

    def send_captured_lock_test(self, *, check_test_mode, accepted) -> None:
        """One fixed four-frame request, exclusive against other local senders.

        No caller-supplied IDs/payloads/timing. The sub-10ms interval is allowed
        only here. Timing is host submission timing, not proven wire timing.
        Already submitted frames cannot be recalled by a stop.
        """
        from .captured_test import FRAMES
        if not self._send_lock.acquire(blocking=False):
            raise DriverError("TX is busy; captured test was not queued.")
        try:
            if time.monotonic() < self._captured_test_after:
                raise DriverError("Wait two seconds between captured tests.")
            driver = self._driver

            def check():
                check_test_mode()
                if not self.connected or driver is None or self._driver is not driver or self._reconnecting:
                    raise DriverError("Connection changed or disconnected; captured test stopped.")
                if self.transmit_guard is None:
                    raise DriverError("Captured test requires an application TX guard.")
                self.transmit_guard()

            def wait_until(target):
                while True:
                    check()
                    delay = target - time.monotonic()
                    if delay <= 0:
                        return
                    time.sleep(min(delay, 0.002))

            check()
            info = driver.info()
            if info.get("listen_only") or info.get("mode") in ("silent", "silent_loopback", "loopback"):
                raise DriverError("Use normal adapter mode for a captured vehicle test.")
            self._captured_test_after = time.monotonic() + 2.0
            wait_until(self._last_send + 0.010)
            started = time.monotonic()
            previous_offset = 0.0
            for index, (offset, data) in enumerate(FRAMES):
                # Preserve minimum spacing when Windows wakes late; never catch
                # up by dumping multiple overdue fragments back-to-back.
                target = started + offset / 1000
                if index:
                    target = max(target, self._last_send + (offset-previous_offset)/1000)
                wait_until(target)
                if time.monotonic() - started > 0.250:
                    raise DriverError("Captured test exceeded its 250ms scheduling limit.")
                check()
                frame = CanFrame(0x7E0, data, direction="tx")
                submitted_ms = (time.monotonic()-started)*1000
                driver.send(frame)
                self._last_send = time.monotonic()
                self._ingest(frame)
                accepted({"index": index, "offset_ms": offset,
                          "submitted_ms": round(submitted_ms, 3),
                          "id_hex": frame.id_hex, "data_hex": frame.data_hex})
                previous_offset = offset
        finally:
            self._send_lock.release()

    # ---- capture (used by Learn) ----------------------------------------
    def start_capture(self) -> None:
        with self._lock:
            self._capture = []

    def stop_capture(self) -> list[CanFrame]:
        with self._lock:
            frames = self._capture or []
            self._capture = None
            return list(frames)

    def capture_window(self, seconds: float) -> list[CanFrame]:
        """Blocking helper: capture for `seconds` and return the frames."""
        self.start_capture()
        time.sleep(max(0.0, seconds))
        return self.stop_capture()

    def frames_since(self, seconds: float) -> list[CanFrame]:
        """Frames from the rolling buffer within the last `seconds` (the 'before'
        control for a trigger capture). Needs a large recent buffer on busy buses."""
        cutoff = time.time() - max(0.0, seconds)
        with self._lock:
            return [f for f in self.recent if f.timestamp >= cutoff]

    def latest_frame(self, can_id: int, is_extended: bool):
        """Most recent RX frame for an id (to merge a learned byte-change onto the
        live frame so counters/checksums/other bytes stay current). O(1) via the
        per-id cache instead of scanning the whole buffer."""
        with self._lock:
            return self._latest_rx.get((can_id, is_extended))

    def frames_for_id_since(self, can_id: int, is_extended: bool, since_seq: int):
        """All RX frames for one id with seq > since_seq, in order. O(new frames)
        via the per-id index - used by the sequence detector at ~40 Hz."""
        with self._lock:
            d = self._by_id.get((can_id, is_extended))
            if not d:
                return []
            return [(s, t, data) for (s, t, data) in d if s > since_seq]

    @property
    def current_seq(self) -> int:
        with self._lock:
            return self._seq

    # ---- views -----------------------------------------------------------
    def snapshot(self, limit: int = 250, since_seq: int = 0) -> list[dict]:
        with self._lock:
            frames = [f for f in self.recent if f.seq > since_seq]
        return [f.to_dict() for f in frames[-limit:]]

    def traffic(self, window: int = 4000) -> list[dict]:
        """Aggregate recent frames by id for a live bus table."""
        with self._lock:
            frames = list(self.recent)[-window:]
            groups = {k: dict(v) for k,v in self._rx_groups.items()}
        agg: dict[tuple, dict] = {}
        for f in frames:
            if f.direction != 'rx':
                continue
            k = f.analysis_key()
            e = agg.get(k)
            if e is None:
                agg[k] = {
                    "id": f.can_id, "id_hex": f.id_hex, "ext": f.is_extended,
                    "fd": f.is_fd, "count": 1, "last": f.data_hex, "dlc": f.dlc,
                    "first_ts": f.timestamp, "last_ts": f.timestamp, "dir": f.direction,
                }
            else:
                e["count"] += 1
                e["last"] = f.data_hex
                e["dlc"] = f.dlc
                e["last_ts"] = f.timestamp
                e["dir"] = f.direction
        out = []
        now = time.monotonic()
        for key, group in groups.items():
            f = group['frame']
            e = agg.get(key, {'id': f.can_id, 'id_hex': f.id_hex, 'ext': f.is_extended,
                             'fd': f.is_fd, 'count': 0, 'last': f.data_hex, 'dlc': f.dlc,
                             'first_ts': f.timestamp, 'last_ts': f.timestamp, 'dir': 'rx'})
            span = e["last_ts"] - e["first_ts"]
            e["hz"] = round((e["count"] - 1) / span, 1) if span > 0 and e["count"] > 1 else 0.0
            e['window_count'] = e['count']
            e['count'] = group['count']
            e['age_ms'] = round((now-group['seen_at'])*1000, 1)
            e['unchanged_ms'] = round((now-group['changed_at'])*1000, 1)
            e['channel'] = f.channel
            e['freshness'] = 'receiving' if e['age_ms'] <= 1000 else 'stale'
            if e['freshness'] == 'stale':
                e['hz'] = 0.0
            e['key'] = repr(key)
            del e["first_ts"]
            out.append(e)
        out.sort(key=lambda x: x["id"])
        return out

    def status(self) -> dict:
        info = self._driver.info() if self._driver else None
        with self._lock:
            rx, tx, seq = self._rx_count, self._tx_count, self._seq
            health = list(self._health_events)
        return {
            "connected": self.connected,
            "driver": info,
            "rx": rx,
            "tx": tx,
            "seq": seq,
            "error": self._last_error,
            "supports_fd": bool(info and info.get("supports_fd")),
            "reconnecting": self._reconnecting,
            "reconnects": self._reconnects,
            "rx_backlog_max": (info or {}).get("rx_backlog_max"),
            "rx_cap_hits": (info or {}).get("rx_cap_hits"),
            "health_events": health,
            "hardware_drops": (info or {}).get('rx_dropped'),
            "channel": self._channel,
            "timestamp_source": self._timestamp_source,
        }
