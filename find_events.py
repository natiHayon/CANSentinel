#!/usr/bin/env python3
"""
find_events.py - locate the CAN ID / byte / bit that encodes a DISCRETE action
(a button press, a lock/unlock, a stalk/switch, a gear position, ...).

This is a READ-ONLY analysis tool. It never opens a CAN interface and never
transmits anything. It reads a recorded log you already captured and ranks the
candidate fields that match your actions.

Why a dedicated discrete-event finder
--------------------------------------
Most CAN reverse-engineering flows are tuned for *continuous* signals (speed,
RPM) where you sweep a value and correlate. Buttons and locks are different:

  * a BUTTON is usually a bit that PULSES (0 -> 1 -> 0) for the moment it is held,
    OR a "which-button" ENUM BYTE on a shared message (idle value + one value per
    button - very common on steering-wheel / console controls),
  * a LOCK / gear / switch is usually a bit (or small field) that TOGGLES between
    stable states.

Aligning a single press by hand is hopeless (reaction lag swamps it). The robust
trick is REPETITION: trigger the action a known number of times, and let the tool
find the field that changed exactly that many times while the rest of the bus was
quiet. No precise timing needed.

Three ways to tell it what you did (pick one)
---------------------------------------------
  (A) --events N        You triggered the action N times during the recording.
                        (Frictionless - no timestamps needed. Start here.)
  (B) --event-times F   A file of epoch timestamps of each trigger (one per line),
                        or a CSS-Electronics sidecar CSV (epoch;kind;label;value).
                        Adds timing confirmation + a lag search.
  (C) --baseline F      A second "quiet" recording where you did NOTHING. Fields
                        active here but idle in the baseline are the candidates.
                        Combine with (A)/(B) for the strongest result.

Human-cadence filter (always on)
--------------------------------
  Candidates whose activations repeat faster than a person plausibly performs
  the action - a rolling counter or periodic status word firing every few
  hundred ms - are downranked (or dropped with --drop-implausible). Tune the
  window with --min-gap (hard floor) and --settle-gap (fully-human spacing).

Supported log formats (auto-detected)
-------------------------------------
  * webCAN / CANsub CSV  (TimestampEpoch;BusChannel;ID;...;DataBytes)
  * candump (Linux SocketCAN):   (1700000000.123456) can0 123#DEADBEEF
  * generic delimited:   <epoch><sep><id-hex><sep><data-hex>   (sep = ; , or space)
  * JSON capture:        {"frames":[{"ts":..,"id":..,"data":[..],"dir":"rx"}, ...]}
                         (or a bare list of such frame objects; RX frames only)

Examples
--------
  # I pressed the volume-up button 8 times during this recording:
  python find_events.py --trace press_volup.csv --events 8

  # Lock/unlock toggled 10 times, and I have a quiet baseline too:
  python find_events.py --trace locking.csv --baseline quiet.csv --events 10 --mode toggle

  # I logged my key presses as timestamps:
  python find_events.py --trace run.log --event-times presses.txt --tol 0.4

  # Prove it works with no hardware (synthetic bus with planted signals):
  python find_events.py --selftest
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# Log loading  (format-agnostic)
# ---------------------------------------------------------------------------

WEBCAN_HEADER = "TimestampEpoch"
_CANDUMP_RE = re.compile(
    r"^\(?\s*(\d+(?:\.\d+)?)\)?\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)"
)


@dataclass
class Frame:
    t: float
    can_id: int
    data: bytes


def _parse_hex_id(s: str) -> int:
    s = s.strip()
    if s.lower().startswith("0x"):
        return int(s, 16)
    return int(s, 16)


def load_log(path: str) -> list[Frame]:
    """Load a CAN log in any of the supported formats into a time-sorted list."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        raw = fh.read()

    frames: list[Frame]
    if raw.lstrip().startswith(("{", "[")):
        frames = _load_json(raw, path)
    else:
        lines = [ln for ln in raw.splitlines() if ln.strip()]
        if not lines:
            raise ValueError(f"{path}: empty log")
        first = lines[0]
        if first.startswith(WEBCAN_HEADER):
            frames = _load_webcan(lines)
        elif _CANDUMP_RE.match(first):
            frames = _load_candump(lines)
        else:
            frames = _load_generic(lines)

    if not frames:
        raise ValueError(f"{path}: no frames parsed (unrecognized format?)")
    frames.sort(key=lambda f: f.t)
    return frames


def _load_json(raw: str, path: str = "<json>") -> list[Frame]:
    """Load a JSON capture (e.g. the project's timed CAN reader).

    Accepts a dict with a "frames" list, or a bare list of frame objects. Each
    frame gives an epoch timestamp, an arbitration ID and a data payload under
    common key spellings (``ts``/``source_timestamp``, ``id``/``id_hex``,
    ``data`` int-list / ``data_hex``). Only receive frames are kept: transmit
    echoes (``dir`` of tx/out/1) and remote-request frames (``rtr``) are
    dropped, so the loader stays read-only-safe and consistent with the webCAN
    loader. Any metadata outside "frames" (make/model/presses) is ignored here.
    """
    doc = json.loads(raw)
    if isinstance(doc, dict):
        records = doc.get("frames")
        if records is None:
            raise ValueError(f"{path}: JSON object has no 'frames' list")
    elif isinstance(doc, list):
        records = doc
    else:
        raise ValueError(f"{path}: unrecognized JSON log shape")

    out: list[Frame] = []
    for r in records:
        if not isinstance(r, dict):
            continue
        if str(r.get("dir", "")).strip().lower() in ("tx", "t", "1", "out"):
            continue  # skip TX echoes; keep RX
        if r.get("rtr"):
            continue  # remote-request frame carries no data payload
        t = r.get("ts", r.get("source_timestamp"))
        if t is None:
            continue
        cid = r.get("id")
        if cid is None:
            idh = r.get("id_hex")
            if idh is None:
                continue
            try:
                cid = int(str(idh), 16)
            except ValueError:
                continue
        data = r.get("data")
        if isinstance(data, list):
            try:
                payload = bytes(int(b) & 0xFF for b in data)
            except (TypeError, ValueError):
                continue
        else:
            dh = r.get("data_hex")
            if dh is None:
                continue
            try:
                payload = bytes.fromhex(str(dh).replace(" ", ""))
            except ValueError:
                continue
        try:
            out.append(Frame(float(t), int(cid), payload))
        except (TypeError, ValueError):
            continue
    return out


def _load_webcan(lines: list[str]) -> list[Frame]:
    rdr = csv.reader(lines, delimiter=";")
    header = next(rdr)
    col = {name: i for i, name in enumerate(header)}
    need = ("TimestampEpoch", "ID", "DataBytes")
    for n in need:
        if n not in col:
            raise ValueError(f"webCAN CSV missing column {n}")
    dir_i = col.get("Dir")
    out = []
    for row in rdr:
        if len(row) <= col["DataBytes"]:
            continue
        if dir_i is not None and len(row) > dir_i and row[dir_i].strip() == "1":
            continue  # skip TX (echo) frames; keep RX
        try:
            t = float(row[col["TimestampEpoch"]])
            cid = int(row[col["ID"]], 16)
            data = bytes.fromhex(row[col["DataBytes"]].strip())
        except (ValueError, IndexError):
            continue
        out.append(Frame(t, cid, data))
    return out


def _load_candump(lines: list[str]) -> list[Frame]:
    out = []
    for ln in lines:
        m = _CANDUMP_RE.match(ln)
        if not m:
            continue
        t = float(m.group(1))
        cid = int(m.group(2), 16)
        try:
            data = bytes.fromhex(m.group(3))
        except ValueError:
            continue
        out.append(Frame(t, cid, data))
    return out


def _load_generic(lines: list[str]) -> list[Frame]:
    # sniff the delimiter
    sample = lines[0]
    if ";" in sample:
        sep = ";"
    elif "," in sample:
        sep = ","
    else:
        sep = None  # whitespace
    out = []
    header_skipped = False
    for ln in lines:
        parts = ln.split(sep) if sep else ln.split()
        parts = [p.strip() for p in parts if p.strip() != ""]
        if len(parts) < 3:
            continue
        try:
            t = float(parts[0])
        except ValueError:
            if not header_skipped:
                header_skipped = True
                continue
            continue
        # id = first field that is valid hex and not the timestamp; data = next
        try:
            cid = int(parts[1], 16)
            data = bytes.fromhex(parts[2].replace("0x", ""))
        except ValueError:
            continue
        out.append(Frame(t, cid, data))
    return out


# ---------------------------------------------------------------------------
# Per-ID payload matrices
# ---------------------------------------------------------------------------

@dataclass
class IdSeries:
    can_id: int
    t: np.ndarray            # (F,) float epoch
    payload: np.ndarray      # (F, L) uint8  (modal length L)
    bits: np.ndarray         # (F, L*8) uint8, LSB-first within each byte
    n: int
    length: int


def build_series(frames: list[Frame]) -> dict[int, IdSeries]:
    by_id: dict[int, list[Frame]] = {}
    for f in frames:
        by_id.setdefault(f.can_id, []).append(f)

    out: dict[int, IdSeries] = {}
    for cid, fl in by_id.items():
        lengths = [len(f.data) for f in fl if f.data]
        if not lengths:
            continue
        modal = int(np.bincount(np.array(lengths)).argmax())
        if modal == 0:
            continue
        kept = [f for f in fl if len(f.data) == modal]
        if len(kept) < 3:
            continue
        t = np.array([f.t for f in kept], dtype=np.float64)
        payload = np.array([list(f.data) for f in kept], dtype=np.uint8)
        # bits LSB-first per byte: bit index = byte*8 + b
        bits = np.unpackbits(payload, axis=1, bitorder="little")
        out[cid] = IdSeries(cid, t, payload, bits, len(kept), modal)
    return out


# ---------------------------------------------------------------------------
# Counter / checksum / noise rejection
# ---------------------------------------------------------------------------

def _byte_is_counter(col: np.ndarray) -> bool:
    """A free-running counter or high-entropy (checksum-like) byte: reject it."""
    n = len(col)
    if n < 8:
        return False
    changes = int(np.count_nonzero(np.diff(col.astype(np.int16)) != 0))
    change_frac = changes / (n - 1)
    distinct = len(np.unique(col))
    # increments mostly by +1 (mod 256) -> rolling counter
    d = np.diff(col.astype(np.int16)) % 256
    inc_frac = float(np.mean(d == 1)) if n > 1 else 0.0
    if inc_frac > 0.6:
        return True
    # changes almost every frame AND many distinct values -> counter/checksum/noise
    if change_frac > 0.6 and distinct > 16:
        return True
    if distinct > 64:
        return True
    return False


# ---------------------------------------------------------------------------
# Candidate extraction
# ---------------------------------------------------------------------------

@dataclass
class Candidate:
    can_id: int
    kind: str                # 'pulse-bit' | 'toggle-bit' | 'value-byte'
    byte: int
    bit: int | None          # bit within byte (LSB=0) for bit kinds; None for byte
    idle: int                # idle level/value
    active_values: list[int] # for value-byte: the non-idle values seen (button codes)
    edge_times: np.ndarray   # times of rising/entering-active transitions
    count: int               # number of activations detected
    activity_frac: float     # fraction of frames spent in the active state
    score: float = 0.0
    detail: str = ""


def _runs_of(mask: np.ndarray) -> list[tuple[int, int]]:
    """Return (start_idx, end_idx_exclusive) of maximal True runs in a bool array."""
    if mask.size == 0:
        return []
    idx = np.flatnonzero(np.diff(mask.astype(np.int8)) != 0) + 1
    bounds = [0, *idx.tolist(), mask.size]
    runs = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        if mask[a]:
            runs.append((a, b))
    return runs


def extract_candidates(
    s: IdSeries,
    idle_frac_max: float,
    baseline_bits: np.ndarray | None,
    baseline_byte_vals: dict[int, set] | None,
) -> list[Candidate]:
    cands: list[Candidate] = []
    F, L = s.payload.shape

    # bytes that are rolling counters / checksums / high-entropy noise: none of
    # their bits is a discrete control, so skip them wholesale (this is what
    # stops a counter's slow high bit from looking like a toggle).
    counter_byte = [_byte_is_counter(s.payload[:, b]) for b in range(L)]

    # ---- per-bit candidates (pulse + toggle) ----
    for bidx in range(L * 8):
        byte = bidx // 8
        bit = bidx % 8
        if counter_byte[byte]:
            continue
        col = s.bits[:, bidx]
        if col.min() == col.max():
            continue  # constant bit
        active_frac = float(col.mean())
        idle = 0 if active_frac <= 0.5 else 1
        in_active = (col == (1 - idle))
        flips = int(np.count_nonzero(np.diff(col.astype(np.int8)) != 0))
        change_frac = flips / max(1, F - 1)
        if change_frac > 0.5:
            continue  # flips too often to be a discrete control
        act = float(in_active.mean())
        runs = _runs_of(in_active)
        if not runs:
            continue
        rising = len(runs)                       # entries into the active state
        mean_run = float(np.mean([b - a for a, b in runs]))

        # baseline gating: skip bits that already moved in the quiet log
        if baseline_bits is not None and bidx < baseline_bits.shape[1]:
            if baseline_bits[:, bidx].min() != baseline_bits[:, bidx].max():
                continue

        # pulse (brief, mostly idle) vs toggle (holds, ~even split)
        is_pulse = (act < 0.3 and mean_run <= 0.25 * F / max(1, rising))
        if is_pulse:
            kind, count = "pulse-bit", rising
            edge_times = s.t[[a for a, _ in runs]]
        else:
            if act > idle_frac_max and not (0.2 <= act <= 0.8):
                continue  # held active most of the time and not a clean toggle
            kind, count = "toggle-bit", flips
            chg = np.flatnonzero(np.diff(col.astype(np.int8)) != 0) + 1
            edge_times = s.t[chg]
        cands.append(Candidate(
            can_id=s.can_id, kind=kind, byte=byte, bit=bit, idle=idle,
            active_values=[1 - idle], edge_times=edge_times, count=count,
            activity_frac=act,
        ))

    # ---- per-byte value-enum candidates (steering-wheel / console buttons) ----
    for byte in range(L):
        col = s.payload[:, byte]
        if _byte_is_counter(col):
            continue
        vals, counts = np.unique(col, return_counts=True)
        if len(vals) < 2 or len(vals) > 16:
            continue
        idle_val = int(vals[np.argmax(counts)])
        if counts.max() / F < 0.4:
            continue  # no dominant idle value -> not an enum control
        in_active = (col != idle_val)
        act = float(in_active.mean())
        if act > idle_frac_max and not (0.25 <= act <= 0.75):
            continue
        runs = _runs_of(in_active)
        if not runs:
            continue

        if baseline_byte_vals is not None:
            base = baseline_byte_vals.get((s.can_id, byte))
            if base is not None:
                active_codes = {int(col[a]) for a, _ in runs}
                if active_codes.issubset(base):
                    continue  # those values already appeared when idle -> noise

        edge_times = s.t[[a for a, _ in runs]]
        active_values = sorted({int(col[a]) for a, _ in runs})
        cands.append(Candidate(
            can_id=s.can_id, kind="value-byte", byte=byte, bit=None,
            idle=idle_val, active_values=active_values, edge_times=edge_times,
            count=len(runs), activity_frac=act,
        ))
    return cands


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------

def score_by_count(c: Candidate, n_events: int) -> None:
    """Rank by how close the activation count is to the expected N."""
    # closeness to N (and to 2N, for a lock toggled both ways -> each counts once
    # per state change; we reward the nearer of N and 2N)
    err = min(abs(c.count - n_events), abs(c.count - 2 * n_events))
    base = 1.0 / (1.0 + err)
    # crispness bonus: a clean control is idle most of the time
    idle_bonus = 1.0 - min(c.activity_frac, 1.0)
    # a value byte that shows several structured codes is the textbook
    # steering-wheel / console button cluster - reward it (small, capped)
    code_bonus = 0.0
    if c.kind == "value-byte":
        code_bonus = 0.03 * min(len(c.active_values), 6)
    c.score = base * (0.7 + 0.3 * idle_bonus) + code_bonus
    tag = {"pulse-bit": "rising edges", "toggle-bit": "state changes",
           "value-byte": "activation bursts"}[c.kind]
    c.detail = f"{c.count} {tag} (you said {n_events})"


def _load_event_times(path: str) -> np.ndarray:
    times = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        lines = [ln.strip() for ln in fh if ln.strip()]
    if lines and lines[0].startswith("epoch;"):  # CSS sidecar
        rdr = csv.DictReader(lines, delimiter=";")
        for row in rdr:
            if row.get("kind", "") in ("event", "anchor"):
                try:
                    times.append(float(row["epoch"]))
                except (ValueError, KeyError):
                    pass
    else:
        for ln in lines:
            tok = ln.split(",")[0].split(";")[0].split()[0]
            try:
                times.append(float(tok))
            except ValueError:
                pass
    return np.array(sorted(times), dtype=np.float64)


def score_by_times(c: Candidate, events: np.ndarray, tol: float) -> None:
    """Lag-tolerant F1 between candidate edge times and expected event times."""
    if c.edge_times.size == 0 or events.size == 0:
        c.score = 0.0
        c.detail = "no edges"
        return
    best = (0.0, 0.0)  # (f1, lag)
    for lag in np.linspace(-2.0, 2.0, 41):
        ev = events + lag
        matched_e = 0
        used = np.zeros(c.edge_times.size, dtype=bool)
        for e in ev:
            d = np.abs(c.edge_times - e)
            d[used] = np.inf
            j = int(np.argmin(d))
            if d[j] <= tol:
                used[j] = True
                matched_e += 1
        recall = matched_e / events.size
        precision = matched_e / c.edge_times.size
        f1 = 0.0 if (precision + recall) == 0 else 2 * precision * recall / (precision + recall)
        if f1 > best[0]:
            best = (f1, lag)
    c.score = best[0]
    c.detail = (f"F1={best[0]:.2f} @ lag {best[1]:+.2f}s "
                f"({c.count} edges vs {events.size} events)")


# ---------------------------------------------------------------------------
# Human-cadence plausibility
# ---------------------------------------------------------------------------

_CADENCE_FLOOR_PENALTY = 0.15  # score retained by an implausibly-fast candidate


def cadence_factor(
    edge_times: np.ndarray, min_gap: float, settle_gap: float
) -> tuple[float, float]:
    """How plausible is this candidate's repeat rate for a *human* action?

    A person triggering a control N times leaves clear, human-scale rests
    between activations. A rolling counter, periodic status word, or other
    "something the car just does" fires on a fast, machine-regular beat. We key
    on the *onset-to-onset* spacing (ignoring how briefly each pulse is held, so
    a clean 80 ms button pulse is judged by the ~1 s between presses, not its
    width):

      * smallest onset gap >= ``settle_gap``  -> 1.0   (clearly human-spaced)
      * smallest onset gap <= ``min_gap``      -> floor (faster than a person can
        deliberately repeat the action -> almost certainly machine traffic)
      * in between                              -> linear ramp

    Returns ``(factor, min_onset_gap)``. With fewer than two activations there
    is no cadence to judge, so the factor is a neutral 1.0 and the gap is inf.
    """
    onsets = np.sort(np.asarray(edge_times, dtype=np.float64))
    if onsets.size < 2:
        return 1.0, float("inf")
    g = float(np.min(np.diff(onsets)))
    if g >= settle_gap:
        return 1.0, g
    if g <= min_gap:
        return _CADENCE_FLOOR_PENALTY, g
    span = settle_gap - min_gap
    frac = (g - min_gap) / span if span > 0 else 0.0
    return _CADENCE_FLOOR_PENALTY + (1.0 - _CADENCE_FLOOR_PENALTY) * frac, g


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def _fmt_candidate(rank: int, c: Candidate) -> str:
    if c.bit is not None:
        loc = f"byte {c.byte}, bit {c.bit}  (LSB-first; cantools start-bit {c.byte*8 + c.bit})"
        enc = f"{c.kind}, idle={c.idle}"
    else:
        codes = ", ".join(f"0x{v:02X}" for v in c.active_values[:8])
        loc = f"byte {c.byte}  (whole byte)"
        enc = f"value-byte, idle=0x{c.idle:02X}, button code(s)=[{codes}]"
    return (f"  #{rank}  ID 0x{c.can_id:X}  {loc}\n"
            f"        {enc}\n"
            f"        score {c.score:.3f}   {c.detail}")


def analyze(
    trace: str,
    n_events: int | None,
    event_times_path: str | None,
    baseline: str | None,
    tol: float,
    mode: str,
    idle_frac_max: float,
    only_ids: set[int] | None,
    exclude_ids: set[int] | None,
    top: int,
    min_gap: float = 0.30,
    settle_gap: float = 1.0,
    drop_implausible: bool = False,
) -> list[Candidate]:
    frames = load_log(trace)
    series = build_series(frames)

    baseline_bits = None
    baseline_byte_vals = None
    if baseline:
        bframes = load_log(baseline)
        bseries = build_series(bframes)
        # concatenate baseline bit columns per matching ID isn't trivial across IDs;
        # we gate per (ID): a bit is "baseline-active" if it moved in the baseline,
        # a byte value set is the set of values it took in the baseline.
        baseline_bits_map = {cid: bs.bits for cid, bs in bseries.items()}
        baseline_byte_vals = {}
        for cid, bs in bseries.items():
            for byte in range(bs.payload.shape[1]):
                baseline_byte_vals[(cid, byte)] = set(np.unique(bs.payload[:, byte]).tolist())

    events = _load_event_times(event_times_path) if event_times_path else None

    all_cands: list[Candidate] = []
    for cid, s in series.items():
        if only_ids and cid not in only_ids:
            continue
        if exclude_ids and cid in exclude_ids:
            continue
        bbits = None
        if baseline:
            bbits = baseline_bits_map.get(cid)
        cands = extract_candidates(s, idle_frac_max, bbits, baseline_byte_vals)
        all_cands.extend(cands)

    if mode != "auto":
        want = {"pulse": "pulse-bit", "toggle": "toggle-bit", "value": "value-byte"}[mode]
        all_cands = [c for c in all_cands if c.kind == want]

    kept: list[Candidate] = []
    for c in all_cands:
        if events is not None:
            score_by_times(c, events, tol)
        elif n_events is not None:
            score_by_count(c, n_events)
        else:
            # no spec: rank by "crispness" - idle most of the time, few clean bursts
            c.score = (1.0 - min(c.activity_frac, 1.0)) / (1.0 + abs(c.count - 1))
            c.detail = f"{c.count} activations, active {c.activity_frac*100:.1f}% of frames"

        # human-cadence plausibility: a rolling counter / periodic status word
        # repeats far faster than a person performs a discrete action, so a
        # candidate firing below human cadence is downranked (or dropped).
        factor, gap = cadence_factor(c.edge_times, min_gap, settle_gap)
        if drop_implausible and np.isfinite(gap) and gap < min_gap:
            continue  # faster than any human action -> machine traffic, drop it
        c.score *= factor
        if factor < 0.999 and np.isfinite(gap):
            c.detail += f"; fast cadence {gap:.2f}s (x{factor:.2f})"
        kept.append(c)

    kept.sort(key=lambda c: c.score, reverse=True)
    return kept[:top]


# ---------------------------------------------------------------------------
# Self-test: synthesize a bus and confirm recovery (no hardware needed)
# ---------------------------------------------------------------------------

def _selftest() -> int:
    rng = np.random.default_rng(0)
    rows = []
    t0 = 1_700_000_000.0
    # background: a 50 Hz ID with a rolling counter + a noisy checksum, and a
    # static ID. Then plant three real controls on their own IDs.
    dur = 20.0
    # timeline of 6 button presses (momentary), 5 lock toggles, 6 wheel-button hits
    press_times = t0 + np.array([2, 5, 8, 11, 14, 17], float)
    lock_times = t0 + np.array([3, 6, 9, 12, 15], float)
    wheel_times = t0 + np.array([2.5, 5.5, 8.5, 11.5, 14.5, 17.5], float)
    wheel_codes = [0x01, 0x02, 0x01, 0x04, 0x02, 0x01]

    def emit(t, cid, data):
        rows.append((f"{t:.6f}", f"{cid:03X}", data.hex()))

    counter = 0
    # 0x100 @ ~50Hz: byte0 counter, byte1 checksum-ish noise, byte2 static
    t = t0
    while t < t0 + dur:
        d = bytearray(8)
        d[0] = counter & 0xFF
        counter += 1
        d[1] = int(rng.integers(0, 256))
        d[2] = 0xA5
        emit(t, 0x100, bytes(d))
        t += 0.02
    # 0x200 @ ~20Hz: the BUTTON pulse on byte3 bit1 (momentary ~0.3s high)
    t = t0
    while t < t0 + dur:
        d = bytearray(8)
        hi = any(abs(t - p) < 0.15 for p in press_times)
        if hi:
            d[3] |= (1 << 1)
        emit(t, 0x200, bytes(d))
        t += 0.05
    # 0x201 @ ~10Hz: the LOCK toggle on byte0 bit0 (flips and holds)
    t = t0
    state = 0
    flips = sorted(lock_times.tolist())
    fi = 0
    while t < t0 + dur:
        while fi < len(flips) and t >= flips[fi]:
            state ^= 1
            fi += 1
        d = bytearray(8)
        if state:
            d[0] |= 1
        emit(t, 0x201, bytes(d))
        t += 0.1
    # 0x5C1 @ ~20Hz: steering-wheel ENUM byte4 (idle 0x00, code during press)
    t = t0
    while t < t0 + dur:
        d = bytearray(8)
        for p, code in zip(wheel_times, wheel_codes):
            if abs(t - p) < 0.15:
                d[4] = code
        emit(t, 0x5C1, bytes(d))
        t += 0.05

    # write a generic CSV and run the finder on it
    import tempfile, os
    path = os.path.join(tempfile.gettempdir(), "can_selftest.csv")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("epoch;id;data\n")
        for r in sorted(rows, key=lambda x: float(x[0])):
            fh.write(";".join(r) + "\n")

    print(f"[selftest] synthesized bus -> {path}")
    ok = True

    def check(label, cands, want_id, want_byte, want_bit, want_kind):
        if not cands:
            print(f"  FAIL {label}: no candidates"); return False
        top = cands[0]
        hit = (top.can_id == want_id and top.byte == want_byte and top.kind == want_kind
               and (want_bit is None or top.bit == want_bit))
        print(f"  {'PASS' if hit else 'FAIL'} {label}: top = ID 0x{top.can_id:X} "
              f"byte {top.byte} bit {top.bit} [{top.kind}] score {top.score:.2f}")
        return hit

    print("\n[selftest] BUTTON (6 presses, --events 6, mode pulse):")
    c = analyze(path, 6, None, None, 0.5, "pulse", 0.2, None, None, 5)
    ok &= check("button", c, 0x200, 3, 1, "pulse-bit")

    print("\n[selftest] LOCK (5 toggles, --events 5, mode toggle):")
    c = analyze(path, 5, None, None, 0.5, "toggle", 0.2, None, None, 5)
    ok &= check("lock", c, 0x201, 0, 0, "toggle-bit")

    print("\n[selftest] WHEEL BUTTON (6 hits, --events 6, mode value):")
    c = analyze(path, 6, None, None, 0.5, "value", 0.2, None, None, 5)
    ok &= check("wheel-enum", c, 0x5C1, 4, None, "value-byte")
    if c:
        print(f"         decoded button codes: {[hex(v) for v in c[0].active_values]}")

    print("\n[selftest] CADENCE plausibility (human vs machine onset spacing):")
    human = np.array([0.0, 1.2, 2.4, 3.6])    # ~1.2s apart: deliberate presses
    medium = np.array([0.0, 0.4, 0.8, 1.2])   # 0.4s beat: too fast to be natural
    machine = np.array([0.0, 0.1, 0.2, 0.3])  # 0.1s beat: rolling/status traffic
    fh = cadence_factor(human, 0.30, 1.0)[0]
    fm = cadence_factor(medium, 0.30, 1.0)[0]
    fx = cadence_factor(machine, 0.30, 1.0)[0]
    cad_ok = (fh == 1.0 and fm < 0.8 and fx <= _CADENCE_FLOOR_PENALTY + 1e-9)
    print(f"  {'PASS' if cad_ok else 'FAIL'} cadence: human x{fh:.2f}, "
          f"0.4s-beat x{fm:.2f}, 0.1s-beat x{fx:.2f}")
    ok &= cad_ok

    print("\n[selftest]", "ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_id_list(s: str | None) -> set[int] | None:
    if not s:
        return None
    return {int(x, 0) for x in s.split(",") if x.strip()}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Find the CAN ID/byte/bit behind a button, lock or switch (read-only).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--trace", help="CAN log to analyze (webCAN CSV / candump / generic)")
    ap.add_argument("--events", type=int, help="how many times you triggered the action")
    ap.add_argument("--event-times", help="file of epoch timestamps, or a CSS sidecar CSV")
    ap.add_argument("--baseline", help="a quiet log where you did nothing (gating)")
    ap.add_argument("--tol", type=float, default=0.5, help="timing tolerance (s) for --event-times")
    ap.add_argument("--mode", choices=["auto", "pulse", "toggle", "value"], default="auto",
                    help="encoding hypothesis (default auto: try all)")
    ap.add_argument("--idle-frac", type=float, default=0.2,
                    help="max fraction of frames a control may spend active (default 0.2)")
    ap.add_argument("--only-ids", help="comma list of IDs to consider, e.g. 0x200,0x5C1")
    ap.add_argument("--exclude-ids", help="comma list of IDs to skip")
    ap.add_argument("--top", type=int, default=10, help="how many candidates to print")
    ap.add_argument("--min-gap", type=float, default=0.30,
                    help="fastest plausible gap (s) between repeats of a human "
                         "action; onsets faster than this are treated as machine "
                         "traffic (rolling counters, status beats) and crushed "
                         "(default 0.30)")
    ap.add_argument("--settle-gap", type=float, default=1.0,
                    help="onset gap (s) at/above which cadence is fully "
                         "human-plausible; between this and --min-gap the score "
                         "ramps down (default 1.0)")
    ap.add_argument("--drop-implausible", action="store_true",
                    help="remove (not just downrank) candidates whose repeat rate "
                         "is faster than --min-gap")
    ap.add_argument("--selftest", action="store_true",
                    help="synthesize a bus with planted signals and verify recovery")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()

    if not args.trace:
        ap.error("--trace is required (or use --selftest)")

    cands = analyze(
        trace=args.trace,
        n_events=args.events,
        event_times_path=args.event_times,
        baseline=args.baseline,
        tol=args.tol,
        mode=args.mode,
        idle_frac_max=args.idle_frac,
        only_ids=_parse_id_list(args.only_ids),
        exclude_ids=_parse_id_list(args.exclude_ids),
        top=args.top,
        min_gap=args.min_gap,
        settle_gap=args.settle_gap,
        drop_implausible=args.drop_implausible,
    )

    if not cands:
        print("No candidate controls found. Tips:")
        print("  * make sure you actually triggered the action during the recording")
        print("  * try --mode value (steering-wheel/console buttons are often an enum byte)")
        print("  * raise --idle-frac if the control is held for a long time")
        print("  * capture a --baseline quiet log and pass it to gate out unrelated traffic")
        return 2

    spec = (f"--events {args.events}" if args.events is not None
            else f"--event-times {args.event_times}" if args.event_times
            else "no spec (ranking by crispness)")
    print(f"\nTop {len(cands)} candidate(s)  [{spec}]:\n")
    for i, c in enumerate(cands, 1):
        print(_fmt_candidate(i, c))
        print()
    print("Next: confirm the #1 hit by re-recording and watching that byte/bit, then\n"
          "add it to your DBC. For buttons encoded as an enum byte, each listed code\n"
          "is a different button on the same message.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
